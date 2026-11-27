"""Local maintenance authority and watchdog, with no chassis transport of its own."""

from dataclasses import dataclass
from enum import Enum
import math
import secrets
from threading import Event, Lock, Thread
import time
from typing import Callable, Generic, Protocol, TypeVar


SITE_FRONT_SECONDS = 5


class Owner(Enum):
    RELEASED = "released"
    NEUTRAL = "neutral"
    OWNED = "owned"
    UNKNOWN = "unknown"


class Direction(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"


def command_direction(linear, angular):
    # 两轴混合和零速保活均不对应允许的运动方向。
    if linear and not angular:
        return Direction.FORWARD if linear > 0 else Direction.BACKWARD
    if angular and not linear:
        return Direction.LEFT if angular > 0 else Direction.RIGHT
    return None


class Feedback(Enum):
    STOPPED = "stopped"
    MOVING = "moving"
    SETTLING = "settling"


T = TypeVar("T")


@dataclass(frozen=True)
class Sample(Generic[T]):
    value: T
    observed_at: float


@dataclass(frozen=True)
class LocalGrant:
    serial: int
    issued_at: float
    expires_at: float | None
    directions: frozenset[Direction]
    site_front_confirmed: bool = False
    repeated_actions: bool = False
    continuous_hold: bool = False


@dataclass(frozen=True)
class FrontLidar:
    near_return_m: float | None
    point_count: int
    frame_id: str


@dataclass(frozen=True)
class OperatorHold:
    # 每次物理按压对应一个周期，松开再按下不得续期旧动作。
    pressed: bool
    epoch: str
    direction: Direction | None = None


@dataclass(frozen=True)
class Inputs:
    owner: Sample[Owner] | None = None
    guard: Sample[frozenset[Direction]] | None = None
    feedback: Sample[Feedback] | None = None
    grant: LocalGrant | None = None
    # 现场量距供锁存下的静止验收，方向许可始终只取 guard。
    static_front_clearance: Sample[float] | None = None
    # 前雷达诊断保留原始帧参考系和时间，仅供观察。
    front_lidar: Sample[FrontLidar] | None = None
    # 现场前向会话结束后仍可读取的停稳反馈，不作为普通许可的放行依据。
    site_stop_feedback: Sample[Feedback] | None = None
    # 独立采集的按住信号，时间戳来自采样端。控制请求不能写入此字段。
    operator_hold: Sample[OperatorHold] | None = None


@dataclass(frozen=True)
class Limits:
    linear_mps: float
    angular_radps: float
    lease_s: float
    deadman_s: float
    max_sample_age_s: float
    tick_s: float
    # 未设前向限速的调用方继续使用原有双向线速度。单独设置时，linear_mps 仍限制后退。
    forward_mps: float | None = None

    def __post_init__(self):
        values = (self.linear_mps, self.angular_radps, self.lease_s, self.deadman_s,
                  self.max_sample_age_s, self.tick_s)
        if not all(_finite(value) and value > 0 for value in values):
            raise ValueError("Positive finite limits are required")
        if self.forward_mps is not None and (not _finite(self.forward_mps) or self.forward_mps <= 0):
            raise ValueError("A positive finite forward limit is required")
        if (not .001 <= self.lease_s <= 60 or not self.tick_s < self.deadman_s <= self.lease_s
                or self.tick_s >= self.max_sample_age_s):
            raise ValueError("Inconsistent lease or watchdog timing")


@dataclass(frozen=True)
class VelocityRequest:
    session_id: str
    sequence: int
    grant_serial: int
    generated_at: float
    expires_at: float
    linear_mps: float
    angular_radps: float
    # 绑定独立采样的按压周期，供底盘适配器在最终发令前复核。
    hold_epoch: str | None = None
    reusable_hold: bool = False
    require_standstill: bool = False


class HoldReleasedBeforeOutput(Exception):
    """接收器在调用底盘 SDK 前观察到实体键松开。"""


class Output(Protocol):
    """Bounded local handoff to the collision monitor and chassis bridge.

    Acknowledging either method means the request was accepted. A separate
    feedback observation must establish actual standstill.
    """

    receiver_id: str

    def velocity(self, request: VelocityRequest) -> bool: ...

    def stop(self, session_id: str, reason: str) -> bool: ...


@dataclass
class _Action:
    action_id: int
    direction: Direction
    expires_at: float | None
    ended: bool = False
    released: bool = False


@dataclass
class _Lease:
    session_id: str
    grant: LocalGrant
    expires_at: float | None
    sequence: int = 0
    command_deadline: float | None = None
    direction: Direction | None = None
    action: _Action | None = None
    last_released_action: int | None = None
    hold_epoch: str | None = None
    neutral_hold_epoch: str | None = None
    hold_direction: Direction | None = None
    reusable_hold: bool = False

    @property
    def uses_actions(self):
        return self.grant.repeated_actions or self.reusable_hold


def _valid_sequence(value):
    return type(value) is int and 1 <= value < 2**53


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _reject(status, reason):
    return status, {"ok": False, "reason": reason}


class MaintenanceArbiter:
    """A single manual lease backed by fresh, independently acquired local evidence.

    Input samples and requests use the same machine's monotonic clock. The
    provider and output callbacks must be bounded and must not call this object.
    The output still requires its own downstream watchdog. No real output or
    field evidence provider is included here.
    """

    def __init__(self, limits: Limits, observe: Callable[[], Inputs], output: Output,
                 *, clock: Callable[[], float] = time.monotonic, idle_deadman=False,
                 operator_supervised=False, require_operator_hold=False, require_hold_direction=False,
                 allow_continuous_hold=False):
        if not isinstance(output.receiver_id, str) or not output.receiver_id:
            raise ValueError("A specific output receiver ID is required")
        if allow_continuous_hold and not (operator_supervised and idle_deadman and require_hold_direction):
            raise ValueError("持续按住须有受管人工驾驶、断流看门狗和独立方向证据")
        self._limits, self._observe, self._output, self._clock = limits, observe, output, clock
        self._idle_deadman = idle_deadman
        self._operator_supervised = operator_supervised
        self._require_operator_hold = require_operator_hold or require_hold_direction
        self._require_hold_direction = require_hold_direction
        self._allow_continuous_hold = allow_continuous_hold is True
        self._started_at = clock()
        self._lock, self._closed = Lock(), Event()
        self._lease = None
        self._last_grant = 0
        self._stop_session_id = None
        self._stop_requested_at = None
        self._site_stop_pending = False
        self._stop_accepted = False
        self._stop_reason = None
        self._retry_stop_after = 0.
        self._yielded_to_owner = False
        self._worker = Thread(target=self._watchdog, name="a2-maintenance-watchdog", daemon=True)
        self._worker.start()

    def _watchdog(self):
        while not self._closed.wait(self._limits.tick_s):
            self.tick()

    def close(self):
        self._closed.set()
        with self._lock:
            self._advance()
        self._worker.join()
        # 正常退出与松键共用停车重试。保留有限的收尾时间，systemd 独立守护继续管理强杀路径。
        deadline = time.monotonic() + self._limits.deadman_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._stop_requested_at is None or self._stop_accepted or self._yielded_to_owner:
                    return
                self._advance()
            time.sleep(min(.02, self._limits.tick_s))

    def _sample(self, sample, now):
        if (isinstance(sample, Sample) and _finite(sample.observed_at)
                and 0 <= now - sample.observed_at <= self._limits.max_sample_age_s):
            return sample.value
        return None

    def _operator_hold(self, inputs, now):
        sample = inputs.operator_hold
        if (not isinstance(sample, Sample) or not _finite(sample.observed_at)
                or not 0 <= now - sample.observed_at < self._limits.max_sample_age_s):
            return None
        hold = sample.value
        if (not isinstance(hold, OperatorHold) or type(hold.pressed) is not bool
                or not isinstance(hold.epoch, str) or not 0 < len(hold.epoch) <= 64):
            return None
        return hold

    def _check_operator_hold(self, inputs, now, epoch, direction):
        hold = self._operator_hold(inputs, now)
        if hold is None:
            return "operator_hold_stale"
        if not hold.pressed:
            return "operator_hold_released"
        if epoch is not None and hold.epoch != epoch:
            return "operator_hold_changed"
        # 键盘证据只为同一物理方向续期。按住前进不能放行网页错误续发的后退。
        if self._require_hold_direction and hold.direction is not direction:
            return "operator_hold_direction_mismatch"
        return None

    def _continuous_hold_sample(self, inputs, now):
        hold = self._operator_hold(inputs, now)
        if hold is None or (hold.pressed and not isinstance(hold.direction, Direction)) or (
                not hold.pressed and hold.direction is not None):
            return None
        return hold

    def _track_continuous_hold(self, inputs, now, lease):
        hold = self._continuous_hold_sample(inputs, now)
        if hold is None:
            return "operator_hold_stale"
        if lease.hold_epoch is not None:
            return self._check_operator_hold(inputs, now, lease.hold_epoch, lease.hold_direction)
        if not hold.pressed:
            # 回中周期变化也可能意味着未观测到的一次按下和松开，旧会话不得起步。
            return None if hold.epoch == lease.neutral_hold_epoch else "operator_hold_changed"
        if hold.epoch == lease.neutral_hold_epoch:
            return "operator_hold_changed"
        if hold.direction not in lease.grant.directions:
            return "operator_hold_direction_mismatch"
        # 从首次看见实体键按下起就绑定周期和方向，包括首个速度包尚未到达时。
        lease.hold_epoch, lease.hold_direction = hold.epoch, hold.direction
        return None

    def _track_reusable_hold(self, inputs, now, lease):
        hold = self._continuous_hold_sample(inputs, now)
        if hold is None:
            return "operator_hold_stale"
        action = lease.action
        if action is None:
            if not hold.pressed:
                lease.neutral_hold_epoch = hold.epoch
            return None
        if action.ended:
            # 浏览器释放、独立键盘回中和停车确认缺一不可。停车期间按下的键不排队。
            if (action.released and self._stop_requested_at is None and not hold.pressed
                    and hold.epoch != lease.hold_epoch):
                lease.neutral_hold_epoch = hold.epoch
                lease.hold_epoch = None
                lease.action = None
            return None
        if not hold.pressed:
            if hold.epoch == lease.hold_epoch:
                return "operator_hold_changed"
            # 即使浏览器继续发旧速度包，独立键盘回中也会结束本段并立即停车。
            self._end_action(lease, "action_hold_released", inputs, now)
            return None
        return self._check_operator_hold(inputs, now, lease.hold_epoch, action.direction)

    def _granted(self, grant, now):
        if (not isinstance(grant, LocalGrant) or type(grant.serial) is not int or grant.serial <= 0
                or not _finite(grant.issued_at) or not self._started_at < grant.issued_at <= now
                or not isinstance(grant.directions, frozenset) or not grant.directions
                or not all(isinstance(direction, Direction) for direction in grant.directions)
                or type(grant.site_front_confirmed) is not bool or type(grant.repeated_actions) is not bool
                or type(grant.continuous_hold) is not bool):
            return False
        if grant.continuous_hold:
            return (self._allow_continuous_hold and not grant.repeated_actions
                    and not grant.site_front_confirmed and grant.expires_at is None)
        if grant.repeated_actions:
            return (self._operator_supervised and self._idle_deadman
                    and not grant.site_front_confirmed and grant.expires_at is None)
        return (_finite(grant.expires_at) and now < grant.expires_at
                and (not grant.site_front_confirmed
                     or (grant.directions == {Direction.FORWARD}
                         and grant.expires_at == grant.issued_at + SITE_FRONT_SECONDS)))

    def _directions(self, inputs, now):
        if self._operator_supervised:
            # 现场操作员负责场地。下面的授权交集仍对每条非零速度指令生效。
            return frozenset(Direction)
        directions = self._sample(inputs.guard, now)
        # 本机现场确认是单次、仅前向的来源。独立方向监护一旦存在，其阻挡和过期仍有否决权。
        if (inputs.guard is None and isinstance(inputs.grant, LocalGrant)
                and inputs.grant.site_front_confirmed is True):
            directions = frozenset({Direction.FORWARD})
        if not isinstance(directions, frozenset) or not all(isinstance(direction, Direction) for direction in directions):
            return None
        return directions

    def _deny_reason(self, inputs, now):
        if self._closed.is_set():
            return "arbiter_closed"
        owner = self._sample(inputs.owner, now)
        if owner not in (Owner.RELEASED, Owner.NEUTRAL):
            return "control_owned" if owner is Owner.OWNED else "control_unknown"
        reusable = self._lease is not None and self._lease.reusable_hold
        feedback = self._sample(inputs.feedback, now)
        feedback_valid = feedback in (Feedback.STOPPED, Feedback.MOVING) or (
            reusable and feedback is Feedback.SETTLING)
        if reusable:
            reason = self._track_reusable_hold(inputs, now, self._lease)
            if reason:
                return reason
        elif self._lease and self._lease.grant.continuous_hold:
            reason = self._track_continuous_hold(inputs, now, self._lease)
            if reason:
                return reason
        elif self._lease and self._lease.hold_epoch is not None:
            # 后台看门狗与每个控制请求都走这里。浏览器仍在发包也不能掩盖松手或断流。
            direction = self._lease.action.direction if self._lease.action else self._lease.direction
            reason = self._check_operator_hold(inputs, now, self._lease.hold_epoch, direction)
            if reason:
                return reason
        if self._lease and self._lease.uses_actions:
            # 减速过程允许有效的运动反馈，但本地授权、链路和实时机身状态仍须持续有效。
            if not self._granted(inputs.grant, now) or inputs.grant != self._lease.grant:
                return "authorization_changed"
            if now >= self._lease.command_deadline:
                return "deadman_expired"
            if self._stop_requested_at is not None:
                return None if feedback_valid else "feedback_stale"
        if self._stop_requested_at is not None:
            return "stop_unconfirmed"
        if not feedback_valid:
            # 现场停稳反馈新鲜、且没有许可时，页面显示等待授权，不把结束后的状态误报为反馈过期。
            if (self._lease is None and inputs.grant is None
                    and self._sample(inputs.site_stop_feedback, now) is Feedback.STOPPED):
                return "local_authorization_required"
            return "feedback_stale"
        if not self._lease or self._lease.direction is None:
            # 已启用但未启动动作时，真实反馈暂未确认停稳，只阻止新动作。
            # 已受理起步但未发速度时，仍要求连续停稳，否则撤权。
            idle_reusable = reusable and self._lease.action is None
            if feedback is not Feedback.STOPPED and not idle_reusable:
                return "standstill_required"
        directions = self._directions(inputs, now)
        if directions is None:
            return "guard_stale"
        if not self._granted(inputs.grant, now):
            return "local_authorization_required"
        allowed = directions & inputs.grant.directions
        if not allowed or (self._lease and self._lease.direction and self._lease.direction not in allowed):
            return "direction_not_authorized" if self._operator_supervised else "obstacle"
        if self._lease:
            if inputs.grant != self._lease.grant:
                return "authorization_changed"
            if self._lease.expires_at is not None and now >= self._lease.expires_at:
                return "lease_expired"
            if self._lease.command_deadline is not None and now >= self._lease.command_deadline:
                return "deadman_expired"
        elif inputs.grant.serial <= self._last_grant:
            return "local_authorization_required"
        elif inputs.grant.continuous_hold:
            hold = self._continuous_hold_sample(inputs, now)
            if hold is None:
                return "operator_hold_stale"
            if hold.pressed:
                return "operator_hold_neutral_required"
        return None

    def _revoke(self, reason, inputs, now):
        lease, self._lease = self._lease, None
        if self._stop_requested_at is not None and self._stop_session_id == lease.session_id:
            # 已停发的动作沿用原停车事务，不重复发送停车，也保留原厂接管判断。
            self._request_pending_stop(inputs, now)
            return
        self._request_stop(lease, reason, inputs, now)

    def _request_stop(self, lease, reason, inputs, now):
        self._stop_session_id = lease.session_id
        self._stop_requested_at = now
        self._site_stop_pending = lease.grant.site_front_confirmed
        self._stop_accepted = False
        self._stop_reason = reason
        self._retry_stop_after = now
        self._yielded_to_owner = False
        self._request_pending_stop(inputs, now)

    def _end_action(self, lease, reason, inputs, now):
        if not lease.action.ended:
            lease.action.ended = True
            lease.direction = None
            if not lease.reusable_hold:
                lease.hold_epoch = None
            self._request_stop(lease, reason, inputs, now)

    def _request_pending_stop(self, inputs, now):
        # 有明确的原厂接管证据即永久让出旧会话；状态源断流时仍须撤回自己的运动。
        if self._sample(inputs.owner, now) is Owner.OWNED:
            self._yielded_to_owner = True
        if self._yielded_to_owner or self._stop_accepted or now < self._retry_stop_after:
            return
        self._retry_stop_after = now + .10
        try:
            self._stop_accepted = self._output.stop(self._stop_session_id, self._stop_reason) is True
        except Exception:
            # 出错后保留撤权和待停车状态；看门狗重试，不依赖浏览器再次发包。
            self._stop_accepted = False

    def _advance(self):
        try:
            inputs = self._observe()
        except Exception:
            inputs = None
        if not isinstance(inputs, Inputs):
            inputs = Inputs()
        now = self._clock()
        if self._stop_requested_at is not None:
            feedback = inputs.feedback
            if self._site_stop_pending and self._sample(feedback, now) is None:
                # 现场许可可能已经到期。继续读取该会话的停稳证据，仍要求停车请求之后取得的样本。
                feedback = inputs.site_stop_feedback
            if ((not self._lease or not self._lease.reusable_hold or self._stop_accepted)
                    and self._sample(feedback, now) is Feedback.STOPPED
                    and feedback.observed_at > self._stop_requested_at):
                self._stop_requested_at = None
                self._site_stop_pending = False
                self._stop_accepted = True
            else:
                self._request_pending_stop(inputs, now)
        reason = self._deny_reason(inputs, now)
        if reason and self._lease:
            self._revoke(reason, inputs, now)
        if self._lease and self._lease.action:
            action = self._lease.action
            if not action.ended and action.expires_at is not None and now >= action.expires_at:
                self._end_action(self._lease, "action_expired", inputs, now)
            if (not self._lease.reusable_hold and action.ended and action.released
                    and self._stop_requested_at is None):
                self._lease.action = None
        return inputs, now, reason

    def tick(self):
        with self._lock:
            self._advance()

    def state(self):
        with self._lock:
            inputs, now, reason = self._advance()
            limits = {"linear_mps": self._limits.linear_mps, "angular_radps": self._limits.angular_radps}
            if self._limits.forward_mps is not None:
                limits["forward_mps"] = self._limits.forward_mps
            state = {"available": reason is None, "armed": self._lease is not None,
                     "reason": reason or ("armed" if self._lease else "ready"), "limits": limits}
            if reason is None and inputs.grant.continuous_hold:
                state["reusable_hold"] = True
            if self._lease and self._lease.uses_actions:
                action = self._lease.action
                if action is None:
                    state["action_state"] = (
                        "standstill_pending" if self._lease.reusable_hold
                        and self._sample(inputs.feedback, now) is not Feedback.STOPPED else "ready")
                else:
                    state["action_id"] = action.action_id
                    if self._stop_requested_at is not None:
                        state["action_state"] = "stopping"
                    else:
                        state["action_state"] = "release_required" if action.ended else "active"
            return state

    def acquire(self, payload):
        if (not isinstance(payload, dict) or (
                payload and (set(payload) != {"reusable_hold"} or payload["reusable_hold"] is not True))):
            return _reject(400, "invalid_request")
        with self._lock:
            inputs, now, reason = self._advance()
            if reason:
                return _reject(409, reason)
            if self._lease:
                return _reject(409, "session_busy")
            reusable = payload.get("reusable_hold") is True
            if reusable and not inputs.grant.continuous_hold:
                return _reject(409, "reusable_hold_unavailable")
            if inputs.grant.continuous_hold:
                # 放行点已核实独立回中。先启用断流看门狗，随后等待新的实体按压周期。
                self._lease = _Lease(secrets.token_urlsafe(24), inputs.grant, None,
                                     command_deadline=now + self._limits.deadman_s,
                                     neutral_hold_epoch=inputs.operator_hold.value.epoch,
                                     reusable_hold=reusable)
                self._last_grant = inputs.grant.serial
                receipt = {"ok": True, "session_id": self._lease.session_id, "continuous_hold": True}
                if reusable:
                    receipt["reusable_hold"] = True
                return 200, receipt
            if inputs.grant.repeated_actions:
                self._lease = _Lease(secrets.token_urlsafe(24), inputs.grant, None,
                                     command_deadline=now + self._limits.deadman_s)
                self._last_grant = inputs.grant.serial
                return 200, {"ok": True, "session_id": self._lease.session_id, "repeated_actions": True,
                             "action_limit_ms": math.floor(self._limits.lease_s * 1000)}
            # 单次会话仍受本机授权有效期与仲裁器的会话上限约束。
            expires = min(now + self._limits.lease_s, inputs.grant.expires_at)
            expires_in_ms = math.floor((expires - now) * 1000)
            if expires_in_ms < 1:
                return _reject(409, "local_authorization_required")
            # 网页遥控从授权起即启用死手，避免空闲掉线后，积压的首个方向包迟到而起步。
            initial_deadline = min(now + self._limits.deadman_s, expires) if self._idle_deadman else None
            self._lease = _Lease(secrets.token_urlsafe(24), inputs.grant, expires, command_deadline=initial_deadline)
            self._last_grant = inputs.grant.serial
            return 200, {"ok": True, "session_id": self._lease.session_id, "expires_in_ms": expires_in_ms}

    def action_start(self, payload):
        if (not isinstance(payload, dict) or set(payload) != {"session_id", "sequence", "direction"}
                or not isinstance(payload["session_id"], str) or not _valid_sequence(payload["sequence"])
                or not isinstance(payload["direction"], str)):
            return _reject(400, "invalid_action")
        try:
            direction = Direction(payload["direction"])
        except ValueError:
            return _reject(400, "invalid_action")
        with self._lock:
            inputs, now, _ = self._advance()
            lease = self._lease
            if not lease or lease.session_id != payload["session_id"] or not lease.uses_actions:
                return _reject(409, "invalid_session")
            if lease.action is not None:
                return _reject(409, "action_stopping" if self._stop_requested_at is not None else "action_release_required")
            if payload["sequence"] <= lease.sequence:
                return _reject(409, "stale_sequence")
            if self._sample(inputs.feedback, now) is not Feedback.STOPPED:
                return _reject(409, "standstill_required")
            if direction not in lease.grant.directions:
                return _reject(409, "direction_not_authorized")
            if self._require_operator_hold:
                reason = self._check_operator_hold(inputs, now, None, direction)
                if reason:
                    if lease.reusable_hold and reason == "operator_hold_direction_mismatch":
                        self._revoke(reason, inputs, now)
                    return _reject(409, reason)
                lease.hold_epoch = inputs.operator_hold.value.epoch
                if lease.reusable_hold and lease.hold_epoch == lease.neutral_hold_epoch:
                    self._revoke("operator_hold_changed", inputs, now)
                    return _reject(409, "operator_hold_changed")
            lease.sequence = payload["sequence"]
            lease.command_deadline = now + self._limits.deadman_s
            if lease.reusable_hold:
                lease.action = _Action(lease.sequence, direction, None)
                return 200, {"ok": True, "action_id": lease.action.action_id}
            lease.action = _Action(lease.sequence, direction, now + self._limits.lease_s)
            return 200, {"ok": True, "action_id": lease.action.action_id,
                         "expires_in_ms": math.floor(self._limits.lease_s * 1000)}

    def action_release(self, payload):
        if (not isinstance(payload, dict) or set(payload) != {"session_id", "sequence", "action_id"}
                or not isinstance(payload["session_id"], str) or not _valid_sequence(payload["sequence"])
                or not _valid_sequence(payload["action_id"]) or payload["action_id"] >= payload["sequence"]):
            return _reject(400, "invalid_action")
        with self._lock:
            inputs, now, _ = self._advance()
            lease = self._lease
            if not lease or lease.session_id != payload["session_id"] or not lease.uses_actions:
                return _reject(409, "invalid_session")
            action_id = payload["action_id"]
            if action_id == lease.last_released_action:
                return 200, {"ok": True}
            if lease.action is not None:
                if lease.action.action_id != action_id:
                    return _reject(409, "invalid_action")
                lease.action.released = True
                self._end_action(lease, "action_release", inputs, now)
            elif action_id <= lease.sequence:
                # 已处理的高序号也能挡住迟到的按键；迟到的松键回执不延长就绪期限。
                return 200, {"ok": True}
            # 若松键先到，推进序号，把可能仍在途的旧按键请求封在本次动作之外。
            lease.last_released_action = action_id
            lease.sequence = max(lease.sequence, payload["sequence"])
            lease.command_deadline = now + self._limits.deadman_s
            return 200, {"ok": True}

    def velocity(self, payload):
        fields = {"session_id", "sequence", "linear_mps", "angular_radps"}
        if not isinstance(payload, dict) or set(payload) not in (fields, fields | {"action_id"}):
            return _reject(400, "invalid_velocity")
        seq, linear, angular = (payload[key] for key in ("sequence", "linear_mps", "angular_radps"))
        forward_limit = self._limits.forward_mps or self._limits.linear_mps
        if (not isinstance(payload["session_id"], str) or not _valid_sequence(seq)
                or not _finite(linear) or not _finite(angular)
                or not -self._limits.linear_mps <= linear <= forward_limit
                or abs(angular) > self._limits.angular_radps
                or (linear and angular) or (not self._idle_deadman and not (linear or angular))
                or ("action_id" in payload and (not _valid_sequence(payload["action_id"])
                                                or payload["action_id"] >= seq or not (linear or angular)))):
            return _reject(400, "invalid_velocity")
        with self._lock:
            inputs, now, _ = self._advance()
            lease = self._lease
            if not lease or payload["session_id"] != lease.session_id:
                return _reject(409, "invalid_session")
            if seq <= lease.sequence:
                return _reject(409, "stale_sequence")
            deadline = now + self._limits.deadman_s
            if not lease.uses_actions:
                if "action_id" in payload:
                    return _reject(400, "invalid_velocity")
                if not lease.grant.continuous_hold:
                    deadline = min(deadline, lease.expires_at, inputs.grant.expires_at)
            if not (linear or angular):
                # 零速只维持就绪或等待停稳。运动中的零帧不能覆盖实速包序号，
                # 漏掉松键时仍按上一个有效运动包的期限停车。
                if lease.direction is not None:
                    return _reject(409, "idle_heartbeat_after_motion")
                if lease.reusable_hold and lease.action is not None and not lease.action.released:
                    # 独立键盘已经停车时，漏掉松键的页面不能靠零速心跳继续保留本段。
                    return _reject(409, "action_release_required")
                lease.sequence, lease.command_deadline = seq, deadline
                return 200, {"ok": True}
            direction = command_direction(linear, angular)
            action = lease.action
            if lease.uses_actions:
                if action is None or payload.get("action_id") != action.action_id:
                    return _reject(409, "invalid_action")
                if action.ended:
                    return _reject(409, "action_expired")
                if action.direction is not direction:
                    if lease.reusable_hold:
                        self._revoke("operator_hold_direction_mismatch", inputs, now)
                    return _reject(409, "direction_not_authorized")
                # 动作时钟和有效速度包间隔分别校验。速度包只能刷新间隔，不能续期动作。
                action_deadline = min(deadline, action.expires_at) if action.expires_at is not None else deadline
            else:
                action_deadline = deadline
            if direction not in self._directions(inputs, now) or direction not in inputs.grant.directions:
                reason = "direction_not_authorized" if self._operator_supervised else "obstacle"
                self._revoke(reason, inputs, now)
                return _reject(409, reason)
            if self._require_operator_hold:
                reason = self._check_operator_hold(inputs, now, lease.hold_epoch, direction)
                if reason:
                    if lease.grant.continuous_hold and reason == "operator_hold_direction_mismatch":
                        self._revoke(reason, inputs, now)
                    return _reject(409, reason)
                hold = inputs.operator_hold
                lease.hold_epoch = hold.value.epoch
                # 独立采样也约束该速度包的有效时间，网络保活不能把旧样本续到采样期限之外。
                action_deadline = min(action_deadline, hold.observed_at + self._limits.max_sample_age_s)
            request = VelocityRequest(lease.session_id, seq, lease.grant.serial, now, action_deadline,
                                      linear, angular, lease.hold_epoch, lease.reusable_hold,
                                      require_standstill=lease.reusable_hold and lease.direction is None)
            try:
                accepted = self._output.velocity(request) is True
            except HoldReleasedBeforeOutput:
                # 松键可能恰好发生在仲裁读取与 SDK 复核之间。重新检查独立样本及全部会话门禁。
                inputs, now, _ = self._advance()
                if self._lease is not lease:
                    return _reject(409, "invalid_session")
                if lease.reusable_hold and action.ended:
                    return _reject(409, "action_expired")
                accepted = False
            except Exception:
                accepted = False
            if not accepted:
                self._revoke("output_rejected", inputs, self._clock())
                return _reject(409, "output_rejected")
            lease.sequence, lease.command_deadline, lease.direction = seq, deadline, direction
            return 200, {"ok": True}

    def stop(self, payload):
        if not isinstance(payload, dict) or set(payload) != {"session_id"} or not isinstance(payload["session_id"], str):
            return _reject(400, "invalid_session")
        with self._lock:
            inputs, now, _ = self._advance()
            session = payload["session_id"]
            if self._lease:
                if session != self._lease.session_id:
                    return _reject(409, "invalid_session")
                self._revoke("operator_stop", inputs, now)
            if session != self._stop_session_id:
                return _reject(409, "invalid_session")
            if not self._stop_accepted:
                return _reject(409, "stop_unconfirmed")
            return 200, {"ok": True}
