"""机身坐标系的维护遥控接收端，运行于 PC2；模拟底盘与实机证据路径彼此独立。"""

from dataclasses import replace
import json
import math
import os
from pathlib import Path
import pwd
import stat
import time

from maintenance_arbiter import (Direction, Feedback, FrontLidar, Inputs, Limits, LocalGrant,
                                 MaintenanceArbiter, OperatorHold, Owner, Sample, _finite, command_direction)
from manual_authority import _distinct_fields, neutral_epoch, read_owner
from manual_grant import grant_status


# 仅允许单轴。转向按 10 度/秒配置，SDK 和遥控协议统一使用弧度/秒。
LIMITS = Limits(linear_mps=.20, angular_radps=math.radians(10), lease_s=30., deadman_s=.4,
                max_sample_age_s=.20, tick_s=.02, forward_mps=.20)
DIRECTIONS = frozenset(Direction)
SAFETY_DIRECTORY = Path('/run/a2-manual-safety')
STATIC_CLEARANCE_SOURCE = 'field_tape_static'
FRONT_LIDAR_SOURCE = 'front_lidar_diagnostic'
FRONT_LIDAR_FRAME = 'hesai_lidar'
READY = Path('/run/a2-remote-stop/ready')
KEYBOARD_HOLD = Path('/run/a2-keyboard-hold/sample.json')


def in_autonomy_cgroup():
    """只认系统管理器的受管控制组，不接受用户级同名单元或任意层级的同名目录。"""
    for line in Path('/proc/self/cgroup').read_text().splitlines():
        hierarchy, controllers, group = line.split(':', 2)
        if (hierarchy == '0' and not controllers) or 'name=systemd' in controllers.split(','):
            return group == '/system.slice/a2-autonomy.service'
    return False


def trusted_file(path):
    """读取 root 原子发布的短快照。目录和文件都拒绝符号链接、非 root 所有或组可写。"""
    directory = os.open(path.parent, os.O_DIRECTORY | os.O_NOFOLLOW | os.O_RDONLY)
    try:
        info = os.fstat(directory)
        if info.st_uid != 0 or info.st_mode & 0o022:
            raise PermissionError('安全证据目录权限不可信')
        fd = os.open(path.name, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW, dir_fd=directory)
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
                raise PermissionError('安全证据文件权限不可信')
            raw = os.read(fd, 8193)
            if len(raw) > 8192:
                raise ValueError('安全证据文件过大')
            return raw
        finally:
            os.close(fd)
    finally:
        os.close(directory)


def guard_ready():
    # 持久锁存目录为 root:0700。systemd 在启动时检查 !latched；守护先撤下 ready
    # 再写入锁存。运行中只信守护的只读状态，不给发令账号增加访问或复位锁存的权限。
    try:
        if os.geteuid() == 0 or os.geteuid() != pwd.getpwnam('a2-autonomy').pw_uid or not in_autonomy_cgroup():
            return False
        # ready 由守护以 0077 umask 创建；非特权账号可通过目录执行位读取元数据。
        parent, marker = READY.parent.lstat(), READY.lstat()
        return (stat.S_ISDIR(parent.st_mode) and stat.S_ISREG(marker.st_mode)
                and parent.st_uid == marker.st_uid == 0 and not (parent.st_mode | marker.st_mode) & 0o022)
    except (OSError, ValueError, KeyError):
        return False


def _front_lidar_sample(report):
    """只保留前雷达实时诊断，沿用安全输入的 PC2 单调时间上限。"""
    if (not isinstance(report, dict) or report.get('source') != FRONT_LIDAR_SOURCE
            or report.get('frame_id') != FRONT_LIDAR_FRAME or 'near_return_m' not in report):
        return None
    observed_at = report.get('observed_at')
    if (not _finite(observed_at) or observed_at < 0
            or not 0 <= time.monotonic() - observed_at <= LIMITS.max_sample_age_s):
        return None
    point_count = report.get('point_count')
    distance = report['near_return_m']
    if (type(point_count) is not int or point_count <= 0
            or (distance is not None and (not _finite(distance) or distance < 0))):
        return None
    return Sample(FrontLidar(None if distance is None else float(distance), point_count, FRONT_LIDAR_FRAME),
                  observed_at)


class UnitreeBody:
    source = 'unitree'
    operator_supervised = False
    keyboard_hold = False
    allow_continuous_hold = False
    # 仅供显式启用的停车测量使用。默认不创建记录器，也不更改 SDK 参数或回执。
    parking_trace = None

    def __init__(self, interface, domain, *, site_forward_feedback=False, operator_supervised=False,
                 keyboard_hold=False, allow_continuous_hold=False):
        if not guard_ready():
            raise RuntimeError('实机发令端须由已放行的 a2-autonomy.service 运行')
        if keyboard_hold and not operator_supervised:
            raise ValueError('独立键盘输入只用于受管人工驾驶')
        if allow_continuous_hold and not (operator_supervised and keyboard_hold):
            raise ValueError('持续按住需要受管人工驾驶和独立方向键证据')
        # 接收器启动默认使用合成后端，只有显式选定实机后端才会加载 SDK 和接触 DDS。
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.a2.sport.sport_client import SportClient

        self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.receiver_id = 'unitree-a2-' + Path('/etc/machine-id').read_text().strip()
        self._handed_over_session = None
        self._site_observer = None
        self.operator_supervised = operator_supervised
        self.keyboard_hold = keyboard_hold
        self.allow_continuous_hold = allow_continuous_hold is True
        ChannelFactoryInitialize(domain, interface)
        self.sport = SportClient()
        self.sport.SetTimeout(1.)
        self.sport.Init()
        code, version = self.sport.GetServerApiVersion()
        if type(code) is not int or code != 0 or version != '1.0.0.1' or self.sport.GetApiVersion() != version:
            raise RuntimeError('A2 sport 服务或客户端的版本核对失败')
        self.sport.SetTimeout(.10)
        if site_forward_feedback or operator_supervised:
            from a2_remote_stop_sdk import ReadOnlyStandstill

            # 站立停稳和运动中的机身反馈来自独立只读订阅。
            observer = ReadOnlyStandstill(interface, domain)
            observer.start()
            self._site_observer = observer

    def close(self):
        if self._site_observer is not None:
            self._site_observer.close()

    def blocked_reason(self):
        return None if guard_ready() else 'stop_guard_not_ready'

    def _keyboard_sample(self):
        # 正式路径仅读取固定的 root 证据，浏览器和发令账号均无写权限。
        # 证据必须绑定到当前 PC2 开机和实际底盘，不能复用旧进程的跨开机记录。
        try:
            report = json.loads(trusted_file(KEYBOARD_HOLD), object_pairs_hook=_distinct_fields)
            fields = {'schema_version', 'boot_id', 'receiver_id', 'pressed', 'epoch', 'direction',
                      'observed_mono_ns'}
            if (not isinstance(report, dict) or report.keys() != fields or type(report['schema_version']) is not int
                    or report['schema_version'] != 1 or report['boot_id'] != self.boot_id
                    or report['receiver_id'] != self.receiver_id or type(report['pressed']) is not bool):
                return None
            stamp, epoch, direction = report['observed_mono_ns'], report['epoch'], report['direction']
            if (type(stamp) is not int or not 0 <= stamp < 2**63 or not isinstance(epoch, str)
                    or not 0 < len(epoch) <= 64 or report['pressed'] != (direction is not None)):
                return None
            observed_at = stamp / 1e9
            if not 0 <= time.monotonic() - observed_at < LIMITS.max_sample_age_s:
                return None
            return Sample(OperatorHold(report['pressed'], epoch, None if direction is None else Direction(direction)),
                          observed_at)
        except (OSError, ValueError, TypeError, RecursionError):
            return None

    def _local_grant(self):
        # 许可每次从 PC2 独立签发记录重新核对，旧合并快照中的 grant 无权放行。
        grant = grant_status(self.boot_id, self.receiver_id)['grant']
        if grant is None:
            return None
        return LocalGrant(grant['serial'], grant['issued_at'], grant['expires_at'],
                          frozenset(Direction(v) for v in grant['directions']),
                          site_front_confirmed=grant.get('site_front_confirmed', False),
                          repeated_actions=grant.get('repeated_actions', False),
                          continuous_hold=grant.get('continuous_hold', False))

    def _site_feedback(self):
        observer = getattr(self, '_site_observer', None)
        sample = None if observer is None else observer.motion_feedback()
        if sample is None:
            return None
        stamp = sample['observed_at']
        if not _finite(stamp) or not 0 <= time.monotonic() - stamp <= LIMITS.max_sample_age_s:
            return None
        return Sample(Feedback(sample['value']), stamp)

    def _site_guard(self, serial, guard):
        # 如果本轮见过独立监护，掉线时保留未知方向，直至该轮结束。
        if guard is not None:
            self._site_guard_serial = serial
        elif getattr(self, '_site_guard_serial', None) == serial:
            return Sample(frozenset(), -1.)
        return guard

    def _available_field_inputs(self):
        try:
            return self._field_inputs()
        except FileNotFoundError:
            return Inputs()

    def _field_inputs(self):
        # 方向净空和机身停稳由独立安全监护提供，且必须匹配本机本次启动。
        data = json.loads(trusted_file(SAFETY_DIRECTORY / 'evidence.json'))
        if data['boot_id'] != self.boot_id or data['receiver_id'] != self.receiver_id:
            raise ValueError('安全证据不属于本机本次启动')
        guard, feedback = data['guard'], data['feedback']
        # 站立/正常行走模式以外直接撤销许可，保持原厂接管语义；无起身、解除故障等指令。
        posture = tuple(feedback[k] for k in ('sport_mode', 'gait_type', 'error_code'))
        body_feedback = None
        if all(type(v) is int for v in posture) and posture == (3, 0, 0):
            body_feedback = Sample(Feedback(feedback['value']), feedback['observed_at'])
        checked_at = guard['observed_at']
        lidar_diagnostic = 'front_lidar' in data
        front_lidar = _front_lidar_sample(data['front_lidar']) if lidar_diagnostic else None
        tape_diagnostic = guard.get('source') == STATIC_CLEARANCE_SOURCE
        static_clearance = None
        if tape_diagnostic or lidar_diagnostic:
            # 只要出现诊断标记就关闭方向许可，诊断值缺失或误填方向也遵守同一规则。
            directions = frozenset()
        else:
            directions = frozenset(Direction(v) for v in guard['directions'])
        if tape_diagnostic:
            distance = guard.get('front_clearance_m')
            if _finite(distance) and distance >= 0 and _finite(checked_at) and checked_at >= 0:
                static_clearance = Sample(float(distance), checked_at)
        return Inputs(guard=Sample(directions, checked_at), feedback=body_feedback,
                      static_front_clearance=static_clearance, front_lidar=front_lidar)

    def observe(self):
        # 手柄反馈独立于场地及机身数据，接管时继续由原厂手柄优先控制。
        report = read_owner(self.boot_id, self.receiver_id)
        owner = Owner.NEUTRAL if neutral_epoch(report) is not None else Owner(report['value'])
        sample = None if report['observed_at'] is None else Sample(owner, report['observed_at'])
        grant = self._local_grant() if owner in (Owner.RELEASED, Owner.NEUTRAL) else None
        if self.operator_supervised:
            # 持续提供机身反馈，供起步前和撤权后确认停稳。场地净空由现场操作员负责。
            hold = self._keyboard_sample() if self.keyboard_hold else None
            return Inputs(owner=sample, feedback=self._site_feedback(), grant=grant, operator_hold=hold)
        try:
            inputs = self._available_field_inputs()
        except (OSError, ValueError, TypeError, KeyError, OverflowError, RecursionError):
            # 读取或校验异常须按未知保留，避免错误地当作独立监护从未配置。
            inputs = Inputs(guard=Sample(frozenset(), -1.))
        # 观察器独立于许可持续采样。许可结束后只把这路反馈交给停车确认。
        site_stop_feedback = self._site_feedback()
        if grant is not None and grant.site_front_confirmed:
            inputs = replace(inputs, guard=self._site_guard(grant.serial, inputs.guard), feedback=site_stop_feedback)
        return replace(inputs, owner=sample, grant=grant, site_stop_feedback=site_stop_feedback)

    def _site_output_allowed(self, request, grant):
        # 发送前再次检查方向、当前机身反馈和可能出现的独立方向监护。
        if request.linear_mps <= 0 or request.angular_radps or self._site_feedback() is None:
            return False
        try:
            directions = self._site_guard(grant['serial'], self._available_field_inputs().guard)
        except (OSError, ValueError, TypeError, KeyError, OverflowError, RecursionError):
            self._site_guard(grant['serial'], Sample(frozenset(), -1.))
            return False
        return directions is None or (
            _finite(directions.observed_at)
            and 0 <= time.monotonic() - directions.observed_at <= LIMITS.max_sample_age_s
            and Direction.FORWARD in directions.value
        )

    def velocity(self, request):
        # 在最后一次 SDK 调用前重新检查 admission，守护还会终止整个系统控制组，覆盖在途请求。
        if self.blocked_reason() is not None or time.monotonic() >= request.expires_at:
            return False
        # 直到调用 SDK 前都要求回中周期和本地许可持续有效。
        status = grant_status(self.boot_id, self.receiver_id)
        if status['reason'] == 'control_owned':
            self._handed_over_session = request.session_id
        grant = status['grant']
        if grant is None or grant['serial'] != request.grant_serial:
            return False
        continuous_hold = grant.get('continuous_hold') is True
        if continuous_hold and (
                not self.allow_continuous_hold or not self.keyboard_hold or not request.hold_epoch
                or grant.get('site_front_confirmed') is True or grant.get('repeated_actions') is True):
            return False
        if continuous_hold or grant.get('repeated_actions') is True:
            if not self.operator_supervised or grant['expires_at'] is not None:
                return False
            # 仲裁器已用动作时限（连续点动）或独立采样时效（持续按住）约束这条请求。
            deadline = request.expires_at
        else:
            deadline = min(grant['expires_at'], request.expires_at)
        if time.monotonic() >= deadline or self.blocked_reason() is not None:
            return False
        site_grant = grant.get('site_front_confirmed') is True
        if self.operator_supervised:
            # 净空由人负责。SDK 输出前仍复核签发方向和机身反馈的有效性。
            direction = command_direction(request.linear_mps, request.angular_radps)
            if direction is None or direction.value not in grant['directions'] or self._site_feedback() is None:
                return False
        elif site_grant and not self._site_output_allowed(request, grant):
            return False
        if self.operator_supervised or site_grant:
            current = grant_status(self.boot_id, self.receiver_id)
            if current['reason'] == 'control_owned':
                self._handed_over_session = request.session_id
            if (current['grant'] != grant or self.blocked_reason() is not None
                    or time.monotonic() >= deadline):
                return False
        if self.keyboard_hold:
            # 仲裁之后可能松键或换键。直接读一次独立证据，再进入 SDK 调用。
            hold = self._keyboard_sample()
            if hold is not None and continuous_hold and not (
                    0 <= time.monotonic() - hold.observed_at < LIMITS.max_sample_age_s):
                hold = None
            if (hold is None or not hold.value.pressed or hold.value.epoch != request.hold_epoch
                    or hold.value.direction is not direction or time.monotonic() >= deadline):
                if self.parking_trace is not None:
                    # 复用实际否决本次输出的同一份快照；再次读文件会引入另一个采样时刻。
                    if hold is None:
                        reason = 'hold_unavailable_or_stale'
                    elif not hold.value.pressed:
                        reason = 'hold_released'
                    elif hold.value.epoch != request.hold_epoch:
                        reason = 'hold_epoch_changed'
                    elif hold.value.direction is not direction:
                        reason = 'hold_direction_mismatch'
                    else:
                        reason = 'deadline_expired'
                    self.parking_trace.record('hold_gate_rejected', reason=reason,
                                              linear_mps=request.linear_mps, angular_radps=request.angular_radps)
                return False
        trace = self.parking_trace
        if trace is not None:
            trace.record('move_call', linear_mps=request.linear_mps, angular_radps=request.angular_radps)
        try:
            result = self.sport.Move(request.linear_mps, 0., request.angular_radps)
        except Exception:
            if trace is not None:
                trace.record('move_exception')
            raise
        accepted = type(result) is int and result == 0 and time.monotonic() < request.expires_at
        if trace is not None:
            trace.record('move_return', sdk_rc=result if type(result) is int else None)
        return accepted

    def stop(self, session_id, reason):
        # 已交给原厂手柄的旧会话不得补发 StopMove 抢占人工操作。停车守护的实体 A 键链路独立。
        if session_id == self._handed_over_session:
            return False
        if read_owner(self.boot_id, self.receiver_id)['value'] == 'owned':
            self._handed_over_session = session_id
            return False
        trace = self.parking_trace
        if trace is not None:
            trace.record('stop_move_call', reason=reason)
        try:
            result = self.sport.StopMove()
        except Exception:
            if trace is not None:
                trace.record('stop_move_exception', reason=reason)
            raise
        if trace is not None:
            trace.record('stop_move_return', reason=reason, sdk_rc=result if type(result) is int else None)
        return type(result) is int and result == 0


class MockBody:
    source = 'mock'
    receiver_id = 'mock-a2-body'

    def __init__(self, stop_file=None, emit=None, clock=time.monotonic, *, hold_file=None,
                 keyboard_hold=False, continuous_hold=False):
        if continuous_hold and (not keyboard_hold or hold_file is None):
            raise ValueError('持续按住模拟须提供方向快照文件')
        self.stop_file, self.emit, self.clock = stop_file, emit, clock
        self.hold_file = hold_file
        self.keyboard_hold = keyboard_hold
        self.continuous_hold = continuous_hold is True
        self.operator_supervised = self.continuous_hold
        self.latched = False
        self.linear = self.angular = 0.
        self.grant = None
        self.serial = 0

    def blocked_reason(self):
        # 模拟 A 键输入一经观测即锁存；删除输入文件不能从当前进程解除锁存。
        if self.stop_file is not None and self.stop_file.exists():
            self.latched = True
        return 'stop_latched' if self.latched else None

    def _hold_sample(self):
        """模拟输入由另一个本地进程发布。读取旧文件时沿用原采样时间。"""
        if self.hold_file is None:
            return None
        try:
            fd = os.open(self.hold_file, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            try:
                info = os.fstat(fd)
                if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_mode & 0o022:
                    return None
                raw = os.read(fd, 257)
            finally:
                os.close(fd)
            if len(raw) > 256:
                return None
            value = json.loads(raw)
            fields = {'pressed', 'epoch', 'observed_mono_ns'}
            if self.keyboard_hold:
                fields.add('direction')
            if not isinstance(value, dict) or value.keys() != fields or type(value['pressed']) is not bool:
                return None
            timestamp = value['observed_mono_ns']
            if type(timestamp) is not int or not 0 <= timestamp < 2**63:
                return None
            direction = value.get('direction')
            if self.keyboard_hold and value['pressed'] != (direction is not None):
                return None
            return Sample(OperatorHold(value['pressed'], value['epoch'],
                                       None if direction is None else Direction(direction)), timestamp / 1e9)
        except (OSError, ValueError, TypeError, RecursionError):
            return None

    def observe(self):
        now = self.clock()
        if self.continuous_hold:
            # 每个模拟接收器只签发一次本地候选许可，停车后需要重新运行隔离用例。
            if self.serial == 0:
                self.serial = 1
                self.grant = LocalGrant(self.serial, now, None, DIRECTIONS, continuous_hold=True)
        elif self.grant is None or now >= self.grant.expires_at:
            self.serial += 1
            self.grant = LocalGrant(self.serial, now, now + 60., DIRECTIONS)
        feedback = Feedback.MOVING if self.linear or self.angular else Feedback.STOPPED
        return Inputs(Sample(Owner.RELEASED, now), Sample(DIRECTIONS, now), Sample(feedback, now),
                      self.grant, operator_hold=self._hold_sample())

    def _event(self, kind, reason=None):
        if self.emit is not None:
            self.emit({'event': kind, 'source': self.source, 'monotonic_s': self.clock(),
                       'linear_mps': self.linear, 'angular_radps': self.angular, 'reason': reason})

    def velocity(self, request):
        if self.blocked_reason() or self.clock() >= request.expires_at:
            return False
        if self.continuous_hold:
            hold = self._hold_sample()
            now = self.clock()
            if (hold is None or not 0 <= now - hold.observed_at < LIMITS.max_sample_age_s
                    or not hold.value.pressed or hold.value.epoch != request.hold_epoch
                    or hold.value.direction is not command_direction(request.linear_mps, request.angular_radps)
                    or now >= request.expires_at):
                return False
        self.linear, self.angular = request.linear_mps, request.angular_radps
        self._event('move')
        return True

    def stop(self, session_id, reason):
        self.linear = self.angular = 0.
        self.grant = None
        self._event('stop', self.blocked_reason() or reason)
        return True


class _ObservedArbiter(MaintenanceArbiter):
    def _check_operator_hold(self, inputs, now, epoch, direction):
        reason = super()._check_operator_hold(inputs, now, epoch, direction)
        if epoch is None:
            # 只记录首次起步校验，复用仲裁实际看见的样本和时刻。不记录按压周期或会话。
            sample = inputs.operator_hold
            hold = sample.value if isinstance(sample, Sample) else None
            observed_at = sample.observed_at if isinstance(sample, Sample) else None
            pressed = hold.pressed if isinstance(hold, OperatorHold) and type(hold.pressed) is bool else None
            sampled_direction = hold.direction if isinstance(hold, OperatorHold) else None
            try:
                self._output.parking_trace.record(
                    'hold_start_check', decision_mono_ns=round(now * 1e9), reason=reason,
                    requested_direction=direction.value if isinstance(direction, Direction) else None,
                    sample_valid=reason != 'operator_hold_stale',
                    sample_observed_mono_ns=(round(observed_at * 1e9)
                                             if _finite(observed_at) and 0 <= observed_at < 2**63 / 1e9 else None),
                    sample_pressed=pressed,
                    sample_direction=sampled_direction.value if isinstance(sampled_direction, Direction) else None,
                )
            except Exception:
                # 诊断记录失败不得改变已计算的仲裁结果。
                pass
        return reason

    def _revoke(self, reason, inputs, now):
        # 撤权与 SDK 停车是独立事件，例如原厂手柄接管时只撤权。这里只记录仲裁给出的原因。
        self._output.parking_trace.record('lease_revoked', reason=reason)
        super()._revoke(reason, inputs, now)


class ManualReceiver:
    def __init__(self, body, *, clock=time.monotonic, allow_continuous_hold=False):
        self.body = body
        self.evidence_error = None
        mock_hold = isinstance(body, MockBody) and body.hold_file is not None
        real_keyboard = isinstance(body, UnitreeBody) and body.keyboard_hold
        if allow_continuous_hold and not ((real_keyboard and body.allow_continuous_hold)
                                           or (mock_hold and body.continuous_hold)):
            raise ValueError('持续按住须显式启用接收端和具备可信方向证据的后端')
        arbiter = _ObservedArbiter if getattr(body, 'parking_trace', None) is not None else MaintenanceArbiter
        self.arbiter = arbiter(LIMITS, self._observe, body, clock=clock, idle_deadman=True,
                               operator_supervised=getattr(body, 'operator_supervised', False) is True,
                               require_operator_hold=mock_hold,
                               require_hold_direction=real_keyboard or (mock_hold and body.keyboard_hold),
                               allow_continuous_hold=allow_continuous_hold)

    def _observe(self):
        try:
            inputs = self.body.observe()
        except (OSError, ValueError, TypeError, KeyError, RecursionError):
            self.evidence_error = 'local_safety_evidence_unavailable'
            return Inputs()
        self.evidence_error = None
        if self.body.blocked_reason():
            # 中断当前会话的本地授权，保留原厂接管和反馈语义，供仲裁器判定停车。
            return replace(inputs, grant=None)
        return inputs

    def handle(self, message):
        if not isinstance(message, dict) or type(message.get('schema_version')) is not int or message['schema_version'] != 1:
            return {'ok': False, 'reason': 'invalid_request'}
        op = message.get('op')
        payload = {k: v for k, v in message.items() if k not in ('op', 'schema_version')}
        if op == 'status' and not payload:
            control = self.arbiter.state()
            blocked = self.body.blocked_reason() or self.evidence_error
            if blocked:
                control.update(available=False, armed=False, reason=blocked)
                # 读取状态与复核守护之间若出现锁存，阻断回执不能继续携带动作信息。
                control.pop('action_state', None)
                control.pop('action_id', None)
            return {'ok': True, 'control': {**control, 'source': self.body.source}}
        if op not in ('acquire', 'velocity', 'stop', 'action_start', 'action_release'):
            return {'ok': False, 'reason': 'invalid_request'}
        blocked = self.body.blocked_reason()
        if op != 'stop' and blocked:
            self.arbiter.tick()
            return {'ok': False, 'reason': blocked}
        return getattr(self.arbiter, op)(payload)[1]

    def close(self):
        self.arbiter.close()
