"""独立停车进程的 A2 SDK 适配。仅在 start() 时接入指定网卡。"""

from collections import deque
import math
from threading import Lock
from time import monotonic
from typing import NamedTuple


REMOTE_TOPIC = "rt/lf/lowstate"
SPORT_TOPIC = "rt/lf/sportmodestate"
API_VERSION = "1.0.0.1"

# 沿用首次受控试验的速度、角速度、倾角、漂移和保持时间。每路实测约 20 Hz。
HOLD_S = 0.50
MAX_GAP_S = 0.20
STILL_MPS = 0.016
# 驾驶舱点动仲裁的只读反馈使用独立线速度阈值，单位为米/秒。
MANUAL_STILL_MPS = 0.010
STILL_RAD_S = 0.05
MAX_TILT_RAD = math.radians(12)
MAX_DRIFT_M = 0.015
# 原厂 A2 状态枚举。下趴的关节与加速度门限为开发机离线候选，单位分别为 rad/s、rad、m/s²。
STAND_DOWN = 1
DEFAULT_MODE = 3
DOWN_MAX_JOINT_RAD_S = 0.10
DOWN_MAX_JOINT_SPAN_RAD = 0.001
DOWN_MAX_ACCEL_RESIDUAL_MPS2 = 0.50
GRAVITY_MPS2 = 9.80665
_FRAME_ERRORS = (ValueError, TypeError, OverflowError, AttributeError)


def _vector(values, size):
    if len(values) != size or not all(math.isfinite(v) for v in values):
        raise ValueError("向量长度或数值异常")
    return tuple(values)


def _upright_gyro(imu):
    """按 A2 的 w,x,y,z 顺序核对姿态，再读取有限的三轴角速度。"""
    w, x, y, z = _vector(imu.quaternion, 4)
    norm = math.hypot(w, x, y, z)
    if not 0.95 <= norm <= 1.05:
        return None
    cosine = 1 - 2 * (x*x + y*y) / (norm*norm)
    return _vector(imu.gyroscope, 3) if cosine >= math.cos(MAX_TILT_RAD) else None


def _level_imu(imu):
    gyro = _upright_gyro(imu)
    return gyro is not None and max(map(abs, gyro)) <= STILL_RAD_S


def _forward_tick(previous, tick):
    """按 uint32 循环计数判断先后，普通倒退和重复不能用作新鲜度证据。"""
    return 0 < (tick - previous) % 2**32 < 2**31


class _DownFeedback(NamedTuple):
    q: tuple
    dq: tuple
    accel: tuple
    gyro: tuple


def _down_feedback(message):
    """调用方已核对四元数、倾角、角速度和 tick。只接收同一低层帧中的实测关节和 IMU。"""
    imu = message.imu_state
    # 将静止重力在机身三轴上的预期读数与加速度计比对，保留对水平加速的敏感性。
    norm = math.hypot(*imu.quaternion)
    w, x, y, z = (v / norm for v in imu.quaternion)
    gravity_axis = (2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y))
    accel = _vector(imu.accelerometer, 3)
    if math.dist(accel, tuple(GRAVITY_MPS2 * axis for axis in gravity_axis)) > DOWN_MAX_ACCEL_RESIDUAL_MPS2:
        return None

    # 前十二个槽位是四条腿的实测 q、dq；后续预留槽位不参与判断。
    legs = message.motor_state[:12]
    q = _vector(tuple(m.q for m in legs), 12)
    dq = _vector(tuple(m.dq for m in legs), 12)
    if max(map(abs, dq)) > DOWN_MAX_JOINT_RAD_S or any(not any(q[i:i+3]) for i in range(0, 12, 3)):
        return None
    return _DownFeedback(q, dq, accel, tuple(imu.gyroscope))


def _down_window_still(rows, now):
    """分别检查 IMU 和关节载荷的更新时间；设备 tick 单独前进不能证明反馈仍在刷新。"""
    first_rx, previous, _ = rows[0]
    joint_update = imu_update = first_rx
    for rx, sample, _ in rows[1:]:
        # 即使其中一路刚恢复，也要等之前的冻结区间移出整个观测窗口。
        if rx - min(joint_update, imu_update) >= MAX_GAP_S:
            return False
        if sample.q != previous.q or sample.dq != previous.dq:
            joint_update = rx
        if sample.accel != previous.accel or sample.gyro != previous.gyro:
            imu_update = rx
        previous = sample
    spans = (max(axis) - min(axis) for axis in zip(*(sample.q for _, sample, _ in rows)))
    return now - min(joint_update, imu_update) < MAX_GAP_S and max(spans) <= DOWN_MAX_JOINT_SPAN_RAD


def _dds_writer(info):
    writer = getattr(info, "publication_handle", None)
    if getattr(info, "valid_data", None) is not True or type(writer) is not int or not 0 < writer < 2**64:
        raise ValueError("DDS 样本无效或写者不可辨认")
    return writer


def _dds_clock(info, writer):
    source = getattr(info, "source_timestamp", None)
    if writer is None or type(source) is not int or not 0 < source < 2**63:
        raise ValueError("缺少有效的 DDS 写者及源时间")
    return ("dds.source_timestamp", writer), source


def sport_clock(message):
    """正常设备时戳优先。全零占位时，使用有效 DDS 样本的发布端时间及写者标识。"""
    sec, ns = message.stamp.sec, message.stamp.nanosec
    if type(sec) is not int or sec < 0 or type(ns) is not int or not 0 <= ns < 10**9:
        raise ValueError("SportModeState.stamp 无效")
    stamp = sec * 10**9 + ns
    info = getattr(message, "sample_info", None)
    writer = _dds_writer(info) if info is not None else None
    if stamp > 0:
        return ("sport.stamp", writer), stamp
    return _dds_clock(info, writer)


class _Samples:
    """仅保存最近一段连续合格帧，间断、设备时序倒退或坏帧均打断证据。"""

    def __init__(self, *, wrapping_tick=False):
        self.frames = deque(maxlen=1024)
        self.last_sequence = None
        self.wrapping_tick = wrapping_tick

    def append(self, now, sequence, value):
        """返回当前样本是否进入合格窗口，便于在同一帧内复用 IMU 和计数检查。"""
        if sequence is None:
            self.frames.clear()
            return False
        if self.last_sequence is not None:
            advanced = (_forward_tick(self.last_sequence, sequence) if self.wrapping_tick
                        else sequence > self.last_sequence)
            if not advanced:
                # 不用迟到帧重设基线，否则一串旧帧可能被误算成新收到的停稳数据。
                self.frames.clear()
                return False
        self.last_sequence = sequence
        if value is None:
            self.frames.clear()
            return False
        if self.frames and not 0 < now - self.frames[-1][0] < MAX_GAP_S:
            self.frames.clear()
        self.frames.append((now, value, sequence))
        while self.frames and now - self.frames[0][0] > HOLD_S + 2 * MAX_GAP_S:
            self.frames.popleft()
        return True


class _TimedSamples(_Samples):
    """按写者及时间来源隔离递增水位，防止积压、乱序或切换时沿用上一段停稳证据。"""

    def __init__(self):
        super().__init__()
        self.clock = None
        self.watermarks = {}

    def receive(self, now, clock, stamp, value):
        if self.clock != clock:
            self.frames.clear()
            self.last_sequence = None
            self.clock = clock
        if stamp is None:
            self.frames.clear()
            return
        now_ns = round(now * 10**9)
        offset = now_ns - stamp
        previous = self.watermarks.get(clock)
        if previous is not None:
            high, previous_rx, fastest = previous
            # 发布时钟无须与 PC2 对时。按同一来源的最短相对传输延时拒绝过迟的帧，
            # 并容许时钟有千分之一的慢漂。PC2 和发布时钟均须连续推进。
            if stamp <= high:
                self.frames.clear()
                return
            fastest = min(offset, fastest + max(0, now_ns - previous_rx) // 1000)
            if stamp - high >= MAX_GAP_S * 10**9 or offset - fastest >= MAX_GAP_S * 10**9:
                self.frames.clear()
            if offset - fastest >= MAX_GAP_S * 10**9:
                value = None
        else:
            fastest = offset
        self.watermarks[clock] = stamp, now_ns, fastest
        self.append(now, stamp, value)

    def window(self, trigger, start, end):
        """保留触发后覆盖公共窗口的一段，同时检查源时间和接收时间，拒绝集中投递的积压帧。"""
        last = next((row for row in reversed(self.frames) if row[0] <= end), None)
        if last is None:
            return []
        for i in reversed(range(len(self.frames))):
            rx, _, src = self.frames[i]
            if trigger <= rx <= start:
                source_s = (last[2] - src) / 10**9
                receive_s = last[0] - rx
                if (source_s >= HOLD_S and receive_s >= HOLD_S
                        and abs(source_s - receive_s) <= MAX_GAP_S / 2):
                    # 也检查较快一路已经到达的最新样本，避免越限数据落在公共终点之后而被遗漏。
                    return list(self.frames)[i:]
        return []

    def recent_end(self, begin, now):
        """取连续两帧的末端，扣除相对运输延时，避免查询或迟到帧刷新样本。"""
        if len(self.frames) < 2:
            return None
        prior, latest = self.frames[-2], self.frames[-1]
        if (prior[0] < begin
                or abs((latest[2] - prior[2]) / 10**9 - (latest[0] - prior[0])) > MAX_GAP_S / 2):
            return None
        _, received_ns, fastest_offset = self.watermarks[self.clock]
        transport_delay = max(0, received_ns - latest[2] - fastest_offset) / 10**9
        observed_at = latest[0] - transport_delay
        return observed_at if observed_at >= begin and 0 <= now - observed_at < MAX_GAP_S else None


class StopHardware:
    def __init__(self, interface, domain, on_keys, on_bad_frame, *, observe_only=False, feedback_only=False):
        if not isinstance(interface, str) or not interface.strip() or type(domain) is not int or domain < 0:
            raise ValueError("需要指定 PC2 网卡及非负 DDS 域编号")
        if observe_only and feedback_only:
            raise ValueError("只读识键和只读停稳应分别启动")
        self.interface = interface
        self.domain = domain
        self.on_keys = on_keys
        self.on_bad_frame = on_bad_frame
        self.observe_only = observe_only
        self._feedback_only = feedback_only
        self._still_mps = MANUAL_STILL_MPS if feedback_only else STILL_MPS
        self._sport = None
        self._readers = []
        self._started = False
        self._active = False
        self._remote_lock = Lock()
        self._last_radio_tick = None
        self._state_lock = Lock()
        self._posture = None
        self._posture_since = 0.
        self._body = _TimedSamples()
        self._imu = _Samples(wrapping_tick=True)
        self._down = _TimedSamples()
        self._motion_body = _TimedSamples()
        self._motion_imu = _TimedSamples()

    def start(self):
        """先完成求版本和停车 RPC 的客户端初始化，再开放遥控输入；失败时撤销订阅。"""
        if self._started:
            raise RuntimeError("这个停车适配器已启动过")
        self._started = True
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        ChannelFactoryInitialize(self.domain, self.interface)
        subscriptions = []
        if not self.observe_only and not self._feedback_only:
            from unitree_sdk2py.a2.sport.sport_client import SportClient

            sport = SportClient()
            sport.SetTimeout(1.0)
            sport.Init()
            local = sport.GetApiVersion()
            code, remote = sport.GetServerApiVersion()
            if type(code) is not int or code != 0 or local != API_VERSION or remote != API_VERSION:
                raise RuntimeError(f"sport 版本核对失败，客户端 {local}，服务端 {remote}，返回码 {code}")
            # 这是单次 RPC 的发送和响应等待参数。它不代表从按键到物理停稳的时限。
            sport.SetTimeout(0.15)
            self._sport = sport
        if not self.observe_only:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

            subscriptions.append((SPORT_TOPIC, SportModeState_, self._receive_sport))
        subscriptions.append((REMOTE_TOPIC, LowState_, self._receive_remote))

        self._active = True
        try:
            for topic, message_type, callback in subscriptions:
                reader = ChannelSubscriber(topic, message_type)
                self._readers.append(reader)
                # 默认 queueLen=0，逐帧在 DDS 回调中置位。此处不阻塞等待停车 RPC。
                reader.Init(callback)
        except BaseException:
            self.close()
            raise

    def close(self):
        with self._remote_lock:
            self._active = False
        readers, self._readers = self._readers, []
        first_error = None
        try:
            for reader in reversed(readers):
                try:
                    reader.Close()
                except Exception as exc:
                    # 即便一条 DDS 订阅的清理失败，另一条也需要关闭。
                    if first_error is None:
                        first_error = exc
        finally:
            self._sport = None
        if first_error is not None:
            raise first_error

    def stop_move(self):
        """只向 A2 sport 发送 StopMove。底盘回执和连续停稳反馈分别判定。"""
        sport = self._sport
        if sport is None or not self._active:
            raise RuntimeError("停车客户端未启用")
        code = sport.StopMove()
        if type(code) is not int:
            raise RuntimeError(f"停车返回码类型异常：{code!r}")
        return code

    def _receive_remote(self, message):
        # 两个字段独立解码。遥控按键无需等待 IMU 合格，也不依赖摇杆是否回中。
        # 串行送出每帧按键，调用方立即锁存，避免按下和松开在两次轮询间丢失。
        with self._remote_lock:
            if not self._active:
                return
            now = monotonic()
            tick = getattr(message, "tick", None)
            if type(tick) is not int or not 0 <= tick < 2**32:
                tick = None
            if not self._feedback_only:
                try:
                    data = message.wireless_remote
                    if len(data) != 40:
                        raise ValueError("wireless_remote 应有 40 字节")
                    keys = int.from_bytes(bytes(data[2:4]), "little")
                except _FRAME_ERRORS as exc:
                    self.on_bad_frame(str(exc))
                else:
                    if self.observe_only:
                        self.on_keys(keys)
                    else:
                        self._dispatch_guard_keys(keys, tick)
            if self.observe_only:
                return
            valid = None
            try:
                if _level_imu(message.imu_state):
                    valid = True
            except _FRAME_ERRORS:
                pass
            with self._state_lock:
                if self._feedback_only:
                    self._receive_motion_imu(message, now, tick)
                imu_accepted = self._imu.append(now, tick, valid)
                if self._posture == STAND_DOWN:
                    # tick 先经原有水位检查；无效帧立即打断下趴窗口，遥控按键已在上方处理。
                    clock = stamp = feedback = None
                    try:
                        info = message.sample_info
                        clock, stamp = _dds_clock(info, _dds_writer(info))
                        if imu_accepted:
                            feedback = _down_feedback(message)
                    except _FRAME_ERRORS:
                        pass
                    self._down.receive(now, clock, stamp, feedback)

    def _receive_motion_imu(self, message, now, tick):
        clock = stamp = valid = None
        try:
            info = message.sample_info
            clock, stamp = _dds_clock(info, _dds_writer(info))
            previous = self._imu.last_sequence
            imu = message.imu_state
            acceleration = math.hypot(*_vector(imu.accelerometer, 3))
            if (tick is not None and (previous is None or _forward_tick(previous, tick))
                    and _upright_gyro(imu) is not None and 0 < acceleration < math.inf):
                # 运动时允许实测角速度高于静止门槛，仍须能核对姿态、低层计数和 DDS 写者。
                valid = True
        except _FRAME_ERRORS:
            pass
        self._motion_imu.receive(now, clock, stamp, valid)

    def _dispatch_guard_keys(self, keys, tick):
        """停车位总是优先处理。只有持续推进的 uint32 帧计数才能为遥控看门狗续期。"""
        # 首帧可以按停，但单个设备计数值无法证明当前链路正在刷新。
        # 半个计数周期内按无符号序号判断先后，包括 0xffffffff 正常回绕到零。
        previous = self._last_radio_tick
        advanced = tick is not None and previous is not None and _forward_tick(previous, tick)
        if tick is not None and (previous is None or advanced):
            self._last_radio_tick = tick
        if advanced:
            self.on_keys(keys)
        elif keys:
            # 冻结或乱序期间，短暂出现的停车位仍立即送达；其他键不能延长遥控有效期。
            self.on_keys(keys, fresh=False)

    def _receive_sport(self, message):
        with self._state_lock:
            if not self._active:
                return
            now = monotonic()
            sequence = None
            position = None
            clock = None
            posture = None
            try:
                clock, sequence = sport_clock(message)
                if message.gait_type == 0 and message.error_code == 0 and type(message.mode) is int:
                    if message.mode == DEFAULT_MODE:
                        posture = DEFAULT_MODE
                        velocity = _vector(message.velocity, 3)
                        if math.hypot(*velocity) <= self._still_mps and abs(message.yaw_speed) <= STILL_RAD_S:
                            position = _vector(message.position, 3)
                    elif message.mode == STAND_DOWN:
                        posture = STAND_DOWN
                        # 下趴模式会重复发送非零速度。位置只用于否决可见位移，
                        # 姿态、加速度、各腿关节的实际反馈都必须独立通过。
                        position = _vector(message.position, 3)
            except _FRAME_ERRORS:
                sequence = None
            if posture != self._posture:
                # 切换姿态后重新累计，不清空各 DDS 写者的时间水位。
                self._body.frames.clear()
                self._down.frames.clear()
                self._motion_body.frames.clear()
                self._motion_imu.frames.clear()
                self._posture = posture
                self._posture_since = now
            self._body.receive(now, clock, sequence, position)
            if self._feedback_only:
                self._receive_motion_sport(message, now, clock, sequence)

    def _receive_motion_sport(self, message, now, clock, sequence):
        moving = None
        try:
            info = message.sample_info
            _dds_clock(info, _dds_writer(info))
            if (type(message.mode) is int and message.mode == DEFAULT_MODE
                    and type(message.gait_type) is int and message.gait_type == 0
                    and type(message.error_code) is int and message.error_code == 0):
                _vector(message.position, 3)
                speed = math.hypot(*_vector(message.velocity, 3))
                yaw_rate = abs(message.yaw_speed)
                if math.isfinite(speed) and math.isfinite(yaw_rate):
                    moving = speed > self._still_mps or yaw_rate > STILL_RAD_S
        except _FRAME_ERRORS:
            pass
        self._motion_body.receive(now, clock, sequence, moving)

    def still_since(self, trigger_monotonic):
        """检查触发后最近的共同保持窗口。重复查询缓存无法延长观测时间。"""
        return self._standstill_time(trigger_monotonic) is not None

    def _standstill_time(self, trigger_monotonic, *, standing_only=False):
        """返回两路停稳样本共同末端的本机时间。缺少停稳证据时返回 None。"""
        with self._state_lock:
            return self._standstill_locked(trigger_monotonic, monotonic(), standing_only=standing_only)

    def _standstill_locked(self, trigger_monotonic, now, *, standing_only=False):
        body = self._body.frames
        down = self._posture == STAND_DOWN
        low = self._down.frames if down else self._imu.frames
        if not self._active or not body or not low or not math.isfinite(trigger_monotonic):
            return None
        if standing_only and self._posture != DEFAULT_MODE:
            return None
        if any(not 0 <= now - frames[-1][0] < MAX_GAP_S for frames in (body, low)):
            return None
        # 两路取时间交集；各取不晚于窗口起点的一帧。只承认触发之后取得的样本。
        trigger = max(trigger_monotonic, self._posture_since)
        end = min(body[-1][0], low[-1][0])
        start = end - HOLD_S
        if not any(trigger <= t <= start for t, _, _ in low):
            return None
        body_window = self._body.window(trigger, start, end)
        if not body_window:
            return None
        # XYZ 包围盒对角线给出任意两样本距离的上界，包含静止速度阈值以下的缓慢漂移。
        positions = [position for _, position, _ in body_window]
        if math.hypot(*(max(axis) - min(axis) for axis in zip(*positions))) > MAX_DRIFT_M:
            return None
        if down:
            samples = self._down.window(trigger, start, end)
            if not samples or not _down_window_still(samples, now):
                return None
        return end

    def _motion_feedback(self, began_at, *, include_settling=False):
        with self._state_lock:
            if not self._active or self._posture != DEFAULT_MODE or not math.isfinite(began_at):
                return None
            now = monotonic()
            begin = max(began_at, self._posture_since)
            body_end = self._motion_body.recent_end(begin, now)
            imu_end = self._motion_imu.recent_end(begin, now)
            if body_end is None or imu_end is None or abs(body_end - imu_end) > MAX_GAP_S / 2:
                return None
            evidence_end = min(body_end, imu_end)
            if self._motion_body.frames[-1][1]:
                return "moving", evidence_end
            # 两路仍新鲜匹配，但刚回到低速或漂移超限时，须重新累计停稳窗口。
            # 仅应显式调用方要求报告这段过渡状态，停稳判据与样本时间保持同一来源。
            stopped_at = self._standstill_locked(began_at, now, standing_only=True)
            if stopped_at is None or not all(stream.window(begin, stopped_at - HOLD_S, stopped_at)
                                             for stream in (self._motion_body, self._motion_imu)):
                return ("settling", evidence_end) if include_settling else None
            return "stopped", min(evidence_end, stopped_at)


class ReadOnlyStandstill:
    """PC2 静止检查只读订阅端。停稳证据与运动、方向和手柄权属分别采集。"""

    def __init__(self, interface, domain):
        self._hardware = StopHardware(interface, domain, None, None, feedback_only=True)
        self._began_at = monotonic()

    def start(self):
        self._hardware.start()

    def close(self):
        self._hardware.close()

    def feedback(self):
        # 点动仲裁器只接受正常站立、步态零且无错误的机身反馈。
        # 使用共同样本末端，而不是本次查询时间，否则反复发布旧窗口会推迟过期。
        stamp = self._hardware._standstill_time(self._began_at, standing_only=True)
        if stamp is None:
            return None
        return {"value": "stopped", "observed_at": stamp, "sport_mode": DEFAULT_MODE,
                "gait_type": 0, "error_code": 0}

    def motion_feedback(self, *, include_settling=False):
        """提供双路实测运动或确认后的停稳反馈，不向底盘发令。"""
        report = self._hardware._motion_feedback(self._began_at, include_settling=include_settling)
        if report is None:
            return None
        value, stamp = report
        return {"value": value, "observed_at": stamp, "sport_mode": DEFAULT_MODE,
                "gait_type": 0, "error_code": 0}
