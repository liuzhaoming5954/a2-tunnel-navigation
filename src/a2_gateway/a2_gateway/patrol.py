"""开发机两点巡检状态机；调用者在同一个模拟器锁内提交点位、更新反馈和读取快照。"""

from dataclasses import asdict, dataclass
import math


# 以下容差和时间窗口只用于理想二维运动模型，单位分别为米和秒。
ARRIVAL_M = 0.02
STOP_CONFIRM_S = 0.20


@dataclass(frozen=True)
class Point:
    sequence: int
    x_m: float
    y_m: float
    dwell_s: float


def parse_point(payload):
    """HTTP 字段严格校验。布尔值、无穷大或不能表示为模拟坐标的数都不能进入控制循环。"""
    if not isinstance(payload, dict) or set(payload) != {"mission_id", "sequence", "x_m", "y_m", "dwell_s"}:
        raise ValueError
    mission = payload["mission_id"]
    seq = payload["sequence"]
    if not isinstance(mission, str) or not mission.strip() or len(mission) > 128:
        raise ValueError
    if type(seq) is not int or seq not in (1, 2):
        raise ValueError
    values = [payload[key] for key in ("x_m", "y_m", "dwell_s")]
    if any(type(value) not in (float, int) for value in values):
        raise ValueError
    try:
        x, y, dwell = map(float, values)
    except OverflowError as error:
        raise ValueError from error
    if not all(map(math.isfinite, (x, y, dwell))) or dwell < 0:
        raise ValueError
    return mission, Point(seq, x, y, dwell)


class Patrol:
    def __init__(self):
        self.mission_id = None
        self.phase = "idle"
        self.points = []
        self.active = None
        self.events = []
        self.reason = None
        self.started = 0.0
        self.stationary_since = None
        self.dwell_started = None
        # 结束任务的标识只在本次进程中保留，防止迟到的旧请求重新驱动机身。
        self.finished_ids = set()

    @property
    def owns_robot(self):
        return self.phase in ("moving", "settling", "dwelling", "waiting")

    def _event(self, seq, name, now):
        self.events.append({"sequence": seq, "event": name, "time_s": now - self.started})

    def _depart(self, point, now):
        self.active = point
        self.stationary_since = None
        self.dwell_started = None
        self.phase = "moving"
        self._event(point.sequence, "departed", now)

    def accept(self, mission, point, now):
        """只接受按次序提交的两个点，点 2 可提前排队，也可在点 1 停留完成后再下发。"""
        if mission != self.mission_id:
            if mission in self.finished_ids:
                return "patrol_finished"
            if self.owns_robot:
                return "patrol_busy"
            if point.sequence != 1:
                return "patrol_out_of_order"
            if self.mission_id is not None:
                self.finished_ids.add(self.mission_id)
            self.mission_id = mission
            self.phase, self.active, self.reason = "idle", None, None
            self.started, self.events, self.points = now, [], []
        if point.sequence <= len(self.points):
            return None if self.points[point.sequence - 1] == point else "patrol_point_conflict"
        if self.phase == "failed":
            return "patrol_finished"
        if point.sequence != len(self.points) + 1:
            return "patrol_out_of_order"
        self.points.append(point)
        self._event(point.sequence, "accepted", now)
        if self.active is None:
            self._depart(point, now)
        return None

    def update(self, now, x, y, yaw, linear, angular, max_linear, max_angular):
        """依据同一机身的实际位姿和速度产生下一周期的请求；每次只执行当前点。"""
        point = self.active
        if point is None:
            return 0.0, 0.0
        if self.phase == "moving":
            dx, dy = point.x_m - x, point.y_m - y
            distance = math.hypot(dx, dy)
            if distance > ARRIVAL_M:
                # 先在原地对准方位，再向目标点前进，合成地图碰撞仍由模拟器逐步核对。
                angle = math.atan2(dy, dx) - yaw
                angle = math.atan2(math.sin(angle), math.cos(angle))
                turn = max(-max_angular, min(max_angular, 4 * angle))
                forward = 0.0 if abs(angle) > 0.08 else min(max_linear, 4 * distance)
                return forward, turn
            self.phase = "settling"
            self._event(point.sequence, "arrived", now)
        if self.phase == "settling":
            # 到点先请求零速；观察到机身线速度与角速度持续为零才发停稳事件并开始停留。
            if linear != 0.0 or angular != 0.0:
                self.stationary_since = None
            elif self.stationary_since is None:
                self.stationary_since = now
            elif now - self.stationary_since >= STOP_CONFIRM_S:
                self.phase = "dwelling"
                self.dwell_started = now
                self._event(point.sequence, "stopped", now)
        if self.phase == "dwelling" and now - self.dwell_started >= point.dwell_s:
            self._event(point.sequence, "dwell_finished", now)
            self.active = None
            if point.sequence == 2:
                self.phase = "completed"
                self._event(point.sequence, "completed", now)
            elif len(self.points) == 2:
                self._depart(self.points[1], now)
            else:
                self.phase = "waiting"
        return 0.0, 0.0

    def fail(self, reason, now):
        # 单条直线遇障时保留已完成和失败点的事件，停在最近的可通行位置。
        if self.active:
            self._event(self.active.sequence, "failed", now)
        self.phase, self.reason, self.active = "failed", reason, None

    def snapshot(self, now):
        remaining = 0.0
        if self.phase == "dwelling":
            remaining = max(0.0, self.active.dwell_s - (now - self.dwell_started))
        return {"mission_id": self.mission_id, "phase": self.phase,
                "active_sequence": self.active.sequence if self.active else None,
                "points": [asdict(p) for p in self.points], "remaining_s": remaining,
                "events": [event.copy() for event in self.events], "reason": self.reason}
