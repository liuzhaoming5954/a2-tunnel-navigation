"""PC2 单点导航收据和状态机。ROS 回调与并发 HTTP 请求通过同一把锁串行更新。"""

import fcntl
import json
import math
from pathlib import Path
import re
import sqlite3
import threading
import time
from uuid import NAMESPACE_URL, uuid5


MAP_DIGEST = re.compile(r"[0-9a-f]{64}")
GOAL_ID = re.compile(r"[A-Za-z0-9_-]{1,128}")
FIELDS = {"goal_id", "map_sha256", "frame_id", "x_m", "y_m", "yaw_rad"}
TERMINAL = {"succeeded", "canceled", "failed"}


def _error(status, reason):
    return status, {"ok": False, "reason": reason}


def _number(value):
    if type(value) not in (float, int):
        raise ValueError
    value = float(value)
    if not math.isfinite(value):
        raise ValueError
    return value


def _goal(payload):
    if not isinstance(payload, dict) or set(payload) != FIELDS:
        raise ValueError
    if not isinstance(payload["goal_id"], str) or not GOAL_ID.fullmatch(payload["goal_id"]):
        raise ValueError
    if payload["frame_id"] != "map" or not isinstance(payload["map_sha256"], str):
        raise ValueError
    if not MAP_DIGEST.fullmatch(payload["map_sha256"]):
        raise ValueError
    target = dict(payload)
    for key in ("x_m", "y_m", "yaw_rad"):
        target[key] = _number(payload[key])
    if abs(target["yaw_rad"]) > math.pi:
        raise ValueError
    return target


def nav2_uuid(target):
    """同一地图、同一业务目标稳定映射为唯一 ROS UUID，断电后仍能与 Action 状态对照。"""
    return uuid5(NAMESPACE_URL, f"urn:a2:pc2:navigate:{target['map_sha256']}:{target['goal_id']}")


def _snapshot(target, phase, reason=None):
    ros_id = str(nav2_uuid(target)) if target["goal_id"] is not None else None
    return dict(target, phase=phase, reason=reason, distance_remaining_m=None, nav2_goal_uuid=ros_id)


class SingleGoalController:
    def __init__(self, map_sha256, backend, readiness=None, *, journal_path=None,
                 ack_timeout_s=5.0, check_target=None):
        if not isinstance(map_sha256, str) or not MAP_DIGEST.fullmatch(map_sha256):
            raise ValueError("请提供小写 SHA-256 地图摘要")
        self._map = map_sha256
        self._backend = backend
        self._readiness = readiness
        self._check_target = check_target
        self._timeout = ack_timeout_s
        self._lock = threading.RLock()
        self._changed = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._closing = False
        self._closed = False
        self._broken_store = False
        self._handle_known = False
        self._cancel_sent = False
        self._fault = None
        self._deadline = None
        self._owner = None
        self._db = None
        self._current = _snapshot(dict(goal_id=None, map_sha256=map_sha256, frame_id="map",
                                       x_m=None, y_m=None, yaw_rad=None), "idle")
        try:
            if journal_path is not None:
                # 每个导航服务独占机载记录；两个进程不能同时从同一个旧快照派发目标。
                p = Path(journal_path)
                self._owner = p.with_name(p.name + ".lock").open("a")
                fcntl.flock(self._owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._db = sqlite3.connect(journal_path or ":memory:", check_same_thread=False)
            self._db.execute("CREATE TABLE IF NOT EXISTS goals "
                             "(id TEXT PRIMARY KEY, target TEXT NOT NULL, phase TEXT NOT NULL, reason TEXT)")
            row = self._db.execute("SELECT target, phase, reason FROM goals ORDER BY rowid DESC LIMIT 1").fetchone()
            if row:
                target, phase, reason = row
                target = json.loads(target)
                # 已换地图时，以新图空闲状态开始；旧图仍在导航的目标必须连同其地图身份一起保留。
                if target["map_sha256"] == self._map or phase not in TERMINAL:
                    self._current = _snapshot(target, phase, reason)
                if phase not in TERMINAL:
                    # 重启丢失旧 ROS action 句柄，必须保留未决状态。不会重发或冒认已停车。
                    with self._lock:
                        self._set("uncertain", "restart_outcome_unknown")
            self._thread = threading.Thread(target=self._watch, name="a2-goal-health", daemon=True)
            self._thread.start()
        except BaseException:
            if self._db is not None:
                self._db.close()
            if self._owner is not None:
                self._owner.close()
            raise

    def _set(self, phase, reason=None):
        self._current.update(phase=phase, reason=reason)
        try:
            with self._db:
                self._db.execute("UPDATE goals SET phase=?, reason=? WHERE id=?",
                                 (phase, reason, self._current["goal_id"]))
        except sqlite3.Error:
            # 旧记录保留未确认的目标，写入失败后锁住本进程，防止新目标接替。
            self._broken_store = True
            self._current.update(phase="uncertain", reason="goal_journal_unavailable")
        self._changed.notify_all()

    def _reply(self):
        return 200, {"ok": True, "goal": self._current.copy()}

    def status(self):
        with self._lock:
            return self._reply()

    def _unavailable(self):
        # 使用机载端受控来源；浏览器不能在目标报文中自行声明定位已就绪。
        try:
            if self._readiness:
                reason = self._readiness()
                if reason:
                    return reason
            if not self._backend.available():
                return "nav2_unavailable"
        except Exception:
            return "navigation_readiness_unknown"
        return None

    def submit(self, payload):
        try:
            target = _goal(payload)
        except (ValueError, OverflowError):
            return _error(400, "invalid_goal")
        with self._lock:
            if self._closing or self._broken_store:
                return _error(503, "goal_controller_unavailable")
            if target["map_sha256"] != self._map:
                return _error(409, "map_mismatch")
            try:
                row = self._db.execute("SELECT target, phase, reason FROM goals WHERE id=?",
                                       (target["goal_id"],)).fetchone()
                if row:
                    previous, phase, reason = row
                    if json.loads(previous) != target:
                        return _error(409, "goal_id_conflict")
                    if self._current["goal_id"] == target["goal_id"]:
                        return self._reply()
                    return 200, {"ok": True, "goal": _snapshot(target, phase, reason)}
                if self._current["phase"] not in TERMINAL | {"idle"}:
                    return _error(409, "goal_busy")
                reason = self._unavailable()
                if reason:
                    return _error(503, reason)
                # 旧请求已优先返回原收据；只对首次派发的目标检查静态可通行性。
                if self._check_target is not None:
                    reason = self._check_target(target)
                    if reason:
                        return _error(422, reason)
                # 先落盘再向 ROS 发送，网络丢包或重复点击都只能对应一次派发。
                with self._db:
                    self._db.execute("INSERT INTO goals VALUES (?,?,?,NULL)",
                                     (target["goal_id"], json.dumps(target), "sending"))
            except sqlite3.Error:
                self._broken_store = True
                return _error(503, "goal_journal_unavailable")
            self._current = _snapshot(target, "sending")
            self._handle_known = self._cancel_sent = False
            self._fault = None
            self._deadline = time.monotonic() + self._timeout
            try:
                self._backend.send(target, lambda event, value=None:
                                   self._on_event(target["goal_id"], event, value))
            except Exception:
                # 异常时请求可能已经交给 DDS，等待确凿的终态。
                self._on_event(target["goal_id"], "uncertain", "goal_send_unknown")
            return self._reply()

    def _request_cancel(self, reason=None):
        if reason and not self._fault:
            self._fault = reason
        if self._current["phase"] not in {"canceling", "uncertain"}:
            self._set("canceling", self._fault)
            self._deadline = time.monotonic() + self._timeout
        if self._handle_known and not self._cancel_sent:
            self._cancel_sent = True
            try:
                self._backend.cancel(self._current["goal_id"])
            except Exception:
                self._set("uncertain", self._fault or "cancel_outcome_unknown")

    def cancel(self, payload):
        if (not isinstance(payload, dict) or set(payload) != {"goal_id"} or
                not isinstance(payload["goal_id"], str) or not GOAL_ID.fullmatch(payload["goal_id"])):
            return _error(400, "invalid_goal")
        with self._lock:
            if self._closing:
                return _error(503, "goal_controller_unavailable")
            if payload["goal_id"] != self._current["goal_id"]:
                return _error(409, "goal_not_current")
            if self._current["phase"] not in TERMINAL:
                self._request_cancel()
            return self._reply()

    def _on_event(self, ident, event, value=None):
        with self._lock:
            phase = self._current["phase"]
            if self._closed or ident != self._current["goal_id"] or phase in TERMINAL:
                return
            if event == "accepted":
                self._handle_known = True
                if phase == "sending":
                    self._deadline = None
                    self._set("active")
                elif phase in {"uncertain", "canceling"}:
                    self._request_cancel()
            elif event == "feedback" and phase == "active":
                try:
                    distance = _number(value)
                except (ValueError, OverflowError):
                    return
                if distance >= 0:
                    self._current["distance_remaining_m"] = distance
            elif event == "uncertain":
                self._deadline = None
                self._set("uncertain", self._fault or value or "nav2_outcome_unknown")
                self._request_cancel()
            elif event in ("succeeded", "canceled", "aborted", "rejected"):
                self._deadline = None
                if self._fault:
                    self._set("failed", self._fault)
                elif event in ("aborted", "rejected"):
                    self._set("failed", value or "nav2_" + event)
                else:
                    self._set(event)

    def _watch(self):
        while not self._stop.wait(0.1):
            with self._lock:
                if self._closing:
                    return
                phase = self._current["phase"]
                if self._broken_store and phase == "uncertain":
                    self._request_cancel("goal_journal_unavailable")
                if phase in {"active", "sending"}:
                    reason = self._unavailable()
                    if reason:
                        self._request_cancel(reason)
                if self._deadline is not None and time.monotonic() >= self._deadline:
                    self._on_event(self._current["goal_id"], "uncertain", "nav2_response_timeout")

    def close(self):
        with self._lock:
            if self._closing:
                return
            self._closing = True
            self._stop.set()
            if self._current["phase"] in {"active", "sending", "canceling"}:
                self._request_cancel("controller_shutdown")
                # 仍运行 ROS 执行器以接收取消结果；超时则持久保留未决状态。
                if not self._changed.wait_for(lambda: self._current["phase"] in TERMINAL, timeout=2.0):
                    self._set("uncertain", "shutdown_outcome_unknown")
            self._closed = True
        self._thread.join()
        try:
            self._backend.close()
        finally:
            self._db.close()
            if self._owner:
                self._owner.close()
