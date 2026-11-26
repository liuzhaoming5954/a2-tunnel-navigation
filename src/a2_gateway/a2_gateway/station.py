"""上位机保管地图，从固定 PC2 获取遥测，启用单点导航时转发目标和取消请求。"""

from hashlib import sha256
from http.client import HTTPConnection, HTTPException
import json
import math
from pathlib import Path
import re
import time
from urllib.parse import urlsplit

from . import manual_link
from . import remote_stop_status
from .manual_link import PC2_STOP_RESET_PATH, valid_stop_reset_confirmation, valid_stop_reset_reason
from .odin1_pose import MAX_AGE_MS, SOURCE, read_odin_object
from .platform_details import fresh_details


_MAX_RESPONSE_BYTES = 65_536
_TIMEOUT_SECONDS = 0.8
_STOP_RESET_TIMEOUT_SECONDS = 24.0
_SHA256 = re.compile(r"[0-9a-f]{64}")
_GOAL_PATH = "/api/navigation/goal"
_GOAL_PHASES = {"idle", "sending", "active", "canceling", "succeeded", "canceled", "failed", "uncertain"}
_TARGET = ("x_m", "y_m", "yaw_rad")
_ROBOT_FIELDS = ("id", "model", "mode", "battery_percent", "linear_mps", "angular_radps",
                 "telemetry_stamp_ms", "telemetry_age_ms")


def _valid_goal_id(value):
    return isinstance(value, str) and re.fullmatch(r"[A-Za-z0-9_-]{1,128}", value) is not None


def _finite(value):
    # JSON 可容纳任意精度整数。先检查其值可表示为 ROS 浮点数，再转给 PC2。
    try:
        return type(value) in (float, int) and math.isfinite(value)
    except OverflowError:
        return False


def _valid_target(goal):
    return all(_finite(goal.get(key)) for key in _TARGET) and abs(goal["yaw_rad"]) <= math.pi


def _robot_fields(message, elapsed_ms):
    """只接受 PC2 实机接口。模拟器、旧网关和其他上位机均不能产生实机在线状态。"""
    if message.get("deployment") != "pc2" or message.get("mode") != "robot_readonly":
        raise ValueError("数据源不是 PC2 只读服务")
    robot = message.get("robot")
    if not isinstance(robot, dict) or type(robot.get("online")) is not bool:
        raise ValueError("缺少机器人状态")
    fields = {key: robot.get(key) for key in _ROBOT_FIELDS}
    for key in ("id", "model", "mode"):
        if fields[key] is not None and not isinstance(fields[key], str):
            raise ValueError("机器人标识、型号或模式无效")
    for key in ("battery_percent", "linear_mps", "angular_radps", "telemetry_stamp_ms", "telemetry_age_ms"):
        value = fields[key]
        if value is not None and (type(value) not in (int, float) or not math.isfinite(value)):
            raise ValueError("机器人遥测数值无效")
    # 断流时可保留设备标识，上一帧的电量、速度与模式不能随离线状态透传。
    if not robot["online"]:
        fields = {key: fields[key] if key in ("id", "model") else None for key in _ROBOT_FIELDS}
    # 各路详情独立判定。与 ODIN1 位姿转发相同，站端剔除途中已失效的读数，
    # age_ms 保留 PC2 创建快照时的年龄，浏览器据此扣除整次请求的往返时间。
    return {**fields, **fresh_details(robot, transfer_ms=elapsed_ms), "online": robot["online"]}


def _remote_stop(sample, transfer_ms):
    """校验独立守护的采样；旧版采集器缺少此字段仍可提供其他遥测。"""
    maximum = remote_stop_status.MAX_AGE_MS
    unknown = {"source": remote_stop_status.SOURCE, "guard": "unknown", "latched": None,
               "age_ms": maximum, "max_age_ms": maximum}
    if not isinstance(sample, dict) or sample.get("source") != remote_stop_status.SOURCE:
        return unknown
    guard, latched = sample.get("guard"), sample.get("latched")
    age, lifetime = sample.get("age_ms"), sample.get("max_age_ms")
    if (guard not in ("running", "stopped", "unknown") or (latched is not None and type(latched) is not bool)
            or type(age) is not int or age < 0 or type(lifetime) is not int
            or lifetime != maximum or age + transfer_ms >= lifetime):
        return unknown
    # 只接受运行中守护对“已解除”的肯定证据。页端会扣除浏览器自己的完整请求耗时，
    # 因此仅在此用站端往返时间剔除过期样本，原始年龄保持不变。
    return {"source": remote_stop_status.SOURCE, "guard": guard,
            "latched": None if latched is False and guard != "running" else latched,
            "age_ms": age, "max_age_ms": lifetime}


def _endpoint(url):
    # 两个上游地址均由部署者固定；浏览器提供的路径和请求体不能指定目的主机。
    origin = urlsplit(url)
    if (origin.scheme != "http" or not origin.hostname or origin.username is not None
            or origin.path not in ("", "/") or any(c in url for c in ("?", "#", "\\"))
            or any(c.isspace() or not c.isprintable() for c in url)
            or origin.netloc.endswith(":") or origin.port == 0):
        raise ValueError("PC2 地址应为 http://主机:端口，不含路径、查询参数或账号")
    return origin.hostname, origin.port or 80


class StationSource:
    def __init__(self, pc2_url, map_path=None, *, goal_map_sha256=None, manual_key=None,
                 manual_pc2_url=None):
        self._telemetry_endpoint = _endpoint(pc2_url)
        self._manual_endpoint = _endpoint(manual_pc2_url) if manual_pc2_url is not None else None
        if self._manual_endpoint is not None and (
                manual_key is None or not manual_link.loopback(self._manual_endpoint[0])
                or self._manual_endpoint[1] == self._telemetry_endpoint[1]):
            raise ValueError("独立遥控通道须使用密钥和单独的回环端口")
        # 机间密钥仅传入预先建立的本地 SSH 转发端口，不能通过未加密的局域网 HTTP 发出。
        if manual_key is not None and (not isinstance(manual_key, str) or not manual_link.loopback(self._telemetry_endpoint[0])
                                       or not manual_link.KEY_PATTERN.fullmatch(manual_key)
                                       or goal_map_sha256 is not None):
            raise ValueError("手动遥控需要私有密钥、回环 PC2 通道，且不能与单点导航同时启用")
        self._manual_key = manual_key
        if goal_map_sha256 is not None and (
                not isinstance(goal_map_sha256, str) or not _SHA256.fullmatch(goal_map_sha256)):
            raise ValueError("单点导航地图必须绑定到 64 位小写 SHA-256")
        self.goal_map_sha256 = goal_map_sha256
        self._grid = None
        self._map_metadata = None
        if map_path is not None:
            # 沿用离线规划器的三态栅格解析，含底行优先坐标约定；上位机不依赖 ROS。
            from a2_navigation.offline_planner import load_trinary_grid
            self._grid = grid = load_trinary_grid(Path(map_path))
            version = sha256((grid.metadata_sha256 + grid.image_sha256).encode("ascii")).hexdigest()
            self._map_metadata = {
                "id": version, "frame_id": "map", "width": grid.width, "height": grid.height,
                "resolution_m": grid.resolution, "origin_x_m": grid.origin[0],
                "origin_y_m": grid.origin[1], "origin_yaw_rad": grid.origin[2],
            }

    def _pc2_request(self, method, path, payload=None, *, control_status=False):
        # 请求路径由本类给出。只连部署时配置的 PC2，不使用环境代理、不跟随重定向。
        # HTTP 回执丢失时，运动目标可能已受理；此处不对 POST 作自动重发。
        body = None
        headers = {"Accept": "application/json", "Cache-Control": "no-store"}
        # 独立遥控部署中，状态鉴权和所有控制请求只能发给控制端口。原有只读隧道
        # 只接收不含密钥的遥测、地图和位姿请求。未指定第二地址的演练仍使用单通道。
        if path == PC2_STOP_RESET_PATH and self._manual_endpoint is None:
            raise ValueError("停车复位需要独立遥控通道")
        manual = path in manual_link.PC2_COMMANDS or path == PC2_STOP_RESET_PATH or (
            path == "/api/robot/state" and (control_status or self._manual_endpoint is None))
        endpoint = self._manual_endpoint if manual and self._manual_endpoint else self._telemetry_endpoint
        if self._manual_key is not None and manual:
            headers["Authorization"] = "Bearer " + self._manual_key
        if payload is not None:
            body = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"
        timeout = _STOP_RESET_TIMEOUT_SECONDS if path == PC2_STOP_RESET_PATH else _TIMEOUT_SECONDS
        connection = HTTPConnection(*endpoint, timeout=timeout)
        try:
            connection.request(method, path, body=body, headers=headers)
            response = connection.getresponse()
            data = response.read(_MAX_RESPONSE_BYTES + 1)
            if len(data) > _MAX_RESPONSE_BYTES:
                raise ValueError("PC2 状态超过大小限制")
            message = json.loads(data)
            if not isinstance(message, dict):
                raise ValueError("PC2 消息必须为对象")
            return response.status, message
        finally:
            connection.close()

    def _get(self, path):
        status, message = self._pc2_request("GET", path)
        if status != 200:
            raise ValueError("PC2 接口状态异常")
        return message

    def _forward_goal(self, method, path, payload=None):
        """校验目标回执确实来自预期地图和目标，再交给浏览器显示。"""
        if self.goal_map_sha256 is None:
            return 405, {"ok": False, "reason": "single_goal_disabled"}
        try:
            status, message = self._pc2_request(method, path, payload)
            if 400 <= status < 600 and message.get("ok") is False and isinstance(message.get("reason"), str):
                return status, {"ok": False, "reason": message["reason"]}
            if status not in (200, 202) or message.get("ok") is not True:
                raise ValueError("上游没有返回目标回执")
            goal = message["goal"]
            if (not isinstance(goal, dict) or goal["phase"] not in _GOAL_PHASES or goal["frame_id"] != "map"
                    or not (goal["reason"] is None or isinstance(goal["reason"], str))):
                raise ValueError("导航状态或坐标系无效")
            if goal["map_sha256"] != self.goal_map_sha256:
                if method == "GET":
                    return 409, {"ok": False, "reason": "map_mismatch"}
                raise ValueError("下发后收到另一张地图的回执")
            if goal["phase"] == "idle":
                if goal["goal_id"] is not None or any(goal[key] is not None for key in _TARGET):
                    raise ValueError("空闲状态中带有未识别的目标")
            elif not _valid_goal_id(goal["goal_id"]) or not _valid_target(goal):
                raise ValueError("上游目标位姿无效")
            distance = goal["distance_remaining_m"]
            if distance is not None and not (_finite(distance) and distance >= 0):
                raise ValueError("剩余距离无效")
            if payload is not None:
                if goal["goal_id"] != payload["goal_id"]:
                    raise ValueError("回执属于另一目标")
                if path == _GOAL_PATH and any(goal[key] != payload[key] for key in _TARGET):
                    raise ValueError("回执中的目标位姿不符")
            reply = {"ok": True, "goal": goal}
            if "readiness" in message:
                ready = message["readiness"]
                if (not isinstance(ready, dict) or type(ready.get("ready")) is not bool
                        or (ready["ready"] and ready.get("reason") is not None)
                        or (not ready["ready"] and not isinstance(ready.get("reason"), str))):
                    raise ValueError("PC2 导航就绪状态无效")
                reply["readiness"] = {"ready": ready["ready"], "reason": ready.get("reason")}
            return status, reply
        except (OSError, HTTPException):
            return 503, {"ok": False, "reason": "pc2_unavailable"}
        except (KeyError, ValueError, TypeError, OverflowError, RecursionError):
            return 502, {"ok": False, "reason": "pc2_invalid_response"}

    def goal_status(self):
        return self._forward_goal("GET", _GOAL_PATH)

    def submit_goal(self, payload):
        if (not isinstance(payload, dict) or set(payload) != {"goal_id", "frame_id", "map_sha256", *_TARGET}
                or payload["frame_id"] != "map" or not _valid_goal_id(payload["goal_id"]) or not _valid_target(payload)):
            return 400, {"ok": False, "reason": "invalid_goal"}
        if self.goal_map_sha256 is not None and payload["map_sha256"] != self.goal_map_sha256:
            return 409, {"ok": False, "reason": "map_mismatch"}
        return self._forward_goal("POST", _GOAL_PATH, payload)

    def cancel_goal(self, payload):
        if not isinstance(payload, dict) or set(payload) != {"goal_id"} or not _valid_goal_id(payload["goal_id"]):
            return 400, {"ok": False, "reason": "invalid_goal"}
        return self._forward_goal("POST", _GOAL_PATH + "/cancel", payload)

    def _manual(self, path, payload):
        if self._manual_key is None:
            return 405, {"ok": False, "reason": "manual_control_disabled"}
        operation = manual_link.PC2_COMMANDS[path]
        if not manual_link.valid_command(operation, payload):
            return 400, {"ok": False, "reason": "invalid_manual_command"}
        if operation not in manual_link.STOP_COMMANDS:
            # 直接访问站端接口时也核对当前 PC2，而不相信网页上一次看到的按钮状态。
            # 停车独立于遥测；即使状态读取失败，旧会话仍可以请求停车。
            _, state = self.state()
            control = state["control"]
            if not control["available"]:
                reason = state["pc2_reason"] or control["reason"]
                code = {"pc2_unavailable": 503, "manual_link_offline": 503, "pc2_invalid_response": 502,
                        "manual_invalid_response": 502, "manual_auth_failed": 403}.get(reason, 409)
                return code, {"ok": False, "reason": reason}
        return manual_link.forward_command(self._pc2_request, path, payload)

    def acquire(self, payload):
        return self._manual("/api/manual/session", payload)

    def velocity(self, payload):
        return self._manual("/api/manual/velocity", payload)

    def stop(self, payload):
        return self._manual("/api/manual/stop", payload)

    def action_start(self, payload):
        return self._manual("/api/manual/action/start", payload)

    def action_release(self, payload):
        return self._manual("/api/manual/action/release", payload)

    def stop_reset(self, payload):
        if not valid_stop_reset_confirmation(payload):
            return 400, {"ok": False, "reason": "stop_reset_confirmation_required"}
        if self._manual_endpoint is None:
            return 405, {"ok": False, "reason": "stop_reset_unavailable"}
        # 重新配对两路守护状态。发令门禁会因锁存关闭，不能拿它挡住复位。
        _, state = self.state()
        readiness = state["stop_reset"]
        if not readiness["available"]:
            return 409, {"ok": False, "reason": readiness["reason"]}
        try:
            status, reply = self._pc2_request("POST", PC2_STOP_RESET_PATH, payload)
            if status == 403 and reply.get("reason") == "manual_auth_failed":
                return 403, {"ok": False, "reason": "manual_auth_failed"}
            ok = reply.get("ok")
            if (reply.get("protocol") != manual_link.PROTOCOL or type(ok) is not bool
                    or not valid_stop_reset_reason(reply.get("reason"))
                    or (ok and status != 200) or (not ok and not 400 <= status < 600)):
                raise ValueError("PC2 复位回执无效")
            return status, {"ok": ok, "reason": reply["reason"]}
        except (OSError, HTTPException, ValueError, TypeError, OverflowError, RecursionError):
            # 请求可能已被机载端处理。转发层不重发，让操作员核对实测锁存状态。
            return 502, {"ok": False, "reason": "stop_reset_outcome_unknown"}

    def _read_control_report(self):
        """遥控隧道故障仅关闭控制。正式只读链路仍可独立展示电池、守护与地图。"""
        try:
            status, report = self._pc2_request("GET", "/api/robot/state", control_status=True)
            if status in (401, 403):
                return None, "manual_auth_failed"
            if status != 200:
                return None, "manual_invalid_response"
            return report, None
        except (OSError, HTTPException):
            return None, "manual_link_offline"
        except (ValueError, TypeError, OverflowError, RecursionError):
            return None, "manual_invalid_response"

    @staticmethod
    def _paired_control(report, elapsed_ms, robot, remote_stop):
        """只有来自同一机器人、两边均新鲜的遥测和守护才能开放正式控制。"""
        try:
            peer = _robot_fields(report, elapsed_ms)
        except (ValueError, TypeError, OverflowError):
            return manual_link.unavailable("manual_invalid_response")
        # 两个端口即使都返回有效报文也可能接到不同机器；未知标识不能相互配对。
        if not robot["id"] or not robot["id"].strip() or robot["id"] != peer["id"]:
            return manual_link.unavailable("manual_robot_mismatch")
        if not isinstance(report.get("control"), dict):
            # 普通只读网关也能报告锁存，却不支持遥控协议；不能把它当作已接好的控制通道。
            return manual_link.unavailable("manual_control_disabled")
        control = manual_link.control_state(report.get("control"))
        if control["source"] == "mock":
            return manual_link.unavailable("manual_receiver_source_mismatch")
        if control["reason"] in ("manual_auth_failed", "manual_receiver_source_mismatch"):
            return control
        guards = (remote_stop, _remote_stop(report.get("remote_stop"), elapsed_ms))
        for guard in guards:
            blocked = remote_stop_status.blocked_reason(guard)
            if blocked:
                return manual_link.unavailable(blocked, control["source"])
        if not robot["online"] or not peer["online"]:
            return manual_link.unavailable("robot_offline", control["source"])
        # PC2 发布者给速度反馈设置了有效期。经过 SSH 和第二次读状态后，
        # 即便机器人在线标志仍为真，已过期的运动反馈也不能用来发放控制。
        if control["available"] and (robot["velocity"] is None or peer["velocity"] is None):
            return manual_link.unavailable("feedback_stale", control["source"])
        return control

    @staticmethod
    def _paired_stop_reset(report, elapsed_ms, robot, remote_stop):
        try:
            peer = _robot_fields(report, elapsed_ms)
        except (ValueError, TypeError, OverflowError):
            return {"available": False, "reason": "manual_invalid_response"}
        if not robot["id"] or not robot["id"].strip() or robot["id"] != peer["id"]:
            return {"available": False, "reason": "manual_robot_mismatch"}
        control = report.get("control")
        if not isinstance(control, dict):
            return {"available": False, "reason": "stop_reset_unavailable"}
        if control.get("reason") == "manual_auth_failed":
            return {"available": False, "reason": "manual_auth_failed"}
        if control.get("source") == "mock" or control.get("reason") == "manual_receiver_source_mismatch":
            return {"available": False, "reason": "manual_receiver_source_mismatch"}
        if not robot["online"] or not peer["online"]:
            return {"available": False, "reason": "robot_offline"}
        for guard in (remote_stop, _remote_stop(report.get("remote_stop"), elapsed_ms)):
            reason = remote_stop_status.reset_blocked_reason(guard)
            if reason:
                return {"available": False, "reason": reason}
        if control.get("reason") != "stop_latched" or control.get("available") is not False:
            return {"available": False, "reason": "manual_invalid_response"}
        status = report.get("stop_reset")
        if (not isinstance(status, dict) or type(status.get("available")) is not bool
                or not valid_stop_reset_reason(status.get("reason"))):
            return {"available": False, "reason": "stop_reset_unavailable"}
        return {"available": status["available"], "reason": status["reason"]}

    def _read_telemetry(self):
        """获取正式遥测并保留 PC2 创建快照时的年龄，供两种站端状态响应共用。"""
        try:
            start = time.monotonic_ns()
            message = self._get("/api/robot/state")
            elapsed_ms = (time.monotonic_ns() - start) // 1_000_000
            robot = _robot_fields(message, elapsed_ms)
            remote_stop = _remote_stop(message.get("remote_stop"), elapsed_ms)
            return message, robot, remote_stop, None
        except (OSError, HTTPException):
            reason = "pc2_unavailable"
        except (ValueError, TypeError, OverflowError, RecursionError):
            reason = "pc2_invalid_response"
        # PC2 离线后不沿用上一帧。站端自身仍在线，地图和其他页面可继续访问。
        offline = {**dict.fromkeys(_ROBOT_FIELDS), **fresh_details({}), "online": False}
        return None, offline, _remote_stop(None, 0), reason

    def telemetry(self):
        """只读驾驶舱无需等控制隧道完成状态查询。"""
        if self._manual_key is not None and self._manual_endpoint is None:
            # 旧的单通道遥控演练共用网关，不能借这个入口发送遥控鉴权。
            return 404, {"ok": False, "reason": "not_found"}
        _, robot, remote_stop, reason = self._read_telemetry()
        return 200, {"deployment": "station", "pc2_connected": reason is None, "pc2_reason": reason,
                     "robot": robot, "remote_stop": remote_stop}

    def state(self):
        control = manual_link.unavailable("pc2_unavailable")
        stop_reset = {"available": False, "reason": "stop_reset_unavailable"}
        if self._manual_endpoint:
            # 配对状态继续先读控制、后读正式遥测，控制与复位请求仍用这条路径就地核对。
            control_start = time.monotonic_ns()
            control_report, control_error = self._read_control_report()
        message, robot, remote_stop, reason = self._read_telemetry()
        try:
            if message is not None and self._manual_endpoint:
                if control_error:
                    control = manual_link.unavailable(control_error)
                    stop_reset = {"available": False, "reason": control_error}
                else:
                    control_age_ms = (time.monotonic_ns() - control_start) // 1_000_000
                    control = self._paired_control(control_report, control_age_ms, robot, remote_stop)
                    stop_reset = self._paired_stop_reset(control_report, control_age_ms, robot, remote_stop)
            elif message is not None and self._manual_key is not None:
                control = manual_link.control_state(message.get("control"))
                if control["source"] == "mock":
                    # 演练使用模拟接收器的锁存；不将测试机上的服务状态冒充机器人遥测。
                    remote_stop = None
                elif (control["source"] == "unitree" or
                      (remote_stop["latched"] is True and control["reason"] != "manual_auth_failed")):
                    blocked = remote_stop_status.blocked_reason(remote_stop)
                    if blocked:
                        control = manual_link.unavailable(blocked, control["source"])
                if not robot["online"] and control["available"]:
                    control = manual_link.unavailable("robot_offline", control["source"])
        except (OSError, HTTPException):
            reason = "pc2_unavailable"
        except (ValueError, TypeError, OverflowError, RecursionError):
            reason = "pc2_invalid_response"
        if self._manual_key is None:
            # 未启用发令的正式站端始终只读，但锁存原因须优先告知操作员。
            control = {"available": False, "armed": False,
                       "reason": "stop_latched" if remote_stop["latched"] is True else "robot_readonly",
                       "limits": {"linear_mps": 0.0, "angular_radps": 0.0}}
        return 200, {
            "deployment": "station", "mode": "manual_control" if self._manual_key else "robot_readonly",
            "server_time_ms": time.time_ns() // 1_000_000,
            "pc2_connected": reason is None, "pc2_reason": reason, "robot": robot,
            "map": self._map_metadata, "pose": None,
            "remote_stop": remote_stop, "control": control,
            **({"independent_telemetry": True,
                "stop_reset": stop_reset if reason is None else {"available": False, "reason": reason}}
               if self._manual_endpoint else {}),
        }

    def map_data(self):
        if self._grid is None:
            return None
        # 内部栅格用无符号字节 255 存储未知格；HTTP 和 ROS 接口均以 -1 表示。
        from a2_navigation.offline_planner import UNKNOWN_CELL
        return {**self._map_metadata, "cells": [-1 if cell == UNKNOWN_CELL else cell for cell in self._grid.cells]}

    def pose(self, map_dir, versions):
        result = {"source": SOURCE, "status": "waiting", "sampleSha256": None,
                  "pose": None, "age_ms": None, "max_age_ms": MAX_AGE_MS}
        try:
            metadata = read_odin_object(Path(map_dir) / "metadata.json")
            sample = metadata["sampleSha256"]
            if not isinstance(sample, str) or not _SHA256.fullmatch(sample):
                raise ValueError("点云展示版本无效")
            result["sampleSha256"] = sample
            digest = metadata.get("relocMapSha256")
            # 浏览器提交的是展示点云版本；PC2 仅识别设备所加载的 BIN，两者通过受控清单绑定。
            if versions != [sample] or not isinstance(digest, str) or not _SHA256.fullmatch(digest):
                return 200, {**result, "status": "map_mismatch"}
        except FileNotFoundError:
            return 200, result
        except (OSError, KeyError, ValueError, TypeError, RecursionError):
            return 200, {**result, "status": "invalid"}

        try:
            start_ns = time.monotonic_ns()
            message = self._get("/api/odin1/pose?map_sha256=" + digest)
            if message.get("source") != SOURCE:
                raise ValueError("PC2 位姿来源无效")
            status = message.get("status")
            if status in ("waiting", "invalid", "stale", "map_mismatch"):
                return 200, {**result, "status": status}
            if status != "live":
                raise ValueError("未知的 PC2 位姿状态")
            if message.get("map_sha256") != digest:
                return 200, {**result, "status": "map_mismatch"}
            age, maximum = message["age_ms"], message["max_age_ms"]
            if type(age) is not int or age < 0 or type(maximum) is not int or not 0 < maximum <= MAX_AGE_MS:
                raise ValueError("位姿时效字段无效")
            result.update(age_ms=age, max_age_ms=maximum)
            # PC2 验证同机接收时间。上位机先排除传输时已失效的数据；浏览器收到后按完整请求
            # 耗时再判断剩余寿命，因此传给浏览器的 age_ms 保留 PC2 实测值，避免重复扣除延迟。
            if age + (time.monotonic_ns() - start_ns) / 1_000_000 >= maximum:
                return 200, {**result, "status": "stale"}
            pose = {key: message["pose"][key] for key in ("x_m", "y_m", "z_m", "yaw_rad")}
            if (not all(type(v) in (int, float) and math.isfinite(v) for v in pose.values())
                    or abs(pose["yaw_rad"]) > math.pi):
                raise ValueError("PC2 位姿坐标无效")
            return 200, {**result, "status": "live", "pose": pose}
        except (OSError, HTTPException):
            return 503, {**result, "status": "disconnected"}
        except (KeyError, ValueError, TypeError, OverflowError, RecursionError):
            return 200, {**result, "status": "invalid"}

    def close(self):
        """每个 HTTP 请求单独关闭，自身没有后台线程或持久连接。"""
