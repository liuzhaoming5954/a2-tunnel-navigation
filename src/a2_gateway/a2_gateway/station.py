"""上位机保管地图，从固定 PC2 获取遥测，启用单点导航时转发目标和取消请求。"""

from hashlib import sha256
from http.client import HTTPConnection, HTTPException
import json
import math
from pathlib import Path
import re
import time
from urllib.parse import urlsplit

from .odin1_pose import MAX_AGE_MS, SOURCE, read_odin_object
from .platform_details import fresh_details


_MAX_RESPONSE_BYTES = 65_536
_TIMEOUT_SECONDS = 0.8
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
    # IMU 和原厂运动状态来自不同话题。各路按各自的观测时间决定是否转发。
    # 快照原始年龄交给浏览器，浏览器按整次请求耗时计算显示有效期。
    return {**fields, **fresh_details(robot, transfer_ms=elapsed_ms), "online": robot["online"]}


class StationSource:
    def __init__(self, pc2_url, map_path=None, *, goal_map_sha256=None):
        # 上游只接受部署时配置的固定源；不从浏览器请求中解析目标地址。
        origin = urlsplit(pc2_url)
        if (origin.scheme != "http" or not origin.hostname or origin.username is not None
                or origin.path not in ("", "/") or any(c in pc2_url for c in ("?", "#", "\\"))
                or any(c.isspace() or not c.isprintable() for c in pc2_url)
                or origin.netloc.endswith(":") or origin.port == 0):
            raise ValueError("PC2 地址应为 http://主机:端口，不含路径、查询参数或账号")
        self._host, self._port = origin.hostname, origin.port or 80
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

    def _pc2_request(self, method, path, payload=None):
        # 请求路径由本类给出。只连部署时配置的 PC2，不使用环境代理、不跟随重定向。
        # HTTP 回执丢失时，运动目标可能已受理；此处不对 POST 作自动重发。
        body = None
        headers = {"Accept": "application/json", "Cache-Control": "no-store"}
        if payload is not None:
            body = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"
        connection = HTTPConnection(self._host, self._port, timeout=_TIMEOUT_SECONDS)
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

    def state(self):
        # PC2 离线与上位机进程离线是两种状态：本地地图与其他页面仍可读取。
        robot = {**dict.fromkeys(_ROBOT_FIELDS), **fresh_details({}), "online": False}
        reason = None
        try:
            started_ns = time.monotonic_ns()
            message = self._get("/api/robot/state")
            elapsed_ms = (time.monotonic_ns() - started_ns) // 1_000_000
            robot = _robot_fields(message, elapsed_ms)
        except (OSError, HTTPException):
            reason = "pc2_unavailable"
        except (ValueError, TypeError, OverflowError, RecursionError):
            reason = "pc2_invalid_response"
        return 200, {
            "deployment": "station", "mode": "robot_readonly", "server_time_ms": time.time_ns() // 1_000_000,
            "pc2_connected": reason is None, "pc2_reason": reason, "robot": robot,
            "map": self._map_metadata, "pose": None,
            "control": {"available": False, "armed": False, "reason": "robot_readonly",
                        "limits": {"linear_mps": 0.0, "angular_radps": 0.0}},
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
