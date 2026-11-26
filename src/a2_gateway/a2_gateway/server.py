"""A2 巡检驾驶舱上位机服务、PC2 只读数据接口及独立模拟入口。"""

import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
from pathlib import Path
import re
import signal
import socket
import struct
from threading import Lock
import time
from urllib.parse import parse_qs, unquote, urlsplit

from . import manual_link
from .manual_link import (BROWSER_STOP_RESET_PATH, PC2_STOP_RESET_PATH, valid_stop_reset_confirmation,
                          valid_stop_reset_reason)
from .manual_link import loopback as _loopback
from .odin1_pose import odin1_pose_snapshot, pc2_pose_snapshot, read_odin_object
from .remote_stop_status import RemoteStopStatus, blocked_reason, reset_blocked_reason


DEFAULT_STATIC_ROOT = Path(__file__).resolve().parents[1] / "web" / "dist"
DEFAULT_ODIN1_MAP_DIR = Path(__file__).resolve().parents[3] / "maps" / "odin1" / "preview"
MAX_REQUEST_BYTES = 8_192
SIMULATOR_COMMANDS = {
    **{path: op for path, op in manual_link.BROWSER_COMMANDS.items() if op in ("acquire", "velocity", "stop")},
    "/api/patrol/point": "submit_patrol_point",
}
GOAL_PATH = "/api/navigation/goal"
_STOP_RESET_SOCKET_BYTES = 512
# systemd 按连接启动状态工作进程，预算还包括解释器启动；站端普通遥测超时为 0.8 秒。
_STOP_RESET_STATUS_TIMEOUT_S = 0.45
_STOP_RESET_TIMEOUT_S = 22.0
_STOP_RESET_STATUS_TTL_S = 1.0


def _authority(value):
    if not value or any(c.isspace() or c in "/\\?#@" for c in value):
        return None
    try:
        parsed = urlsplit("//" + value)
        port = parsed.port
        return parsed.hostname, 80 if port is None else port
    except ValueError:
        return None


def _same_origin(origin, authority):
    if origin is None:
        return True
    try:
        parsed = urlsplit(origin)
    except ValueError:
        return False
    return (
        origin == "http://" + parsed.netloc
        and authority is not None
        and _authority(parsed.netloc) == authority
    )


def _distinct_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key")
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(f"Invalid JSON constant {value}")


class LocalStopReset:
    """仅连接机载 root 复位服务，网关自身不调用 sudo 或 systemctl。"""

    def __init__(self, path):
        self._path = Path(path)
        if not self._path.is_absolute():
            raise ValueError("停车复位套接字必须使用绝对路径")
        self._status_lock = Lock()
        self._reset_lock = Lock()
        self._status_cache = None

    def _request(self, operation, timeout):
        # 只发送固定命令。在发命令前核对对端 UID，拒绝普通账号冒充复位服务。
        deadline = time.monotonic() + timeout
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(timeout)
            connection.connect(str(self._path))
            credentials = struct.Struct("3i")
            _, uid, _ = credentials.unpack(connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED,
                                                                  credentials.size))
            if uid != 0:
                raise PermissionError("停车复位套接字对端不是 root")
            connection.settimeout(max(0, deadline - time.monotonic()))
            request = {"op": operation, "schema_version": 1}
            connection.sendall(json.dumps(request, separators=(",", ":")).encode("ascii") + b"\n")
            record = bytearray()
            while b"\n" not in record:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("等待停车复位回执超时")
                connection.settimeout(remaining)
                chunk = connection.recv(_STOP_RESET_SOCKET_BYTES + 1 - len(record))
                record.extend(chunk)
                if not chunk or len(record) > _STOP_RESET_SOCKET_BYTES:
                    raise ValueError("停车复位回执长度无效")
            if not record.endswith(b"\n") or record.count(b"\n") != 1:
                raise ValueError("停车复位回执应为单行")
            reply = json.loads(record.decode("utf-8"), object_pairs_hook=_distinct_keys,
                               parse_constant=_reject_constant)
            if (not isinstance(reply, dict) or type(reply.get("ok")) is not bool
                    or not valid_stop_reset_reason(reply.get("reason"))
                    or (operation == "status" and reply["ok"] and type(reply.get("available")) is not bool)):
                raise ValueError("停车复位回执格式无效")
            return reply

    def status(self):
        # 多个页面并发轮询时共用短缓存。复位期间避免轮询占用机载服务。
        if self._reset_lock.locked():
            return {"available": False, "reason": "stop_reset_busy"}
        with self._status_lock:
            if self._status_cache and time.monotonic() - self._status_cache[0] < _STOP_RESET_STATUS_TTL_S:
                return self._status_cache[1]
            try:
                reply = self._request("status", _STOP_RESET_STATUS_TIMEOUT_S)
                report = {"available": reply["ok"] and reply.get("available") is True, "reason": reply["reason"]}
            except (OSError, ValueError, RecursionError):
                report = {"available": False, "reason": "stop_reset_service_unavailable"}
            self._status_cache = time.monotonic(), report
            return report

    def reset(self):
        if not self._reset_lock.acquire(blocking=False):
            return 409, {"ok": False, "reason": "stop_reset_busy"}
        try:
            reply = self._request("reset", _STOP_RESET_TIMEOUT_S)
            return (200 if reply["ok"] else 409), {"ok": reply["ok"], "reason": reply["reason"]}
        except (FileNotFoundError, ConnectionRefusedError, PermissionError):
            return 503, {"ok": False, "reason": "stop_reset_service_unavailable"}
        except (OSError, ValueError, RecursionError):
            # 发送后发生通信错误时可能已经复位，不自动再发请求。
            return 502, {"ok": False, "reason": "stop_reset_outcome_unknown"}
        finally:
            with self._status_lock:
                self._status_cache = None
            self._reset_lock.release()


def _read_odin1_cloud(directory):
    """读取已抽样的现场点云，并校验来源清单与坐标、颜色数据属于同一版本。"""
    # 固定读取两份展示文件，避免将原始 PCD、设备重定位地图或预览目录开放为静态目录。
    manifest = json.loads((directory / "metadata.json").read_text(encoding="utf-8"),
                          parse_constant=_reject_constant)
    source = manifest["sourceName"]
    count, total = manifest["pointCount"], manifest["sourcePoints"]
    if (not isinstance(source, str) or "/" in source or "\\" in source or not source.lower().endswith(".pcd")
            or type(count) is not int or type(total) is not int or not 0 < count <= total
            or manifest["encoding"] != "xyz-f32le-rgba-u8" or manifest["strideBytes"] != 16):
        raise ValueError("无效的点云来源、点数或编码")
    bounds = manifest["viewBounds"]
    low, high = bounds["min"], bounds["max"]
    if (not isinstance(low, list) or not isinstance(high, list) or len(low) != 3 or len(high) != 3
            or any(type(n) not in (int, float) or not math.isfinite(n) for n in (*low, *high))
            or any(a > b for a, b in zip(low, high))):
        raise ValueError("无效的点云视图边界")
    with (directory / "points.bin").open("rb") as stream:
        # 配置错误时先从长度识别原始大地图，避免意外将它整体读入网关内存。
        if stream.seek(0, 2) != count * 16:
            raise ValueError("点云文件长度与声明点数不一致")
        stream.seek(0)
        points = stream.read(count * 16)
    if len(points) != count * 16 or hashlib.sha256(points).hexdigest() != manifest["sampleSha256"]:
        raise ValueError("点云内容与来源清单不一致")
    fields = ("sourceName", "sourcePoints", "pointCount", "strideBytes", "encoding", "sampleSha256")
    metadata = {name: manifest[name] for name in fields}
    metadata["viewBounds"] = {"min": low, "max": high}
    return metadata, points


class GatewayHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(5.0)

    def log_message(self, *_args):
        # 轮询客户端会频繁查询状态。特意省略控制台日志中的
        # 请求 URL 和请求体。
        pass

    def _send(self, status, body, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        if self.command != "HEAD":
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

    def _json(self, status, value):
        encoded = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
        self._send(status, encoded, "application/json; charset=utf-8")

    def _error(self, status, reason):
        self._json(status, {"ok": False, "reason": reason})

    def _host(self):
        fields = self.headers.get_all("Host", [])
        authority = _authority(fields[0]) if len(fields) == 1 else None
        if authority is None or not authority[0]:
            self._error(400, "invalid_host")
            return None
        if self.server.loopback_only and not _loopback(authority[0]):
            self._error(403, "host_not_allowed")
            return None
        return authority

    def _request_path(self):
        # 仅接受标准的 origin-form 请求路径。
        try:
            parsed = urlsplit(self.path)
        except ValueError:
            return None
        if parsed.scheme or parsed.netloc or not parsed.path.startswith("/"):
            return None
        return parsed.path

    def do_GET(self):
        if self._host() is None:
            return
        path = self._request_path()
        if path == GOAL_PATH and self.server.single_goal is not None:
            self._json(*self.server.single_goal.goal_status())
            return
        if self.server.role == "pc2":
            self._pc2_get(path)
            return
        if path == "/api/telemetry" and self.server.role == "station":
            self._json(*self.server.telemetry.telemetry())
            return
        if path == "/api/state":
            if self.server.role == "station":
                status, data = self.server.telemetry.state()
                if self.server.single_goal is not None:
                    data = {**data, "single_goal_enabled": True}
                self._json(status, data)
                return
            snapshot = self.server.telemetry.snapshot()
            control = snapshot.get("control")
            if control is None:
                control = self.server.control.state() if self.server.control is not None else {
                    "available": False,
                    "armed": False,
                    "reason": "robot_readonly",
                    "limits": {"linear_mps": 0.0, "angular_radps": 0.0},
                }
            self._json(200, {
                **snapshot,
                "mode": self.server.mode,
                "deployment": self.server.role,
                "server_time_ms": time.time_ns() // 1_000_000,
                "control": control,
            })
        elif path == "/api/patrol" and self.server.role == "simulator":
            # 任务由模拟机身独立推进。重开网页时取回同一任务的进度和事件。
            self._json(200, self.server.telemetry.patrol_state())
        elif path == "/api/map":
            grid = self.server.telemetry.map_data()
            if grid is None:
                self._error(404, "map_unavailable")
            else:
                self._json(200, grid)
        elif path in ("/api/maps/odin1/metadata", "/api/maps/odin1/points"):
            self._odin1_map(path.endswith("/points"))
        elif path == "/api/maps/odin1/pose":
            versions = parse_qs(urlsplit(self.path).query, keep_blank_values=True).get("version")
            if self.server.role == "station":
                self._json(*self.server.telemetry.pose(self.server.odin1_map_dir, versions))
            else:
                # 模拟端可显示本地实测底图；模拟机身位姿不能充当 ODIN1 观测。
                self._json(200, odin1_pose_snapshot(self.server.odin1_map_dir, None, versions))
        elif path is None:
            self._error(400, "invalid_path")
        elif path == "/api" or path.startswith("/api/"):
            self._error(404, "not_found")
        else:
            self._static(path)

    do_HEAD = do_GET

    def _pc2_manual_state(self, report):
        """状态查询和非停车指令共用机载门禁。先查独立守护，再核对接收器来源。"""
        guarded = self.server.remote_stop is not None
        if guarded:
            reason = blocked_reason(report)
            if reason:
                # A 键锁存会清退受管接收器。此时即使套接字不存在，也能返回真正的停车原因。
                return manual_link.unavailable(reason)
        control = manual_link.control_state(self.server.manual_control.state())
        source = control["source"]
        if guarded and source == "mock":
            # 实机和演练的接收器使用同一套机身指令协议；实机入口不得接管演练的会话。
            return manual_link.unavailable("manual_receiver_source_mismatch")
        if not guarded and source == "unitree":
            return manual_link.unavailable("stop_guard_not_ready", source)
        return control

    def _pc2_get(self, path):
        """保留原只读遥测格式，仅为显式配置且认证通过的站端补充控制能力。"""
        if path == "/api/robot/state":
            data = {"deployment": "pc2", "mode": "robot_readonly",
                    "robot": self.server.telemetry.snapshot()["robot"]}
            if self.server.remote_stop is not None:
                # 现场安全状态属于只读遥测，正式站端无需机间遥控密钥也能显示锁存。
                data["remote_stop"] = self.server.remote_stop.snapshot()
            if self.server.manual_control is not None and self.headers.get("Authorization"):
                if manual_link.authenticated(self.headers, self.server.manual_key):
                    data["control"] = self._pc2_manual_state(data.get("remote_stop"))
                    if self.server.stop_reset is not None:
                        reason = self._pc2_stop_reset_blocked(data["robot"], data.get("remote_stop"))
                        data["stop_reset"] = ({"available": False, "reason": reason} if reason
                                              else self.server.stop_reset.status())
                else:
                    data["control"] = manual_link.unavailable("manual_auth_failed")
            self._json(200, data)
        elif path == "/api/odin1/pose":
            values = parse_qs(urlsplit(self.path).query, keep_blank_values=True).get("map_sha256", [])
            if len(values) != 1 or not re.fullmatch("[0-9a-f]{64}", values[0]):
                self._error(400, "invalid_map_sha256")
                return
            self._json(200, pc2_pose_snapshot(self.server.odin1_pose_file, values[0]))
        else:
            self._error(404, "not_found")

    def _odin1_map(self, send_points):
        """文件缺失与文件损坏分别反馈，前端可重新加载来源清单后重试。"""
        try:
            metadata, points = _read_odin1_cloud(self.server.odin1_map_dir)
        except FileNotFoundError:
            self._error(404, "odin1_map_unavailable")
            return
        except (OSError, ValueError, KeyError, TypeError, OverflowError, RecursionError):
            self._error(503, "odin1_map_invalid")
            return
        if send_points:
            # 可选版本来自刚刚获取的元数据，防止两次请求之间替换地图后错配来源或范围。
            versions = parse_qs(urlsplit(self.path).query, keep_blank_values=True).get("version")
            if versions is not None and versions != [metadata["sampleSha256"]]:
                self._error(409, "odin1_map_changed")
                return
            self._send(200, points, "application/octet-stream")
        else:
            self._json(200, metadata)

    def _static(self, path):
        root = self.server.static_root
        if root is None:
            self._error(404, "console_not_built")
            return
        try:
            relative = unquote(path, errors="strict").lstrip("/") or "index.html"
            candidate = (root / relative).resolve()
            candidate.relative_to(root)
            if not candidate.is_file():
                self._error(404, "not_found")
                return
            body = candidate.read_bytes()
        except (ValueError, OSError, RuntimeError):
            self._error(404, "not_found")
            return
        content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
        self._send(200, body, content_type)

    def _read_json(self, maximum=MAX_REQUEST_BYTES):
        if self.headers.get("Transfer-Encoding") is not None:
            self._error(400, "unsupported_transfer_encoding")
            return None
        if self.headers.get_content_type() != "application/json":
            self._error(415, "expected_json")
            return None
        lengths = self.headers.get_all("Content-Length", [])
        raw_length = lengths[0] if len(lengths) == 1 else ""
        if not raw_length.isascii() or not raw_length.isdecimal() or len(raw_length) > 12:
            self._error(400, "invalid_content_length")
            return None
        length = int(raw_length)
        if length > maximum:
            self._error(413, "request_too_large")
            return None
        try:
            body = self.rfile.read(length)
            if len(body) != length:
                raise ValueError("Incomplete JSON body")
            value = json.loads(body.decode("utf-8"), object_pairs_hook=_distinct_keys, parse_constant=_reject_constant)
            if not isinstance(value, dict):
                raise ValueError("JSON object required")
        except (ValueError, RecursionError, TimeoutError):
            self._error(400, "invalid_json")
            return None
        return value

    @staticmethod
    def _pc2_stop_reset_blocked(robot, report):
        if robot.get("online") is not True:
            return "robot_offline"
        return reset_blocked_reason(report)

    def _stop_reset(self, payload):
        if not valid_stop_reset_confirmation(payload):
            self._error(400, "stop_reset_confirmation_required")
            return
        if self.server.role == "station":
            self._json(*self.server.manual_control.stop_reset(payload))
            return
        # POST 即时重读本机遥测。root 服务再校验实时状态，不相信浏览器或 HTTP 网关的放行。
        robot = self.server.telemetry.snapshot()["robot"]
        reason = self._pc2_stop_reset_blocked(robot, self.server.remote_stop.snapshot())
        if reason:
            status, reply = 409, {"ok": False, "reason": reason}
        else:
            status, reply = self.server.stop_reset.reset()
        self._json(status, {**reply, "protocol": manual_link.PROTOCOL})

    def do_POST(self):
        authority = self._host()
        if authority is None:
            return
        path = self._request_path()
        goal_port = self.server.single_goal
        goal_command = goal_port is not None and path in (GOAL_PATH, GOAL_PATH + "/cancel")
        commands = manual_link.PC2_COMMANDS if self.server.role == "pc2" else manual_link.BROWSER_COMMANDS
        manual = self.server.manual_control is not None and path in commands
        stop_reset = ((self.server.role == "station" and self.server.manual_control is not None
                       and path == BROWSER_STOP_RESET_PATH) or (self.server.role == "pc2"
                       and self.server.stop_reset is not None and path == PC2_STOP_RESET_PATH))
        if self.server.role != "simulator" and not (goal_command or manual or stop_reset):
            self._error(405, "readonly")
            return
        origins = self.headers.get_all("Origin", [])
        if len(origins) > 1 or (origins and not _same_origin(origins[0], authority)):
            self._error(403, "cross_origin_request")
            return
        if (manual or stop_reset) and self.server.role == "station" and not origins:
            self._error(403, "origin_required")
            return
        if ((manual or stop_reset) and self.server.role == "pc2"
                and not manual_link.authenticated(self.headers, self.server.manual_key)):
            self._error(403, "manual_auth_failed")
            return
        operation = commands[path] if manual else SIMULATOR_COMMANDS.get(path)
        if operation is None and not (goal_command or stop_reset):
            self._error(404, "not_found")
            return
        payload = self._read_json(_STOP_RESET_SOCKET_BYTES if stop_reset else MAX_REQUEST_BYTES)
        if payload is None:
            return
        if stop_reset:
            self._stop_reset(payload)
            return
        if goal_command:
            # 单点接口与机身遥控使用不同的授权和状态，不共用速度通道。
            handler = goal_port.submit_goal if path == GOAL_PATH else goal_port.cancel_goal
            self._json(*handler(payload))
            return
        if manual:
            if not manual_link.valid_command(operation, payload):
                self._error(400, "invalid_manual_command")
                return
            if self.server.role == "pc2" and operation not in manual_link.STOP_COMMANDS:
                # 页面之外的调用也须经机载门禁；停止请求始终可达。只有独立 mock 可无真实守护。
                observer = self.server.remote_stop
                control = self._pc2_manual_state(observer.snapshot() if observer is not None else None)
                if not control["available"]:
                    self._error(409, control["reason"])
                    return
                if self.server.telemetry.snapshot()["robot"].get("online") is not True:
                    self._error(409, "robot_offline")
                    return
            status, reply = getattr(self.server.manual_control, operation)(payload)
            if self.server.role == "pc2":
                reply = {**reply, "protocol": manual_link.PROTOCOL}
            self._json(status, reply)
            return
        if self.server.control is None:
            self._error(403, "robot_readonly")
            return
        status, response = getattr(self.server.control, operation)(payload)
        self._json(status, response)

    def do_OPTIONS(self):
        self._error(405, "method_not_allowed")


class GatewayServer(ThreadingHTTPServer):
    """HTTP listener with injectable telemetry and control for local integration."""

    def __init__(self, address, telemetry, *, mode, role=None, control=None, static_root=None,
                 odin1_map_dir=DEFAULT_ODIN1_MAP_DIR, odin1_pose_file=None, single_goal=None,
                 manual_control=None, manual_key=None, remote_stop=None, stop_reset_socket=None,
                 platform_watchdog=None):
        # 旧的模拟单测直接注入 Simulator；实机和上位机必须走各自的只读分派。
        self.role = role or ("simulator" if mode == "simulator" else "pc2")
        if self.role not in ("station", "pc2", "simulator"):
            raise ValueError("未知部署角色")
        self.loopback_only = _loopback(address[0])
        if manual_control is not None:
            if (not self.loopback_only or self.role == "simulator" or single_goal is not None or
                    not isinstance(manual_key, str) or not manual_link.KEY_PATTERN.fullmatch(manual_key)):
                raise ValueError("手动遥控仅在持有机间密钥的回环站端与 PC2 启用，并与导航互斥")
        elif manual_key is not None:
            raise ValueError("遥控密钥需要配套的接收或转发端")
        real_mode = "manual_control" if manual_control is not None else "robot_readonly"
        if self.role != "simulator" and (control is not None or mode != real_mode):
            raise ValueError("上位机或 PC2 的手动遥控模式与端口配置不符")
        if self.role == "simulator" and mode != "simulator":
            raise ValueError("模拟入口只接受模拟数据")
        if single_goal is not None and (self.role == "simulator" or not self.loopback_only):
            raise ValueError("单点导航使用上位机与 PC2 的回环监听，通过已有维护通道互连")
        if platform_watchdog is not None and (
                self.role != "pc2" or not self.loopback_only or manual_control is not None or single_goal is not None):
            raise ValueError("遥测看门狗仅供独立的 PC2 回环只读网关")
        self.single_goal = single_goal
        if remote_stop is not None and self.role != "pc2":
            raise ValueError("停车守护采集器仅属于 PC2 本机")
        self.remote_stop = remote_stop
        if stop_reset_socket is not None and (self.role != "pc2" or manual_control is None or remote_stop is None):
            raise ValueError("停车复位套接字仅适用于附带守护观测的 PC2 独立手动遥控网关")
        self.stop_reset = LocalStopReset(stop_reset_socket) if stop_reset_socket is not None else None
        self.manual_control, self.manual_key = manual_control, manual_key
        if ":" in address[0]:
            self.address_family = socket.AF_INET6
        self.telemetry = telemetry
        self.platform_watchdog = platform_watchdog
        self.control = control
        self.mode = mode
        self.static_root = Path(static_root).resolve() if static_root is not None else None
        self.odin1_map_dir = Path(odin1_map_dir).resolve() if self.role != "pc2" else None
        self.odin1_pose_file = Path(odin1_pose_file) if odin1_pose_file is not None else None
        # TCPServer 在绑定失败时也会调用 server_close，此时资源仍由构造调用方持有。
        self._owns_sources = False
        super().__init__(address, GatewayHandler)
        self._owns_sources = True
        if self.remote_stop is not None:
            self.remote_stop.start()

    def service_actions(self):
        # 由 HTTP 主循环检查 ROS 入站进度。任何一端卡住时，都不会替服务续期。
        if self.platform_watchdog is not None:
            self.platform_watchdog.tick(self.telemetry)

    def server_close(self):
        try:
            super().server_close()
        finally:
            # 先让 PC2 导航适配器结束在途目标和 ROS 回调，再释放它所观察的定位状态。
            if self._owns_sources:
                self._owns_sources = False
                try:
                    if self.remote_stop is not None:
                        self.remote_stop.close()
                    if self.single_goal is not None and self.single_goal is not self.telemetry:
                        self.single_goal.close()
                finally:
                    self.telemetry.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("station", "pc2", "simulator"),
                        help="部署角色：上位机（默认）、PC2 只读数据服务或独立模拟器")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址，默认只接受本机连接")
    parser.add_argument("--port", type=int, default=8765, help="HTTP 端口，默认 8765")
    parser.add_argument("--static-root", type=Path, help="上位机 Vue 构建目录")
    parser.add_argument("--odin1-map-dir", type=Path, help="上位机本地 ODIN1 点云预览目录")
    parser.add_argument("--map-yaml", type=Path, help="上位机本地二维栅格 YAML（可选）")
    parser.add_argument("--pc2-url", help="上位机读取的 PC2 服务地址，默认 http://127.0.0.1:8766")
    parser.add_argument("--manual-pc2-url", help="上位机通过独立回环维护通道访问 PC2 真实遥控网关的地址")
    parser.add_argument("--odin1-pose-file", type=Path, help="PC2 本机观测器写入的位姿 JSON")
    parser.add_argument("--watch-platform-stream", action="store_true",
                        help="为 PC2 独立只读采集服务启用 systemd 数据看门狗")
    parser.add_argument("--enable-single-goal", action="store_true", help="在本站显式启用单点导航接口")
    parser.add_argument("--enable-manual-control", action="store_true", help="在本站显式启用低速机身遥控")
    parser.add_argument("--manual-key-file", type=Path, help="上位机与 PC2 遥控接口使用的私有密钥文件")
    parser.add_argument("--manual-socket", type=Path, help="PC2 上独立受管遥控接收器的 UNIX 套接字")
    parser.add_argument("--stop-reset-socket", type=Path, help="PC2 上 root 停车锁存复位服务的 UNIX 套接字")
    parser.add_argument("--goal-map-bin", type=Path, help="PC2 实际用于 ODIN1 重定位的 BIN")
    parser.add_argument("--goal-grid-yaml", type=Path, help="PC2 与该 BIN 坐标对齐的 Nav2 三态导航栅格 YAML")
    parser.add_argument("--goal-action", help="PC2 经现场确认的 Nav2 NavigateToPose 动作名称")
    parser.add_argument("--goal-journal", type=Path, help="PC2 持久保存目标收据的 SQLite 文件")
    parser.add_argument("--ros", action="store_true", help="兼容旧入口，等价于选择 --role pc2")
    from .ros_telemetry import PlatformStreamWatchdog, add_ros_arguments

    add_ros_arguments(parser)
    args = parser.parse_args(argv)
    role = args.role or ("pc2" if args.ros else "station")
    if not 0 <= args.port <= 65535:
        parser.error("--port 必须在 0 至 65535 之间")
    if args.ros and role != "pc2":
        parser.error("--ros 仅适用于 PC2")
    if role != "station" and (args.pc2_url is not None or args.map_yaml is not None):
        parser.error("--pc2-url 和 --map-yaml 属于上位机")
    if args.manual_pc2_url is not None and not (role == "station" and args.enable_manual_control):
        parser.error("--manual-pc2-url 只适用于启用手动遥控的上位机")
    if args.stop_reset_socket is not None and not (
            role == "pc2" and args.enable_manual_control and args.stop_reset_socket.is_absolute()):
        parser.error("--stop-reset-socket 仅适用于 PC2 独立手动遥控网关，且须使用绝对路径")
    if role != "pc2" and args.odin1_pose_file is not None:
        parser.error("--odin1-pose-file 属于 PC2 本机")
    if role == "pc2" and (args.static_root is not None or args.odin1_map_dir is not None):
        parser.error("请在上位机配置网页和展示地图目录")
    if args.enable_single_goal and (role == "simulator" or not _loopback(args.host)):
        parser.error("单点导航仅支持 station 或 pc2 的回环监听")
    platform_watchdog = None
    if args.watch_platform_stream:
        if (role != "pc2" or not _loopback(args.host) or args.enable_manual_control
                or args.enable_single_goal or args.use_sim_time):
            parser.error("遥测看门狗仅供使用系统时钟的 PC2 独立回环只读网关")
        try:
            platform_watchdog = PlatformStreamWatchdog()
        except ValueError as error:
            parser.error(str(error))
    manual_key = None
    if args.enable_manual_control:
        if role == "simulator" or not _loopback(args.host) or args.enable_single_goal:
            parser.error("手动遥控仅支持回环 station/pc2，且与单点导航互斥")
        if (args.manual_key_file is None or (role == "pc2") != (args.manual_socket is not None)
                or (args.manual_socket is not None and not args.manual_socket.is_absolute())):
            parser.error("两端需要 --manual-key-file，仅 PC2 需要绝对路径 --manual-socket")
        try:
            manual_key = manual_link.read_key(args.manual_key_file)
        except (OSError, ValueError) as error:
            parser.error(str(error))
    elif args.manual_key_file is not None or args.manual_socket is not None:
        parser.error("遥控配置需要显式 --enable-manual-control")
    if any((args.goal_grid_yaml, args.goal_map_bin, args.goal_action, args.goal_journal)) and not (
            role == "pc2" and args.enable_single_goal):
        parser.error("机载导航参数需与 --role pc2 --enable-single-goal 同时使用")
    if role == "pc2" and args.enable_single_goal:
        if not all((args.odin1_pose_file, args.goal_map_bin, args.goal_grid_yaml, args.goal_action)):
            parser.error("PC2 单点导航需要 --odin1-pose-file、--goal-map-bin、--goal-grid-yaml 和 --goal-action")
        if args.map_frame != "map" or args.base_frame in ("imu", "odom", "map"):
            parser.error("目标使用 map 坐标系，请指定已标定的机身参考系")

    # PC2 即使在完整源码树中启动也不查找网页。站端独立启动，无需 ROS 环境或在线的机器人。
    static_root = None
    control = None
    single_goal = None
    manual_control = None
    map_dir = args.odin1_map_dir or DEFAULT_ODIN1_MAP_DIR
    mode = "manual_control" if args.enable_manual_control else "robot_readonly"
    if role != "pc2":
        static_root = args.static_root or DEFAULT_STATIC_ROOT
        if not (static_root / "index.html").is_file():
            parser.error("未找到上位机页面，请先构建 Vue 前端或通过 --static-root 指定构建目录")
    if role == "pc2":
        from .ros_telemetry import RosTelemetry

        if args.enable_manual_control:
            from .motion import LocalMotionClient
            # HTTP 网关通过独立套接字接入受管进程。它自身不加载发令 SDK，也不运行系统管理命令。
            try:
                manual_control = LocalMotionClient(args.manual_socket, peer_user="a2-autonomy", require_source=True)
            except ValueError as error:
                parser.error(str(error))
        telemetry = RosTelemetry(args, platform_only=not args.enable_single_goal)
        if args.enable_single_goal:
            backend = None
            try:
                from a2_navigation.nav2_goal_client import Nav2GoalBackend
                from .pc2_goal import Pc2GoalPort, bin_digest
                # 收据使用稳定的机载路径，进程重启后仍能识别同一请求和未决目标。
                journal = args.goal_journal or Path.home() / ".local/state/a2_navigation/single-goal.sqlite"
                journal.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                digest = bin_digest(args.goal_map_bin)
                backend = Nav2GoalBackend(action_name=args.goal_action, use_sim_time=args.use_sim_time)
                single_goal = Pc2GoalPort(telemetry, args.odin1_pose_file, digest, args.goal_grid_yaml,
                                          backend, journal)
            except BaseException:
                if backend is not None:
                    backend.close()
                telemetry.close()
                raise
    elif role == "station":
        from .station import StationSource

        try:
            # 上位机以点云清单中配对的 BIN 摘要绑定目标，不接受浏览器指定导航地图来源。
            digest = None
            if args.enable_single_goal:
                digest = read_odin_object(map_dir / "metadata.json").get("relocMapSha256")
                if not isinstance(digest, str) or not re.fullmatch("[0-9a-f]{64}", digest):
                    raise ValueError("上位机地图清单缺少有效的 relocMapSha256")
            telemetry = StationSource(args.pc2_url or "http://127.0.0.1:8766", args.map_yaml,
                                      goal_map_sha256=digest, manual_key=manual_key,
                                      manual_pc2_url=args.manual_pc2_url)
            if args.enable_manual_control:
                manual_control = telemetry
            if args.enable_single_goal:
                single_goal = telemetry
        except (OSError, ValueError, TypeError) as error:
            parser.error(str(error))
    else:
        from .simulator import Simulator

        mode = "simulator"
        telemetry = control = Simulator()

    try:
        server = GatewayServer((args.host, args.port), telemetry, role=role, mode=mode, control=control,
                               static_root=static_root, odin1_map_dir=map_dir, single_goal=single_goal,
                               odin1_pose_file=args.odin1_pose_file, manual_control=manual_control,
                               manual_key=manual_key, remote_stop=RemoteStopStatus() if role == "pc2" else None,
                               stop_reset_socket=args.stop_reset_socket, platform_watchdog=platform_watchdog)
    except BaseException:
        if single_goal is not None and single_goal is not telemetry:
            single_goal.close()
        telemetry.close()
        raise
    with server:
        url_host = f"[{args.host}]" if ":" in args.host else args.host
        print(f"A2 {role} ({mode}) http://{url_host}:{server.server_port}", flush=True)
        old_sigterm = None
        if role == "pc2" and single_goal is not None:
            # 正常停止机载服务时走与 Ctrl+C 相同的收尾，让动作客户端能够撤销在途目标。
            old_sigterm = signal.signal(signal.SIGTERM, signal.default_int_handler)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            if old_sigterm is not None:
                signal.signal(signal.SIGTERM, old_sigterm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
