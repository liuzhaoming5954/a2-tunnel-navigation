"""Serve the local A2 console and its simulator or ROS API."""

import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
import mimetypes
from pathlib import Path
import socket
import time
from urllib.parse import parse_qs, unquote, urlsplit

from .odin1_pose import odin1_pose_snapshot
from .simulator import Simulator


DEFAULT_STATIC_ROOT = Path(__file__).resolve().parents[1] / "web" / "dist"
DEFAULT_ODIN1_MAP_DIR = Path(__file__).resolve().parents[3] / "maps" / "odin1" / "preview"
MAX_REQUEST_BYTES = 8_192
CONTROL_PATHS = {
    "/api/control/session": "acquire",
    "/api/control/velocity": "velocity",
    "/api/control/stop": "stop",
}


def _loopback(host):
    if host.lower().rstrip(".") == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


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
        if path == "/api/state":
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
                "server_time_ms": time.time_ns() // 1_000_000,
                "control": control,
            })
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
            # 此通道专门接入 PC2 定位观测器，与本体遥测、模拟位姿及运动控制独立。
            self._json(200, odin1_pose_snapshot(self.server.odin1_map_dir, self.server.odin1_pose_file, versions))
        elif path is None:
            self._error(400, "invalid_path")
        elif path == "/api" or path.startswith("/api/"):
            self._error(404, "not_found")
        else:
            self._static(path)

    do_HEAD = do_GET

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

    def _read_json(self):
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
        if length > MAX_REQUEST_BYTES:
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

    def do_POST(self):
        authority = self._host()
        if authority is None:
            return
        origins = self.headers.get_all("Origin", [])
        if len(origins) > 1 or (origins and not _same_origin(origins[0], authority)):
            self._error(403, "cross_origin_request")
            return
        operation = CONTROL_PATHS.get(self._request_path())
        if operation is None:
            self._error(404, "not_found")
            return
        payload = self._read_json()
        if payload is None:
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

    def __init__(self, address, telemetry, *, mode, control=None, static_root=None,
                 odin1_map_dir=DEFAULT_ODIN1_MAP_DIR, odin1_pose_file=None):
        self.loopback_only = _loopback(address[0])
        if mode == "robot" and not self.loopback_only:
            raise ValueError("A gateway connected to a motion arbiter must bind to loopback")
        if ":" in address[0]:
            self.address_family = socket.AF_INET6
        self.telemetry = telemetry
        self.control = control
        self.mode = mode
        self.static_root = Path(static_root).resolve() if static_root is not None else None
        self.odin1_map_dir = Path(odin1_map_dir).resolve()
        self.odin1_pose_file = Path(odin1_pose_file) if odin1_pose_file is not None else None
        super().__init__(address, GatewayHandler)

    def server_close(self):
        try:
            super().server_close()
        finally:
            self.telemetry.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="HTTP listen address (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8765, help="HTTP listen port (default 8765)")
    parser.add_argument("--static-root", type=Path, help="Directory containing the built console")
    parser.add_argument("--odin1-map-dir", type=Path, default=DEFAULT_ODIN1_MAP_DIR,
                        help="ODIN1 静态点云展示目录，包含 metadata.json 和 points.bin")
    parser.add_argument("--odin1-pose-file", type=Path,
                        help="同机 PC2 观测器的原子 JSON 状态文件，用于显示实时 ODIN1 传感器位置")
    parser.add_argument("--ros", action="store_true", help="Use ROS 2 telemetry")
    parser.add_argument("--motion-socket", type=Path, help="Optional local motion arbiter socket in ROS mode")
    from .ros_telemetry import add_ros_arguments

    add_ros_arguments(parser)
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("--port must be between 0 and 65535")
    if args.motion_socket is not None and (not args.ros or not _loopback(args.host)):
        parser.error("--motion-socket requires --ros and a loopback --host")

    static_root = args.static_root
    if static_root is None and (DEFAULT_STATIC_ROOT / "index.html").is_file():
        static_root = DEFAULT_STATIC_ROOT
    if static_root is not None and not (static_root / "index.html").is_file():
        parser.error("--static-root must contain index.html")

    if args.ros:
        from .ros_telemetry import RosTelemetry

        if args.motion_socket is not None:
            from .motion import LocalMotionClient

            mode = "robot"
            control = LocalMotionClient(args.motion_socket)
        else:
            mode, control = "robot_readonly", None
        telemetry = RosTelemetry(args)
    else:
        telemetry = control = Simulator()
        mode = "simulator"

    with GatewayServer((args.host, args.port), telemetry, mode=mode, control=control,
                       static_root=static_root, odin1_map_dir=args.odin1_map_dir,
                       odin1_pose_file=args.odin1_pose_file) as server:
        url_host = f"[{args.host}]" if ":" in args.host else args.host
        print(f"A2 console ({mode}) http://{url_host}:{server.server_port}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
