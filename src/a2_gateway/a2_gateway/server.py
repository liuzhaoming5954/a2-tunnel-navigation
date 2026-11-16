"""Serve the local A2 console and its simulator or ROS API."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import mimetypes
from pathlib import Path
import socket
import time
from urllib.parse import unquote, urlsplit

from .simulator import Simulator


DEFAULT_STATIC_ROOT = Path(__file__).resolve().parents[1] / "web" / "dist"
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


class GatewayHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(5.0)

    def log_message(self, *_args):
        # Polling clients request state frequently. Request URLs and bodies
        # are deliberately kept out of the console log.
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
        # Accept normal origin-form request paths only.
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
        elif path is None:
            self._error(400, "invalid_path")
        elif path == "/api" or path.startswith("/api/"):
            self._error(404, "not_found")
        else:
            self._static(path)

    do_HEAD = do_GET

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

    def __init__(self, address, telemetry, *, mode, control=None, static_root=None):
        self.loopback_only = _loopback(address[0])
        if mode == "robot" and not self.loopback_only:
            raise ValueError("A gateway connected to a motion arbiter must bind to loopback")
        if ":" in address[0]:
            self.address_family = socket.AF_INET6
        self.telemetry = telemetry
        self.control = control
        self.mode = mode
        self.static_root = Path(static_root).resolve() if static_root is not None else None
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

    with GatewayServer((args.host, args.port), telemetry, mode=mode, control=control, static_root=static_root) as server:
        url_host = f"[{args.host}]" if ":" in args.host else args.host
        print(f"A2 console ({mode}) http://{url_host}:{server.server_port}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
