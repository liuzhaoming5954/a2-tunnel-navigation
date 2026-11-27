"""用本机单角色处理器和合成数据验证部署接口边界。"""

import hashlib
from http.client import HTTPConnection
import json
from pathlib import Path
import tempfile
from threading import Thread
import unittest
from unittest.mock import Mock, patch

from a2_gateway.manual_link import PROTOCOL, unavailable
from a2_gateway.remote_stop_status import MAX_AGE_MS, SOURCE
from a2_gateway.server import GatewayServer


class DeploymentRolesTest(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.dist = self.root / "dist"
        self.dist.mkdir()
        (self.dist / "index.html").write_bytes(b"<title>station</title>")

    def start_handler(self, role, source, *, mode="robot_readonly", **options):
        # 每例只启动一个随机回环端口，另一个角色由内存对象取代。
        server = GatewayServer(("127.0.0.1", 0), source, mode=mode, role=role,
                               static_root=self.dist, odin1_map_dir=self.root, **options)
        thread = Thread(target=server.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 2)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.port = server.server_port

    def request(self, path, method="GET", *, payload=None, key=None):
        conn = HTTPConnection("127.0.0.1", self.port, timeout=2)
        try:
            headers = {} if key is None else {"Authorization": "Bearer " + key}
            body = None if payload is None else json.dumps(payload).encode()
            if body is not None:
                headers["Content-Type"] = "application/json"
            conn.request(method, path, body=body, headers=headers)
            response = conn.getresponse()
            data = response.read()
            if response.getheader("Content-Type", "").startswith("application/json"):
                data = json.loads(data)
            return response.status, data
        finally:
            conn.close()

    def check_no_control(self):
        for operation in ("session", "velocity", "stop"):
            with self.subTest(operation=operation):
                self.assertEqual(self.request(f"/api/control/{operation}", "POST")[0], 405)
        # 两个真实部署角色均不可接收模拟点位，也不应展示一份虚构的任务状态。
        self.assertEqual(self.request("/api/patrol/point", "POST")[0], 405)
        self.assertEqual(self.request("/api/patrol")[0], 404)

    def test_pc2_serves_only_robot_and_versioned_pose(self):
        source = Mock(spec=["snapshot", "close"])
        source.snapshot.return_value = {"robot": {"id": "fixture"}, "pose": {"x_m": 99}, "control": {}}
        self.start_handler("pc2", source)
        self.assertEqual(self.request("/api/robot/state"), (200, {
            "deployment": "pc2", "mode": "robot_readonly", "robot": {"id": "fixture"},
        }))
        digest = "7" * 64
        route = "/api/odin1/pose"
        for suffix in ("", "?map_sha256=invalid", "?map_sha256=", f"?map_sha256={digest}&map_sha256={digest}"):
            with self.subTest(suffix=suffix):
                self.assertEqual(self.request(route + suffix)[0], 400)
        status, snapshot = self.request(f"{route}?map_sha256={digest}")
        self.assertEqual((status, snapshot["status"], snapshot["pose"]), (200, "waiting", None))
        # 即使同目录留有历史版本的页面，也不能通过 PC2 数据服务访问页面或展示地图。
        for path in ("/", "/index.html", "/api/state", "/api/telemetry", "/api/map", "/api/maps/odin1/metadata",
                     "/api/maps/odin1/points", "/api/maps/odin1/pose", "/api/control/session"):
            with self.subTest(path=path):
                self.assertEqual(self.request(path)[0], 404)
        self.check_no_control()

    def test_station_keeps_local_page_and_maps_on_disconnect(self):
        source = Mock(spec=["state", "telemetry", "pose", "map_data", "close"])
        source.state.return_value = (200, {"deployment": "station", "mode": "robot_readonly"})
        source.telemetry.return_value = (200, {"deployment": "station", "pc2_connected": True})
        source.map_data.return_value = {"id": "local-grid"}
        self.start_handler("station", source)
        self.assertEqual(self.request("/api/state"), source.state.return_value)
        self.assertEqual(self.request("/api/telemetry"), source.telemetry.return_value)
        source.telemetry.assert_called_once_with()
        self.assertEqual(self.request("/api/telemetry", "POST")[0], 405)
        for path in ("/api/robot/state", "/api/odin1/pose"):
            with self.subTest(path=path):
                self.assertEqual(self.request(path)[0], 404)
        self.check_no_control()

        # 远端状态和位姿均不可用时，本地网页、栅格和已验证版本的展示点云仍可读取。
        source.state.return_value = (200, {"deployment": "station", "mode": "robot_readonly",
                                          "pc2_connected": False, "pc2_reason": "pc2_unavailable",
                                          "robot": {"online": False}, "map": {"id": "local-grid"}})
        source.pose.return_value = (503, {"status": "disconnected", "pose": None})
        points = bytes(16)
        digest = hashlib.sha256(points).hexdigest()
        (self.root / "points.bin").write_bytes(points)
        metadata = {"sourceName": "fixture.pcd", "sourcePoints": 1, "pointCount": 1, "strideBytes": 16,
                    "encoding": "xyz-f32le-rgba-u8", "sampleSha256": digest,
                    "viewBounds": {"min": [0, 0, 0], "max": [1, 1, 1]}}
        (self.root / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
        self.assertEqual(self.request("/api/state"), source.state.return_value)
        self.assertEqual(self.request(f"/api/maps/odin1/pose?version={digest}"), source.pose.return_value)
        source.pose.assert_called_once_with(self.root, [digest])
        self.assertEqual(self.request("/"), (200, b"<title>station</title>"))
        self.assertEqual(self.request("/api/map"), (200, {"id": "local-grid"}))
        self.assertEqual(self.request("/api/maps/odin1/metadata"), (200, metadata))
        self.assertEqual(self.request(f"/api/maps/odin1/points?version={digest}"), (200, points))

    def _start_pc2_activation(self):
        # 模拟接收器仍无运动许可。装入仅返回固定会话的假 broker，验证实际 HTTP 路由。
        telemetry = Mock(spec=["snapshot", "close"])
        telemetry.snapshot.return_value = {"robot": {"id": "field-robot", "online": True}}
        guard = Mock(spec=["snapshot", "start", "close"])
        guard.snapshot.return_value = dict(source=SOURCE, guard="running", latched=False, age_ms=40,
                                           max_age_ms=MAX_AGE_MS)
        receiver = Mock(spec=["state", "acquire", "velocity"])
        receiver.state.return_value = unavailable("local_authorization_required", "unitree")
        broker = Mock(spec=["available", "acquire"])
        broker.available.return_value = True
        broker.acquire.return_value = 200, {"ok": True, "session_id": "mock-session-receipt",
                                             "continuous_hold": True}
        key = "test" * 8
        with patch("a2_gateway.server.LocalManualArm", return_value=broker):
            self.start_handler("pc2", telemetry, mode="manual_control", remote_stop=guard,
                               manual_control=receiver, manual_key=key, manual_arm_socket=self.root / "arm.sock")
        return telemetry, guard, receiver, broker, key

    def test_pc2_one_click_requires_authenticated_session_post_and_never_sends_motion(self):
        _, _, receiver, broker, key = self._start_pc2_activation()
        state = "/api/robot/state"
        session = "/api/manual/session"
        self.assertNotIn("control", self.request(state)[1])
        self.assertEqual(self.request(state, key="wrong-key")[1]["control"], unavailable("manual_auth_failed"))
        self.assertEqual(self.request(state, key=key)[1]["control"],
                         {**receiver.state.return_value, "activation_available": True, "reusable_hold": True})
        broker.acquire.assert_not_called()
        for invalid in (None, "wrong-key"):
            self.assertEqual(self.request(session, "POST", payload={}, key=invalid),
                             (403, {"ok": False, "reason": "manual_auth_failed"}))
        self.assertEqual(self.request(session, "POST", payload={"direction": "forward"}, key=key),
                         (400, {"ok": False, "reason": "invalid_manual_command"}))
        for speed in (0., .05):
            self.assertEqual(self.request("/api/manual/velocity", "POST", key=key, payload={
                "session_id": "mock-session-receipt", "sequence": 1, "linear_mps": speed, "angular_radps": 0.,
            }), (409, {"ok": False, "reason": "local_authorization_required"}))
        broker.acquire.assert_not_called()
        self.assertEqual(self.request(session, "POST", payload={}, key=key),
                         (200, {**broker.acquire.return_value[1], "protocol": PROTOCOL}))
        broker.acquire.assert_called_once_with({})
        receiver.acquire.assert_not_called()
        receiver.velocity.assert_not_called()

    def test_pc2_reusable_hold_forwards_only_explicit_opt_in(self):
        _, _, receiver, broker, key = self._start_pc2_activation()
        session = "/api/manual/session"
        opt_in = {"reusable_hold": True}
        for invalid in ({"reusable_hold": False}, {"reusable_hold": 1}, {**opt_in, "direction": "forward"}):
            self.assertEqual(self.request(session, "POST", payload=invalid, key=key),
                             (400, {"ok": False, "reason": "invalid_manual_command"}))
        receipt = {**broker.acquire.return_value[1], **opt_in}
        broker.acquire.return_value = 200, receipt
        self.assertEqual(self.request(session, "POST", payload=opt_in, key=key),
                         (200, {**receipt, "protocol": PROTOCOL}))
        broker.acquire.assert_called_once_with(opt_in)
        receiver.acquire.assert_not_called()
        receiver.velocity.assert_not_called()

    def test_pc2_one_click_still_checks_guard_and_telemetry_before_broker(self):
        telemetry, guard, receiver, broker, key = self._start_pc2_activation()
        sample = guard.snapshot.return_value
        for values, reason in (({"latched": True}, "stop_latched"),
                               ({"guard": "unknown", "latched": None}, "stop_guard_not_ready"),
                               ({"age_ms": MAX_AGE_MS}, "stop_guard_not_ready")):
            with self.subTest(reason=reason, values=values):
                guard.snapshot.return_value = {**sample, **values}
                status, report = self.request("/api/robot/state", key=key)
                self.assertEqual((status, report["control"]), (200, unavailable(reason)))
                self.assertEqual(self.request("/api/manual/session", "POST", payload={}, key=key),
                                 (409, {"ok": False, "reason": reason}))
        guard.snapshot.return_value = sample
        telemetry.snapshot.return_value["robot"]["online"] = False
        self.assertEqual(self.request("/api/manual/session", "POST", payload={}, key=key),
                         (409, {"ok": False, "reason": "robot_offline"}))
        broker.acquire.assert_not_called()
        receiver.acquire.assert_not_called()
        receiver.velocity.assert_not_called()
