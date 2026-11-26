"""在随机回环端口验证浏览器、上位机和模拟 PC2 之间的手动遥控接口。"""

from http.client import HTTPConnection
import json
import math
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
from threading import Thread
import unittest
from unittest.mock import Mock, patch

from a2_gateway import manual_link
from a2_gateway.motion import LocalMotionClient
from a2_gateway.server import GatewayServer
from a2_gateway.station import StationSource


KEY = "manual-test-only_" * 3
SESSION = "fixture-session-for-manual"
TURN_RADPS = math.pi / 18


class ManualLinkTest(unittest.TestCase):
    def setUp(self):
        # 合成 PC2 仅记录请求；测试进程不导入机身 SDK，也不接入现场端口。
        self.telemetry = Mock(spec=["snapshot", "close"])
        self.telemetry.snapshot.return_value = {"robot": {"online": True, "id": "mock-A2"}}
        self.receiver = Mock(spec=["state", "acquire", "velocity", "stop", "action_start", "action_release"])
        self.receiver.state.return_value = {"available": True, "armed": False, "source": "mock", "reason": "ready",
                                            "limits": {"linear_mps": .20, "forward_mps": .20, "angular_radps": TURN_RADPS}}
        self.receiver.acquire.return_value = (200, {"ok": True, "session_id": SESSION, "expires_in_ms": 8000})
        self.receiver.velocity.return_value = self.receiver.stop.return_value = (200, {"ok": True})
        self.pc2 = self.start("pc2", self.telemetry, self.receiver)
        source = StationSource("http://127.0.0.1:" + str(self.pc2), manual_key=KEY)
        self.station = self.start("station", source, source)
        self.origin = {"Origin": "http://127.0.0.1:" + str(self.station)}

    def start(self, role, telemetry, port, key=KEY):
        server = GatewayServer(("127.0.0.1", 0), telemetry, role=role, mode="manual_control",
                               manual_control=port, manual_key=key)
        thread = Thread(target=server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 2)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return server.server_port

    def request(self, port, path, payload=None, headers=None):
        connection = HTTPConnection("127.0.0.1", port, timeout=2)
        try:
            connection.request("GET" if payload is None else "POST", path,
                               body=None if payload is None else json.dumps(payload),
                               headers={"Content-Type": "application/json", **(headers or {})})
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def test_existing_telemetry_credential_keeps_readonly_access(self):
        _, data = self.request(self.pc2, "/api/robot/state")
        self.assertNotIn("control", data)
        self.assertTrue(data["robot"]["online"])
        for headers in ({}, {"Authorization": "Bearer invalid-test"}):
            self.assertEqual(self.request(self.pc2, "/api/manual/session", {}, headers)[0], 403)
        self.assertEqual(self.request(self.pc2, "/api/control/session", {}, self.origin)[0], 405)
        self.receiver.acquire.assert_not_called()
        # Chrome 或本机脚本不能利用另一个站点对回环端口提交遥控申请。
        for headers in ({}, {"Origin": "null"}, {"Origin": "http://elsewhere.invalid"}):
            self.assertEqual(self.request(self.station, "/api/control/session", {}, headers)[0], 403)
        self.receiver.acquire.assert_not_called()

    def test_station_forwards_only_the_authorized_session_and_body_axes(self):
        _, state = self.request(self.station, "/api/state")
        self.assertEqual(state["mode"], "manual_control")
        self.assertEqual(state["control"], self.receiver.state.return_value)
        self.assertNotIn(KEY, json.dumps(state))
        code, session = self.request(self.station, "/api/control/session", {}, self.origin)
        self.assertEqual((code, session), self.receiver.acquire.return_value)
        # 开始运动前的零向量只负责会话保活，网关透传，接收端判定是否允许。
        for seq, (vx, wz) in enumerate(((0, 0), (.20, 0), (-.20, 0), (0, TURN_RADPS), (0, -TURN_RADPS)), start=1):
            command = dict(session_id=SESSION, sequence=seq, linear_mps=vx, angular_radps=wz)
            self.assertEqual(self.request(self.station, "/api/control/velocity", command, self.origin),
                             (200, {"ok": True}))
            self.receiver.velocity.assert_called_with(command)
        self.receiver.velocity.reset_mock()
        for changes in (dict(sequence=True), dict(session_id=""), dict(sequence=0),
                        dict(linear_mps=.20001, angular_radps=0),
                        dict(linear_mps=-.20001, angular_radps=0),
                        dict(angular_radps=TURN_RADPS + .00001),
                        dict(angular_radps=-TURN_RADPS - .00001),
                        dict(linear_mps=.04, angular_radps=.1), dict(sideways_mps=.03)):
            self.assertEqual(self.request(self.station, "/api/control/velocity", {**command, **changes},
                                          self.origin)[0], 400)
        self.receiver.velocity.assert_not_called()
        self.receiver.velocity.return_value = (409, {"ok": False, "reason": "stale_sequence"})
        self.assertEqual(self.request(self.station, "/api/control/velocity", command, self.origin),
                         self.receiver.velocity.return_value)
        self.receiver.velocity.assert_called_once_with(command)
        self.telemetry.snapshot.return_value["robot"]["online"] = False
        self.assertFalse(self.request(self.station, "/api/state")[1]["control"]["available"])
        self.assertEqual(self.request(self.station, "/api/control/velocity", command, self.origin)[0], 409)
        self.assertEqual(self.request(self.station, "/api/control/stop", {"session_id": SESSION}, self.origin),
                         (200, {"ok": True}))
        self.receiver.stop.assert_called_once_with({"session_id": SESSION})

    def test_pc2_forwards_explicit_speed_and_accepts_legacy_limits(self):
        original = self.receiver.state.return_value
        client = LocalMotionClient("/unused-test-socket", require_source=True)
        for forward in (.20, None):
            with self.subTest(forward=forward):
                limits = {**original["limits"]}
                if forward is None:
                    del limits["forward_mps"]
                    limits["linear_mps"] = .05
                    limits["angular_radps"] = .15
                self.receiver.state.return_value = {**original, "limits": limits}
                with patch.object(client, "_request", return_value={"ok": True, "control": self.receiver.state.return_value}):
                    self.assertEqual(client.state()["limits"], limits)
                self.assertEqual(self.request(self.station, "/api/state")[1]["control"]["limits"], limits)
        self.receiver.state.return_value = {**original, "limits": {**original["limits"], "forward_mps": .20001}}
        self.assertEqual(self.request(self.station, "/api/state")[1]["control"]["reason"], "manual_invalid_limits")

    def test_repeated_session_action_contract_across_station_and_pc2(self):
        base = self.receiver.state.return_value
        session = {"ok": True, "session_id": SESSION, "repeated_actions": True, "action_limit_ms": 30000}
        self.receiver.acquire.return_value = (200, session)
        self.receiver.action_start.return_value = (200, {"ok": True, "action_id": 2, "expires_in_ms": 30000})
        self.receiver.action_release.return_value = (200, {"ok": True})
        self.assertEqual(self.request(self.station, "/api/control/session", {}, self.origin), (200, session))
        start = {"session_id": SESSION, "sequence": 2, "direction": "backward"}
        self.assertEqual(self.request(self.station, "/api/control/action/start", start)[0], 403)
        self.assertEqual(self.request(self.station, "/api/control/action/start", start, self.origin),
                         self.receiver.action_start.return_value)
        self.receiver.action_start.assert_called_once_with(start)
        self.receiver.state.return_value = {**base, "armed": True, "action_state": "active", "action_id": 2}
        control = self.request(self.station, "/api/state")[1]["control"]
        self.assertEqual((control["action_state"], control["action_id"]), ("active", 2))
        velocity = dict(session_id=SESSION, sequence=3, action_id=2, linear_mps=-.20, angular_radps=0.)
        self.assertEqual(self.request(self.station, "/api/control/velocity", velocity, self.origin),
                         (200, {"ok": True}))
        self.receiver.velocity.assert_called_once_with(velocity)
        for bad in ({**velocity, "action_id": 3}, {**velocity, "linear_mps": 0.}):
            self.assertEqual(self.request(self.station, "/api/control/velocity", bad, self.origin)[0], 400)
        # 遥测掉线时仍能转发松键。未发出的停车请求也有机载看门狗兜底。
        self.telemetry.snapshot.return_value["robot"]["online"] = False
        release = dict(session_id=SESSION, sequence=4, action_id=2)
        self.assertEqual(self.request(self.station, "/api/control/action/release", release, self.origin),
                         (200, {"ok": True}))
        self.receiver.action_release.assert_called_once_with(release)

    def test_continuous_hold_requires_an_unambiguous_receipt_at_both_boundaries(self):
        client = LocalMotionClient("/unused-test-socket", require_source=True)
        state = {"ok": True, "control": self.receiver.state.return_value}
        session = {"ok": True, "session_id": SESSION, "continuous_hold": True}
        self.receiver.acquire.return_value = (200, session)
        with patch.object(client, "_request", side_effect=[state, session]):
            self.assertEqual(client.acquire({}), (200, session))
        self.assertEqual(self.request(self.station, "/api/control/session", {}, self.origin), (200, session))

        # 指令仍采用普通会话的格式，HTTP 转发不提供独立按键证据。
        velocity = {"session_id": SESSION, "sequence": 1, "linear_mps": .20, "angular_radps": 0.}
        self.assertEqual(self.request(self.station, "/api/control/velocity", velocity, self.origin), (200, {"ok": True}))
        self.receiver.velocity.assert_called_once_with(velocity)

        for ambiguous in ({"continuous_hold": False}, {"continuous_hold": 1},
                          {"expires_in_ms": 30000}, {"action_limit_ms": 30000}, {"repeated_actions": True}):
            reply = {**session, **ambiguous}
            with self.subTest(reply=reply), patch.object(client, "_request", side_effect=[state, reply]):
                self.assertEqual(client.acquire({})[0], 503)
                self.receiver.acquire.return_value = (200, reply)
                self.assertEqual(self.request(self.station, "/api/control/session", {}, self.origin)[0], 502)

    def test_socket_and_http_reject_unbound_or_overlong_action_receipts(self):
        payload = {"session_id": SESSION, "sequence": 8, "direction": "forward"}
        client = LocalMotionClient("/unused-test-socket")
        for reply in ({"ok": True, "action_id": 7, "expires_in_ms": 30000},
                      {"ok": True, "action_id": 8, "expires_in_ms": 30001},
                      {"ok": True, "action_id": True, "expires_in_ms": 100}):
            with self.subTest(reply=reply), patch.object(client, "_request", return_value=reply):
                self.assertEqual(client.action_start(payload)[0], 503)
                self.receiver.action_start.return_value = (200, reply)
                self.assertEqual(self.request(self.station, "/api/control/action/start", payload, self.origin)[0], 502)
        self.receiver.state.return_value.update(armed=True, action_state="release_required", action_id=None)
        self.assertFalse(self.request(self.station, "/api/state")[1]["control"]["available"])

    def test_wrong_key_cannot_open_motion_or_expose_credentials(self):
        wrong_key = "different-test-secret_" * 2
        wrong = StationSource("http://127.0.0.1:" + str(self.pc2), manual_key=wrong_key)
        _, state = wrong.state()
        self.assertTrue(state["pc2_connected"])
        self.assertFalse(state["control"]["available"])
        self.assertEqual(state["control"]["reason"], "manual_auth_failed")
        self.assertEqual(wrong.acquire({}), (403, {"ok": False, "reason": "manual_auth_failed"}))
        self.receiver.acquire.assert_not_called()

    def test_unexpected_responses_and_lost_replies_never_retry_a_velocity(self):
        cmd = dict(session_id=SESSION, sequence=1, linear_mps=.05, angular_radps=0)
        for expected, reply in ((502, (200, {"ok": True})), (503, OSError("link down"))):
            with self.subTest(reply=reply):
                upstream = Mock(side_effect=[reply])
                self.assertEqual(manual_link.forward_command(upstream, "/api/manual/velocity", cmd)[0], expected)
                upstream.assert_called_once_with("POST", "/api/manual/velocity", cmd)
        for old_state in ({"available": True}, {**self.receiver.state.return_value, "source": "unknown"},
                          {**self.receiver.state.return_value, "limits": {"linear_mps": .5, "angular_radps": 1.}}):
            self.receiver.state.return_value = old_state
            self.assertFalse(self.request(self.station, "/api/state")[1]["control"]["available"])

    def test_keys_and_manual_listeners_are_local_and_explicit(self):
        with TemporaryDirectory() as folder:
            key_path = Path(folder) / "manual.key"
            key_path.write_text(KEY + "\n")
            key_path.chmod(0o600)
            self.assertEqual(manual_link.read_key(key_path), KEY)
            key_path.chmod(0o640)
            with self.assertRaises(ValueError):
                manual_link.read_key(key_path)
        with self.assertRaises(ValueError):
            StationSource("http://192.0.2.1:8765", manual_key=KEY)
        for host, goal, mode, key in (("0.0.0.0", None, "manual_control", KEY),
                                      ("127.0.0.1", Mock(), "manual_control", KEY),
                                      ("127.0.0.1", None, "robot_readonly", KEY),
                                      ("127.0.0.1", None, "manual_control", None)):
            with self.subTest(host=host, mode=mode), self.assertRaises(ValueError):
                GatewayServer((host, 0), self.telemetry, mode=mode, role="pc2",
                              single_goal=goal, manual_control=self.receiver, manual_key=key)

    def test_pc2_rejection_or_unknown_source_never_sends_acquire(self):
        client = LocalMotionClient("/unused-test-socket", require_source=True)
        base = {**self.receiver.state.return_value, "source": "unitree"}
        rejected = [
            ({**base, "available": False, "reason": reason}, reason)
            for reason in ("stop_latched", "stop_unconfirmed", "local_authorization_required")
        ]
        for control, reason in (*rejected, ({**base, "source": None}, "motion_arbiter_unavailable")):
            with self.subTest(reason=reason), patch.object(client, "_request",
                    return_value={"ok": True, "control": control}) as request:
                self.assertEqual(client.acquire({}), (409, {"ok": False, "reason": reason}))
                request.assert_called_once_with("status", {})

    def test_explicit_receiver_uses_only_the_configured_kernel_peer_uid(self):
        with patch("pwd.getpwnam", side_effect=KeyError), self.assertRaisesRegex(ValueError, "a2-autonomy"):
            LocalMotionClient("/example.sock", peer_user="a2-autonomy")
        with patch("pwd.getpwnam", return_value=Mock(pw_uid=3030)), patch("os.geteuid", return_value=2020):
            production = LocalMotionClient("/example.sock", peer_user="a2-autonomy")
            simulated = LocalMotionClient("/example.sock")
        for client, uid, accepted in ((production, 0, False), (production, 2020, False),
                                      (production, 3030, True), (simulated, 0, True),
                                      (simulated, 2020, True), (simulated, 3030, False)):
            with self.subTest(peer=uid, production=client is production), patch("socket.socket") as socket_factory:
                peer = socket_factory.return_value.__enter__.return_value
                peer.getsockopt.return_value = struct.pack("3i", 1001, uid, 17)
                peer.recv.return_value = b'{"ok":true}\n'
                result = client.stop({"session_id": SESSION})
                if accepted:
                    self.assertEqual(result, (200, {"ok": True}))
                    peer.sendall.assert_called_once()
                else:
                    self.assertEqual(result, (503, {"ok": False, "reason": "motion_arbiter_unavailable"}))
                    peer.sendall.assert_not_called()

    def test_old_session_rejection_is_not_a_stop_receipt(self):
        old = "previous-session-already-revoked"
        self.receiver.stop.return_value = (409, {"ok": False, "reason": "invalid_session"})
        self.assertEqual(self.request(self.station, "/api/control/stop", {"session_id": old}, self.origin),
                         self.receiver.stop.return_value)
        self.receiver.stop.assert_called_once_with({"session_id": old})
