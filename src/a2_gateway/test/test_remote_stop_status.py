"""PC2 真实守护协议的合成输入及机载 HTTP 门禁测试，不调用机器人 SDK。"""

from http.client import HTTPConnection
import json
import socket
import stat
import struct
import subprocess
from threading import Thread
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from a2_gateway import remote_stop_status as guard
from a2_gateway.manual_link import BROWSER_STOP_RESET_PATH, PC2_STOP_RESET_PATH, PROTOCOL
from a2_gateway.server import GatewayServer, LocalStopReset
from a2_gateway.station import StationSource


def observation(latched, state="running", age=0):
    return {"source": guard.SOURCE, "guard": state, "latched": latched,
            "age_ms": age, "max_age_ms": guard.MAX_AGE_MS}


class SystemdStatusTest(unittest.TestCase):
    def sample(self, text, ready, **changes):
        fields = dict(LoadState="loaded", ActiveState="active", SubState="running", MainPID="42",
                      InvocationID="c" * 32, StatusText=text)
        fields.update(changes)
        raw = "\n".join(f"{k}={v}" for k, v in fields.items()) + "\n"
        with patch.object(guard.subprocess, "run", return_value=SimpleNamespace(stdout=raw)) as run, \
             patch.object(guard, "_ready_marker", return_value=ready):
            result = guard._read_guard()
        cmd = run.call_args.args[0]
        self.assertEqual(cmd[:4], ["/usr/bin/systemctl", "--system", "show", "a2-remote-stop.service"])
        self.assertLess(run.call_args.kwargs["timeout"], 1.0)
        return result

    def test_requires_agreement_between_live_service_and_marker(self):
        locked = "停车已锁存：恢复已有停车锁存或守护异常退出记录"
        armed = "遥控停车已布防，等待指定按键"
        for status, ready, expected in ((locked, False, True), (locked, True, None), (locked, None, None),
                                        (armed, True, False), (armed, False, None), (armed, None, None),
                                        ("", True, None)):
            with self.subTest(status=status, ready=ready):
                self.assertEqual(self.sample(status, ready), ("running", expected))

    def test_inactive_or_missing_services_never_turn_off_latch(self):
        for props, expected in (({"ActiveState": "inactive", "SubState": "dead", "MainPID": "0"}, "stopped"),
                                ({"ActiveState": "deactivating"}, "unknown"), ({"LoadState": "not-found"}, "unknown"),
                                ({"InvocationID": ""}, "unknown"), ({"InvocationID": "0" * 32}, "unknown")):
            with self.subTest(props=props):
                self.assertEqual(self.sample("停车已锁存：旧实例", False, **props), (expected, None))
        for error in (subprocess.TimeoutExpired("systemctl", .45), PermissionError()):
            with self.subTest(error=error), patch.object(guard.subprocess, "run", side_effect=error):
                self.assertEqual(guard._read_guard(), ("unknown", None))

    def test_marker_metadata_errors_and_untrusted_files_are_unknown(self):
        directory = SimpleNamespace(st_mode=stat.S_IFDIR | 0o755, st_uid=0)
        valid = SimpleNamespace(st_mode=stat.S_IFREG | 0o600, st_uid=0)
        symlink = SimpleNamespace(st_mode=stat.S_IFLNK | 0o777, st_uid=0)
        for marker, expected in ((valid, True), (symlink, None), (PermissionError(), None),
                                 (FileNotFoundError(), False)):
            with self.subTest(marker=marker), patch.object(guard, "_READY") as path:
                path.parent.lstat.return_value = directory
                if isinstance(marker, Exception):
                    path.lstat.side_effect = marker
                else:
                    path.lstat.return_value = marker
                self.assertIs(guard._ready_marker(), expected)

    def test_stale_sample_or_revoked_ready_immediately_blocks(self):
        sampler = guard.RemoteStopStatus()
        sampler._sample = (10., "running", False)
        with patch.object(guard.time, "monotonic", return_value=10.1), \
             patch.object(guard, "_ready_marker", return_value=False):
            status = sampler.snapshot()
            self.assertIsNone(status["latched"])
            self.assertEqual(guard.blocked_reason(status), "stop_guard_not_ready")
        with patch.object(guard.time, "monotonic", return_value=12.5):
            self.assertEqual(sampler.snapshot(), observation(None, "unknown", 2500))


class Pc2GuardGateTest(unittest.TestCase):
    def setUp(self):
        sensor = Mock()
        sensor.snapshot.return_value = {"robot": {"id": "fixture", "online": True}}
        self.receiver = Mock()
        self.receiver.state.return_value = {
            "source": "unitree", "available": True, "armed": False, "reason": "ready",
            "limits": {"linear_mps": .05, "angular_radps": .15},
        }
        self.receiver.stop.return_value = self.receiver.acquire.return_value = (200, {"ok": True})
        self.key = "static-test-only_" * 3
        self.monitor = Mock()
        self.server = GatewayServer(("127.0.0.1", 0), sensor, mode="manual_control", role="pc2",
                                    manual_control=self.receiver, manual_key=self.key, remote_stop=self.monitor)
        thread = Thread(target=self.server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 2)
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def request(self, operation=None, payload=None, *, authorized=False):
        c = HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
        try:
            if operation is None:
                c.request("GET", "/api/robot/state",
                          headers={"Authorization": "Bearer " + self.key} if authorized else {})
            else:
                c.request("POST", "/api/manual/" + operation, body=json.dumps(payload),
                          headers={"Authorization": "Bearer " + self.key, "Content-Type": "application/json"})
            reply = c.getresponse()
            return reply.status, json.loads(reply.read())
        finally:
            c.close()

    def test_read_only_status_and_command_rejection_with_stop_always_available(self):
        token = "fixture-test-session-a2"
        for status, reason in ((observation(True), "stop_latched"), (observation(None), "stop_guard_not_ready"),
                               (observation(None, "stopped"), "stop_guard_not_ready"),
                               (observation(False, age=2500), "stop_guard_not_ready")):
            with self.subTest(status=status):
                self.monitor.snapshot.return_value = status
                code, data = self.request()
                self.assertEqual((code, data["remote_stop"]), (200, status))
                self.assertNotIn("control", data)
                published = self.request(authorized=True)[1]["control"]
                self.assertEqual((published["available"], published["armed"], published["reason"]),
                                 (False, False, reason))
                self.assertEqual(self.request("session", {}), (409, {"ok": False, "reason": reason}))
                self.assertEqual(self.request("velocity", {"session_id": token, "sequence": 1,
                                                          "linear_mps": .05, "angular_radps": 0}),
                                 (409, {"ok": False, "reason": reason}))
                self.assertEqual(self.request("stop", {"session_id": token})[0], 200)
        self.receiver.velocity.assert_not_called()
        self.receiver.acquire.assert_not_called()
        self.monitor.snapshot.return_value = observation(False)
        self.assertEqual(self.request("session", {})[0], 200)
        self.receiver.acquire.assert_called_once_with({})

    def test_live_receiver_without_a_guard_source_cannot_use_mock_exception(self):
        with patch.object(self.server, "remote_stop", None):
            self.assertEqual(self.request("session", {}), (409, {"ok": False, "reason": "stop_guard_not_ready"}))
            self.receiver.acquire.assert_not_called()
            self.assertFalse(self.request(authorized=True)[1]["control"]["available"])

    def test_missing_managed_socket_preserves_guard_rejection_at_both_ends(self):
        # 模拟 A 键已清退整个发令进程组。此时站端和 PC2 仍可解释拒绝原因，完全无需底盘接收器。
        self.receiver.state.side_effect = AssertionError("守护未放行时不应访问套接字")
        station = StationSource(f"http://127.0.0.1:{self.server.server_port}", manual_key=self.key)
        velocity = {"session_id": "old-session-fixture", "sequence": 1, "linear_mps": .05, "angular_radps": 0}
        for report, reason in ((observation(True), "stop_latched"),
                               (observation(None, "stopped"), "stop_guard_not_ready")):
            with self.subTest(reason=reason):
                self.monitor.snapshot.return_value = report
                rejection = (409, {"ok": False, "reason": reason})
                self.assertEqual(station.state()[1]["control"]["reason"], reason)
                self.assertEqual(station.acquire({}), rejection)
                self.assertEqual(station.velocity(velocity), rejection)
                self.assertEqual(self.request("velocity", velocity), rejection)
        self.receiver.state.assert_not_called()
        self.receiver.acquire.assert_not_called()
        self.receiver.velocity.assert_not_called()

    def test_production_gateway_rejects_misconfigured_mock_receiver(self):
        self.monitor.snapshot.return_value = observation(False)
        self.receiver.state.return_value["source"] = "mock"
        rejection = (409, {"ok": False, "reason": "manual_receiver_source_mismatch"})
        state = self.request(authorized=True)[1]["control"]
        self.assertEqual((state["available"], state["reason"], state["source"]), (False, rejection[1]["reason"], None))
        self.assertEqual(self.request("session", {}), rejection)
        self.assertEqual(self.request("velocity", {"session_id": "old-session-fixture", "sequence": 1,
                                                 "linear_mps": .05, "angular_radps": 0}), rejection)
        self.receiver.acquire.assert_not_called()
        self.receiver.velocity.assert_not_called()

    def test_pc2_only_exposes_reset_to_its_authenticated_manual_gateway(self):
        self.monitor.snapshot.return_value = observation(True)
        self.assertEqual(self.request("stop-reset", {"confirmed": True}), (405, {"ok": False, "reason": "readonly"}))
        worker = Mock()
        worker.status.return_value = {"available": True, "reason": "stop_reset_ready"}
        worker.reset.return_value = (200, {"ok": True, "reason": "stop_reset_done"})
        with patch.object(self.server, "stop_reset", worker):
            self.assertNotIn("stop_reset", self.request()[1])
            worker.status.assert_not_called()
            data = self.request(authorized=True)[1]
            self.assertFalse(data["control"]["available"])
            self.assertTrue(data["stop_reset"]["available"])
            connection = HTTPConnection("127.0.0.1", self.server.server_port, timeout=2)
            self.addCleanup(connection.close)
            connection.request("POST", PC2_STOP_RESET_PATH, body='{"confirmed":true}',
                               headers={"Content-Type": "application/json"})
            response = connection.getresponse()
            self.assertEqual((response.status, json.loads(response.read())["reason"]), (403, "manual_auth_failed"))
            self.assertEqual(self.request("stop-reset", {"confirmed": 1})[0], 400)
            for report in (observation(True, "stopped"), observation(True, age=2500), observation(False)):
                self.monitor.snapshot.return_value = report
                self.assertEqual(self.request("stop-reset", {"confirmed": True})[0], 409)
            worker.reset.assert_not_called()
            self.monitor.snapshot.return_value = observation(True)
            self.assertEqual(self.request("stop-reset", {"confirmed": True}),
                             (200, {"ok": True, "reason": "stop_reset_done", "protocol": PROTOCOL}))
            worker.reset.assert_called_once_with()
            self.receiver.acquire.assert_not_called()
            self.receiver.velocity.assert_not_called()

    def test_station_reset_requires_browser_same_origin(self):
        station = Mock()
        station.stop_reset.return_value = 200, {"ok": True, "reason": "stop_reset_done"}
        with GatewayServer(("127.0.0.1", 0), station, mode="manual_control", role="station",
                           manual_control=station, manual_key=self.key) as server:
            thread = Thread(target=server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
            thread.start()
            port = server.server_port
            try:
                for origin, expected in ((None, 403), ("http://example.com", 403), (f"http://127.0.0.1:{port}", 200)):
                    with self.subTest(origin=origin):
                        connection = HTTPConnection("127.0.0.1", port, timeout=2)
                        try:
                            headers = {"Content-Type": "application/json"}
                            if origin:
                                headers["Origin"] = origin
                            connection.request("POST", BROWSER_STOP_RESET_PATH, body='{"confirmed":true}', headers=headers)
                            response = connection.getresponse()
                            self.assertEqual(response.status, expected)
                            response.read()
                        finally:
                            connection.close()
            finally:
                server.shutdown()
                thread.join(2)
        station.stop_reset.assert_called_once_with({"confirmed": True})


class LocalStopResetTest(unittest.TestCase):
    def test_socket_uses_kernel_root_credentials_and_fixed_request(self):
        reset = LocalStopReset("/run/test-only/stop-reset.sock")
        with patch("a2_gateway.server.socket.socket") as factory:
            connection = factory.return_value.__enter__.return_value
            connection.getsockopt.return_value = struct.pack("3i", 99, 1000, 1000)
            self.assertEqual(reset.reset(), (503, {"ok": False, "reason": "stop_reset_service_unavailable"}))
            connection.sendall.assert_not_called()
            connection.getsockopt.assert_called_once_with(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
            connection.getsockopt.return_value = struct.pack("3i", 99, 0, 1000)
            connection.recv.return_value = b'{"ok":true,"reason":"stop_reset_done"}\n'
            self.assertEqual(reset.reset(), (200, {"ok": True, "reason": "stop_reset_done"}))
            connection.sendall.assert_called_once_with(b'{"op":"reset","schema_version":1}\n')

    def test_status_is_bounded_and_reset_rejects_invalid_receipts(self):
        reset = LocalStopReset("/run/test-only/stop-reset.sock")
        with patch("a2_gateway.server.socket.socket") as factory:
            connection = factory.return_value.__enter__.return_value
            connection.getsockopt.return_value = struct.pack("3i", 99, 0, 0)
            connection.recv.return_value = b'{"ok":true,"available":true,"reason":"stop_reset_ready"}\n'
            self.assertEqual(reset.status(), {"available": True, "reason": "stop_reset_ready"})
            self.assertEqual(reset.status(), {"available": True, "reason": "stop_reset_ready"})
            connection.sendall.assert_called_once_with(b'{"op":"status","schema_version":1}\n')
            connection.recv.return_value = b'{"ok":false,"ok":true,"reason":"stop_reset_done"}\n'
            self.assertEqual(reset.reset(), (502, {"ok": False, "reason": "stop_reset_outcome_unknown"}))


if __name__ == "__main__":
    unittest.main()
