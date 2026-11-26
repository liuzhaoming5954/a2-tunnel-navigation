"""用两个独立的假 HTTP 端口验证遥测与实机控制的路由、身份和时效。"""

import json
import unittest
from unittest.mock import MagicMock, patch

from a2_gateway.manual_link import PC2_STOP_RESET_PATH, PROTOCOL, unavailable
from a2_gateway.remote_stop_status import MAX_AGE_MS, SOURCE
from a2_gateway.station import StationSource


class StationManualChannelsTest(unittest.TestCase):
    def setUp(self):
        self.key = "test" * 8
        self.station = StationSource("http://127.0.0.1:8766", manual_key=self.key,
                                     manual_pc2_url="http://127.0.0.1:8768")
        guard = dict(source=SOURCE, guard="running", latched=False, age_ms=40, max_age_ms=MAX_AGE_MS)
        robot = dict(id="field-robot", online=True, battery_percent=79,
                     velocity=dict(x_mps=0.0, age_ms=20, max_age_ms=500))
        self.readonly = dict(deployment="pc2", mode="robot_readonly", remote_stop=guard, robot=robot)
        self.control = dict(self.readonly, remote_stop=dict(guard), robot=dict(robot),
                            control=dict(source="unitree", available=True, armed=False, reason="ready",
                                         limits=dict(linear_mps=.05, angular_radps=.15)))
        self.session = "fake-session-for-test"
        self.reset_reply = {"ok": True, "protocol": PROTOCOL, "reason": "stop_reset_done"}
        self.requests = []
        mocked = patch("a2_gateway.station.HTTPConnection", side_effect=self.connect)
        mocked.start()
        self.addCleanup(mocked.stop)

    def connect(self, host, port, timeout):
        self.assertEqual(host, "127.0.0.1")
        connection = MagicMock()

        def handle(method, path, *, body, headers):
            has_key = headers.get("Authorization") == "Bearer " + self.key
            self.requests.append((port, method, path, has_key))
            if port == 8766:
                self.assertNotIn("Authorization", headers)
                self.assertEqual(method, "GET")
                reply = self.readonly
            else:
                self.assertEqual(port, 8768)
                self.assertTrue(has_key)
                if method == "GET":
                    reply = self.control
                elif path == PC2_STOP_RESET_PATH:
                    self.assertEqual(json.loads(body), {"confirmed": True})
                    self.assertGreater(timeout, 22.0)
                    reply = self.reset_reply
                else:
                    reply = {"ok": True, "protocol": PROTOCOL, "session_id": self.session, "expires_in_ms": 400}
            if isinstance(reply, Exception):
                raise reply
            response = connection.getresponse.return_value
            response.status = 200
            response.read.return_value = json.dumps(reply).encode()

        connection.request.side_effect = handle
        return connection

    def test_control_can_only_use_authenticated_dedicated_port(self):
        self.assertEqual(self.station.acquire({})[0], 200)
        self.assertEqual(self.requests, [(8768, "GET", "/api/robot/state", True),
                                         (8766, "GET", "/api/robot/state", False),
                                         (8768, "POST", "/api/manual/session", True)])
        self.readonly = self.control = OSError()
        # 失去状态时，停车继续直送原控制端口，旧的只读服务永远不接收命令。
        self.requests.clear()
        self.assertEqual(self.station.stop({"session_id": self.session})[0], 200)
        self.assertEqual(self.requests, [(8768, "POST", "/api/manual/stop", True)])

    def test_control_endpoint_failure_keeps_actual_telemetry_and_lock_visible(self):
        self.readonly["remote_stop"]["latched"] = True
        self.control = TimeoutError()
        _, snapshot = self.station.state()
        self.assertIs(snapshot["independent_telemetry"], True)
        self.assertTrue(snapshot["pc2_connected"])
        self.assertTrue(snapshot["remote_stop"]["latched"])
        self.assertEqual(snapshot["robot"]["battery_percent"], 79)
        self.assertEqual(snapshot["control"], unavailable("manual_link_offline"))

    def test_independent_telemetry_only_contacts_readonly_channel(self):
        self.control = TimeoutError()
        status, snapshot = self.station.telemetry()
        self.assertEqual(status, 200)
        self.assertEqual(set(snapshot), {"deployment", "pc2_connected", "pc2_reason", "robot", "remote_stop"})
        self.assertEqual(snapshot["deployment"], "station")
        self.assertTrue(snapshot["pc2_connected"])
        self.assertEqual(snapshot["robot"]["velocity"]["x_mps"], 0)
        self.assertFalse(snapshot["remote_stop"]["latched"])
        self.assertEqual(self.requests, [(8766, "GET", "/api/robot/state", False)])

    def test_independent_telemetry_checks_each_lifetime_and_drops_offline_values(self):
        pack = {"soc_percent": 75, "age_ms": 20, "max_age_ms": 500}
        self.readonly["robot"].update(batteries={"primary": pack, "secondary": {**pack, "age_ms": 490}})
        self.readonly["remote_stop"]["age_ms"] = MAX_AGE_MS - 10
        # 10 毫秒传输用尽副电池和守护的有效期，主电池和速度仍可展示。
        with patch("a2_gateway.station.time.monotonic_ns", side_effect=[0, 10_000_000]):
            _, snapshot = self.station.telemetry()
        self.assertTrue(snapshot["pc2_connected"])
        self.assertEqual(snapshot["robot"]["batteries"]["primary"]["soc_percent"], 75)
        self.assertIsNone(snapshot["robot"]["batteries"]["secondary"])
        self.assertIsNotNone(snapshot["robot"]["velocity"])
        self.assertEqual(snapshot["remote_stop"]["guard"], "unknown")
        self.assertIsNone(snapshot["remote_stop"]["latched"])
        self.readonly = TimeoutError()
        _, snapshot = self.station.telemetry()
        self.assertFalse(snapshot["pc2_connected"])
        self.assertEqual(snapshot["pc2_reason"], "pc2_unavailable")
        self.assertEqual(snapshot["robot"]["batteries"], {"primary": None, "secondary": None})
        self.assertIsNone(snapshot["robot"]["velocity"])
        self.assertIsNone(snapshot["remote_stop"]["latched"])

    def test_both_guards_and_robot_id_must_agree_before_any_command(self):
        for report, field, value, reason in (
            (self.readonly["remote_stop"], "latched", True, "stop_latched"),
            (self.control["remote_stop"], "latched", True, "stop_latched"),
            (self.control["remote_stop"], "age_ms", MAX_AGE_MS, "stop_guard_not_ready"),
            (self.control["robot"], "id", "other-robot", "manual_robot_mismatch"),
            (self.readonly["robot"], "id", "", "manual_robot_mismatch"),
            (self.control["robot"], "online", False, "robot_offline"),
            (self.control["control"], "source", "mock", "manual_receiver_source_mismatch"),
            (self.control["control"], "source", "unknown", "motion_arbiter_unavailable"),
            (self.control, "control", None, "manual_control_disabled"),
        ):
            with self.subTest(field=field, value=value), patch.dict(report, {field: value}):
                self.requests.clear()
                self.assertEqual(self.station.acquire({}), (409, {"ok": False, "reason": reason}))
                self.assertFalse(any(method == "POST" for _, method, _, _ in self.requests))

    def test_lock_can_be_reported_when_the_receiver_was_removed_by_the_guard(self):
        self.readonly["remote_stop"]["latched"] = self.control["remote_stop"]["latched"] = True
        self.control["control"] = unavailable("stop_latched")
        _, snapshot = self.station.state()
        self.assertEqual(snapshot["control"], unavailable("stop_latched"))
        self.assertTrue(snapshot["remote_stop"]["latched"])

    def test_waiting_for_telemetry_can_expire_the_earlier_control_report(self):
        self.control["remote_stop"]["age_ms"] = MAX_AGE_MS - 50
        # 第一份控制报告在正式遥测返回前已耗去 60 毫秒。正式遥测刚取回，
        # 其锁存读数仍然可信，但第二条链路上的旧放行证据不能再签发会话。
        with patch("a2_gateway.station.time.monotonic_ns", side_effect=[0, 50_000_000, 60_000_000, 60_000_000]):
            _, snapshot = self.station.state()
        self.assertFalse(snapshot["remote_stop"]["latched"])
        self.assertEqual(snapshot["control"]["reason"], "stop_guard_not_ready")
        self.assertFalse(snapshot["control"]["available"])

    def _lock_for_reset(self):
        self.readonly["remote_stop"]["latched"] = self.control["remote_stop"]["latched"] = True
        self.control["control"] = unavailable("stop_latched")
        self.control["stop_reset"] = {"available": True, "reason": "stop_reset_ready"}

    def test_stop_reset_is_distinct_from_motion_and_only_uses_dedicated_port(self):
        self._lock_for_reset()
        _, snapshot = self.station.state()
        self.assertEqual(snapshot["control"], unavailable("stop_latched"))
        self.assertEqual(snapshot["stop_reset"], {"available": True, "reason": "stop_reset_ready"})
        self.requests.clear()
        self.assertEqual(self.station.stop_reset({"confirmed": True}), (200, {"ok": True, "reason": "stop_reset_done"}))
        self.assertEqual(self.requests, [(8768, "GET", "/api/robot/state", True),
                                         (8766, "GET", "/api/robot/state", False),
                                         (8768, "POST", PC2_STOP_RESET_PATH, True)])
        self.requests.clear()
        single_channel = StationSource("http://127.0.0.1:8766", manual_key=self.key)
        self.assertEqual(single_channel.telemetry()[0], 404)
        self.assertEqual(single_channel.stop_reset({"confirmed": True})[0], 405)
        self.assertEqual(self.station.stop_reset({"confirmed": 1})[0], 400)
        self.assertEqual(self.requests, [])

    def test_stop_reset_requires_fresh_locked_running_guards_and_matching_robot(self):
        self._lock_for_reset()
        for report, field, value, reason in (
            (self.readonly["remote_stop"], "latched", False, "stop_not_latched"),
            (self.control["remote_stop"], "guard", "stopped", "stop_guard_not_ready"),
            (self.control["remote_stop"], "age_ms", MAX_AGE_MS, "stop_guard_stale"),
            (self.control["robot"], "id", "different-device", "manual_robot_mismatch"),
            (self.control["robot"], "online", False, "robot_offline"),
            (self.control["control"], "reason", "manual_auth_failed", "manual_auth_failed"),
            (self.control, "stop_reset", None, "stop_reset_unavailable"),
        ):
            with self.subTest(field=field, value=value), patch.dict(report, {field: value}):
                self.requests.clear()
                self.assertEqual(self.station.stop_reset({"confirmed": True}), (409, {"ok": False, "reason": reason}))
                self.assertFalse(any(method == "POST" for _, method, _, _ in self.requests))

    def test_stop_reset_does_not_replay_a_lost_or_invalid_receipt(self):
        self._lock_for_reset()
        for reply in (TimeoutError(), {"ok": True, "reason": "stop_reset_done"}):
            with self.subTest(reply=reply):
                self.reset_reply = reply
                self.requests.clear()
                self.assertEqual(self.station.stop_reset({"confirmed": True}),
                                 (502, {"ok": False, "reason": "stop_reset_outcome_unknown"}))
                self.assertEqual(self.requests.count((8768, "POST", PC2_STOP_RESET_PATH, True)), 1)

    def test_only_distinct_configured_loopback_channels_are_allowed(self):
        for url in ("http://localhost:8766", "http://192.0.2.7:8768", "http://127.0.0.1:8768/?key=x"):
            with self.subTest(url=url), self.assertRaises(ValueError):
                StationSource("http://127.0.0.1:8766", manual_key=self.key, manual_pc2_url=url)
        with self.assertRaises(ValueError):
            StationSource("http://127.0.0.1:8766", manual_pc2_url="http://127.0.0.1:8768")
