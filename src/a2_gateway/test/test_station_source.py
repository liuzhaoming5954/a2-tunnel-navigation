"""以合成 PC2 报文和最小地图验证上位机的数据来源与失败行为。"""

from io import BytesIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import MagicMock, patch

from a2_gateway.odin1_pose import MAX_AGE_MS, SOURCE
from a2_gateway import remote_stop_status
from a2_gateway.station import StationSource


class StationSourceTest(unittest.TestCase):
    def setUp(self):
        self.work = TemporaryDirectory()
        self.addCleanup(self.work.cleanup)
        self.root = Path(self.work.name)
        self.bin_hash, self.sample = "a" * 64, "b" * 64
        self.meta = {"sampleSha256": self.sample, "relocMapSha256": self.bin_hash}
        (self.root / "metadata.json").write_text(json.dumps(self.meta))
        self.source = StationSource("http://127.0.0.1:8766")
        self.pose = dict(x_m=1.5, y_m=-2, z_m=0.4, yaw_rad=-0.1)
        self.live = dict(source=SOURCE, status="live", map_sha256=self.bin_hash,
                         pose=self.pose, age_ms=4, max_age_ms=MAX_AGE_MS)

    def test_pc2_state_never_imports_remote_maps_control_or_body_pose(self):
        message = {"deployment": "pc2", "mode": "robot_readonly",
                   "robot": {"id": "test", "online": True, "battery_percent": 72, "extra": "ignored"},
                   "map": {"id": "foreign"}, "pose": self.pose, "control": {"armed": True}}
        with patch.object(self.source, "_get", return_value=message) as read:
            code, state = self.source.state()
            read.assert_called_once_with("/api/robot/state")
        self.assertEqual(code, 200)
        self.assertTrue(state["pc2_connected"])
        self.assertIsNone(state["map"])
        self.assertIsNone(state["pose"])
        self.assertFalse(state["control"]["armed"])
        self.assertFalse(state["control"]["available"])
        self.assertEqual(state["robot"]["battery_percent"], 72)
        self.assertNotIn("extra", state["robot"])

    def test_offline_platform_drops_old_telemetry_without_losing_pc2(self):
        remote = {"deployment": "pc2", "mode": "robot_readonly",
                  "robot": {"online": False, "id": "test", "battery_percent": 64}}
        with patch.object(self.source, "_get", return_value=remote):
            _, state = self.source.state()
        self.assertTrue(state["pc2_connected"])
        self.assertFalse(state["robot"]["online"])
        self.assertEqual(state["robot"]["id"], "test")
        self.assertIsNone(state["robot"]["battery_percent"])
        self.assertEqual(state["robot"]["batteries"], {"primary": None, "secondary": None})

    def test_readonly_station_reports_current_guard_even_when_body_is_offline(self):
        guard = {"source": remote_stop_status.SOURCE, "guard": "running", "latched": True,
                 "age_ms": 90, "max_age_ms": remote_stop_status.MAX_AGE_MS}
        message = {"deployment": "pc2", "mode": "robot_readonly", "robot": {"online": False}}
        for sample, expected_reason in ((guard, "stop_latched"), ({**guard, "latched": False}, "robot_readonly")):
            with self.subTest(sample=sample), patch.object(self.source, "_get", return_value={**message, "remote_stop": sample}):
                _, state = self.source.state()
                self.assertTrue(state["pc2_connected"])
                self.assertEqual(state["remote_stop"], sample)
                self.assertFalse(state["control"]["available"])
                self.assertFalse(state["control"]["armed"])
                self.assertEqual(state["control"]["reason"], expected_reason)

    def test_missing_malformed_and_expired_guard_do_not_hide_valid_telemetry(self):
        guard = {"source": remote_stop_status.SOURCE, "guard": "running", "latched": True,
                 "age_ms": remote_stop_status.MAX_AGE_MS - 5, "max_age_ms": remote_stop_status.MAX_AGE_MS}
        message = {"deployment": "pc2", "mode": "robot_readonly", "robot": {"online": True}}
        for sample in (None, guard, {**guard, "age_ms": False}, {**guard, "source": "mock", "age_ms": 0},
                       {**guard, "guard": {}, "age_ms": 0}, {**guard, "latched": 1, "age_ms": 0},
                       {**guard, "max_age_ms": 99999, "age_ms": 0}):
            with self.subTest(sample=sample), patch.object(self.source, "_get", return_value={**message, "remote_stop": sample}), \
                    patch("a2_gateway.station.time.monotonic_ns", side_effect=[0, 5000000]):
                _, state = self.source.state()
                self.assertTrue(state["pc2_connected"])
                self.assertEqual(state["remote_stop"]["guard"], "unknown")
                self.assertIsNone(state["remote_stop"]["latched"])
                self.assertFalse(state["control"]["available"])
        # 含锁存旧帧的 PC2 一旦掉线，站端不能把它呈现为当前已知状态。
        with patch.object(self.source, "_get", side_effect=OSError):
            _, state = self.source.state()
            self.assertFalse(state["pc2_connected"])
            self.assertIsNone(state["remote_stop"]["latched"])

    def test_direct_manual_request_checks_guard_before_forwarding_and_preserves_stop(self):
        station = StationSource("http://127.0.0.1:12345", manual_key="test" * 8)
        control = {"source": "unitree", "available": True, "armed": True, "reason": "armed",
                   "limits": {"linear_mps": .05, "angular_radps": .15}}
        base = {"deployment": "pc2", "mode": "robot_readonly", "robot": {"online": True}, "control": control}
        guard = {"source": remote_stop_status.SOURCE, "guard": "running", "latched": True,
                 "age_ms": 10, "max_age_ms": remote_stop_status.MAX_AGE_MS}
        cmd = {"session_id": "synthetic-session", "sequence": 1, "linear_mps": .05, "angular_radps": 0}
        for sample, reason in ((guard, "stop_latched"), (None, "stop_guard_not_ready"),
                               ({**guard, "latched": None}, "stop_guard_not_ready"),
                               ({**guard, "guard": "stopped", "latched": False}, "stop_guard_not_ready"),
                               ({**guard, "age_ms": guard["max_age_ms"]}, "stop_guard_not_ready")):
            with self.subTest(sample=sample), patch.object(station, "_get", return_value={**base, "remote_stop": sample}), \
                    patch.object(station, "_pc2_request") as upstream:
                self.assertEqual(station.acquire({}), (409, {"ok": False, "reason": reason}))
                self.assertEqual(station.velocity(cmd), (409, {"ok": False, "reason": reason}))
                upstream.assert_not_called()
        # 控制通道中断时仍可尝试停车；该操作独立于只读状态请求。
        with patch.object(station, "_get", side_effect=OSError) as read, \
                patch.object(station, "_pc2_request", return_value=(200, {"ok": True, "protocol": "a2-manual-v1"})) as send:
            self.assertEqual(station.stop({"session_id": cmd["session_id"]}), (200, {"ok": True}))
            read.assert_not_called()
            send.assert_called_once_with("POST", "/api/manual/stop", {"session_id": cmd["session_id"]})
        with patch.object(station, "_get", return_value={**base, "remote_stop": {**guard, "latched": False}}), \
                patch.object(station, "_pc2_request", return_value=(200, {"ok": True, "protocol": "a2-manual-v1"})) as send:
            self.assertEqual(station.velocity(cmd), (200, {"ok": True}))
            send.assert_called_once_with("POST", "/api/manual/velocity", cmd)

    def test_dual_pack_forwarding_accounts_for_transport_and_preserves_zero(self):
        # PC2 两路电池采集时间不同，同一 HTTP 响应到站后可以只有一侧仍有效。
        primary = {"soc_percent": 0, "soh_percent": 99, "cycle_count": 0, "charging": False,
                   "age_ms": 35, "max_age_ms": 500}
        secondary = {**primary, "age_ms": 495}
        message = {"deployment": "pc2", "mode": "robot_readonly", "robot": {
            "id": "a2", "online": False, "batteries": {"primary": primary, "secondary": secondary},
            "velocity": {"x_mps": False, "y_mps": 0., "z_mps": float("inf"), "yaw_radps": -.5,
                         "age_ms": 2, "max_age_ms": 500},
        }}
        with patch.object(self.source, "_get", return_value=message), \
                patch("a2_gateway.station.time.monotonic_ns", side_effect=[0, 5_000_000]):
            _, state = self.source.state()
        self.assertTrue(state["pc2_connected"])
        robot = state["robot"]
        self.assertEqual(robot["batteries"]["primary"], primary)
        self.assertIsNone(robot["batteries"]["secondary"])
        self.assertEqual(robot["velocity"], {"x_mps": None, "y_mps": 0., "z_mps": None,
                                             "yaw_radps": -.5, "age_ms": 2, "max_age_ms": 500})
        self.assertIsNone(robot["attitude"])
        self.assertIsNone(robot["battery_percent"])

    def test_battery_lifetime_on_a_slow_round_trip(self):
        # 浏览器请求 240 ms，其中站端读取 PC2 占 230 ms。主包仍有 190 ms 可展示，
        # 副包恰好在到站时过期，不能借用主包的时间，也不能延长其有效期。
        primary = {"soc_percent": 18, "soh_percent": 100, "cycle_count": 6, "charging": None,
                   "age_ms": 70, "max_age_ms": 500}
        remote = {"deployment": "pc2", "mode": "robot_readonly", "robot": {
            "online": True, "batteries": {"primary": primary, "secondary": {**primary, "age_ms": 270}},
        }}
        with patch.object(self.source, "_get", return_value=remote), \
                patch("a2_gateway.station.time.monotonic_ns", side_effect=[0, 230_000_000]):
            _, state = self.source.state()
        packs = state["robot"]["batteries"]
        self.assertEqual(packs["primary"], primary)
        self.assertEqual(packs["primary"]["max_age_ms"] - packs["primary"]["age_ms"] - 240, 190)
        self.assertIsNone(packs["secondary"])

    def test_no_remote_falls_back_to_simulator_or_previous_telemetry(self):
        for value in [dict(mode="simulator", robot={"online": True}, deployment="pc2"),
                      dict(mode="robot_readonly", robot={"online": True}, deployment="station"),
                      dict(mode="robot_readonly", robot={"online": "true"}, deployment="pc2"),
                      OSError("offline")]:
            with self.subTest(value=value), patch.object(self.source, "_get", side_effect=[value]):
                code, state = self.source.state()
                self.assertEqual(code, 200)
                self.assertEqual(state["mode"], "robot_readonly")
                self.assertFalse(state["pc2_connected"])
                self.assertFalse(state["robot"]["online"])
                self.assertIsNone(state["robot"]["battery_percent"])
                self.assertIsNotNone(state["pc2_reason"])

    def test_local_grid_available_at_cold_start_without_pc2(self):
        # 两行极小的 PGM 显式验证自下而上的行序，以及 255 到 HTTP 未知值 -1 的转换。
        (self.root / "floor.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([255, 0, 128, 255]))
        path = self.root / "floor.yaml"
        path.write_text(json.dumps(dict(image="floor.pgm", resolution=0.25, origin=[5,-2,0],
                                        occupied_thresh=0.65, free_thresh=0.25)))
        source = StationSource("http://localhost:8766", path)
        with patch.object(source, "_get", side_effect=OSError):
            _, state = source.state()
        data = source.map_data()
        self.assertFalse(state["pc2_connected"])
        self.assertEqual(data["cells"], [-1, 0, 0, 100])
        self.assertEqual(data["id"], state["map"]["id"])
        self.assertEqual(data["origin_x_m"], 5)

    def test_live_response_ties_both_map_versions_together(self):
        with patch.object(self.source, "_get", return_value=self.live) as read:
            code, result = self.source.pose(self.root, [self.sample])
            read.assert_called_once_with("/api/odin1/pose?map_sha256=" + self.bin_hash)
        self.assertEqual(code, 200)
        self.assertEqual(result["sampleSha256"], self.sample)
        self.assertEqual(result["pose"], self.pose)
        self.assertEqual(result["status"], "live")

    def test_wrong_browser_version_or_unbound_bin_does_not_poll_pc2(self):
        with patch.object(self.source, "_get") as read:
            for versions in (None, [""], ["f"*64], [self.sample, self.sample]):
                _, response = self.source.pose(self.root, versions)
                self.assertEqual(response["status"], "map_mismatch")
            del self.meta["relocMapSha256"]
            (self.root / "metadata.json").write_text(json.dumps(self.meta))
            _, response = self.source.pose(self.root, [self.sample])
            self.assertEqual(response["status"], "map_mismatch")
            read.assert_not_called()

    def test_failed_or_invalid_pc2_samples_hide_the_marker(self):
        for changes, expected in [
                ({"map_sha256": "c" * 64}, "map_mismatch"), ({"status": "waiting"}, "waiting"),
                ({"age_ms": MAX_AGE_MS}, "stale"), ({"max_age_ms": 10000}, "invalid"),
                ({"source": "simulator"}, "invalid"), ({"pose": dict(self.pose, x_m=True)}, "invalid"),
                ({"pose": dict(self.pose, z_m=float("nan"))}, "invalid"),
                ({"pose": dict(self.pose, yaw_rad=4)}, "invalid")]:
            with self.subTest(changes=changes), patch.object(self.source, "_get", return_value={**self.live, **changes}):
                _, result = self.source.pose(self.root, [self.sample])
                self.assertEqual((result["status"], result["pose"]), (expected, None))
        with patch.object(self.source, "_get", side_effect=OSError):
            code, result = self.source.pose(self.root, [self.sample])
            self.assertEqual((code, result["status"], result["pose"]), (503, "disconnected", None))

    def test_slow_transport_cannot_revive_old_signal(self):
        with patch.object(self.source, "_get", return_value=self.live), \
                patch("a2_gateway.station.time.monotonic_ns", side_effect=[0, MAX_AGE_MS * 1_000_000]):
            _, response = self.source.pose(self.root, [self.sample])
            self.assertEqual(response["status"], "stale")
            self.assertIsNone(response["pose"])

    def test_blocks_redirects_oversized_responses_and_unexpected_peer_address(self):
        for url in ["http://localhost/x", "http://u@localhost", "http://localhost:0", "http://localhost:99999",
                    "file:///tmp", "http://localhost?host=elsewhere", "http://localhost#fragment"]:
            with self.subTest(url=url), self.assertRaises(ValueError):
                StationSource(url)
        for code, data in [(302, b"{}"), (200, b" " * 65_537), (200, b"null")]:
            reply = BytesIO(data)
            reply.status = code
            connection = MagicMock()
            connection.getresponse.return_value = reply
            with self.subTest(code=code), patch("a2_gateway.station.HTTPConnection", return_value=connection):
                _, state = self.source.state()
                connection.request.assert_called_once()
                connection.close.assert_called_once()
                self.assertFalse(state["pc2_connected"])
