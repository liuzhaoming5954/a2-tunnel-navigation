import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest

from a2_gateway.odin1_pose import BOOT_ID_FILE, MAX_AGE_MS, odin1_pose_snapshot


class Odin1PoseTest(unittest.TestCase):
    def setUp(self):
        # 使用单独的合成地图绑定及本机接收时间，任何测试都不连接设备。
        work = TemporaryDirectory()
        self.addCleanup(work.cleanup)
        self.root = Path(work.name)
        self.file = self.root / "pose.json"
        self.metadata = {"sampleSha256": "1" * 64, "relocMapSha256": "2" * 64}
        (self.root / "metadata.json").write_text(json.dumps(self.metadata))
        self.live = {"status": "live", "source": "odin1_relocalization", "map_sha256": "2" * 64,
                     "boot_id": BOOT_ID_FILE.read_text().strip(), "received_monotonic_ns": time.monotonic_ns(),
                     "stamp_ns": 100, "frame": "map", "child_frame": "imu",
                     "position_m": [1.25, -2.5, 0.4], "yaw_rad": -1.0}

    def read(self, changes=None):
        self.file.write_text(json.dumps({**self.live, **(changes or {})}))
        return odin1_pose_snapshot(self.root, self.file, [self.metadata["sampleSha256"]])

    def test_valid_current_sensor_pose(self):
        value = self.read()
        self.assertEqual(value["status"], "live")
        self.assertEqual(value["pose"], dict(x_m=1.25, y_m=-2.5, z_m=0.4, yaw_rad=-1.0))
        self.assertEqual(value["sampleSha256"], self.metadata["sampleSha256"])
        self.assertGreaterEqual(value["age_ms"], 0)
        self.assertLess(value["age_ms"], MAX_AGE_MS)

    def test_invalid_or_stale_samples_hide_the_pose(self):
        # 即使来自同一份状态文件，图上也只能显示精确匹配的 map→imu、新鲜位姿。
        for changes, expected in [
            ({"map_sha256": "0" * 64}, "map_mismatch"),
            ({"received_monotonic_ns": time.monotonic_ns() - 2_000_000_000}, "stale"),
            ({"boot_id": "previous-boot"}, "stale"),
            ({"received_monotonic_ns": time.monotonic_ns() + 30_000_000_000}, "invalid"),
            ({"stamp_ns": 0}, "invalid"),
            ({"frame": "odom"}, "invalid"), ({"child_frame": "base_link"}, "invalid"),
            ({"source": "simulator"}, "invalid"), ({"position_m": [1, True, 3]}, "invalid"),
            ({"yaw_rad": float("nan")}, "invalid"), ({"yaw_rad": 20}, "invalid"),
            ({"status": "waiting"}, "waiting"), ({"status": "stale"}, "stale"),
        ]:
            with self.subTest(changes=changes):
                state = self.read(changes)
                self.assertEqual(state["status"], expected)
                self.assertIsNone(state["pose"])

    def test_replaced_cloud_or_unbound_legacy_map(self):
        self.read()
        for versions in [["old"], [""], ["1" * 64, "1" * 64]]:
            state = odin1_pose_snapshot(self.root, self.file, versions)
            self.assertEqual(state["status"], "map_mismatch")
            self.assertIsNone(state["pose"])
        del self.metadata["relocMapSha256"]
        (self.root / "metadata.json").write_text(json.dumps(self.metadata))
        self.assertEqual(self.read()["status"], "map_mismatch")

    def test_unconfigured_missing_stopped_or_corrupt_observer(self):
        for path in [None, self.file]:
            self.assertEqual(odin1_pose_snapshot(self.root, path, None)["status"], "waiting")
        self.file.write_text(json.dumps({"source": "odin1_relocalization", "status": "waiting"}))
        self.assertIsNone(odin1_pose_snapshot(self.root, self.file, None)["pose"])
        self.file.write_text("{")
        self.assertEqual(odin1_pose_snapshot(self.root, self.file, None)["status"], "invalid")
