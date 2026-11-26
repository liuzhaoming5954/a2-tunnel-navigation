import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

from scripts.publish_a2_localization import (
    BodyPose, ScanLog, load_anchor, load_trinary_grid, local_body_pose, map_matches, process_identity,
)


IDENTITY = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


class LocalizationDisplayTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.map = self.write_json("map.yaml", {"image": "map.pgm", "resolution": .2,
            "origin": [-1, 2, 0], "occupied_thresh": .65, "free_thresh": .196})
        image = self.root / "map.pgm"
        image.write_bytes(b"P5\n2 1\n255\n\x00\xff")
        self.manifest = self.write_json("manifest.json", {"source_sha256": {"database": "matching"},
            "output_sha256": {name.name: hashlib.sha256(name.read_bytes()).hexdigest() for name in (image, self.map)}})
        self.extrinsics = self.write_json("extrinsics.json", {"source": "synthetic test", "T_base_from_lidar": IDENTITY})
        self.report = {"map_sha256": "matching", "status": "diagnostic_anchored_stream_observed",
            "alignment": {"accepted": True, "decision": "map_handoff_verified", "T_map_from_fresh_odom": IDENTITY},
            "poses_after_alignment": [
                {"sensor_reference_ns": reference, "sensor_scan_start_ns": reference - 50,
                 "T_map_from_front_lidar_reference": IDENTITY}
                for reference in (100, 200)
            ]}

    def write_json(self, name, content):
        path = self.root / name
        path.write_text(json.dumps(content), encoding="utf-8")
        return path

    def anchor(self):
        return load_anchor(self.write_json("report.json", self.report), self.manifest, self.map, self.extrinsics)

    def test_only_accepted_anchor_for_exact_map(self):
        _, _, origin, pairs = self.anchor()
        self.assertEqual(origin, BodyPose(0, 0, 0, 100))
        self.assertEqual(pairs, [(100, 50), (200, 150)])
        self.report["alignment"]["accepted"] = False
        with self.assertRaisesRegex(ValueError, "did not pass"):
            self.anchor()
        self.report["alignment"]["accepted"] = True
        (self.root / "map.pgm").write_bytes(b"P5\n2 1\n255\n\xff\x00")
        with self.assertRaisesRegex(ValueError, "manifest do not match"):
            self.anchor()

    def test_scan_log_only_accepts_published_pairs_and_exact_process_file(self):
        log = self.root / "dlio.log"
        other = self.root / "other.log"
        other.touch()
        with log.open("w") as output:
            process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(10)"], stdout=output)
            try:
                process_identity(process.pid, log)
                with self.assertRaisesRegex(ValueError, "supplied session log"):
                    process_identity(process.pid, other)
            finally:
                process.terminate()
                process.wait(timeout=2)
        log.write_text("A2_SCAN_SYNC input_ns=50 reference_ns=100 result=published\n"
                       "A2_SCAN_SYNC input_ns=150 reference_ns=200 result=imu_warmup\n"
                       "A2_SCAN_SYNC input_ns=250 reference_ns=300 result=pub", encoding="utf-8")
        with log.open() as stream:
            ledger = ScanLog(stream)
            ledger.read()
            self.assertEqual(ledger.references, {100: 50})
            with log.open("a") as append:
                append.write("lished\n")
            ledger.read()
            self.assertEqual(ledger.references, {100: 50, 300: 250})

    def test_display_map_content_and_relative_heading(self):
        expected = load_trinary_grid(self.map)
        info = SimpleNamespace(width=2, height=1, resolution=.20000000298,
            origin=SimpleNamespace(position=SimpleNamespace(x=-1, y=2, z=0),
                                   orientation=SimpleNamespace(x=0, y=0, z=0, w=-1)))
        message = SimpleNamespace(info=info, header=SimpleNamespace(frame_id="map"), data=[100, 0])
        self.assertTrue(map_matches(message, expected))
        message.data = [0, 100]
        self.assertFalse(map_matches(message, expected))
        result = local_body_pose(BodyPose(1, 2, math.pi / 2, 100), BodyPose(0, 4, -math.pi, 200))
        self.assertAlmostEqual(result.x_m, 2)
        self.assertAlmostEqual(result.y_m, 1)
        self.assertAlmostEqual(result.yaw_rad, math.pi / 2)
        self.assertEqual(result.stamp_ns, 200)


if __name__ == "__main__":
    unittest.main()
