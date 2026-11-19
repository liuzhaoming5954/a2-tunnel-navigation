import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.observe_odin1_pose import MAX_AGE_NS, MapSession, Pairs, Sample, map_pose


class SensorPoseTest(unittest.TestCase):
    def test_non_identity_map_transform_and_sensor_forward(self):
        # map 原点在 odom 的 (10,20,1)，旋转 90°。传感器相对 map 的航向应为 30°。
        def z_rotation(degrees):
            a = math.radians(degrees)/2
            return (0., 0., math.sin(a), math.cos(a))
        tf = Sample(0, (10., 20., 1.), z_rotation(90))
        odom = Sample(0, (8., 23., 5.), z_rotation(120))
        position, yaw = map_pose(odom, tf)
        for got, expected in zip(position, (3, 2, 4)):
            self.assertAlmostEqual(got, expected)
        self.assertAlmostEqual(yaw, math.pi/6)
        with self.assertRaises(ValueError):
            map_pose(Sample(0, (0, 0, 0), (0., 2**-.5, 0., 2**-.5)),
                     Sample(0, (0, 0, 0), (0., 0., 0., 1.)))

    def test_exact_pair_only_once_and_receipt_does_not_refresh(self):
        pair = Pairs()
        p = SimpleNamespace(x=0, y=0, z=0)
        q = SimpleNamespace(x=0, y=0, z=0, w=1)

        def add(series, stamp, received):
            header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=stamp))
            with patch("scripts.observe_odin1_pose.time.monotonic_ns", return_value=received):
                pair.add(series, header, p, q)
        add(pair.odom, 10, 1)
        add(pair.tf, 11, 2)
        self.assertIsNone(pair.latest(2))
        add(pair.odom, 10, 4)
        add(pair.tf, 10, 5)
        self.assertEqual(pair.latest(6)["received_monotonic_ns"], 1)
        add(pair.odom, 10, 7)
        add(pair.tf, 10, 7)
        self.assertIsNone(pair.latest(7))
        add(pair.odom, 11, MAX_AGE_NS + 3)
        self.assertIsNone(pair.latest(MAX_AGE_NS + 3))

    def test_session_checks_digest_mode_success_and_subsequent_switch(self):
        h = "1"*32
        path = Path("/tmp/fixture.bin")
        lines = [f"[service]: set_reloc_map: transferring [{path}]",
                 f"[send_file_handler]: file md5: {h}.",
                 f"[map_transfer]: Relocalization map swap completed: {path} (SLAM may restart asynchronously)",
                 "[service]: set_slam_mode: map_mode set to 2 (SLAM may restart asynchronously)",
                 "[odom]: relocalization success!"]
        session = MapSession(path, h)
        for line in lines[:-1]:
            session.line(line)
            self.assertNotEqual(session.status, "live")
        session.line(lines[-1])
        self.assertEqual(session.status, "live")
        session.line("[service]: set_slam_mode: map_mode set to 0 (SLAM may restart asynchronously)")
        self.assertEqual(session.status, "waiting")
        session.line("[service]: set_reloc_map: transferring [/tmp/other.bin]")
        self.assertEqual(session.status, "map_mismatch")
        for line in lines:
            session.line(line.replace(h, "2"*32))
        self.assertEqual(session.status, "map_mismatch")
