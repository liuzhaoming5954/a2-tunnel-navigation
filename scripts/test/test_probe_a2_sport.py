"""验证 A2 探针的只读查询、遥控数据布局及零帧判定。"""

from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import probe_a2_sport as probe


class ProbeTests(unittest.TestCase):
    def test_embedded_remote_decodes_four_axes_and_keys(self):
        data = bytearray(40)
        struct.pack_into("<Hfffff", data, 2, 0x8421, 0.25, -0.5, 1, 99.0, -0.75)
        sample = probe.remote_sample("rt/lowstate", SimpleNamespace(wireless_remote=data))
        self.assertEqual(sample["keys"], 0x8421)
        self.assertEqual(sample["axes"], {"lx": 0.25, "rx": -0.5, "ry": 1, "ly": -0.75})

    def test_static_zero_does_not_prove_remote_and_inactive_source_does_not_mask_input(self):
        feedback = probe.Feedback()
        neutral = SimpleNamespace(lx=0.0, ly=0.0, rx=0.0, ry=0.0, keys=0)
        raw = "rt/wirelesscontroller_unprocessed"
        feedback._receive("rt/lowstate", SimpleNamespace(wireless_remote=bytes(40)))
        feedback._receive(raw, neutral)
        self.assertFalse(feedback.snapshot()["remote"]["observed_input"])
        feedback._receive(raw, SimpleNamespace(lx=0.8, ly=0.0, rx=0.0, ry=0.0, keys=0))
        feedback._receive(raw, neutral)
        feedback._receive("rt/wirelesscontroller", neutral)
        remote = feedback.snapshot()["remote"]
        self.assertEqual(remote["topic"], raw)
        self.assertEqual((remote["activity_count"], remote["neutral_count"]), (1, 2))
        self.assertTrue(remote["observed_input"])
        self.assertIsNotNone(remote["last_activity_monotonic_s"])

    def test_truncated_remote_and_invalid_axis_are_excluded(self):
        feedback = probe.Feedback()
        feedback._receive("rt/lf/lowstate", SimpleNamespace(wireless_remote=bytes(24)))
        feedback._receive("rt/wirelesscontroller", SimpleNamespace(lx=float("nan"), ly=0, rx=0, ry=0, keys=0))
        snapshot = feedback.snapshot()
        self.assertIsNone(snapshot["remote"])
        self.assertEqual(set(snapshot["errors"]), {"rt/lf/lowstate", "rt/wirelesscontroller"})

    def test_hg_imu_preserves_original_order_and_remote_signature(self):
        sensor = SimpleNamespace(quaternion=[0.5, -0.5, 0.5, -0.5], gyroscope=[0.12, -0.23, 0.34])
        topic = "rt/lf/lowstate"
        feedback = probe.Feedback(topic)
        raw = bytearray(40)
        struct.pack_into("<H", raw, 2, 0x1200)
        feedback._receive(topic, SimpleNamespace(wireless_remote=raw, imu_state=sensor))
        feedback._receive(topic, SimpleNamespace(wireless_remote=bytes(40), imu_state=sensor))
        record = feedback.snapshot()["topics"][topic]
        self.assertEqual(record["imu"], {"valid": True, "quaternion_wxyz": sensor.quaternion,
                                         "gyroscope_rad_s": sensor.gyroscope})
        self.assertEqual(record["keys"], 0)
        self.assertEqual(record["last_activity_keys"], 0x1200)
        self.assertEqual(record["last_activity_axes"], dict(lx=0.0, ly=0.0, rx=0.0, ry=0.0))

    def test_activity_edge_is_first_active_sample_and_persists_after_release(self):
        topic = "rt/lf/lowstate"
        feedback = probe.Feedback(topic)

        def submit(keys, lx=0.0):
            data = bytearray(40)
            struct.pack_into("<Hf", data, 2, keys, lx)
            feedback._receive(topic, SimpleNamespace(wireless_remote=data))

        # 在长按途中启动监听，既没有见过中立，也不能推断此前的按键边沿。
        with patch.object(probe.time, "monotonic", return_value=1.0):
            submit(0x0010)
            self.assertIsNone(feedback.snapshot()["remote"]["last_activity_start_monotonic_s"])
        with patch.object(probe.time, "monotonic", return_value=2.0):
            submit(0)
        with patch.object(probe.time, "monotonic", return_value=3.0):
            submit(0x0008, 0.25)
        with patch.object(probe.time, "monotonic", return_value=4.0):
            submit(0x000C, 0.5)
        with patch.object(probe.time, "monotonic", return_value=5.0):
            submit(0)
            remote = feedback.snapshot()["remote"]
            self.assertEqual(remote["last_activity_start_monotonic_s"], 3.0)
            self.assertEqual(remote["last_activity_start_keys"], 0x0008)
            self.assertEqual(remote["last_activity_start_axes"]["lx"], 0.25)
            self.assertEqual(remote["last_activity_monotonic_s"], 4.0)
            self.assertEqual(remote["last_activity_keys"], 0x000C)
        with patch.object(probe.time, "monotonic", return_value=6.0):
            submit(0x0200)
            remote = feedback.snapshot()["remote"]
            self.assertEqual(remote["last_activity_start_monotonic_s"], 6.0)
            self.assertEqual(remote["last_activity_start_keys"], 0x0200)
            self.assertEqual(remote["last_activity_start_axes"]["lx"], 0.0)

    def test_missing_or_corrupt_imu_keeps_remote_and_never_reuses_old_pose(self):
        topic = "rt/lf/lowstate"
        feedback = probe.Feedback(topic)
        sensors = [
            None,
            SimpleNamespace(quaternion=[0, 0, 0, 0], gyroscope=[1, 2, 3]),
            SimpleNamespace(quaternion=[float("inf"), 0, 0, 0], gyroscope=[1, 2, 3]),
            SimpleNamespace(quaternion=[1, 0, 0, 0], gyroscope=[0, float("nan"), 0]),
            SimpleNamespace(quaternion=[1, 0, 0], gyroscope=[0, 0, 0]),
        ]
        good = SimpleNamespace(quaternion=[1, 0, 0, 0], gyroscope=[0, 0, 0])
        for invalid in sensors:
            with self.subTest(sensor=invalid):
                feedback._receive(topic, SimpleNamespace(wireless_remote=bytes(40), imu_state=good))
                feedback._receive(topic, SimpleNamespace(wireless_remote=bytes(40), imu_state=invalid))
                record = feedback.snapshot()["remote"]
                self.assertEqual(record["keys"], 0)
                self.assertFalse(record["imu"]["valid"])
                self.assertIsNone(record["imu"]["quaternion_wxyz"])
                self.assertIsNone(record["imu"]["gyroscope_rad_s"])

    def test_sport_falls_back_to_latest_stream(self):
        message = SimpleNamespace(mode=3, gait_type=0, error_code=0, velocity=[0, 0, 0],
                                  yaw_speed=0, position=[1, 2, 3], imu_state=SimpleNamespace(rpy=[0, 0, 1]))
        feedback = probe.Feedback()
        with patch.object(probe.time, "monotonic", side_effect=[2.0, 2.5, 3.0]):
            for topic in probe.SPORT_TOPICS:
                feedback._receive(topic, message)
            observed = feedback.snapshot()
        self.assertEqual(observed["sport"]["topic"], "rt/lf/sportmodestate")
        self.assertEqual(observed["sport"]["age_s"], 0.5)
        self.assertEqual(observed["sport"]["position_m"], [1, 2, 3])

    def test_read_status_continues_after_version_timeout(self):
        def timeout():
            raise TimeoutError("模拟客户端超时")

        def state(values):
            values["fsm_name"] = "NORMAL_WALK"
            return 0

        sport = SimpleNamespace(GetServerApiVersion=timeout, GetState=state)
        switcher = SimpleNamespace(CheckMode=lambda: (0, {"name": "ai"}))
        result = probe.read_status(sport, switcher)
        self.assertIsNone(result["server_version"]["code"])
        self.assertEqual(result["sport_state"]["value"]["fsm_name"], "NORMAL_WALK")
        self.assertEqual(result["motion_mode"]["value"], {"name": "ai"})


if __name__ == "__main__":
    unittest.main()
