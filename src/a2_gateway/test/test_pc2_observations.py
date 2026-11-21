"""验证机载数据接口仅需观测文件与单项平台订阅，全部输入为本地夹具。"""

import argparse
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import MagicMock, patch

from a2_gateway.odin1_pose import BOOT_ID_FILE, MAX_AGE_MS, pc2_pose_snapshot
from a2_gateway.ros_telemetry import RosTelemetry, add_ros_arguments


class PC2ObservationsTest(unittest.TestCase):
    def test_pose_uses_observed_bin_without_cloud_or_metadata(self):
        # 该目录刻意只放采集器快照，设备载图摘要由观测器在收到重定位回执后写入。
        with TemporaryDirectory() as directory:
            pose_file = Path(directory) / "pose.json"
            digest = "a" * 64
            observation = {
                "source": "odin1_relocalization", "status": "live", "frame": "map",
                "child_frame": "imu", "map_sha256": digest, "stamp_ns": 2020,
                "boot_id": BOOT_ID_FILE.read_text().strip(), "received_monotonic_ns": time.monotonic_ns(),
                "position_m": [1.0, -2.0, 0.5], "yaw_rad": 0.25,
            }
            self.assertEqual(pc2_pose_snapshot(pose_file, digest)["status"], "waiting")
            pose_file.write_text(json.dumps(observation))
            state = pc2_pose_snapshot(pose_file, digest)
            self.assertEqual(state["status"], "live")
            self.assertEqual(state["map_sha256"], digest)
            self.assertEqual(state["pose"], dict(x_m=1.0, y_m=-2.0, z_m=0.5, yaw_rad=0.25))
            self.assertNotIn("sampleSha256", state)
            for requested in ("b" * 64, None, "", "a"):
                with self.subTest(requested=requested):
                    state = pc2_pose_snapshot(pose_file, requested)
                    self.assertEqual(state["status"], "map_mismatch")
                    self.assertEqual(state["map_sha256"], digest)
                    self.assertIsNone(state["pose"])
            # 以 PC2 的单调时钟判断失效边界，不依赖上位机墙钟。
            with patch("a2_gateway.odin1_pose.time.monotonic_ns",
                       return_value=observation["received_monotonic_ns"] + MAX_AGE_MS * 1_000_000):
                state = pc2_pose_snapshot(pose_file, digest)
                self.assertEqual(state["status"], "stale")
                self.assertIsNone(state["pose"])

    def test_platform_only_skips_navigation_imports_and_subscriptions(self):
        # 以轻量替身记录实际订阅，禁止导入导航消息和 TF；全程无 ROS 节点或外部连接。
        parser = argparse.ArgumentParser()
        add_ros_arguments(parser)
        args = parser.parse_args([])
        names = ("rclpy", "rclpy.context", "rclpy.executors", "rclpy.parameter",
                 "rclpy.qos", "rclpy.signals", "a2_interfaces.msg")
        modules = {name: MagicMock() for name in names}
        modules.update(dict.fromkeys(("nav_msgs.msg", "tf2_ros", "rclpy.time")))
        node = modules["rclpy"].create_node.return_value
        context = modules["rclpy.context"].Context.return_value
        with patch.dict(sys.modules, modules), patch("a2_gateway.ros_telemetry.Thread") as thread:
            telemetry = RosTelemetry(args, platform_only=True)
            self.assertEqual(node.create_subscription.call_count, 1)
            self.assertEqual(node.create_subscription.call_args.args[:2],
                             (modules["a2_interfaces.msg"].PlatformState, args.platform_topic))
            self.assertIsNone(telemetry.map_data())
            # 模拟 ROS 时钟回跳，确认没有创建 TF 缓冲区的只读路径也能正常继续及关闭。
            telemetry._clock_ns = 1000
            node.get_clock.return_value.now.return_value.nanoseconds = 900
            context.ok.side_effect = [True, False]
            telemetry._run()
            self.assertEqual(telemetry._clock_ns, 900)
            self.assertFalse(telemetry.snapshot()["robot"]["online"])
            node.destroy_node.assert_called_once()
            context.try_shutdown.assert_called_once()
            telemetry.close()
            thread.return_value.join.assert_called_once()


if __name__ == "__main__":
    unittest.main()
