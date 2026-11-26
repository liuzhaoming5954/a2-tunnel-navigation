"""验证机载只读观测、平台遥测及各来源的时效，全部输入为本地夹具。"""

import argparse
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import time
from types import SimpleNamespace as Fields
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
            subscriptions = [call.args[:2] for call in node.create_subscription.call_args_list]
            self.assertEqual(subscriptions, [
                (modules["a2_interfaces.msg"].PlatformState, args.platform_topic),
                (modules["a2_interfaces.msg"].PlatformDetails, args.platform_details_topic),
            ])
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

    def test_live_details_bind_sources_and_expire_each_sensor_independently(self):
        # 真实回调与快照入口使用同一组假时钟，120 毫秒的旧电池不会被新 IMU 续期。
        parser = argparse.ArgumentParser()
        add_ros_arguments(parser)
        names = ("rclpy", "rclpy.context", "rclpy.executors", "rclpy.parameter",
                 "rclpy.qos", "rclpy.signals", "a2_interfaces.msg")
        modules = {name: MagicMock() for name in names}
        steady = 10_000_000_000
        ros = 3_000_000_000
        with patch.dict(sys.modules, modules), patch("a2_gateway.ros_telemetry.Thread"), \
                patch("a2_gateway.ros_telemetry.time.monotonic_ns", return_value=steady) as clock:
            telemetry = RosTelemetry(parser.parse_args([]), platform_only=True)
            self.addCleanup(telemetry.close)
            telemetry._node.get_clock.return_value.now.return_value.nanoseconds = ros
            telemetry._clock_ns = ros
            header = Fields(stamp=Fields(sec=3, nanosec=0), frame_id="base_link")
            identity = dict(schema_version=1, source="unitree_sdk2", source_session_id="one",
                            simulated=False, robot_id="a2", header=header)
            body = Fields(**identity, mode_valid=False, battery_valid=False, velocity_valid=False, age_ms=0,
                          reason="NO_DATA", body_velocity=Fields(linear=Fields(x=0), angular=Fields(z=0)))
            telemetry._read_platform(body)
            primary = Fields(received_mono_ns=steady - 120_000_000, soc_percent=0, soh_percent=101,
                             cycle_count=0, charging_state="unknown")
            secondary = Fields(received_mono_ns=steady + 1, soc_percent=90, soh_percent=99,
                               cycle_count=3, charging_state="charging")
            detail_identity = {**identity, "header": Fields(stamp=header.stamp, frame_id="")}
            msg = Fields(**detail_identity, primary_battery=primary, secondary_battery=secondary, max_age_ms=500,
                         velocity_received_mono_ns=steady, velocity_mps=[1., 0., float("nan")],
                         yaw_rate_radps=-0.5, attitude_received_mono_ns=steady, rpy_rad=[0., -1., 2.])
            telemetry._read_details(msg)
            robot = telemetry.snapshot()["robot"]
            self.assertFalse(robot["online"])
            self.assertEqual(robot["batteries"]["primary"], {
                "soc_percent": 0, "soh_percent": None, "cycle_count": 0,
                "charging": None, "age_ms": 120, "max_age_ms": 500})
            self.assertIsNone(robot["batteries"]["secondary"])
            self.assertEqual(robot["velocity"]["y_mps"], 0.)
            self.assertIsNone(robot["velocity"]["z_mps"])
            self.assertEqual(robot["attitude"]["pitch_rad"], -1.)
            msg.source_session_id = "other"
            msg.header = Fields(stamp=Fields(sec=3, nanosec=2), frame_id="")
            telemetry._node.get_clock.return_value.now.return_value.nanoseconds = ros + 2
            msg.primary_battery.soc_percent = 99
            telemetry._read_details(msg)
            self.assertEqual(telemetry.snapshot()["robot"]["batteries"]["primary"]["soc_percent"], 0)
            clock.return_value = steady + 380_000_000
            robot = telemetry.snapshot()["robot"]
            self.assertIsNone(robot["batteries"]["primary"])
            self.assertIsNone(robot["batteries"]["secondary"])
            self.assertIsNotNone(robot["attitude"])
            clock.return_value = steady + 500_000_000
            self.assertIsNone(telemetry.snapshot()["robot"]["attitude"])

    def _telemetry_for_details_test(self, *, details_supported=True):
        parser = argparse.ArgumentParser()
        add_ros_arguments(parser)
        names = ("rclpy", "rclpy.context", "rclpy.executors", "rclpy.parameter",
                 "rclpy.qos", "rclpy.signals", "a2_interfaces.msg")
        modules = {name: MagicMock() for name in names}
        if not details_supported:
            # 老接口包只能导入 PlatformState，保留基本平台观测订阅。
            modules["a2_interfaces.msg"] = Fields(PlatformState=MagicMock())
        with patch.dict(sys.modules, modules), patch("a2_gateway.ros_telemetry.Thread"):
            telemetry = RosTelemetry(parser.parse_args([]), platform_only=True)
        self.addCleanup(telemetry.close)
        return telemetry

    def _platform_without_sdk_data(self, *, source_session_id="one", stamp_ns=0, reason="NO_DATA"):
        return Fields(schema_version=1, source="unitree_sdk2", simulated=False, robot_id="a2",
                      source_session_id=source_session_id, header=Fields(stamp=Fields(sec=3, nanosec=stamp_ns)),
                      mode_valid=False, battery_valid=False, velocity_valid=False, age_ms=0, valid=False,
                      reason=reason, body_velocity=Fields(linear=Fields(x=0), angular=Fields(z=0)))

    def test_gateway_keeps_basic_telemetry_with_older_ros_interfaces(self):
        telemetry = self._telemetry_for_details_test(details_supported=False)
        topics = [call.args[1] for call in telemetry._node.create_subscription.call_args_list]
        self.assertEqual(topics, ["/a2/platform/state"])
        self.assertIsNone(telemetry.snapshot()["robot"]["attitude"])

    def test_imu_rate_uses_advancing_sample_time_and_signed_wrapped_delta(self):
        telemetry = self._telemetry_for_details_test()
        steady = 10_000_000_000
        ms = 1_000_000

        def update(clock, elapsed, sample, yaw_deg, *, session="one"):
            clock.return_value = steady + elapsed * ms
            message = self._platform_without_sdk_data(source_session_id=session, stamp_ns=elapsed * ms)
            telemetry._clock_ns = 3_000_000_000 + elapsed * ms
            telemetry._node.get_clock.return_value.now.return_value.nanoseconds = telemetry._clock_ns
            telemetry._read_platform(message)
            message.primary_battery = message.secondary_battery = Fields(received_mono_ns=0, soc_percent=-1,
                                                                          soh_percent=-1, cycle_count=-1,
                                                                          charging_state="unknown")
            message.max_age_ms = 500
            message.velocity_received_mono_ns = steady + elapsed * ms
            message.yaw_rate_radps = 0.0
            message.velocity_mps = [0., 0., 0.]
            message.attitude_received_mono_ns = steady + sample * ms
            message.rpy_rad = [0., 0., math.radians(yaw_deg)]
            telemetry._read_details(message)
            return telemetry.snapshot()["robot"]

        with patch("a2_gateway.ros_telemetry.time.monotonic_ns") as clock:
            self.assertIsNone(update(clock, 0, 0, 179)["attitude"]["yaw_rate_radps"])
            left = update(clock, 100, 100, -179)
            self.assertEqual(left["velocity"]["yaw_radps"], 0.)
            self.assertAlmostEqual(math.degrees(left["attitude"]["yaw_rate_radps"]), 20.)
            repeated = update(clock, 190, 100, -179)["attitude"]
            self.assertEqual(repeated["age_ms"], 90)
            self.assertAlmostEqual(math.degrees(repeated["yaw_rate_radps"]), 20.)
            self.assertAlmostEqual(math.degrees(update(clock, 200, 200, 179)["attitude"]["yaw_rate_radps"]), -20.)
            self.assertEqual(update(clock, 300, 300, 179)["attitude"]["yaw_rate_radps"], 0.)
            clock.return_value = steady + 550 * ms
            self.assertIsNone(telemetry.snapshot()["robot"]["attitude"]["yaw_rate_radps"])
            self.assertIsNone(update(clock, 600, 600, 170)["attitude"]["yaw_rate_radps"])
            self.assertIsNone(update(clock, 620, 620, 169)["attitude"]["yaw_rate_radps"])
            self.assertAlmostEqual(math.degrees(update(clock, 720, 720, 168)["attitude"]["yaw_rate_radps"]), -10.)
            self.assertIsNone(update(clock, 730, 700, 160)["attitude"]["yaw_rate_radps"])
            self.assertAlmostEqual(math.degrees(update(clock, 820, 820, 167)["attitude"]["yaw_rate_radps"]), -10.)
            self.assertIsNone(update(clock, 900, 900, 5, session="two")["attitude"]["yaw_rate_radps"])
            self.assertIsNone(update(clock, 950, 950, float("nan"), session="two")["attitude"]["yaw_rate_radps"])
            self.assertIsNone(update(clock, 980, 980, 4, session="two")["attitude"]["yaw_rate_radps"])

    def test_ros_fixed_double_arrays_reach_http_as_numbers(self):
        from a2_gateway.platform_details import fresh_details, observation_groups

        # 模拟 ROS 固定数组返回的非内建浮点标量；测试本身无需安装 ROS 或 NumPy。
        class IdlDouble(float):
            pass

        missing = Fields(received_mono_ns=0, soc_percent=-1, soh_percent=-1,
                         cycle_count=-1, charging_state="unknown")
        msg = Fields(primary_battery=missing, secondary_battery=missing,
                     velocity_received_mono_ns=2, velocity_mps=tuple(map(IdlDouble, [0.1, -0.2, 0.])),
                     yaw_rate_radps=0., attitude_received_mono_ns=2,
                     rpy_rad=tuple(map(IdlDouble, [0.3, -0.4, 2.9])))
        groups = fresh_details(observation_groups(msg, 2, 500))
        payload = json.loads(json.dumps(groups, allow_nan=False))
        self.assertEqual([payload["velocity"][key] for key in ("x_mps", "y_mps", "z_mps")], [0.1, -0.2, 0.])
        self.assertEqual([payload["attitude"][key] for key in ("roll_rad", "pitch_rad", "yaw_rad")],
                         [0.3, -0.4, 2.9])


if __name__ == "__main__":
    unittest.main()
