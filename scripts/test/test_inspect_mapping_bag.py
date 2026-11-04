"""Structural checks run without ROS; one bag round-trip runs when ROS is sourced."""

import copy
import hashlib
import importlib.util
import json
import math
import struct
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS


SCRIPT = Path(__file__).resolve().parents[1] / "inspect_mapping_bag.py"
SPEC = importlib.util.spec_from_file_location("inspect_mapping_bag", SCRIPT)
bag_check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bag_check)

CONFIG = {
    "schema_version": 1, "topics": {"front_points": "/front", "imu": "/imu"},
    "point_time": {"field": "time", "unit": "s"}, "storage_id": "sqlite3",
}
TYPES = {"/front": "sensor_msgs/msg/PointCloud2", "/imu": "sensor_msgs/msg/Imu"}


def header(sec=1, frame="lidar"):
    return NS(stamp=NS(sec=sec, nanosec=0), frame_id=frame)


def cloud(sec=1):
    return NS(header=header(sec), fields=[NS(name=name, offset=index * 4, datatype=7, count=1)
                                        for index, name in enumerate(("x", "y", "z", "time"))],
              width=2, height=1, point_step=16, row_step=32, data=bytes(32), is_bigendian=False)


def imu(sec=1):
    return NS(header=header(sec, "imu"), angular_velocity=NS(x=0.0, y=0.0, z=0.0),
              linear_acceleration=NS(x=0.0, y=0.0, z=9.81),
              angular_velocity_covariance=[0.0] * 9, linear_acceleration_covariance=[0.0] * 9,
              orientation_covariance=[-1.0] + [0.0] * 8)


class StructuralTests(unittest.TestCase):
    def checks(self, config=None):
        return bag_check.BagChecks(copy.deepcopy(config or CONFIG), TYPES)

    def populate(self, checks):
        for second in (1, 2):
            checks.consume("/front", cloud(second), second * 1_000_000_000)
            checks.consume("/imu", imu(second), second * 1_000_000_000)
        return checks

    def test_actual_samples_pass_with_separate_tf_warning_and_six_axis_imu(self):
        report = self.populate(self.checks()).report()
        self.assertEqual(report["issues"], [])
        self.assertEqual(report["topics"]["/front"]["message_count"], 2)
        self.assertEqual(report["topics"]["/imu"]["imu_axis_min_max"]["linear_acceleration"][2], [9.81, 9.81])
        self.assertEqual({item["topic"] for item in report["warnings"]}, {"/tf", "/tf_static"})

    def test_metadata_alone_does_not_pass(self):
        report = self.checks().report()
        self.assertEqual({item["topic"] for item in report["issues"] if item["code"] == "no_messages"},
                         {"/front", "/imu"})

    def test_missing_time_and_truncated_payload_report_issues(self):
        checks = self.populate(self.checks())
        message = cloud(3)
        message.fields.pop()
        message.data = bytes(16)
        checks.consume("/front", message, 3_000_000_000)
        codes = {item["code"] for item in checks.report()["issues"]}
        self.assertIn("missing_point_field", codes)
        self.assertIn("invalid_cloud_layout", codes)

    def test_unknown_point_time_is_an_explicit_unresolved_issue(self):
        config = copy.deepcopy(CONFIG)
        config["point_time"] = {"field": "", "unit": ""}
        report = self.populate(self.checks(config)).report()
        self.assertEqual([item["code"] for item in report["issues"]], ["point_time_unresolved"])

    def test_nonfinite_imu_and_unavailable_measurement_are_rejected(self):
        checks = self.populate(self.checks())
        message = imu(3)
        message.linear_acceleration.x = math.nan
        message.angular_velocity_covariance[0] = -1.0
        checks.consume("/imu", message, 3_000_000_000)
        codes = {item["code"] for item in checks.report()["issues"]}
        self.assertIn("nonfinite_imu", codes)
        self.assertIn("imu_channel_unavailable", codes)

    def test_header_order_checked_independently_of_bag_order(self):
        checks = self.populate(self.checks())
        checks.consume("/front", cloud(1), 3_000_000_000)
        report = checks.report()
        self.assertEqual(report["topics"]["/front"]["header_time"]["backwards_count"], 1)
        self.assertEqual(report["topics"]["/front"]["bag_receive_time"]["backwards_count"], 0)
        self.assertIn("backwards_header_stamp", {item["code"] for item in report["issues"]})

    def test_tf_edges_handle_same_stamp_on_distinct_edges_and_zero_static_time(self):
        checks = self.populate(self.checks())
        checks.topic_types["/tf_static"] = "tf2_msgs/msg/TFMessage"
        transforms = [NS(header=header(0, "base_link"), child_frame_id=child,
                         transform=NS(translation=NS(x=0.0, y=0.0, z=0.0),
                                      rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))
                      for child in ("lidar", "imu")]
        checks.consume("/tf_static", NS(transforms=transforms), 1_000_000_000)
        report = checks.report()
        self.assertEqual(report["issues"], [])
        self.assertEqual(len(report["tf_edges"]), 2)
        self.assertTrue(all(edge["time"]["zero_count"] == 1 for edge in report["tf_edges"]))

    def test_tf_backwards_checked_per_edge(self):
        checks = self.populate(self.checks())
        checks.topic_types["/tf"] = "tf2_msgs/msg/TFMessage"
        for second in (2, 1):
            transform = NS(header=header(second, "odom"), child_frame_id="base_link",
                           transform=NS(translation=NS(x=0.0, y=0.0, z=0.0),
                                        rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)))
            checks.consume("/tf", NS(transforms=[transform]), 3_000_000_000)
        self.assertIn("backwards_tf_stamp", {item["code"] for item in checks.report()["issues"]})

    def test_wrong_topic_type_is_reported_without_reading_it(self):
        report = bag_check.BagChecks(CONFIG, {"/front": "std_msgs/msg/String"}).report()
        self.assertIn("unexpected_type", {item["code"] for item in report["issues"]})
        self.assertIn("missing_topic", {item["code"] for item in report["issues"]})

    def test_config_rejects_duplicate_roles_and_accepts_unresolved_point_time(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            config = copy.deepcopy(CONFIG)
            config["point_time"] = {"field": "", "unit": ""}
            path.write_text(json.dumps(config))
            self.assertEqual(bag_check.load_config(path)["point_time"]["unit"], "")
            config["topics"]["imu"] = config["topics"]["front_points"]
            path.write_text(json.dumps(config))
            with self.assertRaises(ValueError):
                bag_check.load_config(path)


class RosBagRoundTripTests(unittest.TestCase):
    def test_serialized_messages_read_from_bag_and_report_not_overwritten(self):
        try:
            import rosbag2_py
            from rclpy.serialization import serialize_message
            from sensor_msgs.msg import Imu, PointCloud2, PointField
        except ImportError:
            self.skipTest("Source ROS 2 for the actual rosbag round-trip test")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bag = root / "bag"
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                        rosbag2_py.ConverterOptions("", ""))
            for index, (name, message_type) in enumerate(TYPES.items()):
                try:
                    metadata = rosbag2_py.TopicMetadata(id=index, name=name, type=message_type, serialization_format="cdr")
                except TypeError:  # Humble has no id constructor argument.
                    metadata = rosbag2_py.TopicMetadata(name=name, type=message_type, serialization_format="cdr")
                writer.create_topic(metadata)
            for second in (1, 2):
                points = PointCloud2()
                points.header.stamp.sec = second
                points.header.frame_id = "lidar"
                points.width = 2
                points.height = 1
                points.point_step = 16
                points.row_step = 32
                points.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                                 for index, name in enumerate(("x", "y", "z", "time"))]
                points.data = struct.pack("<8f", 1.0, 2.0, 3.0, 0.0, 4.0, 5.0, 6.0, 0.001)
                inertial = Imu()
                inertial.header.stamp.sec = second
                inertial.header.frame_id = "imu"
                inertial.linear_acceleration.z = 9.81
                inertial.orientation_covariance[0] = -1.0
                writer.write("/front", serialize_message(points), second * 1_000_000_000)
                writer.write("/imu", serialize_message(inertial), second * 1_000_000_000)
            del writer
            config_path = root / "config.json"
            config_path.write_text(json.dumps(CONFIG))
            output = root / "report.json"
            raw_files = {str(path.relative_to(bag)): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in bag.rglob("*") if path.is_file()}
            self.assertEqual(bag_check.main([str(bag), "--config", str(config_path), "--output", str(output)]), 0)
            original = output.read_bytes()
            report = json.loads(original)
            self.assertEqual(report["topics"]["/front"]["message_count"], 2)
            self.assertEqual(report["topics"]["/imu"]["message_count"], 2)
            self.assertEqual(bag_check.main([str(bag), "--config", str(config_path), "--output", str(output)]), 2)
            self.assertEqual(output.read_bytes(), original)
            self.assertEqual(bag_check.main([str(bag), "--config", str(config_path),
                                            "--output", str(bag / "should-not-write.json")]), 2)
            self.assertFalse((bag / "should-not-write.json").exists())
            self.assertEqual({str(path.relative_to(bag)): hashlib.sha256(path.read_bytes()).hexdigest()
                              for path in bag.rglob("*") if path.is_file()}, raw_files)


if __name__ == "__main__":
    unittest.main()
