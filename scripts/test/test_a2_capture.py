import importlib.util
import json
from pathlib import Path
import signal
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location("a2_capture", Path(__file__).parents[1] / "a2_capture.py")
capture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture)


class CaptureContractTest(unittest.TestCase):
    def setUp(self):
        self.config = {"schema_version": 1, "topics": {
            "front_points": "/sensor/front/points", "imu": "/sensor/imu"},
            "point_time": {"field": "", "unit": ""}, "storage_id": "sqlite3"}

    def load(self, config):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "capture.json"
            path.write_text(json.dumps(config))
            return capture.load_config(path)

    def test_incomplete_example_does_not_record(self):
        with self.assertRaisesRegex(ValueError, "front_points and imu"):
            capture.load_config(Path(__file__).parents[2] / "config/examples/mapping-capture.json")

    def test_raw_capture_can_preserve_unknown_point_time(self):
        self.assertEqual(self.load(self.config)["point_time"], {"field": "", "unit": ""})

    def test_control_role_and_duplicate_topic_rejected(self):
        self.config["topics"]["cmd_vel"] = "/cmd_vel"
        with self.assertRaisesRegex(ValueError, "unknown sensor role"):
            self.load(self.config)
        del self.config["topics"]["cmd_vel"]
        self.config["topics"]["imu"] = self.config["topics"]["front_points"]
        with self.assertRaisesRegex(ValueError, "distinct"):
            self.load(self.config)

    def test_non_absolute_and_option_like_topic_rejected(self):
        for topic in ("--all", "relative", "/sensor/*", "/tf_static"):
            with self.subTest(topic=topic):
                self.config["topics"]["imu"] = topic
                with self.assertRaisesRegex(ValueError, "absolute ROS"):
                    self.load(self.config)

    def test_partial_time_declaration_rejected(self):
        for value in ({"field": "time", "unit": ""}, {"field": "", "unit": "ns"},
                      {"field": "time", "unit": "ticks"}):
            self.config["point_time"] = value
            with self.assertRaises(ValueError):
                self.load(self.config)

    def test_storage_and_schema_rejected(self):
        self.config["storage_id"] = "unknown"
        with self.assertRaisesRegex(ValueError, "storage_id"):
            self.load(self.config)
        self.config["storage_id"] = "sqlite3"
        self.config["schema_version"] = True
        with self.assertRaisesRegex(ValueError, "schema_version"):
            self.load(self.config)

    def test_command_has_only_explicit_sensors_and_tf_with_both_ros_clis(self):
        common_help = "--storage --max-bag-size --qos-profile-overrides-path"
        for topic_option in ("", " --topics"):
            command = capture.record_command(self.config, Path("/tmp/session with spaces"), common_help + topic_option)
            self.assertEqual(command[-4:], ["/sensor/front/points", "/sensor/imu", "/tf", "/tf_static"])
            self.assertNotIn("-a", command)
            self.assertEqual(command.count("--topics"), bool(topic_option))
            self.assertIn("/tmp/session with spaces/bag", command)

    def test_reports_are_not_overwritten(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            capture.write_json(path, {"original": True})
            with self.assertRaises(FileExistsError):
                capture.write_json(path, {"original": False})
            self.assertEqual(json.loads(path.read_text()), {"original": True})

    def test_disk_reserve_stops_child_instead_of_filling_storage(self):
        with TemporaryDirectory() as directory, patch.object(capture.shutil, "disk_usage", return_value=SimpleNamespace(free=0)):
            code, reason = capture.run_recorder(["sleep", "10"], 10, directory, 1)
        self.assertEqual(reason, "disk_reserve")
        self.assertEqual(code, -signal.SIGINT)

    def test_duration_sends_graceful_interrupt(self):
        with TemporaryDirectory() as directory:
            code, reason = capture.run_recorder(["sleep", "10"], 0.02, directory, 0)
        self.assertEqual(reason, "duration_elapsed")
        self.assertEqual(code, -signal.SIGINT)


if __name__ == "__main__":
    unittest.main()
