import tempfile
import unittest
from pathlib import Path

from a2_bringup.configuration import parse_mock_configuration


class MockConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        self.plan = root / "mock-plan.json"
        self.plan.write_text("{}")
        self.options = {
            "profile": "mock", "robot_id": "test-robot",
            "database_path": str(root / "new-parent" / "mission.sqlite"),
            "mock_plan_path": str(self.plan), "platform_drop_after_s": "-1.0",
        }

    def parse(self, **changes):
        return parse_mock_configuration(**{**self.options, **changes})

    def test_mock_configuration_preserves_identity_and_absolute_paths(self):
        config = self.parse()
        self.assertEqual(config.robot_id, "test-robot")
        self.assertEqual(config.database_path, self.options["database_path"])
        self.assertEqual(config.mock_plan_path, str(self.plan))
        self.assertEqual(config.platform_drop_after_s, -1.0)
        # Launch validation must not create persistent runtime data.
        self.assertFalse(Path(config.database_path).parent.exists())

    def test_only_mock_profile_is_allowed(self):
        for profile in ("robot_motion", "robot_readonly", "replay", "", "MOCK"):
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                self.parse(profile=profile)

    def test_database_path_must_be_absolute_and_not_a_directory(self):
        for path in ("data/mock.sqlite", self.directory.name):
            with self.subTest(path=path), self.assertRaises(ValueError):
                self.parse(database_path=path)

    def test_plan_path_must_be_absolute_existing_regular_file(self):
        for path in ("mock-plan.json", str(self.plan.parent / "missing.json"), self.directory.name):
            with self.subTest(path=path), self.assertRaises(ValueError):
                self.parse(mock_plan_path=path)

    def test_dropout_accepts_exact_disable_and_nonnegative_values(self):
        for value in ("-1", "0", "0.25", "86400"):
            with self.subTest(value=value):
                self.assertEqual(self.parse(platform_drop_after_s=value).platform_drop_after_s,
                                 float(value))

    def test_dropout_rejects_invalid_negative_nonfinite_and_oversized_values(self):
        for value in ("-0.1", "-2", "nan", "inf", "-inf", "86400.1", "bad"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.parse(platform_drop_after_s=value)

    def test_robot_identity_cannot_be_empty(self):
        for value in ("", "  ", "a" * 257):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.parse(robot_id=value)


if __name__ == "__main__":
    unittest.main()
