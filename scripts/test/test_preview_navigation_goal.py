import math
import unittest

from scripts.preview_navigation_goal import (
    body_pose_from_lidar,
    load_straight_route,
    map_database_digest,
    multiply_transform,
    pose_matrix,
    replay,
    simulate,
)


class NavigationPreviewTest(unittest.TestCase):
    PLAN = {
        "status": "planned", "frame_id": "map",
        "start_m": [1.0, 2.0], "goal_m": [1.0, 3.0],
        "path_length_m": 1.0, "path_m": [[1.0, 2.0], [1.0 + 1e-15, 2.0], [1.0, 3.0]],
        "map": {"metadata_sha256": "a" * 64, "image_sha256": "b" * 64},
    }
    BASE_FROM_LIDAR = (
        (0, 0, 1, 0.3),
        (1, 0, 0, 0.1),
        (0, 1, 0, 0.2),
        (0, 0, 0, 1),
    )

    def setUp(self):
        self.route = load_straight_route(self.PLAN)

    def test_lidar_offset_and_axes_in_map_frame(self):
        # The LiDAR is 0.3 m forward and 0.1 m left of the body, which
        # points along map +Y in this fixture.
        map_from_lidar = (
            (-1, 0, 0, 0.9),
            (0, 0, 1, 2.3),
            (0, 1, 0, 0.2),
            (0, 0, 0, 1),
        )
        pose = body_pose_from_lidar(map_from_lidar, self.BASE_FROM_LIDAR, 1_000_000_001)
        self.assertAlmostEqual(pose.x_m, 1.0)
        self.assertAlmostEqual(pose.y_m, 2.0)
        self.assertAlmostEqual(pose.yaw_rad, math.pi / 2)
        self.assertEqual(pose.stamp_ns, 1_000_000_001)

    def test_simulated_feedback_advances_then_stops_at_goal(self):
        samples = simulate(self.route, self.BASE_FROM_LIDAR, lose_pose=False)
        self.assertGreater(samples[0]["command"]["linear_x_mps"], 0)
        self.assertLess(samples[0]["command"]["angular_z_radps"], 0)
        terminal = samples[-2]
        self.assertEqual(terminal["command"]["state"], "arrived")
        error = math.dist((terminal["body_pose"]["x_m"], terminal["body_pose"]["y_m"]),
                          self.route.goal_m)
        self.assertLessEqual(error, 0.05)
        for sample in samples[-2:]:
            self.assertEqual(sample["command"]["linear_x_mps"], 0)
            self.assertEqual(sample["command"]["angular_z_radps"], 0)

    def test_pose_dropout_stops_and_fresh_pose_does_not_resume(self):
        samples = simulate(self.route, self.BASE_FROM_LIDAR, lose_pose=True)
        self.assertGreater(samples[0]["command"]["linear_x_mps"], 0)
        stale, recovered = samples[-2:]
        self.assertEqual(stale["command"]["reason"], "stale_pose")
        self.assertGreater(recovered["body_pose"]["stamp_ns"], stale["body_pose"]["stamp_ns"])
        self.assertEqual(stale["command"], recovered["command"])
        self.assertEqual(recovered["command"]["linear_x_mps"], 0)
        self.assertEqual(recovered["command"]["angular_z_radps"], 0)

    def test_recorded_pose_at_another_start_cannot_trigger_motion(self):
        stamp = 2_000_000_001
        transform = multiply_transform(pose_matrix((0, 0, 0), (0, 0, 0, 1)), self.BASE_FROM_LIDAR)
        report = {"map_sha256": "c" * 64, "alignment": {"accepted": True},
                  "poses_after_alignment": [{"sensor_reference_ns": stamp,
                                             "observer_clock_at_pair_ns": stamp + 10_000_000,
                                             "T_map_from_front_lidar_reference": transform}]}
        samples = replay(self.route, self.BASE_FROM_LIDAR, report, "c" * 64)
        self.assertEqual(samples[0]["body_pose"]["stamp_ns"], stamp)
        self.assertEqual(samples[0]["command"]["reason"], "not_at_start")
        self.assertEqual(samples[0]["command"]["linear_x_mps"], 0)
        self.assertEqual(samples[0]["command"]["angular_z_radps"], 0)

    def test_route_cannot_use_another_map_export(self):
        manifest = {"source_sha256": {"database": "c" * 64},
                    "output_sha256": {"map.yaml": "a" * 64, "map.pgm": "d" * 64}}
        with self.assertRaisesRegex(ValueError, "different occupancy maps"):
            map_database_digest(self.PLAN, manifest)


if __name__ == "__main__":
    unittest.main()
