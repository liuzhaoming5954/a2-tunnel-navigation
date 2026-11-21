"""用小型合成图检查导航准入。无需 ROS、设备或网络。"""

import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import Mock

from a2_gateway.odin1_pose import BOOT_ID_FILE, SOURCE
from a2_gateway.pc2_goal import Pc2GoalPort, bin_digest


class Pc2GoalInputsTest(unittest.TestCase):
    def setUp(self):
        folder = TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        root = Path(folder.name)
        binary = root / "fixture.bin"
        binary.write_bytes(b"synthetic")
        self.digest = bin_digest(binary)
        raster = root / "grid.yaml"
        raster.write_text(json.dumps(dict(image="grid.pgm", resolution=0.5, origin=[-1, -1, 0],
                                         occupied_thresh=0.65, free_thresh=0.25)))
        (root / "grid.pgm").write_bytes(b"P5\n3 1\n255\n" + bytes((255, 0, 127)))
        self.meta = dict(id="grid", frame_id="map", width=3, height=1, resolution_m=0.5,
                         origin_x_m=-1., origin_y_m=-1., origin_yaw_rad=0.)
        self.telemetry = Mock()
        self.telemetry.navigation_grid.return_value = (self.meta, bytes((1, 101, 0)))
        self.state = dict(robot={"online": True}, pose={"map_id": "grid"}, map=self.meta)
        self.telemetry.snapshot.return_value = self.state
        self.backend = Mock()
        self.backend.available.return_value = True
        self.path = root / "pose.json"
        self.pose = dict(source=SOURCE, status="live", frame="map", child_frame="imu",
                         map_sha256=self.digest, boot_id=BOOT_ID_FILE.read_text().strip(), stamp_ns=1,
                         received_monotonic_ns=time.monotonic_ns(), position_m=[0., 0., 0.], yaw_rad=0.)
        self.path.write_text(json.dumps(self.pose))
        self.port = Pc2GoalPort(self.telemetry, self.path, self.digest, raster, self.backend, root / "test.sqlite")
        self.addCleanup(self.port.close)
        self.goal = dict(goal_id="example", map_sha256=self.digest, frame_id="map", x_m=-0.75,
                         y_m=-0.75, yaw_rad=0.)

    def test_target_cell_uses_map_origin_and_rejects_obstacle_unknown_and_outside(self):
        self.assertIsNone(self.port.check_target(self.goal))
        for x, reason in ((-0.25, "goal_cell_not_free"), (0.25, "goal_cell_not_free"),
                          (0.50, "goal_outside_map"), (1e308, "goal_outside_map")):
            with self.subTest(x=x):
                code, response = self.port.submit_goal(dict(self.goal, x_m=x))
                self.assertEqual((code, response["reason"]), (422, reason))
        self.backend.send.assert_not_called()

    def test_changed_ros_grid_and_stale_pose_prevent_submission(self):
        self.assertIsNone(self.port.readiness())
        # 即使地图尺寸与原点不变，更换一个像素仍要使旧地图绑定失效。
        self.meta["id"] = "new-grid"
        self.telemetry.navigation_grid.return_value = (self.meta, bytes((1, 1, 0)))
        self.assertEqual(self.port.submit_goal(self.goal)[1]["reason"], "navigation_grid_mismatch")
        self.path.write_text(json.dumps(dict(self.pose, received_monotonic_ns=1)))
        self.assertEqual(self.port.submit_goal(self.goal)[1]["reason"], "odin1_stale")
        self.backend.send.assert_not_called()

    def test_readiness_shows_missing_body_or_action_without_sending_a_goal(self):
        self.state["pose"] = None
        self.assertEqual(self.port.goal_status()[1]["readiness"],
                         {"ready": False, "reason": "body_pose_unavailable"})
        self.state["pose"] = {"map_id": "grid"}
        self.backend.available.return_value = False
        self.assertEqual(self.port.goal_status()[1]["readiness"], {"ready": False, "reason": "nav2_unavailable"})
        self.backend.send.assert_not_called()
