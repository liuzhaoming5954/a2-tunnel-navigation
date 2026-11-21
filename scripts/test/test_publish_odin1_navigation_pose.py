"""用合成安装外参和非单位地图变换验证导航机身位姿、时间与 ROS 发布边界。"""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import unittest

from scripts.observe_odin1_pose import MAX_AGE_NS
from scripts.publish_odin1_navigation_pose import BodyProjection, Transform, load_extrinsics, planar_velocity


DIGEST = "3" * 64
S = math.sqrt(0.5)
# 安装外参绕 X 倒装 180°，IMU 在机身中的平移为 (1, 2, 3)。
EXTRINSIC = {"translation_m":[1., 2., 3.], "quaternion_xyzw":[1., 0., 0., 0.]}
OBS = {
    "source":"odin1_relocalization", "status":"live", "frame":"map", "child_frame":"imu",
    "source_domain_id":41, "map_sha256":DIGEST, "boot_id":"test-boot",
    "stamp_ns":1000, "received_monotonic_ns":20_000_000_000,
    "T_odom_from_map":{"translation_m":[10., 20., 3.], "quaternion_xyzw":[0., 0., S, S]},
    # 机身在 odom 系的航向为 +90°，姿态附加安装倒装后即 IMU 的实测姿态。
    "T_odom_from_imu":{"translation_m":[6., 25., 10.], "quaternion_xyzw":[S, S, 0., 0.]},
}


class NavigationPoseTest(unittest.TestCase):
    def setUp(self):
        self.project = BodyProjection(DIGEST, "test-boot", 43, Transform.read(EXTRINSIC))
        self.steady = OBS["received_monotonic_ns"] + 100_000_000
        self.ros = 1_700_000_000_000_000_000

    def test_two_transforms_apply_full_installation_and_share_original_measurement_age(self):
        self.assertIsNone(self.project.sample(OBS, self.steady, self.ros))
        dt = 100_000_000
        sample = dict(OBS, stamp_ns=OBS["stamp_ns"]+dt, received_monotonic_ns=OBS["received_monotonic_ns"]+dt)
        stamp, map_from_odom, odom_from_base, velocity = self.project.sample(sample, self.steady+dt, self.ros+dt)
        self.assertEqual(stamp, self.ros+dt-100_000_000)
        self.assertEqual(velocity, (0.,0.,0.))
        # odom 下的机身原点为 (8,24,7)，在 map 下为 (4,2,4)；map 下的机头沿 +X。
        for actual, expected in zip(odom_from_base.p, (8., 24., 7.)):
            self.assertAlmostEqual(actual, expected)
        body = map_from_odom.compose(odom_from_base)
        for actual, expected in zip(body.p, (4., 2., 4.)):
            self.assertAlmostEqual(actual, expected)
        for actual in body.q[:3]:
            self.assertAlmostEqual(actual, 0.)
        self.assertAlmostEqual(abs(body.q[3]), 1.)
        self.assertIsNone(self.project.sample(sample, self.steady+dt+50_000, self.ros+dt+50_000))
        with self.assertRaises(ValueError):
            self.project.sample(OBS, self.steady+dt+50_000, self.ros+dt+50_000)

    def test_body_velocity_direction_wrap_and_reacquisition(self):
        def quat(degrees):
            half = math.radians(degrees)/2
            return (0., 0., math.sin(half), math.cos(half))
        a = Transform((1., 2., 0.), quat(179))
        b = Transform((0.9, 2., 0.), quat(-179))
        vx, vy, wz = planar_velocity(a, b, 0.1)
        self.assertAlmostEqual(vx, math.cos(math.radians(1)))
        self.assertAlmostEqual(vy, -math.sin(math.radians(1)))
        self.assertAlmostEqual(wz, math.radians(20))
        dt = 100_000_000
        def observe(t, device=None):
            frame = dict(OBS, stamp_ns=1000+(t if device is None else device),
                         received_monotonic_ns=OBS["received_monotonic_ns"]+t)
            return self.project.sample(frame, self.steady+t, self.ros+t)
        self.assertIsNone(observe(0))
        self.assertIsNotNone(observe(dt))
        self.assertIsNone(observe(dt*11))
        self.assertIsNotNone(observe(dt*12))
        self.assertIsNone(observe(dt*13, 0))
        self.assertIsNotNone(observe(dt*14, dt))
        with self.assertRaises(ValueError):
            self.project.sample(dict(OBS, status="waiting"), self.steady+dt*15, self.ros+dt*15)
        self.assertIsNone(observe(dt*16))
        self.assertIsNotNone(observe(dt*17))

    def test_invalid_stale_foreign_and_old_observations_cannot_publish(self):
        for changes in ({"source_domain_id":43}, {"source_domain_id":None}, {"boot_id":"old-boot"},
                        {"source":"mock"}, {"status":"waiting"}, {"child_frame":"base_link"},
                        {"map_sha256":"5"*64}, {"received_monotonic_ns":self.steady-MAX_AGE_NS},
                        {"received_monotonic_ns":self.steady+1}, {"stamp_ns":0},
                        {"T_odom_from_map":{"translation_m":[1,2,3],"quaternion_xyzw":[0,0,0,0]}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.project.sample({**OBS, **changes}, self.steady, self.ros)
        with self.assertRaises(KeyError):
            self.project.sample({key: val for key, val in OBS.items() if key != "T_odom_from_map"},
                                self.steady, self.ros)

    def test_pure_body_turn_does_not_mistake_sensor_lever_arm_motion_for_translation(self):
        # IMU 离机身原点 1 米。原地转 0.1 rad 时传感器沿弧线移动，机身中心仍静止。
        base_from_imu = Transform((1., 0., 0.), (0., 0., 0., 1.))
        project = BodyProjection(DIGEST, "test-boot", 43, base_from_imu)
        for i, angle in enumerate((0., 0.1)):
            odom_from_imu = dict(translation_m=[math.cos(angle), math.sin(angle), 0.],
                                 quaternion_xyzw=[0., 0., math.sin(angle/2), math.cos(angle/2)])
            frame = dict(OBS, T_odom_from_imu=odom_from_imu, stamp_ns=100+i*100_000_000,
                         received_monotonic_ns=OBS["received_monotonic_ns"]+i*100_000_000)
            result = project.sample(frame, self.steady+i*100_000_000, self.ros+i*100_000_000)
        vx, vy, wz = result[3]
        self.assertAlmostEqual(vx, 0.)
        self.assertAlmostEqual(vy, 0.)
        self.assertAlmostEqual(wz, 1.)

    def test_extrinsics_requires_matching_frame_and_source(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp)/"mount.json"
            for fields in ({}, {"source":"", "base_frame":"base_link"},
                           {"source":"合成校验", "base_frame":"another_frame"}):
                path.write_text(json.dumps({**fields, "T_base_from_imu":EXTRINSIC}))
                with self.assertRaises(ValueError):
                    load_extrinsics(path, "base_link")
            path.write_text(json.dumps(dict(source="合成校验",base_frame="base_link",T_base_from_imu=EXTRINSIC)))
            self.assertEqual(load_extrinsics(path,"base_link").p, (1.,2.,3.))


@unittest.skipUnless(os.environ.get("A2_RUN_LOCAL_NAV2_LINK") == "1", "按需验证真实 ROS TF 与 Odometry")
class NavigationPoseWireTest(unittest.TestCase):
    def test_bridge_publishes_two_transforms_and_pose_without_replaying_stale_files(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage

        self.assertEqual(os.environ.get("ROS_LOCALHOST_ONLY"), "1")
        domain = int(os.environ["ROS_DOMAIN_ID"])
        self.assertNotEqual(domain, OBS["source_domain_id"])
        with TemporaryDirectory() as tmp:
            folder = Path(tmp)
            pose_file, extrinsics = folder/"obs.json", folder/"mount.json"
            extrinsics.write_text(json.dumps(dict(source="合成试验",base_frame="base_link",T_base_from_imu=EXTRINSIC)))
            rclpy.init(args=[])
            node = rclpy.create_node("odin_body_wire_observer")
            tf, odom = [], []
            node.create_subscription(TFMessage, "/tf", lambda msg: tf.extend(msg.transforms), 20)
            node.create_subscription(Odometry, "/odom", odom.append, 10)
            command = [sys.executable, str(Path(__file__).resolve().parents[1]/"publish_odin1_navigation_pose.py"),
                       "--pose-file", str(pose_file), "--extrinsics", str(extrinsics),
                       "--map-sha256", DIGEST, "--output-domain", str(domain)]
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                deadline = time.monotonic()+5
                while (node.count_publishers("/odom") == 0 or node.count_publishers("/tf") == 0):
                    rclpy.spin_once(node, timeout_sec=.05)
                    self.assertLess(time.monotonic(), deadline, "未发现导航发布者")
                # 给 DDS 留一个匹配周期，第一帧只做差分基线；第二帧发布后固定文件，不应持续补帧。
                time.sleep(.2)
                input_record = dict(OBS, boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                                    received_monotonic_ns=time.monotonic_ns()-60_000_000)
                pose_file.write_text(json.dumps(input_record))
                time.sleep(.15)
                input_record["received_monotonic_ns"] += 150_000_000
                input_record["stamp_ns"] += 150_000_000
                stage = folder/"stage"
                stage.write_text(json.dumps(input_record))
                stage.replace(pose_file)
                deadline = time.monotonic()+1.1
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.04)
                self.assertEqual(len(odom), 1)
                self.assertEqual([(t.header.frame_id,t.child_frame_id) for t in tf],
                                 [("map","odin_nav_odom"),("odin_nav_odom","base_link")])
                self.assertTrue(all(t.header.stamp == odom[0].header.stamp for t in tf))
                self.assertEqual((odom[0].header.frame_id,odom[0].child_frame_id),("odin_nav_odom","base_link"))
                self.assertAlmostEqual(odom[0].pose.pose.position.x, 8.)
                self.assertEqual(odom[0].twist.twist.linear.x, 0.)
                self.assertEqual(odom[0].twist.twist.angular.z, 0.)
                self.assertTrue(all(map(math.isfinite, odom[0].twist.covariance)))
            finally:
                proc.send_signal(signal.SIGINT)
                proc.communicate(timeout=5)
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == "__main__":
    unittest.main()
