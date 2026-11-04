#!/usr/bin/env python3
"""Exercise capture with local synthetic ROS sensors, never an A2 connection."""

import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[1]


def publisher(ready):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=[])
    node = rclpy.create_node("a2_capture_synthetic_test")
    cloud = node.create_publisher(PointCloud2, "/capture_smoke/front_points", qos_profile_sensor_data)
    imu = node.create_publisher(Imu, "/capture_smoke/imu", qos_profile_sensor_data)
    unused = node.create_publisher(Imu, "/capture_smoke/not_selected", qos_profile_sensor_data)
    static = node.create_publisher(TFMessage, "/tf_static", QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    transforms = []
    for frame in ("front_lidar", "imu_link"):
        tf = TransformStamped()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = frame
        tf.transform.rotation.w = 1.0
        transforms.append(tf)
    # Publish exactly once, before the recorder exists: tests late-joiner QoS.
    static.publish(TFMessage(transforms=transforms))

    def tick():
        stamp = node.get_clock().now().to_msg()
        points = PointCloud2()
        points.header.stamp = stamp
        points.header.frame_id = "front_lidar"
        points.height = points.width = 1
        points.point_step = points.row_step = 16
        points.fields = [PointField(name=name, offset=index * 4,
                                   datatype=PointField.FLOAT32, count=1)
                         for index, name in enumerate(("x", "y", "z", "time"))]
        points.data = struct.pack("<ffff", 1.0, 2.0, 3.0, 0.0)
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = "imu_link"
        message.linear_acceleration.z = 9.81
        message.orientation_covariance[0] = -1.0
        cloud.publish(points)
        imu.publish(message)
        unused.publish(message)

    node.create_timer(0.05, tick)
    Path(ready).write_text("synthetic publishers ready\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def run():
    # The caller also chooses a dedicated domain. No SDK or hardware adapter is loaded.
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or "ROS_DOMAIN_ID" not in os.environ:
        raise RuntimeError("set ROS_LOCALHOST_ONLY=1 and a dedicated ROS_DOMAIN_ID before running this synthetic test")
    with tempfile.TemporaryDirectory(prefix="a2-capture-smoke-") as directory:
        work = Path(directory)
        ready = work / "ready"
        config = work / "config.json"
        config.write_text(json.dumps({
            "schema_version": 1,
            "topics": {"front_points": "/capture_smoke/front_points", "imu": "/capture_smoke/imu"},
            "point_time": {"field": "time", "unit": "s"}, "storage_id": "sqlite3",
        }))
        process = subprocess.Popen([sys.executable, __file__, "--publisher", str(ready)],
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 10
            while not ready.exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("synthetic publisher did not start")
                time.sleep(0.05)
            session = work / "capture"
            # Preflight and recorder are separate DDS participants; bound both
            # discovery and recording windows for cold CI startup.
            subprocess.run([sys.executable, str(ROOT / "scripts/a2_capture.py"), "record",
                            "--config", str(config), "--session", str(session), "--duration", "10",
                            "--discovery-seconds", "8", "--sample-seconds", "3",
                            "--min-free-gib", "0.1", "--execute"], check=True, timeout=90)
            report_path = work / "report.json"
            subprocess.run([sys.executable, str(ROOT / "scripts/inspect_mapping_bag.py"), str(session / "bag"),
                            "--config", str(config), "--output", str(report_path)], check=True, timeout=15)
            report = json.loads(report_path.read_text())
            if report["issues"]:
                raise RuntimeError(f"unexpected bag issues: {report['issues']}")
            for topic in ("/capture_smoke/front_points", "/capture_smoke/imu", "/tf_static"):
                if report["topics"][topic]["message_count"] < 1:
                    raise RuntimeError(f"actual messages missing: {topic}")
            if "/capture_smoke/not_selected" in report["stored_topic_types"]:
                raise RuntimeError("recorder captured a topic outside the whitelist")
            if len(report["tf_edges"]) != 2:
                raise RuntimeError("static transforms published before recording were not retained")
            result = json.loads((session / "recording-result.json").read_text())
            if result["stop_reason"] != "duration_elapsed" or not result["metadata_present"]:
                raise RuntimeError(f"recorder did not finalize normally: {result}")
            print("PASS: synthetic ROS sensors -> whitelist recording -> actual bag inspection; late static TF retained.")
            print("This does not test A2 sensors, time synchronization, DLIO or mapping quality.")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            if process.returncode != 0:
                raise RuntimeError(f"synthetic publisher exited with code {process.returncode}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--publisher", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or "ROS_DOMAIN_ID" not in os.environ:
        parser.error("set ROS_LOCALHOST_ONLY=1 and a dedicated ROS_DOMAIN_ID")
    if args.publisher:
        publisher(args.publisher)
    else:
        run()
