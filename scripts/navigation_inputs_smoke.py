#!/usr/bin/env python3
"""Exercise the read-only navigation probe in an isolated synthetic ROS 2 graph."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def run_case(node, output, name, map_transform=True, lag_s=0.10):
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.duration import Duration
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_ros import TransformBroadcaster

    suffix = f"{os.getpid()}_{name}"
    map_frame, odom_frame, base_frame = (f"smoke_{suffix}_{kind}" for kind in ("map", "odom", "base"))
    map_topic, odom_topic = f"/smoke_{suffix}/map", f"/smoke_{suffix}/odom"
    map_pub = node.create_publisher(OccupancyGrid, map_topic, QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = node.create_publisher(Odometry, odom_topic, 10)
    broadcaster = TransformBroadcaster(node)
    grid = OccupancyGrid()
    grid.header.frame_id = map_frame
    grid.header.stamp = node.get_clock().now().to_msg()
    grid.info.resolution = 0.2
    grid.info.width = grid.info.height = 5
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 24 + [-1]
    map_pub.publish(grid)

    path = Path(__file__).with_name("observe_navigation_inputs.py")
    command = [sys.executable, str(path), "--map-topic", map_topic, "--odom-topic", odom_topic,
               "--map-frame", map_frame, "--odom-frame", odom_frame, "--base-frame", base_frame,
               "--duration-s", "3", "--max-pose-age-s", "0.50", "--output", str(output)]
    with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as process:
        deadline = time.monotonic() + 12.0
        while process.poll() is None:
            if time.monotonic() > deadline:
                process.kill()
                raise RuntimeError("Read-only navigation probe exceeded the test budget")
            stamp = (node.get_clock().now() - Duration(seconds=lag_s)).to_msg()
            transforms = []
            edges = [(odom_frame, base_frame, 0.4)]
            if map_transform:
                edges.insert(0, (map_frame, odom_frame, 0.8))
            for parent, child, x in edges:
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = parent, child
                transform.header.stamp = stamp
                transform.transform.translation.x = x
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
            broadcaster.sendTransform(transforms)
            odometry = Odometry()
            odometry.header.frame_id, odometry.child_frame_id = odom_frame, base_frame
            odometry.header.stamp = stamp
            odometry.pose.pose.position.x = 0.4
            odometry.pose.pose.orientation.w = 1.0
            odom_pub.publish(odometry)
            time.sleep(0.06)
        stdout, _ = process.communicate()
        expected_status = 0 if name == "healthy" else 1
        if process.returncode != expected_status or not output.is_file():
            raise RuntimeError(f"Unexpected {name} result, exit={process.returncode}\n{stdout}")
    result = json.loads(output.read_text())
    if not result["checks"]["valid_occupancy_grid"] or result["counts"].get("odometry_messages", 0) < 3:
        raise RuntimeError(f"Missing synthetic inputs for {name}: {result}")
    if name == "healthy":
        if not result["passed"] or abs(result["latest_matching_pose"]["position_m"][0] - 1.2) > 1e-6:
            raise RuntimeError(f"Expected map pose at x=1.2 m: {result}")
    elif result["passed"] or result["counts"].get("fresh_odometry_with_tf", 0):
        raise RuntimeError(f"Probe accepted missing or stale evidence: {result}")
    node.destroy_publisher(map_pub)
    node.destroy_publisher(odom_pub)
    print(f"PASS {name}")


def main():
    legacy_local = os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    scoped_local = os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST"
    if not (legacy_local or scoped_local) or os.environ.get("ROS_STATIC_PEERS") or not os.environ.get("ROS_DOMAIN_ID", "").isdigit():
        raise SystemExit("Use localhost-only DDS, no static peers and an explicit isolated ROS_DOMAIN_ID")
    import rclpy
    rclpy.init(args=[])
    node = rclpy.create_node("a2_navigation_inputs_smoke", enable_rosout=False, start_parameter_services=False)
    try:
        with tempfile.TemporaryDirectory(prefix="a2-navigation-inputs-") as scratch:
            for name, transform, lag in (("healthy", True, 0.10), ("missing_tf", False, 0.10),
                                          ("stale_odometry", True, 2.0)):
                run_case(node, Path(scratch) / f"{name}.json", name, transform, lag)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
