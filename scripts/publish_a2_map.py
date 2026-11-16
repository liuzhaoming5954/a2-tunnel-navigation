#!/usr/bin/env python3
"""Publish a saved trinary PGM map for the A2 maintenance console."""

import argparse
from array import array
import math
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/a2_navigation"))

from a2_navigation.offline_planner import load_trinary_grid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True, help="ROS trinary map YAML")
    parser.add_argument("--map-topic", default="/map", help="OccupancyGrid topic")
    parser.add_argument("--map-frame", default="map", help="Map coordinate frame")
    args = parser.parse_args()
    if not args.map_frame or args.map_frame.startswith("/") or any(c.isspace() for c in args.map_frame):
        parser.error("--map-frame requires a nonempty TF frame without whitespace or a leading slash")
    try:
        grid = load_trinary_grid(args.map)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    try:
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as error:
        parser.error(f"source a ROS 2 environment with rclpy and nav_msgs: {error}")

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("a2_map_publisher")
        publisher = node.create_publisher(OccupancyGrid, args.map_topic, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ))
        message = OccupancyGrid()
        message.header.frame_id = args.map_frame
        message.header.stamp = node.get_clock().now().to_msg()
        message.info.map_load_time = message.header.stamp
        message.info.resolution = grid.resolution
        message.info.width = grid.width
        message.info.height = grid.height
        x, y, yaw = grid.origin
        message.info.origin.position.x = x
        message.info.origin.position.y = y
        message.info.origin.orientation.z = math.sin(yaw / 2)
        message.info.origin.orientation.w = math.cos(yaw / 2)
        message.data = array("b", grid.cells)
        publisher.publish(message)
        node.get_logger().info(f"Published {grid.width} x {grid.height} map on {publisher.topic_name} ({args.map_frame})")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
