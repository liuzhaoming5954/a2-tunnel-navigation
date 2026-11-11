#!/usr/bin/env python3
"""Read a ROS 2 map and look up navigation transforms at odometry timestamps."""

import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time


def pose_values(position, rotation):
    xyz = [position.x, position.y, position.z]
    xyzw = [rotation.x, rotation.y, rotation.z, rotation.w]
    if not all(map(math.isfinite, xyz + xyzw)) or not math.isclose(math.hypot(*xyzw), 1.0, abs_tol=0.002):
        raise ValueError("Nonfinite position or invalid quaternion")
    return {"position_m": xyz, "quaternion_xyzw": xyzw}


def observe(args, node):
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    pending = deque(maxlen=256)
    counters = Counter()
    map_info = None
    last_stamp_ns = 0
    last_matching = None
    latest_tf_error = None
    clock_type = node.get_clock().clock_type
    maximum_age_ns = round(args.max_pose_age_s * 1e9)

    def read_map(message):
        nonlocal map_info
        try:
            info = message.info
            if (message.header.frame_id != args.map_frame or info.width <= 0 or info.height <= 0
                    or not math.isfinite(info.resolution) or info.resolution <= 0
                    or len(message.data) != info.width * info.height):
                raise ValueError("Unexpected frame, resolution or dimensions")
            origin = pose_values(info.origin.position, info.origin.orientation)
            cells = Counter(message.data)
            if any(value < -1 or value > 100 for value in cells) or not cells[0]:
                raise ValueError("Invalid occupancy values or no known-free cells")
        except ValueError as error:
            counters["invalid_map_messages"] += 1
            map_info = {"valid": False, "reason": str(error)}
            return
        counters["valid_map_messages"] += 1
        map_info = {"valid": True, "frame_id": message.header.frame_id, "width": info.width, "height": info.height,
                    "resolution_m": info.resolution, "origin": origin, "known_free_cells": cells[0],
                    "unknown_cells": cells[-1]}

    def read_odometry(message):
        nonlocal last_stamp_ns
        counters["odometry_messages"] += 1
        try:
            stamp = Time.from_msg(message.header.stamp, clock_type=clock_type)
            if (message.header.frame_id != args.odom_frame or message.child_frame_id != args.base_frame
                    or stamp.nanoseconds <= last_stamp_ns):
                raise ValueError("Unexpected odometry frames or nonincreasing stamp")
            pose_values(message.pose.pose.position, message.pose.pose.orientation)
        except ValueError:
            counters["invalid_odometry_messages"] += 1
            return
        last_stamp_ns = stamp.nanoseconds
        if len(pending) == pending.maxlen:
            counters["odometry_queue_evictions"] += 1
        pending.append(stamp)

    map_qos = QoSProfile(
        depth=1,
        reliability=(ReliabilityPolicy.RELIABLE if args.map_reliability == "reliable"
                     else ReliabilityPolicy.BEST_EFFORT),
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if args.map_durability == "transient-local"
                    else DurabilityPolicy.VOLATILE),
    )
    node.create_subscription(OccupancyGrid, args.map_topic, read_map, map_qos)
    node.create_subscription(Odometry, args.odom_topic, read_odometry,
                             QoSProfile(depth=32, reliability=ReliabilityPolicy.BEST_EFFORT))
    began = time.monotonic()
    deadline = began + args.duration_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.04, max(0.0, deadline - time.monotonic())))
            now_ns = node.get_clock().now().nanoseconds
            for _ in range(len(pending)):
                stamp = pending.popleft()
                age_ns = now_ns - stamp.nanoseconds
                if age_ns > maximum_age_ns:
                    counters["expired_without_transform"] += 1
                    continue
                if age_ns < 0 or now_ns <= 0:
                    pending.append(stamp)
                    continue
                try:
                    buffer.lookup_transform(args.map_frame, args.odom_frame, stamp)
                    buffer.lookup_transform(args.odom_frame, args.base_frame, stamp)
                    transform = buffer.lookup_transform(args.map_frame, args.base_frame, stamp).transform
                    pose = pose_values(transform.translation, transform.rotation)
                except (TransformException, ValueError) as error:
                    latest_tf_error = str(error)
                    pending.append(stamp)
                    continue
                counters["fresh_odometry_with_tf"] += 1
                if last_matching is None or stamp.nanoseconds > last_matching["stamp_ns"]:
                    last_matching = {"stamp_ns": stamp.nanoseconds, **pose}
    finally:
        listener.unregister()

    now_ns = node.get_clock().now().nanoseconds
    latest_age = now_ns - last_matching["stamp_ns"] if last_matching else None
    checks = {
        "valid_occupancy_grid": bool(map_info and map_info["valid"]),
        "odometry_samples_with_same_time_tf": counters["fresh_odometry_with_tf"] >= args.min_pose_samples,
        "latest_odometry_has_fresh_tf": (latest_age is not None and 0 <= latest_age <= maximum_age_ns
                                         and last_matching["stamp_ns"] == last_stamp_ns),
        "odometry_frames_and_sequence": counters["invalid_odometry_messages"] == 0,
    }
    return {
        "scope": "Read-only observations of ROS navigation inputs. No localization confidence or motion admission.",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(), "checks": checks,
        "passed": all(checks.values()), "map_topic": args.map_topic, "odom_topic": args.odom_topic,
        "frames": [args.map_frame, args.odom_frame, args.base_frame],
        "use_sim_time": node.get_parameter("use_sim_time").value,
        "duration_s": time.monotonic() - began, "max_pose_age_s": args.max_pose_age_s,
        "minimum_pose_samples": args.min_pose_samples, "counts": dict(counters),
        "map": map_info, "latest_matching_pose": last_matching, "latest_pose_age_s": latest_age / 1e9
        if latest_age is not None else None, "latest_tf_error": latest_tf_error,
    }


def main():
    import rclpy
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-topic", required=True)
    parser.add_argument("--odom-topic", required=True)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--map-durability", choices=("transient-local", "volatile"), default="transient-local")
    parser.add_argument("--map-reliability", choices=("reliable", "best-effort"), default="reliable")
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--max-pose-age-s", type=float, required=True)
    parser.add_argument("--min-pose-samples", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if (not math.isfinite(args.duration_s) or not 0 < args.duration_s <= 60
            or not math.isfinite(args.max_pose_age_s) or not 1e-9 <= args.max_pose_age_s <= args.duration_s
            or args.min_pose_samples <= 0):
        parser.error("Choose a positive duration up to 60 s, a positive maximum age within it and a sample count")
    frames = (args.map_frame, args.odom_frame, args.base_frame)
    if len(set(frames)) != 3 or any(not frame or frame.startswith("/") for frame in frames):
        parser.error("Map, odometry and base frames must be distinct nonempty TF names without a leading slash")
    if args.output.exists():
        parser.error("Use a new output path")

    rclpy.init()
    node = rclpy.create_node("a2_navigation_input_probe", enable_rosout=False, start_parameter_services=False)
    try:
        result = observe(args, node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"ROS navigation inputs {'passed' if result['passed'] else 'incomplete'}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
