#!/usr/bin/env python3
"""Bridge a verified, uninterrupted DLIO session into a local display ROS graph."""

import argparse
from array import array
from collections import OrderedDict
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src" / name) for name in ("a2_localization", "a2_navigation")]

from a2_localization.body_pose import BodyPose, body_pose_from_lidar, pose_matrix
from a2_navigation.offline_planner import load_metadata, load_trinary_grid


NS_PER_S = 1_000_000_000
ODOM_FRAME = "a2_probe_odom"
LIDAR_FRAME = "front_lidar_reference"
SCAN_PAIR = re.compile(r"\bA2_SCAN_SYNC .*?input_ns=(\d+) reference_ns=(\d+).*?result=published\b")
MAX_SAMPLE_AGE_NS = 750_000_000
MAX_STREAM_GAP_NS = 1_250_000_000


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_anchor(report_path, manifest_path, map_path, extrinsics_path):
    report = json.loads(report_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    extrinsics = json.loads(extrinsics_path.read_text(encoding="utf-8"))
    metadata = load_metadata(map_path.read_text(encoding="utf-8"))
    if not isinstance(metadata.get("image"), str) or Path(metadata["image"]).name != metadata["image"]:
        raise ValueError("Map image must be a file beside the map YAML")
    map_image = map_path.with_name(metadata["image"])
    output = manifest["output_sha256"]
    if (report["map_sha256"] != manifest["source_sha256"]["database"]
            or output["map.yaml"] != digest(map_path) or output["map.pgm"] != digest(map_image)):
        raise ValueError("The anchor database and displayed map manifest do not match")
    alignment = report.get("alignment")
    if (report.get("status") != "diagnostic_anchored_stream_observed"
            or not isinstance(alignment, dict) or alignment.get("accepted") is not True
            or alignment.get("decision") != "map_handoff_verified"):
        raise ValueError("The live map anchor did not pass independent scan verification")
    if not isinstance(extrinsics.get("source"), str) or not extrinsics["source"].strip():
        raise ValueError("The LiDAR extrinsic must include its source")
    map_from_odom = alignment["T_map_from_fresh_odom"]
    base_from_lidar = extrinsics["T_base_from_lidar"]
    body_pose_from_lidar(map_from_odom, base_from_lidar, 0)
    samples = report["poses_after_alignment"]
    pairs = [(sample["sensor_reference_ns"], sample["sensor_scan_start_ns"]) for sample in samples]
    if (len(pairs) < 2 or any(type(ref) is not int or type(raw) is not int or not 0 < raw < ref
                              for ref, raw in pairs)
            or any(a[0] >= b[0] for a, b in zip(pairs, pairs[1:]))):
        raise ValueError("The live map anchor has no ordered post-verification scan pairs")
    origin = body_pose_from_lidar(samples[0]["T_map_from_front_lidar_reference"], base_from_lidar, pairs[0][0])
    return map_from_odom, base_from_lidar, origin, pairs


def process_identity(pid, log_path):
    proc = Path("/proc") / str(pid)
    # Read start ticks after the parenthesized comm field, which may contain spaces.
    fields = (proc / "stat").read_text(encoding="utf-8").rsplit(")", 1)[1].split()
    descriptor = (proc / "fd/1").stat()
    log = log_path.stat()
    if fields[0] == "Z" or (descriptor.st_dev, descriptor.st_ino) != (log.st_dev, log.st_ino):
        raise ValueError("DLIO stdout must still be writing the supplied session log")
    return fields[19], log.st_dev, log.st_ino


class ScanLog:
    def __init__(self, stream):
        self.stream = stream
        self.partial = ""
        self.references = OrderedDict()
        # A fresh report only needs the recent scan log, even during long sessions.
        self.stream.seek(max(0, os.fstat(stream.fileno()).st_size - 1_048_576))

    def read(self):
        lines = (self.partial + self.stream.read(1_048_576)).split("\n")
        self.partial = lines.pop()
        for line in lines:
            match = SCAN_PAIR.search(line)
            if match:
                scan, reference = map(int, match.groups())
                self.references[reference] = scan
                if len(self.references) > 1024:
                    self.references.popitem(last=False)


def matrix_product(left, right):
    columns = tuple(zip(*right))
    return tuple(tuple(sum(a * b for a, b in zip(row, column)) for column in columns) for row in left)


def local_body_pose(origin, body):
    dx, dy = body.x_m - origin.x_m, body.y_m - origin.y_m
    sine, cosine = math.sin(origin.yaw_rad), math.cos(origin.yaw_rad)
    angle = body.yaw_rad - origin.yaw_rad
    return BodyPose(cosine * dx + sine * dy, -sine * dx + cosine * dy,
                    math.atan2(math.sin(angle), math.cos(angle)), body.stamp_ns)


@contextmanager
def output_process(args, origin):
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.output_domain), ROS_LOCALHOST_ONLY="1")
    # The console was launched with the default RMW and the localhost option.
    # Source-specific middleware or XML may bind exclusively to the native NIC.
    for name in ("RMW_IMPLEMENTATION", "CYCLONEDDS_URI", "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_STATIC_PEERS"):
        env.pop(name, None)
    command = [sys.executable, str(Path(__file__).resolve()), "--map-yaml", str(args.map_yaml),
               "--output-only", json.dumps([origin.x_m, origin.y_m, origin.yaw_rad])]
    with subprocess.Popen(command, stdin=subprocess.PIPE, bufsize=0, env=env) as child:
        os.set_blocking(child.stdin.fileno(), False)
        try:
            yield child
        finally:
            child.stdin.close()
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                child.kill()


def source_session(args):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    map_from_odom, base_from_lidar, origin, anchor_pairs = load_anchor(
        args.anchor_report, args.map_manifest, args.map_yaml, args.extrinsics,
    )
    source_identity = process_identity(args.dlio_pid, args.dlio_log)
    with args.dlio_log.open(encoding="utf-8", errors="replace") as stream:
        ledger = ScanLog(stream)
        ledger.read()
        if not all(ledger.references.get(ref) == scan for ref, scan in anchor_pairs):
            raise ValueError("Verified anchor pairs are absent from the active DLIO session log")

        rclpy.init(args=[])
        node = rclpy.create_node("a2_localization_display_source", start_parameter_services=False)
        pending = OrderedDict()
        topic = args.source_namespace + "/scan_sync_odom"
        last_stamp = anchor_pairs[-1][0]
        current_time = node.get_clock().now().nanoseconds
        if not 0 <= current_time - last_stamp <= args.max_anchor_age_s * NS_PER_S:
            node.destroy_node()
            rclpy.try_shutdown()
            raise ValueError("The accepted anchor report is outside the live attach window")

        def on_odom(message):
            if message.header.frame_id != ODOM_FRAME or message.child_frame_id != LIDAR_FRAME:
                raise ValueError("Unexpected DLIO odometry frames")
            stamp = message.header.stamp
            ref = stamp.sec * NS_PER_S + stamp.nanosec
            if ref > last_stamp:
                pending[ref] = message.pose.pose
                if len(pending) > 32:
                    pending.popitem(last=False)

        node.create_subscription(Odometry, topic, on_odom, QoSProfile(depth=16, reliability=ReliabilityPolicy.BEST_EFFORT))
        writer = None
        last_valid = time.monotonic_ns()
        try:
            with output_process(args, origin) as child:
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=.04)
                    if child.poll() is not None:
                        raise ValueError(f"Local display publisher exited with status {child.returncode}")
                    if process_identity(args.dlio_pid, args.dlio_log) != source_identity:
                        raise ValueError("DLIO process or log changed, re-anchor required")
                    endpoints = node.get_publishers_info_by_topic(topic)
                    pending_discovery = (not endpoints or (len(endpoints) == 1
                                         and endpoints[0].node_name == "_NODE_NAME_UNKNOWN_"))
                    if pending_discovery and writer is None and time.monotonic_ns() - last_valid < 5 * NS_PER_S:
                        continue
                    names = [f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}" for endpoint in endpoints]
                    if names != [args.source_namespace + "/dlio_odom_node"]:
                        raise ValueError(f"Expected one diagnostic DLIO odometry publisher, found {names}")
                    gid = bytes(endpoints[0].endpoint_gid)
                    if writer is not None and gid != writer:
                        raise ValueError("DLIO publisher changed, re-anchor required")
                    writer = gid
                    streaming = last_stamp > anchor_pairs[-1][0]
                    stream_age = time.monotonic_ns() - last_valid
                    if stream_age > (MAX_STREAM_GAP_NS if streaming else 5 * NS_PER_S):
                        raise ValueError("DLIO stream stopped, re-anchor required")
                    ledger.read()
                    now = node.get_clock().now().nanoseconds
                    for reference in sorted(pending):
                        if reference not in ledger.references and now - reference <= MAX_SAMPLE_AGE_NS:
                            continue
                        pose = pending.pop(reference)
                        if (reference not in ledger.references or reference <= last_stamp
                                or not 0 <= now - reference <= MAX_SAMPLE_AGE_NS):
                            continue
                        if streaming and reference - last_stamp > MAX_STREAM_GAP_NS:
                            raise ValueError("DLIO scan timestamps are discontinuous, re-anchor required")
                        position, rotation = pose.position, pose.orientation
                        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
                        norm = math.hypot(*quaternion)
                        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=.002):
                            raise ValueError("Invalid DLIO odometry rotation")
                        odom_from_lidar = pose_matrix((position.x, position.y, position.z),
                                                     tuple(value / norm for value in quaternion))
                        body = body_pose_from_lidar(matrix_product(map_from_odom, odom_from_lidar),
                                                   base_from_lidar, reference)
                        local = local_body_pose(origin, body)
                        os.write(child.stdin.fileno(), (json.dumps([local.stamp_ns, local.x_m, local.y_m,
                                                                    local.yaw_rad], allow_nan=False) + "\n").encode())
                        if last_stamp == anchor_pairs[-1][0]:
                            print(f"Anchor linked to active DLIO PID {args.dlio_pid} at {reference}", flush=True)
                        last_stamp = reference
                        last_valid = time.monotonic_ns()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


def map_matches(message, expected):
    info = message.info
    p, q = info.origin.position, info.origin.orientation
    x, y, yaw = expected.origin
    return (message.header.frame_id == "map" and (info.width, info.height) == (expected.width, expected.height)
            and math.isclose(info.resolution, expected.resolution, abs_tol=1e-7)
            and all(math.isclose(a, b, abs_tol=1e-7) for a, b in zip((p.x, p.y, p.z), (x, y, 0.0)))
            and math.isclose(math.hypot(q.x, q.y), 0.0, abs_tol=1e-7)
            and math.isclose(math.hypot(q.z, q.w), 1.0, abs_tol=1e-7)
            and math.isclose(abs(q.z * math.sin(yaw / 2) + q.w * math.cos(yaw / 2)), 1.0, abs_tol=1e-7)
            and array("b", message.data).tobytes() == expected.cells)


def output_session(args):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_ros import TransformBroadcaster

    expected = load_trinary_grid(args.map_yaml)
    map_origin = json.loads(args.output_only)
    rclpy.init(args=[])
    node = rclpy.create_node("a2_localization_display", start_parameter_services=False)
    publisher = node.create_publisher(Odometry, "/odom", 16)
    tf = TransformBroadcaster(node)
    map_ready = False
    last_stamp = 0
    partial = b""

    def on_map(message):
        nonlocal map_ready
        if not map_matches(message, expected):
            raise ValueError("The ROS map differs from the verified map export")
        if not map_ready:
            print("Verified local /map for live localization display", flush=True)
        map_ready = True

    def transform(stamp, parent, child, x, y, yaw):
        result = TransformStamped()
        result.header.frame_id, result.child_frame_id = parent, child
        result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, NS_PER_S)
        result.transform.translation.x, result.transform.translation.y = x, y
        result.transform.rotation.z, result.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return result

    node.create_subscription(OccupancyGrid, "/map", on_map, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    ))
    os.set_blocking(sys.stdin.fileno(), False)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.04)
            try:
                chunk = os.read(sys.stdin.fileno(), 8192)
            except BlockingIOError:
                continue
            if not chunk:
                return
            *lines, partial = (partial + chunk).split(b"\n")
            for line in lines:
                stamp, x, y, yaw = json.loads(line)
                if (not map_ready or stamp <= last_stamp
                        or not 0 <= node.get_clock().now().nanoseconds - stamp <= MAX_SAMPLE_AGE_NS):
                    continue
                if (node.count_publishers("/odom") != 1 or node.count_publishers("/tf") != 1
                        or node.count_publishers("/tf_static") != 0 or node.count_publishers("/map") != 1):
                    raise ValueError("Conflicting odometry, TF or map publisher in display ROS graph")
                local = transform(stamp, "odom", "base_footprint", x, y, yaw)
                tf.sendTransform([transform(stamp, "map", "odom", *map_origin), local])
                message = Odometry()
                message.header = local.header
                message.child_frame_id = local.child_frame_id
                message.pose.pose.position.x, message.pose.pose.position.y = x, y
                message.pose.pose.orientation = local.transform.rotation
                # This diagnostic stream does not estimate covariances or body twist.
                message.pose.covariance = [float("nan")] * 36
                message.twist.covariance = [float("nan")] * 36
                for vector in (message.twist.twist.linear, message.twist.twist.angular):
                    vector.x = vector.y = vector.z = float("nan")
                publisher.publish(message)
                last_stamp = stamp
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--anchor-report", type=Path)
    parser.add_argument("--map-manifest", type=Path)
    parser.add_argument("--extrinsics", type=Path)
    parser.add_argument("--dlio-pid", type=int)
    parser.add_argument("--dlio-log", type=Path)
    parser.add_argument("--source-namespace", default="/a2_probe")
    parser.add_argument("--output-domain", type=int, default=63)
    parser.add_argument("--max-anchor-age-s", type=float, default=30.0)
    parser.add_argument("--output-only", help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        if args.output_only is not None:
            output_session(args)
        else:
            if (not all((args.anchor_report, args.map_manifest, args.extrinsics, args.dlio_pid, args.dlio_log))
                    or args.dlio_pid < 1 or not re.fullmatch(r"/[A-Za-z_]\w*(/[A-Za-z_]\w*)*", args.source_namespace)
                    or not 0 <= args.output_domain <= 232 or not 0 < args.max_anchor_age_s <= 120):
                parser.error("Supply live report, map manifest, extrinsics, DLIO PID/log, and a valid domain and namespace")
            source_session(args)
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, ImportError, ValueError, KeyError, IndexError, TypeError) as error:
        print(f"A2 localization display: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
