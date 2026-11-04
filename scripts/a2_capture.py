#!/usr/bin/env python3
"""Read-only A2 ROS inventory and explicit sensor-only rosbag recording.

No SDK, motion publisher, service client, driver startup or network setup.
Only the `record --execute` subcommand creates a recording session.
"""

import argparse
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone


ROLE_TYPES = {
    "front_points": "sensor_msgs/msg/PointCloud2",
    "imu": "sensor_msgs/msg/Imu",
    "rear_points": "sensor_msgs/msg/PointCloud2",
    "image": "sensor_msgs/msg/Image",
    "camera_info": "sensor_msgs/msg/CameraInfo",
}
TF_TYPES = {"/tf": "tf2_msgs/msg/TFMessage", "/tf_static": "tf2_msgs/msg/TFMessage"}
TOPIC_NAME = re.compile(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*\Z")
UNITS = {"s", "ms", "us", "ns"}


def load_config(path):
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(config, dict) or type(config.get("schema_version")) is not int or config["schema_version"] != 1:
        raise ValueError("config.schema_version must be 1")
    if set(config) != {"schema_version", "topics", "point_time", "storage_id"}:
        raise ValueError("config requires schema_version, topics, point_time, storage_id only")
    topics = config["topics"]
    if not isinstance(topics, dict) or not {"front_points", "imu"} <= topics.keys():
        raise ValueError("topics must map at least front_points and imu to actual ROS topic names")
    if not topics.keys() <= ROLE_TYPES.keys():
        raise ValueError("unknown sensor role; arbitrary/control topics are not accepted")
    for topic in topics.values():
        if not isinstance(topic, str) or not TOPIC_NAME.fullmatch(topic) or topic in TF_TYPES:
            raise ValueError("sensor topics must be explicit absolute ROS names, excluding /tf and /tf_static")
    if len(set(topics.values())) != len(topics):
        raise ValueError("each sensor role must have a distinct topic")
    point_time = config["point_time"]
    if not isinstance(point_time, dict) or set(point_time) != {"field", "unit"}:
        raise ValueError("point_time must contain field and unit")
    field, unit = point_time["field"], point_time["unit"]
    if not isinstance(field, str) or not isinstance(unit, str):
        raise ValueError("point_time.field and unit must be strings")
    if (field or unit) and (not field.strip() or unit not in UNITS):
        raise ValueError("point_time needs a field and unit s/ms/us/ns, or both empty for investigation")
    if config["storage_id"] not in {"sqlite3", "mcap"}:
        raise ValueError("storage_id must be sqlite3 or mcap")
    return config


def environment():
    os_release = Path("/etc/os-release")
    return {
        "collected_utc": datetime.now(timezone.utc).isoformat(),
        "platform": platform.platform(), "machine": platform.machine(),
        "python": platform.python_version(),
        "os_release": os_release.read_text() if os_release.exists() else None,
        "ros_environment": {key: os.environ.get(key) for key in (
            "ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY",
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_STATIC_PEERS")},
        "limits": "Read-only observation; not proof of synchronization, calibration or mapping accuracy.",
    }


def write_json(path, value):
    path = Path(path)
    # Exclusive creation prevents accidental loss of an earlier evidence report.
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def message_summary(message, type_name):
    result = {}
    if hasattr(message, "header"):
        result["frame_id"] = message.header.frame_id
        stamp = message.header.stamp
        result["header_stamp_ns"] = stamp.sec * 1_000_000_000 + stamp.nanosec
    if type_name == ROLE_TYPES["front_points"]:
        result.update(width=message.width, height=message.height,
                      point_step=message.point_step, row_step=message.row_step,
                      is_bigendian=message.is_bigendian, data_bytes=len(message.data),
                      fields=[{"name": field.name, "offset": field.offset,
                               "datatype": field.datatype, "count": field.count}
                              for field in message.fields])
    elif type_name == ROLE_TYPES["imu"]:
        numbers = [getattr(vector, axis) for vector in
                   (message.angular_velocity, message.linear_acceleration) for axis in "xyz"]
        result["six_axis_finite"] = all(math.isfinite(value) for value in numbers)
        result["orientation_covariance_first"] = (
            message.orientation_covariance[0] if math.isfinite(message.orientation_covariance[0]) else None)
    elif type_name == ROLE_TYPES["image"]:
        result.update(width=message.width, height=message.height, encoding=message.encoding)
    elif type_name == ROLE_TYPES["camera_info"]:
        result.update(width=message.width, height=message.height,
                      distortion_model=message.distortion_model)
    elif type_name == TF_TYPES["/tf"]:
        result["edges"] = [{"parent": tf.header.frame_id, "child": tf.child_frame_id}
                           for tf in message.transforms]
    return result


def observe(config, discovery_s, sample_s):
    # Imported lazily so --help, config validation and command preview need no ROS.
    import rclpy
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy, qos_profile_sensor_data)
    from rosidl_runtime_py.utilities import get_message

    rclpy.init(args=[])
    node = rclpy.create_node(f"a2_capture_observer_{os.getpid()}", enable_rosout=False,
                            start_parameter_services=False)
    subscriptions = []
    try:
        def spin_for(seconds):
            deadline = time.monotonic() + seconds
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(0.1, max(0, deadline - time.monotonic())))

        spin_for(discovery_s)
        graph = {}
        for name, types in sorted(node.get_topic_names_and_types()):
            publishers = []
            for endpoint in node.get_publishers_info_by_topic(name):
                qos = endpoint.qos_profile
                publishers.append({"node": endpoint.node_name, "namespace": endpoint.node_namespace,
                                   "type": endpoint.topic_type,
                                   "qos": {"reliability": qos.reliability.name,
                                           "durability": qos.durability.name,
                                           "history": qos.history.name, "depth": qos.depth}})
            graph[name] = {"types": types, "publishers": publishers}
        report = {"environment": environment(), "graph": graph, "observations": {},
                  "errors": [], "warnings": []}
        if config is None:
            return report
        wanted = {topic: ROLE_TYPES[role] for role, topic in config["topics"].items()}
        wanted.update(TF_TYPES)
        for topic, expected in wanted.items():
            actual = graph.get(topic, {})
            if actual.get("types") != [expected] or not actual.get("publishers"):
                target = "warnings" if topic in TF_TYPES else "errors"
                report[target].append(f"{topic}: expected live {expected}; found {actual or 'nothing'}")
                continue
            qos = (QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if topic == "/tf_static" else qos_profile_sensor_data)
            observation = {"messages": 0}
            report["observations"][topic] = observation

            def receive(message, topic=topic, expected=expected, observation=observation):
                observation["messages"] += 1
                summary = message_summary(message, expected)
                if "first" not in observation:
                    observation["first"] = summary
                observation["last"] = summary

            subscriptions.append(node.create_subscription(get_message(expected), topic, receive, qos))
        spin_for(sample_s)
        for topic in wanted:
            if report["observations"].get(topic, {}).get("messages", 0) == 0:
                target = "warnings" if topic in TF_TYPES else "errors"
                report[target].append(f"{topic}: no message received in {sample_s:g}s (check driver, QoS, domain and NIC)")
        for role, topic in config["topics"].items():
            summary = report["observations"].get(topic, {}).get("last", {})
            if summary and not summary.get("frame_id"):
                report["warnings"].append(f"{topic}: empty frame_id; resolve before mapping")
            if role == "imu" and summary.get("six_axis_finite") is False:
                report["errors"].append(f"{topic}: non-finite IMU acceleration/angular velocity")
            if role in {"front_points", "rear_points"} and summary:
                names = {field["name"] for field in summary["fields"]}
                if not {"x", "y", "z"} <= names or not summary["data_bytes"]:
                    report["errors"].append(f"{topic}: empty point cloud or missing xyz fields")
                field = config["point_time"]["field"]
                if field and field not in names:
                    report["warnings"].append(f"{topic}: configured point time field {field!r} absent; retain raw data for investigation")
        report["warnings"].append("Header timing, point time units, source clocks and extrinsics need separate verification; this is not DLIO acceptance.")
        if not config["point_time"]["field"]:
            report["warnings"].append("Point time field/unit unknown; raw capture allowed, DLIO deskew input not verified.")
        return report
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def record_command(config, session, help_text):
    required_options = ("--qos-profile-overrides-path", "--storage", "--max-bag-size")
    if any(option not in help_text for option in required_options):
        raise ValueError("ros2 bag record CLI lacks required options; inspect this ROS installation before recording")
    command = ["ros2", "bag", "record", "-o", str(session / "bag"),
               "--storage", config["storage_id"], "--max-bag-size", str(2 * 1024**3),
               "--qos-profile-overrides-path", str(session / "qos.yaml")]
    if "--topics" in help_text:
        command.append("--topics")
    return command + list(config["topics"].values()) + list(TF_TYPES)


def run_recorder(command, duration, disk_path, min_free_bytes):
    """Stop with SIGINT so rosbag can finalize metadata, including on Ctrl-C."""
    process = subprocess.Popen(command, start_new_session=True)
    stop_reason = "recorder_exit"
    try:
        deadline = time.monotonic() + duration
        try:
            while process.poll() is None:
                if shutil.disk_usage(disk_path).free < min_free_bytes:
                    stop_reason = "disk_reserve"
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    stop_reason = "duration_elapsed"
                    break
                try:
                    process.wait(timeout=min(1, remaining))
                except subprocess.TimeoutExpired:
                    pass
        except KeyboardInterrupt:
            stop_reason = "operator_interrupt"
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=30)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                stop_reason += "_shutdown_incomplete"
    finally:
        if process.poll() is None:
            # Do not silently leave an orphan recorder consuming robot storage.
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    return process.returncode, stop_reason


def positive_number(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)
    inventory = subparsers.add_parser("inventory", help="observe environment, topic types/QoS, and optionally sensor samples")
    inventory.add_argument("--config", type=Path)
    inventory.add_argument("--output", type=Path, required=True, help="new JSON file; parent must exist")
    record = subparsers.add_parser("record", help="preview or execute a sensor-only recording session")
    record.add_argument("--config", type=Path, required=True)
    record.add_argument("--session", type=Path, required=True, help="new session directory; parent must exist")
    record.add_argument("--duration", type=positive_number, default=60, help="seconds, including optional early Ctrl-C (default 60)")
    record.add_argument("--min-free-gib", type=positive_number, default=5)
    record.add_argument("--execute", action="store_true", help="run live preflight and recorder; default only previews the command")
    for child in (inventory, record):
        child.add_argument("--discovery-seconds", type=positive_number, default=3)
        child.add_argument("--sample-seconds", type=positive_number, default=3)
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config) if args.config else None
        if args.action == "inventory":
            if args.output.exists():
                raise ValueError(f"refusing to overwrite {args.output}")
            report = observe(config, args.discovery_seconds, args.sample_seconds)
            write_json(args.output, report)
            print(f"Saved {args.output}; {len(report['graph'])} topics; {len(report['errors'])} errors.")
            return 1 if report["errors"] else 0
        session = args.session.expanduser().resolve()
        if session.exists() or not session.parent.is_dir():
            raise ValueError("session must not exist and its parent must already be a directory")
        cli_help = subprocess.run(["ros2", "bag", "record", "--help"], check=True,
                                  capture_output=True, text=True, timeout=15).stdout
        command = record_command(config, session, cli_help)
        print(shlex.join(command), flush=True)
        if not args.execute:
            print("Preview only. --execute performs live preflight and records; no files created.")
            return 0
        if os.environ.get("ROS_LOCALHOST_ONLY") == "1":
            print("ROS_LOCALHOST_ONLY=1: only local publishers can be discovered; check this matches the robot topology.", file=sys.stderr)
        if shutil.disk_usage(session.parent).free < args.min_free_gib * 1024**3:
            raise ValueError("insufficient free disk for configured minimum; free space and recheck expected capture size")
        report = observe(config, args.discovery_seconds, args.sample_seconds)
        for warning in report["warnings"]:
            print(f"WARNING: {warning}", file=sys.stderr)
        if report["errors"]:
            for error in report["errors"]:
                print(f"ERROR: {error}", file=sys.stderr)
            return 1
        session.mkdir()
        write_json(session / "config.json", config)
        write_json(session / "preflight.json", report)
        (session / "qos.yaml").write_text(
            "/tf_static:\n  reliability: reliable\n  durability: transient_local\n"
            "  history: keep_all\n", encoding="utf-8")
        write_json(session / "recording.json", {
            "started_utc": datetime.now(timezone.utc).isoformat(), "argv": command,
            "duration_limit_s": args.duration,
            "note": "Sensor recorder only; operator uses OEM controls independently. Not mapping acceptance.",
        })
        try:
            code, reason = run_recorder(command, args.duration, session.parent,
                                        args.min_free_gib * 1024**3)
        except (OSError, subprocess.SubprocessError) as exc:
            write_json(session / "recording-result.json", {
                "ended_utc": datetime.now(timezone.utc).isoformat(), "error": str(exc),
                "stop_reason": "recorder_error",
                "metadata_present": (session / "bag" / "metadata.yaml").is_file(),
            })
            raise
        write_json(session / "recording-result.json", {
            "ended_utc": datetime.now(timezone.utc).isoformat(), "return_code": code,
            "stop_reason": reason, "metadata_present": (session / "bag" / "metadata.yaml").is_file(),
        })
        print(f"Recorder stopped ({reason}, code {code}). Inspect actual messages before accepting this bag.")
        normal_stop = reason in {"recorder_exit", "duration_elapsed", "operator_interrupt"}
        return 0 if normal_stop and code in (0, 130, -signal.SIGINT) and (session / "bag" / "metadata.yaml").is_file() else 1
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
