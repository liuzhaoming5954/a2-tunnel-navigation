#!/usr/bin/env python3
"""Read actual ROS 2 bag messages and report mapping input integrity.

Only the configured sensor topics and /tf[/_static] are deserialized. No node,
publisher, SDK, image decoder or network connection is created. Passing these
structural checks does not validate clock synchronization, calibration or SLAM.
"""

import argparse
import json
import math
import sys
from pathlib import Path


ROLE_TYPES = {
    "front_points": {"sensor_msgs/msg/PointCloud2"},
    "rear_points": {"sensor_msgs/msg/PointCloud2"},
    "imu": {"sensor_msgs/msg/Imu"},
    "image": {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"},
    "camera_info": {"sensor_msgs/msg/CameraInfo"},
}
FIELD_BYTES = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
LIMITATIONS = [
    "Point time units are declared by the configuration, not measured by this tool.",
    "Header time and bag receive time are different clocks; their offsets do not prove synchronization.",
    "No check here establishes IMU axes/gravity, scan reference time, calibration accuracy or DLIO readiness.",
    "Cloud field layout is checked; individual XYZ/time values and their numerical semantics are not evaluated.",
    "TF edges are listed; their calibration accuracy and required transform chains need separate review.",
    "A structurally usable bag does not establish mapping quality or permission for autonomous motion.",
]


def load_config(path):
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    if (not isinstance(config, dict) or type(config.get("schema_version")) is not int
            or config["schema_version"] != 1):
        raise ValueError("config.schema_version must be 1")
    topics = config.get("topics")
    if not isinstance(topics, dict) or not {"front_points", "imu"} <= set(topics):
        raise ValueError("config.topics requires front_points and imu")
    if set(topics) - set(ROLE_TYPES):
        raise ValueError("config.topics contains an unknown role")
    for topic in topics.values():
        if not isinstance(topic, str) or not topic.startswith("/") or topic in {"/", "/tf", "/tf_static"}:
            raise ValueError("sensor topic names must be absolute and distinct from TF")
        if any(char.isspace() for char in topic):
            raise ValueError("topic names cannot contain whitespace")
    if len(set(topics.values())) != len(topics):
        raise ValueError("each sensor role requires a distinct topic")
    point_time = config.get("point_time")
    if (not isinstance(point_time, dict) or not isinstance(point_time.get("field"), str)
            or not isinstance(point_time.get("unit"), str)
            or point_time["unit"] not in {"", "s", "ms", "us", "ns"}):
        raise ValueError("point_time requires a field string and unit (s, ms, us, ns or blank)")
    if not isinstance(config.get("storage_id"), str) or config["storage_id"] not in {"sqlite3", "mcap"}:
        raise ValueError("storage_id must be sqlite3 or mcap")
    return config


class Timeline:
    """Constant-memory timing statistics in message encounter order."""

    def __init__(self):
        self.count = 0
        self.first = self.last = self.minimum = self.maximum = None
        self.duplicates = self.backwards = self.zero = 0
        self.maximum_forward_gap = 0

    def add(self, value):
        if self.count:
            difference = value - self.last
            self.duplicates += difference == 0
            self.backwards += difference < 0
            self.maximum_forward_gap = max(self.maximum_forward_gap, difference)
        else:
            self.first = self.minimum = self.maximum = value
        self.last = value
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)
        self.zero += value == 0
        self.count += 1

    def report(self):
        span = None if not self.count else self.maximum - self.minimum
        return {
            "count": self.count, "first_ns": self.first, "last_ns": self.last,
            "minimum_ns": self.minimum, "maximum_ns": self.maximum,
            "zero_count": self.zero, "duplicate_count": self.duplicates,
            "backwards_count": self.backwards,
            "maximum_forward_gap_ns": self.maximum_forward_gap,
            "span_average_hz": ((self.count - 1) * 1e9 / span) if span and self.count > 1 else None,
        }


class BagChecks:
    def __init__(self, config, topic_types):
        self.config = config
        self.roles = {topic: role for role, topic in config["topics"].items()}
        self.expected = {topic: ROLE_TYPES[role] for topic, role in self.roles.items()}
        self.expected.update({"/tf": {"tf2_msgs/msg/TFMessage"},
                              "/tf_static": {"tf2_msgs/msg/TFMessage"}})
        self.topic_types = topic_types
        self.stats = {}
        self.issues = {}
        self.warnings = {}
        self.edges = {}
        if not config["point_time"]["field"] or not config["point_time"]["unit"]:
            self.note(config["topics"]["front_points"], "point_time_unresolved",
                      "Point time field/units are not yet identified; DLIO input suitability remains unresolved")
        for topic, types in self.expected.items():
            if topic not in topic_types:
                # Some systems supply extrinsics in a separate reviewed calibration file.
                self.note(topic, "missing_topic", "No topic stored in this bag",
                          warning=topic in {"/tf", "/tf_static"})
            elif topic_types[topic] not in types:
                self.note(topic, "unexpected_type", f"Expected {sorted(types)}, got {topic_types[topic]}")

    def note(self, topic, code, detail, warning=False):
        records = self.warnings if warning else self.issues
        key = (topic, code)
        if key not in records:
            records[key] = {"topic": topic, "code": code, "count": 0, "detail": detail}
        records[key]["count"] += 1

    def stat(self, topic):
        if topic not in self.stats:
            self.stats[topic] = {
                "count": 0, "bag_time": Timeline(), "header_time": Timeline(),
                "frames": set(), "layouts": {}, "point_count_min": None,
                "point_count_max": 0, "imu_ranges": {},
            }
        return self.stats[topic]

    def stamp(self, stamp, topic):
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            self.note(topic, "invalid_stamp", "Negative seconds or nanoseconds outside [0, 1e9)")
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def frame(self, frame, topic):
        stats = self.stat(topic)
        if not frame:
            self.note(topic, "empty_frame", "A frame_id is empty")
        if len(stats["frames"]) < 64:
            stats["frames"].add(frame)
        elif frame not in stats["frames"]:
            self.note(topic, "too_many_frames", "Only the first 64 distinct frame IDs are retained")

    def consume(self, topic, message, bag_time_ns):
        if topic not in self.expected:
            return
        stats = self.stat(topic)
        stats["count"] += 1
        stats["bag_time"].add(int(bag_time_ns))
        if topic in {"/tf", "/tf_static"}:
            self.consume_tf(topic, message)
            return
        stats["header_time"].add(self.stamp(message.header.stamp, topic))
        self.frame(message.header.frame_id, topic)
        role = self.roles[topic]
        if role in {"front_points", "rear_points"}:
            self.consume_cloud(topic, message)
        elif role == "imu":
            self.consume_imu(topic, message)
        elif role == "image":
            if not len(message.data):
                self.note(topic, "empty_image", "Image payload is empty")
        elif role == "camera_info":
            if message.width <= 0 or message.height <= 0:
                self.note(topic, "invalid_image_size", "CameraInfo width and height must be positive")
            if (not all(math.isfinite(value) for value in message.k)
                    or message.k[0] <= 0 or message.k[4] <= 0):
                self.note(topic, "invalid_intrinsics", "CameraInfo K contains nonfinite or nonpositive focal lengths")

    def consume_cloud(self, topic, message):
        stats = self.stat(topic)
        layout = {
            "fields": [{"name": field.name, "offset": field.offset,
                        "datatype": field.datatype, "count": field.count} for field in message.fields],
            "point_step": message.point_step, "is_bigendian": message.is_bigendian,
        }
        signature = json.dumps(layout, sort_keys=True)
        if signature not in stats["layouts"]:
            if len(stats["layouts"]) < 16:
                stats["layouts"][signature] = layout
            else:
                self.note(topic, "too_many_layouts", "Only the first 16 cloud layouts are retained")
        points = message.width * message.height
        stats["point_count_min"] = points if stats["point_count_min"] is None else min(stats["point_count_min"], points)
        stats["point_count_max"] = max(stats["point_count_max"], points)
        if not points:
            self.note(topic, "empty_cloud", "Point cloud has no points")
        if (message.point_step <= 0 or message.row_step < message.width * message.point_step
                or len(message.data) != message.row_step * message.height):
            self.note(topic, "invalid_cloud_layout", "Point step, row step or payload length is inconsistent")
        fields = {field.name: field for field in message.fields}
        if len(fields) != len(message.fields):
            self.note(topic, "duplicate_field", "PointCloud2 field names are repeated")
        for field in message.fields:
            size = FIELD_BYTES.get(field.datatype)
            if (size is None or field.count <= 0 or field.offset < 0
                    or field.offset + (size or 0) * field.count > message.point_step):
                self.note(topic, "invalid_field_layout", f"Field {field.name!r} does not fit in point_step")
        required = ["x", "y", "z"]
        if self.config["point_time"]["field"]:
            required.append(self.config["point_time"]["field"])
        for name in required:
            field = fields.get(name)
            if field is None:
                self.note(topic, "missing_point_field", f"Required field {name!r} is missing")
            elif field.count != 1 or field.datatype not in FIELD_BYTES:
                self.note(topic, "invalid_point_field", f"Required field {name!r} must be a scalar numeric field")

    def consume_imu(self, topic, message):
        stats = self.stat(topic)
        for name in ("angular_velocity", "linear_acceleration"):
            values = [getattr(getattr(message, name), axis) for axis in ("x", "y", "z")]
            if not all(math.isfinite(value) for value in values):
                self.note(topic, "nonfinite_imu", f"{name} contains a nonfinite value")
            else:
                ranges = stats["imu_ranges"].setdefault(name, [[value, value] for value in values])
                for bounds, value in zip(ranges, values):
                    bounds[0] = min(bounds[0], value)
                    bounds[1] = max(bounds[1], value)
            covariance = getattr(message, name + "_covariance")
            if covariance[0] == -1:
                self.note(topic, "imu_channel_unavailable", f"{name}_covariance[0] declares the measurement unavailable")
        # Orientation may legitimately be unavailable for a six-axis IMU.

    def consume_tf(self, topic, message):
        if not message.transforms:
            self.note(topic, "empty_tf_message", "TF message contains no transforms", warning=True)
        for transform in message.transforms:
            parent, child = transform.header.frame_id, transform.child_frame_id
            key = (topic, parent, child)
            if key not in self.edges:
                if len(self.edges) >= 1024:
                    self.note(topic, "too_many_tf_edges", "Only the first 1024 transform edges are retained")
                    continue
                self.edges[key] = Timeline()
            self.edges[key].add(self.stamp(transform.header.stamp, topic))
            if not parent or not child or parent == child:
                self.note(topic, "invalid_tf_frame", "Transform frame names must be nonempty and distinct")
            translation, rotation = transform.transform.translation, transform.transform.rotation
            values = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
            if not all(math.isfinite(value) for value in values):
                self.note(topic, "nonfinite_tf", "Transform contains a nonfinite value")
            elif sum(value * value for value in values[3:]) <= 1e-12:
                self.note(topic, "invalid_tf_rotation", "Transform quaternion has zero norm")

    def report(self):
        topics = {}
        for topic in self.expected:
            stats = self.stat(topic)
            if topic in self.topic_types and stats["count"] == 0:
                self.note(topic, "no_messages", "Topic exists but no actual messages were read",
                          warning=topic in {"/tf", "/tf_static"})
            header = stats["header_time"]
            if header.count:
                if header.backwards:
                    self.note(topic, "backwards_header_stamp", "Header timestamps move backwards in bag encounter order")
                if header.zero:
                    self.note(topic, "zero_header_stamp", "Sensor header has a zero timestamp")
                if header.duplicates:
                    self.note(topic, "duplicate_header_stamp", "Repeated header timestamps require review", warning=True)
                if len(stats["frames"]) > 1:
                    self.note(topic, "changing_sensor_frame", "A sensor topic uses more than one frame_id")
            if stats["bag_time"].backwards:
                self.note(topic, "backwards_bag_time", "Bag receive timestamps move backwards", warning=True)
            topics[topic] = {
                "role": self.roles.get(topic, "tf_static" if topic == "/tf_static" else "tf"),
                "type": self.topic_types.get(topic), "message_count": stats["count"],
                "bag_receive_time": stats["bag_time"].report(),
                "header_time": header.report(), "frame_ids": sorted(stats["frames"]),
            }
            if stats["layouts"]:
                topics[topic].update({"cloud_layouts": list(stats["layouts"].values()),
                                      "point_count_min": stats["point_count_min"],
                                      "point_count_max": stats["point_count_max"]})
            if stats["imu_ranges"]:
                topics[topic]["imu_axis_min_max"] = stats["imu_ranges"]
        for (topic, parent, child), timeline in self.edges.items():
            if topic == "/tf" and timeline.backwards:
                self.note(topic, "backwards_tf_stamp", f"Transform {parent} -> {child} moves backwards")
        return {
            "schema_version": 1,
            "assessment": "issues_found" if self.issues else "structural_checks_passed_requires_manual_review",
            "point_time_declared": self.config["point_time"],
            "topics": topics,
            "tf_edges": [{"topic": topic, "parent": parent, "child": child, "time": timeline.report()}
                         for (topic, parent, child), timeline in sorted(self.edges.items())],
            "issues": list(self.issues.values()), "warnings": list(self.warnings.values()),
            "limitations": LIMITATIONS,
        }


def inspect_bag(bag_path, config):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(f"ROS 2 Python packages unavailable; source the ROS environment: {exc}") from exc
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=config["storage_id"]),
                rosbag2_py.ConverterOptions("", ""))
    metadata = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in metadata}
    checks = BagChecks(config, topic_types)
    selected = [topic for topic, allowed in checks.expected.items() if topic_types.get(topic) in allowed]
    classes = {topic: get_message(topic_types[topic]) for topic in selected}
    if selected:
        reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic not in classes:
                continue
            try:
                message = deserialize_message(data, classes[topic])
                checks.consume(topic, message, timestamp)
            except (ValueError, TypeError, RuntimeError, AttributeError, IndexError, OverflowError) as exc:
                checks.note(topic, "message_decode_or_structure_error", str(exc)[:300])
    report = checks.report()
    report["stored_topic_types"] = topic_types
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Completed ROS 2 bag directory or supported bag file")
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="New JSON file outside the raw bag directory")
    args = parser.parse_args(argv)
    try:
        bag = args.bag.resolve(strict=True)
        output = args.output.absolute()
        if output.exists():
            raise ValueError(f"Output already exists: {output}")
        if bag.is_dir() and (output.resolve() == bag or bag in output.resolve().parents):
            raise ValueError("Output must be outside the raw bag directory")
        config = load_config(args.config)
        report = inspect_bag(bag, config)
        report["bag"] = str(bag)
        report["config"] = str(args.config.resolve())
        with output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
        print(f"{report['assessment']}: {len(report['issues'])} issue categories, "
              f"{len(report['warnings'])} warning categories; report: {output}")
        return 1 if report["issues"] else 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
