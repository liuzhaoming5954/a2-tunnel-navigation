"""Read occupancy, timestamped localization, and measured platform state."""

from collections import deque
from dataclasses import dataclass
from hashlib import sha256
import json
import logging
import math
from threading import Event, Lock, Thread
import time


_NS_PER_S = 1_000_000_000
_NS_PER_MS = 1_000_000
_LOG = logging.getLogger(__name__)


def _duration(value):
    nanoseconds = float(value) * _NS_PER_S
    if not math.isfinite(nanoseconds) or nanoseconds < 1:
        raise ValueError("ROS telemetry maximum ages must be finite and at least one nanosecond")
    return round(nanoseconds)


def add_ros_arguments(parser):
    """Add ROS input options without requiring a ROS environment at import time."""
    group = parser.add_argument_group("ROS 2 telemetry")
    group.add_argument("--map-topic", default="/map")
    group.add_argument("--odom-topic", default="/odom")
    group.add_argument("--platform-topic", default="/a2/platform/state")
    group.add_argument("--map-frame", default="map")
    group.add_argument("--odom-frame", default="odom")
    group.add_argument("--base-frame", default="base_footprint")
    group.add_argument("--map-durability", choices=("transient-local", "volatile"), default="transient-local")
    group.add_argument("--map-reliability", choices=("reliable", "best-effort"), default="reliable")
    group.add_argument("--max-pose-age-s", type=float, default=0.75)
    group.add_argument("--max-platform-age-s", type=float, default=1.0)
    group.add_argument("--use-sim-time", action="store_true", help="Use the ROS /clock timeline for message timestamps")
    group.add_argument("--robot-model", help="Model label for the connected robot, when known")


def _stamp_ns(stamp):
    return stamp.sec * _NS_PER_S + stamp.nanosec


def _normalized_xyzw(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    norm = math.hypot(*values)
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=0.002):
        raise ValueError("Invalid ROS quaternion")
    return tuple(value / norm for value in values)


def _projected_yaw(quaternion, *, planar=False):
    x, y, z, w = _normalized_xyzw(quaternion)
    if planar and math.hypot(x, y) > 1e-6:
        raise ValueError("Occupancy grid origin must be planar")
    forward_x = 1.0 - 2.0 * (y * y + z * z)
    forward_y = 2.0 * (x * y + w * z)
    if math.hypot(forward_x, forward_y) <= 1e-6:
        raise ValueError("No horizontal forward projection")
    angle = math.atan2(forward_y, forward_x)
    if angle == -math.pi:
        return math.pi
    return angle or 0.0


@dataclass(frozen=True, slots=True)
class _Grid:
    metadata: dict
    cells: bytes


def _grid_from_message(message, map_frame):
    info = message.info
    position = info.origin.position
    xyz = (position.x, position.y, position.z)
    if (message.header.frame_id != map_frame or info.width <= 0 or info.height <= 0
            or not math.isfinite(info.resolution) or info.resolution <= 0
            or len(message.data) != info.width * info.height or not all(map(math.isfinite, xyz))):
        raise ValueError("Unexpected occupancy grid frame or geometry")
    if any(value < -1 or value > 100 for value in message.data):
        raise ValueError("Invalid occupancy cell")
    yaw = _projected_yaw(info.origin.orientation, planar=True)
    cells = bytes(value + 1 for value in message.data)
    # The ROS array is already row-major with the bottom row first.
    identity = [map_frame, info.width, info.height, info.resolution, *(value or 0.0 for value in xyz), yaw]
    digest = sha256(json.dumps(identity, separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\0")
    digest.update(cells)
    return _Grid({
        "id": digest.hexdigest(), "frame_id": map_frame, "width": info.width, "height": info.height,
        "resolution_m": info.resolution, "origin_x_m": xyz[0], "origin_y_m": xyz[1], "origin_yaw_rad": yaw,
    }, cells)


def _fresh(stamp_ns, receipt_ns, ros_now_ns, steady_now_ns, max_age_ns, source_age_ns=0):
    ros_age = ros_now_ns - stamp_ns
    steady_age = steady_now_ns - receipt_ns
    return (stamp_ns > 0 and ros_now_ns > 0 and ros_age >= 0 and steady_age >= 0
            and max(ros_age, steady_age) + source_age_ns <= max_age_ns)


class RosTelemetry:
    """Own an executor for receive-only topics and exact-time TF lookups."""

    def __init__(self, args):
        self._args = args
        self._pose_max_ns = _duration(args.max_pose_age_s)
        self._platform_max_ns = _duration(args.max_platform_age_s)
        frames = (args.map_frame, args.odom_frame, args.base_frame)
        if len(set(frames)) != 3 or any(not frame or frame.startswith("/") for frame in frames):
            raise ValueError("Map, odometry, and base frames must be distinct TF names without a leading slash")

        try:
            import rclpy
            from a2_interfaces.msg import PlatformState
            from nav_msgs.msg import OccupancyGrid, Odometry
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.parameter import Parameter
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from rclpy.signals import SignalHandlerOptions
            from rclpy.time import Time
            from tf2_ros import Buffer, TransformException, TransformListener
        except ImportError as exc:
            raise RuntimeError("ROS telemetry requires a sourced ROS 2 and a2_interfaces workspace") from exc

        self._lock = Lock()
        self._stop = Event()
        self._map = None
        self._pose = None
        self._platform = None
        self._robot_id = None
        self._model = args.robot_model
        self._clock_ns = 0
        self._pending = deque(maxlen=128)
        self._last_odom_ns = 0
        self._map_pose_floor_ns = 0
        self._received_map = False
        self._time_type = Time
        self._tf_exception = TransformException
        self._context = Context()
        self._node = self._executor = self._listener = None
        rclpy.init(args=[], context=self._context, signal_handler_options=SignalHandlerOptions.NO)
        try:
            self._node = rclpy.create_node(
                "a2_gateway_telemetry", context=self._context, enable_rosout=False,
                start_parameter_services=False, parameter_overrides=[Parameter("use_sim_time", value=args.use_sim_time)],
            )
            self._buffer = Buffer(node=self._node)
            self._listener = TransformListener(self._buffer, self._node, spin_thread=False)
            self._node.create_subscription(OccupancyGrid, args.map_topic, self._read_map, QoSProfile(
                depth=1,
                durability=(DurabilityPolicy.TRANSIENT_LOCAL if args.map_durability == "transient-local"
                            else DurabilityPolicy.VOLATILE),
                reliability=(ReliabilityPolicy.RELIABLE if args.map_reliability == "reliable"
                             else ReliabilityPolicy.BEST_EFFORT),
            ))
            sample_qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._node.create_subscription(Odometry, args.odom_topic, self._read_odom, sample_qos)
            self._node.create_subscription(PlatformState, args.platform_topic, self._read_platform, sample_qos)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = Thread(target=self._run, name="a2-ros-telemetry", daemon=True)
            self._thread.start()
        except Exception:
            self._release_ros()
            raise

    def _read_map(self, message):
        try:
            grid = _grid_from_message(message, self._args.map_frame)
        except ValueError:
            grid = None
        with self._lock:
            previous_id = self._map.metadata["id"] if self._map else None
            new_id = grid.metadata["id"] if grid else None
            if previous_id == new_id:
                return
            self._map, self._pose = grid, None
        self._pending.clear()
        if self._received_map:
            self._map_pose_floor_ns = max(
                self._last_odom_ns, _stamp_ns(message.header.stamp), self._node.get_clock().now().nanoseconds
            )
        self._received_map = True

    def _read_odom(self, message):
        stamp_ns = _stamp_ns(message.header.stamp)
        if (self._map is None or message.header.frame_id != self._args.odom_frame
                or message.child_frame_id != self._args.base_frame
                or stamp_ns <= max(self._last_odom_ns, self._map_pose_floor_ns)):
            return
        now_ns = self._node.get_clock().now().nanoseconds
        if now_ns <= 0 or abs(now_ns - stamp_ns) > self._pose_max_ns:
            return
        self._last_odom_ns = stamp_ns
        self._pending.append((stamp_ns, time.monotonic_ns()))

    def _read_platform(self, message):
        if message.schema_version != 1 or message.simulated or not message.robot_id:
            return
        stamp_ns = _stamp_ns(message.header.stamp)
        if stamp_ns <= 0:
            return
        now_ns = self._node.get_clock().now().nanoseconds
        if not 0 <= now_ns - stamp_ns <= self._platform_max_ns:
            return
        linear = message.body_velocity.linear.x
        angular = message.body_velocity.angular.z
        values = {
            "mode": str(message.mode) if message.mode_valid else None,
            "battery_percent": float(message.battery_percent)
            if message.battery_valid and 0 <= message.battery_percent <= 100 else None,
            "linear_mps": linear if message.velocity_valid and math.isfinite(linear) else None,
            "angular_radps": angular if message.velocity_valid and math.isfinite(angular) else None,
        }
        # The aggregate valid flag can be false when only some measurements are known.
        available = (message.reason not in ("NO_DATA", "STALE", "SOURCE_TIME_IN_FUTURE")
                     and (message.valid or any(value is not None for value in values.values())))
        with self._lock:
            if self._platform is not None and stamp_ns <= self._platform[0]:
                return
            self._robot_id = message.robot_id
            self._platform = (stamp_ns, time.monotonic_ns(), message.age_ms * _NS_PER_MS, available, values)

    def _resolve_pose(self, now_ns, steady_ns):
        while self._pending and not _fresh(
            self._pending[0][0], self._pending[0][1], now_ns, steady_ns, self._pose_max_ns
        ):
            if self._pending[0][0] > now_ns and steady_ns - self._pending[0][1] <= self._pose_max_ns:
                break
            self._pending.popleft()
        for stamp_ns, receipt_ns in reversed(self._pending):
            if stamp_ns > now_ns:
                continue
            stamp = self._time_type(nanoseconds=stamp_ns, clock_type=self._node.get_clock().clock_type)
            try:
                self._buffer.lookup_transform(self._args.map_frame, self._args.odom_frame, stamp)
                self._buffer.lookup_transform(self._args.odom_frame, self._args.base_frame, stamp)
                transform = self._buffer.lookup_transform(self._args.map_frame, self._args.base_frame, stamp).transform
                x, y, z = transform.translation.x, transform.translation.y, transform.translation.z
                if not all(map(math.isfinite, (x, y, z))):
                    continue
                yaw = _projected_yaw(transform.rotation)
            except (self._tf_exception, ValueError):
                continue
            values = {"x_m": x, "y_m": y, "yaw_rad": yaw, "stamp_ms": stamp_ns // _NS_PER_MS,
                      "map_id": self._map.metadata["id"]}
            with self._lock:
                self._pose = (stamp_ns, receipt_ns, values)
            while self._pending and self._pending[0][0] <= stamp_ns:
                self._pending.popleft()
            break

    def _run(self):
        try:
            while not self._stop.is_set() and self._context.ok():
                self._executor.spin_once(timeout_sec=0.04)
                now_ns = self._node.get_clock().now().nanoseconds
                with self._lock:
                    reset_clock = now_ns < self._clock_ns
                    self._clock_ns = now_ns
                    if reset_clock:
                        self._pose = self._platform = None
                if reset_clock:
                    self._pending.clear()
                    self._buffer.clear()
                    self._last_odom_ns = self._map_pose_floor_ns = 0
                elif self._map is not None:
                    self._resolve_pose(now_ns, time.monotonic_ns())
        except Exception:
            _LOG.exception("ROS telemetry executor stopped")
        finally:
            with self._lock:
                self._pose = self._platform = None
            self._release_ros()

    def snapshot(self):
        """Return a detached summary without the potentially large map cells."""
        with self._lock:
            steady_ns = time.monotonic_ns()
            robot = {"id": self._robot_id, "model": self._model, "online": False, "mode": None,
                     "battery_percent": None, "linear_mps": None, "angular_radps": None,
                     "telemetry_stamp_ms": None, "telemetry_age_ms": None}
            if self._platform:
                stamp_ns, receipt_ns, source_age_ns, available, values = self._platform
                if available and _fresh(stamp_ns, receipt_ns, self._clock_ns, steady_ns,
                                        self._platform_max_ns, source_age_ns):
                    age_ns = max(self._clock_ns - stamp_ns, steady_ns - receipt_ns) + source_age_ns
                    robot.update(values, online=True, telemetry_stamp_ms=stamp_ns // _NS_PER_MS,
                                 telemetry_age_ms=age_ns // _NS_PER_MS)
            pose = None
            if self._pose:
                stamp_ns, receipt_ns, values = self._pose
                if _fresh(stamp_ns, receipt_ns, self._clock_ns, steady_ns, self._pose_max_ns):
                    age_ns = max(self._clock_ns - stamp_ns, steady_ns - receipt_ns)
                    pose = {**values, "age_ms": age_ns // _NS_PER_MS}
            return {"robot": robot, "pose": pose, "map": self._map.metadata.copy() if self._map else None}

    def map_data(self):
        """Return metadata and bottom-up row-major occupancy values."""
        with self._lock:
            grid = self._map
        return {**grid.metadata, "cells": [value - 1 for value in grid.cells]} if grid else None

    def _release_ros(self):
        if self._listener is not None:
            self._listener.unregister()
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        self._context.try_shutdown()

    def close(self):
        """Stop the executor and release its private ROS context."""
        self._stop.set()
        self._thread.join()
