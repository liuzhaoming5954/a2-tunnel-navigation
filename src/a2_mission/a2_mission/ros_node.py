"""Mock-only ROS adapter for the offline mission core; no hardware capabilities."""

import dataclasses
import math
from pathlib import Path

import rclpy
from a2_interfaces.msg import CommandReceipt, MissionStatus, PlatformState
from a2_interfaces.srv import GetMissionState, MissionCommand
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from a2_storage import RequestConflict, SQLiteJournal, StorageError

from .controller import MissionController
from .mock_adapter import AdapterError, MockMissionAdapter, MockPlatformTracker, MockRequest, load_mock_plan


class MockMissionNode(Node):
    def __init__(self):
        super().__init__("a2_mission_mock")
        immutable = ParameterDescriptor(read_only=True)
        profile = self.declare_parameter("profile", "mock", immutable).value
        robot_id = self.declare_parameter("robot_id", "mock-a2", immutable).value
        database = self.declare_parameter("database_path", "", immutable).value
        plan_path = self.declare_parameter("plan_path", "", immutable).value
        ttl = self.declare_parameter("authorization_ttl_s", 30.0, immutable).value
        timeout = self.declare_parameter("platform_timeout_s", 1.0, immutable).value
        if profile != "mock":
            raise ValueError("mission_node only supports profile=mock")
        if not database or not Path(database).is_absolute() or database == ":memory:":
            raise ValueError("database_path must be an explicit absolute file path")
        if not plan_path or not Path(plan_path).is_absolute():
            raise ValueError("plan_path must be an explicit absolute synthetic-plan path")
        plan = load_mock_plan(plan_path)
        # Fail configuration before opening or modifying any existing journal.
        tracker = MockPlatformTracker(robot_id, timeout)
        if not isinstance(ttl, (float, int)) or not math.isfinite(ttl) or ttl <= 0:
            raise ValueError("authorization_ttl_s must be finite and positive")
        boot_path = Path("/proc/sys/kernel/random/boot_id")
        boot_id = boot_path.read_text().strip()
        if not boot_id:
            raise ValueError("cannot obtain local OS boot identity")
        Path(database).parent.mkdir(parents=True, exist_ok=True)
        self.journal = SQLiteJournal(database)
        try:
            self.controller = MissionController(self.journal, robot_id, boot_id,
                                                authorization_ttl_s=ttl, execution_mode="mock")
        except Exception:
            self.journal.close()
            raise
        self.adapter = MockMissionAdapter(self.controller, plan, tracker)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.status_pub = self.create_publisher(MissionStatus, "/a2/mission/status", qos)
        self.receipt_pub = self.create_publisher(CommandReceipt, "/a2/mission/receipts", qos)
        self.subscription = self.create_subscription(
            PlatformState, "/a2/platform/state", self._platform, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE))
        self.command_service = self.create_service(MissionCommand, "/a2/mission/command", self._command)
        self.query_service = self.create_service(GetMissionState, "/a2/mission/get_state", self._query)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.2, self._tick, clock=self._steady_clock)
        self.get_logger().info("Synthetic mission service ready; simulated=true, motion_capable=false")

    def _platform(self, message):
        self.adapter.tracker.observe(
            robot_id=message.robot_id, source=message.source, simulated=message.simulated,
            motion_capable=message.motion_capable, valid=message.valid,
            source_session_id=message.source_session_id, sample_seq=message.sample_seq,
            received_mono_ns=message.received_mono_ns, age_ms=message.age_ms,
            battery_valid=message.battery_valid, battery_percent=message.battery_percent,
            velocity_valid=message.velocity_valid,
            velocity=(message.body_velocity.linear.x, message.body_velocity.linear.y,
                      message.body_velocity.angular.z), mode_valid=message.mode_valid, reason=message.reason)

    def _status(self, observation):
        status = MissionStatus()
        status.schema_version = 1
        status.header.stamp = self.get_clock().now().to_msg()
        status.robot_id = self.controller.robot_id
        status.simulated = True
        status.motion_capable = False
        status.state = MissionStatus.STATE_UNAVAILABLE
        snapshot = observation.get("snapshot")
        if not snapshot:
            status.reason = ("STORAGE_FAULT_LATCHED" if observation["storage_fault_latched"]
                             else "STATE_UNAVAILABLE")
            return status
        status.boot_id = snapshot["boot_id"]
        status.controller_session_id = snapshot["controller_session_id"]
        mission = snapshot["mission"]
        if mission:
            for name in ("mission_id", "bundle_id", "route_id", "map_version", "config_version"):
                setattr(status, name, mission[name])
            status.state = getattr(MissionStatus, mission["state"])
            status.completed_visits = mission["completed_visits"]
            status.reason = mission["reason"] or ""
        else:
            status.reason = "NO_MISSION"
        if observation["storage_fault_latched"]:
            status.reason = "STORAGE_FAULT_LATCHED"
            if mission:
                # Local inhibition remains true even when its journal write failed.
                status.state = MissionStatus.PAUSED_FAULT
        return status

    @staticmethod
    def _receipt(raw, request_id, mission_id):
        result = CommandReceipt()
        result.schema_version = 1
        result.request_id = request_id
        result.mission_id = mission_id
        result.controller_session_id = raw["controller_session_id"]
        result.accepted = raw["accepted"]
        result.code = raw["code"]
        result.state = (getattr(MissionStatus, raw["state"]) if raw["state"] is not None
                        else CommandReceipt.STATE_UNAVAILABLE)
        return result

    def _observation(self):
        return self.controller.current()

    def _command(self, request, response):
        response.receipt.state = CommandReceipt.STATE_UNAVAILABLE
        response.current = self._status({"snapshot": None, "storage_fault_latched": True})
        try:
            values = {field.name: getattr(request, field.name) for field in dataclasses.fields(MockRequest)}
            reply = self.adapter.execute(MockRequest(**values))
            response.processed = True
            response.receipt = self._receipt(reply["receipt"], request.request_id, request.mission_id)
            response.current = self._status(reply["current"])
            response.has_mission = reply["current"]["snapshot"]["mission"] is not None
            response.authorization_id = reply["receipt"].get("authorization_id", "")
            self.receipt_pub.publish(response.receipt)  # Never contains authorization_id.
        except (AdapterError, RequestConflict, ValueError, StorageError, RuntimeError) as error:
            if isinstance(error, StorageError):
                self.controller.storage_fault_latched = True
            response.error_code = (error.code if isinstance(error, AdapterError) else
                                   "REQUEST_CONFLICT" if isinstance(error, RequestConflict) else
                                   "STORAGE_UNAVAILABLE" if isinstance(error, StorageError) else
                                   "CONTROLLER_INHIBITED" if isinstance(error, RuntimeError) else "INVALID_REQUEST")
            response.error_message = str(error)
            try:
                observation = self._observation()
                response.current = self._status(observation)
                response.has_mission = observation["snapshot"]["mission"] is not None
            except (StorageError, ValueError):
                self.controller.storage_fault_latched = True
        self.status_pub.publish(response.current)
        return response

    def _query(self, request, response):
        del request
        try:
            observation = self._observation()
            response.available = True
            response.has_mission = observation["snapshot"]["mission"] is not None
            response.current = self._status(observation)
            response.storage_fault_latched = observation["storage_fault_latched"]
            response.mock_platform_healthy = self.adapter.healthy()
        except (StorageError, ValueError):
            self.controller.storage_fault_latched = True
            response.available = False
            response.error_code = "STORAGE_UNAVAILABLE"
            response.storage_fault_latched = True
            response.current = self._status({"snapshot": None, "storage_fault_latched": True})
        return response

    def _tick(self):
        try:
            self.adapter.monitor()
            self.status_pub.publish(self._status(self._observation()))
        except (StorageError, ValueError, RuntimeError):
            self.controller.storage_fault_latched = True
            self.status_pub.publish(self._status({"snapshot": None, "storage_fault_latched": True}))

    def destroy_node(self):
        try:
            self.journal.close()
        except StorageError:
            self.get_logger().error("Failed to close mock mission journal")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockMissionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
