#!/usr/bin/env python3
"""Finite mock-only ROS mission lifecycle and restart check.

Build/source the workspace, set ROS_LOCALHOST_ONLY=1 and an isolated
ROS_DOMAIN_ID, then run from the project root. The script owns a temporary
SQLite database and launches only a2_bringup/mock.launch.py with profile=mock.
No arbitrary ROS arguments, real source selection or hardware access is exposed.
"""

import argparse
from collections import deque
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from readonly_smoke import stop_process_group


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        print("ERROR: set ROS_LOCALHOST_ONLY=1 for this local mock check.", file=sys.stderr)
        return 2
    if not os.environ.get("ROS_DOMAIN_ID", "").isdecimal():
        print("ERROR: set an explicit isolated ROS_DOMAIN_ID.", file=sys.stderr)
        return 2
    try:
        import rclpy
        from a2_interfaces.msg import CommandReceipt, MissionStatus, PlatformState
        from a2_interfaces.srv import GetMissionState, MissionCommand
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        print(f"ERROR: mission interfaces unavailable ({exc}); build and source the workspace first.",
              file=sys.stderr)
        return 2

    robot_id = f"mission-smoke-{os.getpid()}"
    mission_id = f"smoke-mission-{os.getpid()}"
    observed = {"error": None, "platform_session": None, "platform_received": 0.0,
                "states": set(), "receipts": 0}
    public_text = deque(maxlen=200)
    credentials = set()
    # Includes three cold starts, service rediscovery and the dropout fixture;
    # this is a test harness budget, not a robot health or stop deadline.
    deadline = time.monotonic() + 90.0
    process = None
    node = None
    initialized = False
    failure = None
    calls = 0
    node_log = tempfile.TemporaryFile(mode="w+t", encoding="utf-8")
    data_directory = tempfile.TemporaryDirectory(prefix="a2-mission-ros-smoke-")

    def inspect_public(message):
        text = str(message)
        public_text.append(text)
        if hasattr(message, "authorization_id") or any(token in text for token in credentials):
            observed["error"] = "Admission credential appeared on a public observation topic"

    def mission_callback(message):
        if message.robot_id != robot_id:
            return
        inspect_public(message)
        if not message.simulated or message.motion_capable:
            observed["error"] = "Mission status is not explicitly mock-only"
        if message.mission_id == mission_id:
            observed["states"].add(message.state)

    def receipt_callback(message):
        if message.mission_id == mission_id:
            inspect_public(message)
            observed["receipts"] += 1

    def platform_callback(message):
        if message.robot_id != robot_id:
            return
        if message.source != "mock" or not message.simulated or message.motion_capable:
            observed["error"] = "Bringup loaded a non-mock platform state source"
            return
        if message.valid:
            observed["platform_session"] = message.source_session_id
            observed["platform_received"] = time.monotonic()

    def check_running():
        if observed["error"]:
            raise RuntimeError(observed["error"])
        if process is not None and process.poll() is not None:
            raise RuntimeError(f"Mock bringup exited early with status {process.returncode}")
        if time.monotonic() >= deadline:
            raise RuntimeError("Mission smoke exceeded its 90s observation budget")

    def wait_for(predicate, description, timeout=10.0):
        until = min(deadline, time.monotonic() + timeout)
        while time.monotonic() < until:
            check_running()
            if predicate():
                return
            rclpy.spin_once(node, timeout_sec=0.05)
        raise RuntimeError(f"Timed out waiting for {description}")

    def call(client, request, description, timeout=3.0):
        # DDS discovery has a separate startup budget; actual RPC responses
        # retain their original deadline once the service is discovered.
        wait_for(client.service_is_ready, f"{description} service", max(10.0, timeout))
        future = client.call_async(request)
        until = min(deadline, time.monotonic() + timeout)
        while not future.done() and time.monotonic() < until:
            check_running()
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            future.cancel()
            raise RuntimeError(f"Timed out calling {description}")
        response = future.result()
        if response is None:
            raise RuntimeError(f"No response from {description}")
        return response

    def query(expected_session=None, different_session=None):
        response = call(query_client, GetMissionState.Request(), "mission query")
        current = response.current
        if not response.available or current.robot_id != robot_id:
            return None
        if expected_session and current.controller_session_id != expected_session:
            return None
        if different_session and current.controller_session_id == different_session:
            return None
        if not current.simulated or current.motion_capable or response.storage_fault_latched:
            raise RuntimeError("Query returned a non-mock or storage-faulted controller")
        return response

    def command(label, kind, expected_state=None, accepted=True, **fields):
        nonlocal calls
        request = MissionCommand.Request()
        request.schema_version = 1
        request.request_id = f"smoke-{label}"
        request.command = kind
        request.mission_id = mission_id
        for key, value in fields.items():
            setattr(request, key, value)
        response = call(command_client, request, f"mission {kind}")
        calls += 1
        if response.current.robot_id != robot_id:
            raise RuntimeError(f"{kind} response belongs to another controller; use an isolated ROS domain")
        if not response.current.simulated or response.current.motion_capable:
            raise RuntimeError(f"{kind} response is not mock-only")
        succeeded = response.processed and response.receipt.accepted
        if succeeded != accepted:
            raise RuntimeError(f"{kind}: unexpected acceptance; processed={response.processed}, "
                               f"code={response.error_code or response.receipt.code}")
        if expected_state is not None and response.current.state != expected_state:
            raise RuntimeError(f"{kind}: expected state {expected_state}, got {response.current.state}")
        return response

    def authorize(label):
        response = command(label, "authorize", operator_id="mock-operator", local_confirmation=True)
        token = response.authorization_id
        if not token:
            raise RuntimeError("Authorize succeeded without a direct-response credential")
        credentials.add(token)
        if any(token in text for text in public_text):
            raise RuntimeError("Credential leaked to a public observation before its response")
        return token

    def clear_fault(label):
        command(label, "clear_fault", MissionStatus.PAUSED_OPERATOR,
                operator_id="mock-operator", local_confirmation=True)

    try:
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node(f"mission_smoke_{os.getpid()}")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(MissionStatus, "/a2/mission/status", mission_callback, qos)
        node.create_subscription(CommandReceipt, "/a2/mission/receipts", receipt_callback, qos)
        node.create_subscription(PlatformState, "/a2/platform/state", platform_callback, qos)
        command_client = node.create_client(MissionCommand, "/a2/mission/command")
        query_client = node.create_client(GetMissionState, "/a2/mission/get_state")

        database = str(Path(data_directory.name) / "mission.sqlite")
        drop_after = -1.0

        def launch():
            return subprocess.Popen(
                ["ros2", "launch", "a2_bringup", "mock.launch.py", "profile:=mock",
                 f"robot_id:={robot_id}", f"database_path:={database}",
                 f"platform_drop_after_s:={drop_after}"],
                stdout=node_log, stderr=subprocess.STDOUT, start_new_session=True,
            )

        process = launch()
        wait_for(lambda: observed["platform_session"] is not None,
                 "fresh mock platform state")
        idle = query()
        if idle is None or idle.has_mission or idle.current.state != MissionStatus.STATE_UNAVAILABLE:
            raise RuntimeError("New temporary database did not start in an observable idle state")
        first_session = idle.current.controller_session_id
        if not first_session:
            raise RuntimeError("Idle controller did not identify its session")
        wait_for(lambda: (ready := query(expected_session=first_session)) is not None
                 and ready.mock_platform_healthy, "mock adapter health readiness")

        command("prepare", "prepare", MissionStatus.READY, bundle_id="demo-bundle",
                route_id="demo-route", map_version="demo-map", config_version="demo-config")
        command("unauthorized-start", "start", MissionStatus.READY, accepted=False,
                authorization_id="missing")
        first_token = authorize("authorize")
        started = command("start", "start", MissionStatus.INSPECTING, authorization_id=first_token)
        if started.receipt.state != MissionStatus.PRECHECK:
            raise RuntimeError("Start receipt must preserve its original PRECHECK state")
        command("visit", "record_visit", MissionStatus.INSPECTING, visit_id="demo-valve-01")
        repeated = command("start", "start", MissionStatus.INSPECTING, authorization_id=first_token)
        if repeated.receipt != started.receipt:
            raise RuntimeError("Retry did not return the original persisted Start receipt")
        wait_for(lambda: MissionStatus.INSPECTING in observed["states"],
                 "INSPECTING status on the public topic")
        command("pause", "pause", MissionStatus.PAUSED_OPERATOR)
        command("old-token-resume", "resume", MissionStatus.PAUSED_OPERATOR, accepted=False,
                authorization_id=first_token)
        token = authorize("authorize-resume")
        command("resume", "resume", MissionStatus.INSPECTING, authorization_id=token)
        command("fault", "inject_fault", MissionStatus.PAUSED_FAULT, fault_reason="DEMO_FAULT")
        clear_fault("clear-fault")
        token = authorize("authorize-after-fault")
        command("resume-after-fault", "resume", MissionStatus.INSPECTING, authorization_id=token)
        command("unknown-visit", "record_visit", MissionStatus.INSPECTING,
                accepted=False, visit_id="unknown-visit")

        previous_platform = observed["platform_session"]
        stop_process_group(process)
        process = None
        process = launch()
        wait_for(lambda: observed["platform_session"] not in {None, previous_platform},
                 "new mock platform source session after restart")
        restarted = None
        restart_deadline = min(deadline, time.monotonic() + 10.0)
        while time.monotonic() < restart_deadline and restarted is None:
            restarted = query(different_session=first_session)
        if restarted is None or not restarted.has_mission:
            raise RuntimeError("Restarted controller did not recover the mission")
        if restarted.current.state != MissionStatus.PAUSED_FAULT:
            raise RuntimeError("Controller restart resumed automatically instead of fault-pausing")
        if "demo-valve-01" not in restarted.current.completed_visits:
            raise RuntimeError("Controller restart lost the recorded visit")
        wait_for(lambda: (ready := query(expected_session=restarted.current.controller_session_id)) is not None
                 and ready.mock_platform_healthy, "restarted mock adapter health readiness")
        historical = command("start", "start", MissionStatus.PAUSED_FAULT,
                             authorization_id=first_token)
        if historical.receipt.state != MissionStatus.PRECHECK:
            raise RuntimeError("Historical Start receipt was rewritten after restart")
        clear_fault("clear-after-restart")
        command("expired-session-token", "resume", MissionStatus.PAUSED_OPERATOR, accepted=False,
                authorization_id=token)
        token = authorize("authorize-after-restart")
        command("resume-after-restart", "resume", MissionStatus.INSPECTING, authorization_id=token)
        command("cancel", "cancel", MissionStatus.CANCELLED)
        final = query()
        if final is None or not final.has_mission or final.current.state != MissionStatus.CANCELLED:
            raise RuntimeError("Final query did not retain the cancelled mission")
        wait_for(lambda: observed["receipts"] > 0 and MissionStatus.CANCELLED in observed["states"],
                 "public cancelled status and command receipts")
        check_running()
        stop_process_group(process)
        process = None

        # An actual mock-source dropout must drive the task into a latched fault.
        mission_id += "-dropout"
        database = str(Path(data_directory.name) / "dropout.sqlite")
        # Allow discovery and explicit start before this synthetic source drops.
        # The controller's stale/health rules and fault assertions stay unchanged.
        drop_after = 20.0
        previous_platform = observed["platform_session"]
        process = launch()
        wait_for(lambda: observed["platform_session"] not in {None, previous_platform},
                 "fresh platform for dropout scenario")
        wait_for(lambda: (ready := query()) is not None and ready.mock_platform_healthy,
                 "dropout scenario adapter health readiness")
        command("drop-prepare", "prepare", MissionStatus.READY, bundle_id="demo-bundle",
                route_id="demo-route", map_version="demo-map", config_version="demo-config")
        token = authorize("drop-authorize")
        command("drop-start", "start", MissionStatus.INSPECTING, authorization_id=token)
        wait_for(lambda: (faulted := query()) is not None and
                 faulted.current.state == MissionStatus.PAUSED_FAULT and
                 not faulted.mock_platform_healthy, "task fault after real mock-state dropout",
                 timeout=drop_after + 5.0)
        command("drop-clear-denied", "clear_fault", MissionStatus.PAUSED_FAULT, accepted=False,
                operator_id="mock-operator", local_confirmation=True)
        command("drop-cancel", "cancel", MissionStatus.CANCELLED)
        stop_process_group(process)
        process = None
    except (Exception, KeyboardInterrupt) as exc:
        failure = str(exc) or type(exc).__name__
    finally:
        stop_process_group(process)
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        data_directory.cleanup()
        node_log.flush()
        node_log.seek(0)
        log_tail = node_log.read()[-4000:]
        node_log.close()

    if failure:
        print(f"ERROR: {failure}", file=sys.stderr)
        if log_tail.strip():
            print("mock bringup output (tail):\n" + log_tail, file=sys.stderr)
        return 1
    print(f"PASS: mock mission ROS lifecycle ({calls} command calls), one-use authorization, "
          "historical receipts, public observations, visit validation, fault recovery and "
          "persistent controller restart without automatic execution, plus platform dropout inhibition.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
