#!/usr/bin/env python3
"""Finite ROS smoke test for the synthetic readonly platform node.

Source the built workspace first and set ROS_LOCALHOST_ONLY=1 plus an explicit
isolated ROS_DOMAIN_ID. This script only launches source=mock; it accepts no
source selection, ROS remapping or other passthrough command arguments.
"""

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time


def stop_process_group(process):
    """Bound cleanup for ros2 run and the executable it launches."""
    if process is None:
        return
    for signum, timeout in ((signal.SIGINT, 2), (signal.SIGTERM, 1), (signal.SIGKILL, 1)):
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
            # The group may still contain a descendant after the wrapper exits.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
        except subprocess.TimeoutExpired:
            continue


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        print("ERROR: set ROS_LOCALHOST_ONLY=1 before running this local smoke test.", file=sys.stderr)
        return 2
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if not domain.isdecimal():
        print("ERROR: set an explicit isolated ROS_DOMAIN_ID before running this test.", file=sys.stderr)
        return 2
    try:
        import rclpy
        from a2_interfaces.msg import PlatformState
        from a2_interfaces.srv import GetPlatformState
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        print(f"ERROR: ROS interfaces unavailable ({exc}). Build the packages and source the installed workspace.",
              file=sys.stderr)
        return 2

    robot_id = f"readonly-smoke-{os.getpid()}"
    observed = {"fresh": 0, "stale": [], "stale_diagnostic": False, "error": None}
    node = None
    process = None
    initialized = False
    failure = None
    node_log = tempfile.TemporaryFile(mode="w+t", encoding="utf-8")

    def state_callback(message):
        if message.robot_id != robot_id:
            return
        if message.source != "mock" or not message.simulated or message.motion_capable:
            observed["error"] = "Synthetic readonly state has incorrect source/simulated/motion_capable flags"
            return
        if message.valid:
            if observed["stale"]:
                observed["error"] = "Fresh data resumed after the configured permanent mock dropout"
            observed["fresh"] += 1
        elif message.reason == "STALE":
            if observed["fresh"] == 0:
                # Missing every fresh sample must lead to timeout, never PASS.
                return
            sample = (message.sample_seq, message.received_mono_ns, message.age_ms)
            if observed["stale"]:
                previous = observed["stale"][-1]
                if sample[:2] != previous[:2]:
                    observed["error"] = "Stale republication refreshed sample_seq or received_mono_ns"
                elif sample[2] <= previous[2]:
                    observed["error"] = "Stale sample age did not increase"
            observed["stale"].append(sample)

    def diagnostic_callback(message):
        for status in message.status:
            if (status.hardware_id == robot_id and status.message == "STALE"
                    and status.level == DiagnosticStatus.ERROR):
                observed["stale_diagnostic"] = True

    try:
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node(f"readonly_smoke_{os.getpid()}")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(PlatformState, "/a2/platform/state", state_callback, qos)
        node.create_subscription(DiagnosticArray, "/diagnostics", diagnostic_callback, qos)
        client = node.create_client(GetPlatformState, "/a2/platform/get_state")
        process = subprocess.Popen(
            ["ros2", "run", "a2_platform", "readonly_node", "--ros-args",
             "-p", "source:=mock", "-p", f"robot_id:={robot_id}",
             "-p", "mock_drop_after_s:=3.0", "-p", "stale_timeout_ms:=400",
             "-p", "publish_period_ms:=100"],
            stdout=node_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        deadline = time.monotonic() + 12.0
        future = None
        service_ok = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if observed["error"]:
                raise RuntimeError(observed["error"])
            if process.poll() is not None:
                raise RuntimeError(f"readonly_node exited early with status {process.returncode}")
            ready = (observed["fresh"] > 0 and len(observed["stale"]) >= 2
                     and observed["stale_diagnostic"])
            if ready and future is None and client.service_is_ready():
                future = client.call_async(GetPlatformState.Request())
            if future is not None and future.done():
                response = future.result()
                if response is None:
                    raise RuntimeError("Platform state service returned no response")
                state = response.state
                if not (response.available and state.robot_id == robot_id and state.source == "mock"
                        and state.simulated and not state.motion_capable
                        and not state.valid and state.reason == "STALE"):
                    raise RuntimeError("Service did not report the available but stale synthetic state")
                if (state.sample_seq, state.received_mono_ns) != observed["stale"][-1][:2]:
                    raise RuntimeError("Service stale sample identity differs from the published sample")
                service_ok = True
                break
        if not service_ok:
            raise RuntimeError(
                "Timed out after 12s: "
                f"fresh={observed['fresh']}, stale={len(observed['stale'])}, "
                f"stale_diagnostic={observed['stale_diagnostic']}, "
                f"service_ready={client.service_is_ready()}. "
                "At least one fresh sample must be observed before stale validation can pass."
            )
    except (Exception, KeyboardInterrupt) as exc:
        failure = str(exc) or type(exc).__name__
    finally:
        stop_process_group(process)
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        node_log.flush()
        node_log.seek(0)
        log_tail = node_log.read()[-3000:]
        node_log.close()

    if failure:
        print(f"ERROR: {failure}", file=sys.stderr)
        if log_tail.strip():
            print("readonly_node output (tail):\n" + log_tail, file=sys.stderr)
        return 1
    print(f"PASS: mock readonly ROS state ({observed['fresh']} fresh, {len(observed['stale'])} stale), "
          "unchanged stale sample identity, increasing age, ERROR diagnostic and stale query service.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
