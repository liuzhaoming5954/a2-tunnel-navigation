#!/usr/bin/env python3
"""Publish a receive-only A2 SDK mode as a ROS 2 PlatformState observation."""

import argparse
from contextlib import contextmanager
import operator
import os
from pathlib import Path
import subprocess
import sys
from threading import Lock, Thread
import time
from uuid import uuid4


SPORT_STATE_TOPIC = "rt/lf/sportmodestate"
PLATFORM_STATE_TOPIC = "/a2/platform/state"
MAX_MODE = (1 << 32) - 1
NS_PER_MS = 1_000_000


def run_sdk_reader(args):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

    descriptor = args.sample_fd
    os.set_inheritable(descriptor, False)
    os.set_blocking(descriptor, False)

    def receive(message):
        # Both Linux processes share the host monotonic clock.
        received_ns = time.monotonic_ns()
        try:
            mode = operator.index(message.mode)
        except (AttributeError, TypeError):
            return
        if 0 <= mode <= MAX_MODE:
            try:
                os.write(descriptor, f"{received_ns} {mode}\n".encode("ascii"))
            except (BlockingIOError, BrokenPipeError):
                # Preserve the transport callback when the ROS process stalls or exits.
                pass

    ChannelFactoryInitialize(args.unitree_domain_id, args.network_interface)
    subscriber = ChannelSubscriber(SPORT_STATE_TOPIC, SportModeState_)
    subscriber.Init(receive, 1)
    try:
        # The ROS parent's control pipe closes when it shuts down or exits.
        sys.stdin.buffer.read(1)
    finally:
        subscriber.Close()


class Samples:
    def __init__(self):
        self._lock = Lock()
        self._latest = None

    def read(self, stream):
        for record in stream:
            received_ns, mode = map(int, record.split())
            with self._lock:
                previous = self._latest
                if previous is None or received_ns > previous[1]:
                    self._latest = (1 if previous is None else previous[0] + 1, received_ns, mode)

    def latest(self):
        with self._lock:
            return self._latest


def platform_state(message_type, stamp, robot_id, session, sample, now_ns, stale_timeout_ms):
    msg = message_type()
    msg.schema_version = 1
    msg.header.stamp = stamp
    msg.header.frame_id = "base_link"
    msg.robot_id = robot_id
    msg.source = "unitree_sdk2"
    msg.source_session_id = session
    msg.simulated = msg.motion_capable = msg.valid = False
    msg.battery_valid = msg.velocity_valid = msg.mode_valid = False
    msg.battery_percent = float("nan")
    for vector in (msg.body_velocity.linear, msg.body_velocity.angular):
        vector.x = vector.y = vector.z = float("nan")
    msg.reason = "NO_DATA"
    if sample is None:
        return msg

    msg.sample_seq, msg.received_mono_ns, msg.mode = sample
    msg.mode_valid = True
    age_ns = now_ns - msg.received_mono_ns
    msg.age_ms = max(0, age_ns) // NS_PER_MS
    if age_ns < 0:
        msg.reason = "SOURCE_TIME_IN_FUTURE"
    elif age_ns >= stale_timeout_ms * NS_PER_MS:
        msg.reason = "STALE"
    else:
        msg.reason = "INVALID_BATTERY"
    return msg


@contextmanager
def sdk_process(args, samples):
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd, "rb") as stream:
        try:
            child = subprocess.Popen([
                args.sdk_python, str(Path(__file__).resolve()),
                "--network-interface", args.network_interface,
                "--unitree-domain-id", str(args.unitree_domain_id),
                "--robot-id", args.robot_id,
                "--sample-fd", str(write_fd),
            ], stdin=subprocess.PIPE, pass_fds=(write_fd,), start_new_session=True)
        finally:
            os.close(write_fd)
        reader = Thread(target=samples.read, args=(stream,), daemon=True)
        reader.start()
        try:
            yield child
        finally:
            child.stdin.close()
            try:
                child.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            reader.join()


def run_publisher(args):
    import rclpy
    from a2_interfaces.msg import PlatformState
    from rclpy.executors import ExternalShutdownException

    samples = Samples()
    session = uuid4().hex
    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("a2_platform_sdk_readonly", start_parameter_services=False)
        publisher = node.create_publisher(PlatformState, PLATFORM_STATE_TOPIC, 10)

        def publish():
            sample = samples.latest()
            msg = platform_state(PlatformState, node.get_clock().now().to_msg(), args.robot_id,
                                 session, sample, time.monotonic_ns(), args.stale_timeout_ms)
            publisher.publish(msg)

        node.create_timer(args.publish_period_ms / 1000, publish)
        with sdk_process(args, samples) as child:
            while rclpy.ok():
                if child.poll() is not None:
                    raise RuntimeError(f"Unitree SDK subscriber exited with status {child.returncode}")
                rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def nonblank(value):
    if not value.strip():
        raise argparse.ArgumentTypeError("must not be empty")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network-interface", required=True, type=nonblank, help="Unitree SDK network interface")
    parser.add_argument("--unitree-domain-id", required=True, type=int, help="Unitree DDS domain, separate from ROS")
    parser.add_argument("--robot-id", required=True, type=nonblank)
    parser.add_argument("--sdk-python", default=sys.executable, help="Python interpreter with Unitree SDK2Py installed")
    parser.add_argument("--publish-period-ms", type=int, default=100)
    parser.add_argument("--stale-timeout-ms", type=int, default=500)
    parser.add_argument("--sample-fd", type=int, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.unitree_domain_id < 0 or args.publish_period_ms <= 0 or args.stale_timeout_ms <= 0:
        parser.error("domain must be nonnegative and timing options must be positive")
    try:
        if args.sample_fd is None:
            run_publisher(args)
        else:
            run_sdk_reader(args)
    except (ImportError, OSError, RuntimeError) as error:
        print(f"A2 platform state: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
