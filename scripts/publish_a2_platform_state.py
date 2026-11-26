#!/usr/bin/env python3
"""从原厂 SDK 只读采集 A2 模式、双电池、速度和机身姿态，发布至 PC2 的 ROS 域。"""

import argparse
from contextlib import contextmanager
import json
import math
import operator
import os
from pathlib import Path
import subprocess
import sys
from threading import Lock, Thread
import time
from typing import NamedTuple
from uuid import uuid4


SPORT_STATE_TOPIC = "rt/lf/sportmodestate"
PLATFORM_STATE_TOPIC = "/a2/platform/state"
PLATFORM_DETAILS_TOPIC = "/a2/platform/details"
MAX_MODE = (1 << 32) - 1
NS_PER_MS = 1_000_000


def integer_value(value, maximum):
    """保留有效整数，包括 0；SDK 哨兵值不可当作实测数值。"""
    try:
        number = operator.index(value)
    except TypeError:
        return None
    return number if not isinstance(value, bool) and 0 <= number <= maximum else None


def finite_value(value):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def vector3(values):
    if values is None or len(values) != 3:
        return [None, None, None]
    return [finite_value(component) for component in values]


def battery_values(message):
    # 实机两路 BMS 使用相同 IDL，cycle 为 uint16。按字段保留，某一项
    # 无效时同帧其他可用值仍可展示。电流方向及状态字无法确认充电语义。
    return [integer_value(getattr(message, "soc", None), 100),
            integer_value(getattr(message, "soh", None), 100),
            integer_value(getattr(message, "cycle", None), 65535)]


def sport_values(message):
    # 旧版 SDK 若只提供模式，仍应正常上报模式，缺少的测量字段留空。
    return [integer_value(getattr(message, "mode", None), MAX_MODE),
            *vector3(getattr(message, "velocity", None)),
            finite_value(getattr(message, "yaw_speed", None))]


def attitude_values(message):
    return vector3(getattr(getattr(message, "imu_state", None), "rpy", None))


def run_sdk_reader(args):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_

    descriptor = args.sample_fd
    os.set_inheritable(descriptor, False)
    os.set_blocking(descriptor, False)

    def receiver(source, extract):
        def receive(message):
            # 四个 SDK 回调线程共享这条非阻塞管道。记录只有时间戳、来源和
            # 最多五个数值，单次写入小于 PIPE_BUF，不会在并发写入时交错。
            received_ns = time.monotonic_ns()
            data = json.dumps([source, received_ns, extract(message)], allow_nan=False)
            try:
                os.write(descriptor, (data + "\n").encode("ascii"))
            except (BlockingIOError, BrokenPipeError):
                # ROS 进程卡住或退出时允许丢帧，避免阻塞 SDK 接收线程。
                pass
        return receive

    ChannelFactoryInitialize(args.unitree_domain_id, args.network_interface)
    sources = (
        ("sport", SPORT_STATE_TOPIC, SportModeState_, sport_values),
        ("primary_battery", "rt/lf/bmsstate", BmsState_, battery_values),
        ("secondary_battery", "rt/lf/secondary_bmsstate", BmsState_, battery_values),
        ("attitude", "rt/lf/lowstate", LowState_, attitude_values),
    )
    subscribers = []
    try:
        for source, topic, message_type, extract in sources:
            subscriber = ChannelSubscriber(topic, message_type)
            subscriber.Init(receiver(source, extract), 1)
            subscribers.append(subscriber)
        # ROS 父进程关闭或退出时，其控制管道会关闭。
        sys.stdin.buffer.read(1)
    finally:
        for subscriber in subscribers:
            subscriber.Close()


class Sample(NamedTuple):
    seq: int
    received_ns: int
    values: list


class Samples:
    def __init__(self):
        self._lock = Lock()
        self._latest = {}

    def read(self, stream):
        for record in stream:
            source, received_ns, values = json.loads(record)
            with self._lock:
                previous = self._latest.get(source)
                if previous is None or received_ns > previous.received_ns:
                    seq = 1 if previous is None else previous.seq + 1
                    self._latest[source] = Sample(seq, received_ns, values)

    def latest(self):
        with self._lock:
            return self._latest.copy()


def set_source(msg, stamp, robot_id, session):
    msg.schema_version = 1
    msg.header.stamp = stamp
    msg.robot_id = robot_id
    msg.source = "unitree_sdk2"
    msg.source_session_id = session
    msg.simulated = False


def fresh(sample, now_ns, stale_timeout_ms):
    return sample is not None and 0 <= now_ns - sample.received_ns < stale_timeout_ms * NS_PER_MS


def as_float(value):
    return float("nan") if value is None else value


def platform_state(message_type, stamp, robot_id, session, sample, now_ns, stale_timeout_ms):
    msg = message_type()
    set_source(msg, stamp, robot_id, session)
    msg.header.frame_id = "base_link"
    msg.motion_capable = msg.valid = False
    msg.battery_valid = msg.velocity_valid = msg.mode_valid = False
    msg.battery_percent = float("nan")
    for vector in (msg.body_velocity.linear, msg.body_velocity.angular):
        vector.x = vector.y = vector.z = float("nan")
    msg.reason = "NO_DATA"
    if sample is None:
        return msg

    msg.sample_seq, msg.received_mono_ns = sample.seq, sample.received_ns
    mode = sample.values[0]
    if mode is not None:
        msg.mode, msg.mode_valid = mode, True
    age_ns = now_ns - msg.received_mono_ns
    msg.age_ms = max(0, age_ns) // NS_PER_MS
    if age_ns < 0:
        msg.reason = "SOURCE_TIME_IN_FUTURE"
    elif age_ns >= stale_timeout_ms * NS_PER_MS:
        msg.reason = "STALE"
    else:
        # 原厂速度的参考系与双电池的整机 SOC 汇总规则尚待确认。
        # 旧接口不填 base_link 速度或整机电量；原始分量在明细中提供。
        msg.reason = "INVALID_BATTERY"
    return msg


def platform_details(message_type, stamp, robot_id, session, latest, now_ns, stale_timeout_ms):
    msg = message_type()
    set_source(msg, stamp, robot_id, session)
    msg.max_age_ms = stale_timeout_ms
    for name in ("primary_battery", "secondary_battery"):
        battery = getattr(msg, name)
        sample = latest.get(name)
        battery.soc_percent = battery.soh_percent = battery.cycle_count = -1
        battery.charging_state = "unknown"
        if fresh(sample, now_ns, stale_timeout_ms):
            battery.received_mono_ns = sample.received_ns
            battery.soc_percent, battery.soh_percent, battery.cycle_count = [
                -1 if value is None else value for value in sample.values
            ]

    # 未测量和过期均填 NaN，合法的零速度和零角度原样保留。
    msg.velocity_mps = [float("nan")] * 3
    msg.yaw_rate_radps = float("nan")
    msg.rpy_rad = [float("nan")] * 3
    sport = latest.get("sport")
    if fresh(sport, now_ns, stale_timeout_ms):
        msg.velocity_received_mono_ns = sport.received_ns
        msg.velocity_mps = [as_float(value) for value in sport.values[1:4]]
        msg.yaw_rate_radps = as_float(sport.values[4])
    attitude = latest.get("attitude")
    if fresh(attitude, now_ns, stale_timeout_ms):
        msg.attitude_received_mono_ns = attitude.received_ns
        msg.rpy_rad = [as_float(value) for value in attitude.values]
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
    from a2_interfaces.msg import PlatformDetails, PlatformState
    from rclpy.executors import ExternalShutdownException

    samples = Samples()
    session = uuid4().hex
    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("a2_platform_sdk_readonly", start_parameter_services=False)
        publisher = node.create_publisher(PlatformState, PLATFORM_STATE_TOPIC, 10)
        details_publisher = node.create_publisher(PlatformDetails, PLATFORM_DETAILS_TOPIC, 10)

        def publish():
            latest = samples.latest()
            stamp, now_ns = node.get_clock().now().to_msg(), time.monotonic_ns()
            msg = platform_state(PlatformState, stamp, args.robot_id, session,
                                 latest.get("sport"), now_ns, args.stale_timeout_ms)
            publisher.publish(msg)
            details_publisher.publish(platform_details(
                PlatformDetails, stamp, args.robot_id, session, latest, now_ns, args.stale_timeout_ms))

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
