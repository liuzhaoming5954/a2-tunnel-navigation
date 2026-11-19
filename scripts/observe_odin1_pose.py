#!/usr/bin/env python3
"""在 PC2 只读订阅 ODIN1 的同帧里程计与地图变换，将传感器位姿交给同机网关。"""

import argparse
from dataclasses import dataclass
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import time


ODOMETRY = "/odin1/odometry"
DYNAMIC_TF = "/tf"
SOURCE = "odin1_relocalization"
MAX_AGE_NS = 750_000_000
NS = 1_000_000_000


def file_digests(path):
    # 厂商传输日志给出 MD5，网页地图绑定使用 SHA-256。一次顺序读完成两个校验。
    sha = hashlib.sha256()
    md5 = hashlib.md5(usedforsecurity=False)
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
            md5.update(block)
    return sha.hexdigest(), md5.hexdigest()


class MapSession:
    """根据 v0.14.5 当次驱动日志识别已传输且正在重定位的地图。"""

    def __init__(self, map_path, md5):
        self.path = str(map_path)
        self.md5 = md5
        self.clear()

    def clear(self):
        self.requested = self.digest_ok = self.loaded = self.mode_two = self.matched = False
        self.mismatch = False

    def line(self, line):
        # 一旦换图就立即撤销旧位姿，即使新地图尚在传输或最终传输失败。
        if "[service]: set_reloc_map: transferring [" in line:
            self.clear()
            self.requested = f"[service]: set_reloc_map: transferring [{self.path}]" in line
            self.mismatch = not self.requested
        elif "send_file_handler" in line and "file md5:" in line:
            match = re.search(r"file md5: ([0-9a-fA-F]{32})\b", line)
            self.digest_ok = bool(self.requested and match and match.group(1).lower() == self.md5)
            self.mismatch = not self.digest_ok
        elif "[map_transfer]: Relocalization map swap completed:" in line:
            completed = line.partition("Relocalization map swap completed: ")[2].rstrip("\n")
            completed = completed.removesuffix(" (SLAM may restart asynchronously)")
            self.loaded = self.requested and self.digest_ok and completed == self.path
            self.mismatch = not self.loaded
        elif "[service]: set_slam_mode: map_mode set to " in line:
            self.mode_two = bool(re.search(r"map_mode set to 2(?:\s|$)", line))
            self.matched = False
        elif "[odom]: relocalization success!" in line:
            self.matched = self.loaded and self.mode_two
        elif ("[lydros_node]: Services ready:" in line or "Relocalization map swap failed" in line
              or "[signal_handler]: Shutting down driver" in line or "Device disconnected" in line):
            self.clear()

    @property
    def status(self):
        if self.mismatch:
            return "map_mismatch"
        return "live" if self.loaded and self.mode_two and self.matched else "waiting"


def driver_process(log_stat):
    """只接受本用户实际写入给定日志的 SDK 进程，避免误用上次会话的成功日志。"""
    drivers = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            if (proc / "comm").read_text().strip() != "host_sdk_sample":
                continue
            if any(os.path.samestat((proc / f"fd/{fd}").stat(), log_stat) for fd in (1, 2)):
                drivers.append(proc)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    if len(drivers) != 1:
        raise ValueError("找不到唯一正在向该日志写入的 host_sdk_sample，请核对本轮驱动日志")
    return drivers[0]


@dataclass(frozen=True)
class Sample:
    receipt: int
    p: tuple
    q: tuple


def product(a, b):
    x, y, z, w = a
    i, j, k, l = b
    return (w*i + x*l + y*k - z*j, w*j - x*k + y*l + z*i,
            w*k + x*j - y*i + z*l, w*l - x*i - y*j - z*k)


def map_pose(odom, tf):
    # ODIN1 发布的是 odom←map。先取逆变换，再左乘 odom←imu；平移与朝向均在三维空间计算。
    x, y, z, w = tf.q
    inverse = (-x, -y, -z, w)
    delta = tuple(a - b for a, b in zip(odom.p, tf.p))
    position = product(product(inverse, (*delta, 0.)), tf.q)[:3]
    x, y, z, w = product(inverse, odom.q)
    forward_x, forward_y = 1 - 2*(y*y + z*z), 2*(x*y + w*z)
    if not all(map(math.isfinite, position)) or math.hypot(forward_x, forward_y) <= 1e-6:
        raise ValueError("传感器位置无效或 +X 轴无水平投影")
    return position, math.atan2(forward_y, forward_x)


class Pairs:
    """只使用相同设备时间戳，异步抵达、重发和超时均不能把旧位置刷新为实时。"""

    def __init__(self):
        self.odom = {}
        self.tf = {}
        self.last_stamp = 0

    def add(self, series, header, p, q):
        stamp = header.stamp.sec * NS + header.stamp.nanosec
        if stamp <= self.last_stamp or stamp in series:
            return
        xyz = (p.x, p.y, p.z)
        quat = (q.x, q.y, q.z, q.w)
        norm = math.hypot(*quat)
        if not all(map(math.isfinite, xyz)) or not 0.95 <= norm <= 1.05:
            return
        series[stamp] = Sample(time.monotonic_ns(), xyz, tuple(v/norm for v in quat))
        if len(series) > 24:
            del series[next(iter(series))]

    def latest(self, now):
        for series in (self.odom, self.tf):
            for stamp, sample in tuple(series.items()):
                if stamp <= self.last_stamp or now - sample.receipt > MAX_AGE_NS:
                    del series[stamp]
        stamp = max(self.odom.keys() & self.tf.keys(), default=None)
        if stamp is None:
            return None
        self.last_stamp = stamp
        odom, tf = self.odom[stamp], self.tf[stamp]
        position, yaw = map_pose(odom, tf)
        return dict(stamp_ns=stamp, position_m=position, yaw_rad=yaw,
                    received_monotonic_ns=min(odom.receipt, tf.receipt))


def write_status(path, status, **fields):
    # 网关会并发读取，使用同目录私有临时文件再原子替换，避免看到部分 JSON。
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix="." + path.name + ".")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(dict(source=SOURCE, status=status, **fields), stream, allow_nan=False)
            stream.write("\n")
        os.replace(tmp, path)
    finally:
        Path(tmp).unlink(missing_ok=True)


def endpoints_ready(node):
    # PC2 已验证的专用域内，这两条话题各有唯一的 /lydros_node 发布者。
    # 排除重复驱动、回放以及来自同名坐标系的另一个动态 TF 树。
    for topic in (ODOMETRY, DYNAMIC_TF):
        endpoints = node.get_publishers_info_by_topic(topic)
        if [(e.node_namespace, e.node_name) for e in endpoints] != [("/", "lydros_node")]:
            return False
    return True


def collect(args, log, sha, md5):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    log_stat = os.fstat(log.fileno())
    proc = driver_process(log_stat)
    start = (proc / "stat").read_text().rsplit(")", 1)[1].split()[19]
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    session = MapSession(args.map_bin, md5)
    pairs = Pairs()
    active = False

    def on_odom(msg):
        if active and (msg.header.frame_id.lstrip("/"), msg.child_frame_id.lstrip("/")) == ("odom", "imu"):
            pose = msg.pose.pose
            pairs.add(pairs.odom, msg.header, pose.position, pose.orientation)

    def on_tf(msg):
        if active:
            for t in msg.transforms:
                if (t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/")) == ("odom", "map"):
                    pairs.add(pairs.tf, t.header, t.transform.translation, t.transform.rotation)

    rclpy.init(args=[])
    node = rclpy.create_node("a2_odin1_sensor_display", start_parameter_services=False)
    qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(Odometry, ODOMETRY, on_odom, qos)
    node.create_subscription(TFMessage, DYNAMIC_TF, on_tf, qos)
    last_status, last_receipt, graph_check = "waiting", 0, 0
    graph_ok = False
    partial = ""
    try:
        while rclpy.ok():
            # 保留日志中尚未换行的尾部，成功标记及后续换图均须读到本轮的完整记录。
            now = time.monotonic_ns()
            if now >= graph_check:
                fields = (proc / "stat").read_text().rsplit(")", 1)[1].split()
                if (fields[0] == "Z" or fields[19] != start or not os.path.samestat(args.driver_log.stat(), log_stat)
                        or not any(os.path.samestat((proc / f"fd/{fd}").stat(), log_stat) for fd in (1, 2))):
                    raise ValueError("驱动进程或日志已切换，请接入新的驱动会话")
                graph_ok = endpoints_ready(node)
                graph_check = now + 200_000_000
            lines = (partial + log.read(128 * 1024)).split("\n")
            partial = lines.pop()
            for line in lines:
                session.line(line)
            # 积压的控制日志必须先读完，否则可能错过紧跟在成功标记之后的换图或退出。
            caught_up = log.tell() == os.fstat(log.fileno()).st_size
            ready = session.status == "live" and graph_ok and caught_up
            if active != ready:
                pairs = Pairs()
                last_receipt = 0
                active = ready
            status = session.status if session.status != "live" else "waiting"
            pose = None
            if active:
                try:
                    pose = pairs.latest(now)
                except ValueError:
                    last_receipt = 0
                    status = "invalid"
                if pose:
                    last_receipt = pose["received_monotonic_ns"]
                    status = "live"
                elif last_receipt:
                    status = "live" if now - last_receipt <= MAX_AGE_NS else "stale"
            if pose:
                write_status(args.output, "live", frame="map", child_frame="imu",
                             map_sha256=sha, boot_id=boot_id, **pose)
            elif status != last_status:
                write_status(args.output, status)
            last_status = status
            rclpy.spin_once(node, timeout_sec=0.04)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-bin", required=True, type=Path, help="已加载的厂商 BIN 文件")
    parser.add_argument("--driver-log", required=True, type=Path, help="当次驱动会话合并的标准输出和错误日志")
    parser.add_argument("--output", required=True, type=Path, help="与本机网关共用的受控 JSON 状态文件")
    args = parser.parse_args(argv)
    try:
        args.map_bin = args.map_bin.resolve(strict=True)
        args.driver_log = args.driver_log.resolve(strict=True)
        if args.output.resolve() in (args.map_bin, args.driver_log):
            raise ValueError("输出不能覆盖输入地图或日志")
        # 同一状态文件只允许一个活动写入者，重试启动不会干扰正在服务的网关。
        with args.output.with_name(args.output.name + ".lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            write_status(args.output, "waiting")
            try:
                sha, md5 = file_digests(args.map_bin)
                with args.driver_log.open(encoding="utf-8", errors="replace") as log:
                    collect(args, log, sha, md5)
            finally:
                write_status(args.output, "waiting")
    except (OSError, ValueError, ImportError) as error:
        print(f"ODIN1 只读定位接入失败：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
