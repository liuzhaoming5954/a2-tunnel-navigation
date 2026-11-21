#!/usr/bin/env python3
"""在独立导航域发布 ODIN1 的地图、局部里程计和经实测外参换算的机身位姿。"""

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import sys
import time

# 正常 CLI 与开发机单测分别从脚本目录和工程命名空间导入同一套四元数运算。
if __package__:
    from .observe_odin1_pose import MAX_AGE_NS, NS, SOURCE, product
else:
    from observe_odin1_pose import MAX_AGE_NS, NS, SOURCE, product


def read_object(path):
    with path.open("rb") as stream:
        data = stream.read(65_537)
    if len(data) > 65_536:
        raise ValueError("位姿或外参文件超过 64 KiB")
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError("位姿和外参文件必须是 JSON 对象")
    return value


def components(value, count):
    if (not isinstance(value, (list, tuple)) or len(value) != count
            or not all(type(v) in (int, float) and math.isfinite(v) for v in value)):
        raise ValueError("平移或四元数分量无效")
    return tuple(float(v) for v in value)


def rotate(q, point):
    x, y, z, w = q
    return product(product(q, (*point, 0.0)), (-x, -y, -z, w))[:3]


@dataclass(frozen=True)
class Transform:
    """父坐标系中子坐标系原点的米制平移与 XYZW 旋转。"""

    p: tuple
    q: tuple

    @classmethod
    def read(cls, value):
        if not isinstance(value, dict):
            raise ValueError("变换必须包含平移和旋转")
        p = components(value["translation_m"], 3)
        q = components(value["quaternion_xyzw"], 4)
        norm = math.hypot(*q)
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-5):
            raise ValueError("旋转须为单位四元数")
        return cls(p, tuple(v / norm for v in q))

    def inverse(self):
        x, y, z, w = self.q
        q = (-x, -y, -z, w)
        p = rotate(q, tuple(-v for v in self.p))
        if not all(map(math.isfinite, p)):
            raise ValueError("逆变换平移溢出")
        return Transform(p, q)

    def compose(self, child):
        # T_A_C = T_A_B × T_B_C：先将子变换的平移旋入父系，再加父变换原点。
        p = tuple(a + b for a, b in zip(self.p, rotate(self.q, child.p)))
        if not all(map(math.isfinite, p)):
            raise ValueError("合成机身位姿溢出")
        return Transform(p, product(self.q, child.q))


def load_extrinsics(path, base_frame):
    data = read_object(path)
    if not isinstance(data.get("source"), str) or not data["source"].strip():
        raise ValueError("请为实测外参填写 source 记录来源")
    if data.get("base_frame") != base_frame:
        raise ValueError("外参 base_frame 与要发布的机身帧不一致")
    return Transform.read(data["T_base_from_imu"])


def heading(q):
    x, y, z, w = q
    forward = (1 - 2*(y*y + z*z), 2*(x*y + w*z))
    if math.hypot(*forward) < 1e-6:
        raise ValueError("机身前向轴没有可用的水平投影")
    return math.atan2(forward[1], forward[0])


def planar_velocity(previous, current, seconds):
    # 平移以连续的局部里程计计算，地图配准的校正不应产生虚假车速。
    # ROS Odometry 的 twist 属于 child_frame_id；将水平速度旋到当前机头坐标系。
    yaw = heading(current.q)
    delta_yaw = yaw - heading(previous.q)
    dx, dy = ((current.p[i] - previous.p[i]) / seconds for i in (0, 1))
    vx = math.cos(yaw)*dx + math.sin(yaw)*dy
    vy = -math.sin(yaw)*dx + math.cos(yaw)*dy
    wz = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw)) / seconds
    if not all(map(math.isfinite, (vx, vy, wz))):
        raise ValueError("相邻样本无法生成有限的平面速度估计")
    return vx, vy, wz


class BodyProjection:
    def __init__(self, digest, boot_id, output_domain, base_from_imu):
        self.digest, self.boot_id, self.domain = digest, boot_id, output_domain
        self.imu_from_base = base_from_imu.inverse()
        self.last_device = self.last_receipt = self.last_ros = 0
        self.previous = None

    def invalidate(self):
        # 留下最后处理过的时间戳用于去重，只撤掉速度差分所需的前一测量。
        self.previous = None

    def sample(self, data, steady_now, ros_now):
        """用同源且连续的两个样本估计平面速度。首帧、恢复首帧和重复快照均不发布。"""
        try:
            return self._sample(data, steady_now, ros_now)
        except (ValueError, KeyError, OverflowError, TypeError):
            self.invalidate()
            raise

    def _sample(self, data, steady_now, ros_now):
        if (data.get("source") != SOURCE or data.get("status") != "live"
                or data.get("map_sha256") != self.digest or data.get("boot_id") != self.boot_id
                or (data.get("frame"), data.get("child_frame")) != ("map", "imu")):
            raise ValueError("等待本次启动、指定地图的 ODIN1 实时位姿")
        source_domain = data.get("source_domain_id")
        if type(source_domain) is not int or not 0 <= source_domain <= 232 or source_domain == self.domain:
            raise ValueError("观测必须来自另一已标识的设备 ROS 域")
        device = data["stamp_ns"]
        receipt = data["received_monotonic_ns"]
        if type(device) is not int or device <= 0 or type(receipt) is not int or receipt <= 0:
            raise ValueError("设备时间或同机接收时间无效")
        age = steady_now - receipt
        if not 0 <= age < MAX_AGE_NS:
            raise ValueError("同机接收的设备样本已失效")
        if device == self.last_device and receipt == self.last_receipt:
            return None
        # 设备时间用于顺序、去重。导航 ROS 系统时钟使用两路中较早的本机接收时刻投影，
        # 避免将已在文件中存放数百毫秒的观测重新盖成“此刻”的数据。
        stamp = ros_now - age
        if device == self.last_device or receipt <= self.last_receipt:
            raise ValueError("重复的设备时间或过期的接收时刻")
        map_from_odom = Transform.read(data["T_odom_from_map"]).inverse()
        odom_from_base = Transform.read(data["T_odom_from_imu"]).compose(self.imu_from_base)
        # 设备重启、长间隙和系统时钟回拨都需要重新积累一对样本，不能跨越这些断点求速度。
        dt = device - self.last_device
        velocity = None
        if (self.previous is not None and 0 < dt <= MAX_AGE_NS
                and receipt - self.last_receipt <= MAX_AGE_NS and 0 < stamp - self.last_ros <= MAX_AGE_NS):
            velocity = planar_velocity(self.previous, odom_from_base, dt/NS)
        if stamp <= 0:
            raise ValueError("导航时钟无效")
        self.previous = odom_from_base
        self.last_device, self.last_receipt, self.last_ros = device, receipt, stamp
        if velocity is not None:
            return stamp, map_from_odom, odom_from_base, velocity
        return None


def publish(args, projection):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from tf2_ros import TransformBroadcaster

    def as_tf(stamp, parent, child, transform):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, NS)
        p, q = msg.transform.translation, msg.transform.rotation
        p.x, p.y, p.z = transform.p
        q.x, q.y, q.z, q.w = transform.q
        return msg

    os.environ["ROS_DOMAIN_ID"] = str(args.output_domain)
    rclpy.init(args=[])
    node = rclpy.create_node("a2_odin1_navigation_pose", start_parameter_services=False)
    broadcaster = TransformBroadcaster(node)
    odometry = node.create_publisher(Odometry, args.odom_topic, 10)
    # PoseWithCovariance/TwistWithCovariance 没有通用的未知方差标志。给出保守的大对角值，
    # 避免全零被当作精确测量，也避免向控制器传入 NaN。此桥自身不估计统计协方差。
    covariance = [1e6 if i % 7 == 0 else 0.0 for i in range(36)]
    previous_status = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.025)
            try:
                if node.count_publishers(args.odom_topic) != 1:
                    raise ValueError("等待导航里程计话题的唯一发布者")
                data = read_object(args.pose_file)
                stamp = projection.sample(data, time.monotonic_ns(), node.get_clock().now().nanoseconds)
                status = previous_status or "等待两帧连续机身位姿"
                if stamp is not None:
                    ros_stamp, map_from_odom, odom_from_base, velocity = stamp
                    status = "导航位姿输出中"
                    map_tf = as_tf(ros_stamp, "map", args.odom_frame, map_from_odom)
                    base_tf = as_tf(ros_stamp, args.odom_frame, args.base_frame, odom_from_base)
                    broadcaster.sendTransform([map_tf, base_tf])
                    msg = Odometry()
                    msg.header, msg.child_frame_id = base_tf.header, base_tf.child_frame_id
                    pos = msg.pose.pose.position
                    pos.x, pos.y, pos.z = odom_from_base.p
                    msg.pose.pose.orientation = base_tf.transform.rotation
                    # 此处仅构造二维导航所用的机身速度估计。其余三个分量依平面模型为零；
                    # 这些值和差分速度均不能充当底盘传感器的停稳证据。
                    msg.pose.covariance = covariance
                    msg.twist.covariance = covariance
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = velocity
                    odometry.publish(msg)
            except (OSError, ValueError, KeyError, OverflowError, TypeError, RecursionError) as error:
                projection.invalidate()
                status = f"暂停发布：{error}"
            if status != previous_status:
                print(status, flush=True)
                previous_status = status
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-file", required=True, type=Path, help="同机新版 ODIN1 观测器输出的 JSON")
    parser.add_argument("--map-sha256", required=True, help="本次设备重定位 BIN 的小写 SHA-256")
    parser.add_argument("--extrinsics", required=True, type=Path, help="带实测来源和 base_frame 的 T_base_from_imu")
    parser.add_argument("--output-domain", required=True, type=int, help="与厂商驱动隔离的导航 ROS 域")
    parser.add_argument("--odom-frame", default="odin_nav_odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--odom-topic", default="/odom")
    args = parser.parse_args(argv)
    if not re.fullmatch("[0-9a-f]{64}", args.map_sha256) or not 0 <= args.output_domain <= 232:
        parser.error("请核对地图摘要和导航 ROS 域")
    if (not all(re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_]*", s) for s in (args.odom_frame, args.base_frame))
            or len({"map", "odom", "imu", args.odom_frame, args.base_frame}) != 5):
        parser.error("局部里程计和机身帧必须独立命名，不能复用厂商的 map、odom、imu 帧")
    try:
        ext = load_extrinsics(args.extrinsics, args.base_frame)
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        publish(args, BodyProjection(args.map_sha256, boot_id, args.output_domain, ext))
    except (OSError, ValueError, KeyError, TypeError, OverflowError, RuntimeError, ImportError) as error:
        print(f"无法启动 ODIN1 机身位姿桥：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
