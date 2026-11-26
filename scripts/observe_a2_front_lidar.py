#!/usr/bin/env python3
"""旁读 PC2 前雷达，记录有限截面内的最近回波，供锁存下静止联调。"""

import argparse
import json
from pathlib import Path
import tempfile
import time

import numpy as np


TOPIC = '/unitree/slam_lidar/points1'
FRAME = 'hesai_lidar'
SOURCE = 'front_lidar_diagnostic'
MAX_AGE_S = 0.20


def summarize_cloud(message):
    """按原厂结构轴向观察雷达轴线附近的窄截面，结果以雷达原点为参考。"""
    if message.header.frame_id != FRAME:
        raise ValueError('非预期的前雷达坐标系')
    width, height = message.width, message.height
    point_step, row_step = message.point_step, message.row_step
    if (width <= 0 or height <= 0 or point_step < 12 or row_step < width * point_step
            or len(message.data) != row_step * height or len(message.data) > 8 * 1024 * 1024):
        raise ValueError('点云行、点或数据长度无效')
    axes = {}
    for field in message.fields:
        if field.name not in ('x', 'y', 'z'):
            continue
        if (field.name in axes or field.count != 1 or field.datatype != 7
                or not 0 <= field.offset <= point_step - 4):
            raise ValueError('前雷达 XYZ 字段布局无效')
        axes[field.name] = field.offset
    if len(axes) != 3 or any(b - a < 4 for a, b in zip(sorted(axes.values()), sorted(axes.values())[1:])):
        raise ValueError('前雷达点云缺少独立的 XYZ 字段')
    dtype = '>f4' if message.is_bigendian else '<f4'
    x, y, z = (np.ndarray((height, width), dtype=dtype, buffer=message.data,
                          offset=axes[axis], strides=(row_step, point_step)) for axis in ('x', 'y', 'z'))
    # 结构轴向约定为 x 左右、y 高度、z 前向。固定窄截面供读数对照，机身外廓与近地覆盖另行核对。
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    inside = finite & (z > 0) & (np.abs(x) <= .25) & (np.abs(y) <= .10)
    return (float(z[inside].min()) if inside.any() else None, int(np.count_nonzero(finite)))


def publish(path, observation):
    # 两个只读采集进程分别保持各自的 DDS 运行环境。文件替换后另一端只会看到完整帧。
    with tempfile.NamedTemporaryFile(dir=path.parent, mode='w', encoding='utf-8', delete=False) as stream:
        staged = Path(stream.name)
        try:
            json.dump(observation, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            staged.replace(path)
        finally:
            staged.unlink(missing_ok=True)


def run(output, seconds):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    rclpy.init(args=[])
    node = rclpy.create_node('a2_front_lidar_diagnostic', enable_rosout=False, start_parameter_services=False)
    newest_stamp = 0
    count = 0
    last = None

    def receive(message):
        nonlocal newest_stamp, count, last
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (time.time_ns() - stamp) / 10**9
        observed_at = time.monotonic() - age
        if not stamp > newest_stamp or not 0 <= age < MAX_AGE_S:
            return
        newest_stamp = stamp
        try:
            near_return, points = summarize_cloud(message)
        except (ValueError, TypeError, BufferError):
            return
        if not points:
            return
        last = {'source': SOURCE, 'frame_id': FRAME, 'observed_at': observed_at,
                'near_return_m': near_return, 'point_count': points}
        publish(output, last)
        count += 1

    try:
        node.create_subscription(PointCloud2, TOPIC, receive,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.06)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'front_lidar_frames': count, 'last': last}, ensure_ascii=False))
    return 0 if count else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='本轮私有临时目录中的回波摘要文件')
    parser.add_argument('--seconds', type=int, default=10)
    args = parser.parse_args()
    if not 3 <= args.seconds <= 30 or not args.output.is_absolute() or args.output.exists():
        parser.error('输出须为尚不存在的绝对路径，采样时长取 3 至 30 秒')
    return run(args.output, args.seconds)


if __name__ == '__main__':
    raise SystemExit(main())
