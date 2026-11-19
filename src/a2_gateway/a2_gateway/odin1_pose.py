"""由网关只读获取同机 PC2 观测器生成的 ODIN1 传感器位姿。"""

import json
import math
from pathlib import Path
import re
import time


SOURCE = "odin1_relocalization"
# 与现有网关的位姿新鲜度默认值一致。时间从两条同步观测中较早的接收时刻算起。
MAX_AGE_MS = 750
BOOT_ID_FILE = Path("/proc/sys/kernel/random/boot_id")


def _read_object(path):
    """只读取小型 JSON 快照，原始地图及轨迹文件不属于此接口的输入。"""
    with path.open("rb") as stream:
        data = stream.read(65_537)
    if len(data) > 65_536:
        raise ValueError("定位清单或快照过大")
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError("定位清单或快照必须为对象")
    return value


def odin1_pose_snapshot(map_dir, pose_file, versions):
    """地图版本、时效或坐标语义不满足时只返回状态，杜绝复用上一帧。"""
    result = {"source": SOURCE, "status": "waiting", "sampleSha256": None,
              "pose": None, "age_ms": None, "max_age_ms": MAX_AGE_MS}
    try:
        # 点云加载接口已按 sampleSha256 校验浏览器收到的点。高频轮询只读这份轻量清单，
        # 将已显示的点云摘要绑定到设备实际加载的 BIN 摘要，地图切换后旧视图立即失效。
        metadata = _read_object(map_dir / "metadata.json")
        sample = metadata["sampleSha256"]
        if not isinstance(sample, str) or not re.fullmatch("[0-9a-f]{64}", sample):
            raise ValueError("点云样本摘要无效")
        result["sampleSha256"] = sample
        if versions is not None and versions != [sample]:
            return {**result, "status": "map_mismatch"}
        if pose_file is None:
            return result
        observation = _read_object(pose_file)
        if observation["source"] != SOURCE:
            raise ValueError("位姿来源不符")
        status = observation["status"]
        if status in ("waiting", "stale", "invalid", "map_mismatch"):
            return {**result, "status": status}
        if status != "live" or observation["frame"] != "map" or observation["child_frame"] != "imu":
            raise ValueError("位姿坐标系或观测状态不符")
        digest = metadata.get("relocMapSha256")
        if (not isinstance(digest, str) or not re.fullmatch("[0-9a-f]{64}", digest)
                or observation["map_sha256"] != digest):
            return {**result, "status": "map_mismatch"}
        # 设备时间来自 ODIN1 自身时钟。只用同机、同一次启动的单调时钟判断接收新鲜度。
        if observation["boot_id"] != BOOT_ID_FILE.read_text(encoding="ascii").strip():
            return {**result, "status": "stale"}
        received = observation["received_monotonic_ns"]
        if (type(received) is not int or received <= 0 or type(observation["stamp_ns"]) is not int
                or observation["stamp_ns"] <= 0):
            raise ValueError("定位时间戳无效")
        age_ns = time.monotonic_ns() - received
        if age_ns < 0:
            raise ValueError("单调接收时间在未来")
        # 向上取整，使浏览器基于返回的剩余有效时间撤图标时不会多延长一毫秒。
        result["age_ms"] = (age_ns + 999_999) // 1_000_000
        if result["age_ms"] >= MAX_AGE_MS:
            return {**result, "status": "stale"}
        xyz, yaw = observation["position_m"], observation["yaw_rad"]
        if (not isinstance(xyz, list) or len(xyz) != 3 or
                not all(type(v) in (float, int) and math.isfinite(v) for v in (*xyz, yaw)) or
                abs(yaw) > math.pi):
            raise ValueError("传感器坐标或航向无效")
        return {**result, "status": "live", "pose": {"x_m": xyz[0], "y_m": xyz[1],
                                                     "z_m": xyz[2], "yaw_rad": yaw}}
    except FileNotFoundError:
        return result
    except (OSError, ValueError, TypeError, KeyError, OverflowError, RecursionError):
        return {**result, "status": "invalid"}
