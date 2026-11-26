#!/usr/bin/env python3
"""限时记录 A2 机身反馈到 JSONL。只订阅状态，可从标准输入追加观察标记。"""

import argparse
from collections import Counter
from contextlib import ExitStack
from functools import partial
import json
import math
import os
from pathlib import Path
from queue import Empty, Full, Queue
import re
import select
import struct
import sys
from threading import Lock
from time import monotonic_ns

if __package__:
    from .a2_remote_stop_sdk import REMOTE_TOPIC, SPORT_TOPIC, _dds_clock, _dds_writer, _vector, sport_clock
else:
    from a2_remote_stop_sdk import REMOTE_TOPIC, SPORT_TOPIC, _dds_clock, _dds_writer, _vector, sport_clock


NANOSECONDS = 1_000_000_000
MAX_DURATION_S = 120
LABEL = re.compile(rb"[a-zA-Z0-9][a-zA-Z0-9_.-]{0,63}")


def _sport(message):
    (clock, _), source_ns = sport_clock(message)
    yaw_rate = message.yaw_speed
    if not math.isfinite(yaw_rate) or any(
        type(getattr(message, field)) is not int for field in ("mode", "gait_type", "error_code")
    ):
        raise ValueError("机身模式、步态、故障码或偏航角速度无效")
    return {
        "source_clock": clock,
        "source_ns": source_ns,
        "mode": message.mode,
        "gait_type": message.gait_type,
        "error_code": message.error_code,
        "position_m": _vector(message.position, 3),
        "velocity_mps": _vector(message.velocity, 3),
        "rpy_rad": _vector(message.imu_state.rpy, 3),
        "yaw_speed_rad_s": yaw_rate,
    }


def _low(message):
    tick = message.tick
    if type(tick) is not int or not 0 <= tick < 2**32:
        raise ValueError("低层设备计数无效")
    imu = message.imu_state
    quaternion = _vector(imu.quaternion, 4)
    # 只记录足以区分现场人工介入的信息，不保存原始手柄字节。
    remote = bytes(message.wireless_remote)
    if len(remote) != 40:
        raise ValueError("原厂手柄反馈长度无效")
    keys = int.from_bytes(remote[2:4], "little")
    axes = _vector(struct.unpack_from("<fff4xf", remote, 4), 4)
    return {
        "source_clock": "dds.source_timestamp",
        "source_ns": message.sample_info.source_timestamp,
        "tick": tick,
        "quaternion_wxyz": quaternion,
        "quaternion_valid": 0.95 <= math.hypot(*quaternion) <= 1.05,
        "rpy_rad": _vector(imu.rpy, 3),
        "gyroscope_rad_s": _vector(imu.gyroscope, 3),
        "accelerometer_mps2": _vector(imu.accelerometer, 3),
        "remote_a_pressed": bool(keys & 0x0100),
        "remote_other_button_pressed": bool(keys & ~0x0100),
        "remote_max_abs_axis": max(map(abs, axes)),
    }


class FeedbackRecorder:
    def __init__(self, started_ns, deadline_ns):
        self.started_ns = started_ns
        self.deadline_ns = deadline_ns
        self.samples = Queue(maxsize=2048)
        self.counts = Counter()
        self.invalid = Counter()
        self.dropped = Counter()
        self.lock = Lock()

    def receive(self, topic, decode, message):
        # 回调入口记 PC2 接收时刻。原时间戳与 DDS 写者标识分别保留，不能直接把源时钟拿来与本机事件相减。
        received_ns = monotonic_ns()
        if not self.started_ns <= received_ns < self.deadline_ns:
            return
        fields = {}
        try:
            info = message.sample_info
            writer = _dds_writer(info)
            _, dds_ns = _dds_clock(info, writer)
            fields.update(dds_writer=writer, dds_source_ns=dds_ns)
            fields.update(decode(message))
        except (AttributeError, TypeError, ValueError, OverflowError, struct.error) as exc:
            fields["error"] = str(exc)
        now_invalid = "error" in fields
        with self.lock:
            # 无效帧也进入序列。队列满时不阻塞 DDS 路径，结束记录显式报告丢帧。
            self.counts[topic] += 1
            if now_invalid:
                self.invalid[topic] += 1
            item = {
                "kind": "invalid" if now_invalid else "sample",
                "topic": topic,
                "received_monotonic_ns": received_ns,
                "receive_count": self.counts[topic],
                **fields,
            }
            try:
                self.samples.put_nowait(item)
            except Full:
                self.dropped[topic] += 1


def markers(fd, pending, emit, deadline_ns):
    """非阻塞读取短标记。标记时间是 PC2 收到整行时刻，独立于实际停车触发。"""
    if fd is None or not select.select([fd], [], [], 0)[0]:
        return fd, pending
    block = os.read(fd, 4096)
    if not block:
        return None, b""
    received_ns = monotonic_ns()
    if received_ns >= deadline_ns:
        return None, b""
    *lines, pending = (pending + block).split(b"\n")
    for line in lines:
        label = line.strip()
        if LABEL.fullmatch(label):
            emit({"kind": "marker", "source": "stdin", "label": label.decode("ascii"),
                  "received_monotonic_ns": received_ns})
    # 超长行只保留一个无效前缀，防止拆段后将该行末尾误记成单独标记。
    return fd, pending if len(pending) <= 64 else b"\x00"


def record(args):
    # 使用的原厂入口仅为两个 ChannelSubscriber，整个进程不创建发布器或运动服务客户端。
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    sid = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    subscriptions = ((SPORT_TOPIC, SportModeState_, _sport), (REMOTE_TOPIC, LowState_, _low))
    stdin_fd = sys.stdin.fileno() if args.stdin_markers else None
    pending = b""
    fd = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8", buffering=1) as stream:
        def emit(row):
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")

        ChannelFactoryInitialize(args.domain, args.interface)
        started_ns = monotonic_ns()
        stop_ns = started_ns + round(args.duration_s * NANOSECONDS)
        recorder = FeedbackRecorder(started_ns, stop_ns)
        emit({"kind": "metadata", "schema_version": 1, "boot_id": sid, "clock": "CLOCK_MONOTONIC",
              "started_ns": started_ns, "deadline_ns": stop_ns, "topics": [t for t, _, _ in subscriptions]})
        complete_window = False
        try:
            # 任一订阅初始化或关闭失败都继续释放已创建的订阅，保留已落盘的数据。
            with ExitStack() as stack:
                for topic, message_type, decode in subscriptions:
                    reader = ChannelSubscriber(topic, message_type)
                    stack.callback(reader.Close)
                    reader.Init(partial(recorder.receive, topic, decode))
                print(f"已开始只读记录，最长 {args.duration_s:g} 秒。", file=sys.stderr, flush=True)
                while (remaining_ns := stop_ns - monotonic_ns()) > 0:
                    stdin_fd, pending = markers(stdin_fd, pending, emit, stop_ns)
                    try:
                        emit(recorder.samples.get(timeout=min(0.05, remaining_ns / NANOSECONDS)))
                    except Empty:
                        pass
            complete_window = True
        finally:
            # 关闭订阅后写完在截止时刻之前收到的帧。结束行用于验证录制边界和队列丢帧。
            with recorder.lock:
                while not recorder.samples.empty():
                    emit(recorder.samples.get_nowait())
                complete = complete_window and all(
                    recorder.counts[t] > 0 and not recorder.invalid[t] and not recorder.dropped[t]
                    for t, _, _ in subscriptions
                )
                emit({"kind": "end", "boot_id": sid, "ended_ns": monotonic_ns(), "complete": complete,
                      "received": recorder.counts, "invalid": recorder.invalid, "dropped": recorder.dropped})
    return complete


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True, help="已核实的 PC2 机身反馈网卡")
    parser.add_argument("--domain", type=int, default=0, help="机身反馈的 DDS 域编号")
    parser.add_argument("--duration-s", type=float, required=True, help=f"观测窗口 0.1 至 {MAX_DURATION_S} 秒")
    parser.add_argument("--output", type=Path, required=True, help="新的记录文件路径，不覆盖现有文件")
    parser.add_argument("--stdin-markers", action="store_true", help="从标准输入读入短标记，回车提交")
    args = parser.parse_args()
    if not args.interface.strip() or args.domain < 0:
        parser.error("需要指定非空网卡和非负 DDS 域编号")
    if not math.isfinite(args.duration_s) or not 0.1 <= args.duration_s <= MAX_DURATION_S:
        parser.error(f"记录时长须在 0.1 至 {MAX_DURATION_S} 秒之间")
    return 0 if record(args) else 1


if __name__ == "__main__":
    sys.exit(main())
