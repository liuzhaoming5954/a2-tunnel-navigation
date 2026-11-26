#!/usr/bin/env python3
"""仅离线分析 A2 停车反馈。用 PC2 单调时钟给出触发至双路停稳的时间范围。"""

import argparse
from bisect import bisect_left, bisect_right
import json
import math
from pathlib import Path

if __package__:
    from .a2_remote_stop_sdk import (DEFAULT_MODE, HOLD_S, MAX_DRIFT_M, MAX_GAP_S, MAX_TILT_RAD,
                                     REMOTE_TOPIC, SPORT_TOPIC, STILL_MPS, STILL_RAD_S, _TimedSamples,
                                     _forward_tick)
else:
    from a2_remote_stop_sdk import (DEFAULT_MODE, HOLD_S, MAX_DRIFT_M, MAX_GAP_S, MAX_TILT_RAD,
                                    REMOTE_TOPIC, SPORT_TOPIC, STILL_MPS, STILL_RAD_S, _TimedSamples,
                                    _forward_tick)


NS = 1_000_000_000
GAP_NS = round(MAX_GAP_S * NS)
TOPICS = (SPORT_TOPIC, REMOTE_TOPIC)
TIME_LIMIT_NS = NS
GROUND_OFFSET_LIMIT_M = 0.10


def require(condition, message):
    if not condition:
        raise ValueError(message)


def positive_int(value):
    return type(value) is int and value > 0


def vector(values, length):
    require(isinstance(values, list) and len(values) == length and all(
        type(value) in (int, float) and math.isfinite(value) for value in values), "记录向量不完整或非有限")
    return values


def load_record(rows, earliest, latest, boot_id):
    require(len(rows) >= 4 and rows[0]["kind"] == "metadata" and rows[-1]["kind"] == "end",
            "记录缺少完整的起止标记")
    meta, end = rows[0], rows[-1]
    require(type(meta["schema_version"]) is int and meta["schema_version"] == 1
            and meta["clock"] == "CLOCK_MONOTONIC" and tuple(meta["topics"]) == TOPICS,
            "记录来源、版本或时钟不符")
    require(isinstance(meta["boot_id"], str) and meta["boot_id"] and meta["boot_id"] == end["boot_id"]
            and (boot_id is None or boot_id == meta["boot_id"]), "触发事件与采集记录不在同一次 PC2 开机")
    require(positive_int(earliest) and positive_int(latest)
            and positive_int(meta["started_ns"]) and positive_int(meta["deadline_ns"])
            and positive_int(end["ended_ns"])
            and meta["started_ns"] < earliest <= latest < meta["deadline_ns"] <= end["ended_ns"],
            "触发范围无效，或超出完整记录窗口")
    require(end["complete"] is True and not any(end["invalid"].values()) and not any(end["dropped"].values()),
            "记录不完整、有无效帧或有队列丢失")
    streams = {topic: [] for topic in TOPICS}
    for row in rows[1:-1]:
        if row["kind"] == "marker":
            # 标准输入标记的时间是 PC2 收到文本的时间。本工具不把它当成实体触发。
            continue
        require(row["kind"] == "sample" and row["topic"] in streams, "记录含未知或无效帧")
        timestamp = row["received_monotonic_ns"]
        require(positive_int(timestamp) and meta["started_ns"] <= timestamp < meta["deadline_ns"],
                "收到样本的时刻超出记录窗口")
        require(positive_int(row["receive_count"]) and positive_int(row["dds_writer"])
                and positive_int(row["dds_source_ns"]) and positive_int(row["source_ns"])
                and row["source_clock"] in {"dds.source_timestamp", "sport.stamp"}
                and (row["topic"] != REMOTE_TOPIC or row["source_clock"] == "dds.source_timestamp"),
                "样本计数、写者或源时间无效")
        streams[row["topic"]].append(row)
    for topic in TOPICS:
        stream = streams[topic]
        # SDK 两路回调在队列中可交错。各路先保留记录器计数次序，再核对本机时钟。
        require(positive_int(end["received"].get(topic)) and len(stream) == end["received"][topic]
                and all(row["receive_count"] == count for count, row in enumerate(stream, start=1)),
                "记录器样本计数不连续")
        require(all(a["received_monotonic_ns"] < b["received_monotonic_ns"] for a, b in zip(stream, stream[1:])),
                "某一路的 PC2 接收时刻未递增")
    return meta, streams


def quiet(row):
    # 与原有站立停车判据一致。这份报告不把下趴反馈视作站立停车。
    if row["topic"] == SPORT_TOPIC:
        identity = [row[key] for key in ("mode", "gait_type", "error_code")]
        require(all(type(value) is int for value in identity), "机身模式、步态或故障字段无效")
        yaw = row["yaw_speed_rad_s"]
        require(type(yaw) in (int, float) and math.isfinite(yaw), "机身转速无效")
        velocity = vector(row["velocity_mps"], 3)
        vector(row["position_m"], 3)
        return identity == [DEFAULT_MODE, 0, 0] and math.hypot(*velocity) <= STILL_MPS and abs(yaw) <= STILL_RAD_S

    quaternion = vector(row["quaternion_wxyz"], 4)
    gyro = vector(row["gyroscope_rad_s"], 3)
    tick = row["tick"]
    require(type(tick) is int and 0 <= tick < 2**32, "低层设备计数无效")
    norm = math.hypot(*quaternion)
    if row["quaternion_valid"] is not True or not 0.95 <= norm <= 1.05:
        return False
    _, x, y, _ = quaternion
    cosine = 1 - 2 * (x*x + y*y) / (norm*norm)
    return cosine >= math.cos(MAX_TILT_RAD) and max(map(abs, gyro)) <= STILL_RAD_S


def contiguous(previous, current):
    require(previous["dds_writer"] == current["dds_writer"] and previous["source_clock"] == current["source_clock"],
            "测量区间内 DDS 写者或源时钟切换")
    # 用两路独立序号排除缓存、倒序以及采样间隔过大的窗口。
    for name in ("received_monotonic_ns", "source_ns", "dds_source_ns"):
        require(0 < current[name] - previous[name] < GAP_NS, "测量区间内反馈断流或源时间未推进")
    if current["topic"] == REMOTE_TOPIC:
        require(_forward_tick(previous["tick"], current["tick"]), "测量区间内低层计数未推进")


def find_stationary(streams, earliest, latest):
    start_at = {}
    for topic, stream in streams.items():
        times = [row["received_monotonic_ns"] for row in stream]
        index = bisect_left(times, earliest) - 1
        require(index >= 0 and 0 < earliest - times[index] < GAP_NS, "缺少触发前两路连续反馈")
        start_at[topic] = index
    body_before = streams[SPORT_TOPIC][start_at[SPORT_TOPIC]]
    require(not quiet(body_before) and math.hypot(*body_before["velocity_mps"][:2]) > STILL_MPS,
            "触发前没有观测到高于阈值的水平运动")

    events = sorted(
        (row for topic, stream in streams.items() for row in stream[start_at[topic]:]),
        key=lambda row: row["received_monotonic_ns"],
    )
    windows = {topic: _TimedSamples() for topic in TOPICS}
    previous = {}
    # 只用确定晚于物理触发上界的样本累计停稳。两路 DDS 时间只比较相对推进。
    for row in events:
        topic, timestamp = row["topic"], row["received_monotonic_ns"]
        is_quiet = quiet(row)
        if topic in previous:
            contiguous(previous[topic], row)
        previous[topic] = row
        position = row["position_m"] if topic == SPORT_TOPIC else True
        windows[topic].receive(timestamp / NS, (row["source_clock"], row["dds_writer"]), row["source_ns"],
                               position if timestamp >= latest and is_quiet else None)
        if not all(window.frames and timestamp - round(window.frames[-1][0] * NS) < GAP_NS
                   for window in windows.values()):
            continue
        end = min(window.frames[-1][0] for window in windows.values())
        start = end - HOLD_S
        spans = {name: window.window(latest / NS, start, end) for name, window in windows.items()}
        if not all(spans.values()):
            continue
        xyz = [frame[1] for frame in spans[SPORT_TOPIC]]
        if math.hypot(*(max(axis) - min(axis) for axis in zip(*xyz))) > MAX_DRIFT_M:
            continue
        t1 = round(max(span[0][0] for span in spans.values()) * NS)
        # 不把慢路数据末刻当成系统已知时刻。以两路证据首次均已到达的 PC2 时刻作为 t2。
        return t1, timestamp, round(end * NS)
    raise ValueError("未取得两路共同连续 0.50 秒的站立停稳证据")


def odometry(body, earliest, latest, end):
    times = [row["received_monotonic_ns"] for row in body]
    first = bisect_left(times, earliest) - 1
    last_trigger = bisect_right(times, latest)
    last = bisect_left(times, end)
    require(0 <= first < last_trigger < len(body) and last < len(body), "里程计缺少触发或停稳时刻的边界采样")
    for a, b in zip(body[first:last], body[first + 1:last + 1]):
        contiguous(a, b)
    # 将触发区间两侧相邻采样也纳入起点范围。采样间的实际路径仍未知，此数值仅用作里程计对照。
    origins = [vector(row["position_m"], 3)[:2] for row in body[first:last_trigger + 1]]
    xs, ys = zip(*origins)
    corners = [(x,y) for x in (min(xs), max(xs)) for y in (min(ys), max(ys))]
    observations = [vector(row["position_m"], 3)[:2] for row in body[first:last + 1]]
    return {
        "max_sampled_xy_offset_m": max(math.dist(point, corner) for point in observations for corner in corners),
        "trigger_bracket_ns": [times[first], times[last_trigger]],
        "confirmation_bracket_ns": [times[last - 1], times[last]],
        "independent_ground_distance": False,
    }


def analyze(rows, earliest, latest, *, boot_id=None, time_limit_ns=TIME_LIMIT_NS):
    # 各用例只选择验收时限，实际计时和双路停稳判据共用。
    require(positive_int(time_limit_ns), "验收时限须为正整数纳秒")
    result = {
        "telemetry_only": True,
        "measurement_valid": False,
        "ground_distance_verified": False,
        "overall_acceptance": None,
        "time_limit_s": time_limit_ns / NS,
        "ground_offset_limit_m": GROUND_OFFSET_LIMIT_M,
        "required_dual_stream_hold_s": HOLD_S,
        "trigger_interval_ns": [earliest, latest],
    }
    try:
        meta, streams = load_record(rows, earliest, latest, boot_id)
        t1, t2, common_end = find_stationary(streams, earliest, latest)
        evidence = odometry(streams[SPORT_TOPIC], earliest, latest, t2)
    except (KeyError, ValueError, TypeError, OverflowError) as error:
        return {**result, "reason": str(error)}

    t2_lower, t2_upper = t2 - latest, t2 - earliest
    if t2_upper <= time_limit_ns:
        timing = "within_limit"
    elif t2_lower > time_limit_ns:
        timing = "exceeds_limit"
    else:
        timing = "trigger_uncertainty_crosses_limit"
    return {
        **result,
        "measurement_valid": True,
        "boot_id": meta["boot_id"],
        "t1_first_observed_still_ns": t1,
        "t2_dual_stream_confirmation_ns": t2,
        "dual_stream_common_end_ns": common_end,
        "observed_t1_minus_trigger_s": [(t1 - latest) / NS, (t1 - earliest) / NS],
        "observed_t2_minus_trigger_s": [t2_lower / NS, t2_upper / NS],
        "telemetry_time_assessment": timing,
        "chassis_odometry": evidence,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="record_a2_parking_feedback.py 输出的 JSONL")
    parser.add_argument("--trigger-earliest-ns", required=True, type=int, help="同一 PC2 单调时钟上的触发下界")
    parser.add_argument("--trigger-latest-ns", required=True, type=int, help="同一 PC2 单调时钟上的触发上界")
    parser.add_argument("--boot-id", help="来自独立触发记录的 PC2 开机标识，指定后必须与反馈记录一致")
    args = parser.parse_args()
    with args.recording.open(encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream]
    report = analyze(rows, args.trigger_earliest_ns, args.trigger_latest_ns, boot_id=args.boot_id)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
    return 0 if report["measurement_valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
