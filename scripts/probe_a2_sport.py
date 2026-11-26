#!/usr/bin/env python3
"""读取 A2 运动服务、机身速度和遥控输入，在标准输出打印一次 JSON。"""

import argparse
from contextlib import redirect_stdout
from functools import partial
import json
import math
import struct
import sys
from threading import Lock
import time


SPORT_TOPICS = ("rt/sportmodestate", "rt/lf/sportmodestate")
REMOTE_TOPIC = "rt/wirelesscontroller"
DIRECT_REMOTE_TOPICS = (REMOTE_TOPIC, "rt/wirelesscontroller_unprocessed")
LOW_STATE_TOPICS = ("rt/lf/lowstate", "rt/lowstate")
REMOTE_TOPICS = (*DIRECT_REMOTE_TOPICS, *LOW_STATE_TOPICS)
REMOTE_AXIS_THRESHOLD = 0.15


def create_clients(interface, domain=0, timeout_s=1.0):
    """在指定网卡创建 A2 专用客户端，只注册 RPC 并设置客户端等待时间。"""
    from unitree_sdk2py.a2.sport.sport_client import SportClient
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    if not interface or not math.isfinite(timeout_s) or timeout_s <= 0 or domain < 0:
        raise ValueError("需要指定网卡、非负域编号和有限的正超时")
    ChannelFactoryInitialize(domain, interface)
    sport, switcher = SportClient(), MotionSwitcherClient()
    for client in (sport, switcher):
        client.SetTimeout(timeout_s)
        client.Init()
    return sport, switcher


def read_status(sport, switcher):
    """分别保留三个只读 RPC 的返回码，一路超时时仍收集其余证据。"""
    def get_state():
        state = {}
        return sport.GetState(state), state

    result = {}
    for name, query in (("server_version", sport.GetServerApiVersion),
                        ("sport_state", get_state), ("motion_mode", switcher.CheckMode)):
        try:
            code, value = query()
            result[name] = {"code": code, "value": value}
        except Exception as exc:
            result[name] = {"code": None, "error": f"{type(exc).__name__}: {exc}"}
    return result


def finite_number(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("状态字段不是有限值")
    return number


def vector3(values):
    if len(values) != 3:
        raise ValueError("状态向量长度应为 3")
    return [finite_number(value) for value in values]


def low_state_imu(message):
    """读取同帧 HG IMU。部分 A2 固件会留下全零姿态，不能用作姿态证据。"""
    try:
        imu = message.imu_state
        if len(imu.quaternion) != 4:
            raise ValueError("四元数长度应为 4")
        # 宇树实机示例明确按 w,x,y,z 读取此 IDL 字段，原分量顺序和精度均予以保留。
        quaternion = [finite_number(value) for value in imu.quaternion]
        if not 0.95 <= math.hypot(*quaternion) <= 1.05:
            raise ValueError("姿态四元数模应接近 1")
        gyro = vector3(imu.gyroscope)
        return {"valid": True, "quaternion_wxyz": quaternion, "gyroscope_rad_s": gyro}
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        return {"valid": False, "quaternion_wxyz": None, "gyroscope_rad_s": None, "error": str(exc)}


def remote_sample(topic, message):
    if topic in LOW_STATE_TOPICS:
        # 字节布局取自固定版本原厂 example/wireless_controller/wireless_controller.py。
        # 先核对长度，再按小端解析四个轴；偏移 16 是原厂样例标注的保留字段。
        data = bytes(message.wireless_remote)
        if len(data) != 40:
            raise ValueError("LowState.wireless_remote 应有 40 字节")
        lx, rx, ry, ly = struct.unpack_from("<fff4xf", data, 4)
        axes = dict(lx=lx, ly=ly, rx=rx, ry=ry)
        keys = int.from_bytes(data[2:4], "little")
    else:
        axes = {axis: float(getattr(message, axis)) for axis in ("lx", "ly", "rx", "ry")}
        keys = int(message.keys)
    if not 0 <= keys <= 65535 or any(not math.isfinite(v) or abs(v) > 1.05 for v in axes.values()):
        raise ValueError("遥控轴或按键位掩码超出可解析范围")
    return {"axes": axes, "keys": keys}


class Feedback:
    """缓存各 DDS 话题的最后有效样本。回调只读消息并更新内存。"""

    def __init__(self, remote_topic=None):
        if remote_topic is not None and remote_topic not in REMOTE_TOPICS:
            raise ValueError(f"不支持的遥控话题 {remote_topic}")
        self._remote_topics = (remote_topic,) if remote_topic else REMOTE_TOPICS
        self._lock = Lock()
        self._samples = {}
        self._errors = {}
        self._readers = []

    def start(self):
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, WirelessController_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        if self._readers:
            raise RuntimeError("订阅已经启动")
        # A2 原厂 Python 示例使用 lf 话题，现场也可能发布常规频率的话题。
        # 当前 A2 的 DDS 发布端使用 unitree_hg/LowState_，运动与遥控专用话题使用 unitree_go。
        for topic in (*SPORT_TOPICS, *self._remote_topics):
            msg_type = SportModeState_
            if topic in DIRECT_REMOTE_TOPICS:
                msg_type = WirelessController_
            elif topic in LOW_STATE_TOPICS:
                msg_type = LowState_
            reader = None
            try:
                reader = ChannelSubscriber(topic, msg_type)
                reader.Init(partial(self._receive, topic))
                self._readers.append(reader)
            except Exception as exc:
                self._remember_error(topic, exc)
                if reader is not None:
                    reader.Close()
        return self

    def _remember_error(self, topic, exc):
        with self._lock:
            self._errors[topic] = f"{type(exc).__name__}: {exc}"

    def _receive(self, topic, message):
        # 使用 PC2 单调时钟标记接收时间，供调用方判断新鲜度及与发令事件对齐。
        now = time.monotonic()
        try:
            if topic in REMOTE_TOPICS:
                entry = remote_sample(topic, message)
                if topic in LOW_STATE_TOPICS:
                    # 遥控和 IMU 共用同一 DDS 帧，传感器缺失时仍需保留操作者的遥控证据。
                    entry["imu"] = low_state_imu(message)
            else:
                entry = {
                    "mode": int(message.mode), "gait_type": int(message.gait_type),
                    "error_code": int(message.error_code), "velocity_mps": vector3(message.velocity),
                    "yaw_speed_rad_s": finite_number(message.yaw_speed),
                    "position_m": vector3(message.position), "rpy_rad": vector3(message.imu_state.rpy),
                }
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            self._remember_error(topic, exc)
            return
        with self._lock:
            previous = self._samples.get(topic, {})
            if previous and now <= previous["received_monotonic_s"]:
                return
            gap = now - previous.get("received_monotonic_s", now)
            entry.update(topic=topic, count=previous.get("count", 0) + 1,
                         received_monotonic_s=now, max_gap_s=max(gap, previous.get("max_gap_s", 0.0)))
            if topic in REMOTE_TOPICS:
                # 只记录遥控动作迹象。非中立样本数和实际接管次数是两个不同观测量。
                active = bool(entry["keys"]) or any(abs(v) > REMOTE_AXIS_THRESHOLD for v in entry["axes"].values())
                # 以首次采到的中立→活动边沿打点，避免长按期间持续刷新的样本缩短停车响应时间。
                # 第一帧可能来自一枚已经按住的键。没有上一帧中立依据时不生成起始时间。
                began = active and previous.get("active") is False
                entry["active"] = active
                entry["last_activity_start_monotonic_s"] = now if began else previous.get("last_activity_start_monotonic_s")
                entry["last_activity_start_keys"] = entry["keys"] if began else previous.get("last_activity_start_keys")
                entry["last_activity_start_axes"] = entry["axes"] if began else previous.get("last_activity_start_axes")
                entry["activity_count"] = previous.get("activity_count", 0) + int(active)
                entry["last_activity_monotonic_s"] = now if active else previous.get("last_activity_monotonic_s")
                entry["last_activity_keys"] = entry["keys"] if active else previous.get("last_activity_keys")
                entry["last_activity_axes"] = entry["axes"] if active else previous.get("last_activity_axes")
                entry["neutral_count"] = previous.get("neutral_count", 0) + int(not active)
                # 只有同时观测到中立和活动帧，才能证明此输入的数值发生过变化。
                entry["observed_input"] = entry["activity_count"] > 0 and entry["neutral_count"] > 0
            self._samples[topic] = entry

    def snapshot(self):
        with self._lock:
            now = time.monotonic()
            topics = {topic: dict(value, age_s=now - value["received_monotonic_s"])
                      for topic, value in self._samples.items()}
            errors = self._errors.copy()
        # 双话题同时存在时选接收时间最新的一帧，仍保留两路原始计数以便核查。
        latest_sport = max((topics[topic] for topic in SPORT_TOPICS if topic in topics),
                           key=lambda value: value["received_monotonic_s"], default=None)
        # 优先返回已经观测过活动和回中的新鲜通道。有帧但全零时保留读数并标出未验证。
        candidates = [topics[topic] for topic in self._remote_topics if topic in topics]
        remote = max(candidates, key=lambda value: (value["age_s"] <= 0.5, value["observed_input"]),
                     default=None)
        return {"sport": latest_sport, "remote": remote,
                "topics": topics, "errors": errors}

    def close(self):
        for reader in self._readers:
            reader.Close()
        self._readers.clear()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True, help="连接底盘 DDS 的 PC2 网卡")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=1.0, help="各 RPC 的客户端超时（秒）")
    parser.add_argument("--seconds", type=float, default=2.0, help="完成 RPC 后继续收样的秒数")
    parser.add_argument("--remote-topic", choices=REMOTE_TOPICS, help="只读取所选来源的遥控输入")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds 需要有限的正值")
    result = {"interface": args.interface, "domain": args.domain}
    feedback = Feedback(args.remote_topic)
    try:
        # 厂商 SDK 会将 RPC 错误打印到标准输出，把这些诊断与最终 JSON 分流。
        with redirect_stdout(sys.stderr):
            try:
                sport, switcher = create_clients(args.interface, args.domain, args.timeout)
                feedback.start()
                result["client_version"] = sport.GetApiVersion()
                result["rpc"] = read_status(sport, switcher)
                time.sleep(args.seconds)
                result["feedback"] = feedback.snapshot()
            finally:
                feedback.close()
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    if "error" in result:
        return 1
    rpc, received = result["rpc"], result["feedback"]
    return 0 if (all(item["code"] == 0 for item in rpc.values())
                 and rpc["server_version"]["value"] == result["client_version"]
                 and received["sport"] and received["sport"]["age_s"] <= 0.5
                 and received["remote"] and received["remote"]["age_s"] <= 0.5
                 and received["remote"]["observed_input"]) else 2


if __name__ == "__main__":
    sys.exit(main())
