"""机载本体详情的字段和时效约定，供 ROS 接收端与上位机 HTTP 转发共用。"""

import math


_VELOCITY = ("x_mps", "y_mps", "z_mps", "yaw_radps")
_ATTITUDE = ("roll_rad", "pitch_rad", "yaw_rad", "yaw_rate_radps")
_BATTERY = ("soc_percent", "soh_percent", "cycle_count", "charging")
IMU_YAW_RATE_MAX_AGE_MS = 250


def _value(key, value):
    # 逐项保留有效值，特别是 0% 电量、零速和零次循环。布尔量不能冒充数值。
    if key == "charging":
        return value if type(value) is bool else None
    if key in ("soc_percent", "soh_percent", "cycle_count"):
        upper = 65535 if key == "cycle_count" else 100
        return value if type(value) is int and 0 <= value <= upper else None
    try:
        return value if type(value) in (float, int) and math.isfinite(value) else None
    except OverflowError:
        return None


def _fresh_group(group, keys, elapsed_ms, transfer_ms):
    if not isinstance(group, dict):
        return None
    age, limit = group.get("age_ms"), group.get("max_age_ms")
    if type(age) is not int or type(limit) is not int or age < 0 or age + elapsed_ms + transfer_ms >= limit:
        return None
    fields = {key: _value(key, group.get(key)) for key in keys}
    if all(value is None for value in fields.values()):
        return None
    return {**fields, "age_ms": age + elapsed_ms, "max_age_ms": limit}


def fresh_details(details, elapsed_ms=0, *, transfer_ms=0):
    """机载缓存时间计入采样年龄。转发耗时仅用于排除途中失效的读数。"""
    batteries = details.get("batteries")
    if not isinstance(batteries, dict):
        batteries = {}
    attitude = _fresh_group(details.get("attitude"), _ATTITUDE, elapsed_ms, transfer_ms)
    if attitude and attitude["age_ms"] + transfer_ms >= IMU_YAW_RATE_MAX_AGE_MS:
        # 机身角度和差分转速各有有效期。转速过期后仍可显示独立有效的姿态。
        attitude["yaw_rate_radps"] = None
    return {
        "batteries": {slot: _fresh_group(batteries.get(slot), _BATTERY, elapsed_ms, transfer_ms)
                      for slot in ("primary", "secondary")},
        "velocity": _fresh_group(details.get("velocity"), _VELOCITY, elapsed_ms, transfer_ms),
        "attitude": attitude,
    }


def observation_groups(message, receipt_ns, limit_ms):
    """SDK 与网关同在 PC2 上，可直接用同一台主机的单调时钟计算采集年龄。"""
    def sample(source_ns, fields):
        # 先在收包时剔除未来时间和未采到的零时间，稍后轮询不能把这两种情况变为有效。
        if source_ns <= 0 or source_ns > receipt_ns:
            return None
        return {**fields, "age_ms": (receipt_ns - source_ns) // 1_000_000, "max_age_ms": limit_ms}

    def battery(pack):
        # 充电标志只接受采集器明确解码的状态；SDK 当前输出 unknown。
        return sample(pack.received_mono_ns, {
            "soc_percent": pack.soc_percent, "soh_percent": pack.soh_percent,
            "cycle_count": pack.cycle_count,
            "charging": {"charging": True, "not_charging": False}.get(pack.charging_state),
        })

    # Humble 的 rosidl 会把 float64[3] 解码为 NumPy 数组。只在已知 IDL
    # 浮点字段的边界转成 Python float，HTTP 入口仍使用严格的 JSON 类型校验。
    velocity = dict(zip(_VELOCITY, map(float, (*message.velocity_mps, message.yaw_rate_radps))))
    attitude = dict(zip(_ATTITUDE, map(float, message.rpy_rad)))
    return {
        "batteries": {"primary": battery(message.primary_battery), "secondary": battery(message.secondary_battery)},
        "velocity": sample(message.velocity_received_mono_ns, velocity),
        "attitude": sample(message.attitude_received_mono_ns, attitude),
    }
