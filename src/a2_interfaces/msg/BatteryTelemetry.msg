# 一块电池一次 BMS 回调的观测值。时间来自 PC2 的 CLOCK_MONOTONIC；
# 缺少样本或采集超时填 0。该时间只与同一台 PC2 的单调时钟比较。
uint64 received_mono_ns
# SOC、SOH 为百分比，范围 0–100；循环次数来自 BMS 的 cycle。
# 各数值独立校验，-1 表示该字段未知，0 是合法实测值。
int16 soc_percent
int16 soh_percent
int32 cycle_count
# 当前机型未核实充电状态位的定义，填 unknown。
string charging_state
