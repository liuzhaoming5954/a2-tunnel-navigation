# 与 PlatformState 使用相同 robot_id 和来源会话。消息按固定频率汇总，
# header.stamp 为 ROS 发布时间，不能代替各传感器最后一次实际回调的时间。
# 各组数据来源不同，header.frame_id 留空，不声明统一的坐标系。
uint16 schema_version
std_msgs/Header header
string robot_id
string source
string source_session_id
bool simulated
# 采集器使用的源样本有效期，消费方在采集器停止后仍能按相同期限撤下读数。
uint64 max_age_ms

# 分别来自 rt/lf/bmsstate 和 rt/lf/secondary_bmsstate。
BatteryTelemetry primary_battery
BatteryTelemetry secondary_battery

# 运动状态中的三轴速度（m/s）和偏航角速度（rad/s），保留原厂分量顺序。
# 原厂线速度参考系尚待核实，不能用作 base_link 速度或地图速度。
# 时间是 PC2 的单调时钟纳秒。无新鲜样本填 0，未知分量为 NaN。
uint64 velocity_received_mono_ns
float64[3] velocity_mps
float64 yaw_rate_radps

# rt/lf/lowstate 的机身 IMU 姿态，原厂顺序为横滚、俯仰、偏航（rad）。
# 独立于 ODIN1 地图坐标与航向。
uint64 attitude_received_mono_ns
float64[3] rpy_rad
