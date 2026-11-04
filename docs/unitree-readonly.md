# Unitree SDK2 可选只读状态源

本轮提供接收状态的可选 C++ 适配源，默认构建不包含它。**尚未安装、编译或链接真实 SDK，也未连接 A2；现阶段不能宣称实机状态接入已通过。** 模拟源的测试不能证明厂商 DDS、固件或 ARM64 ABI 兼容。

## 1. 已核实的上游契约

2026-11-04 检查 Unitree 官方仓库提交 `8e1b927586712edd6a4c29d50dd52dff1e163439`。该 SHA 是本轮阅读依据，正式发布前仍需纳入项目依赖锁并在目标实机验证。

| 项目 | 官方源码证据与本轮决定 |
|---|---|
| A2 状态 topic | 官方 `example/a2/sport/a2_sport_state.cpp` 使用 `rt/lf/sportmodestate`。适配仅订阅此状态 topic。 |
| IDL | 同一 A2 示例明确包含 `unitree/idl/go2/SportModeState_.hpp` 并使用 `unitree_go::msg::dds_::SportModeState_`。采用该头文件基于 A2 官方示例，不是推测 Go2 与 A2 相同。 |
| 初始化 | 示例调用 `ChannelFactory::Instance()->Init(0, networkInterface)`；适配要求显式提供机内网卡，domain 非负。实际 domain 与网卡在 M0 核实。 |
| Subscriber | `ChannelSubscriber<MSG>` 支持 topic 构造、`InitChannel(function<void(const void*)>, queue_length)` 与 `CloseChannel()`。本轮设置队列长度为 1。 |
| 构建 target | 官方安装配置提供 `find_package(unitree_sdk2)`，导入 target 名称为 `unitree_sdk2`，并关联 `ddsc`、`ddscxx`、`Threads::Threads`。默认 mock 不查找或链接这些依赖。 |

对应源码：[A2 状态示例](https://github.com/unitreerobotics/unitree_sdk2/blob/8e1b927586712edd6a4c29d50dd52dff1e163439/example/a2/sport/a2_sport_state.cpp)、[ChannelSubscriber](https://github.com/unitreerobotics/unitree_sdk2/blob/8e1b927586712edd6a4c29d50dd52dff1e163439/include/unitree/robot/channel/channel_subscriber.hpp)、[ChannelFactory](https://github.com/unitreerobotics/unitree_sdk2/blob/8e1b927586712edd6a4c29d50dd52dff1e163439/include/unitree/robot/channel/channel_factory.hpp)、[安装 targets](https://github.com/unitreerobotics/unitree_sdk2/blob/8e1b927586712edd6a4c29d50dd52dff1e163439/cmake/unitree_sdk2Targets.cmake)。

## 2. 当前字段映射与限制

`mode()` 以厂商原始数值发布，尚不把某个数值解释为“原厂已释放控制权”。每次状态 callback 使用本进程 `steady_clock` 记录接收时刻并递增样本序号；重复读取缓存不会刷新接收时间。该时刻不是传感器采集时间，不与另一进程的单调时钟直接比较。

本轮检查的 [SportModeState_ IDL 生成头](https://github.com/unitreerobotics/unitree_sdk2/blob/8e1b927586712edd6a4c29d50dd52dff1e163439/include/unitree/idl/go2/SportModeState_.hpp) 含 `velocity()` 与 `yaw_speed()`，但该头和 A2 示例没有给出足以确认机身坐标映射的定义；也没有电池电量字段。因此适配保持 `body_velocity` 和 `battery_percent` 缺失，不能用零速或零电量代替未知。后续在原厂字段与实机坐标验证后再扩充。

收到 mode 和新鲜报文只说明状态传输有数据，不证明机器人健康、静止、定位有效或允许运动；下游不得以此授予运动许可。该状态源不读取原厂控制权协议，也未提供电池或完整诊断适配。

适配不包含运动客户端、业务命令 publisher 或速度指令入口；DDS 发现本身仍产生通信，故只读不表示网卡完全无发送流量。

## 3. 生命周期与启用关口

回调仅缓存数据，不访问 ROS、任务数据库或设备控制。缓存由 mutex 保护，回调捕获共享缓存而非源对象地址；析构先关闭接收标志再 `CloseChannel()`。工厂是 SDK 的进程级单例，适配不在 reader 析构时释放全局工厂；同一进程不允许热切换网卡/domain，变更配置需退出并重新启动。

启用 SDK 构建前，确认：

1. 原厂许可、SDK SHA、子依赖与目标 ARM64 库已登记；按 M0 记录 A2 固件、JetPack/L4T、Ubuntu、ROS 与网卡。
2. 目标安装目录能被 `find_package(unitree_sdk2)` 找到；显式启用而缺 SDK 时构建必须失败，不能降级成看似真实的 mock。
3. 在只读 profile 下验证状态 topic/类型、回调速率、断流后的年龄增长和失效诊断，确认未创建任何运动客户端。
4. 验证 SDK DDS 与 ROS DDS 的同进程配置、动态库及退出行为；本机 ROS domain 与 SDK domain 分别记录，不能因 ROS 环境变量设置就推断厂商链路也已隔离。

当前不自动下载 SDK、不安装厂商库、不连接机器人。实机启用步骤由平台只读节点文档与现场 M0 记录共同约束。
