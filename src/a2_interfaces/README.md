# a2_interfaces

状态：`in_progress`，公共 ROS 接口 **v0.2 草案**。已添加 rosidl 消息/服务定义与 ament_cmake 构建元数据，依赖仅为标准 `std_msgs`、`geometry_msgs` 和 ROS 接口生成工具，不依赖业务实现。目标 A2/Humble 的实机接口尚未冻结；本机生成结果由集成验证记录确认。当前仍未发布外部稳定协议，`schema_version` 保持 `1`；草案版本与字段变化不代表已建立跨版本兼容承诺。

| 接口 | 用途 |
|---|---|
| `msg/PlatformState` | 平台只读观察、样本来源/年龄与分字段有效性 |
| `srv/GetPlatformState` | 无控制副作用地查询当前平台观察 |
| `msg/MissionStatus` | 任务当前状态和进度观察，包含设计方案的 13 个状态 |
| `msg/CommandReceipt` | 某条命令原始受理/拒绝结果，区别于当前任务状态 |
| `srv/MissionCommand` | mock-only 任务准备、准入与状态变更请求，返回历史收据和当前观察 |
| `srv/GetMissionState` | 无状态变更地查询当前任务观察及存储锁存状态 |

### 平台观察约定

`schema_version` 当前填写 `1`。`header.stamp` 是本机 ROS 接收或发布时间，不能冒充厂商采集时间；`header.frame_id` 给出测得 `body_velocity` 的坐标系。`source`、`source_session_id` 和 `simulated` 必须保留，不把模拟数据伪装成实机数据。

`sample_seq` 只在接收新原始样本时递增，重新发布不刷新它或 `received_mono_ns`。后者属于来源进程的 steady clock，仅用于该时钟域内追溯，不能直接与其他进程/机器的单调时间比较；`age_ms` 是发布端计算的样本年龄。

只读节点的 `motion_capable` 始终为 `false`。`valid` 及诊断原因用于观察质量，不能作为运动许可或定位健康的替代。消费方同时检查分字段有效性；`battery_valid=false` 时电量使用 NaN，不能把未知值显示为正常 0%。`velocity_valid=false` 时不解释 Twist；该 Twist 始终是测得速度，不是底盘命令。

`GetPlatformState.available` 表示已经存在原始样本。尚无样本时为 `false`；有陈旧样本时仍为 `true`，但 `state.valid=false`，不能只检查 `available` 就认为平台健康。

### 任务状态与历史收据

`MissionStatus` 的 13 个任务状态常量保持 `0..12`，无当前任务时使用 `STATE_UNAVAILABLE=255`；业务层字符串枚举由 ROS 适配器显式映射，不能按枚举声明顺序猜测。消息不包含启动凭据、授权 token 或运动许可。当前 mock 任务节点固定发布 `simulated=true`、`motion_capable=false`，不能把模拟自检结果当作实机健康证据。

`CommandReceipt.state` 对应同一组状态，表示命令原始受理/拒绝时的结果；没有任务状态时使用 `STATE_UNAVAILABLE=255`。重启后重复请求可以返回旧成功收据，同时当前 `MissionStatus` 仍为故障暂停，消费方应分别显示。`controller_session_id` 用于识别原受理会话。

### mock 任务命令与查询

`MissionCommand` 白名单为 `prepare`、`authorize`、`start`、`pause`、`resume`、`cancel`、`clear_fault`、`record_visit`、`inject_fault`。请求使用显式字段；自检、健康、定位、停稳观测由 mock 适配内部提供，调用者不能通过服务传入这些布尔量。`local_confirmation` 和 `operator_id` 只服务于模拟准入流程，尚未提供真实操作者身份认证或实机控制授权。

响应的 `processed=true` 仅表示取得核心收据，仍须检查 `receipt.accepted` 和 `receipt.code`；服务边界拒绝或数据库失败可返回 `processed=false` 及错误码。`receipt` 是历史结果，`current` 是当前观察，两者不可混为一项。`has_mission=false` 时当前状态使用 `STATE_UNAVAILABLE`，不能显示为 READY。

`authorization_id` 仅在 `authorize` 的直接服务响应返回，不出现在任务状态或收据 topic 中。重试可能返回旧凭据及旧成功收据；它们不能恢复 controller 重建后已清除的准入状态。凭据属于任务准入，不是运动许可。

`GetMissionState.available` 表示能否读取控制器观察：空闲、尚无任务时仍可为 `true`，同时 `has_mission=false`；数据库读取失败时为 `false`，错误通过 `error_code` 表达。`storage_fault_latched` 不能由健康信号恢复自动清除；`mock_platform_healthy` 只用于等待模拟适配就绪，不是传感器或运动健康证明。

本包只定义 ROS 契约，服务实现位于任务适配器。当前任务服务仅用于无硬件 mock 流程；原装雷达、IMU 和摄像头已由用户确认可用，优先按[录包指南](../../docs/mapping-capture.md)核查真实数据并验证建图，新增巡检载荷接入后移。平台查询服务继续保持只读，没有运动接口。维护者身份为明显占位，`LicenseRef-Proprietary` 不授予开源许可。

在已加载目标 ROS underlay 的工作区可构建并检查定义：

```bash
colcon build --base-paths src --packages-select a2_interfaces
source install/local_setup.bash
ros2 interface show a2_interfaces/msg/PlatformState
ros2 interface show a2_interfaces/srv/GetPlatformState
ros2 interface show a2_interfaces/srv/MissionCommand
ros2 interface show a2_interfaces/srv/GetMissionState
```

生成通过只证明该构建环境能处理接口，不代表 A2 SDK、ARM64 或实机时序已验证。接口与配置变更同步更新[架构约定](../../docs/architecture.md)及[开发流程](../../docs/development.md)。
