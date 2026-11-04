# a2_platform

状态：`in_progress`。当前提供 C++17 只读 ROS 状态节点、独立状态校验库和 mock 源；默认构建不查找或链接 Unitree SDK，不包含运动入口。

- `readonly_node` 发布 `/a2/platform/state`、`/diagnostics`，提供 `/a2/platform/get_state`。
- 源进程 steady clock 计算年龄，重复发布/查询不刷新样本接收时刻或序号；无样本、陈旧、缺失或非法字段均明确诊断。
- 缺失数值保留 NaN/无效标记；mock 明示 simulated=true，所有源均 motion_capable=false。
- `A2_ENABLE_UNITREE_SDK` 默认 OFF。启用后才编译可选的接收适配；未安装 SDK 却显式启用时构建失败，运行时选择未编译的源也报错。
- 可选原厂源已按官方 A2 示例核对 topic/类型，仅映射 mode 和接收时序；电量、机身速度缺少可靠映射，保持不可用。本轮未编译或连接真实 SDK。

本机构建、演示和测试见[只读节点指南](../../docs/platform-readonly.md)，上游依据见[SDK说明](../../docs/unitree-readonly.md)。公共消息由[a2_interfaces](../a2_interfaces/README.md)提供。真实数据可用性不等于任务健康、原厂释放或运动许可，不能将 mode 直接解释为可运动。

后续才实现经验证的电池/速度/IMU/相机接入和底盘运动桥接。当前 `robot_motion`、全套profile启动器及与任务核心的ROS连接仍未实现。
