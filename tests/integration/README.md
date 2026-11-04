# 集成验证

[只读 ROS smoke](../../scripts/readonly_smoke.py)实际订阅 mock 平台状态、接收诊断并调用只读服务，验证新鲜→断流→陈旧全过程。构建、加载环境及运行步骤见[只读演示](../../docs/platform-readonly.md)。脚本不接受真实 source，也不加载厂商 SDK。

任务核心与 SQLite 的合成演示仍在 `a2_mission.demo`，任务管理器另有 mock-only ROS 适配。完整 mock 业务闭环、回放与 A2 验收继续按阶段实现。

[任务 ROS smoke](../../scripts/mission_ros_smoke.py)验证 mock 启动组、服务生命周期、公共状态、重启恢复和平台断流后的任务锁存；使用临时数据库与固定模拟源。指南见[模拟任务演示](../../docs/mock-mission.md)。
