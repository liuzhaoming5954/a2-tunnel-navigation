# a2_mission

状态：`in_progress`。首个增量实现纯 Python 离线任务准入与恢复规则，通过可注入时钟和事务 journal 在本机运行；已有 mock-only ROS 服务节点，仍无底盘控制。

已实现任务准备、单活动任务、任务 ID 防复用、绑定版本的任务记录；一次性本地准入凭据绑定任务、包、boot 和 controller session，按单调时间过期。状态规则覆盖自检、定位、巡检、暂停、新授权继续、故障锁存、现场确认清除和停稳确认后取消。

完成点位去重保存；每次重建 controller 清旧凭据，保留进度且不自动继续，并隔离旧 controller 写入。同请求同内容返回历史收据，响应同时提供当前快照，避免把重启前成功收据当作当前运行状态。存储失败后拒绝后续业务命令，需诊断并重新创建 controller 进入恢复流程。

项目根目录运行：

```bash
PYTHONPATH=src/a2_storage:src/a2_mission python3 -m a2_mission.demo
PYTHONPATH=src/a2_storage:src/a2_mission python3 -m unittest discover -s src/a2_mission/test -v
```

演示使用临时 SQLite 与合成标识，覆盖未授权启动拒绝、检查通过、点位记录和同 boot 下 controller 重建：旧 Start 收据仍成功，当前状态保持故障暂停。

`MissionController.command(request_id, command, mission_id, **parameters)` 返回 `receipt` 与 `current`。后者是另一次当前观察，不与历史收据组成原子快照；并发接管时以 owner/session 和独立安全模块为准。API 输入来自可信本地适配器，尚无远程入口。

能力边界：

- `bundle_verified`、自检与定位结果是外部验证器的观察，尚不校验文件签名、传感器或真实电量。
- `local_confirmation`/`operator_id` 是授权适配的输入，尚未实现身份认证；本库准入凭据不等于 `a2_safety` 运动许可。
- `stopped_confirmed` 必须来自可信停稳观测。实际停车、撤权和超时在独立快速通道先执行，不能等待 SQLite。本库维护业务状态，无停止或运动输出。
- 示例授权 TTL 仅用于本机分支，非实机控制时限。故障恢复保留未完成的自检/定位阶段，不能直接跳到巡检。
- 低电预算、导航、返桩/交接、真实载荷/媒体、健康监控、ROS 接口和硬件适配仍待实现。

本包依赖 `a2_storage` 的小型 journal API，ROS 观察与 mock 服务已接入 `a2_interfaces`。维护者身份为显式占位，未授予开源许可。参见[架构](../../docs/architecture.md)与[开发流程](../../docs/development.md)。


## mock ROS 适配

`mission_node` 只支持 profile=mock，与平台 mock 状态和持久 SQLite 组成无硬件演示；一键启动、服务契约和限制见[指南](../../docs/mock-mission.md)。公开状态与收据带模拟标记并排除凭据，准入凭据只在直接服务响应中返回。客户端不能传入自检/定位/健康/停稳布尔量；这些由显式模拟适配提供，不构成实机验证。

服务的完整 wire 请求作为稳定去重身份，服务器当前健康观察不改变重试身份。每轮健康故障使用独立内部命令 ID，故障恢复后再次断流仍会锁存。健康时效在返回当前状态前重查，状态监控使用 steady clock，避免 ROS 时间暂停掩盖模拟断流。mock journal 与普通 core journal 不能接管彼此已有任务。
