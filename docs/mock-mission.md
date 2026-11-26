# 不依赖硬件的 ROS 模拟任务演示

此启动组合用于不连接硬件的任务软件演示与回归，固定使用平台模拟源和合成任务方案，没有 SDK、真实导航、拍照、气体采集或运动输出；模拟检查和停稳确认不作为实机证据。A2 原装雷达、IMU 和摄像头已由用户确认可用，新增巡检载荷尚未安装，当前开发优先级已转向[实机录包与建图验证](mapping-capture.md)。

## 1. 构建与自动演示

在项目根目录执行（本机已安装 Jazzy）：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to a2_bringup
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
python3 scripts/mission_ros_smoke.py
```

SDK 默认关闭。构建五个包：接口、平台模拟节点、存储、任务节点和 bringup。域号 42 用于本机演示，应与其他作业隔离；没有 ROS 通信套接字权限的执行沙箱需先获得相应权限。执行过程不需要外网或硬件。

脚本使用唯一模拟机器人 ID 和临时 SQLite，检查：

1. 新库空闲、合成任务准备、未授权启动拒绝。
2. 一次性授权后模拟自检/定位，进入巡检，记录已登记点位。
3. 相同 Start 请求返回原收据，不重复执行；历史收据与当前状态分开。
4. 暂停后旧凭据无效，新授权才能继续；注入故障后不能自行解除。
5. 完整退出并重启同一数据库，任务保持故障暂停、保留点位和版本，新 session 不接受旧凭据。
6. 模拟平台实际停止更新时，任务自动锁存故障；状态仍不健康时清除故障请求被拒。
7. 取消后状态持久保存；公共状态/收据不发布授权凭据。

脚本有限时运行，结束后清理子进程和临时数据库。返回 PASS 仅表示上述软件路径通过。

## 2. 启动持久演示并查询

同样加载环境并设置本地域后执行：

```bash
ros2 launch a2_bringup mock.launch.py \
  database_path:="$PWD/data/mock/mission.sqlite"
```

默认计划位于安装后的 `a2_bringup/config/mock-plan.json`。默认数据库也在启动目录的 `data/mock/`；显式路径便于重复演示与恢复。保留数据库后，已用过的任务 ID 不能再次作为新任务使用；可以换任务 ID 或为一次独立演示指定新库，禁止拿现场库作演示。

在另一终端加载相同环境与 ROS domain 后查询：

```bash
ros2 service call /a2/mission/get_state a2_interfaces/srv/GetMissionState '{}'
ros2 topic echo /a2/mission/status
```

`topic echo` 用 Ctrl+C 结束。状态枚举：255 无任务、0 已准备、1 自检、2 定位、3 巡检、5 操作暂停、11 故障暂停、12 取消。所有状态均带 `simulated=true`、`motion_capable=false`。

需要模拟断流时，退出整组节点后重新启动并设置 `platform_drop_after_s:=10.0`。只有已经进入模拟执行的任务会因断流进入故障暂停；空闲和已准备状态可以继续等待，但不会把缺失状态判为健康。任一节点退出时 launch 关闭整组，不自动 respawn 或恢复任务。

## 3. 任务服务契约

`/a2/mission/command` 使用 `MissionCommand`，仅允许下列命令：

| command | 输入及行为 |
|---|---|
| `prepare` | 任务 ID 和加载的合成方案四个版本标识必须匹配；初始为 READY。 |
| `authorize` | 提供模拟操作员 ID 和显式 `local_confirmation`，准入凭据只在直接服务响应中返回。 |
| `start` | 消费准入凭据；自动模拟自检/定位，条件不足则故障暂停。 |
| `pause` / `resume` | 暂停清旧凭据；恢复要求新凭据与当前 mock 健康。 |
| `record_visit` | 只能记录方案内模拟点位，重复点位不重复计数。 |
| `inject_fault` / `clear_fault` | 显式模拟故障；清除要求本地确认及健康恢复，清除后仍暂停。 |
| `cancel` | 记录模拟停稳后取消，不自动返桩。 |

请求须包含 `schema_version=1`、唯一 `request_id`、`command` 和 `mission_id`；额外非默认且与命令无关的字段被拒。核心请求及收据在 SQLite 中去重，服务请求的身份与服务器当时的健康观察分离，因此重试不会因为健康变化而变成新命令。

响应的 `processed` 表示命令取得了核心处理结果；业务是否接受看 `receipt.accepted`。`receipt.state` 是原请求当时的状态；`current` 是另外读取的当前观察，不能与历史结果视为同一原子快照。响应 `authorization_id` 不出现在 `/a2/mission/status` 或 `/a2/mission/receipts`。

`/a2/mission/get_state` 返回 `available`（能读取控制器）、`has_mission`（有任务）、`storage_fault_latched`、`mock_platform_healthy` 和当前状态。无任务时 available=true、has_mission=false；数据库不可读时 available=false；读取成功不等于任务允许运行。存储失败的本地禁止优先于历史任务状态。

mock 健康来自同机器人 ID 的新鲜、显式模拟平台样本。重复样本不能延长有效期，旧 source session 不得覆盖已接管的新 session；监控计时使用 steady clock。它不代表传感器、真实定位、控制权或真实停稳。

## 4. 当前开发安排

mock 任务与持久恢复增量已完成本机验证。独立的建图试验已用 A2 原装前雷达和 IMU 的断网录包完成本机环线回放，得到地图及闭环候选。新增巡检载荷接入后移，媒体、气体和离线业务随后按阶段集成。M0 未完成，实机运动仍须满足对应准入条件。[建图准备与回放边界](mapping-readiness.md) · [实现进度](implementation-status.md)

mock journal 与普通 core journal 的任务执行模式隔离，已有任务不能由另一模式接管。这里的本地确认是模拟操作输入，未实现实际操作者认证。完整 replay、robot_readonly、robot_motion 启动器及安全控制模块仍待实现；当前 launch 拒绝任何非 mock profile。
