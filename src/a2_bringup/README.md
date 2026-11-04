# a2_bringup

状态：`in_progress`。当前实现 `mock.launch.py`，组合模拟平台状态源和 mock-only 任务 ROS 节点。用户已确认原装雷达、IMU 和摄像头可用，新增巡检载荷尚未安装；当前优先[实机录包与建图验证](../../docs/mapping-capture.md)，本启动组合继续用于软件演示与回归。

- 固定启动 `a2_platform/readonly_node` 的 `source=mock` 和 `a2_mission/mission_node` 的 `profile=mock`，使用同一机器人标识。
- 本轮只接受 `profile=mock`；`replay`、`robot_readonly`、`robot_motion` 均拒绝，不加载真实 SDK 或运动客户端。
- 任务方案为本包的 `config/mock-plan.json`，仅包含合成版本与两个设备点 ID，不是实机地图或导航路径。
- 任一节点退出即关闭整组，不无限 respawn；重新启动时数据库由任务核心恢复，旧运动/准入凭据不能恢复。

加载工作区后，在项目根目录运行：

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
ros2 launch a2_bringup mock.launch.py
```

默认数据库位于启动目录下的 `data/mock/mission.sqlite`，属于被 Git 忽略的运行数据。该路径保持持久化，供任务节点重启恢复记录；需要独立演示时显式传入新的绝对路径：

```bash
ros2 launch a2_bringup mock.launch.py \
  database_path:=/tmp/a2-demo-session/mission.sqlite
```

参数：`robot_id` 默认 `mock-a2`；`mock_plan_path` 默认已安装的本包方案；`platform_drop_after_s=-1.0` 表示持续模拟状态，设置非负秒数可演示平台断流。演示参数不是实机健康或停车时限。ROS 本地域是辅助隔离，真实硬件 profile 始终不在此启动组合中。

任务推进、自检、定位与停稳均为模拟输入，不能作为真实导航、传感器、停车或 A2 放行证据。完整四种 profile、生产进程监督和真实设备准入继续按[架构约定](../../docs/architecture.md)与[开发流程](../../docs/development.md)实现。

纯配置测试不启动 ROS：

```bash
PYTHONPATH=src/a2_bringup python3 -m unittest discover -s src/a2_bringup/test -v
```
