# A2 地下管廊离线巡检

面向 A2-PRO / Jetson Orin 的定位导航与巡检项目。运行期间无外部网络，由机载系统完成固定路线巡检、载荷采集、故障处置和返桩对准，恢复联网后同步成果。

**当前已完成新 A2 的整圈断网录包、离线建图和闭环候选地图。** 同一录包的有序回放已可复现。地图尺度、重定位、实机控制权和停车仍待验收，M0 未关闭。当前状态见[开发进度说明](docs/implementation-status.md)。

## 文档入口

| 文档 | 用途 |
|---|---|
| [项目说明](docs/project-overview.md) | 项目目标、离线运行、软件架构、开发流程和目录 |
| [开发进度说明](docs/implementation-status.md) | 阶段状态、实机录包、建图和闭环结果、待验收事项 |
| [设计方案 v0.5](docs/design/A2-PRO_Utility_Tunnel_Navigation_v0.5.md) | 功能边界、离线运行、控制状态、验收及工程实施决策 |
| [软件架构](docs/architecture.md) | 分层、包职责、接口、进程和项目目录 |
| [本机开发与 A2 验证](docs/development.md) | 环境、模拟/回放、ARM64 构建、现场验证和证据回传 |
| [GitHub 建库准备](docs/github-setup.md) | 建库配置、初始提交、CI、数据边界和发布安排 |
| [连接 A2 操作流程](docs/a2-connection.md) | GitHub 固定版本、ZIP/在线克隆/离线 bundle、SSH、首次采集与数据回传 |
| [原装传感器接口](docs/a2-sensor-interfaces.md) | 官方 PC2 服务、DDS/ROS 通信及雷达、IMU、相机接入规则 |
| [实机录包与建图验证](docs/mapping-capture.md) | 核查原装传感器、采集真实数据并带回本机分析 |
| [首次受控运动与停车验证](docs/a2-first-motion-test.md) | 核对实机控制权、独立停车、接收端失联停车及实测记录 |
| [A2 地图目标点验证](docs/a2-goal-validation.md) | 导出离线栅格、规划地图目标、检查定位输入和实机试验前置条件 |
| [建图准备状态](docs/mapping-readiness.md) | 本机候选构建、无输入启动检查、适配要求及待验证项 |
| [模拟任务演示](docs/mock-mission.md) | 一键启动 mock，任务服务、故障与持久恢复 |
| [ROS 只读演示](docs/platform-readonly.md) | 构建节点、观察状态/诊断、查询与模拟断流 |
| [依赖管理](third_party/README.md) | 候选第三方组件与版本锁定要求 |

开发环境以 x86_64 Ubuntu 24.04 / ROS 2 Jazzy 和 Python 3、colcon、CMake、C++ 工具链为基线；Humble/Jammy 容器用于候选目标兼容验证。目标机器人采用的实际环境和验证结果登记在私有记录中，公开候选基线不替代 M0 依赖锁定或验收。

## 当前可执行的检查与演示

在项目根目录运行：

```bash
python3 scripts/check_repository.py
python3 -m unittest discover -s scripts/test -v
PYTHONPATH=src/a2_storage python3 -m unittest discover -s src/a2_storage/test -v
PYTHONPATH=src/a2_storage:src/a2_mission python3 -m unittest discover -s src/a2_mission/test -v
PYTHONPATH=src/a2_storage:src/a2_mission python3 -m a2_mission.demo
```

这些命令检查仓库、采集工具和任务/存储规则并运行纯合成演示；均不连接机器人。采集工具的实际 rosbag 测试需加载 ROS，未加载时明确跳过该项。测试范围与限制见[实现进度](docs/implementation-status.md)。安装并配置容器引擎后，按开发文档建立匹配目标发行版的环境。

## 项目约定

- 自研模块放在 `src/`。任务、存储、公共接口、平台、bringup 和离线导航工具已开始实现。完整进度见[包清单](src/package-plan.json)。
- 第三方源码放在忽略的 `third_party/src/`；[建图候选清单](third_party/mapping-candidate.repos)单独记录本机试验版本。正式[依赖清单](third_party/dependencies.repos)仍为空，待 M0 核实后登记，不能据候选构建宣称完整导航栈或 A2 已验收。
- `config/examples/` 与 `deploy/templates/` 只包含不具备运动授权的示例；实机配置位于忽略的 `config/local/`、`deploy/local/`。
- 默认模式为 `mock`；`replay` 不加载运动 SDK。`robot_readonly` 与 `robot_motion` 分开，后者必须有实机验证记录和当前本地授权。
- 现场地图、标定、录包、照片、气体原始记录、SQLite 业务数据库和密钥保存在受控数据目录，不进入源码仓库。
- 机载运行不依赖 GitHub、CI、开发机、外部 NTP 或 Web 后端在线；维护期联网与运行期离线分别验证。

## 下一步实现顺序

使用闭环候选地图复测重定位并核验地图尺度和几何。实机控制准备从目标环境、原厂控制权、独立停车和接收端失联停稳开始。新增巡检载荷按安装进度接入。[开发进度说明](docs/implementation-status.md) · [停车验证](docs/a2-first-motion-test.md)

源码仓库：[liuzhaoming5954/a2-tunnel-navigation](https://github.com/liuzhaoming5954/a2-tunnel-navigation)，可见性为 **public**，默认分支 `main`。尚未选定开源许可证。日常在本机开发并提交，A2 在维护阶段获取固定提交，运行时不访问 GitHub；现场数据、凭据和平台构建产物不进入源码仓库。
