# A2 地下管廊离线巡检

面向 A2-PRO / Jetson Orin 的定位导航与巡检项目。运行期间无外部网络，由机载系统完成固定路线巡检、载荷采集、故障处置和返桩对准，恢复联网后同步成果。

**当前阶段：准备实机采集工具与本机建图环境。** 用户已确认原装雷达、IMU 和摄像头可用，尚未安装的是新增巡检载荷。目前无法连接 A2，也没有实机录包，原装数据的话题、时钟和标定仍待核查。本机 Jazzy/x86_64 已完成固定版本 DLIO 候选构建及 DLIO/RTAB-Map 无输入启动检查；数据适配、完整建图和 A2/Humble 验证仍待。任务/存储核心、公共 ROS 接口、平台模拟状态和 mock 启动组合已实现；五个包为 in_progress，其他七个为 planned，模拟检查不作为实机验收证据。

## 文档入口

| 文档 | 用途 |
|---|---|
| [设计方案 v0.5](docs/design/A2-PRO_Utility_Tunnel_Navigation_v0.5.md) | 功能边界、离线运行、控制状态、验收及工程实施决策 |
| [软件架构](docs/architecture.md) | 分层、包职责、接口、进程和项目目录 |
| [本机开发与 A2 验证](docs/development.md) | 环境、模拟/回放、ARM64 构建、现场验证和证据回传 |
| [GitHub 建库准备](docs/github-setup.md) | 建库配置、初始提交、CI、数据边界和发布安排 |
| [连接 A2 操作流程](docs/a2-connection.md) | GitHub 固定版本、ZIP/在线克隆/离线 bundle、SSH、首次采集与数据回传 |
| [实机录包与建图验证](docs/mapping-capture.md) | 核查原装传感器、采集真实数据并带回本机分析 |
| [建图准备状态](docs/mapping-readiness.md) | 本机候选构建、无输入启动检查、适配要求及待验证项 |
| [模拟任务演示](docs/mock-mission.md) | 一键启动 mock，任务服务、故障与持久恢复 |
| [ROS 只读演示](docs/platform-readonly.md) | 构建节点、观察状态/诊断、查询与模拟断流 |
| [依赖管理](third_party/README.md) | 候选第三方组件与版本锁定要求 |

本机为 x86_64 Ubuntu 24.04，已安装 ROS 2 Jazzy（需 source），具备 Python 3、colcon、CMake 和 C++ 编译器。Humble/Jammy 容器是与候选 A2 环境对齐的开发方案；最终发行版须经 M0 核实。当前未执行容器构建或 A2 实机测试。

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

- 自研模块放在 `src/`，包清单见 [package-plan.json](src/package-plan.json)；任务、存储、公共接口、平台与 bringup 包为 `in_progress`，其他包为 `planned`。
- 第三方源码放在忽略的 `third_party/src/`；[建图候选清单](third_party/mapping-candidate.repos)单独记录本机试验版本。正式[依赖清单](third_party/dependencies.repos)仍为空，待 M0 核实后登记，不能据候选构建宣称完整导航栈或 A2 已验收。
- `config/examples/` 与 `deploy/templates/` 只包含不具备运动授权的示例；实机配置位于忽略的 `config/local/`、`deploy/local/`。
- 默认模式为 `mock`；`replay` 不加载运动 SDK。`robot_readonly` 与 `robot_motion` 分开，后者必须有实机验证记录和当前本地授权。
- 现场地图、标定、录包、照片、气体原始记录、SQLite 业务数据库和密钥保存在受控数据目录，不进入源码仓库。
- 机载运行不依赖 GitHub、CI、开发机、外部 NTP 或 Web 后端在线；维护期联网与运行期离线分别验证。

## 下一步实现顺序

A2 当前无法连接，先继续完成本机候选依赖、输入适配核查和采集工具准备，记录在[建图准备状态](docs/mapping-readiness.md)。DLIO 固定版本已在本机 Jazzy/x86_64 构建成功，使用时需显式加载候选 overlay；实机数据与完整建图尚未验证。可连接 A2 后，按[录包指南](docs/mapping-capture.md)核查系统、ROS、原装传感器话题、时钟和外参，完成短包与路线采集，再验证 DLIO 与 RTAB-Map。新增巡检载荷接入后移；M0 仍未关闭，实际控制继续遵守正式准入条件。

源码仓库：[liuzhaoming5954/a2-tunnel-navigation](https://github.com/liuzhaoming5954/a2-tunnel-navigation)，可见性为 **public**，默认分支 `main`。尚未选定开源许可证。日常在本机开发并提交，A2 在维护阶段获取固定提交，运行时不访问 GitHub；现场数据、凭据和平台构建产物不进入源码仓库。
