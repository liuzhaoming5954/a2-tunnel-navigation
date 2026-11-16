# 项目工作约定

## 技能使用

项目技能安装在本机，以下路径相对于当前工作区。

- A2 项目开发、模块设计、接口集成和分阶段验证，使用 `a2-project-development`，相对路径为 `.agents/skills/a2-project-development/SKILL.md`。
- ROS 2、传感器、话题、QoS、时间戳、TF 和 RViz 排障，使用 `ros2-integration-diagnostics`，相对路径为 `.agents/skills/ros2-integration-diagnostics/SKILL.md`。
- 开展任务时，以当前代码、配置和日志为依据，将历史开发记录与当前验证结果分开。

## 开发和部署

- 修改前确认实际工作区和目标分支。代码在开发机编写和验证，部署时核对 A2 目标机的系统、CPU 架构、ROS 版本和 SDK 版本，使用固定版本的程序与配置。操作流程见 [本机开发与 A2 实机验证](docs/development.md)。
- 机载任务按无外部网络运行设计并验证，使用机载程序、地图和配置。GitHub、开发机和维护网络用于维护阶段的代码传递与诊断。
- 公开仓库保存可公开的源码、通用文档和合成夹具。现场地图、录包、标定、实机配置及凭据保存在受控目录，外发诊断材料按用途脱敏。见 [仓库约定](docs/github-setup.md)。

## 实机验证

- 静止检查和录包工具仅接入传感器、状态和记录接口。人工采集由现场人员用原厂遥控操作。自主运动按本轮授权和 [实机控制与停车验证](docs/a2-first-motion-test.md) 执行。
- ODIN1 集成使用其设备端算法。建图使用 `LIDAR_MODE_SLAM` 下的 `map_mode=1`，重定位加载 `.bin` 地图并使用 `map_mode=2`。PC2 调用留形 SDK、接收数据并保存地图文件。
- 设备联调按对应版本的厂商流程核对调用返回值，再用设备发送频率、主机回调及实际数据验证结果。建图、重定位和运动验证的日期、环境、配置与结果记录在 [开发进度](docs/implementation-status.md)。

## 各模块代码入口

路径相对于当前工作区。ROS 2 命令在构建并加载工作区后运行。模块清单见 [src/package-plan.json](src/package-plan.json)。

| 模块 | 启动入口与主要源码 |
| --- | --- |
| `a2_interfaces` 公共接口 | [msg](src/a2_interfaces/msg/) 和 [srv](src/a2_interfaces/srv/) 定义消息和服务，[CMakeLists.txt](src/a2_interfaces/CMakeLists.txt) 注册接口。 |
| `a2_description` 机身模型与外参 | [模块说明](src/a2_description/README.md)，URDF/Xacro 和静态 TF 配置待实现。 |
| `a2_platform` 平台状态 | `ros2 run a2_platform readonly_node` → [readonly_node.cpp](src/a2_platform/src/readonly_node.cpp)，Unitree 状态适配见 [unitree_state_source.cpp](src/a2_platform/src/unitree_state_source.cpp)。 |
| `a2_localization` 定位 | [body_pose.py](src/a2_localization/a2_localization/body_pose.py) 换算机身位姿，[publish_a2_localization.py](scripts/publish_a2_localization.py) 接入 DLIO 只读定位。 |
| `a2_navigation` 导航 | [plan_navigation_goal.py](scripts/plan_navigation_goal.py) → [offline_planner.py](src/a2_navigation/a2_navigation/offline_planner.py) 做离线规划，[preview_navigation_goal.py](scripts/preview_navigation_goal.py) → [straight_preview.py](src/a2_navigation/a2_navigation/straight_preview.py) 做指令预演。 |
| `a2_safety` 安全 | [maintenance_arbiter.py](src/a2_safety/maintenance_arbiter.py) 的 `MaintenanceArbiter` 为原型，实机仲裁服务待集成。 |
| `a2_mission` 任务 | `ros2 run a2_mission mission_node` → [ros_node.py](src/a2_mission/a2_mission/ros_node.py) 启动 mock 节点，[controller.py](src/a2_mission/a2_mission/controller.py) 是任务控制器，[demo.py](src/a2_mission/a2_mission/demo.py) 是 `a2-mission-demo` 的入口。 |
| `a2_payloads` 巡检载荷 | [模块说明](src/a2_payloads/README.md)，云台和气体传感器接入待实现。 |
| `a2_storage` 存储 | [journal.py](src/a2_storage/a2_storage/journal.py) 的 `SQLiteJournal` 是 SQLite 持久化入口，由任务模块调用。 |
| `a2_gateway` 维护上位机 | [run_a2_console.py](scripts/run_a2_console.py) → [server.py](src/a2_gateway/a2_gateway/server.py)。Vue 前端从 [main.js](src/a2_gateway/web/src/main.js) 进入 [App.vue](src/a2_gateway/web/src/App.vue)。[start_a2_console_readonly.sh](scripts/start_a2_console_readonly.sh) 启动地图、平台状态和只读界面，详见[操作说明](docs/a2-operator-console.md)。 |
| `a2_bringup` 系统启动 | `ros2 launch a2_bringup mock.launch.py` → [mock.launch.py](src/a2_bringup/launch/mock.launch.py)。运行模式约定见 [profiles.json](config/examples/profiles.json)。 |
| `a2_test_support` 测试支持 | [模块说明](src/a2_test_support/README.md)。独立包待实现，现有任务和平台验证入口为 [mission_ros_smoke.py](scripts/mission_ros_smoke.py) 和 [readonly_smoke.py](scripts/readonly_smoke.py)。 |

- 录包和地图工具从 [a2_capture.py](scripts/a2_capture.py) 采集，[inspect_mapping_bag.py](scripts/inspect_mapping_bag.py) 检查录包，[export_navigation_map.py](scripts/export_navigation_map.py) 导出导航地图，[record_a2_anchor_evidence.py](scripts/record_a2_anchor_evidence.py) 记录定位匹配证据。
- 导入 ODIN1 厂商源码后，ROS 驱动入口为 `third_party/src/odin_ros_driver/src/host_sdk_sample.cpp`，配置为 `third_party/src/odin_ros_driver/config/control_command.yaml`。直接调用 SDK 时查阅同一源码目录的 `include/lidar_api.h`。第三方源码布局见 [third_party/README.md](third_party/README.md)。

## 本机检查

- 仓库检查入口为 `python3 scripts/check_repository.py`。代码和配置变更按受影响模块运行相应测试，记录实测环境及结果。
