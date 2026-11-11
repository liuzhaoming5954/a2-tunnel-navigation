# 本机开发与 A2 实机验证

本文规定开发、交付和验证流程；系统行为以当前设计方案和 [软件架构](architecture.md) 为准。**代码在本机开发和评审，A2 负责 ARM64 构建兼容性、设备接入及真实运动验证。运行期间无外部网络；传输和安装只在出发前或维护窗口进行。**

首次现场连接按[连接 A2 操作流程](a2-connection.md)执行：先从公共 GitHub 仓库确认固定提交，再完成维护网络、SSH、目标主机核查，选择 ZIP / A2 在线克隆 / 离线 bundle 获取工具，最后将源码版本记录与首包一并回传；实际地址、账号和 ROS 环境现场填写。

## 1. 环境与当前完成状态

| 环境 | 已知情况 | 用途与约束 |
|---|---|---|
| 开发宿主基线 | x86_64、Ubuntu 24.04、Python 3.12；准备 Git、colcon、CMake、g++ 与 ROS 2 Jazzy | 编辑、仓库检查、纯逻辑测试；Jazzy 可用于独立试验，其结果不代替 Humble 兼容验证。 |
| 本机开发容器 | 候选 Ubuntu 22.04 + ROS 2 Humble，定义见 `docker/development/Dockerfile` | 提供与候选目标一致的 ROS/Python 依赖；本次未构建或运行该镜像。 |
| A2 目标环境 | 设计目标为 Orin / ARM64；实际型号、系统、驱动和 SDK 组合在私有实机清单中登记 | 验证构建兼容、驱动、时序、功耗、散热和故障停车；维护连接和软件验证已开展，现场结果保存在私有记录中。 |
| GitHub CI | 仓库契约、Python 核心与无 SDK 的 Jazzy 只读构建/通信检查 | 不接机器人，不具备实机放行权；后续有实际 ROS 包再增加编译和测试任务。 |

Humble 官方二进制目标包含 Ubuntu 22.04 的 amd64/arm64。本机宿主是 24.04，故候选方案使用容器，不直接混装 Jammy 软件源。若 M0 确认 A2 采用其他受支持组合，则同步调整本机镜像、CI 和依赖锁定；不能把本机 Jazzy 构建成功作为 Humble 已验证。[ROS 官方安装文档源码](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)

任务、存储、公共接口、平台和 bringup 已有五个可构建 ROS 2 包。导航目录新增了独立的离线规划工具，实机执行包待实现。正式依赖待 M0 核实后登记，本机试验依赖单独记录。[实现进度](implementation-status.md) · [建图候选清单](../third_party/mapping-candidate.repos)

本机已完成官方 DLIO `feature/ros2` 固定提交的 Jazzy/x86_64 基线构建。真实数据回放使用单独构建的试验 overlay，启动时记录源码补丁、参数、回放方式和输出位置。A2 的正式建图依赖与 ARM64 构建待验证。[候选版本与重建](mapping-readiness.md) · [最新回放结果](implementation-status.md)

建图验证使用本机回环网络和独立 ROS 域。从无输入启动检查推进到实机录包回放，已验证前雷达与配套 IMU 的点时间、同步点云和位姿、整圈建图与候选闭环。地图尺度、重定位和机载负载按对应阶段继续核验。[开发进度说明](implementation-status.md)

## 2. 每次变更的开发闭环

新 A2 的前雷达和配套 IMU 已完成断网整圈录包与本机闭环复测，现场原始数据保存在受控目录。地图几何、尺度与重定位进入下一轮核验。M0 实机控制权与停车待验证，新增巡检载荷按安装进度接入。[录包指南](mapping-capture.md) · [首次受控运动与停车验证](a2-first-motion-test.md)

1. 从 `main` 建立短期功能分支，明确修改影响的模块、接口、配置和验收用例。
2. 在本机完成实现与对应测试，运行仓库检查；使用 mock 或固定数据回放复现故障。控制策略测试覆盖真实状态转换、超时、取消和恢复，不只检查消息能够发布。
3. 提交变更，记录完整提交 SHA；通过 PR 审阅和适用的 CI。未通过的项单独列出，不能标记为“待上机自然解决”。
4. 生成候选发布包，固定源码 SHA、依赖、接口、地图/配置版本和目标平台；部署到 A2 的独立候选目录。
5. 在 A2 依次通过 ARM64 构建、静态检查、只读接入、受控动作和对应 M0–M4 验证。测试结果绑定候选版本。
6. 将日志与证据带回本机分析；修复继续走上述流程。通过的候选版本才可登记为该机器人已验收版本。

现场紧急诊断可在 A2 的独立诊断副本修改；修改须导出为补丁、回到本机评审并重新生成版本。不得在当前运行目录内持续手改而丢失版本对应关系。

## 3. 本机工作方式

在仓库根目录可先执行不依赖 ROS 的检查：

```bash
python3 scripts/check_repository.py
```

完成 Docker 安装并确认 M0 采用候选 Humble 基线后，可在本机准备开发镜像：

```bash
docker build -f docker/development/Dockerfile \
  --build-arg DEV_UID="$(id -u)" --build-arg DEV_GID="$(id -g)" \
  -t a2-dev:humble .
docker run --rm -it --network none \
  --user "$(id -u):$(id -g)" \
  --mount "type=bind,src=$PWD,dst=/work/a2_project" \
  --workdir /work/a2_project \
  -e ROS_DOMAIN_ID=42 -e ROS_LOCALHOST_ONLY=1 \
  a2-dev:humble bash
```

镜像构建和依赖下载允许在开发准备阶段联网；mock/replay 执行使用无外部网络的容器，不使用 `--network host`、设备映射或真实 SDK 凭据。示例域号只供本机隔离，最终值在开发配置中登记；域号及 localhost 限制是辅助措施，不能替代“未加载硬件桥接”和网络隔离。[ROS 环境变量说明](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.rst)

实际 ROS 包建立后，在容器的新终端内先加载 underlay，检查当前 profile 的明确包清单再构建。下面构建当前五个已存在的包，平台 SDK 默认关闭；后续按 profile 的依赖关系扩充白名单，不能直接全量构建并加载真实 SDK 适配器。

```bash
source /opt/ros/humble/setup.bash
colcon list --base-paths src --packages-select a2_interfaces a2_platform a2_storage a2_mission a2_bringup
# 确认实际发现的 ROS 2 包，离线规划工具独立运行。
colcon build --base-paths src --packages-select a2_interfaces a2_platform a2_storage a2_mission a2_bringup --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --base-paths src --packages-select a2_interfaces a2_platform a2_storage a2_mission a2_bringup --event-handlers console_direct+
colcon test-result --verbose
```

构建后另开终端加载 `/opt/ros/humble/setup.bash` 和 `install/local_setup.bash` 再启动测试进程；不要在已经加载旧 overlay 的终端继续构建。`build/`、`install/`、`log/` 仅为本平台生成物，不提交或复制到另一架构。开发使用 `--symlink-install`，发布采用非链接安装并检查产物不依赖开发源码路径。[colcon 官方教程](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.rst)

纯逻辑测试、ROS 集成测试和回放测试各自记录用例数及结果；空测试集不得代表功能已验证。本机 Python 3.12 只用于仓库工具，业务 Python 的兼容性由目标容器和 A2 环境验证。

## 4. mock、回放和硬件隔离

| 模式 | 输入与输出 | 运行边界 |
|---|---|---|
| `mock` | 合成传感器、模拟底盘/云台/气体结果，输出到模拟端 | 不加载 Unitree SDK，不打开真实设备或机器人网络。用于状态机、超时、接口和持久化测试。 |
| `replay` | 录包中的传感器白名单与固定配置，输出到隔离结果目录 | 不启动任何硬件发送端，不复用现场数据库/授权。用于定位、地标、点云和数据关联回归。 |
| `robot_readonly` | A2 现场实时设备状态，仅观察和录制 | 禁止实例化运动客户端或发放运动授权；是否存在控制副作用需由适配实现验证。 |
| `robot_motion` | 已登记候选版本、实测参数和一次性现场授权 | 只在通过前置检查的受限场地运行；不由普通 CI 或远程代码推送自动启动。 |

模式名是实现必须满足的启动配置契约；尚无相应 launch 文件或验证证据时，不将其列为可执行能力。配置缺失、模式未知或真实适配器混入 mock/replay 时，启动应失败。

首次录包可在原厂传感器链路上由只读工具采集，不以自研 `robot_motion` profile 已实现为前提。现场移动由人员使用原厂操作方式完成，并落实对应的停车与急停条件；采集脚本不发送运动命令。先确认短包的点云、IMU 和 TF 完整，再继续路线采集，具体操作见[录包指南](mapping-capture.md)。原装相机可同时采集图像与标定；仅验证 LiDAR–IMU 建图时，不因新增巡检相机或气体传感器缺席而等待。

回放先使用 `ros2 bag info` 审查元数据，形成明确的话题白名单。只播放本轮需要的点云、IMU、图像、标定和只读状态；**不得回放历史 `cmd_vel`、动作请求、控制权、租约、启动/继续/返桩命令到实机链路**。保留原始采集数据用于追溯，测试时是否播放 `/tf`、`/tf_static` 或由节点重算须明确，防止重复 TF。每次回放使用新输出目录和测试任务 ID。[rosbag 官方教程](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.rst)

回放的数据配准使用相应 ROS 仿真时间；运动有效期与故障超时仍按设计使用单调时间。需要暂停/倍速验证超时时，用可注入的测试时钟验证纯逻辑；不得把回放墙钟结果直接当成 A2 真实停车时延。

## 5. 候选版本与离线传输

候选发布清单至少包含：`release_id`、完整源码 SHA、源码是否有未提交修改、目标 `arch/os/ROS/JetPack`、依赖锁定摘要、构建日志、文件 SHA-256、`schema_version`、`bundle_id`、`map_version`、`config_version`、适用机器人、回退版本和已通过用例。正式候选要求源码工作区干净；未经记录的本地修改不能混入同一 SHA 的发布物。

| 交付路径 | 具体方式 |
|---|---|
| 首期推荐 | 本机生成带校验清单的源码与依赖材料包，在 A2 原生 ARM64 环境编译、测试和安装。源码应来自固定 Git 提交，而非复制含私密数据的整个工作目录。 |
| 后续可选 | 由已验证 ARM64 构建机/流水线生成运行包或镜像，在 A2 验证后使用。产物须匹配目标系统、JetPack/驱动与 SDK ABI。 |
| 不作为发布捷径 | 将本机 x86_64 `install/` 或 amd64 镜像直接复制到 A2。跨架构模拟构建只用于兼容性辅助，不能替代 ARM64 性能和设备验证。 |

容器共享宿主内核，镜像仍有架构约束；多架构构建可采用原生节点、模拟或交叉编译。当前方案优先原生 ARM64 构建，尤其核对 Jetson 视频/GPU 和原厂库，而非假定通用 ROS 镜像能够直接访问这些设备。[Docker 多平台构建说明](https://docs.docker.com/build/building/multi-platform/)

依赖准备在有网络的维护阶段完成：记录 apt 包版本及 ARM64 架构，准备所需 `.deb` 及其传递依赖、固定源码依赖、Python wheel 和配置材料；有容器部署时准备目标架构镜像及摘要。`rosdep` 用于准备阶段解析依赖，不能要求机器人进入管廊后再访问软件源。断开外网进行一次安装/冷启动复验，证明依赖齐全。

传输通过经过确认的维护 SSH/有线连接或受控可移动介质；A2 地址、用户、密钥保存在本机私有配置，不能写入仓库。复制到新的暂存目录，校验摘要后才能安装候选；不使用同步命令直接覆盖活动目录，也不执行远程“一键上传并运动”。维护连接和软件验证已开展，现场版本、环境和验证结果保存在私有记录中，不据此自动启动运动。

建议运行目录布局（具体部署根目录在 M0 固定）：

```text
/opt/a2/releases/<release_id>/     # 不可变程序与清单
/opt/a2/current                   # 已验收版本指针
/var/lib/a2/                      # 持久任务、授权、数据库、地图包和媒体
/var/log/a2/                      # 运行日志与诊断
```

候选验证结束后，确认机器人停稳、无自研运动授权、原厂控制权明确，才切换版本指针并按静止启动流程重新自检。软件回滚不能回滚 SQLite、已消费授权、任务记录或同步水位；数据库 schema 不兼容时拒绝简单回滚，并按已验证的数据迁移方案处理。启动服务不自动恢复运动。

## 6. A2 测试阶梯与证据

| 顺序 | 测试 | 进入下一步的依据 |
|---|---|---|
| A | 核对实机清单，ARM64 原生构建，依赖/动态库/配置检查 | 平台匹配、预期包确实构建、测试有实际用例、无缺失库；候选清单完整。 |
| B | 静止启动和 `robot_readonly` | 无运动授权；传感器/SDK 状态、TF、时钟、数据记录可解释；启动不存在隐式运动副作用。 |
| C | M0/M2 受限运动与停车 | 先验证接收端失联保障，再验证低速动作、取消、桥接退出、传感器/SDK 断链、控制权交接；结果满足本轮冻结边界。 |
| D | M1/M3 代表性路线与载荷 | 定位、固定路径、移动拍摄、气体关联、重启/写盘/预算策略通过对应测试。阶段编号按方案，允许回到录包分析，不跳过前置条件。 |
| E | M4 实际载荷全程离线 | 出发后关闭外部网络直至本地任务结束，验证巡检、返桩/对准、本地故障处置和记录；回到可联网位置后核对补传完整性。 |

每次实机测试记录：用例 ID、操作者、时间质量、机器人/固件、源码 SHA、发布/地图/配置版本、初始电量、载荷、地面和路线条件、预期行为、实测结果、故障注入点、停稳证据、数据缺口及结论。不能只截图软件状态而不记录真实底盘行为。

原始 rosbag、图像、气体数据和运行 SQLite 放在被忽略的 `data/` 或受控外部存储，不上传代码仓库。`tests/hil/` 保存可复现步骤及脱敏报告模板；可公开的小型合成回归样本才进入 `tests/fixtures/`。证据索引记录文件校验值和受控位置，仓库不保存登录凭据或私有现场路径。

## 7. CI 与实机放行的边界

当前 CI 检查仓库契约、任务/存储核心测试和纯合成演示，另有 `ros-readonly` 工作流构建无 SDK 的 Jazzy 接口/只读节点并执行测试。当前已在本机 Jazzy/x86_64 完成五个包的 colcon 构建与 mock ROS 通信检查；目标 Humble/ARM64 尚未验证。后续增加接口生成/兼容检查、匹配目标发行版的 ROS 编译、mock 集成与回放回归，再接入 ARM64 构建。每项输出实际包/用例数和版本。

GitHub 托管 runner 不可访问机器人。后续若使用 ARM64 自托管构建机，其职责仅是构建/测试，不能把 A2 运动底盘注册为收到 PR 就执行动作的 runner。A2 现场验证使用独立步骤、明确候选版本和现场授权，结果作为发布证据回填。
