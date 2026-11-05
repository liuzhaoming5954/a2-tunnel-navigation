# 原装传感器接入与建图准备

维护连接、软件验证和原装传感器接入核查已开展，现场环境、数据与时间核查结果保存在私有记录中。公开部分保留候选依赖、无输入启动与适配要求；当前没有真实录包或完整建图，M0 未关闭，新增巡检载荷接入后移。官方规则见[原装传感器接口](a2-sensor-interfaces.md)，采集步骤见[采集指南](mapping-capture.md)。

## 1. 已完成和仍待验证

| 项目 | 本轮结果 | 边界 |
|---|---|---|
| DLIO ROS 2 | 官方提交 `c8acc37100e349d70a9d8432d656cbce7e5072cd`，版本声明 1.1.1；Jazzy/x86_64 构建成功，发现 `dlio_odom_node`、`dlio_map_node` | 未修改上游，无子模块；不等于 Humble/ARM64 或 A2 已验证 |
| 本机依赖 | PCL 1.14.0、Eigen 3.4.0，利用现有依赖构建；未新增系统安装 | 实际依赖仍须按 A2 环境重新登记和离线准备 |
| RTAB-Map | 系统安装 `rtabmap_slam` / `rtabmap_util` 0.23.13；有 `rtabmap` 可执行入口 | 外部 DLIO 供里程计时可以直接启动节点，首条无界面链路不必依赖 `rtabmap_launch` / `rtabmap_odom` |
| 进程启动 | 在本机回环网络、独立 ROS domain 79 启动 DLIO odom 和 RTAB-Map，确认进程存活、输入订阅和 `use_sim_time=true` | 没有输入包和有效算法输出；启动时 TF 隔离，不加载 SDK |
| 采集工具 | 21 项测试与合成 ROS 录包检查通过；可制作独立离线源码工具包 | 不含 ROS/驱动/标定，不是机器人镜像或完整离线依赖包 |
| 维护与地图 | 已开展维护连接和软件验证，现场结果保存在私有记录；继续准备原始短包、时间/标定核查及隔离回放 | 当前没有真实录包或完整建图；不能以编译、接口连通或合成录包关闭 M0/M1 |

DLIO 源码保存在忽略的 `third_party/src/direct_lidar_inertial_odometry/`。本轮构建目录为 `/tmp/a2-dlio-candidate-build`，安装 overlay 为 `/tmp/a2-dlio-candidate-install`，日志为 `/tmp/a2-dlio-candidate-log`；这些是本机临时产物，可能被清理，不复制给 ARM64。系统 `/opt/ros/jazzy` underlay 本身未新增 DLIO，需要显式加载候选 overlay。

## 2. 固定版本和重建

[候选清单](../third_party/mapping-candidate.repos)只固定本机评估用的 DLIO。正式 [dependencies.repos](../third_party/dependencies.repos) 仍为空，待 A2 实际发行版、驱动和接口核实后再锁正式组合。当前未制定完整 RTAB-Map 源码/二进制依赖锁，也未准备离线 ARM64 安装材料。

首次在干净目标目录导入源码（需要网络，运行期不执行）：

```bash
mkdir -p third_party/src
vcs import third_party/src < third_party/mapping-candidate.repos
git -C third_party/src/direct_lidar_inertial_odometry rev-parse HEAD
git -C third_party/src/direct_lidar_inertial_odometry status --short
```

确认提交一致且工作树无修改。本轮只通过稀疏 checkout 获取构建所需目录，完整导入也可构建。然后在未加载旧候选 overlay 的新终端运行：

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /tmp/a2-dlio-candidate-log build \
  --base-paths third_party/src/direct_lidar_inertial_odometry \
  --packages-select direct_lidar_inertial_odometry \
  --build-base /tmp/a2-dlio-candidate-build \
  --install-base /tmp/a2-dlio-candidate-install \
  --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_CXX_FLAGS=-Wno-deprecated-declarations
```

这是本机候选构建命令，不是 A2 安装命令。上游 CMake 强制 Release，测试关闭的构建不能记作算法测试通过。本轮有 PCL/FLANN 的 CMake CMP0144 开发警告，未阻止编译；没有绕过编译错误或修改上游。

构建后在另一个终端检查入口：

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/a2-dlio-candidate-install/local_setup.bash
ros2 pkg executables direct_lidar_inertial_odometry
ros2 pkg executables rtabmap_slam
```

## 3. 源码发现的接入待项

下列事实来自固定 [DLIO 提交](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/tree/c8acc37100e349d70a9d8432d656cbce7e5072cd)，须在开发适配和首包回放中逐项解决；目前未提供一键完整建图启动器。

| 项目 | 固定版本的实际行为 | 接入工作 |
|---|---|---|
| 点云订阅 QoS | `create_subscription(..., 1, ...)` 为默认 reliable；IMU 使用 SensorDataQoS | 对照实机发布端 QoS。发布 best effort 时显式适配订阅或受控回放 QoS，避免有话题却无回调 |
| 每点时间 | `t` 用相对纳秒，`time` 用相对秒；Hesai `timestamp` 按绝对秒解析，PCL 对应 double 字段；不能任意重命名 | 核实 JT128 真实布局、单位、参考时刻与时钟。保留原始字段；适配器才做有证据的转换 |
| 去畸变状态 | 时间字段 UNKNOWN 时关闭 deskew；时间同步失败也可能产生退化输出 | 必须有明确成功/失败证据，不能仅以收到 `deskewed` 话题放行 |
| 单帧点云坐标 | 输出是当前单帧经修正变换后的 `odom` 坐标云，非累计地图；header 仍为原始 scan header | 输入 RTAB-Map 前统一扫描坐标和参考时刻，避免把已经在 odom 的点再次当传感器坐标变换 |
| 参考时刻 | 内部 `scan_stamp` 取每点时间中位数，而云 header 保持扫描 header；odom 独立每 10 ms wall timer 发布，stamp 是最新 IMU 时间 | 确认算法内部参考与发布语义，必要时显式适配或维护小补丁；不能用近似同步掩盖时刻定义错误 |
| TF | 没有 `publish_tf` 开关，直接广播 `odom→base_link` 及两个传感器外参（后者也在动态 TF） | 先将上游 `/tf` remap 到私有输入，后续由适配器和标定发布器按设计唯一发布；单独设置不存在的参数无效 |
| 样例配置 | `params.yaml` 启用 `use_sim_time`、`computeTimeOffset`、`waitUntilMove`，另有样例外参和裁剪值 | 回放统一仿真时间；静止输入需明确设置 `waitUntilMove=false`。点时间偏移和外参根据真实数据决定，不能照搬样例 |
| 空扫描 | NaN/裁剪过滤后识别 `timestamp` 类型会访问首点，早于后续最少点数检查 | 在接入前增加输入/空扫描保护和回归验证，避免崩溃；当前未提交上游补丁 |

对应源码：[订阅、参数及输出实现](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/blob/c8acc37100e349d70a9d8432d656cbce7e5072cd/src/dlio/odom.cc)、[PCL 字段类型](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/blob/c8acc37100e349d70a9d8432d656cbce7e5072cd/include/dlio/dlio.h)、[官方 launch](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/blob/c8acc37100e349d70a9d8432d656cbce7e5072cd/launch/dlio.launch.py)、[样例参数](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/blob/c8acc37100e349d70a9d8432d656cbce7e5072cd/cfg/params.yaml)。

## 4. RTAB-Map 组合边界

本机 0.23.13 的首条外部里程计接入拟使用 `rtabmap_slam/rtabmap`，显式配置 `subscribe_scan_cloud=true`、`subscribe_depth=false`、`subscribe_rgb=false`、`subscribe_odom_info=false`，只连接已适配的单帧点云和里程计。DLIO 没有 RTAB-Map `OdomInfo`。摄像头与地标在校验内参、时间、消息与外参后加入。[官方 3D LiDAR 示例](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_examples/launch/lidar3d.launch.py)

0.23.13 的 `odom_frame_id` 非空时会关闭 odom 消息订阅，改从 TF 取里程计；使用 odom 话题时该参数应留空，并设置 remap。纯扫描链默认精确同步，原始 DLIO 的 odom 与扫描 stamp 不同，不能直接假定精确同步会有输出。先统一参考时刻，再选择相同时间输出或有界近似同步/TF 插值方案，记录误差和超时预算。[订阅实现](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_sync/src/CommonDataSubscriber.cpp)

RTAB-Map 只发布 `map→odom`，位姿适配器负责机身动态 TF，标定发布器负责固定外参。上游默认 TF 不能与这些发布者叠加。DLIO 已去畸变的云也不能再次交给 `lidar_deskewing` 重复处理。

## 5. 可带到现场的采集包

在项目根目录生成独立 ZIP（目标文件必须不存在）：

```bash
mkdir -p artifacts
python3 scripts/package_capture_tools.py --output artifacts/a2-capture-kit.zip
```

归档固定包含两份采集/检查 Python 脚本、空话题配置示例、独立操作说明、[连接手册](a2-connection.md)和 `SHA256SUMS`；不递归打包工作区，不包含源码仓库凭据、现场配置、录包或 x86_64 二进制。解压后进入 `a2-capture-kit`，执行 `sha256sum -c SHA256SUMS` 核对，再加载目标机实际 ROS/驱动环境。它不能替代 ROS 和厂商驱动的离线依赖准备。

本轮产物位于忽略的 `artifacts/a2-capture-kit.zip`，已核对归档白名单、ZIP CRC 和逐文件 SHA-256。脚本更新后用新文件名重新生成，不复用旧包冒充新版本。

现场最新材料按连接手册从公共仓库的干净固定提交生成，并配套 `SOURCE_COMMIT.txt` 与外层校验清单；保留旧包作为历史产物，不以旧包替代新版手册对应材料。A2 维护时可联网则可直接克隆相同 SHA，需完整历史且无外网时可传离线 bundle，三条路径使用相同的采集与回传目录。

继续按[官方接口](a2-sensor-interfaces.md)与采集指南核查目标数据、字段、时序和外参，准备原始短包与环境记录；完成适配和候选问题修正后，再隔离运行 DLIO→RTAB-Map 并保存、重载与评估地图。现场结果保持私有；尚无真实录包或完整建图，M0 未关闭。
