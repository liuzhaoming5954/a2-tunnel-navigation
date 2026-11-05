# A2 原装传感器接口与通信核对

2026-11-05 核对。本文整理宇树官方 A2 开发文档中的接口约定，供原装雷达、IMU、相机接入使用。**官方约定与本轮实机观测分别记录**：地址、服务版本、网卡、话题及消息内容仍需在目标机器人核实，接口文档不能替代接入或建图验收。

维护连接和代码传递见[连接流程](a2-connection.md)，录包要求见[采集与建图验证](mapping-capture.md)。机器人运行时无需外网；内部网络、DDS 和传感器数据链仍需正常工作。

## 1. 原厂入口与网络边界

| 项目 | 官方文档约定 | 接入时核查 |
|---|---|---|
| 原厂感知与 SLAM 计算机 | PC2，内部地址 `192.168.123.162` | 主机身份、服务实际部署位置、维护入口 |
| 运控计算机 | PC1，内部地址 `192.168.123.161` | 仅核查通信与时钟来源，不发送控制命令 |
| 前 / 后 JT128 雷达 | `192.168.124.20` / `192.168.124.21` | 从原厂感知计算机核查链路与实际配置 |
| 原厂雷达 / SLAM 服务入口 | `unitree_slam.service` | 只读查看状态、版本、启动文件与日志 |
| 雷达 DDS 订阅示例 | 域 ID 为 `0`，绑定内部 123 网段的网卡 | 使用实机对应网卡，不照抄示例中的网卡名 |

以上来自[雷达服务接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/lidar_service_interface)与[SLAM 和导航服务接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/slam_and_navigation_service_interface)。官方说明相关雷达接口要求 SLAM NAV 服务达到 1.0.0 或以上；先核实已装版本，版本不满足时记录差异并确认维护方案，不自动 OTA。

维护 Wi-Fi、内部 SDK/DDS 网络和雷达数据网络用途不同。能登录某台开发计算机，不表示该机就是 PC2，也不表示其能直接接收雷达 UDP。不要将雷达地址当作 SSH 目标，也不要为发现数据修改原厂机内网卡、路由或雷达配置。

## 2. DDS 与 ROS 2 通信配置

原厂 SDK2 在 DDS 之上封装发布 / 订阅和请求 / 响应。官方初始化接口可以分别指定域 ID、网卡与共享内存；未指定网卡时由中间件自动选择。外部开发应用应关闭共享内存，使用与机器人通信的实际接口。[DDS 通信接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/dds_service)

ROS 2 接入按以下顺序核对：

1. 确认目标运行环境已加载，核查是否具备 `rclpy`、标准传感器消息、rosbag2 与所需 RMW。安装了 SDK2 或 SDK2 Python，并不等于 ROS 环境已安装 Cyclone DDS 的 RMW 插件。
2. 官方 `unitree_ros2` 配置使用 `rmw_cyclonedds_cpp`，通过 `CYCLONEDDS_URI` 显式选择内部通信网卡。确认插件存在后，仅在本次诊断进程或容器中应用配置，不全局改写终端启动文件。
3. 原厂雷达示例使用域 `0`；本项目开发机的测试域不是实机域。先从原厂配置和实测核实域，再登记。
4. 容器内应能使用已确认的内部网络接口；仅有维护 Wi-Fi 或容器默认隔离网络时，发现结果不能代表内部 DDS 状态。
5. 检查真实发布者、消息类型和 QoS，再短暂订阅样本。没有发现 ROS 话题时，分别核对服务、路由、网卡选择、域、RMW 和消息支持，不能直接判定传感器未启动。

配置依据为[官方 unitree_ros2 说明](https://github.com/unitreerobotics/unitree_ros2)，其列出 Ubuntu 22.04 / Humble 作为已测试组合；A2 的 [DDS 文档](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/dds_service)也链接该仓库。具体机器人是否能通过此配置接收数据，仍以本轮观测为准。

DDS 名称与 ROS 名称分开记录。下文列出的名称是官方 DDS 通道；ROS 2 通常去掉 DDS 的 `rt/` 前缀并显示为以 `/` 开头的话题，这是命名映射预期，配置文件必须填写实测名称。

## 3. 雷达与雷达内置 IMU

| 官方 DDS 名称 | 官方消息类型 | 数据语义 |
|---|---|---|
| `rt/unitree/slam_lidar/points` | `sensor_msgs::msg::dds_::PointCloud2_` | 前后融合点云，以前雷达为坐标原点 |
| `rt/unitree/slam_lidar/points1` | 同上 | 前雷达独立点云，以前雷达为坐标原点 |
| `rt/unitree/slam_lidar/imu1` | `sensor_msgs::msg::dds_::Imu_` | 前雷达内置 IMU，以前雷达为坐标原点 |
| `rt/unitree/slam_lidar/points2` | 点云类型同上 | 后雷达独立点云，官方说明其坐标原点仍为前雷达 |
| `rt/unitree/slam_lidar/imu2` | IMU 类型同上 | 后雷达内置 IMU，以后雷达为坐标原点 |

来源：[雷达服务接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/lidar_service_interface)。这些类型对应 ROS 标准 `sensor_msgs/msg/PointCloud2` 和 `sensor_msgs/msg/Imu`，但需要实际中间件和类型支持匹配后才能订阅。

首条数据链优先核查前雷达独立点云与前雷达内置 IMU。融合点云不能直接当作单雷达的未经变换原始扫描；后点云与后 IMU 也不能仅凭编号相同就当作共用坐标系。录制时保留独立话题及实际 `frame_id`，登记变换来源。

官方称 JT128 内部 IMU 与雷达之间不涉及旋转和平移变换，同时提供前后雷达、前雷达与机身之间的结构设计外参。**结构设计外参不是标定结果**；不能据此宣称机身 TF、安装误差或运动补偿已经验证。外参文件、实际安装、坐标方向与唯一 TF 发布方须另行确认。

该接口页没有完整确定本项目需要的点内时间字段、类型、单位、扫描参考时刻、角速度 / 加速度单位或同步误差。因此，即使接收到标准消息，也需要保存原始消息并核查这些语义，再决定是否满足 DLIO 输入要求。

## 4. 机身 IMU 与状态消息

| 官方 DDS 名称 | 官方 SDK 类型组 / 消息类型 | 用途 |
|---|---|---|
| `rt/secondary_imu` | `hg` / `IMUState_` | 机身 IMU |
| `rt/lf/secondary_imu` | `hg` / `IMUState_` | 机身 IMU 的低频通道 |
| `rt/lowstate` | `hg` / `LowState_` | 包含 IMU、电机等底层反馈 |
| `rt/lf/lowstate` | `hg` / `LowState_` | 底层反馈的低频通道 |

来源：[DDS 通信接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/dds_service)。这些是厂商状态类型，不能直接填写到要求标准 `sensor_msgs/msg/Imu` 的采集角色中。若选择机身 IMU，需要先核实消息定义、时间戳、单位和坐标，再建立明确的转换；不能用接收时刻冒充采样时刻，也不能用姿态四元数替代六轴数据。

本项目首次 LiDAR/IMU 采集不要求发布运动命令，不执行低层控制示例或高层运动客户端。

## 5. 原厂建图输出与传感器输入分开保存

| 官方 DDS 名称 | 官方消息类型 / 内容 | 含义 |
|---|---|---|
| `rt/unitree/slam_mapping/points` | `sensor_msgs::msg::dds_::PointCloud2_` | 原厂建图过程输出的点云 |
| `rt/unitree/slam_mapping/odom` | `nav_msgs::msg::dds_::Odometry_` | 原厂建图过程输出的里程计 |
| `rt/unitree/slam_relocation/points` | 点云类型同上 | 原厂定位过程输出的点云 |
| `rt/unitree/slam_relocation/odom` | 里程计类型同上 | 原厂定位过程输出的里程计 |
| `rt/unitree/slam_relocation/global_map` | 点云类型同上 | 定位开始后发送一次的全局地图 |
| `rt/slam_info` | `std_msgs::msg::dds_::String_`，JSON 内容 | 周期状态、定位或控制状态反馈 |
| `rt/slam_key_info` | 字符串类型同上，JSON 内容 | 单次任务执行结果反馈 |

来源：[SLAM 和导航服务接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/slam_and_navigation_service_interface)。官方说明 SLAM 输出使用机身坐标约定；不要将这些结果话题与上一节的传感器输入混用，也不要把原厂里程计或地图当作独立真值。

部分输出只有进入相应建图或定位状态后才发布。发现这些话题为空，不授权自动调用开始建图、初始化位姿或导航接口。本轮确认传感器入口时优先使用已有数据，保持原厂任务状态。

## 6. 原装相机与多媒体入口

官方相机有以下数据入口，不能假定已存在 ROS 图像发布者：

| 入口 | 官方约定 | 本项目接入要求 |
|---|---|---|
| 视频接收 | RTP/H264，UDP 组播地址 `230.1.1.1`、端口 `1720`；接收端指定通信网卡 | 先验证实际收流与解码，不启动其他相机驱动抢占设备 |
| 视频规格 | 文档参数为 1280×720、15 Hz | 实测分辨率、频率、延迟与曝光时间来源 |
| JPEG 拍照 | SDK 的 `VideoClient` 提供 `GetImageSample` 获取照片 | 这是读取照片的请求 / 响应接口；与运动控制分开使用 |
| DDS 视频 | 编码后的视频，需要调用方解码 | 不能将编码字节当成标准原始图像消息 |

来源：[多媒体服务接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/Multimedia_Services)。视频订阅或照片读取成功，只证明图像入口可用，不能证明曝光时间、内参或外参合格。

当前采集工具的相机角色接收 ROS 原始 `Image`；只有 UDP / JPEG 接口时先登记实际入口。后续适配需明确解码、图像时间戳、光学坐标与相机参数，不能以网络到达时间冒充曝光时间。相机适配不阻塞已满足条件的雷达 / IMU 原始采集。

## 7. 时间同步与两条接入路线

官方将 PC1 内部时钟作为机内同步源，给出 Chrony 或 systemd-timesyncd 的配置方式。文档同时说明切换到 Wi-Fi 模式后可能触发网络授时并带来短时波动。[时间同步接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/time_sync_interface)

首次核查先读取现有同步状态，保存时钟源、跟踪结果及采集时间。不要在录制中修改系统时间、切换时钟服务或重启同步进程。当前维护连接若依赖 Wi-Fi，也不照搬文档中关闭 Wi-Fi 的建议；先安排独立维护窗口并确认替代连接。系统 NTP 同步不能单独证明雷达、IMU、相机硬件时间已经对齐。

### 路线 A：订阅原厂 DDS，优先采用

适用条件：原厂服务已提供上述雷达与 IMU 数据，版本、网络与类型支持能够核对。

保持原厂雷达服务运行，通过已确认的内部网卡与域接收数据。已有 ROS 2 / Cyclone RMW 时先核实标准消息直连；只有 SDK 环境时可先用 SDK 做有限时长订阅，确认消息与时间字段。若之后需要桥接到 ROS，必须保留源端时间和帧语义，桥接输入、输出及变换另行登记。

本路线不要求停止原厂服务，也不要求更换雷达驱动、修改雷达 IP 或占用原厂 UDP 接收端口。

### 路线 B：使用 Hesai 原生 SDK / ROS 驱动

适用条件：明确需要直接接收 JT128 原始数据，且已确认厂商 SDK / 驱动与固件兼容、雷达数据网可达、端口归属、时间同步及外参保存方式。

官方要求单独使用 Hesai SDK / ROS 驱动前先关闭 PC2 的原厂 SLAM 服务，以避免与现有数据链冲突。[雷达服务接口](https://support.unitree.com/home/zh/A2_SDK_Development_Guide/lidar_service_interface)

这会改变原厂感知 / SLAM 的可用性，因此先核实依赖、记录原状态并准备恢复方式，在停稳的维护窗口执行。首次接口确认不进入这一步，不复制永久禁用服务或 OTA 的命令作为默认操作。按本轮已授权范围和实际影响确定是否实施，不把文档中的操作建议本身当作执行授权。

## 8. 实机记录与公开边界

本页仅整理上述官方公开接口及通用核查方法，不披露具体机器的环境、内部部署、服务版本、实测计数、时钟偏差或标定配置。已开展的维护连接和软件验证，以及后续接入结果，分别保存于私有 `config/local`、`data` 或 `artifacts`，不能从本页推断某台机器人已经通过验收。

现场记录应逐项覆盖：

| 核查项 | 应保存的私有证据 |
|---|---|
| 主机角色与拓扑 | 已核实的维护入口、计算机职责、网卡和路由 |
| 原厂服务 | 状态、启动入口、实际配置与版本；屏蔽凭据 |
| ROS / SDK / DDS | 发行版、消息支持、RMW、域、接口、共享内存设置 |
| 传感器数据 | 独立通道、实际消息计数、字段、帧、时间与 QoS |
| 相机 | 收流、解码、曝光时间、内外参及实际参数 |
| 时间同步 | 机内来源、主机时差及传感器时间映射；不在采集中改钟 |
| 坐标关系 | 原始/变换后数据含义、标定文件与唯一 TF 发布方 |
| 原始录包 | 完整分片、来源版本、检查报告、摘要及备份 |

收到接口数据不等于已通过同步、标定、去畸变、地图质量或运动准入；软件测试与现场验收分别登记。凭据、现场配置、日志、地图和原始数据不进入公共源码仓库。
