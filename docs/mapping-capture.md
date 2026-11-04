# A2 原装传感器采集与建图验证

2026-11-05：原装雷达、IMU、摄像头可用；新增巡检载荷未安装。**现在先完成 A2 原始数据采集，再在本机验证 DLIO 和 RTAB-Map。** 新增云台、气体等载荷不阻塞这一步。当前尚未连接 A2、未取得实机录包、未完成建图验证，M0 仍未关闭。

## 1. 首次采集要回答的问题

| 数据 | 必须保存或查明的内容 |
|---|---|
| 前雷达原始点云 | 实际话题与 `sensor_msgs/msg/PointCloud2` 类型、xyz/intensity/ring/每点时间字段的名称、类型、单位、扫描参考时刻；保留未经裁剪的原始消息 |
| IMU | 实际来源（雷达内置或机身）、`sensor_msgs/msg/Imu`、角速度与加速度单位/轴向、频率、采集时间；不要把姿态四元数当作六轴原始数据 |
| 后雷达 | 可用时同时录制，区分前后话题、时钟和外参；首条里程计链以核实合格的前雷达与 IMU 为主 |
| 原装相机 | 驱动已提供 ROS 图像时录制 `Image` 与 `CameraInfo`；核实曝光时间、光学坐标与内参。只有视频/原厂接口时先登记，不为本次 LiDAR/IMU 采集临时更换驱动 |
| `/tf`、`/tf_static` | 保存当前 TF；同时复制厂商标定/驱动配置。未发布 TF 时先留原始数据，明确缺项，后续补齐标定，不能声称坐标链已验证 |
| 环境与时钟 | A2 型号、Ubuntu/JetPack/ROS、驱动/固件版本、机内网卡、ROS domain/RMW、时钟来源、同步配置与误差核查方法 |

DLIO 官方 ROS 2 说明要求点云、六轴 IMU 及正确的时间同步；本项目不能仅凭话题有数据判断其输入已合格。[DLIO ROS 2 说明](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/blob/feature/ros2/README.md)

工具只有订阅和文件写入，不创建运动客户端、启动驱动或改变网卡。运动由现场操作员通过原厂方式操作，使用熟悉的低速路线并保留独立急停。运行无外网不妨碍机内传感器通信；不要为测试关闭机内数据网络，也不要依赖外部 NTP 修正采集时钟。

## 2. A2 上先核查环境和真实话题

维护网络、SSH 登录、上传工具及回传数据的完整操作见[连接 A2 流程](a2-connection.md)。

通过已确认的维护 SSH 或本机终端，将本项目的 `scripts/` 及配置示例放到 A2 独立工作目录。Python 需要 3.10+ 和目标 ROS 环境提供的 `rclpy`、`sensor_msgs`、`tf2_msgs`、rosbag2 Python/CLI、对应存储插件。未满足依赖时在维护阶段准备；工具不会联网安装。不要把开发机的 Jazzy overlay 或 x86_64 编译产物复制给 A2。

先加载 **A2 实际安装的 ROS 与厂商驱动环境**，不要照搬开发机的发行版、domain 或 `ROS_LOCALHOST_ONLY`。保留已有原厂进程，先检查它们是否提供 ROS 2 话题。若只有原厂 UDP/DDS/视频数据，先保存此项核查结果，再按其实际驱动接口接入，不能猜话题名。

在 A2 的项目目录运行：

```bash
mkdir -p data/captures config/local
python3 scripts/a2_capture.py inventory \
  --output data/captures/a2-inventory.json
```

默认发现 3 秒，报告记录系统/ROS 环境、实际话题类型、发布者和 QoS。它不采集全量点云或图像，也不读取密钥。报告可能含现场话题名，随现场数据保存，不上传源码仓库。发现较慢时可增加 `--discovery-seconds 8`。

根据报告填写 `config/local/mapping-capture.json`，从 [配置示例](../config/examples/mapping-capture.json)复制；示例的空话题映射故意不能直接录制。必须提供 `front_points`、`imu`，可选 `rear_points`、`image`、`camera_info`。值必须是发现的绝对话题名，各角色不得重复，TF 由工具固定纳入。**不要将下表的角色名当作真实话题名。**

| 配置项 | 填写规则 |
|---|---|
| `schema_version` | 固定 `1` |
| `topics.front_points` / `topics.imu` | 原厂驱动实际输出的话题，消息类型必须对应上表 |
| `topics.rear_points` / `topics.image` / `topics.camera_info` | 可选；仅填写实际可收到数据的输入。当前相机入口支持原始 `Image`，不假装支持 `CompressedImage` 或 RTSP |
| `point_time.field` / `point_time.unit` | 根据驱动实现核实名称及 `s`/`ms`/`us`/`ns`；尚不清楚时两项均留空，允许先保存原始数据，检查会列为未决项。前后雷达字段不同须分别记录，当前单一声明只作为检查提示 |
| `storage_id` | 默认 `sqlite3`；A2 已安装并验证相应插件时也可用 `mcap` |

再次带配置核查，会短暂订阅传感器，记录点云字段和布局、首末 header/frame、IMU 有限性以及 TF 边：

```bash
python3 scripts/a2_capture.py inventory \
  --config config/local/mapping-capture.json \
  --output data/captures/a2-sensor-check.json
```

无核心话题、类型不符、配置输入收不到数据或空点云等返回非零。排查实际驱动、ROS 环境、QoS 和机内网络；不要以 mock 替代。TF 缺失、空 frame 和每点时间未知会警告，原始采集可继续，但不能放行建图。首次核查需另外登记固件/驱动版本、硬件型号、时钟映射和标定文件；脚本不会自动识别这些语义。

## 3. 先短包，再路线包

首次先录约 60 秒静止数据，确认写盘和输入完整后再录路线。下面先打印准确命令，不创建会话；添加 `--execute` 才执行预检和录制：

```bash
python3 scripts/a2_capture.py record \
  --config config/local/mapping-capture.json \
  --session data/captures/a2-static-01 --duration 60
python3 scripts/a2_capture.py record \
  --config config/local/mapping-capture.json \
  --session data/captures/a2-static-01 --duration 60 --execute
```

会话目录必须不存在，父目录必须存在；工具检查至少 5 GiB 空闲（可通过 `--min-free-gib` 调整，这只是启动下限，不是预计录包大小），显式录制已配置传感器及 TF，按约 2 GiB 分片。传感器 QoS 由 rosbag2 适配；`/tf_static` 显式请求 reliable/transient_local，以取得录制前发布且仍由发布者保留的变换。该发布者需实际提供对应持久性，覆盖不能找回已退出发布者的数据。[ROS 2 QoS 覆盖说明](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.rst)

达到时限或按 Ctrl-C 后发送 SIGINT，给 rosbag2 时间写完元数据；剩余空间低于 `--min-free-gib` 保留线时也提前收尾并返回非零。空间约每秒检查一次，不能保证突发写入期间绝不耗尽，因此仍需用短包估算磁盘速率并预留足够容量。预检有样本不代表整个录制过程无丢帧。异常或磁盘错误需保留现场日志并重新检查，强制收尾的结果会标记不完整。

静止短包合格后，使用新的目录，例如 `a2-route-01`，将 `--duration` 改为现场预计上限（如 `300` 秒）。建议依次采集：

1. 静止初始化，记录独立参考点和机器人朝向。
2. 平稳直行、停车、低速转弯，检查点云运动补偿所需输入。
3. 短路线往返，回到已标记参考点；条件允许时包含转角或闭环，保留长直重复结构片段。
4. 再次静止，记录结束参考点、实际路程/时间与异常。相机可用时覆盖光照变化、可识别结构及已有标签。

这是初次采集建议，不是性能验收阈值。独立测量与现场记录用于后续误差评估，不以 SLAM 自己的闭环结果充当真值。首批采集不运行本项目自主运动或任务控制代码。

每次会话包含 `config.json`、`preflight.json`、`qos.yaml`、`recording.json`、正常收尾时的 `recording-result.json` 和 `bag/`。另在会话下保存 `field-notes.md`、原厂标定与配置副本，注明 IMU 来源、点内时间参考、时钟来源、路线参考、人工操作和异常；不要写入 SSH 密钥。进程被杀或断电时可能缺少结果/metadata，不能视为完成。

## 4. 离场前检查实际消息

先运行 `ros2 bag info`，再用 [离线检查脚本](../scripts/inspect_mapping_bag.py)逐条检查实际传感器消息：

```bash
ros2 bag info data/captures/a2-static-01/bag
python3 scripts/inspect_mapping_bag.py data/captures/a2-static-01/bag \
  --config data/captures/a2-static-01/config.json \
  --output data/captures/a2-static-01/bag-check.json
```

检查消息数量与类型、header 时间范围/重复/倒序、消息间最大间隔、frame、点云字段和布局、配置的每点时间字段、IMU 数值以及 TF 边。报告不输出原始图像/点数组；原始包只读。未通过项需要排查或补录，先检查报告再决定是否离场。检查也不能独自证明雷达/IMU 共钟、时间单位正确、轴向外参合格或达到建图精度。

在维护网络或受控介质传回本机时复制 **整个会话及 bag 全部分片**，核对源端与目标端逐文件 SHA-256，保留源端副本直至确认。原始数据位于被忽略的 `data/`，不进入 GitHub。开发机本身的 ROS 也需安装对应消息类型/存储插件；跨发行版格式与序列化兼容性以实际短包读取验证为准。

## 5. 拿到数据后的开发顺序

1. 根据实机环境与短包固定驱动、DLIO ROS 2、RTAB-Map 版本，准备匹配环境的本机依赖。已完成固定 DLIO 提交在本机 Jazzy/x86_64 的独立候选构建和进程启动，RTAB-Map 使用本机 0.23.13；详细版本、输入契约待项与离线采集包见[建图准备记录](mapping-readiness.md)。完整回放 launch 尚未实现，候选尚未通过 A2/真实数据验证。
2. 根据真实 JT128 每点时间、IMU 来源/轴向/外参实现必要输入适配，先验证 DLIO 连续里程计与单帧去畸变点云。缺时钟/外参证据时先补齐，不能只换话题名。
3. 接入 RTAB-Map，明确每段 TF 的唯一发布者，验证地图保存、重新加载定位及往返表现。
4. 用独立参考评估入口初始化、长直段漂移、转角/闭环及返回误差，整理 M0/M1 证据与剩余项。

回放在与机器人网络隔离的本机环境执行，只播放审查后的传感器话题；遵循[开发流程](development.md)的白名单、TF 和模拟时钟约定。本轮工具可用于采集和数据核查，不代表 DLIO/RTAB-Map 已打通，更不代表 M0 所需的底盘失效与停车边界已验证。

## 6. 工具自身的本机验证

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s scripts/test -v
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=78 python3 scripts/capture_ros_smoke.py
```

第二条需要本机 DDS 套接字权限，使用独立 domain 和回环网络、临时目录、合成点云/IMU/TF。它检查真实录制与实际消息读取、话题白名单、时限正常收尾以及录制前一次发布的静态 TF 能被保存，不运行回放或算法，也不验证 A2。此命令用于开发机测试；不要将其 ROS 环境变量照搬到 A2 采集终端。CI 的 ROS 工作流也运行该检查。

当前无法连接 A2 时，可先按[建图准备记录](mapping-readiness.md)生成带操作说明和文件校验清单的独立采集 ZIP，通过受控介质带到现场。该包只有源码工具，目标机仍须具备实际 ROS/驱动及存储插件。
