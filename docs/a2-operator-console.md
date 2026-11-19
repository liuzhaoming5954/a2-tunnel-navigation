# A2 上位机

Vue 3 工作台以巡检驾驶舱为首页，维护遥控位于独立页面。Python 网关提供同源 HTTP 接口，构建后的工作台可在本地离线运行。机载系统负责离线巡检和运动授权。

巡检业务页面的规划、交互流程和分期验收见[上位机设计方案](design/a2-operator-console-design.md)。

## 启动模拟操作台

在工程根目录用 Node.js 22 和 Python 3.10 或更新版本执行。

```bash
npm ci --prefix src/a2_gateway/web
npm run build --prefix src/a2_gateway/web
python3 scripts/run_a2_console.py
```

打开[巡检驾驶舱](http://127.0.0.1:8765/#/cockpit)。「巡检态势地图」位于标题下方，占满工作区宽度，默认显示「ODIN1 点云」。状态卡、当班任务和告警位于地图下方。拖动旋转，使用滚轮或缩放按钮查看局部，点击「俯视」可从上方查看地图。图中保留源 PCD 颜色，面板标明文件名、展示点数和原始点数。接入 PC2 后，圆点标出 ODIN1 的 IMU 原点，箭头表示传感器 +X 轴在地图 XY 平面内的朝向。机身安装偏移待确认。

网关从受控目录 `maps/odin1/preview/` 读取配套的 `metadata.json` 和 `points.bin`。浏览器先读取 `/api/maps/odin1/metadata`，再携带清单中的样本版本读取 `/api/maps/odin1/points`。现场点云独立于前端构建和通用发布包。在其他工作区部署时，可将这对文件放在网关默认目录，或用 `--odin1-map-dir` 指定其所在目录。文件缺失时，补齐后点击「重新加载」即可。

展示文件采用固定记录格式。`points.bin` 每点占 16 字节，前 12 字节是按小端序存储的三个 float32 坐标，接着是 R、G、B 三个颜色字节和一个填充字节。`metadata.json` 中的 `sourceName`、`sourcePoints`、`pointCount` 分别记录源 PCD 文件名、原始点数和抽样点数。`strideBytes` 为 16，`encoding` 为 `xyz-f32le-rgba-u8`，`sampleSha256` 取自展示文件本身。`viewBounds.min` 和 `viewBounds.max` 是初始取景所用的三维坐标范围。

切到「示意场景」可查看合成管廊、机身和巡检点位。点击本体、充电仓、任务或告警可看详情，从演示任务或告警选择关联点会进入该场景。切到「二维栅格」可查看网关地图和与其匹配的机身位姿。

| 驾驶舱信息 | 如何识别数据来源 |
| --- | --- |
| 本体、电量、位姿和二维栅格 | 来自当前 Python 网关。默认启动标记为「网关模拟」，接入 ROS 时按连接模式标明实机来源。缺测电量和过期位姿显示未知。 |
| ODIN1 三维点云 | 本地实测 PCD 的静态抽样，地图来源独立于网关遥测模式。 |
| ODIN1 传感器位置和航向 | 来自 PC2 同一设备时刻的重定位观测，面板显示地图坐标（米）和航向（度）。等候输入、位姿过期或地图不匹配时撤下标记。 |
| 示意场景、三维机身和示例点位 | 本地合成示意，与网关地图使用独立坐标。 |
| 充电仓、当班任务、告警 | 本地模拟样例。演示阅处只影响当前页面；「数据来源」可切换为仅查看已接入数据。 |

导航支持直接打开链接、刷新和浏览器前进／后退。其余业务页面已预留入口并标为待接入。需要测试模拟遥控时打开[维护遥控](http://127.0.0.1:8765/#/maintenance)，点击「启用控制」后按住页面方向按钮或 W、A、S、D。松开会停下模拟机器人并解除会话，也可点击「停止并解除」。离开维护页、窗口失焦、断连或会话到期时解除控制，停止请求未获确认时继续显示状态。模拟速度限值用于联调。

开发界面时可保持网关运行，在另一个终端执行 `npm run dev --prefix src/a2_gateway/web`，Vite 将 `/api` 转发到 8765。

## 将 PC2 的 ODIN1 实时重定位叠加到点云

先在 PC2 按[已验证的载图和重定位流程](development.md#32-pc2-上的-odin1-两点重定位)启动对应的厂商驱动。驱动的标准输出和标准错误写入本次会话独立的 `driver.log`。点云目录中的 `metadata.json` 需要登记 `relocMapSha256`，值为与该 PCD 配对的厂商 BIN 的 SHA-256。现场地图、标定和状态文件均留在受控目录。

在 **PC2 的驱动环境**中另开终端，加载相同的 ROS 2、RMW 和 ROS 域配置。将下面的变量设置为本轮使用的 BIN 绝对路径、驱动日志路径和可写的私有状态目录。以运行驱动的用户启动只读采集器。

```bash
python3 scripts/observe_odin1_pose.py --map-bin "$ODIN1_ACTIVE_BIN" \
  --driver-log "$ODIN1_DRIVER_LOG" --output "$ODIN1_SESSION_DIR/pose.json"
```

在 **同一台 PC2 的上位机环境**中启动网关。如下例采用已有的只读 ROS 遥测环境；网关使用的 ROS 域可与 ODIN1 驱动域不同。若使用下文的只读启动脚本，可通过 `A2_ODIN1_MAP_DIR` 和 `A2_ODIN1_POSE_FILE` 传入同样的路径。

```bash
python3 scripts/run_a2_console.py --ros --host 127.0.0.1 \
  --odin1-map-dir "$ODIN1_DISPLAY_DIR" --odin1-pose-file "$ODIN1_SESSION_DIR/pose.json"
```

开发机通过维护 SSH 本地端口转发访问 PC2 网关。采集器从本次驱动日志确认已加载的地图和重定位模式，按设备时间戳配对 `/odin1/odometry` 与动态 `/tf`，以 `inverse(T_odom_map) × T_odom_imu` 计算传感器地图位姿。网关在 PC2 上用同次系统启动的单调时钟判断时效，默认有效期 750 毫秒。浏览器请求 `/api/maps/odin1/pose?version=<点云样本的 SHA-256>` 并在位姿到期时撤下标记。

验收时可先让 A2 静止，将页面 XYZ 和航向与 PC2 同时刻的读数对照，在已知点核对图上位置。切到俯视再旋转地图，箭头应始终指向同一地图方向。停止采集或断开转发后，页面应撤下标记并保留已加载的点云。

## 接入 ROS 2 遥测

加载与实机一致的 ROS 2 环境和已构建的 `a2_interfaces`，再执行下例。先在目标 ROS 图中核对实际话题、坐标系、QoS 和机内网卡。下列名称是项目默认输入。

```bash
python3 scripts/run_a2_console.py --ros \
  --map-topic /map --odom-topic /odom --platform-topic /a2/platform/state \
  --map-frame map --odom-frame odom --base-frame base_footprint
```

地图使用 `nav_msgs/OccupancyGrid`。位姿按里程计采样时刻查 `map → odom → base_footprint`，网关将其关联到当前栅格。电量、速度和模式使用 `a2_interfaces/PlatformState` 的有效标记。过期或缺失字段在界面显示为不可用。实机实时显示依赖已启动的机载发布者与维护连接。订阅配置可通过 `--help` 查看。

已有建图产物可使用同一套 `map.yaml` 和 PGM，由机载地图服务加载后发布 `/map`。定位进程需使用这套地图对应的建图数据和坐标系，才能在界面上正确叠加机身位姿。[地图导出与目标点验证](a2-goal-validation.md)

## PC2 只读部署

开发机完成 Vue 构建后，用已核对的地图生成离线包。包内有静态页面、Python 工具、接口源码和所选地图。地图随维护包传到机器人，保存在受控目录。[维护连接与传输](a2-connection.md)

```bash
python3 scripts/package_a2_console.py --map "$A2_LOCAL_MAP_YAML" \
  --output artifacts/a2-console-readonly.zip
```

在 PC2 的新候选目录解压，执行 `sha256sum -c SHA256SUMS`。加载 PC2 的 ROS 2 和本机编译的 `a2_interfaces`，把原厂 Python SDK 目录加入 `PYTHONPATH`。如需构建接口，可在候选目录运行 `colcon build --base-paths src/a2_interfaces --packages-select a2_interfaces`，再加载 `install/local_setup.bash`。

以下命令在已加载环境的 PC2 候选目录执行。按本轮配置导出 `A2_SDK_NIC`、`A2_SDK_DOMAIN_ID`、`A2_ROBOT_ID`，并设置供 PC2 本机显示链使用的 `A2_DISPLAY_DOMAIN_ID`。原厂 SDK 使用自己的网卡和域，状态接收放在独立进程。

```bash
ROS_DOMAIN_ID="$A2_DISPLAY_DOMAIN_ID" ROS_LOCALHOST_ONLY=1 A2_MAP_YAML="$PWD/data/map.yaml" \
  bash scripts/start_a2_console_readonly.sh
```

启动脚本一起运行只读状态发布器、占据栅格发布器和回环网关。前台执行时可按 Ctrl+C 停止三个服务。通过认证的 SSH 本地转发访问网关，经过 PC4 时将其作为 SSH 跳板。用 `/api/state` 核对 `robot.online` 和 `robot.telemetry_stamp_ms`，用 `/api/map` 核对地图尺寸和坐标系。接入连续定位时，定位、地图和网关须使用一致的 ROS 域和地图坐标，先核对各话题的发布者与 TF。

原厂状态桥当前只标记已核对的运控模式字段。未验证的电量和速度显示为未知。

## 连续定位联调

在所选地图的可识别参考点停稳后，使用实时点云与惯导建立本轮锚定。定位桥要求报告的留出扫描验证通过，报告中的扫描时间戳必须出现在同一运行中 DLIO 的日志，地图文件也须与导出摘要一致。保持该 DLIO 进程连续运行，在 PC2 加载它的 ROS 环境后接入显示。

```bash
python3 scripts/publish_a2_localization.py \
  --anchor-report "$A2_LIVE_ANCHOR_REPORT" --dlio-log "$A2_LIVE_DLIO_LOG" \
  --dlio-pid "$A2_LIVE_DLIO_PID" --map-yaml "$A2_CONSOLE_MAP_YAML" \
  --map-manifest "$A2_CONSOLE_MAP_MANIFEST" --extrinsics "$A2_LIDAR_EXTRINSICS" \
  --output-domain "$A2_DISPLAY_DOMAIN_ID"
```

定位桥从原厂域读取诊断 DLIO，经独立进程在隔离显示域发布 `/odom` 和 `map → odom → base_footprint`，使用原始扫描时间戳。机身位置和朝向按提供的前雷达外参换算。源中断、进程重启、坐标冲突或地图不符会使定位停止更新，页面将过期位姿标为无数据。恢复时使用新锚定。[机身定位与验证方法](a2-goal-validation.md)

### 静止波动与终点匹配复测

同一停姿下重新匹配时，取证工具使用 `--probe` 指定受信任的本地定位诊断脚本。探针和它的辅助模块由本地诊断环境提供。输出包含扫描、拟合候选、时间戳、里程计变换和校验摘要，保存在现场受控目录。录包回放仍可使用点云与 IMU 短包。[实机录包指南](mapping-capture.md)

```bash
python3 scripts/record_a2_anchor_evidence.py live --probe "$A2_ANCHOR_PROBE" \
  --map-database "$A2_MAP_DB" --map-poses "$A2_MAP_POSES" --dlio-log "$A2_LIVE_DLIO_LOG" \
  --samples-output "$A2_TRIAL_DIR/selected-scans.npz" --output "$A2_TRIAL_DIR/anchor-evidence.json" \
  --duration-s 25 --after-anchor-s 4

python3 scripts/record_a2_anchor_evidence.py replay --probe "$A2_ANCHOR_PROBE" \
  --map-database "$A2_MAP_DB" --map-poses "$A2_MAP_POSES" \
  --samples "$A2_TRIAL_DIR/selected-scans.npz" --geometry \
  --output "$A2_TRIAL_DIR/anchor-replay.json"
```

离线回放加 `--geometry` 时，报告列出每个拟合的前雷达地图变换、平移信息归一化特征值，以及地图坐标系下约束最弱的平移方向。候选是否通过仍以原几何和留出扫描检验为准。

用已锚定的参考扫描复现终点冷启动候选。

```bash
python3 scripts/record_a2_anchor_evidence.py replay --probe "$A2_ANCHOR_PROBE" \
  --map-database "$A2_MAP_DB" --map-poses "$A2_MAP_POSES" \
  --samples "$A2_TRIAL_DIR/selected-scans.npz" --geometry \
  --output "$A2_TRIAL_DIR/anchor-replay-with-references.json" \
  --reference-samples "$A2_EARLIER_SELECTED_SCANS" \
  --reference-anchor "$A2_EARLIER_ACCEPTED_ANCHOR" \
  --reference-dlio-log "$A2_EARLIER_DLIO_LOG"
```

补充参考来自同一地图下已锚定的独立录包。程序通过采样与地图摘要、锚定报告及 DLIO 日志的精确时间戳配对核对来源，并检查参考与查询的采样时间。该入口生成离线报告。

## 实机运动接入

当机载运动仲裁、避障、底盘桥接和接收端失联停车已联调，可在 ROS 模式下增加 `--motion-socket /absolute/path/to/arbiter.sock`。网关在回环地址运行，浏览器可在同一主机打开，维护时也可使用 SSH 本地端口转发。实际运动权限、限值、会话有效期和底盘断流处理由机载仲裁与接收端负责。当前仓库的实机运动仲裁和底盘指令桥待集成，现有 A2 实机接入范围为只读遥测。

若网关运行在机载维护主机，开发机可以将本地 8766 端口转发到机载网关。设置 `A2_SSH_TARGET` 为实际 SSH 用户名和维护地址，保留该终端，再打开 [转发后的操作台](http://127.0.0.1:8766/)。

```bash
ssh -N -L 127.0.0.1:8766:127.0.0.1:8765 "$A2_SSH_TARGET"
```

Unix 套接字使用一问一答的换行 JSON。网关发出 `schema_version: 1`，每次连接传送一个请求，服务端返回带布尔值 `ok` 的单行响应。服务端进程须以网关同一账号或 root 运行。

| `op` | 请求和返回 |
| --- | --- |
| `status` | 返回 `control`，包括 `available`、`armed`、`receiver_stop_verified`、可选 `reason` 及 `limits.linear_mps`、`limits.angular_radps`。 |
| `acquire` | 核实当前本地授权后签发 `session_id` 和 `expires_in_ms`。会话标识使用 16 至 128 位 ASCII 字母、数字、下划线或连字符。 |
| `velocity` | 接收 `session_id`、递增的 `sequence`、机身前向 `linear_mps` 和绕 Z 轴的 `angular_radps`。服务端校验时效、控制权和限值后受理。 |
| `stop` | 接收 `session_id`，撤销对应会话并触发停止。现场独立急停由原厂通道负责。 |

仲裁服务不可达、拒绝请求或未确认接收端停车时，界面显示控制不可用。操作页面与遥控会话的断开处理属于维护模式，离线自主任务继续使用其独立授权。[首次受控运动与停车验证](a2-first-motion-test.md)

## 本机联调检查

```bash
PYTHONPATH=src/a2_gateway python3 -m unittest discover -s src/a2_gateway/test -v
```

测试覆盖 HTTP 接口、模拟超时停车和本地仲裁接口的授权响应。浏览器验证可以使用模拟地图检查缩放、键鼠操作、松手和失焦。实机运动和停车按照现场验证流程执行。
