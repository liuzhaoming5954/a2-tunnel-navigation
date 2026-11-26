# A2 上位机

巡检驾驶舱部署在操作员使用的上位机。PC2 默认通过两个只读接口传回机器人本体状态和 ODIN1 传感器地图位姿。两端显式启用单点导航后，操作员还可下发或取消一个地图目标。上位机在浏览器内绘制点云、位置与航向，统一提供业务页面。更新界面时只需重新构建并替换上位机的前端资源。[设计方案](design/a2-operator-console-design.md)

| 运行位置 | 持有的数据或文件 | 对外提供的内容 |
| --- | --- | --- |
| 上位机 | Vue 构建产物、展示点云及其清单，可选的二维栅格 YAML 和相邻 PGM | 本地驾驶舱网页、地图资源和浏览器使用的同源 HTTP 接口 |
| PC2 | ODIN1 驱动及已加载的厂商 BIN、位姿观测文件、平台状态发布器 | 机器人只读状态和经地图与时效校验的 ODIN1 实时位姿 |
| 上位机浏览器 | 由上位机 HTTP 服务提供的页面和地图 | 绘图、查看详情、切换视角与数据来源 |

开发机回环验证和 PC2 实机联调的条件与结果分别记录在[开发进度](implementation-status.md#巡检驾驶舱)。

## 在上位机启动驾驶舱

从源码启动时，先在工程根目录构建前端。开发环境使用 Node.js 22 和 Python 3.10 或更新版本。使用已构建的上位机离线包时，可以直接进入解压目录运行 Python 入口。

```bash
npm ci --prefix src/a2_gateway/web
npm run build --prefix src/a2_gateway/web
python3 scripts/run_a2_console.py --role station
```

正式站端使用默认的 `station` 角色，在本机 8765 端口提供巡检驾驶舱，默认经本机 8766 端口读取 PC2 数据。未接设备时仍可查看本地地图。实时状态与位姿随连接及观测有效性显示。[打开巡检驾驶舱](http://127.0.0.1:8765/#/cockpit)

查看单点导航时，在同一驾驶舱选择「巡检作业 → 任务管理」。页面显示目标 X、Y 和朝向。站端只读时，页面提示单点导航未启用，输入和下发操作禁用。执行导航需要上位机与 PC2 同时启用 `--enable-single-goal` 并满足导航就绪条件。此前使用 8776 临时预览的书签可换为同站任务页。[打开任务管理](http://127.0.0.1:8765/#/tasks) · [单点导航部署](pc2-single-goal.md)

静态三维点云默认位于上位机的 `maps/odin1/preview/`。若目录不同，通过 `--odin1-map-dir` 指定。需要二维地图时，额外传入 `--map-yaml` 并将配套 PGM 放在 YAML 引用的位置。下面的环境变量分别指向上位机存放点云的目录和二维地图的 YAML 文件。省略 `--map-yaml` 即可只使用实测点云和合成示意场景。

```bash
python3 scripts/run_a2_console.py --role station \
  --odin1-map-dir "$ODIN1_DISPLAY_DIR" --map-yaml "$A2_LOCAL_MAP_YAML" \
  --pc2-url http://127.0.0.1:8766
```

来源清单 `metadata.json` 与抽样文件 `points.bin` 应配套保存。每个点占 16 字节，含小端 float32 的 XYZ、RGB 和一个填充字节。清单的 `sampleSha256` 校验展示样本，`relocMapSha256` 关联该点云对应的厂商重定位 BIN。浏览器请求 `/api/maps/odin1/metadata` 后携带样本版本请求 `/api/maps/odin1/points`。文件缺失时，补齐后可点击「重新加载」。

| 驾驶舱信息 | 来源与显示 |
| --- | --- |
| 本体、电量和运动状态 | 上位机转取 PC2 的只读平台状态。机器人本体可查看原厂偏航角速度和 IMU 估算转速。无有效数据时对应读数显示未知。 |
| ODIN1 点云、二维栅格 | 上位机本地地图。点云面板显示原始文件名、抽样点数和原始点数。 |
| ODIN1 传感器位置及航向 | PC2 同图重定位。圆点对应传感器 IMU 原点，箭头对应传感器 +X 轴在地图 XY 平面内的投影。机身安装外参有待量测。 |
| 示意场景与点位 | 本地合成场景，可从示例任务或告警跳转到关联点位。 |
| 充电仓、任务和告警 | 上位机默认仅显示已接入数据。可在「数据来源」开启已标明来源的本地模拟业务，独立模拟角色默认展示样例。 |

## IMU 偏航转速

本体摘要、详情和维护页以 °/秒显示带符号的 IMU 转速估计。正值表示左转，负值表示右转。「—」表示暂无有效读数。原厂 `rt/lf/sportmodestate.yaw_speed` 通过 `robot.velocity.yaw_radps` 单独保留，界面用 rad/s 显示。

PC2 以 `rt/lf/lowstate` 的 IMU 偏航角为输入，取同一采集会话相邻观测在 ±180° 内的有符号角差，再除以两帧在 PC2 的单调接收时间间隔。用于差分的间隔为 25–250 毫秒。估算在 250 毫秒后过期，重复发布不续期。首帧、新采集会话或无有效偏航角时显示未知。

`robot.attitude.yaw_rate_radps` 是可选的只读字段。对照原厂数据、实测后台记录和现场停稳确认时，可分别查阅估算转速和原厂字段。[机载计算](../src/a2_gateway/a2_gateway/ros_telemetry.py) · [部署与实机记录](implementation-status.md#imu-偏航转速实机验证)

从源码新建 PC2 环境时，先编译并加载包含 `PlatformDetails` 和 `BatteryTelemetry` 的 `a2_interfaces`，再启动原厂只读采集器和 PC2 网关。IMU 话题明细由 `/a2/platform/details` 提供。上位机遇到仅提供基本状态的旧服务时继续显示已有读数，IMU 转速显示「—」。[平台明细消息](../src/a2_interfaces/msg/PlatformDetails.msg)

## 开发机离线模拟

开发或演示两点巡检时，按需在开发机启动 `simulator` 角色。它使用与正式站端相同的前端构建，运动和停留均由本地模拟器执行。示例使用独立的开发端口 8775。

```bash
python3 scripts/run_a2_console.py --role simulator --port 8775
```

如需演练维护遥控，在模拟器打开 `#/maintenance`。模拟控制启用后，可按住方向按钮或 W、A、S、D。松开、离开维护页、窗口失焦或会话超时后，既有解除逻辑结束控制。开发界面时可在默认 8765 端口运行所需角色，再执行 `npm run dev --prefix src/a2_gateway/web`。Vite 将同源 `/api` 请求转至本机 8765。

### 两点模拟巡检

启动开发机模拟器后，进入「巡检作业 → 任务管理」，也可以直接打开[两点模拟巡检](http://127.0.0.1:8775/#/tasks)。在合成地图坐标系内分别填写两个点的 X、Y（米）和停留时间（秒），点击「下发两点巡检」。页面使用同一任务 ID 分两次下发，收到点 1 的受理回执后再发点 2。

默认点位位于同一条直通道。模拟器从 `(−4.85, −1.10)` 起步，按顺序去 `(−4.15, −1.10)` 和 `(−3.45, −1.10)`，默认分别停留 2 秒和 4 秒。到点容差为 0.02 米。进入容差后先停车，模拟机身线速度和角速度连续 0.20 秒为零才开始计时。点 1 停留结束后才会向点 2 出发，点 2 停留结束后任务完成。

执行记录保留每个点的接收、出发、到达、停稳和停留结束时刻。相减「停留结束」与「已停稳」的任务用时即可复核每点的停留时长。网页刷新后会重新读取当前任务。如果点 1 已接收而点 2 下发失败，可以修改未接收的点并继续下发。任务由模拟进程在后台执行，重启模拟进程会清空本轮任务。点间按直线行进，遇到合成地图中的障碍会停下并报告失败。

## 机身遥控演练

驾驶舱提供「系统管理 → 维护遥控」页面，地址为 `#/maintenance`。方向按机身坐标解释，定位地图和 ODIN1 重定位均可单独接入。按「启用控制」取得一次独占会话，再持续按住屏幕按钮、键盘方向键或 WASD。每次只执行一个方向。

取得会话后，按钮显示「控制已启用」并禁止重复点击。单次许可从 PC2 签发时起算，启用后显示剩余会话时间。连续点动许可启用后，维护页按每段动作显示剩余时间。

| 操作 | 键盘 | PC2 允许的机身速度上限 |
| --- | --- | --- |
| 前进、后退 | ↑ 或 W、↓ 或 S | 沿机身 X 轴 +0.20、−0.20 m/s |
| 左转、右转 | ← 或 A、→ 或 D | 绕机身 Z 轴 +10、−10°/s，接口使用弧度/秒 |

表中为当前源码的指令速度。受管接收器通过 `control.limits.forward_mps` 单独上报前向限速。旧版接收器只有 `linear_mps`，网页继续按其上报值发出前进和后退指令。实际速度以机身遥测为准。

驾驶舱手动遥控由现场操作员判断净空。PC2 根据签发的方向限制前进、后退和转向。

单次许可下，松开方向键或按钮会撤销本次控制并请求停车。连续点动许可下，松键结束当前动作，驾驶舱等待 PC2 确认停稳，随后可重新按键。窗口失焦、切走页面和断开控制链路均会结束整个会话。即使停车请求没有送达，PC2 自建立会话起也使用本机单调时钟检查 **0.40 秒**超时。就绪时，页面发送零速心跳保持连接。运动中只有对应当前动作、序号递增的速度包才能刷新断流计时。超时会撤权并请求停车，连接恢复后需重新启用。实际停车耗时还包括看门狗调度、底盘响应和制动。

**键盘 A 用于向左转。原厂遥控器的实体 A 键用于停车并锁存。** 停车锁存由 PC2 独立守护管理。驾驶舱顶部的「复位停车锁存」通过专用维护通道请求 PC2 复位。[独立遥控停车](a2-remote-stop.md)

正式 PC2 在原有只读接口 `GET /api/robot/state` 增加 `remote_stop`，上位机由 `GET /api/state` 提供给驾驶舱。PC2 以遥测服务的普通账号约每秒读取一次 systemd 的守护状态，并与 `/run/a2-remote-stop/ready` 交叉核对。守护报告「停车已锁存」且没有放行标志时，返回 `guard=running`、`latched=true`。正式只读站端把禁用原因设为 `stop_latched`。只有运行中的守护报告已布防且存在有效放行标志时，页面才显示「停车未锁存」，此时只读模式仍禁止遥控。

采样超过 2.5 秒、链路中断、旧版 PC2 缺少字段或机载证据不一致时，页面撤下无法确认的状态，禁止启用实机遥控。能确认守护已停时显示「未运行」，锁存显示「未知」。锁存展示和禁用提示可在静止状态核对，全程只需查看驾驶舱和维护页。[机载采集](../src/a2_gateway/a2_gateway/remote_stop_status.py) · [独立停车守护](a2-remote-stop.md)

在配置了独立维护通道的上位机，驾驶舱另从本机只读接口 `/api/telemetry` 刷新机身与守护状态。维护通道的响应时间不计入这次遥测。双电池、速度或姿态超过各自有效期时，页面最多额外保留 200 毫秒并标为「上次读数」。持续断流或已确认断开时撤下读数。停车守护和锁存照常按有效期显示，复位与运动控制请求仍分别即时校验。[站端遥测](../src/a2_gateway/a2_gateway/station.py) · [驾驶舱轮询](../src/a2_gateway/web/src/useGateway.js)

### 开发机全链路模拟

在开发机工程根目录构建前端后，运行下列程序。它在同一台开发机创建临时站端、PC2 网关、Unix 套接字和模拟底盘，网页与这三个进程之间的数据均留在本机。程序会打印本次演练页面地址和模拟 A 键输入文件路径。已运行的正式驾驶舱可以保持打开。[演练入口](../scripts/run_a2_manual_demo.py)

```bash
npm run build --prefix src/a2_gateway/web
python3 scripts/run_a2_manual_demo.py --port 0
```

在程序打印的页面确认控制来源为模拟底盘，按住不同方向时观察接收器输出的 `move` 事件和机身速度。每次重新启用后，分别验证松键、切走窗口、断开演练链路。停车对应 `stop` 事件，模拟机身线速度和角速度归零。模拟实体 A 键时，在第二个终端新建程序打印的那个模拟输入文件，确认锁存后新会话被拒绝。测试底盘未收到网页停车报文的场景，应只断开演练链路，通过接收器自己的日志核实 `deadman_expired`。模拟停车回执、真实底盘收到指令以及真实机身停稳分别验收。[接收端测试](../src/a2_safety/test/test_manual_receiver.py)

### PC2 仅运行模拟底盘

需要验证上位机与真实 PC2 之间的维护网络时，可以把临时网关和模拟底盘放在 PC2，仍由开发中的站端演练页发操作。这个入口始终创建 `mock` 接收器，无需加载 ROS 或宇树 SDK。原有驾驶舱、只读遥测服务和独立停车守护继续使用各自的端口和配置。

1. 在两台主机上准备同一份临时机间密钥。各自的 `$A2_MANUAL_KEY_FILE` 指向当前账号或 root 所有的私有普通文件，权限为 `0600`，内容为 32 至 128 个 URL 安全的字母、数字、下划线或连字符。凭据由两端程序读取，不进入网页。
2. 在 PC2 的本次演练目录运行以下命令，记下程序打印的回环 HTTP 端口，作为上位机环境中的 `$A2_MANUAL_PC2_PORT`。

   ```bash
   python3 scripts/run_a2_manual_demo.py --pc2-only --port 0 \
     --manual-key-file "$A2_MANUAL_KEY_FILE"
   ```

3. 在上位机通过已核对的维护 SSH 登录建立专用转发，选择空闲的 `$A2_MANUAL_LOCAL_FORWARD_PORT`。现场固定转到只读网关的专用密钥只服务其登记的目标，此处使用具有本次临时端口转发权限的维护登录。切断此终端即可单独演练上位机到 PC2 断网。

   ```bash
   ssh -N -o ExitOnForwardFailure=yes \
     -L "127.0.0.1:${A2_MANUAL_LOCAL_FORWARD_PORT}:127.0.0.1:${A2_MANUAL_PC2_PORT}" \
     "$A2_SSH_TARGET"
   ```

4. 在上位机另一终端启动独立站端，`$A2_MANUAL_STATION_PORT` 选择与正式驾驶舱和转发都不同的空闲端口。浏览器访问这个新端口的 `/#/maintenance`，核对模拟底盘来源再演练。停车与断网的测量取 PC2 接收器输出的 `monotonic_s`。

   ```bash
   python3 scripts/run_a2_console.py --role station --host 127.0.0.1 \
     --port "$A2_MANUAL_STATION_PORT" --enable-manual-control \
     --manual-key-file "$A2_MANUAL_KEY_FILE" \
     --pc2-url "http://127.0.0.1:${A2_MANUAL_LOCAL_FORWARD_PORT}"
   ```

## 上位机与 PC2 离线打包

单点导航在上位机与 PC2 两端均通过 `--enable-single-goal` 单独启用。按[PC2 单点导航](pc2-single-goal.md)准备配对地图、机身外参和导航执行端，再在任务管理页查询就绪状态。

在开发机分别打包。上位机包应在 Vue 构建成功后生成。PC2 包含只读服务、采集器、平台状态发布脚本和接口定义。两个包都将现场地图作为独立配置。

```bash
mkdir -p artifacts
python3 scripts/package_a2_console.py --role station --output artifacts/a2-station.zip
python3 scripts/package_a2_console.py --role pc2 --output artifacts/a2-pc2-data.zip
```

打包命令省略 `--role` 时生成 PC2 包。请使用新的输出文件名。在各自目标目录解压相应包并执行 `sha256sum -c SHA256SUMS`。点云和二维栅格配置在上位机。厂商重定位 BIN 和驱动日志位于 PC2 的受控目录。[维护连接与传输](a2-connection.md)

## 将 PC2 的 ODIN1 实时重定位叠加到点云

现场只读联调时，按以下步骤接入 ODIN1 位姿与 PC2 机身状态。

1. **在 PC2 启动并核对 ODIN1 定位观测。** 在设备驱动所属环境按既有流程载入与上位机点云配对的 BIN，启用重定位，保留当次会话的驱动日志。在相同 ROS 域和 RMW 环境运行只读采集器。变量依次指向现场 BIN、当次驱动日志和可写会话目录。[既有重定位流程](development.md#32-pc2-上的-odin1-两点重定位)

   ```bash
   python3 scripts/observe_odin1_pose.py --map-bin "$ODIN1_ACTIVE_BIN" \
     --driver-log "$ODIN1_DRIVER_LOG" --output "$ODIN1_SESSION_DIR/pose.json"
   ```

2. **在 PC2 启动只读数据接口。** 加载与平台状态发布者相同的 ROS 2、`a2_interfaces` 工作区及 ROS 域。若 `/a2/platform/state` 已有发布者，直接运行以下命令。PC2 仅订阅平台状态，ODIN1 的观测器可在另一 ROS 域工作。

   ```bash
   python3 scripts/run_a2_console.py --role pc2 --host 127.0.0.1 --port 8765 \
     --odin1-pose-file "$ODIN1_SESSION_DIR/pose.json"
   ```

   若同时需要启动原厂 SDK 只读状态发布器，在已核对 `A2_SDK_NIC`、`A2_SDK_DOMAIN_ID`、`A2_ROBOT_ID` 并加载对应原厂 Python SDK 后，改用 `A2_ODIN1_POSE_FILE="$ODIN1_SESSION_DIR/pose.json" bash scripts/start_a2_console_readonly.sh`。脚本会启动状态发布器和 PC2 数据接口。

3. **在上位机建立维护通道。** `A2_SSH_TARGET` 指已核对的 PC2 SSH 目标。需要经过 PC3 时按现场配置使用 SSH 跳板。此命令将**上位机的 8766** 接至 **PC2 的 8765**。保留该连接，在另一个终端运行上文的 `station`，浏览器仍打开上位机的 8765。

   ```bash
   ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:8766:127.0.0.1:8765 "$A2_SSH_TARGET"
   ```

4. **核对两项数据。** PC2 的 `GET /api/robot/state` 提供平台状态，`GET /api/odin1/pose?map_sha256=<BIN摘要>` 提供 ODIN1 观测结果。PC2 角色不提供网页、地图下载或运动控制。上位机依据本地清单确认浏览器点云版本，取其配对的 BIN 摘要请求位姿。PC2 检查观测器记录的实际 BIN、系统启动标识和接收时间，按本机单调时钟应用 750 毫秒有效期。浏览器从上位机同源接口取得位姿并在有效期结束时撤下标记。

5. **在 A2 静止时对照读数。** 核对上位机显示的机器人状态、传感器 XYZ 与航向和 PC2 同时刻的观测。切换俯视或旋转地图，检查箭头方向随相机正确投影。断开转发后，静态地图应继续可操作，过期位置应撤下。恢复后须收到新鲜且同图的观测才能重新显示。

## 历史实机记录

2026-11-12 的旧只读包在 PC2 同时运行网页、栅格发布器和原厂状态桥。2026-11-19 在这种部署中验证了 25 组静止位置与航向观测。以下连续定位和运动仲裁资料保留用于回查当时的验证方法。现行双端数据链使用上一节的两个只读接口。[开发进度](implementation-status.md)

## 连续定位联调

在原建图起点停稳后，使用实时点云与惯导建立本轮 `ring-02` 锚定。定位桥要求报告的留出扫描验证通过，报告中的扫描时间戳必须出现在同一运行中 DLIO 的日志，地图文件也须与导出摘要一致。保持该 DLIO 进程连续运行，在 PC2 加载它的 ROS 环境后接入显示。

```bash
python3 scripts/publish_a2_localization.py \
  --anchor-report "$A2_LIVE_ANCHOR_REPORT" --dlio-log "$A2_LIVE_DLIO_LOG" \
  --dlio-pid "$A2_LIVE_DLIO_PID" --map-yaml "$A2_CONSOLE_MAP_YAML" \
  --map-manifest "$A2_CONSOLE_MAP_MANIFEST" --extrinsics "$A2_LIDAR_EXTRINSICS" \
  --output-domain 63
```

定位桥从原厂域读取诊断 DLIO，经独立进程在隔离显示域发布 `/odom` 和 `map → odom → base_footprint`，使用原始扫描时间戳。机身位置和朝向按提供的前雷达外参换算。试验使用原厂结构外参，现场安装偏差和地图绝对精度待量测。源中断、进程重启、坐标冲突或地图不符会使定位停止更新，页面将过期位姿标为无数据。恢复时使用新锚定。[机身定位与验证方法](a2-goal-validation.md)

### 静止波动与终点匹配复测

候选版 DLIO 增加了静止配准保护。连续三帧独立扫描配准及陀螺仪确认近静止后，将主、备候选与独立估计的平移差限制为 2.5 厘米。大偏差时可以尝试用独立估计生成候选，候选须通过现有子地图几何检验。静止证据失效即恢复原运动门槛。补丁默认关闭，在候选构建的 `recalculate` 模式下用 `-p diagnostic/stationary_registration_guard:=true` 启用。[候选构建说明](../third_party/README.md#dlio-静止配准候选补丁)

同一停姿下重新匹配时，用已有 `probe_map_anchor.py` 和取证入口保存本轮扫描及每个拟合候选的结果。保留原始点云、过滤后点云、时间戳、里程计变换和校验摘要。保存文件放在现场受控目录。点云与 IMU 短包仍用于完整 DLIO 回放。[实机录包指南](mapping-capture.md)

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

补充参考需要来自同一地图下已锚定的独立录包。程序通过采样与地图摘要、锚定报告及 DLIO 日志的精确时间戳配对核对来源，并检查参考和查询的采样时间不重叠。补充视角单独按 40 厘米体素参与检索，原地图继续用原分辨率和候选名额。两路候选共同执行原几何门槛、歧义和留出检查。该入口只生成离线报告。

## 实机运动接入

正式站端在实机维护窗口使用两条通道。原 `--pc2-url http://127.0.0.1:8766` 继续承载机身遥测和 ODIN1 位姿。站端另以 `--enable-manual-control --manual-key-file <站端私有文件> --manual-pc2-url http://127.0.0.1:18777` 连接专用维护隧道。该隧道只将本机回环端口转至同一台 PC2 的回环 18777。控制状态查询和命令均在专用通道鉴权。站端核对两路机器人标识、守护和遥测新鲜度。关闭维护连接后，正式遥测可独立刷新。单地址方式仍供既有模拟演练使用。[站端实现](../src/a2_gateway/a2_gateway/station.py) · [机间协议](../src/a2_gateway/a2_gateway/manual_link.py)

PC2 的独立网关使用 `a2-cockpit-manual-gateway.service`，订阅正式采集器已发布的机身话题。它从 systemd 的 `LoadCredential` 读取机间密钥，以服务账号访问受管接收器和停车复位服务各自的套接字。现场可单独启动、停止该网关。[网关单元](../deploy/templates/a2-cockpit-manual-gateway.service) · [网关入口](../scripts/start_a2_manual_gateway.sh)

驾驶舱提示「驾驶舱至 PC2 的控制隧道未连接」时，指的是上位机到 PC2 回环 18777 的 SSH 转发。恢复维护窗口时，先通过 PC2 本机核对网关已启动并监听回环 18777，再建立本地转发。此顺序可以避免站端轮询尚未监听的端口，在维护终端里重复打印连接失败。

开通网页复位时，在已经完成遥控受管部署、A2 已锁存且发令服务已退出的 PC2 上安装同一固定版本中的 `a2-stop-reset.socket` 和 `a2-stop-reset@.service`。将 `a2-cockpit-stop-reset.conf`、`a2-remote-stop-reset-recovery.conf`、`a2-autonomy-stop-reset.conf` 分别登记到对应服务的增补目录，并将 `/usr/local/libexec/a2-stop-reset` 和独立网关入口指向该版脚本。重读 systemd 配置并启动独立网关后，核对停车守护持续运行、锁存仍在，再建立维护隧道。已建立的同一条隧道可用于多次网页复位。[机载复位流程](a2-remote-stop.md#驾驶舱复位)

安装受管入口时，先校验固定版本 PC2 包的 SHA-256 和包内清单，将其解包到 `/opt/a2` 下仅 root 可写的版本目录。确认 `a2-autonomy` 为独立禁登录的系统账号，赋予版本目录内 `scripts/start_a2_manual_receiver.sh` 执行权限。在锁存生效、守护运行、发令服务未运行的状态下执行 `sudo /bin/bash "$release/scripts/manage_a2_manual_entry.sh" install`，其中 `$release` 指该固定版本目录。工具登记启动入口和服务增补，重读 systemd 配置。机身遥控套接字为 `/run/a2-manual/receiver.sock`，实际发令进程随守护原有条件放行后才启动。[安装与移除](../scripts/manage_a2_manual_entry.sh) · [受管服务增补](../deploy/templates/a2-manual-receiver.conf)

退出该部署时，可先停止独立遥控网关，撤下上位机本次的服务增补并重启站端，恢复原只读配置。在同一锁存条件下，使用固定版本的 `manage_a2_manual_entry.sh remove` 移除该版本登记的受管入口。原厂实体 A 键继续由原停车守护处理。

PC2 接收器使用 `scripts/run_a2_manual_receiver.py`，默认后端为 `mock`。选用宇树适配器时，受管入口参数为 `--backend unitree --interface <实测网卡> --socket <机载套接字绝对路径> --gateway-user <PC2网关账号>`。唯一实机发令进程须在系统 `a2-autonomy.service` 控制组中以 `a2-autonomy` 账号运行。自主单元启动条件检查持久锁存和守护 `ready` 标志，适配器在每次向原厂 SDK 发送速度前还会核对进程归属和 `ready`。守护触发 A 键时会撤销 `ready`、持久化锁存、发停并清退整个运动进程组。网页复位期间，机载复位服务以独立状态标志阻止自主单元启动。[受管服务模板](../deploy/templates/a2-autonomy.service) · [遥控接收器](../src/a2_safety/manual_receiver.py)

Unix 套接字采用单行 JSON 请求、单行 JSON 回应，版本为 `schema_version=1`。网关调用 `status`、`acquire`、`velocity`、`stop`。连续点动另用 `action_start` 和 `action_release`。接收器在 `control.source` 返回 `mock` 或 `unitree`。跨账号部署时建议将套接字放在接收器所有、仅属主可写的目录中，例如权限 `2750` 的 `/run/a2-manual/`，组仅供网关和接收器共享。新建套接字权限为 `0660`，双方用内核提供的 `SO_PEERCRED` 核对进程账号。

**原厂手柄优先。** 现场已确认手柄介入操作会使 PC2 的运动指令失效。PC2 上的 `a2-vendor-owner.service` 用独立低权限账号读取实际低层反馈 `rt/lf/lowstate`。PC2 接受年龄小于 0.20 秒的可信回中帧作为手柄放行条件。有效按键、超过既有死区的摇杆输入或反馈异常会撤销遥控。手柄重新回中后需要新许可。实体 A 键继续由独立停车守护负责发停和锁存。手柄无线连接由现场核对，PC2 观测的是低层报文里的手柄输入。[手柄观察器](../scripts/observe_a2_handset_owner.py) · [回中反馈核验](../src/a2_safety/manual_authority.py)

2026-11-23 的 PC2 实机复查中，`GetState` 返回自动恢复开关、运动模式、过程状态和速度档，`CheckMode` 返回机身形式与运控模式。已核实的两项回包均缺少实际发令方字段。底盘厂商需要按完整固件版本确认受支持的控制权反馈或仲裁方式。运动 API 报告的 `1.0.0.1` 用于核对 SDK 服务接口。[本机只读接口](unitree-readonly.md)

在 PC2 运行 `/usr/local/libexec/a2-manual-grant status` 可分别查看手柄回中、采样时刻、当前授权及代号。现场授权需要停车守护放行和可信的回中反馈，在机载管理入口指定方向和刚读取的代号。单次许可还需填写有效秒数，软件上限为 30 秒，并依 PC2 单调时钟到期。撤销、换机、重启、重新布防或回中周期改变后，旧许可失效。运动授权在 PC2 本机单独签发。[本机限时授权](../src/a2_safety/manual_grant.py) · [管理入口](../scripts/a2_manual_grant.py)

连续点动采用本机管理员显式签发的 `--repeated-actions` 许可，须逐项指定方向。驾驶舱仍须启用会话。PC2 在收到一次新的方向按下请求后，按本机单调时钟为该段动作设置 **30 秒**截止，持续发包只能刷新断流看门狗。PC2 收到该段的松键且确认停稳后，才可接受新的起步。一直按住到动作到期时，PC2 会停车，页面提示松键。动作编号和递增的请求序号共同拦截延迟到达的旧指令。主动停止遥控、断流、手柄介入或停车锁存会结束整个就绪会话。[本机仲裁器](../src/a2_safety/maintenance_arbiter.py)

正式接入后，上位机从现场指定的实体键盘读取方向键，独立于浏览器速度包。上位机与 PC2 通过 SSH 回环转发和逐帧双向认证传递键盘证据。PC2 按本机单调时钟计龄，并在接收方向请求及调用 SDK 前核对按压周期、方向和 200 毫秒时效。松开实体键、证据断流或换键使旧会话失效。重连后的新输入须重新确认键盘回中，许可模式决定会话期限。[正式证据服务](../scripts/a2_keyboard_hold_service.py) · [接收器](../src/a2_safety/manual_receiver.py)

正式驾驶舱手动入口采用人工负责净空的模式。管理员在发令接收器启动后指定本轮方向和授权模式，接收器按签发方向拦截越权指令。连续站立停稳、手柄回中且停车守护已布防时才可建立会话。机身反馈来自 SDK 的只读双路订阅。单次许可到期、手柄介入或反馈失效时撤权，旧许可在手柄再次回中后保持失效。仍在运行的旧现场前向方案沿用 `--site-front-confirmed` 的前向 5 秒规则。[受管手动入口](../scripts/start_a2_manual_receiver.sh) · [仲裁规则](../src/a2_safety/maintenance_arbiter.py)

点动仲裁的只读反馈要求三轴合速度不超过 0.010 米/秒，连续覆盖至少 0.50 秒，并通过姿态、IMU、位移和数据有效性检查。仲裁器须收到停车请求之后的新鲜停稳样本才结束等待。全会话撤权后需要重新签发许可，连续点动会话内只需重新按键。实体 A 键停车守护使用独立判据。[只读反馈](../scripts/a2_remote_stop_sdk.py) · [实体 A 键停车](a2-remote-stop.md#分模式确认停稳)

### 持续按住模式与验证

PC2 从新鲜的独立键盘回中样本建立持续按住会话，并绑定首次实体按压的方向和周期。持续运动同时依赖时效为 200 毫秒的键盘证据和 400 毫秒的速度包看门狗。键盘松开、证据过期、方向或周期变化时停车并撤权。故障恢复后，旧会话的指令会被拒绝，重新起步需要回中和新的本地许可。驾驶舱仅接受可信的实体键事件。原厂手柄介入和实体 A 键停车锁存继续独立生效。

该模式已在停车锁存下部署到上位机与 PC2。受管接收器启用 `A2_REQUIRE_KEYBOARD_HOLD=1` 后，由 `A2_ALLOW_CONTINUOUS_HOLD=1` 显式开启持续按住能力。PC2 管理员使用 `--continuous-hold` 签发方向许可时，会话以独立键盘证据和新鲜速度包维持，可跨过 30 秒。普通单次许可及连续点动的每段动作仍采用 30 秒上限。[受管入口](../scripts/start_a2_manual_receiver.sh) · [授权入口](../scripts/a2_manual_grant.py) · [持续按住配置模板](../deploy/templates/a2-autonomy-continuous-hold.conf)

操作员在「待启用」时保持实体方向键松开，点击「启用控制」。等页面显示「控制已启用」再按下实体方向键。前向实机已验证超过 30 秒持续行进、松键判定、撤权及停稳。开发机可在项目根目录运行以下测试，用模拟底盘分别验证松键、键盘证据过期、控制链路断开、发令进程退出和旧会话拒绝。[实机验证记录](implementation-status.md#持续按住超过-30-秒的前向实机验证)

```bash
python3 -m pytest -q -s scripts/test/test_a2_continuous_hold_reconnect.py \
  scripts/test/test_a2_continuous_sender_exit.py
```

## 开发机单元检查

```bash
PYTHONPATH=src/a2_gateway:src/a2_navigation python3 -m unittest discover -s src/a2_gateway/test -v
```

开发机使用合成输入检查网关接口和数据失效处理，现场部署与运动验收另行记录。
