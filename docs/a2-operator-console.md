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
| 本体、电量和运动状态 | 上位机转取 PC2 的只读平台状态。原厂发布者没有有效数据时，对应读数显示未知。 |
| ODIN1 点云、二维栅格 | 上位机本地地图。点云面板显示原始文件名、抽样点数和原始点数。 |
| ODIN1 传感器位置及航向 | PC2 同图重定位。圆点对应传感器 IMU 原点，箭头对应传感器 +X 轴在地图 XY 平面内的投影。机身安装外参有待量测。 |
| 示意场景与点位 | 本地合成场景，可从示例任务或告警跳转到关联点位。 |
| 充电仓、任务和告警 | 上位机默认仅显示已接入数据。可在「数据来源」开启已标明来源的本地模拟业务，独立模拟角色默认展示样例。 |

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

3. **在上位机建立维护通道。** `A2_SSH_TARGET` 指已核对的 PC2 SSH 目标。需要经过 PC4 时按现场配置使用 SSH 跳板。此命令将**上位机的 8766** 接至 **PC2 的 8765**。保留该连接，在另一个终端运行上文的 `station`，浏览器仍打开上位机的 8765。

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

原单机维护台为后续试验预留了本地运动仲裁协议，下表为既有接口约定。当前上位机与 PC2 的部署仅交换只读数据，模拟遥控请使用上文的 `simulator` 角色。实机运动需要另行验证机载仲裁、避障、底盘桥接和接收端失联停车。

Unix 套接字使用一问一答的换行 JSON。网关发出 `schema_version: 1`，每次连接传送一个请求，服务端返回带布尔值 `ok` 的单行响应。服务端进程须以网关同一账号或 root 运行。

| `op` | 请求和返回 |
| --- | --- |
| `status` | 返回 `control`，包括 `available`、`armed`、`receiver_stop_verified`、可选 `reason` 及 `limits.linear_mps`、`limits.angular_radps`。 |
| `acquire` | 核实当前本地授权后签发 `session_id` 和 `expires_in_ms`。会话标识使用 16 至 128 位 ASCII 字母、数字、下划线或连字符。 |
| `velocity` | 接收 `session_id`、递增的 `sequence`、机身前向 `linear_mps` 和绕 Z 轴的 `angular_radps`。服务端校验时效、控制权和限值后受理。 |
| `stop` | 接收 `session_id`，撤销对应会话并触发停止。现场独立急停由原厂通道负责。 |

仲裁服务不可达、拒绝请求或未确认接收端停车时，界面显示控制不可用。操作页面与遥控会话的断开处理属于维护模式，离线自主任务继续使用其独立授权。[首次受控运动与停车验证](a2-first-motion-test.md)

## 开发机单元检查

```bash
PYTHONPATH=src/a2_gateway:src/a2_navigation python3 -m unittest discover -s src/a2_gateway/test -v
```

开发机使用合成输入检查网关接口和数据失效处理，现场部署与运动验收另行记录。
