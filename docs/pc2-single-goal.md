# PC2 单点导航

从正式驾驶舱进入「巡检作业 → 任务管理」，即可看到 PC2 单点导航表单。目标 X、Y 以米为单位，朝向以度为单位，从地图 +X 向 +Y 为正。站端只读时，输入和下发操作禁用。上位机与 PC2 均启用 `--enable-single-goal` 且导航就绪后，可下发目标。PC2 将目标交给 Nav2 的 `NavigateToPose` 动作，回传目标编号、动作状态、剩余距离和最终结果。[打开单点导航](http://127.0.0.1:8765/#/tasks)

## 开发机验证记录

2026-11-21 在 ROS 2 Jazzy 的本机回环分别从源码、从已解压的上位机和 PC2 离线包运行正式网关。每轮先看到 `odin1_waiting`，注入合成地图、定位、机身状态、TF 和里程计后转为就绪，随后才提交一个地图目标。两个入口均收到同一目标的反馈与成功终态。另一次回环用例对执行中的目标发起取消并收到 `canceled`。独立的机身位姿桥测试使用合成安装外参，核对了导航域的 TF、里程计和测量时间。目标数据、重试结果和复测命令见[开发进度](implementation-status.md#pc2-单点导航开发机验证)和[开发机回环复测](development.md#33-巡检与单点导航的开发机复测)。

## 现场需要的输入

| 输入 | 用途 |
| --- | --- |
| 上位机点云清单和 PC2 实际加载的 ODIN1 BIN | 清单的 `relocMapSha256`、PC2 文件摘要及本轮设备定位观测必须一致。 |
| 与 BIN 对齐的导航栅格 YAML 和 PGM | 栅格必须与上位机目标共用地图原点、朝向和尺度。PC2 比对 ROS 发布的栅格几何及所有格值，并在下发前排除图外、未知和占据格。现场二维图的配准需要独立核对。 |
| ODIN1 IMU 到 A2 机身的实测安装外参 | JSON 提供 `source`、`base_frame` 以及 `T_base_from_imu`。其 `translation_m` 是 IMU 原点在机身系中的三维位置，`quaternion_xyzw` 把 IMU 的轴转到机身系。外参文件的机身帧必须与导航配置一致。 |
| PC2 的 Humble Nav2、现场导航参数、障碍观测及平台状态 | 核对机身外廓、限速、障碍源、控制器和代价地图。这里的设备域只接 ODIN1；平台状态、机身位姿和 Nav2 使用同一导航域。机身、地图和 TF 各由一个来源发布。 |

## 部署后先做静止联调

解压各自的离线包，执行 `sha256sum -c SHA256SUMS`。PC2 的 ROS 终端加载 Humble 和本机已经构建的 `a2_interfaces`。下列大写环境变量表示现场的真实路径、ROS 域或服务器地址。分别在不同终端运行常驻进程。

1. 按既有流程使 ODIN1 加载选定 BIN，进入重定位模式 2。在驱动域运行本版本的 `scripts/observe_odin1_pose.py`，参数为 `--map-bin`、`--driver-log` 和 `--output`。记录的 `ODIN_POSE` JSON 包含本次观测的域号及两项三维变换。[ODIN1 重定位接入](a2-operator-console.md#将-pc2-的-odin1-实时重定位叠加到点云)
2. 在 PC2 将定位投影到导航域。机身桥按实测外参换算，输出 `map → odin_nav_odom → base_link` 和 `/odom`。相邻设备时刻的局部位姿用于估计平面速度。首次启动或数据中断后，等到两个连续有效样本才发布。时间戳根据本机接收时刻投影到 ROS 系统时钟。设备域与导航域使用不同编号，防止厂商 TF 与导航 TF 形成环。

   ```bash
   python3 scripts/publish_odin1_navigation_pose.py \
     --pose-file "$ODIN_POSE" --map-sha256 "$ODIN_BIN_SHA256" \
     --extrinsics "$A2_ODIN_EXTRINSICS" --output-domain "$A2_NAV_DOMAIN" --base-frame base_link
   ```

3. 在导航域启动已配置的原厂状态发布器，使 `/a2/platform/state` 有新鲜本体数据。使用本轮入口启动 Nav2。现场参数按 Humble `navigation_launch.py` 的节点键编写，使用 `map`、`odin_nav_odom`、`base_link` 三个坐标系。若导航域已有对应的 `/map` 发布器，省略 `map_yaml`。如包含该参数，启动入口会一并激活 Nav2 的地图服务器。

   ```bash
   export ROS_DOMAIN_ID="$A2_NAV_DOMAIN"
   ros2 launch "$PWD/scripts/pc2_single_goal_nav2.launch.py" \
     params_file:="$PC2_NAV2_PARAMS" map_yaml:="$PC2_GRID"
   ```

4. 在同一导航域启动 PC2 网关。若已启动旧只读网关，使用本轮单点服务替换它或选用另一个回环端口，在上位机配置相应的 SSH 转发。目标收据保存在 PC2 的 SQLite 文件中，后续重启继续使用同一份收据库。

   ```bash
   export ROS_DOMAIN_ID="$A2_NAV_DOMAIN"
   python3 scripts/run_a2_console.py --role pc2 --port 8765 --enable-single-goal \
     --odin1-pose-file "$ODIN_POSE" --goal-map-bin "$ODIN_BIN" --goal-grid-yaml "$PC2_GRID" \
     --odom-frame odin_nav_odom --base-frame base_link \
     --goal-action /a2_single_goal/navigate_to_pose
   ```

5. 上位机通过既有 SSH 转发访问 PC2 回环服务。启动上位机并打开「任务管理」。先查看“就绪”及原因，此时页面没有下发目标。“就绪”覆盖地图、机身定位、平台在线和动作服务器可用性。底盘控制权和停车能力另按现场流程验证。可用 `ros2 action info /a2_single_goal/navigate_to_pose` 在导航域核对动作服务器。

   ```bash
   python3 scripts/run_a2_console.py --role station --port 8765 --enable-single-goal \
     --pc2-url http://127.0.0.1:8766 --odin1-map-dir "$ODIN1_DISPLAY_DIR"
   ```

## 下发、取消与验收

1. 在 PC2 记录固定提交、离线包校验、系统架构、Humble/Nav2 和 ODIN1 驱动版本，核对设备加载的 BIN、二维栅格与机身 TF 的现场对齐情况。
2. 保持候选速度与底盘输出断开，给定一个已知空闲格内的目标，观察规划、障碍输入和 `/a2/navigation/cmd_vel_candidate`，再按目标 ID 取消并核对回执。
3. 按现场流程接通仲裁、防撞和唯一底盘桥，取得本轮运动授权后验证控制权、独立人工停车与失联停车。通过后才做低速单点试验，记录目标和实测终点的 X、Y、航向，以及实际停稳和接管结果。[实机控制与停车验证](a2-first-motion-test.md)

- 请求受理后，上位机显示目标 ID、导航反馈与剩余距离。相同 ID 和内容重试会取回已有收据。回复丢失时先查询原目标，页面刷新不会自动发起下一次移动。点击取消只针对当前目标 ID。失去有效定位或机身状态时，PC2 请求撤销执行中的动作。
- `succeeded` 表示 Nav2 返回成功，`canceled` 表示 Nav2 返回取消终态。若进程重启时目标结果仍未知，PC2 保留未决状态并阻止派发新目标，先核对 Nav2 的动作状态和现场机身状态。
- 站端接口为 `GET /api/navigation/goal`（就绪和状态）、`POST /api/navigation/goal`（目标 ID、地图摘要、`map` 帧、X/Y 和弧度航向）以及 `POST /api/navigation/goal/cancel`（目标 ID）。地图或位姿缺失时可保留 HTTP、ROS 和 UI 的反馈，用于定位未就绪的环节。
