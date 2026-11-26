# A2 地图目标点验证

本机已把闭环候选地图导出为二维占据栅格，并用录包位置验证目标点规划。本页说明如何复现离线结果，以及从地图点选进入实机试验所需的证据。真实地图、候选坐标和原始日志存放在受控试验目录。

## 1. 导出地图与验证目标

在开发机上使用已保存并校验的闭环数据库、对应的建图日志和有序回放扫描清单。导出工具需要 Python 3 和 NumPy。指定在建图记录中核实的传感器向上轴，地图 Z 轴为重力反向。三个输入必须来自同一建图结果。

```bash
python3 scripts/export_navigation_map.py \
  --database "$A2_CLOSED_DB" --expected-database-sha256 "$A2_CLOSED_DB_SHA256" \
  --mapping-log "$A2_MAPPING_LOG" --input-ledger "$A2_ORDERED_SCAN_LEDGER" \
  --sensor-up y --output-dir "$A2_CANDIDATE_GRID"
```

`A2_ORDERED_SCAN_LEDGER` 是逐帧 JSONL，提供 `input`、`header_ns`、`audit_scan_end_ns`、`receipt_ns`。导出时将日志中的已发布扫描和数据库关键帧关联到这些原始时间。对于所测的前雷达参考系，正 Y 为向上轴。其他数据按自身外参选择。输出 `map.yaml`、`map.pgm`、`keyframes.json` 和 `manifest.json`，其中清单记录输入和输出文件的 SHA-256。

地图区分占据、已观测空闲和未知。每个空闲格须同时具备地面回波和障碍高度范围内的扫描视线证据。占据证据优先，未观测区域保留未知。离线规划使用指定的圆形净空半径，拒绝从未知格、占据格或净空不足的位置起步或到达。经由点可约束经过顺序。[离线规划工具](../src/a2_navigation/README.md)

```bash
python3 scripts/plan_navigation_goal.py \
  --map "$A2_CANDIDATE_GRID/map.yaml" \
  --start "$A2_TEST_START_X" "$A2_TEST_START_Y" \
  --goal "$A2_TEST_GOAL_X" "$A2_TEST_GOAL_Y" \
  --footprint-radius-m "$A2_TEST_RADIUS_M" --output "$A2_PLAN_REPORT"
```

本轮在 0.20 米分辨率的候选图上，用录包中两个通道位置得到约 1.75 米的路径。原停稳起点在本轮试算半径下净空不足，未知目标和不连通区段也被拒绝。试算半径属于离线测试参数。实际外廓、地面落差、现场障碍、允许通行区域和地图尺度由现场核验。

## 2. 实机停稳时的定位与 ROS 检查

1. 在维护窗口确认目标 Orin 的 ROS 2、RMW、域和机内网卡。核对候选地图文件、配置及代码版本，再加载对应的目标环境。
2. 在实际的地图与定位节点运行后，确认占据栅格及里程计话题、端点 QoS 和坐标系。使用实际名称调用只读观察工具。时间上限按本轮确定的诊断条件填入。
3. 核对 `map → odom → base_footprint` 在里程计采样时刻可查询。把显示位置和机头方向与现场已知位置对照，记录是否发生跳变、错锁或定位拒绝。

```bash
python3 scripts/observe_navigation_inputs.py \
  --map-topic "$A2_MAP_TOPIC" --odom-topic "$A2_ODOM_TOPIC" \
  --map-frame "$A2_MAP_FRAME" --odom-frame "$A2_ODOM_FRAME" --base-frame "$A2_BASE_FRAME" \
  --max-pose-age-s "$A2_POSE_AGE_LIMIT_S" --output "$A2_INPUT_REPORT"
```

观察工具输出地图元数据、同时间 TF 配对数、最近位姿年龄和失败原因。启动等待与正式采样分别计时，`--startup-timeout-s` 设置首批地图和里程计的等待上限。定位可信度由定位节点与独立现场观测确认。可用 `--map-durability volatile` 或 `--map-reliability best-effort` 匹配实际发布者配置。ROS 2 Jazzy 合成测试已覆盖正常链路、缺失 TF 和里程计过期。目标机运行和实机坐标关系待核实。

## 3. 首次实机目标点试验

| 核验项 | 进入实机动作前的证据 |
| --- | --- |
| 地图和起终点 | 版本、坐标、现场可识别位置、机身实际外廓和净空一致。起终点及沿线均通过现场确认。 |
| 定位与导航执行 | 目标机上具备连续可信定位、正确 TF、路线跟踪与障碍监控，定位异常会撤销运动许可。 |
| 控制与停车 | 已核实原厂控制权、独立人工停车和接收端失联停车。完成唯一指令通道与受控动作验证。 |
| 试验条件 | 现场监护人与操作员确认目标、速度、方向、范围、停车边界和终止条件，并记录本地新授权。 |

条件齐备后，从可稳定定位且经量测有足够净空的地点开始，按已确认的限速和范围发起一次指定目标点试验。记录到点位置、停稳、定位健康、障碍反馈和控制权。实机运动用例、停稳证据与失败处置见[首次受控运动与停车验证](a2-first-motion-test.md)。
