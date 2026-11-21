# a2_navigation

当前提供离线目标点规划、直线指令预演及 PC2 单点导航接入。单点下发、反馈和取消已在开发机验证，实机到点待测。

机载单点导航入口将上位机目标交给 Nav2 的 `NavigateToPose`。动作收据与 ROS 客户端位于 `a2_navigation/single_goal.py` 和 `a2_navigation/nav2_goal_client.py`，可选启动文件位于 `scripts/pc2_single_goal_nav2.launch.py`。按[PC2 单点导航](../../docs/pc2-single-goal.md)配置导航域、地图、机身定位与上位机。

## 离线规划

输入为 ROS trinary 地图的 YAML 与 8 位 PGM。地图原点为 `[x, y, yaw]`，位置用 `map` 坐标系下的米表示，角度用弧度表示。常规 YAML 需要 Python 的 PyYAML，采用 JSON 语法的地图描述可由标准库读取。[Nav2 地图读取实现](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp)

在工程根目录执行。坐标和半径请按待验证地图和机器人外廓填写，下例为合成坐标。

```bash
python3 scripts/plan_navigation_goal.py \
  --map path/to/map.yaml --start 1.2 1.4 --goal 2.3 3.1 \
  --footprint-radius-m 0.4 --output path/to/new-plan.json
```

工具以 A* 搜索栅格路径，对未知、占据和地图外边界留出圆形外廓净空。斜向跨格同时检查相邻两个正向格。可用重复的 `--via X Y` 指定依次经过的地图位置。规划输出包含地图文件哈希、输入点、格路径、米制路径和长度。输入点不在可通行区域时报告原因，起终点互不连通时返回 `unreachable`。

结果用于检查地图、目标点和静态路径。经由点约束通过顺序，现场允许通行的区域、动态障碍、定位连续性、实际底盘外廓和停车能力仍需按对应阶段验证。工具以文件为输入输出，未接入 ROS 运动链路。实机流程见[首次受控运动与停车验证](../../docs/a2-first-motion-test.md)。

```bash
PYTHONPATH=src/a2_navigation python3 -m unittest discover -s src/a2_navigation/test -v
```

## 本机指令预演

`preview_navigation_goal.py` 读取直线路径、地图导出清单和指定的雷达到机身外参。外参 JSON 提供 `T_base_from_lidar` 4×4 矩阵及 `source`。脚本校验路线和地图来源一致，保留采样时间，按机身在地图上的平面位置和朝向计算前向速度、转向速度及停车原因。预演报告只写入 JSON。

```bash
python3 scripts/preview_navigation_goal.py \
  --plan path/to/goal-plan.json --map-manifest path/to/manifest.json \
  --extrinsics path/to/base-from-lidar.json --simulate \
  --output path/to/new-preview.json
```

`--simulate` 运行到点与位姿断流两个场景。使用 `--anchor-report path/to/anchor-report.json` 时，按保存的机载匹配报告中的原始位姿和时刻回放。到点或故障后，该次预演保持零速。默认速度和容差是本机测试参数，实机参数待验证。正式路线执行采用项目设计的 Nav2，实机试验见[目标点验证流程](../../docs/a2-goal-validation.md)。
