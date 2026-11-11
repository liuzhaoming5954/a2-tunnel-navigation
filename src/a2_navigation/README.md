# a2_navigation

当前提供本机离线目标点规划工具。实机固定路线执行、路线进度和桩前对准由后续导航节点负责。

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
