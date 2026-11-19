# a2_gateway

当前提供巡检工作台的 Vue 3 界面和 Python 网关。默认首页为巡检驾驶舱，地图区默认打开 ODIN1 实测静态点云，也可切换到二维栅格或合成示意场景。驾驶舱标明遥测、地图、充电仓、任务和告警各自的数据来源。维护遥控保留独立入口。ROS 2 模式读取实机遥测，维护控制支持连接独立运动仲裁服务。

前端源码位于 `web/`，网关位于 `a2_gateway/`。从工程根目录运行 `python3 scripts/run_a2_console.py` 启动已构建的界面。完整命令、ROS 输入和仲裁协议见 [A2 上位机操作说明](../../docs/a2-operator-console.md)。

PC2 的 ODIN1 重定位可在实测点云上叠加传感器原点和航向。以 `scripts/observe_odin1_pose.py` 生成同机位姿快照，将 `--odin1-pose-file` 传给网关。地图匹配和时效检查均通过后才显示实时标记。[ODIN1 传感器只读接入](../../docs/a2-operator-console.md#将-pc2-的-odin1-实时重定位叠加到点云)
