# a2_gateway

当前提供维护上位机的 Vue 3 界面和 Python 网关。模拟模式可运行完整的方向操作、状态、占据栅格和位姿显示。ROS 2 模式读取实机遥测，并可连接独立运动仲裁服务。

前端源码位于 `web/`，网关位于 `a2_gateway/`。从工程根目录运行 `python3 scripts/run_a2_console.py` 启动已构建的界面。完整命令、ROS 输入和仲裁协议见 [A2 上位机操作说明](../../docs/a2-operator-console.md)。
