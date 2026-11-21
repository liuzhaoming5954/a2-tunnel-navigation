# a2_gateway

巡检驾驶舱部署在上位机。上位机持有 Vue 3 页面、ODIN1 展示点云以及可选的二维栅格。PC2 提供机器人状态和 ODIN1 实时传感器位姿。上位机利用本地点云与厂商 BIN 的配对摘要校验地图身份，浏览器将有效的位置和航向叠加到点云上。数据过期或缺失时撤下标记，地图仍可操作。

| 角色 | 启动方式 | 作用 |
| --- | --- | --- |
| 上位机 | 构建前端后运行 `python3 scripts/run_a2_console.py --role station`，省略角色时亦为 `station` | 在本机 8765 提供驾驶舱，默认经本机 8766 读取转发的 PC2 数据。`--odin1-map-dir` 指定展示点云，`--map-yaml` 可选。 |
| PC2 | 加载平台 ROS 环境后运行 `python3 scripts/run_a2_console.py --role pc2 --odin1-pose-file "$ODIN1_SESSION_DIR/pose.json"` | 在 PC2 的 8765 受理只读请求。两个数据入口为 `GET /api/robot/state` 与 `GET /api/odin1/pose?map_sha256=<BIN摘要>`。 |
| 开发机离线模拟 | 按需运行 `python3 scripts/run_a2_console.py --role simulator --port 8775` | 复用同一前端，在「任务管理」演练两点巡检，也可单独演练维护遥控。[两点模拟巡检](../../docs/a2-operator-console.md#两点模拟巡检) |

正式入口是本机 8765 端口的巡检驾驶舱。「巡检作业 → 任务管理」可查看地图目标表单。站端只读时，输入和下发操作禁用。执行单点导航需在上位机和 PC2 两端启用 `--enable-single-goal` 并满足导航就绪条件。[打开驾驶舱](http://127.0.0.1:8765/#/cockpit) · [单点导航部署和验收](../../docs/pc2-single-goal.md)

程序入口位于 `scripts/run_a2_console.py`，前端源码位于 `web/`，HTTP 服务位于 `a2_gateway/`。开发机打包时分别使用 `scripts/package_a2_console.py --role station` 和 `--role pc2` 并指定 `--output`。现场地图单独配置。两点模拟巡检和 PC2 单点导航的开发机验证、实机联调记录见[开发进度](../../docs/implementation-status.md#开发机两点模拟巡检)。[部署和启动命令](../../docs/a2-operator-console.md) · [位姿接入与实机验收](../../docs/a2-operator-console.md#将-pc2-的-odin1-实时重定位叠加到点云)
