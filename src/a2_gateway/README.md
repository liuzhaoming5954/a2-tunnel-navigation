# a2_gateway

巡检驾驶舱部署在上位机。上位机持有 Vue 3 页面、ODIN1 展示点云以及可选的二维栅格。PC2 提供机器人状态和 ODIN1 实时传感器位姿。上位机利用本地点云与厂商 BIN 的配对摘要校验地图身份，浏览器将有效的位置和航向叠加到点云上。数据过期或缺失时撤下标记，地图仍可操作。

| 角色 | 启动方式 | 作用 |
| --- | --- | --- |
| 上位机 | 构建前端后运行 `python3 scripts/run_a2_console.py --role station`，省略角色时亦为 `station` | 在本机 8765 提供驾驶舱，默认经本机 8766 读取转发的 PC2 数据。`--odin1-map-dir` 指定展示点云，`--map-yaml` 可选。 |
| PC2 | 加载平台 ROS 环境后运行 `python3 scripts/run_a2_console.py --role pc2 --odin1-pose-file "$ODIN1_SESSION_DIR/pose.json"` | 在 PC2 的 8765 受理只读请求。两个数据入口为 `GET /api/robot/state` 与 `GET /api/odin1/pose?map_sha256=<BIN摘要>`。 |
| 离线模拟 | `python3 scripts/run_a2_console.py --role simulator --port 8775` | 在「任务管理」下发两个点，查看同一模拟机身的行进、停稳、停留和完成记录。也可单独演练维护遥控。[两点模拟巡检](../../docs/a2-operator-console.md#两点模拟巡检) |

单点导航需在上位机和 PC2 两端启用 `--enable-single-goal`。上位机在任务页下发地图目标，PC2 核对导航输入后提交动作并回传反馈、取消或完成结果。[单点导航部署和验收](../../docs/pc2-single-goal.md)

程序入口位于 `scripts/run_a2_console.py`，前端源码位于 `web/`，HTTP 服务位于 `a2_gateway/`。开发机打包时分别使用 `scripts/package_a2_console.py --role station` 和 `--role pc2` 并指定 `--output`。现场地图单独配置。2026-11-21 已在开发机验证两点模拟巡检与双端单点导航接口，双机部署和底盘到点待现场验证。[部署和启动命令](../../docs/a2-operator-console.md) · [位姿接入与实机验收](../../docs/a2-operator-console.md#将-pc2-的-odin1-实时重定位叠加到点云)
