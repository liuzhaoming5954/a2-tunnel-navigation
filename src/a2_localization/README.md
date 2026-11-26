# a2_localization

状态为开发中。Python 几何模块提供机身平面位姿换算，支持本机回放和 PC2 的只读联调。

- 职责：点云/时间/位姿适配、地标质量与定位健康。
- DLIO 定位桥和锚定取证工具用于现有诊断数据链。ODIN1 集成采用传感器内置建图和重定位，按 SDK 输出适配机身位姿。
- `body_pose_from_lidar` 接收地图下的前雷达三维位姿与明确提供的雷达至机身外参，输出地图下机身的平面坐标、朝向和原始纳秒时间戳。输入矩阵须为有限的刚体变换。

只读定位桥从 `scripts/publish_a2_localization.py` 启动。取证工具 `scripts/record_a2_anchor_evidence.py` 使用 NumPy 和 `--probe` 指定的本地诊断脚本。使用 `--help` 查看输入与参数。实机外参与 TF 发布职责见[架构约定](../../docs/architecture.md)。
