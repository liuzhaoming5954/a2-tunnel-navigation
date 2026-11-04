# a2_localization

状态：`planned`，仅建立职责边界，尚无 ROS 包、节点或可执行入口。

- 职责：点云/时间/位姿适配、地标质量与定位健康。
- 计划实现：C++17。
- 依赖边界：依赖a2_interfaces及锁定的DLIO/RTAB-Map/AprilTag。

实现时添加实际需要的 `package.xml`、构建文件、源码与测试；接口和配置变更须同步更新[架构约定](../../docs/architecture.md)及[开发流程](../../docs/development.md)。不通过空节点或零测试结果宣称功能完成。
