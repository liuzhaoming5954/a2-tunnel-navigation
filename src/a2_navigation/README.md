# a2_navigation

状态：`planned`，仅建立职责边界，尚无 ROS 包、节点或可执行入口。

- 职责：固定路线执行、路线进度和桩前对准。
- 计划实现：C++17。
- 依赖边界：依赖a2_interfaces及Nav2动作接口；不直接调用SDK。

实现时添加实际需要的 `package.xml`、构建文件、源码与测试；接口和配置变更须同步更新[架构约定](../../docs/architecture.md)及[开发流程](../../docs/development.md)。不通过空节点或零测试结果宣称功能完成。
