# a2_safety

状态：`planned`，仅建立职责边界，尚无 ROS 包、节点或可执行入口。

- 职责：健康汇总、运动授权、速度仲裁和防撞配置。
- 计划实现：C++17。
- 依赖边界：依赖a2_interfaces及Collision Monitor；不依赖Web在线。

实现时添加实际需要的 `package.xml`、构建文件、源码与测试；接口和配置变更须同步更新[架构约定](../../docs/architecture.md)及[开发流程](../../docs/development.md)。不通过空节点或零测试结果宣称功能完成。
