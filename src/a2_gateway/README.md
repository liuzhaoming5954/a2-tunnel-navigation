# a2_gateway

状态：`planned`，仅建立职责边界，尚无 ROS 包、节点或可执行入口。

- 职责：维护期WSS/HTTPS连接、命令映射和成果同步。
- 计划实现：Python 3。
- 依赖边界：依赖a2_interfaces；无任意ROS透传或直接SDK调用。

实现时添加实际需要的 `package.xml`、构建文件、源码与测试；接口和配置变更须同步更新[架构约定](../../docs/architecture.md)及[开发流程](../../docs/development.md)。不通过空节点或零测试结果宣称功能完成。
