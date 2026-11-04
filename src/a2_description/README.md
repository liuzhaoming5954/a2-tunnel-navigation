# a2_description

状态：`planned`，仅建立职责边界，尚无 ROS 包、节点或可执行入口。

- 职责：机身模型、传感器外参和唯一静态TF发布配置。
- 计划实现：URDF/Xacro/config。
- 依赖边界：依赖标准模型与TF工具；实际标定保存在现场配置。

实现时添加实际需要的 `package.xml`、构建文件、源码与测试；接口和配置变更须同步更新[架构约定](../../docs/architecture.md)及[开发流程](../../docs/development.md)。不通过空节点或零测试结果宣称功能完成。
