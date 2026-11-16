# a2_safety

已实现维护运动仲裁的 Python 原型 `maintenance_arbiter.py`。它根据注入的权属、授权、防撞和停稳证据管理独占会话，在条件失效后撤权，并区分停车请求受理和实际停稳反馈。机载证据输入、Unix 套接字服务、底盘输出和 ROS 运行组合待集成。

- 职责：健康汇总、运动授权、速度仲裁和防撞配置。
- 计划实现：C++17。
- 依赖边界：依赖a2_interfaces及Collision Monitor；不依赖Web在线。

实现时添加实际需要的 `package.xml`、构建文件、源码与测试；接口和配置变更须同步更新[架构约定](../../docs/architecture.md)及[开发流程](../../docs/development.md)。不通过空节点或零测试结果宣称功能完成。

```bash
PYTHONPATH=src/a2_safety python3 -m unittest discover -s src/a2_safety/test -v
```
