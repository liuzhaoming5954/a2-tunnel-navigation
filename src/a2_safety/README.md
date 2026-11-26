# a2_safety

维护遥控由 PC2 的 [maintenance_arbiter.py](maintenance_arbiter.py) 管理独占会话、控制权、新鲜度、方向净空、限速和停车反馈。[manual_receiver.py](manual_receiver.py) 提供机身坐标系的模拟和宇树后端。[接收器入口](../../scripts/run_a2_manual_receiver.py) 在 Unix 套接字接收网关请求，PC2 的独立看门狗检测空闲保活和运动方向包的超时。会话到期或失联后，旧令牌和迟到的速度包均不能恢复运动。[模拟演练与参数](../../docs/a2-operator-console.md#机身遥控演练)

模拟底盘将受理的速度和停车事件写入日志，适合在开发机或真实 PC2 的独立临时进程中验证。实机适配器只接受 `a2-autonomy.service` 控制组中的非特权发令进程，核对独立停车守护的 `ready` 和本机安全监护快照。正式维护入口根据现场监督模式核对新鲜的手柄回中、站立停稳和本机方向授权。持续按住模式另外核对独立键盘证据，并由运动请求看门狗监督控制链路。仲裁器区分停车请求的底盘回执与机身反馈确认的实际停稳。[实机运动接入](../../docs/a2-operator-console.md#实机运动接入)

PC2 的独立停车守护由 [a2_remote_stop.py](../../scripts/a2_remote_stop.py) 实现，与受管发令进程分开运行。`observe` 用于静止识别原厂按键，`guard` 响应实体 A 键并持久锁存，`reset` 仅供现场人员按既有复位流程操作。驾驶舱键盘 A 用于左转。守护组合机身速度、IMU 和关节反馈，按站立或下趴模式确认停稳。遥控无线断开后机身是否仍发布缓存按键值仍待现场核验。[守护部署与验收](../../docs/a2-remote-stop.md)

健康汇总、ROS 安全集成及 Collision Monitor 联调仍按[架构约定](../../docs/architecture.md)和[开发流程](../../docs/development.md)推进。以下命令在开发机用合成输入验证接收器、仲裁器和许可状态机。

```bash
PYTHONPATH=src/a2_safety python3 -m unittest discover -s src/a2_safety/test -v
```
