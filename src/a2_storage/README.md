# a2_storage

状态：`in_progress`。首个可运行切片为纯 Python `SQLiteJournal`；没有 ROS 服务、媒体提交或实机适配。

- 已实现：SQLite 原子事务、跨重启命令去重、快照历史与有序事件。
- 接口：`from a2_storage import SQLiteJournal, RequestConflict, StorageError`。
- `apply(robot_id, request_id, payload, transition)` 在事务中调用纯回调，回调返回 `(snapshot, result, events)`。同 ID 同内容返回原结果，不重执行；同 ID 异内容抛 `RequestConflict`。
- `read_snapshot(robot_id)` 读取最新快照，`events(robot_id)` 返回含 `event_seq`、`request_id` 和 `payload` 的事件列表。
- 快照、命令收据、结果和事件一并提交；回调或提交失败时回滚。SQLite 故障抛 `StorageError`，业务回调异常原样传播。JSON 拒绝非有限数值。
- 每个线程/进程使用独立连接；`BEGIN IMMEDIATE` 串行化同库写入，锁等待上限默认 1 秒。回调不能执行设备、网络或耗时 I/O。
- SQLite 文件数据库使用 WAL 与 FULL 同步，异常断电能力仍需在 A2 实际介质上验证。父目录必须预先准备，不隐式创建现场目录。

本机执行：

```bash
PYTHONPATH=src/a2_storage python3 -m unittest discover -s src/a2_storage/test -v
```

测试仅需 Python 标准库，并覆盖事务末端写入失败、跨连接并发去重、提交前后子进程异常退出、跨重开恢复与损坏记录。此证据不代替真实介质断电验收。

ament_python 元数据用于后续 colcon 集成；维护者身份为明确占位，`LicenseRef-Proprietary` 表示尚未授予开源许可。当前库只用 Python 标准库，未来 ROS 消息与服务接入由 `a2_interfaces` 契约统一。

后续媒体文件提交、检查点业务含义和同步确认水位仍待实现。停车、撤权等动作不能等待此库的事务或重试。参见[架构约定](../../docs/architecture.md)及[开发流程](../../docs/development.md)。
