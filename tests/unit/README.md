# 单元与核心回归测试

首个增量的测试随包放置：

- [任务状态、授权与恢复](../../src/a2_mission/test/test_controller.py)
- [SQLite journal](../../src/a2_storage/test/test_journal.py)

在根 README 中提供本机命令；CI 使用相同标准库 unittest，无需接入 ROS、网络或 A2。测试包括真实临时数据库、事务故障和子进程退出；真实存储介质断电、底盘停车与导航仍按实机验收流程验证。
