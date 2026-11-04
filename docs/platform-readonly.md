# ROS 公共接口与平台只读演示

本增量提供 `a2_interfaces` 的观察消息/查询服务，以及 C++17 `a2_platform/readonly_node`。默认源为合成 `mock`，SDK 编译选项默认关闭。任务核心仍为独立业务库，本轮没有实现任务 ROS 服务、运动许可、底盘控制或完整 bringup。

## 1. 本机构建

当前验证环境为本机 x86_64 Ubuntu 24.04 / ROS 2 Jazzy。项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select a2_interfaces a2_platform \
  --cmake-args -DA2_ENABLE_UNITREE_SDK=OFF -DBUILD_TESTING=ON
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
```

域号 42 只是本机测试约定，使用前确认未与其他 ROS 作业冲突。ROS 发行版、编译器、SDK 与目标 ARM64 的最终组合仍由 M0 核实；本机结果不代表 Humble 或 A2 已验收。

## 2. 自动演示与手动观察

已加载环境后执行有限时检查：

```bash
python3 scripts/readonly_smoke.py
```

脚本只启动 mock：先观察有效状态，约三秒后模拟断流，再验证 `STALE`、样本身份不刷新、年龄增加、错误诊断和只读查询，最后清理进程。没有新鲜样本或服务不符合预期时返回非零；不能把全部陈旧数据当作测试成功。

手动观察时，在第一个终端启动：

```bash
ros2 run a2_platform readonly_node --ros-args \
  --params-file config/examples/platform-readonly.yaml
```

第二个终端加载同一 underlay/overlay，设置相同本地域，再查询：

```bash
ros2 topic echo /a2/platform/state
# 或单独查看诊断/请求只读快照
ros2 topic echo /diagnostics
ros2 service call /a2/platform/get_state a2_interfaces/srv/GetPlatformState '{}'
```

`topic echo` 持续运行，用 Ctrl+C 结束后再执行下一条。配置文件必须在项目根目录引用；手动节点与自动 smoke 分别运行，避免同名查询服务冲突。需要演示断流时在节点参数中设 `mock_drop_after_s:=3.0`，完整退出并重启节点后生效。

## 3. 数据语义与边界

| 接口 | 行为 |
|---|---|
| `/a2/platform/state` | 发布 `PlatformState`，含来源、session、序号、接收时刻、年龄、各字段有效性及原因。可靠 QoS、volatile、depth 10。 |
| `/diagnostics` | `a2_platform/readonly` 诊断；模拟有效数据为 OK，无样本、陈旧或字段不足为 ERROR；始终注明来源与不具备运动能力。 |
| `/a2/platform/get_state` | 空请求，重新计算缓存年龄。`available` 只表示有过原始样本；陈旧状态仍 available=true、valid=false。查询不从源获取新样本。 |

`header.stamp` 是发布/查询时的 ROS 时间，不声称是原厂采集时刻。`received_mono_ns` 属于源进程的 steady clock，消费者不能与不相关时钟直接比较。重复发布不会更新样本序号或接收时间；即使 ROS 时间暂停/跳变，新鲜度仍按 steady clock 计算。

`valid` 要求新鲜且电量、平面速度、mode 均可用；它不是定位有效、静止、原厂控制权已释放或运动许可。单字段 flag 只描述样本字段，陈旧时可能仍为 true，使用时必须同时检查整体有效性/年龄。

平面速度只包含 `linear.x`、`linear.y` 和 `angular.z`，其他未测分量为 NaN；缺失电量同样为 NaN 并清除有效标记。mode 的有效标记只表示字段存在，不解释厂商枚举的控制含义。

source 和配置参数创建时只读，未知 source、未编译 SDK 却选择 unitree 都会退出报错，不静默退到 mock。真实源始终 `simulated=false`；mock 为 true；两者均 `motion_capable=false`。该节点没有速度订阅、运动服务或运动客户端。查询机器人状态与授权机器人运动是独立能力。

## 4. 可选 A2 接收适配

官方依据、字段限制与生命周期见 [SDK 只读说明](unitree-readonly.md)。当前只实现 `rt/lf/sportmodestate` 的 mode 与接收时序；电量未提供、速度机身坐标未验证，因此真实源当前整体 `valid=false` 是预期结果，不能冒充完整健康输入。

真实 SDK 适配源本轮未编译、未链接、未连接实机。完成 M0 核查、取得许可和锁定安装后，才在独立构建目录启用 `A2_ENABLE_UNITREE_SDK=ON`，提供正确的 `unitree_sdk2_DIR`。缺 SDK 时构建必须失败。

实机维护窗口内需显式提供 `source=unitree`、真实 `robot_id`、已核实的 `network_interface` 和 `unitree_domain_id`。ROS domain 与厂商 DDS domain 分开管理；本机 localhost 限制不能替代机内网卡配置。节点不主动查询网络、下载 SDK 或自动探测机器人。

## 5. 测试与后续工作

```bash
colcon test --packages-select a2_platform --return-code-on-test-failure
colcon test-result --verbose
```

本轮本机四包构建、14 项 C++ 用例及实际 ROS mock 通信检查通过；原 36 项 Python 核心测试也通过。

当前 C++ 用例覆盖无数据、陈旧边界、非法字段、接收时间不刷新及 mock 断流；有限时 ROS 检查覆盖真实序列化、发布/订阅、诊断和服务。新增 `ros-readonly` CI 工作流使用不含 SDK 的 Jazzy 环境执行这些检查，远端运行结果以实际 GitHub 记录为准。

下一步根据实机只读录包与原厂字段补齐电池/速度映射、时钟域和控制权观察，再对接健康监控与任务适配。参数 100/500ms 和模拟电量仅是演示值，不能直接作为实机准入指标。
