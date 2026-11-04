# 第三方依赖

`dependencies.repos` 是 vcstool 工作区清单，当前为空，表示依赖尚未按 A2 实机环境锁定。此文件采用 JSON 语法（也是合法 YAML），不代表导航栈已可构建。

另有 [mapping-candidate.repos](mapping-candidate.repos) 固定 DLIO 的本机候选提交，已在 Jazzy/x86_64 构建，通过显式 overlay 加载；不是已确认的 A2 发行版或正式发布依赖。构建方式、版本和接入待项见[建图准备记录](../docs/mapping-readiness.md)。源码位于忽略的 `third_party/src/`，未修改上游。RTAB-Map 本轮使用本机已安装的 0.23.13 二进制包，尚未制作离线 ARM64 依赖包。

| 组件 | 官方来源 | 锁定前核查 |
|---|---|---|
| 禾赛驱动 | [HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) | JT128、IMU、点时间字段、SDK 子模块与 ARM64 |
| DLIO | [ROS 2 分支](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/tree/feature/ros2) | 发行版、去畸变输出、TF 发布行为 |
| RTAB-Map ROS | [rtabmap_ros](https://github.com/introlab/rtabmap_ros/tree/ros2) | ROS 消息、地标支持及底层 rtabmap 对应版本 |
| Nav2 | [navigation2](https://github.com/ros-navigation/navigation2) | 与锁定发行版对应的 Controller Server、RPP、Collision Monitor |
| AprilTag | [apriltag_ros](https://github.com/christianrauch/apriltag_ros) | 相机标定与检测消息类型 |
| Unitree SDK2 | [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | A2 接口、固件、ARM64 链接和失联语义 |

M0 确定每项来自 ROS 二进制包还是源码；源码填写完整 commit SHA，记录嵌套子模块 SHA，二进制包记录版本与来源。相同功能只安装一个有效提供者，不能叠加来源不明的 underlay。正式发布不得使用浮动 `main`、`master` 或分支名作为最终版本锁。

源码导入目标是 `third_party/src/`（Git 忽略），自己的适配放入 `src/`。如必须修改上游，维护可追踪的 fork/commit 或小补丁及原因；不把大型上游仓库直接复制提交。导入和依赖下载仅在开发或维护环境完成，机载运行时不得在线拉取。

发布清单同时登记依赖清单摘要、APT/Python 依赖、构建镜像摘要、架构及原厂预编译库。检查第三方许可证与再分发范围后再制作离线安装包；当前没有添加项目开源许可证，也没有再分发原厂库。

ROS 官方镜像提供 Humble/Jammy 与 Jazzy/Noble 的 amd64、arm64v8 变体，需与实际目标选择一致。[镜像清单](https://github.com/docker-library/official-images/blob/master/library/ros)中的 tag 是候选入口，正式构建另锁 digest；普通 ROS 镜像不能替代 Jetson 的 L4T/CUDA/视频运行时验证。
