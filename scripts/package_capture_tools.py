#!/usr/bin/env python3
"""Package only standalone sensor capture tools for offline transfer to A2."""

import argparse
import hashlib
from pathlib import Path
import zipfile


ROOT = Path(__file__).resolve().parents[1]
FILES = ("scripts/a2_capture.py", "scripts/inspect_mapping_bag.py",
         "config/examples/mapping-capture.json", "docs/a2-connection.md")
README = """A2 原装传感器采集工具包

用途：A2 上核查 ROS 2 传感器话题、按白名单录制、检查实际消息。
工具不启动驱动、不调用运动 SDK、不发送运动命令，不依赖外网。
仅包含 Python 源码，不包含 ROS、驱动、标定、现场数据或凭据。
已在开发环境完成软件检查；现场系统、通信依赖和验证结果以私有记录为准。
维护网络、SSH、传包与回传的完整步骤见 docs/a2-connection.md。
公共仓库：https://github.com/liuzhaoming5954/a2-tunnel-navigation
按手册从干净的固定提交生成工具包，配套传入 SOURCE_COMMIT.txt 和校验文件。
A2 将版本记录保存到 config/local/SOURCE_COMMIT.txt，录包回传时一起保存。

准备环境：Python 3.10+，A2 实际安装的 ROS 2、厂商驱动环境，
rclpy、sensor_msgs、tf2_msgs、rosidl_runtime_py、rosbag2_py、
ros2 bag CLI 和配置所选的 sqlite3/mcap 存储插件。
在维护阶段准备缺失依赖；不要复制 x86_64 编译产物给 ARM64。
不要修改已有原厂进程、时钟或网卡来套用开发机配置。

1. 解压后进入 a2-capture-kit 目录，核对文件：
   sha256sum -c SHA256SUMS
2. 加载 A2 实际 ROS/驱动环境，然后运行：
   mkdir -p data/captures config/local
   python3 scripts/a2_capture.py inventory --output data/captures/inventory.json
3. 复制 config/examples/mapping-capture.json 为 config/local/mapping-capture.json。
   根据 inventory.json 的真实话题填写 topics 映射，必须有 front_points 和 imu。
   可选 rear_points、image（sensor_msgs/msg/Image）、camera_info。
   /tf、/tf_static 由工具固定纳入，无需配置。
   point_time.field / unit 根据驱动核实；单位支持 s/ms/us/ns；
   暂时未知则两项均留空，允许保留原始数据，不能据此判断 DLIO 可用。
   storage_id 默认 sqlite3，必须已安装相应插件。空示例不能直接录制。
4. 带配置短暂订阅，检查类型/QoS/点云字段/IMU/TF：
   python3 scripts/a2_capture.py inventory --config config/local/mapping-capture.json --output data/captures/sensors.json
5. 原地先录 60 秒短包（新会话目录），现场由原厂方式独立控制：
   python3 scripts/a2_capture.py record --config config/local/mapping-capture.json --session data/captures/a2-static-01 --duration 60 --execute
   不带 --execute 只预览命令。时限到达、Ctrl-C 或磁盘保留线触发收尾。
   默认空间保留线 5 GiB，按约 2 GiB 分片；仍需预估容量和观察磁盘。
6. 离场前检查：
   ros2 bag info data/captures/a2-static-01/bag
   python3 scripts/inspect_mapping_bag.py data/captures/a2-static-01/bag --config data/captures/a2-static-01/config.json --output data/captures/a2-static-01/bag-check.json
   输出文件必须不存在。非零退出或报告 issues 需要排查/补录。
   TF 缺失可能只警告；点内时间未知会列 issue，并不意味着原始数据应丢弃。
7. 短包检查后采集新的静止/直行/转弯/短路线往返包。
   独立记录参考点、IMU 来源、点内时间参考、同步方式、异常，
   并保存厂商标定和驱动配置，不能把 SLAM 自己的结果当独立真值。
8. 完整复制每个会话和 bag 全部分片到开发机，核对逐文件 SHA-256。
   保留源副本直至核对完成。现场数据不进入 GitHub。

限制：预检与结构检查不证明时钟同步、点时间单位、外参、去畸变或建图精度。
本工具包不包含 DLIO/RTAB-Map 回放启动器，也不授予自主运动资格。
已建立维护连接与限时订阅核查流程；现场结果保留私有，录包及算法建图按准入条件验证。
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="new ZIP file; parent must exist")
    args = parser.parse_args()
    entries = {relative: (ROOT / relative).read_bytes() for relative in FILES}
    entries["README.txt"] = README.encode("utf-8")
    entries["SHA256SUMS"] = "".join(
        f"{hashlib.sha256(content).hexdigest()}  {relative}\n"
        for relative, content in sorted(entries.items())).encode("ascii")
    # Constant file allowlist: no recursive repository, data, configs or credentials.
    with zipfile.ZipFile(args.output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for relative, content in sorted(entries.items()):
            archive.writestr("a2-capture-kit/" + relative, content)
    print(f"Created {args.output}: {len(entries)} files, {args.output.stat().st_size} bytes")
    print(f"SHA256 {hashlib.sha256(args.output.read_bytes()).hexdigest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
