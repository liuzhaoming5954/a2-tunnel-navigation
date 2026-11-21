#!/usr/bin/env python3
"""生成上位机驾驶舱或 PC2 只读数据服务的独立离线包。"""

import argparse
import hashlib
import os
from pathlib import Path
import sys
import tempfile
import zipfile


ROOT = Path(__file__).resolve().parents[1]
DIST = Path("src/a2_gateway/web/dist")

# 两端共用入口和 HTTP 协议。ros_telemetry 在实例化时才加载 ROS，站端无需安装 ROS。
COMMON_FILES = (
    "scripts/run_a2_console.py",
    "src/a2_gateway/a2_gateway/__init__.py",
    "src/a2_gateway/a2_gateway/server.py",
    "src/a2_gateway/a2_gateway/odin1_pose.py",
    "src/a2_gateway/a2_gateway/ros_telemetry.py",
    "src/a2_navigation/a2_navigation/__init__.py",
    "src/a2_navigation/a2_navigation/offline_planner.py",
)
PC2_FILES = (
    "src/a2_gateway/a2_gateway/pc2_goal.py",
    "src/a2_navigation/a2_navigation/single_goal.py",
    "src/a2_navigation/a2_navigation/nav2_goal_client.py",
    "scripts/start_a2_console_readonly.sh",
    "scripts/observe_odin1_pose.py",
    "scripts/publish_odin1_navigation_pose.py",
    "scripts/pc2_single_goal_nav2.launch.py",
    "scripts/publish_a2_platform_state.py",
    "src/a2_interfaces/package.xml",
    "src/a2_interfaces/CMakeLists.txt",
)
STATION_FILES = (
    "src/a2_gateway/a2_gateway/station.py",
    "src/a2_gateway/a2_gateway/simulator.py",
    "src/a2_gateway/a2_gateway/patrol.py",
)


def bundle_files(role):
    """按角色选取运行文件，现场点云、重定位 BIN、标定及凭据均由现场单独配置。"""
    paths = [ROOT / name for name in COMMON_FILES]
    if role == "pc2":
        paths.extend(ROOT / name for name in PC2_FILES)
        interface_root = ROOT / "src/a2_interfaces"
        # 接口包按自己的 CMakeLists 编译，保留其声明的消息与服务定义。
        paths.extend(sorted(interface_root.glob("msg/*.msg")))
        paths.extend(sorted(interface_root.glob("srv/*.srv")))
    elif role == "station":
        paths.extend(ROOT / name for name in STATION_FILES)
        dist = ROOT / DIST
        scripts = sorted((dist / "assets").glob("*.js"))
        styles = sorted((dist / "assets").glob("*.css"))
        if not scripts or not styles:
            raise ValueError("请先构建 Vue 前端，再打包上位机")
        # 当前驾驶舱的离线依赖是入口、图标及 Vite 生成的脚本和样式。
        paths.extend((dist / "index.html", dist / "a2.svg", *scripts, *styles))
    else:
        raise ValueError(f"未知角色 {role!r}")
    return {path.relative_to(ROOT).as_posix(): path.read_bytes() for path in paths}


def write_bundle(role, output):
    entries = bundle_files(role)
    entries["SHA256SUMS"] = "".join(
        f"{hashlib.sha256(data).hexdigest()}  {name}\n" for name, data in sorted(entries.items())
    ).encode("utf-8")
    # 同目录写完并关闭 ZIP 后才公布目标文件。已有同名包时保留原件，也能避免并发覆盖。
    with tempfile.NamedTemporaryFile(dir=output.parent) as staged:
        with zipfile.ZipFile(staged, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for name, data in sorted(entries.items()):
                archive.writestr(name, data)
        staged.flush()
        os.link(staged.name, output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("pc2", "station"), default="pc2",
                        help="pc2 为只读状态和位姿服务（默认）；station 为上位机驾驶舱")
    parser.add_argument("--output", type=Path, required=True, help="新 ZIP 文件路径")
    args = parser.parse_args()
    try:
        write_bundle(args.role, args.output)
    except (OSError, ValueError) as error:
        print(f"A2 {args.role} 打包失败：{error}", file=sys.stderr)
        return 1
    print(f"SHA256 {hashlib.sha256(args.output.read_bytes()).hexdigest()}  {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
