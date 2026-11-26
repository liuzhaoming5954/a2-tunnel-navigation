"""离线验证 PC2 和上位机的发布边界，不启动 ROS、设备或数据链路。"""

import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

from scripts import package_a2_console as packager


class ConsolePackageTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.work = Path(temp.name)
        self.root = self.work / "project"
        # 复制真实入口与运行时源码，测试用的前端构建产物独立合成。
        for relative in (*packager.COMMON_FILES, *packager.PC2_FILES, *packager.STATION_FILES):
            target = self.root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(packager.ROOT / relative, target)
        for kind in ("msg", "srv"):
            subtree = Path("src/a2_interfaces") / kind
            shutil.copytree(packager.ROOT / subtree, self.root / subtree)
        root_patch = patch.object(packager, "ROOT", self.root)
        root_patch.start()
        self.addCleanup(root_patch.stop)

    def build_and_check(self, role):
        output = self.work / f"{role}.zip"
        packager.write_bundle(role, output)
        with zipfile.ZipFile(output) as archive:
            names = set(archive.namelist())
            manifest = archive.read("SHA256SUMS").decode().splitlines()
            checksums = {}
            for line in manifest:
                digest, name = line.split("  ", 1)
                checksums[name] = digest
            self.assertEqual(set(checksums), names - {"SHA256SUMS"})
            for name, digest in checksums.items():
                self.assertEqual(hashlib.sha256(archive.read(name)).hexdigest(), digest)
            deployed = self.work / role
            archive.extractall(deployed)
        # 在解压目录运行独立 Python，避免从原始项目意外找到漏打包的模块。
        subprocess.run([sys.executable, "-I", "scripts/run_a2_console.py", "--role", role, "--help"],
                       cwd=deployed, capture_output=True, check=True, timeout=5)
        return names

    def test_pc2_builds_without_vue_or_map(self):
        names = self.build_and_check("pc2")
        # 在只有 PC2 角色文件的解包目录检查两个模拟入口的依赖，禁止使用外部 SDK/site 包。
        for script in ("scripts/run_a2_manual_demo.py", "scripts/run_a2_manual_receiver.py"):
            self.assertIn(script, names)
            subprocess.run([sys.executable, "-I", "-S", "-B", script, "--help"], cwd=self.work / "pc2",
                           capture_output=True, check=True, timeout=5)
        demo = subprocess.run([sys.executable, "-I", "-S", "-B", "scripts/run_a2_manual_demo.py",
                               "--pc2-only", "--port", "0"], cwd=self.work / "pc2",
                              capture_output=True, text=True, timeout=5)
        self.assertEqual(demo.returncode, 2)
        self.assertIn("--manual-key-file", demo.stderr)
        self.assertIn("scripts/observe_odin1_pose.py", names)
        self.assertIn("scripts/start_a2_console_readonly.sh", names)
        self.assertIn("scripts/start_a2_console_readonly_boot.sh", names)
        self.assertIn("deploy/templates/a2-cockpit-telemetry.service", names)
        self.assertIn("deploy/templates/a2-autonomy-continuous-hold.conf", names)
        self.assertIn("src/a2_interfaces/msg/PlatformState.msg", names)
        self.assertIn("src/a2_interfaces/msg/PlatformDetails.msg", names)
        self.assertIn("src/a2_interfaces/msg/BatteryTelemetry.msg", names)
        self.assertIn("src/a2_gateway/a2_gateway/platform_details.py", names)
        self.assertIn("src/a2_gateway/a2_gateway/remote_stop_status.py", names)
        self.assertFalse(any("/web/" in n or n.startswith(("maps/", "data/")) for n in names))
        self.assertNotIn("scripts/publish_a2_map.py", names)
        self.assertNotIn("src/a2_gateway/a2_gateway/station.py", names)
        self.assertNotIn("src/a2_gateway/a2_gateway/simulator.py", names)
        self.assertNotIn("src/a2_gateway/a2_gateway/patrol.py", names)
        self.assertIn("src/a2_gateway/a2_gateway/pc2_goal.py", names)
        self.assertIn("src/a2_navigation/a2_navigation/nav2_goal_client.py", names)
        self.assertIn("src/a2_navigation/a2_navigation/single_goal.py", names)
        self.assertIn("src/a2_navigation/a2_navigation/offline_planner.py", names)

    def test_station_bundles_vue_without_field_data_or_pc2_launchers(self):
        dist = self.root / packager.DIST
        (dist / "assets").mkdir(parents=True)
        for name in ("index.html", "a2.svg", "assets/app.js", "assets/app.css", "assets/field.pcd"):
            (dist / name).write_bytes(b"fixture")
        names = self.build_and_check("station")
        self.assertIn(f"{packager.DIST}/assets/app.js", names)
        self.assertIn("deploy/templates/a2-cockpit-station.service", names)
        self.assertIn("deploy/templates/a2-cockpit-ssh.conf.example", names)
        self.assertNotIn(f"{packager.DIST}/assets/field.pcd", names)
        self.assertIn("src/a2_navigation/a2_navigation/offline_planner.py", names)
        self.assertIn("src/a2_gateway/a2_gateway/patrol.py", names)
        self.assertIn("src/a2_gateway/a2_gateway/platform_details.py", names)
        self.assertIn("src/a2_gateway/a2_gateway/remote_stop_status.py", names)
        self.assertNotIn("scripts/publish_a2_platform_state.py", names)
        self.assertNotIn("scripts/observe_odin1_pose.py", names)
        self.assertNotIn("scripts/run_a2_manual_demo.py", names)
        self.assertNotIn("scripts/run_a2_manual_receiver.py", names)
        self.assertFalse(any(n.startswith("src/a2_interfaces/") for n in names))
        output = self.work / "station.zip"
        original = output.read_bytes()
        with self.assertRaises(FileExistsError):
            packager.write_bundle("station", output)
        self.assertEqual(original, output.read_bytes())


if __name__ == "__main__":
    unittest.main()
