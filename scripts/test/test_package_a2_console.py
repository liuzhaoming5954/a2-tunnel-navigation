import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import zipfile

from scripts import package_a2_console


class PackageConsoleTest(unittest.TestCase):
    def test_extracted_bundle_contains_importable_gateway(self):
        with tempfile.TemporaryDirectory() as temp:
            workspace = Path(temp) / "workspace"
            packager = "scripts/package_a2_console.py"
            for name in (packager, *package_a2_console.FILES):
                target = workspace / name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(package_a2_console.ROOT / name, target)
            for name in ("src/a2_interfaces/msg", "src/a2_interfaces/srv"):
                shutil.copytree(package_a2_console.ROOT / name, workspace / name)

            frontend = workspace / package_a2_console.DIST
            (frontend / "assets").mkdir(parents=True)
            for name in ("index.html", "a2.svg", "assets/index.js", "assets/index.css"):
                (frontend / name).write_text("synthetic frontend fixture", encoding="utf-8")

            map_yaml = workspace / "map.yaml"
            map_yaml.write_text(json.dumps({
                "image": "map.pgm", "resolution": 0.25, "origin": [0, 0, 0],
                "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25,
            }), encoding="utf-8")
            map_yaml.with_suffix(".pgm").write_bytes(b"P5\n2 1\n255\n\xff\x00")
            archive = Path(temp) / "console.zip"
            packaged = subprocess.run(
                [sys.executable, packager, "--map", str(map_yaml), "--output", str(archive)],
                cwd=workspace, capture_output=True, text=True, timeout=10,
            )
            self.assertEqual(packaged.returncode, 0, packaged.stderr)
            extracted = Path(temp) / "extracted"
            with zipfile.ZipFile(archive) as bundle:
                bundle.extractall(extracted)

            launch = """\
import runpy
import sys
sys.path.insert(0, 'src/a2_gateway')
from a2_gateway.motion import LocalMotionClient
sys.argv = ['scripts/run_a2_console.py', '--help']
runpy.run_path(sys.argv[0], run_name='__main__')
"""
            imported = subprocess.run(
                [sys.executable, "-I", "-c", launch],
                cwd=extracted, capture_output=True, text=True, timeout=10,
            )
            self.assertEqual(imported.returncode, 0, imported.stderr)
            self.assertIn("--motion-socket", imported.stdout)


if __name__ == "__main__":
    unittest.main()
