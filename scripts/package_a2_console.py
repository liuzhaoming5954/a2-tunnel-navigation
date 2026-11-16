#!/usr/bin/env python3
"""Create an offline A2 read-only console bundle from built frontend assets."""

import argparse
import hashlib
from pathlib import Path
import re
import sys
import zipfile


ROOT = Path(__file__).resolve().parents[1]
DIST = Path("src/a2_gateway/web/dist")
FILES = (
    "scripts/start_a2_console_readonly.sh",
    "scripts/run_a2_console.py",
    "scripts/publish_a2_map.py",
    "scripts/publish_a2_platform_state.py",
    "src/a2_gateway/a2_gateway/__init__.py",
    "src/a2_gateway/a2_gateway/motion.py",
    "src/a2_gateway/a2_gateway/ros_telemetry.py",
    "src/a2_gateway/a2_gateway/server.py",
    "src/a2_gateway/a2_gateway/simulator.py",
    "src/a2_navigation/a2_navigation/__init__.py",
    "src/a2_navigation/a2_navigation/offline_planner.py",
    "src/a2_interfaces/package.xml",
    "src/a2_interfaces/CMakeLists.txt",
)


def bundle_files(map_path):
    sys.path.insert(0, str(ROOT / "src/a2_navigation"))
    from a2_navigation.offline_planner import load_map, load_metadata

    metadata = load_metadata(map_path.read_text(encoding="utf-8"))
    image_name = metadata.get("image")
    if not isinstance(image_name, str) or not re.fullmatch(r"[\w.-]+\.pgm", image_name, re.ASCII):
        raise ValueError("Map image must be a PGM filename beside the YAML")
    load_map(map_path)
    assets = sorted((ROOT / DIST / "assets").glob("*"))
    if not any(asset.suffix == ".js" for asset in assets) or not any(asset.suffix == ".css" for asset in assets):
        raise ValueError("Build the Vue frontend before packaging")
    interface_root = ROOT / "src/a2_interfaces"
    paths = [ROOT / name for name in FILES] + [ROOT / DIST / "index.html", ROOT / DIST / "a2.svg"]
    paths.extend(assets)
    paths.extend(sorted(interface_root.glob("msg/*.msg")))
    paths.extend(sorted(interface_root.glob("srv/*.srv")))
    entries = {path.relative_to(ROOT).as_posix(): path.read_bytes() for path in paths}
    entries["data/map.yaml"] = map_path.read_bytes()
    entries[f"data/{image_name}"] = map_path.with_name(image_name).read_bytes()
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True, help="Local trinary map YAML for this A2")
    parser.add_argument("--output", type=Path, required=True, help="New ZIP file")
    args = parser.parse_args()
    try:
        entries = bundle_files(args.map)
        entries["SHA256SUMS"] = "".join(
            f"{hashlib.sha256(data).hexdigest()}  {name}\n" for name, data in sorted(entries.items())
        ).encode("utf-8")
        with zipfile.ZipFile(args.output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
            for name, data in sorted(entries.items()):
                archive.writestr(name, data)
    except (OSError, ValueError) as error:
        print(f"A2 console bundle: {error}", file=sys.stderr)
        return 1
    print(f"SHA256 {hashlib.sha256(args.output.read_bytes()).hexdigest()}  {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
