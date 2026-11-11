#!/usr/bin/env python3
"""Plan to an offline map coordinate using an explicit circular footprint."""

import argparse
import json
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/a2_navigation"))

from a2_navigation.offline_planner import load_map, plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True, help="ROS trinary map YAML")
    parser.add_argument("--start", type=float, nargs=2, metavar=("X", "Y"), required=True)
    parser.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"), required=True)
    parser.add_argument("--via", type=float, nargs=2, metavar=("X", "Y"), action="append", default=[])
    parser.add_argument("--footprint-radius-m", type=float, required=True)
    parser.add_argument("--output", type=Path, help="New JSON report file. Defaults to stdout.")
    args = parser.parse_args()
    try:
        report = plan(load_map(args.map), args.start, args.goal, args.footprint_radius_m, args.via)
    except (OSError, ValueError) as error:
        report = {"status": "invalid_input", "reason": str(error)}
    if args.output:
        try:
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump(report, stream, indent=2, allow_nan=False)
                stream.write("\n")
        except OSError as error:
            print(error, file=sys.stderr)
            return 2
    else:
        print(json.dumps(report, indent=2, allow_nan=False))
    if report["status"] == "planned":
        return 0
    return 1 if report["status"] == "unreachable" else 2


if __name__ == "__main__":
    raise SystemExit(main())
