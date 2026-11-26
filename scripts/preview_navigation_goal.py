#!/usr/bin/env python3
"""Convert saved map-to-LiDAR poses into a straight-route command preview."""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src" / package) for package in ("a2_localization", "a2_navigation")]

from a2_localization.body_pose import body_pose_from_lidar, pose_matrix
from a2_navigation.straight_preview import StraightPreview, load_straight_route


STEP_NS = 100_000_000
FIRST_STAMP_NS = 1_000_000_000


def read_object(path):
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return document


def map_database_digest(plan, manifest):
    if (manifest["output_sha256"]["map.yaml"] != plan["map"]["metadata_sha256"]
            or manifest["output_sha256"]["map.pgm"] != plan["map"]["image_sha256"]):
        raise ValueError("Route and manifest refer to different occupancy maps")
    digest = manifest["source_sha256"]["database"]
    if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
        raise ValueError("The manifest must identify the source map database by SHA-256")
    return digest


def multiply_transform(left, right):
    return tuple(
        tuple(sum(left[row][k] * right[k][col] for k in range(4)) for col in range(4))
        for row in range(4)
    )


def command_sample(controller, transform, base_from_lidar, stamp_ns, now_ns):
    try:
        pose = body_pose_from_lidar(transform, base_from_lidar, stamp_ns)
    except ValueError:
        pose = None
        command = controller.stop("invalid_lidar_pose")
    else:
        command = controller.step(x_m=pose.x_m, y_m=pose.y_m, yaw_rad=pose.yaw_rad,
                                  pose_stamp_ns=pose.stamp_ns, now_ns=now_ns)
    return {"observer_time_ns": now_ns, "body_pose": asdict(pose) if pose is not None else None,
            "command": asdict(command)}


def simulate(route, base_from_lidar, *, lose_pose):
    controller = StraightPreview(route)
    x, y = route.start_m
    yaw = math.atan2(route.goal_m[1] - y, route.goal_m[0] - x) + 0.08
    stamp_ns = FIRST_STAMP_NS
    samples = []

    def lidar_transform():
        map_from_base = pose_matrix((x, y, 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        return multiply_transform(map_from_base, base_from_lidar)

    for step in range(200):
        now_ns = FIRST_STAMP_NS + step * STEP_NS
        if not lose_pose or step < 4:
            stamp_ns = now_ns
            measured_transform = lidar_transform()
        sample = command_sample(controller, measured_transform, base_from_lidar, stamp_ns, now_ns)
        samples.append(sample)
        command = sample["command"]
        if command["state"] != "tracking":
            # Returning fresh data must not revive this run after a stop.
            samples.append(command_sample(controller, lidar_transform(), base_from_lidar,
                                          now_ns + STEP_NS, now_ns + STEP_NS))
            break
        duration_s = STEP_NS / 1e9
        yaw_delta = command["angular_z_radps"] * duration_s
        displacement = command["linear_x_mps"] * duration_s
        x += displacement * math.cos(yaw + yaw_delta / 2)
        y += displacement * math.sin(yaw + yaw_delta / 2)
        yaw += yaw_delta
    return samples


def replay(route, base_from_lidar, anchor_report, map_digest):
    if anchor_report["map_sha256"] != map_digest:
        raise ValueError("Replay poses and route refer to different map databases")
    if anchor_report["alignment"]["accepted"] is not True:
        raise ValueError("Replay poses must have an accepted map alignment")
    records = anchor_report["poses_after_alignment"]
    if not isinstance(records, list) or not records:
        raise ValueError("The anchor report has no poses after alignment")
    controller = StraightPreview(route)
    return [
        command_sample(controller, record["T_map_from_front_lidar_reference"], base_from_lidar,
                       record["sensor_reference_ns"], record["observer_clock_at_pair_ns"])
        for record in records
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True, help="Straight offline planner result")
    parser.add_argument("--map-manifest", type=Path, required=True, help="Manifest from the map exporter")
    parser.add_argument("--extrinsics", type=Path, required=True,
                        help="JSON with T_base_from_lidar and source fields")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--simulate", action="store_true", help="Exercise goal arrival and stale pose in simulation")
    mode.add_argument("--anchor-report", type=Path, help="Replay a saved map-alignment report at its original poses")
    parser.add_argument("--output", type=Path, required=True, help="New JSON report")
    args = parser.parse_args()
    try:
        plan = read_object(args.plan)
        route = load_straight_route(plan)
        digest = map_database_digest(plan, read_object(args.map_manifest))
        extrinsics = read_object(args.extrinsics)
        matrix, source = extrinsics["T_base_from_lidar"], extrinsics["source"]
        if not isinstance(source, str) or not source.strip():
            raise ValueError("The extrinsic source must be identified")
        if args.simulate:
            cases = {"arrival": simulate(route, matrix, lose_pose=False),
                     "pose_loss": simulate(route, matrix, lose_pose=True)}
        else:
            cases = {"recorded_poses": replay(route, matrix, read_object(args.anchor_report), digest)}
        report = {"scope": "local diagnostic preview", "map_database_sha256": digest,
                  "route": asdict(route), "extrinsic_source": source, "cases": cases}
        serialized = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(serialized + "\n")
    except (IndexError, KeyError, OSError, TypeError, ValueError) as error:
        print(f"Invalid command preview input: {error}", file=sys.stderr)
        return 2
    for name, samples in cases.items():
        final = samples[-1]["command"]
        print(f"{name}: {final['state']} ({final['reason'] or 'complete'}), {len(samples)} samples")
    if args.simulate and (cases["arrival"][-1]["command"]["state"] != "arrived"
                          or cases["pose_loss"][-1]["command"]["reason"] != "stale_pose"):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
