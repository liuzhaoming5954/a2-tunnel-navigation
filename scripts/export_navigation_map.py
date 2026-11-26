#!/usr/bin/env python3
"""Export a conservative offline 2D map from a verified RTAB-Map replay."""

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys
import zlib

import numpy as np


MAP_CELLS_LIMIT = 4_000_000
STAMP_TOLERANCE_NS = 1_000
FLOOR_INLIER_M = 0.08
FLOOR_MAX_SLOPE = 0.35
FLOOR_MIN_SUPPORT = 18
MIN_OCCUPIED_HEIGHT_M = 0.16
MAX_OCCUPIED_HEIGHT_M = 1.25


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            result.update(chunk)
    return result.hexdigest()


def matrix(blob, cv_type, dtype, channels, limit):
    if blob is None or len(blob) <= 12:
        raise ValueError("Missing RTAB-Map matrix")
    rows, columns, stored_type = struct.unpack("<iii", blob[-12:])
    if rows != 1 or not 0 < columns <= limit or stored_type != cv_type:
        raise ValueError(f"Unsupported RTAB-Map matrix header {(rows, columns, stored_type)}")
    expected = columns * channels * np.dtype(dtype).itemsize
    decoder = zlib.decompressobj()
    payload = decoder.decompress(blob[:-12], expected + 1)
    if len(payload) != expected or not decoder.eof or decoder.unused_data:
        raise ValueError("RTAB-Map matrix payload does not match its dimensions")
    return np.frombuffer(payload, dtype=dtype)


def check_transform(transform):
    rotation = transform[:, :3]
    if (not np.isfinite(transform).all()
            or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-4, rtol=0)
            or not math.isclose(float(np.linalg.det(rotation)), 1, abs_tol=1e-4)):
        raise ValueError("Stored scan transform is not rigid")


def load_scans(database, up_axis):
    if any(Path(str(database) + suffix).exists() for suffix in ("-wal", "-journal")):
        raise ValueError("Close the RTAB-Map database before exporting")
    with sqlite3.connect(database.resolve().as_uri() + "?mode=ro&immutable=1", uri=True) as connection:
        if connection.execute("PRAGMA quick_check").fetchall() != [("ok",)]:
            raise ValueError("RTAB-Map database integrity check failed")
        entries = connection.execute("SELECT opt_ids,opt_poses FROM Admin").fetchall()
        if len(entries) != 1:
            raise ValueError("Expected one persisted optimized pose graph")
        ids_blob, poses_blob = entries[0]
        ids = matrix(ids_blob, 4, "<i4", 1, 20_000)
        poses = matrix(poses_blob, 5, "<f4", 1, len(ids) * 12).reshape(-1, 3, 4)
        rows = connection.execute(
            """SELECT n.id,n.stamp,d.scan,d.scan_info FROM Node n JOIN Data d USING(id)
               WHERE n.weight>=0 AND length(d.scan)>12 ORDER BY n.stamp,n.id""").fetchall()

    if (len(poses) != len(ids) or len(set(ids)) != len(ids)
            or set(ids) != {node for node, *_ in rows}):
        raise ValueError("Optimized nodes and retained scan nodes differ")
    transforms = dict(zip(ids, poses))
    scans = []
    for node, stamp, blob, info in rows:
        if node <= 0 or not math.isfinite(stamp) or stamp <= 0 or info is None or len(info) != 76:
            raise ValueError(f"Unsupported timestamp or scan metadata at node {node}")
        local = np.frombuffer(info[-48:], dtype="<f4").reshape(3, 4).astype(float)
        pose = transforms[node].astype(float)
        check_transform(local)
        check_transform(pose)
        rotation = pose[:, :3] @ local[:, :3]
        origin = pose[:, :3] @ local[:, 3] + pose[:, 3]
        if rotation[2, up_axis] < 0.90:
            raise ValueError(f"Declared scanner up axis disagrees with graph Z at node {node}")
        points = matrix(blob, 37, "<f4", 5, 250_000).reshape(-1, 5)[:, :3]
        scans.append((node, round(stamp * 1e9), rotation, origin, points[np.isfinite(points).all(axis=1)]))
    return scans


def join_scan_provenance(scans, mapping_log, input_ledger):
    acked = {}
    previous_header = 0
    with input_ledger.open() as stream:
        for line in stream:
            row = json.loads(line)
            sequence = row["input"]
            header, end, receipt = row["header_ns"], row["audit_scan_end_ns"], row["receipt_ns"]
            if (any(type(value) is not int for value in (sequence, header, end, receipt))
                    or sequence != len(acked) + 1 or not previous_header < header <= end or receipt < header):
                raise ValueError("Invalid acknowledged scan ledger")
            acked[sequence] = header, end, receipt
            previous_header = header
    published = {}
    published_inputs = set()
    with mapping_log.open() as stream:
        for line in stream:
            if "A2_SCAN_SYNC " not in line:
                continue
            fields = dict(field.split("=", 1) for field in line.split("A2_SCAN_SYNC ", 1)[1].split())
            if fields["result"] != "published":
                continue
            sequence, header, reference = (int(fields[key]) for key in ("input", "input_ns", "reference_ns"))
            acknowledged_header, end, receipt = acked[sequence]
            if (reference in published or sequence in published_inputs
                    or header != acknowledged_header or not header <= reference <= end):
                raise ValueError("Published scan does not match its acknowledged raw time interval")
            published[reference] = sequence, header, receipt
            published_inputs.add(sequence)
    references = sorted(published)
    matched = set()
    result = {}
    for node, stamp, *_ in scans:
        left = bisect.bisect_left(references, stamp - STAMP_TOLERANCE_NS)
        right = bisect.bisect_right(references, stamp + STAMP_TOLERANCE_NS)
        if right - left != 1 or references[left] in matched:
            raise ValueError(f"No unique synchronized scan for retained map node {node}")
        reference = references[left]
        matched.add(reference)
        sequence, header, receipt = published[reference]
        result[node] = {
            "mapping_input": sequence, "raw_header_ns": header, "receipt_ns": receipt,
            "reference_ns": reference, "database_stamp_delta_ns": stamp - reference,
        }
    return result


def fit_floor(points, height_range, rng):
    """Fit local z = ax + by + c in leveled axes relative to the scanner."""
    low, high = height_range
    near = points[(np.linalg.norm(points[:, :2], axis=1) <= 3.5)
                  & (points[:, 2] >= -high - 0.35) & (points[:, 2] <= -low + 0.25)]
    if len(near) < FLOOR_MIN_SUPPORT:
        return None
    triangles = near[rng.integers(len(near), size=(128, 3))]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    valid = np.abs(normals[:, 2]) > 0.04
    normals, triangles = normals[valid], triangles[valid]
    gradients = -normals[:, :2] / normals[:, 2:3]
    intercepts = triangles[:, 0, 2] - np.sum(gradients * triangles[:, 0, :2], axis=1)
    valid = (np.linalg.norm(gradients, axis=1) <= FLOOR_MAX_SLOPE) & (-intercepts >= low) & (-intercepts <= high)
    if not valid.any():
        return None
    features = np.column_stack((near[:, :2], np.ones(len(near))))
    hypotheses = np.column_stack((gradients[valid], intercepts[valid]))
    inliers = np.abs(near[:, 2:3] - features @ hypotheses.T) <= FLOOR_INLIER_M
    # Require lateral extent as well as support before accepting a surface.
    for index in np.argsort(-inliers.sum(axis=0), kind="stable"):
        chosen = inliers[:, index]
        if chosen.sum() < FLOOR_MIN_SUPPORT:
            return None
        xy = near[chosen, :2]
        if np.linalg.eigvalsh(np.cov(xy.T))[0] < 0.025:
            continue
        coefficients = np.linalg.lstsq(features[chosen], near[chosen, 2], rcond=None)[0]
        if (not low <= -coefficients[2] <= high
                or np.linalg.norm(coefficients[:2]) > FLOOR_MAX_SLOPE):
            continue
        support = int((np.abs(near[:, 2] - features @ coefficients) <= FLOOR_INLIER_M).sum())
        if support >= FLOOR_MIN_SUPPORT:
            return coefficients, support
    return None


def ray_cells(start, end, width):
    """Traverse cells intersected for nonzero length, without corner padding."""
    x, y = start
    dx, dy = end[0] - x, end[1] - y
    ix, iy = math.floor(x), math.floor(y)
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    # A ray beginning on a cell boundary and moving backwards starts in the
    # preceding cell. Endpoint cells touched only at a boundary get no vote.
    if dx < 0 and x == ix:
        ix -= 1
    if dy < 0 and y == iy:
        iy -= 1
    tx = ((ix + (sx > 0)) - x) / dx if dx else math.inf
    ty = ((iy + (sy > 0)) - y) / dy if dy else math.inf
    step_x = abs(1 / dx) if dx else math.inf
    step_y = abs(1 / dy) if dy else math.inf
    while True:
        yield iy * width + ix
        crossing = min(tx, ty)
        if crossing >= 1:
            break
        if tx == crossing:
            ix += sx
            tx += step_x
        if ty == crossing:
            iy += sy
            ty += step_y


def mark_scan(points, floor, origin, grid_origin, resolution, width, height):
    point_height = points[:, 2] - points[:, :2] @ floor[:2] - floor[2]
    cells = (points[:, :2] + origin[:2] - grid_origin) / resolution
    pixel = np.floor(cells).astype(int)
    if ((pixel < 0).any() or (pixel[:, 0] >= width).any() or (pixel[:, 1] >= height).any()):
        raise ValueError("Scan endpoints lie outside the export grid")
    indices = pixel[:, 1] * width + pixel[:, 0]
    floor_hits = set(indices[np.abs(point_height) <= FLOOR_INLIER_M].tolist())
    obstacle_hits = set(indices[(point_height >= MIN_OCCUPIED_HEIGHT_M)
                                & (point_height <= MAX_OCCUPIED_HEIGHT_M)].tolist())
    origin_cell = (origin[:2] - grid_origin) / resolution
    scanner_height = -floor[2]
    observed = set()
    # Only traverse the portion of an observed ray inside the height slice.
    # A ground return is separately required before declaring any cell free.
    for endpoint, hit_height in zip(cells, point_height):
        delta = hit_height - scanner_height
        first, last = 0.0, 1.0
        if delta:
            enter = (MIN_OCCUPIED_HEIGHT_M - scanner_height) / delta
            leave = (MAX_OCCUPIED_HEIGHT_M - scanner_height) / delta
            first, last = max(first, min(enter, leave)), min(last, max(enter, leave))
        elif not MIN_OCCUPIED_HEIGHT_M <= scanner_height <= MAX_OCCUPIED_HEIGHT_M:
            continue
        if last - first <= 1e-8 or np.array_equal(origin_cell, endpoint):
            continue
        direction = endpoint - origin_cell
        start = origin_cell + first * direction
        end = origin_cell + last * direction
        observed.update(ray_cells(start, end, width))
    return floor_hits, obstacle_hits, observed


def rasterize(scans, provenance, resolution, height_range, maximum_range, free_observations):
    centers = np.array([origin[:2] for _, _, _, origin, _ in scans])
    grid_origin = np.floor((centers.min(axis=0) - maximum_range - resolution) / resolution) * resolution
    upper = centers.max(axis=0) + maximum_range + resolution
    extents = np.ceil((upper - grid_origin) / resolution)
    if (not np.isfinite(extents).all() or (extents < 1).any()
            or np.prod(extents) > MAP_CELLS_LIMIT):
        raise ValueError("Map exceeds the grid cell limit")
    width, height = extents.astype(int)
    ground = np.zeros(width * height, dtype=bool)
    occupied = np.zeros_like(ground)
    seen = np.zeros_like(ground, dtype=np.uint16)
    records = []
    for node, _, rotation, origin, points in scans:
        leveled = points @ rotation.T
        ranges = np.linalg.norm(leveled, axis=1)
        leveled = leveled[(ranges >= 0.80) & (ranges <= maximum_range)]
        fit = fit_floor(leveled, height_range, np.random.default_rng(node))
        record = {"node_id": node, **provenance[node], "sensor_origin_m": origin.tolist(),
                  "map_from_sensor_rotation": rotation.tolist(), "floor_fit": None}
        records.append(record)
        if fit is None:
            continue
        floor, count = fit
        record["floor_fit"] = {"relative_plane": floor.tolist(), "support": count}
        floor_hits, obstacle_hits, observed = mark_scan(leveled, floor, origin, grid_origin, resolution, width, height)
        ground[list(floor_hits)] = True
        occupied[list(obstacle_hits)] = True
        seen[list(observed)] += 1
    free = ground & (seen >= free_observations) & ~occupied
    pixels = np.full(width * height, 205, dtype=np.uint8)
    pixels[free], pixels[occupied] = 254, 0
    counts = {"free": int(free.sum()), "occupied": int(occupied.sum()), "unknown": int((pixels == 205).sum())}
    return pixels.reshape(height, width)[::-1], grid_origin, records, counts


def export_map(args):
    sources = {"database": args.database, "mapping_log": args.mapping_log, "input_ledger": args.input_ledger}
    source_hashes = {name: digest(path) for name, path in sources.items()}
    if args.expected_database_sha256 and source_hashes["database"] != args.expected_database_sha256:
        raise ValueError("Database hash does not match the requested map version")
    scans = load_scans(args.database, "xyz".index(args.sensor_up))
    provenance = join_scan_provenance(scans, args.mapping_log, args.input_ledger)
    pixels, origin, records, counts = rasterize(
        scans, provenance, args.resolution, args.floor_offset_range, args.max_range, args.free_observations)
    if counts["free"] == 0 or counts["occupied"] == 0:
        raise ValueError("No usable observed free and occupied space")
    if any(digest(path) != source_hashes[name] for name, path in sources.items()):
        raise ValueError("A map export input changed during processing")

    output = args.output_dir
    output.mkdir()
    height, width = pixels.shape
    paths = {name: output / name for name in ("map.pgm", "map.yaml", "keyframes.json", "manifest.json")}
    try:
        paths["map.pgm"].write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + pixels.tobytes())
        paths["map.yaml"].write_text(
            f"image: map.pgm\nmode: trinary\nresolution: {args.resolution}\n"
            f"origin: [{origin[0]:.9f}, {origin[1]:.9f}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")
        paths["keyframes.json"].write_text(json.dumps(records, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        manifest = {
            "format_version": 1,
            "purpose": "offline_navigation_preflight",
            "field_validated": False,
            "source_sha256": source_hashes,
            "exporter_sha256": digest(Path(__file__)),
            "output_sha256": {name: digest(path) for name, path in paths.items() if name != "manifest.json"},
            "retained_scans": len(records),
            "scans_with_floor_fit": sum(row["floor_fit"] is not None for row in records),
            "max_database_stamp_delta_ns": max(abs(row["database_stamp_delta_ns"]) for row in records),
            "geometry": {
                "frame": "optimized_graph_xy", "world_up": "+z", "sensor_up": "+" + args.sensor_up,
                "resolution_m": args.resolution, "origin_m": origin.tolist(), "size_cells": [width, height],
                "floor_offset_range_m": args.floor_offset_range, "floor_inlier_m": FLOOR_INLIER_M,
                "floor_max_slope": FLOOR_MAX_SLOPE, "occupied_height_m": [MIN_OCCUPIED_HEIGHT_M, MAX_OCCUPIED_HEIGHT_M],
                "max_range_m": args.max_range, "min_distinct_scan_rays_for_free": args.free_observations,
            },
            "cell_counts": counts,
            "limitations": [
                "Ray projections use persisted deskewed and downsampled scan points with optimized sensor poses.",
                "Free cells require a measured ground return and observed rays through the height slice.",
                "Any occupied hit overrides free evidence. Unobserved cells stay unknown.",
                "The map is an offline projection of a recorded route and does not establish traversability, "
                "present obstacles, scale, footprint, drop-off detection, or permission to move.",
            ],
        }
        paths["manifest.json"].write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (OSError, ValueError):
        for path in paths.values():
            path.unlink(missing_ok=True)
        output.rmdir()
        raise
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", required=True, type=Path, help="Closed optimized RTAB-Map database")
    parser.add_argument("--expected-database-sha256", help="Known digest of the verified map")
    parser.add_argument("--mapping-log", required=True, type=Path, help="DLIO A2_SCAN_SYNC publication log")
    parser.add_argument("--input-ledger", required=True, type=Path, help="Acknowledged raw scan JSONL ledger")
    parser.add_argument("--sensor-up", choices=tuple("xyz"), required=True, help="Verified positive sensor up axis")
    parser.add_argument("--output-dir", required=True, type=Path, help="New directory for offline files")
    parser.add_argument("--resolution", type=float, default=0.20)
    parser.add_argument("--max-range", type=float, default=4.0)
    parser.add_argument("--floor-offset-range", nargs=2, type=float, default=(0.30, 0.85), metavar=("MIN", "MAX"))
    parser.add_argument("--free-observations", type=int, default=2)
    args = parser.parse_args()
    low, high = args.floor_offset_range
    if not (0.05 <= args.resolution <= 1 and 1 <= args.max_range <= 6
            and 0.18 <= low < high <= 1.4 and 1 <= args.free_observations <= 20_000):
        parser.error("Resolution, range, floor offsets or free evidence count are outside the supported bounds")
    try:
        manifest = export_map(args)
    except (OSError, sqlite3.Error, ValueError, KeyError, zlib.error) as error:
        print(f"Map export failed: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"manifest": str(args.output_dir / "manifest.json"),
                      "cell_counts": manifest["cell_counts"], "scans_with_floor_fit": manifest["scans_with_floor_fit"]},
                     indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
