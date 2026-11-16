#!/usr/bin/env python3
"""Record or replay an A2 map-anchor probe with scan and fit evidence."""

import argparse
from collections import OrderedDict
from contextlib import contextmanager
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import sys
from types import SimpleNamespace


SUPPLEMENTAL_VOXEL_M = .40
PUBLISHED_PAIR = re.compile(
    r"\bA2_SCAN_SYNC input=(\d+) pairs=\d+ input_ns=(\d+) reference_ns=(\d+).*?\bresult=published\b"
)


@contextmanager
def observe_attribute(owner, name, wrapper):
    original = getattr(owner, name)
    setattr(owner, name, wrapper(original))
    try:
        yield
    finally:
        setattr(owner, name, original)


def sha256(path):
    with Path(path).open("rb") as stream:
        checksum = hashlib.sha256()
        for block in iter(lambda: stream.read(1 << 20), b""):
            checksum.update(block)
    return checksum.hexdigest()


def load_probe(path):
    # The probe is executable code from the trusted local diagnostic bundle.
    spec = importlib.util.spec_from_file_location("a2_recorded_anchor_probe", path)
    if spec is None or spec.loader is None:
        raise ValueError("Expected a Python anchor probe file")
    probe = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(path.parent))
    try:
        spec.loader.exec_module(probe)
    finally:
        sys.path.pop(0)
    return probe


def save_samples(path, samples, raw_clouds, provenance):
    import numpy as np

    payload = {
        "local_poses": np.asarray([pose for _, pose, _ in samples]),
        "raw_header_ns": np.asarray([meta["header_ns"] for _, _, meta in samples], dtype=np.int64),
        "odom_reference_ns": np.asarray([meta["odom_reference_ns"] for _, _, meta in samples], dtype=np.int64),
        "provenance_json": np.asarray(json.dumps(provenance, allow_nan=False)),
    }
    for i, (points, _, meta) in enumerate(samples):
        payload[f"raw_xyz_{i}"] = points  # Original LiDAR coordinates after the probe's range/finite audit.
        payload[f"cloud_cdr_{i}"] = np.frombuffer(raw_clouds[meta["header_ns"]], dtype=np.uint8)
    with path.open("xb") as stream:
        np.savez_compressed(stream, **payload)


def load_samples(path):
    import numpy as np

    with np.load(path, allow_pickle=False) as payload:
        poses = payload["local_poses"]
        headers = payload["raw_header_ns"]
        references = payload["odom_reference_ns"]
        provenance = json.loads(payload["provenance_json"].item()) if "provenance_json" in payload else None
        if (references.ndim != 1 or headers.shape != references.shape or poses.shape != (len(headers), 4, 4)
                or not 5 <= len(headers) <= 8 or references.dtype.kind not in "iu" or headers.dtype.kind not in "iu"
                or not np.isfinite(poses).all()):
            raise ValueError("Invalid anchor sample shapes or timestamps")
        samples = []
        for i, (header, reference) in enumerate(zip(headers.tolist(), references.tolist())):
            points = payload[f"raw_xyz_{i}"]
            if (not 0 < header < reference or (samples and (
                    header <= samples[-1][2]["header_ns"] or reference <= samples[-1][2]["odom_reference_ns"]))
                    or points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()):
                raise ValueError("Invalid or unordered anchor sample")
            samples.append((points, poses[i], {
                "header_ns": header, "odom_reference_ns": reference,
                "reference_s": (reference - int(references[0])) / 1_000_000_000,
                "points_after_finite_and_range_filter": len(points),
            }))
    return samples, provenance


def inspect_fit_geometry(search, source, reference, source_to_reference):
    """Report the translation information of the fitted scan in map axes."""
    import numpy as np

    _, map_from_reference, _, tree, normals, planar = reference
    source_to_reference = np.asarray(source_to_reference)
    level = search.LEVEL
    rotation = level @ source_to_reference[:3, :3] @ level.T
    translation = level @ source_to_reference[:3, 3]
    projected = source @ rotation.T + translation
    indices, distances = search.nearest(tree, projected)
    indices = indices.ravel()
    keep = (distances.ravel() <= .20) & planar[indices]
    points = projected[keep]
    geometry = {"map_from_scan": (map_from_reference @ source_to_reference).tolist(),
                "normalized_translation_information_eigenvalues": None,
                "weakest_translation_axis_in_map": None}
    if len(points) < 3:
        return geometry

    matched_normals = normals[indices[keep]]
    lever = np.cross(points - points.mean(axis=0), matched_normals)
    mixed = matched_normals.T @ lever
    information = matched_normals.T @ matched_normals - mixed @ np.linalg.pinv(lever.T @ lever) @ mixed.T
    eigenvalues, axes = np.linalg.eigh(information)
    eigenvalues = np.maximum(eigenvalues, 0.)
    if eigenvalues.sum() > 0:
        geometry["normalized_translation_information_eigenvalues"] = (eigenvalues / eigenvalues.sum()).tolist()
        weakest = map_from_reference[:3, :3] @ level.T @ axes[:, 0]
        # Both signs describe the same unobservable direction.
        geometry["weakest_translation_axis_in_map"] = weakest.tolist()
    return geometry


class FitEvidence:
    """Observe actual registration results and leave verifier acceptance unchanged."""

    def __init__(self, verifier, samples, include_geometry=False):
        self.verifier = verifier
        self.headers = {id(points): meta["header_ns"] for points, _, meta in samples}
        self.frames = []
        self.include_geometry = include_geometry

    def __getattr__(self, name):
        return getattr(self.verifier, name)

    def fit_candidates(self, points, epoch_from_scan, references, map_spectra):
        references_by_cloud = {id(row[2]): row for row in references}
        attempts = []

        def observe_fit(original):
            def observe(source, reference, source_tree, reference_tree, normals, planar, initial_yaw):
                fit = original(source, reference, source_tree, reference_tree, normals, planar, initial_yaw)
                reference_row = references_by_cloud[id(reference)]
                item = {"node_id": reference_row[0], "yaw_seed_deg": initial_yaw}
                if fit is None:
                    item["result"] = "no_rigid_fit"
                else:
                    item.update({key: value for key, value in fit.items() if key != "source_to_reference"})
                    if self.include_geometry:
                        item["geometry"] = inspect_fit_geometry(
                            self.verifier.search, source, reference_row, fit["source_to_reference"],
                        )
                attempts.append(item)
                return fit
            return observe

        with observe_attribute(self.verifier.search, "verify_fit", observe_fit):
            matches, ranked = self.verifier.fit_candidates(points, epoch_from_scan, references, map_spectra)
        accepted = {(row["node_id"], row["yaw_seed_deg"]) for _, row in matches}
        for item in attempts:
            if item.get("result") == "no_rigid_fit":
                continue
            if (item["node_id"], item["yaw_seed_deg"]) in accepted:
                item["result"] = "accepted_fit"
                continue
            item["result"] = "rejected_fit"
            item["failed_gates"] = [
                name for name, passed in (
                    ("geometric_gate", item["pass_geometric_gate"]),
                    ("bidirectional_overlap", min(item["inlier_fraction_0p20m"]) >= self.MIN_INLIER_FRACTION),
                    ("median_distance", max(item["median_distance_m"]) <= self.MAX_MEDIAN_DISTANCE_M),
                    ("translation_observability", item["translation_observability_ratio"] >= self.MIN_OBSERVABILITY),
                ) if not passed
            ]
        self.frames.append({"raw_header_ns": self.headers[id(points)],
                            "ranked_references": ranked, "attempts": attempts})
        return matches, ranked


def verify_with_evidence(retest, verifier, references, samples, include_geometry=False):
    recorder = FitEvidence(verifier, samples, include_geometry)
    result, used = retest.verify_epoch(samples, references, recorder)
    return result, used, {
        "minimum_bidirectional_overlap_at_0p20m": verifier.MIN_INLIER_FRACTION,
        "maximum_median_distance_m": verifier.MAX_MEDIAN_DISTANCE_M,
        "minimum_translation_observability": verifier.MIN_OBSERVABILITY,
        "fit_frames": recorder.frames,
    }


def check_reference_epoch(log_path, anchor_pairs, reference_samples):
    """Require the accepted anchor and the added views in the same logged DLIO epoch."""
    expected = set(anchor_pairs)
    expected.update((meta["header_ns"], meta["odom_reference_ns"]) for _, _, meta in reference_samples)
    first = min(ref for _, ref in expected)
    last = max(ref for _, ref in expected)
    previous = (0, 0, 0)
    with log_path.open(encoding="utf-8") as log:
        for line in log:
            match = PUBLISHED_PAIR.search(line)
            if match is None:
                continue
            pair = tuple(map(int, match.groups()))
            if not first <= pair[2] <= last:
                continue
            if any(old >= new for old, new in zip(previous, pair)):
                raise ValueError("Reference DLIO log crosses an epoch boundary or has unordered pairs")
            previous = pair
            expected.discard(pair[1:])
    if expected:
        raise ValueError("Anchor and supplemental scans must have exact pairs in the reference DLIO log")


class SupplementalViews:
    """Replay earlier mapped views alongside the original independent shortlist."""

    def __init__(self, probe, verifier, base, samples, map_from_odom):
        import numpy as np

        self.verifier = verifier
        self.base = base
        self.base_filter = verifier.filter_query
        self.fixed_queries = {}
        self.nodes = set()
        rows, descriptors = [], []
        next_id = max(10_000, max(row[0] for row in base[0]) + 1)
        for node, (points, odom_from_scan, _) in enumerate(samples, start=next_id):
            cloud = self.coarse_query(points)
            if len(cloud) < 120:
                raise ValueError("A supplemental reference has fewer than 120 filtered points")
            tree = probe.cv2.flann_Index(cloud, {"algorithm": 4})
            rows.append((node, map_from_odom @ odom_from_scan, cloud, tree,
                         *verifier.search.surface_normals(cloud, tree)))
            descriptors.append(verifier.search.descriptor(cloud))
            self.nodes.add(node)
        self.supplement = rows, np.fft.rfft(descriptors, axis=1)
        self.references = base[0] + rows, np.concatenate((base[1], self.supplement[1]))

    def __getattr__(self, name):
        return getattr(self.verifier, name)

    def coarse_query(self, points):
        import numpy as np

        _, groups, counts = np.unique(np.floor(points / SUPPLEMENTAL_VOXEL_M).astype(np.int32),
                                      axis=0, return_inverse=True, return_counts=True)
        centers = np.asarray([np.bincount(groups, weights=axis) / counts for axis in points.T],
                             dtype=np.float32).T
        leveled = centers @ self.verifier.search.LEVEL.T
        distance = np.linalg.norm(leveled, axis=1)
        keep = (distance > .8) & (distance < 18.) & (leveled[:, 2] > -.3) & (leveled[:, 2] < 3.5)
        return np.ascontiguousarray(leveled[keep], dtype=np.float32)

    def fit_candidates(self, points, epoch_from_scan, _references, _spectra):
        base_matches, base_ranked = self.verifier.fit_candidates(points, epoch_from_scan, *self.base)
        with observe_attribute(self.verifier, "filter_query", lambda _: self.coarse_query):
            matches, ranked = self.verifier.fit_candidates(points, epoch_from_scan, *self.supplement)
        return base_matches + matches, base_ranked + ranked

    def filter_query(self, points):
        filtered = self.base_filter(points)
        # The epoch verifier passes the filtered array back during fixed checks.
        self.fixed_queries[id(filtered)] = filtered, self.coarse_query(points)
        return filtered

    def fixed_check(self, original):
        def check(points, map_from_scan, reference, verifier):
            if reference[0] in self.nodes:
                points = self.fixed_queries[id(points)][1]
                if len(points) < 120:
                    # The verifier sorts fixed fits by median, including failures.
                    return {"node_id": reference[0], "fixed_inlier_fraction_0p20m": [0., 0.],
                            "median_distance_m": [sys.float_info.max] * 2, "pass": False,
                            "reason": "insufficient_supplemental_query_points"}
            return original(points, map_from_scan, reference, verifier)
        return check


def replay_with_supplement(args, probe, retest, verifier, references, samples, provenance):
    import numpy as np

    prior, prior_provenance = load_samples(args.reference_samples)
    report = json.loads(args.reference_anchor.read_text(encoding="utf-8"))
    alignment = report["alignment"]
    sources = (prior_provenance, report.get("evidence_provenance"))
    if (report.get("status") != "diagnostic_anchored_stream_observed"
            or report.get("map_sha256") != provenance["map_database_sha256"]
            or alignment.get("accepted") is not True or alignment.get("decision") != "map_handoff_verified"
            or any(source is None or any(source.get(key) != provenance[key]
                                        for key in ("map_database_sha256", "map_poses_sha256"))
                   for source in sources)):
        raise ValueError("Supplemental scans require a verified anchor for the exact replay map")
    if (prior[-1][2]["odom_reference_ns"] >= samples[0][2]["header_ns"]
            and samples[-1][2]["odom_reference_ns"] >= prior[0][2]["header_ns"]):
        raise ValueError("Supplemental and query scan intervals must be disjoint")
    anchor_pairs = [(row["sensor_scan_start_ns"], row["sensor_reference_ns"])
                    for row in report["poses_after_alignment"]]
    if len(anchor_pairs) < 2 or max(ref for _, ref in anchor_pairs) >= prior[0][2]["header_ns"]:
        raise ValueError("Supplemental scans must follow the verified anchor observations")
    check_reference_epoch(args.reference_dlio_log, anchor_pairs, prior)
    matrix = np.asarray(alignment["T_map_from_fresh_odom"], dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0., 0., 0., 1.])
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
            or np.linalg.det(matrix[:3, :3]) <= 0):
        raise ValueError("Invalid supplemental map anchor transform")
    added = SupplementalViews(probe, verifier, references, prior, matrix)
    with observe_attribute(retest, "fixed_scan_fit", added.fixed_check):
        result, _, evidence = verify_with_evidence(retest, added, added.references, samples, args.geometry)
    evidence["supplemental_views"] = {
        "voxel_m": SUPPLEMENTAL_VOXEL_M,
        "base_reference_count": len(references[0]),
        "nodes": [{"node_id": node, "raw_header_ns": row[2]["header_ns"]}
                  for node, row in zip(sorted(added.nodes), prior)],
        "reference_samples_sha256": sha256(args.reference_samples),
        "reference_anchor_sha256": sha256(args.reference_anchor),
        "reference_dlio_log_sha256": sha256(args.reference_dlio_log),
    }
    return result, evidence


def run_live(args, probe, retest, verifier, references, provenance):
    from rclpy.serialization import serialize_message

    clouds = OrderedDict()
    evidence = None

    def audit_scan(original):
        def observe(message, reference, imu):
            result = original(message, reference, imu)
            clouds[probe.stamp_ns(message)] = serialize_message(message)
            if len(clouds) > 16:
                clouds.popitem(last=False)
            return result
        return observe

    def verify_epoch(samples, reference_data, original_verifier):
        nonlocal evidence
        save_samples(args.samples_output, samples, clouds, provenance)
        result, used, evidence = verify_with_evidence(retest, original_verifier, reference_data, samples)
        return result, used

    # The collector calls verify_epoch only after completing its IMU/timing checks.
    recording_retest = SimpleNamespace(FRAME_SPACING_NS=retest.FRAME_SPACING_NS, verify_epoch=verify_epoch)
    args.selected_samples_output = None  # The wrapper owns persistence, including full CDR.
    with observe_attribute(probe, "audit_raw_scan", audit_scan):
        report = probe.collect(args, recording_retest, verifier, references)
    report.update({
        "anchor_evidence": evidence,
        "selected_samples_sidecar": str(args.samples_output) if evidence is not None else None,
        "evidence_provenance": provenance,
    })
    return report, 0 if report.get("status") == "diagnostic_anchored_stream_observed" else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    live = commands.add_parser("live", help="Capture evidence during a stationary anchor probe")
    replay = commands.add_parser("replay", help="Re-run verification on saved scans without a robot")
    for command in (live, replay):
        command.add_argument("--probe", type=Path, required=True, help="Trusted diagnostic probe .py file")
        command.add_argument("--helper-root", type=Path, help="Defaults to helpers/ beside the probe")
        command.add_argument("--map-database", type=Path, required=True)
        command.add_argument("--map-poses", type=Path, required=True)
        command.add_argument("--output", type=Path, required=True)
    live.add_argument("--dlio-log", type=Path, required=True)
    live.add_argument("--samples-output", type=Path, required=True)
    live.add_argument("--namespace", default="/a2_probe")
    live.add_argument("--duration-s", type=float, default=25.0)
    live.add_argument("--after-anchor-s", type=float, default=4.0)
    live.add_argument("--anchor-samples", type=int, choices=range(5, 9), default=8)
    replay.add_argument("--samples", type=Path, required=True)
    replay.add_argument("--geometry", action="store_true",
                        help="Include each fitted LiDAR pose and weak translation axis in map coordinates")
    replay.add_argument("--reference-samples", type=Path, help="Earlier scan sidecar for an offline map-view trial")
    replay.add_argument("--reference-anchor", type=Path, help="Verified anchor from the earlier DLIO session")
    replay.add_argument("--reference-dlio-log", type=Path, help="DLIO log covering that anchor and those views")
    args = parser.parse_args(argv)
    if args.command == "live":
        if not 0 < args.after_anchor_s < args.duration_s <= 30:
            parser.error("Choose a duration of at most 30 s and a shorter after-anchor window")
        if args.samples_output.resolve() == args.output.resolve():
            parser.error("Use separate files for the report and scans")
    elif len([path for path in (args.reference_samples, args.reference_anchor, args.reference_dlio_log)
              if path is not None]) not in (0, 3):
        parser.error("Specify all three supplemental reference inputs together")
    targets = (args.output, args.samples_output) if args.command == "live" else (args.output,)
    if any(path.exists() for path in targets):
        parser.error("Output paths must be new files")
    try:
        probe_path = args.probe.resolve(strict=True)
        probe = load_probe(probe_path)
        probe.cv2.setNumThreads(1)
        helpers = (args.helper_root or probe_path.parent / "helpers").resolve()
        retest, verifier, references = probe.load_map_inputs(
            args.map_database.resolve(), args.map_poses.resolve(), helpers,
        )
        provenance = {
            "map_database_sha256": sha256(args.map_database),
            "map_poses_sha256": sha256(args.map_poses),
            "probe_sha256": sha256(probe_path),
            "verifier_sha256": sha256(verifier.__file__),
            "epoch_verifier_sha256": sha256(retest.__file__),
            "registration_sha256": sha256(verifier.search.__file__),
        }
        if args.command == "live":
            report, exit_status = run_live(args, probe, retest, verifier, references, provenance)
        else:
            samples, previous = load_samples(args.samples)
            if previous is not None and any(previous.get(key) != provenance[key] for key in (
                    "map_database_sha256", "map_poses_sha256")):
                raise ValueError("Replay map differs from the captured map")
            if args.reference_samples is not None:
                result, evidence = replay_with_supplement(
                    args, probe, retest, verifier, references, samples, provenance,
                )
            else:
                result, _, evidence = verify_with_evidence(retest, verifier, references, samples, args.geometry)
            report = {
                "scope": "offline evidence replay",
                "alignment": result,
                "anchor_evidence": evidence,
                "selected_samples_sha256": sha256(args.samples),
                "sample_provenance": previous,
                "evidence_provenance": provenance,
            }
            exit_status = 0 if result["accepted"] else 1
        with args.output.open("x", encoding="utf-8") as output:
            json.dump(report, output, allow_nan=False, indent=2)
            output.write("\n")
        return exit_status
    except (AttributeError, ImportError, KeyError, OSError, TypeError, ValueError) as error:
        print(f"A2 anchor evidence: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
