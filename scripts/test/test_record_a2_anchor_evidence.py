from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from scripts.record_a2_anchor_evidence import (
    FitEvidence, check_reference_epoch, inspect_fit_geometry, load_samples, save_samples,
)


class AnchorEvidenceTest(unittest.TestCase):
    def test_sidecar_preserves_scans_pose_and_timestamp(self):
        samples = [(np.full((2, 3), i, dtype=np.float32), np.eye(4),
                    {"header_ns": 100 + i * 50, "odom_reference_ns": 110 + i * 50})
                   for i in range(5)]
        raw_clouds = {meta["header_ns"]: bytes([i, i + 1]) for i, (_, _, meta) in enumerate(samples)}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "selected.npz"
            save_samples(path, samples, raw_clouds, {"map_database_sha256": "expected"})
            loaded, provenance = load_samples(path)
            self.assertEqual(provenance, {"map_database_sha256": "expected"})
            for original, restored in zip(samples, loaded):
                np.testing.assert_array_equal(original[0], restored[0])
                np.testing.assert_array_equal(original[1], restored[1])
                self.assertEqual(restored[2]["header_ns"], original[2]["header_ns"])
                self.assertEqual(restored[2]["odom_reference_ns"], original[2]["odom_reference_ns"])
            with np.load(path, allow_pickle=False) as sidecar:
                self.assertEqual(sidecar["cloud_cdr_2"].tobytes(), b"\x02\x03")
            with self.assertRaises(FileExistsError):
                save_samples(path, samples, raw_clouds, {})

    def test_observer_records_rejections_without_changing_verifier_result(self):
        source, reference = np.ones((2, 3)), np.zeros((2, 3))
        nodes = [(25, None, reference)]

        def register(_src, _ref, _src_tree, _ref_tree, _normals, _planar, yaw):
            if yaw == 180:
                return None
            overlap = 0.82 if yaw == 0 else 0.72
            return {"inlier_fraction_0p20m": [0.90, overlap], "median_distance_m": [0.07, 0.10],
                    "translation_observability_ratio": 0.051, "planar_support": 155,
                    "pass_geometric_gate": True, "source_to_reference": np.eye(4).tolist()}

        search = SimpleNamespace(verify_fit=register)
        accepted = [(np.eye(4), {"node_id": 25, "yaw_seed_deg": 0})]
        ranked = [{"node_id": 25, "descriptor_score": 0.78}]

        def fit_candidates(points, _pose, refs, _spectra):
            for yaw in (0, 12, 180):
                search.verify_fit(points, refs[0][2], None, None, None, None, yaw)
            return accepted, ranked

        verifier = SimpleNamespace(search=search, fit_candidates=fit_candidates,
                                   MIN_INLIER_FRACTION=.78, MAX_MEDIAN_DISTANCE_M=.13, MIN_OBSERVABILITY=.045)
        observer = FitEvidence(verifier, [(source, np.eye(4), {"header_ns": 100})])
        result = observer.fit_candidates(source, np.eye(4), nodes, None)
        self.assertIs(result[0], accepted)
        self.assertIs(result[1], ranked)
        self.assertIs(search.verify_fit, register)
        attempts = observer.frames[0]["attempts"]
        self.assertEqual([row["result"] for row in attempts], ["accepted_fit", "rejected_fit", "no_rigid_fit"])
        self.assertEqual(attempts[1]["failed_gates"], ["bidirectional_overlap"])

    def test_fit_geometry_reports_weak_axis_in_map_frame(self):
        # Two perpendicular planes each extend along reference X.
        source = np.array([
            [1., 0., 1.], [2., 0., 1.], [1., 0., 2.], [2., 0., 2.],
            [1., 1., 0.], [2., 1., 0.], [1., 2., 0.], [2., 2., 0.],
        ])
        normals = np.repeat([[0., 1., 0.], [0., 0., 1.]], 4, axis=0)
        map_from_reference = np.array([
            [0., -1., 0., 2.], [1., 0., 0., 3.], [0., 0., 1., 4.], [0., 0., 0., 1.],
        ])
        source_to_reference = np.eye(4)
        source_to_reference[0, 3] = .25
        reference = (25, map_from_reference, source, None, normals, np.ones(len(source), dtype=bool))
        search = SimpleNamespace(
            LEVEL=np.eye(3),
            nearest=lambda _tree, points: (np.arange(len(points))[:, None], np.zeros((len(points), 1))),
        )
        geometry = inspect_fit_geometry(search, source, reference, source_to_reference.tolist())
        np.testing.assert_allclose(geometry["map_from_scan"], map_from_reference @ source_to_reference)
        np.testing.assert_allclose(np.abs(geometry["weakest_translation_axis_in_map"]), [0., 1., 0.])
        eigenvalues = geometry["normalized_translation_information_eigenvalues"]
        self.assertAlmostEqual(eigenvalues[0], 0.)
        self.assertAlmostEqual(sum(eigenvalues), 1.)
        self.assertGreater(eigenvalues[1], 0.)

    def test_supplement_requires_exact_pairs_in_one_dlio_epoch(self):
        sample = (None, None, {"header_ns": 200, "odom_reference_ns": 210})
        anchor_pairs = [(100, 110), (150, 160)]

        def row(index, raw, ref):
            return f"A2_SCAN_SYNC input={index} pairs=1 input_ns={raw} reference_ns={ref} result=published\n"

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "dlio.log"
            prefix = row(5, 100, 110) + row(6, 150, 160)
            path.write_text(prefix + row(7, 200, 210))
            check_reference_epoch(path, anchor_pairs, [sample])
            path.write_text(prefix + row(7, 199, 210))
            with self.assertRaisesRegex(ValueError, "exact pairs"):
                check_reference_epoch(path, anchor_pairs, [sample])
            path.write_text(prefix + row(1, 200, 210))
            with self.assertRaisesRegex(ValueError, "epoch boundary"):
                check_reference_epoch(path, anchor_pairs, [sample])
