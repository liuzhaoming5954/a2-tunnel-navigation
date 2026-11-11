import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch
import zlib

try:
    import numpy as np
except ImportError:
    np = None


if np is not None:
    spec = importlib.util.spec_from_file_location(
        "export_navigation_map", Path(__file__).resolve().parents[1] / "export_navigation_map.py")
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)


@unittest.skipIf(np is None, "The offline navigation map exporter needs NumPy")
class NavigationMapExporterTest(unittest.TestCase):
    def test_compressed_matrix_needs_declared_dimensions(self):
        content = zlib.compress(struct.pack("<5f", 1, 2, 3, 4, 5))
        good = content + struct.pack("<iii", 1, 1, 37)
        self.assertEqual(exporter.matrix(good, 37, "<f4", 5, 10).tolist(), [1, 2, 3, 4, 5])
        bad = content + struct.pack("<iii", 1, 2, 37)
        with self.assertRaisesRegex(ValueError, "does not match"):
            exporter.matrix(bad, 37, "<f4", 5, 10)

    def test_timestamp_join_requires_acknowledged_header_and_reference_interval(self):
        with tempfile.TemporaryDirectory() as directory:
            ledger = Path(directory) / "input.jsonl"
            log = Path(directory) / "dlio.log"
            ledger.write_text(json.dumps({
                "input": 1, "header_ns": 100_000, "audit_scan_end_ns": 120_000, "receipt_ns": 121_000,
            }) + "\n", encoding="utf-8")
            log.write_text(
                "A2_SCAN_SYNC input=1 input_ns=100000 reference_ns=110000 result=published\n", encoding="utf-8")
            provenance = exporter.join_scan_provenance([(7, 110_120)], log, ledger)
            self.assertEqual(provenance[7]["database_stamp_delta_ns"], 120)
            self.assertEqual(provenance[7]["raw_header_ns"], 100_000)
            with self.assertRaisesRegex(ValueError, "No unique synchronized scan"):
                exporter.join_scan_provenance([(7, 111_001)], log, ledger)
            log.write_text(
                "A2_SCAN_SYNC input=1 input_ns=100001 reference_ns=110000 result=published\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "acknowledged raw time interval"):
                exporter.join_scan_provenance([(7, 110_120)], log, ledger)

    def test_free_requires_ground_and_multiple_scans_and_occupied_overrides(self):
        def scan(node, points):
            return node, node, np.eye(3), np.array([0.1, 0.1, 0.0]), np.array(points)

        floor_and_wall = [[2.1, 0.0, -0.5], [3.4, 0.0, 0.0]]
        floor = np.array([0.0, 0.0, -0.5])
        first = scan(1, floor_and_wall)
        second = scan(2, floor_and_wall)
        obstacle = scan(3, [[2.1, 0.0, 0.0]])

        def read_cells(scans):
            with patch.object(exporter, "fit_floor", return_value=(floor, 20)):
                pixels, origin, _, _ = exporter.rasterize(
                    scans, {node: {} for node in (1, 2, 3)}, 1.0, (0.3, 0.8), 4.0, 2)
            return [int(pixels[-1 - int(0.1 - origin[1]), int(x - origin[0])]) for x in (1.1, 2.2, 3.5)]

        self.assertEqual(read_cells([first]), [205, 205, 0])
        self.assertEqual(read_cells([first, second]), [205, 254, 0])
        self.assertEqual(read_cells([first, second, obstacle]), [205, 0, 0])


if __name__ == "__main__":
    unittest.main()
