import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from a2_navigation.offline_planner import load_map, plan, read_pgm, search


ROOT = Path(__file__).resolve().parents[3]


class OfflinePlannerTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def write_map(self, width, height, cells=None, **changes):
        cells = cells or {}
        image = bytes(cells.get((x, y), 254)
                      for y in range(height - 1, -1, -1) for x in range(width))
        (self.root / "map.pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode() + image)
        metadata = {
            "image": "map.pgm", "resolution": 1.0, "origin": [0.0, 0.0, 0.0],
            "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196, "mode": "trinary",
        }
        path = self.root / "map.yaml"
        path.write_text(json.dumps({**metadata, **changes}), encoding="utf-8")
        return path

    def test_raster_origin_rotation_and_unknown(self):
        grid = load_map(self.write_map(5, 4, {(1, 3): 0, (2, 0): 205},
                                      origin=[10, -2, math.pi / 2], resolution=0.5))
        self.assertEqual(grid.known_free[3] & (1 << 1), 0)
        self.assertEqual(grid.known_free[0] & (1 << 2), 0)
        self.assertTrue(grid.known_free[0] & (1 << 1))
        self.assertEqual(grid.world_to_cell(grid.cell_center((1, 3))), (1, 3))
        x, y = grid.cell_center((1, 3))
        self.assertAlmostEqual(x, 8.25)
        self.assertAlmostEqual(y, -1.25)
        self.assertEqual(len(grid.image_sha256), 64)

    def test_pgm_binary_delimiter_preserves_pixel_and_ascii_comments(self):
        raster = bytes((10, 32, 35))
        self.assertEqual(read_pgm(b"P5\n3 1\n255\n" + raster), (3, 1, 255, raster))
        self.assertEqual(read_pgm(b"P5\r\n3 1\r\n255\r\n" + raster), (3, 1, 255, raster))
        self.assertEqual(read_pgm(b"P2\n# map\n3 1 255\n10 32 # cell\n35"), (3, 1, 255, raster))
        with self.assertRaisesRegex(ValueError, "raster"):
            read_pgm(b"P5\n3 1\n255\n" + raster[:2])

    def test_wall_detour_and_exact_requested_endpoints(self):
        wall = {(5, y): 0 for y in range(8)}
        grid = load_map(self.write_map(11, 12, wall))
        start, goal = (2.3, 2.4), (8.7, 2.5)
        result = plan(grid, start, goal, 0.2)
        self.assertEqual(result["status"], "planned")
        self.assertEqual(result["path_m"][0], start)
        self.assertEqual(result["path_m"][-1], goal)
        self.assertGreater(result["path_length_m"], math.dist(start, goal))
        self.assertTrue(all(y >= 9 for x, y in result["path_cells"] if x == 5))

    def test_unknown_cells_separate_free_components(self):
        wall = {(5, y): 205 for y in range(10)}
        grid = load_map(self.write_map(11, 10, wall))
        self.assertEqual(plan(grid, (2.5, 2.5), (8.5, 2.5), 0.2)["status"], "unreachable")

    def test_no_diagonal_between_blocked_corners(self):
        grid = load_map(self.write_map(2, 2, {(0, 1): 0, (1, 0): 0}))
        self.assertEqual(search(grid, grid.known_free, (0, 0), (1, 1)), [])

    def test_rejects_invalid_start_goal_via_and_clearance(self):
        grid = load_map(self.write_map(10, 10, {(5, 5): 0, (7, 7): 205}))
        result = plan(grid, (-1, 2.5), (3.5, 3.5), 0.2)
        self.assertEqual((result["status"], result["reason"]), ("invalid_start", "outside_map"))
        for goal in ((5.5, 5.5), (7.5, 7.5)):
            result = plan(grid, (2.5, 2.5), goal, 0.2)
            self.assertEqual((result["status"], result["reason"]), ("invalid_goal", "not_known_free"))
        for goal in ((4.5, 5.5), (0.5, 3.5)):
            result = plan(grid, (2.5, 2.5), goal, 0.2)
            self.assertEqual((result["status"], result["reason"]), ("invalid_goal", "insufficient_clearance"))
        result = plan(grid, (2.5, 2.5), (3.5, 3.5), 0.2, via=[(7.5, 7.5)])
        self.assertEqual((result["status"], result["endpoint_index"]), ("invalid_via", 1))

    def test_waypoint_and_same_cell_path(self):
        grid = load_map(self.write_map(10, 10))
        start, via, goal = (2.2, 2.2), (3.7, 5.7), (7.3, 2.8)
        result = plan(grid, start, goal, 0.2, via=[via])
        self.assertEqual(result["status"], "planned")
        self.assertIn(via, result["path_m"])
        self.assertEqual(result["path_m"][-1], goal)
        same_cell = plan(grid, start, (2.7, 2.9), 0.2)
        self.assertEqual(same_cell["path_m"], [start, (2.7, 2.9)])
        self.assertAlmostEqual(same_cell["path_length_m"], math.dist(start, (2.7, 2.9)))

    def test_cli_returns_plan_or_rejection_and_keeps_existing_report(self):
        map_path = self.write_map(10, 10)
        output = self.root / "result.json"
        command = [sys.executable, str(ROOT / "scripts/plan_navigation_goal.py"), "--map", str(map_path),
                   "--start", "2.5", "2.5", "--footprint-radius-m", "0.2"]
        result = subprocess.run(command + ["--goal", "7.5", "7.5", "--output", str(output)],
                                check=False, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(output.read_text())["status"], "planned")
        previous = output.read_bytes()
        result = subprocess.run(command + ["--goal", "8.5", "8.5", "--output", str(output)],
                                check=False, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(output.read_bytes(), previous)
        result = subprocess.run(command + ["--goal", "nan", "7.5"], check=False,
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(json.loads(result.stdout)["status"], "invalid_input")


if __name__ == "__main__":
    unittest.main()
