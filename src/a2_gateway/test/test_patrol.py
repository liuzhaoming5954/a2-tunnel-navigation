"""验证模拟任务与维护遥控使用同一机身，并按到达、停稳、停留的顺序推进。"""

import math
import unittest

from a2_gateway.patrol import ARRIVAL_M, STOP_CONFIRM_S, Patrol, Point
from a2_gateway.simulator import Simulator, TICK_SECONDS


class PatrolTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.sim = Simulator(clock=lambda: self.now)
        self.addCleanup(self.sim.close)
        self.first = dict(mission_id="round-a", sequence=1, x_m=-4.15, y_m=-1.1, dwell_s=0.13)
        self.second = dict(mission_id="round-a", sequence=2, x_m=-3.45, y_m=-1.1, dwell_s=0.39)

    def tick(self):
        self.now += TICK_SECONDS
        return self.sim.patrol_state()

    def run_until(self, phase, limit=5000):
        for _ in range(limit):
            state = self.tick()
            if state["phase"] == phase:
                return state
        self.fail(f"未到达 {phase}，最终状态为 {state}")

    def test_shared_robot_moves_and_waits_at_both_points(self):
        for point in (self.first, self.second):
            self.assertEqual(self.sim.submit_patrol_point(point)[0], 200)
        for point in (self.first, self.second):
            self.run_until("dwelling")
            robot = self.sim.snapshot()
            pose = robot["pose"]
            self.assertLessEqual(math.hypot(pose["x_m"]-point["x_m"], pose["y_m"]-point["y_m"]), ARRIVAL_M)
            self.assertEqual((robot["robot"]["linear_mps"], robot["robot"]["angular_radps"]), (0., 0.))
            self.now += point["dwell_s"] / 2
            self.assertEqual(self.sim.patrol_state()["phase"], "dwelling")
            self.assertEqual(self.sim.snapshot()["pose"]["x_m"], pose["x_m"])
            result = self.run_until("moving" if point["sequence"] == 1 else "completed")
        events = result["events"]
        for seq, dwell in ((1, self.first["dwell_s"]), (2, self.second["dwell_s"])):
            segment = [e for e in events if e["sequence"] == seq and e["event"] != "completed"]
            self.assertEqual([e["event"] for e in segment],
                             ["accepted", "departed", "arrived", "stopped", "dwell_finished"])
            self.assertGreaterEqual(segment[3]["time_s"]-segment[2]["time_s"], STOP_CONFIRM_S-1e-9)
            self.assertGreaterEqual(segment[4]["time_s"]-segment[3]["time_s"], dwell-1e-9)
        self.assertEqual(events[-1]["event"], "completed")
        self.assertEqual(self.sim.acquire({})[0], 200)

    def test_retry_order_and_exclusive_ownership(self):
        self.assertEqual(self.sim.submit_patrol_point(self.second)[1]["reason"], "patrol_out_of_order")
        self.assertEqual(self.sim.patrol_state()["phase"], "idle")
        lease = self.sim.acquire({})[1]["session_id"]
        self.assertEqual(self.sim.submit_patrol_point(self.first)[1]["reason"], "session_busy")
        self.sim.stop({"session_id": lease})
        self.sim.submit_patrol_point(self.first)
        self.assertEqual(self.sim.acquire({})[1]["reason"], "patrol_busy")
        self.assertEqual(self.sim.submit_patrol_point({**self.first, "mission_id": "round-b"})[1]["reason"],
                         "patrol_busy")
        self.assertEqual(self.sim.submit_patrol_point({**self.first, "dwell_s": 3})[1]["reason"],
                         "patrol_point_conflict")
        waiting = self.run_until("waiting")
        self.assertIsNone(waiting["active_sequence"])
        self.assertEqual(self.sim.submit_patrol_point(self.first)[1]["patrol"]["events"], waiting["events"])
        self.assertEqual(self.sim.submit_patrol_point(self.second)[0], 200)
        completed = self.run_until("completed")
        self.assertEqual(self.sim.submit_patrol_point(self.second)[1]["patrol"]["events"], completed["events"])
        self.assertEqual(self.sim.submit_patrol_point({**self.first, "mission_id": "round-b"})[0], 200)
        self.assertEqual(self.sim.submit_patrol_point(self.first)[1]["reason"], "patrol_finished")

    def test_invalid_targets_and_input_are_rejected_without_progress(self):
        for patch in ({"x_m": True}, {"dwell_s": -1}, {"x_m": math.inf}, {"dwell_s": math.nan},
                      {"sequence": False}, {"sequence": 3}, {"mission_id": ""}, {"y_m": 10**500}):
            self.assertEqual(self.sim.submit_patrol_point({**self.first, **patch})[0], 400)
        for x, y in ((0, -8), (1e308, 0)):
            self.assertEqual(self.sim.submit_patrol_point({**self.first, "x_m": x, "y_m": y})[0], 422)
        self.assertEqual(self.sim.patrol_state()["phase"], "idle")

    def test_path_blocked_by_wall_stops_without_marking_arrival(self):
        # 终点位于上方横廊内，从初始位置指向它的直线会撞上两廊之间的墙。
        point = {**self.first, "x_m": 0, "y_m": 5.4}
        self.assertEqual(self.sim.submit_patrol_point(point)[0], 200)
        failed = self.run_until("failed")
        self.assertEqual(failed["reason"], "obstacle")
        self.assertNotIn("arrived", [e["event"] for e in failed["events"]])
        self.assertEqual(self.sim.snapshot()["robot"]["mode"], "standing")

    def test_stationary_window_restarts_on_residual_motion(self):
        route = Patrol()
        route.accept("r", Point(1, 0., 0., 1.), 0.)
        def observe(now, angular=0.):
            return route.update(now, 0., 0., 0., 0., angular, .4, 1.)
        observe(1.)
        observe(1.15, 0.1)
        observe(1.16)
        observe(1.31)
        self.assertEqual(route.phase, "settling")
        observe(1.37)
        self.assertEqual(route.phase, "dwelling")
        self.assertAlmostEqual(route.snapshot(1.47)["remaining_s"], .9)


if __name__ == "__main__":
    unittest.main()
