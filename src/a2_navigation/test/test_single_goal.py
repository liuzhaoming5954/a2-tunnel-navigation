"""单点派发、取消、失联和进程重建均用可控的导航端验证，不依赖机器人。"""

import math
from pathlib import Path
import sqlite3
import tempfile
import threading
import time
import unittest

from a2_navigation.single_goal import SingleGoalController


DIGEST = "b" * 64


def target(ident="g1", **overrides):
    return {"goal_id": ident, "map_sha256": DIGEST, "frame_id": "map",
            "x_m": 1.0, "y_m": -2.0, "yaw_rad": 0.4, **overrides}


class Backend:
    def __init__(self):
        self.sends = []
        self.cancel_ids = []
        self.ready = True
        self.notify = None
        self.cancel_seen = threading.Event()

    def available(self):
        return self.ready

    def send(self, point, notify):
        self.sends.append(point)
        self.notify = notify

    def cancel(self, ident):
        self.cancel_ids.append(ident)
        self.cancel_seen.set()

    def close(self):
        pass


class GoalTests(unittest.TestCase):
    def make(self, backend=None, **kw):
        b = backend or Backend()
        c = SingleGoalController(DIGEST, b, **kw)
        self.addCleanup(c.close)
        return c, b

    def test_acceptance_is_not_arrival_and_retries_are_idempotent(self):
        c, b = self.make()
        self.assertEqual(c.status()[1]["goal"]["phase"], "idle")
        self.assertEqual(c.submit(target())[1]["goal"]["phase"], "sending")
        self.assertEqual(c.submit(target())[0], 200)
        self.assertEqual(c.submit(target(x_m=99))[1]["reason"], "goal_id_conflict")
        self.assertEqual(c.submit(target("g2"))[1]["reason"], "goal_busy")
        b.notify("accepted")
        b.notify("feedback", 0.6)
        self.assertEqual(c.status()[1]["goal"]["distance_remaining_m"], 0.6)
        b.notify("succeeded")
        old = b.notify
        self.assertEqual(c.submit(target("g2"))[0], 200)
        old("aborted")
        self.assertEqual(c.submit(target())[1]["goal"]["phase"], "succeeded")
        self.assertEqual(c.status()[1]["goal"]["phase"], "sending")
        b.notify("rejected")
        self.assertEqual(c.status()[1]["goal"]["phase"], "failed")
        self.assertEqual(len(b.sends), 2)

    def test_cancel_before_accept_and_duplicate_cancel(self):
        c, b = self.make()
        c.submit(target())
        self.assertEqual(c.cancel({"goal_id": "other"})[0], 409)
        self.assertEqual(c.cancel({"goal_id": "g1"})[1]["goal"]["phase"], "canceling")
        self.assertEqual(b.cancel_ids, [])
        b.notify("accepted")
        c.cancel({"goal_id": "g1"})
        self.assertEqual(b.cancel_ids, ["g1"])
        self.assertEqual(c.status()[1]["goal"]["phase"], "canceling")
        b.notify("canceled")
        self.assertEqual(c.status()[1]["goal"]["phase"], "canceled")

    def test_unconfirmed_send_blocks_new_goal_and_cancels_late_acceptance(self):
        c, b = self.make(ack_timeout_s=0.03)
        c.submit(target())
        for _ in range(40):
            if c.status()[1]["goal"]["phase"] == "uncertain":
                break
            time.sleep(0.01)
        self.assertEqual(c.status()[1]["goal"]["phase"], "uncertain")
        self.assertEqual(c.submit(target("other"))[0], 409)
        b.notify("accepted")
        self.assertEqual(b.cancel_ids, ["g1"])
        b.notify("canceled")
        self.assertEqual(c.status()[1]["goal"]["phase"], "canceled")

    def test_running_readiness_loss_issues_cancel_without_http_poll(self):
        invalid = threading.Event()
        c, b = self.make(readiness=lambda: "pose_stale" if invalid.is_set() else None)
        c.submit(target())
        b.notify("accepted")
        invalid.set()
        self.assertTrue(b.cancel_seen.wait(1))
        self.assertEqual(c.status()[1]["goal"]["phase"], "canceling")
        b.notify("canceled")
        self.assertEqual(c.status()[1]["goal"]["reason"], "pose_stale")
        self.assertEqual(c.status()[1]["goal"]["phase"], "failed")
        self.assertEqual(c.submit(target("other"))[1]["reason"], "pose_stale")

    def test_input_validation_and_nav2_unavailable_do_not_dispatch(self):
        c, b = self.make()
        self.assertEqual(c.submit(target(map_sha256="a"*64))[0], 409)
        for bad in [target(frame_id="odom"), target(x_m=True), target(y_m=math.nan),
                    target(y_m=10**500), target(yaw_rad=4.0), target(extra=42), target("has spaces")]:
            self.assertEqual(c.submit(bad)[0], 400, str(bad))
        b.ready = False
        self.assertEqual(c.submit(target())[1]["reason"], "nav2_unavailable")
        self.assertEqual(b.sends, [])

    def test_restart_replays_terminal_and_does_not_adopt_or_resend_live_goal(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "state.sqlite"
            c, b = self.make(journal_path=p)
            c.submit(target())
            b.notify("rejected")
            with self.assertRaises(BlockingIOError):
                SingleGoalController(DIGEST, Backend(), journal_path=p)
            c.close()
            recovered, b = self.make(journal_path=p)
            self.assertEqual(recovered.submit(target())[1]["goal"]["phase"], "failed")
            self.assertEqual(b.sends, [])
            recovered.close()
            new_map = SingleGoalController("c"*64, Backend(), journal_path=p)
            try:
                self.assertEqual(new_map.status()[1]["goal"]["phase"], "idle")
                self.assertEqual(new_map.status()[1]["goal"]["map_sha256"], "c"*64)
            finally:
                new_map.close()
            # 模拟提交后的断电，磁盘留有一条已发出而尚未收到终态的记录。
            with sqlite3.connect(p) as db:
                db.execute("UPDATE goals SET phase='sending'")
            active, b = self.make(journal_path=p)
            self.assertEqual(active.status()[1]["goal"]["reason"], "restart_outcome_unknown")
            self.assertEqual(active.submit(target("new"))[1]["reason"], "goal_busy")
            self.assertEqual(b.sends, [])
            active.close()
            other_map = SingleGoalController("c"*64, Backend(), journal_path=p)
            try:
                self.assertEqual(other_map.status()[1]["goal"]["map_sha256"], DIGEST)
                self.assertEqual(other_map.submit(target("other", map_sha256="c"*64))[1]["reason"], "goal_busy")
            finally:
                other_map.close()

    def test_simultaneous_submissions_only_send_once(self):
        c, b = self.make()
        start = threading.Barrier(5)
        def run():
            start.wait()
            c.submit(target())
        workers = [threading.Thread(target=run) for _ in range(4)]
        for worker in workers:
            worker.start()
        start.wait()
        for worker in workers:
            worker.join()
        self.assertEqual(len(b.sends), 1)
        b.notify("rejected")


if __name__ == "__main__":
    unittest.main()
