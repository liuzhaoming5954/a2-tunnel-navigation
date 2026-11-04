import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from a2_mission import MissionController
from a2_mission.controller import PRECHECK_FIELDS
from a2_storage import RequestConflict, SQLiteJournal, StorageError


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.journal = SQLiteJournal(Path(self.directory.name) / "mission.sqlite")
        self.addCleanup(self.journal.close)
        self.clock = Clock()
        self.controller = self.new_controller()
        self.sequence = 0

    def new_controller(self):
        return MissionController(self.journal, "robot", "same-boot", clock=self.clock,
                                 authorization_ttl_s=10)

    def send(self, command, mission_id="mission", request_id=None, **params):
        self.sequence += 1
        return self.controller.command(request_id or f"request-{self.sequence}", command,
                                       mission_id, **params)

    def prepare(self, mission_id="mission", **overrides):
        params = dict(bundle_id="bundle", route_id="route", map_version="map",
                      config_version="config", bundle_verified=True)
        params.update(overrides)
        return self.send("prepare", mission_id, **params)

    def authorize(self):
        return self.send("authorize", operator_id="operator", local_confirmation=True)["receipt"]["authorization_id"]

    def inspect(self):
        self.prepare()
        token = self.authorize()
        self.send("start", authorization_id=token)
        self.send("precheck", checks={field: True for field in PRECHECK_FIELDS})
        self.send("localized", valid=True)
        return token

    def state(self):
        return self.controller.current()["snapshot"]["mission"]["state"]

    def test_no_authorization_or_skipping_precheck(self):
        self.prepare()
        reply = self.send("start", authorization_id="missing")
        self.assertEqual(reply["receipt"]["code"], "AUTHORIZATION_INVALID")
        self.assertEqual(self.send("localized", valid=True)["receipt"]["code"], "INVALID_STATE")
        self.assertEqual(self.state(), "READY")

    def test_start_retries_are_idempotent(self):
        self.prepare()
        token = self.authorize()
        first = self.send("start", request_id="stable-start", authorization_id=token)
        events = len(self.journal.events("robot"))
        again = self.send("start", request_id="stable-start", authorization_id=token)
        self.assertEqual(first["receipt"], again["receipt"])
        self.assertEqual(len(self.journal.events("robot")), events)
        with self.assertRaises(RequestConflict):
            self.send("start", request_id="stable-start", authorization_id="changed")

    def test_authorization_expiry_boundary(self):
        self.prepare()
        token = self.authorize()
        self.clock.now += 10
        reply = self.send("start", authorization_id=token)
        self.assertEqual(reply["receipt"]["code"], "AUTHORIZATION_INVALID")
        self.assertEqual(self.state(), "READY")

    def test_authorization_expiring_while_waiting_for_journal_is_rejected(self):
        self.prepare()
        token = self.authorize()
        apply = self.journal.apply

        def delayed_apply(*args, **kwargs):
            self.clock.now += 10
            return apply(*args, **kwargs)

        with patch.object(self.journal, "apply", side_effect=delayed_apply):
            reply = self.send("start", authorization_id=token)
        self.assertEqual(reply["receipt"]["code"], "AUTHORIZATION_INVALID")
        self.assertEqual(self.state(), "READY")

    def test_precheck_failure_latches(self):
        self.prepare()
        self.send("start", authorization_id=self.authorize())
        checks = {field: True for field in PRECHECK_FIELDS}
        checks["stop_guarantee_verified"] = False
        self.send("precheck", checks=checks)
        self.assertEqual(self.state(), "PAUSED_FAULT")
        self.assertFalse(self.send("localized", valid=True)["receipt"]["accepted"])

    def test_localization_failure_latches(self):
        self.prepare()
        self.send("start", authorization_id=self.authorize())
        self.send("precheck", checks={field: True for field in PRECHECK_FIELDS})
        self.send("localized", valid=False)
        self.assertEqual(self.state(), "PAUSED_FAULT")

    def test_pause_invalidates_old_credential(self):
        old_token = self.inspect()
        self.send("pause", stopped_confirmed=True)
        reply = self.send("resume", authorization_id=old_token, health_confirmed=True, stopped_confirmed=True)
        self.assertFalse(reply["receipt"]["accepted"])
        self.send("resume", authorization_id=self.authorize(), health_confirmed=True, stopped_confirmed=True)
        self.assertEqual(self.state(), "INSPECTING")

    def test_recovery_does_not_skip_unfinished_precheck_or_localization(self):
        self.prepare()
        self.send("start", authorization_id=self.authorize())
        self.send("precheck", checks={})
        self.send("clear_fault", operator_id="operator", local_confirmation=True,
                  health_confirmed=True, stopped_confirmed=True)
        self.send("resume", authorization_id=self.authorize(), health_confirmed=True, stopped_confirmed=True)
        self.assertEqual(self.state(), "PRECHECK")
        self.send("precheck", checks={field: True for field in PRECHECK_FIELDS})
        self.send("localized", valid=False)
        self.send("clear_fault", operator_id="operator", local_confirmation=True,
                  health_confirmed=True, stopped_confirmed=True)
        self.send("resume", authorization_id=self.authorize(), health_confirmed=True, stopped_confirmed=True)
        self.assertEqual(self.state(), "LOCALIZING")

    def test_fault_clear_is_local_and_does_not_resume(self):
        self.inspect()
        self.send("fault", reason="SENSOR_STALE", stopped_confirmed=True)
        self.assertEqual(self.send("pause", stopped_confirmed=True)["receipt"]["code"], "FAULT_LATCHED")
        self.send("clear_fault", operator_id="operator", local_confirmation=False,
                  health_confirmed=True, stopped_confirmed=True)
        self.assertEqual(self.state(), "PAUSED_FAULT")
        self.send("clear_fault", operator_id="operator", local_confirmation=True,
                  health_confirmed=True, stopped_confirmed=True)
        self.assertEqual(self.state(), "PAUSED_OPERATOR")

    def test_unconfirmed_stop_is_not_cancelled(self):
        self.inspect()
        reply = self.send("cancel", stopped_confirmed=False)
        self.assertEqual(reply["receipt"]["code"], "STOP_UNCONFIRMED")
        self.assertEqual(self.state(), "PAUSED_FAULT")
        self.send("cancel", stopped_confirmed=True)
        self.assertEqual(self.state(), "CANCELLED")

    def test_single_active_task_and_no_mission_id_reuse(self):
        self.prepare()
        self.assertEqual(self.prepare("second")["receipt"]["code"], "ACTIVE_MISSION")
        self.send("cancel", stopped_confirmed=True)
        self.prepare("second")
        self.send("cancel", "second", stopped_confirmed=True)
        self.assertEqual(self.prepare()["receipt"]["code"], "MISSION_ID_REUSED")

    def test_wrong_mission_cannot_cancel_current_task(self):
        self.inspect()
        self.send("cancel", "other-task", stopped_confirmed=True)
        self.assertEqual(self.state(), "INSPECTING")

    def test_restart_fences_old_controller_and_keeps_progress(self):
        self.inspect()
        self.send("record_visit", visit_id="visit-1")
        old = self.controller
        self.controller = self.new_controller()
        self.assertEqual(self.state(), "PAUSED_FAULT")
        current = self.controller.current()["snapshot"]["mission"]
        self.assertEqual(current["completed_visits"], ["visit-1"])
        self.assertNotIn("authorization", current)
        reply = old.command("stale-controller", "cancel", "mission", stopped_confirmed=True)
        self.assertEqual(reply["receipt"]["code"], "CONTROLLER_REPLACED")
        self.assertEqual(self.state(), "PAUSED_FAULT")

    def test_historical_start_does_not_resume_after_restart(self):
        self.prepare()
        token = self.authorize()
        self.send("start", request_id="old-start", authorization_id=token)
        self.controller = self.new_controller()
        reply = self.send("start", request_id="old-start", authorization_id=token)
        self.assertTrue(reply["receipt"]["accepted"])
        self.assertEqual(reply["receipt"]["state"], "PRECHECK")
        self.assertEqual(reply["current"]["snapshot"]["mission"]["state"], "PAUSED_FAULT")

    def test_ready_restart_clears_unconsumed_credential(self):
        self.prepare()
        token = self.authorize()
        self.controller = self.new_controller()
        self.assertEqual(self.state(), "READY")
        reply = self.send("start", authorization_id=token)
        self.assertEqual(reply["receipt"]["code"], "AUTHORIZATION_INVALID")

    def test_visit_is_not_duplicated_by_different_requests(self):
        self.inspect()
        self.send("record_visit", visit_id="visit")
        self.send("record_visit", visit_id="visit")
        self.assertEqual(self.controller.current()["snapshot"]["mission"]["completed_visits"], ["visit"])

    def test_storage_failure_latches_and_cannot_auto_clear(self):
        self.prepare()
        with patch.object(self.journal, "apply", side_effect=StorageError("disk unavailable")):
            with self.assertRaises(StorageError):
                self.send("authorize", operator_id="operator", local_confirmation=True)
        self.assertTrue(self.controller.storage_fault_latched)
        with self.assertRaises(RuntimeError):
            self.authorize()

    def test_offline_core_never_connects(self):
        with patch("socket.create_connection", side_effect=AssertionError("network forbidden")):
            self.inspect()
            self.send("record_visit", visit_id="offline-visit")
            self.send("cancel", stopped_confirmed=True)
        self.assertEqual(self.state(), "CANCELLED")

    def test_strict_confirmation_and_parameter_validation(self):
        self.prepare(bundle_verified=1)
        self.assertIsNone(self.controller.current()["snapshot"]["mission"])
        self.prepare()
        reply = self.send("authorize", operator_id="operator", local_confirmation=1)
        self.assertFalse(reply["receipt"]["accepted"])
        self.assertEqual(self.send("start", authorization_id="missing", extra=True)["receipt"]["code"], "INVALID_COMMAND")

    def test_invalid_clocks_and_ttl(self):
        for ttl in (0, -1, float("inf"), float("nan"), True):
            with self.subTest(ttl=ttl), self.assertRaises(ValueError):
                MissionController(self.journal, "other", "boot", authorization_ttl_s=ttl)
        self.prepare()
        self.clock.now -= 1
        with self.assertRaises(ValueError):
            self.authorize()


if __name__ == "__main__":
    unittest.main()
