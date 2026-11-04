import json
import tempfile
import unittest
from pathlib import Path

from a2_mission import MissionController
from a2_mission.mock_adapter import (
    AdapterError, MockMissionAdapter, MockPlatformTracker, MockRequest, load_mock_plan,
)
from a2_storage import RequestConflict, SQLiteJournal


PLAN = dict(schema_version=1, simulated=True, bundle_id="demo-bundle", route_id="demo-route",
            map_version="demo-map", config_version="demo-config", visit_ids=["demo-valve-01"])


class MockAdapterTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.now = 100.0
        self.journal = SQLiteJournal(Path(self.directory.name) / "mock.sqlite")
        self.addCleanup(self.journal.close)
        self.controller = MissionController(self.journal, "robot", "boot", authorization_ttl_s=10,
                                            clock=lambda: self.now, execution_mode="mock")
        self.tracker = MockPlatformTracker("robot", clock=lambda: self.now)
        self.adapter = MockMissionAdapter(self.controller, PLAN.copy(), self.tracker)
        self.counter = 0

    def observe(self, **overrides):
        fields = dict(robot_id="robot", source="mock", simulated=True, motion_capable=False,
                      valid=True, source_session_id="source-1", sample_seq=1, received_mono_ns=10,
                      age_ms=0, battery_valid=True, battery_percent=82.0, velocity_valid=True,
                      velocity=(0.0, 0.0, 0.0), mode_valid=True, reason="VALID")
        fields.update(overrides)
        self.tracker.observe(**fields)

    def send(self, command, request_id=None, mission_id="mission", **kwargs):
        self.counter += 1
        return self.adapter.execute(MockRequest(request_id=request_id or f"req-{self.counter}",
                                               command=command, mission_id=mission_id, **kwargs))

    def prepare(self, mission_id="mission"):
        return self.send("prepare", mission_id=mission_id,
                         **{key: PLAN[key] for key in ("bundle_id", "route_id", "map_version", "config_version")})

    def authorize(self):
        return self.send("authorize", operator_id="mock-operator", local_confirmation=True)["receipt"]["authorization_id"]

    def inspect(self):
        self.observe()
        self.prepare()
        token = self.authorize()
        self.send("start", authorization_id=token)
        self.assertEqual(self.state(), "INSPECTING")
        return token

    def state(self):
        return self.controller.current()["snapshot"]["mission"]["state"]

    def clear(self):
        return self.send("clear_fault", operator_id="mock-operator", local_confirmation=True)

    def test_plan_loader_rejects_real_or_unrecognized_configuration(self):
        path = Path(self.directory.name) / "plan.json"
        for plan in [dict(PLAN, simulated=False), dict(PLAN, schema_version=2),
                     dict(PLAN, extra=True), dict(PLAN, visit_ids=["duplicate", "duplicate"])]:
            with self.subTest(plan=plan):
                path.write_text(json.dumps(plan))
                with self.assertRaises(ValueError):
                    load_mock_plan(path)
        path.write_text(json.dumps(PLAN))
        self.assertEqual(load_mock_plan(path), PLAN)

    def test_adapter_rejects_wrong_plan_and_unknown_visit(self):
        with self.assertRaises(AdapterError):
            self.send("prepare", bundle_id="real", route_id="x", map_version="x", config_version="x")
        self.inspect()
        with self.assertRaises(AdapterError):
            self.send("record_visit", visit_id="not-configured")
        self.send("record_visit", visit_id="demo-valve-01")
        self.assertEqual(self.controller.current()["snapshot"]["mission"]["completed_visits"], ["demo-valve-01"])

    def test_client_cannot_inject_checks_or_unused_fields(self):
        for request in [MockRequest(schema_version=2, command="prepare"),
                        MockRequest(request_id="sim-internal:x", command="pause"),
                        MockRequest(command="localized"),
                        MockRequest(command="pause", authorization_id="irrelevant")]:
            with self.subTest(request=request), self.assertRaises(AdapterError):
                self.adapter.execute(request)

    def test_start_automatically_runs_only_mock_precheck_and_localization(self):
        self.observe()
        self.prepare()
        reply = self.send("start", authorization_id=self.authorize())
        self.assertEqual(reply["receipt"]["state"], "PRECHECK")
        self.assertEqual(self.state(), "INSPECTING")

    def test_no_platform_is_not_silently_healthy(self):
        self.prepare()
        self.send("start", authorization_id=self.authorize())
        self.assertEqual(self.state(), "PAUSED_FAULT")
        self.assertFalse(self.clear()["receipt"]["accepted"])

    def test_duplicate_start_has_no_additional_effects(self):
        self.observe()
        self.prepare()
        token = self.authorize()
        reply = self.send("start", request_id="start-once", authorization_id=token)
        event_count = len(self.journal.events("robot"))
        repeated = self.send("start", request_id="start-once", authorization_id=token)
        self.assertEqual(reply["receipt"], repeated["receipt"])
        self.assertEqual(len(self.journal.events("robot")), event_count)
        with self.assertRaises(RequestConflict):
            self.send("start", request_id="start-once", authorization_id="different")

    def test_resume_retry_uses_original_wire_identity_after_health_changes(self):
        self.inspect()
        self.send("pause")
        token = self.authorize()
        first = self.send("resume", request_id="resume-once", authorization_id=token)
        self.now += 2
        self.adapter.monitor()
        self.assertEqual(self.state(), "PAUSED_FAULT")
        count = len(self.journal.events("robot"))
        reply = self.send("resume", request_id="resume-once", authorization_id=token)
        self.assertEqual(reply["receipt"], first["receipt"])
        self.assertEqual(self.state(), "PAUSED_FAULT")
        self.assertEqual(len(self.journal.events("robot")), count)

    def test_each_health_fault_cycle_uses_a_new_internal_identity(self):
        self.inspect()
        for seq in (2, 3):
            self.now += 2
            self.adapter.monitor()
            self.assertEqual(self.state(), "PAUSED_FAULT")
            self.observe(sample_seq=seq, received_mono_ns=10 * seq)
            self.clear()
            self.send("resume", authorization_id=self.authorize())
            self.assertEqual(self.state(), "INSPECTING")

    def test_clear_fault_retry_cannot_unlock_a_later_fault(self):
        self.inspect()
        self.send("inject_fault", fault_reason="FIRST")
        first = self.send("clear_fault", request_id="clear-once", operator_id="operator", local_confirmation=True)
        self.send("resume", authorization_id=self.authorize())
        self.now += 2
        self.adapter.monitor()
        replay = self.send("clear_fault", request_id="clear-once", operator_id="operator", local_confirmation=True)
        self.assertEqual(first["receipt"], replay["receipt"])
        self.assertEqual(self.state(), "PAUSED_FAULT")

    def test_republishing_same_sample_does_not_refresh_health(self):
        self.observe()
        self.assertTrue(self.tracker.healthy())
        self.now += 0.8
        self.observe()
        self.now += 0.2
        self.assertFalse(self.tracker.healthy())

    def test_wrong_robot_and_real_source_do_not_supply_mock_health(self):
        self.observe(robot_id="other")
        self.assertFalse(self.tracker.healthy())
        self.observe(simulated=False, source="unitree_sdk2")
        self.assertFalse(self.tracker.healthy())
        self.observe(battery_percent=float("nan"))
        self.assertFalse(self.tracker.healthy())

    def test_retired_source_session_cannot_replace_current_session(self):
        self.observe()
        self.observe(source_session_id="source-2", sample_seq=1)
        self.now += 0.8
        self.observe(source_session_id="source-1", sample_seq=2, received_mono_ns=20)
        self.now += 0.2
        self.assertFalse(self.tracker.healthy())

    def test_reordered_source_sample_invalidates_health(self):
        self.observe(sample_seq=3, received_mono_ns=30)
        self.observe(sample_seq=2, received_mono_ns=20)
        self.assertFalse(self.tracker.healthy())

    def test_increasing_sequence_without_new_reception_time_is_not_fresh(self):
        self.observe()
        self.observe(sample_seq=2)
        self.assertFalse(self.tracker.healthy())

    def test_retired_invalid_source_does_not_poison_new_source(self):
        self.observe()
        self.observe(source_session_id="source-2")
        self.observe(source_session_id="source-1", valid=False, reason="STALE")
        self.assertTrue(self.tracker.healthy())

    def test_restart_preserves_visit_but_requires_new_confirmation(self):
        old_token = self.inspect()
        self.send("record_visit", visit_id="demo-valve-01")
        self.controller = MissionController(self.journal, "robot", "boot", authorization_ttl_s=10,
                                            clock=lambda: self.now, execution_mode="mock")
        self.adapter = MockMissionAdapter(self.controller, PLAN.copy(), self.tracker)
        self.assertEqual(self.state(), "PAUSED_FAULT")
        self.send("resume", authorization_id=old_token)
        self.assertEqual(self.state(), "PAUSED_FAULT")
        self.clear()
        self.send("resume", authorization_id=self.authorize())
        self.assertEqual(self.state(), "INSPECTING")
        self.assertEqual(self.controller.current()["snapshot"]["mission"]["completed_visits"], ["demo-valve-01"])

    def test_health_expiring_during_journal_wait_cannot_return_running_state(self):
        self.inspect()
        self.send("pause")
        token = self.authorize()
        apply = self.journal.apply
        delayed = False

        def wait_once(*args, **kwargs):
            nonlocal delayed
            if not delayed:
                self.now += 2
                delayed = True
            return apply(*args, **kwargs)

        from unittest.mock import patch
        with patch.object(self.journal, "apply", side_effect=wait_once):
            reply = self.send("resume", authorization_id=token)
        self.assertEqual(reply["current"]["snapshot"]["mission"]["state"], "PAUSED_FAULT")

    def test_core_controller_cannot_adopt_a_mock_task_database(self):
        self.inspect()
        session = self.controller.session_id
        with self.assertRaises(ValueError):
            MissionController(self.journal, "robot", "boot", authorization_ttl_s=10)
        self.assertEqual(self.journal.read_snapshot("robot")["controller_session_id"], session)


if __name__ == "__main__":
    unittest.main()
