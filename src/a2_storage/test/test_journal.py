import math
import os
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from a2_storage import RequestConflict, SQLiteJournal, StorageError


def accepted(snapshot):
    return {"mission_id": "mission-1", "state": "READY"}, {"accepted": True}, [{"state": "READY"}]


class JournalTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="a2-journal-test-")
        self.root = Path(self.temp.name)
        self.path = self.root / "journal.sqlite"

    def tearDown(self):
        self.temp.cleanup()

    def test_durable_atomic_result_and_replay_across_reopen(self):
        with SQLiteJournal(self.path) as journal:
            result = journal.apply("robot", "prepare", {"b": 2, "a": 1}, accepted)
            result["accepted"] = False
        with SQLiteJournal(self.path) as journal:
            def must_not_run(snapshot):
                self.fail("a persisted duplicate must not run its transition")
            self.assertEqual(journal.apply("robot", "prepare", {"a": 1, "b": 2}, must_not_run),
                             {"accepted": True})
            self.assertEqual(journal.read_snapshot("robot"), {"mission_id": "mission-1", "state": "READY"})
            self.assertEqual(journal.events("robot"), [
                {"event_seq": 1, "request_id": "prepare", "payload": {"state": "READY"}},
            ])
            with self.assertRaises(RequestConflict):
                journal.apply("robot", "prepare", {"a": 999}, must_not_run)
            self.assertEqual(len(journal.events("robot")), 1)

    def test_revisions_preserve_previous_missions_and_explicit_reset(self):
        with SQLiteJournal(self.path) as journal:
            journal.apply("robot", "one", {}, accepted)
            journal.apply("robot", "two", {}, lambda state: (
                {"mission_id": "mission-2", "state": "READY"}, {"ok": True}, []))
            journal.apply("robot", "reset", {}, lambda state: (None, {"ok": True}, []))
            self.assertIsNone(journal.read_snapshot("robot"))
        with sqlite3.connect(self.path) as connection:
            rows = connection.execute("SELECT snapshot_json FROM snapshots ORDER BY revision").fetchall()
        self.assertEqual(len(rows), 3)
        self.assertIn('"mission_id":"mission-1"', rows[0][0])
        self.assertIn('"mission_id":"mission-2"', rows[1][0])
        self.assertEqual(rows[2][0], "null")

    def test_callback_exception_is_original_and_mutation_is_not_persisted(self):
        failures = [RuntimeError("transition failed"), sqlite3.OperationalError("callback SQL")]
        for index, failure in enumerate(failures):
            with self.subTest(failure=failure), SQLiteJournal(self.root / f"failure-{index}.sqlite") as journal:
                journal.apply("robot", "prepare", {}, accepted)
                def explode(snapshot):
                    snapshot["state"] = "INSPECTING"
                    raise failure
                with self.assertRaises(type(failure)) as caught:
                    journal.apply("robot", "failing", {}, explode)
                self.assertIs(caught.exception, failure)
                self.assertEqual(journal.read_snapshot("robot")["state"], "READY")
                self.assertEqual(len(journal.events("robot")), 1)
                self.assertEqual(journal.apply("robot", "failing", {}, accepted), {"accepted": True})

    def test_final_receipt_failure_rolls_back_snapshot_and_events(self):
        with SQLiteJournal(self.path) as journal:
            journal.apply("robot", "prepare", {}, accepted)
            with sqlite3.connect(self.path) as connection:
                connection.execute("CREATE TRIGGER reject_receipt BEFORE INSERT ON command_receipts "
                                   "WHEN NEW.request_id='advance' BEGIN SELECT RAISE(ABORT, 'injected'); END")
            def advance(snapshot):
                return {**snapshot, "state": "PRECHECK"}, {"accepted": True}, [{"state": "PRECHECK"}]
            with self.assertRaisesRegex(StorageError, "injected"):
                journal.apply("robot", "advance", {}, advance)
            self.assertEqual(journal.read_snapshot("robot")["state"], "READY")
            self.assertEqual([event["event_seq"] for event in journal.events("robot")], [1])
            with sqlite3.connect(self.path) as connection:
                self.assertEqual(connection.execute("SELECT count(*) FROM snapshots").fetchone()[0], 1)
                self.assertEqual(connection.execute("SELECT count(*) FROM command_receipts").fetchone()[0], 1)
                connection.execute("DROP TRIGGER reject_receipt")
            journal.apply("robot", "advance", {}, advance)
            self.assertEqual(journal.read_snapshot("robot")["state"], "PRECHECK")
            self.assertEqual([event["event_seq"] for event in journal.events("robot")], [1, 2])

    def test_concurrent_connections_execute_same_request_once(self):
        with SQLiteJournal(self.path):
            pass
        barrier = threading.Barrier(2)
        executions = []
        def worker():
            with SQLiteJournal(self.path) as journal:
                barrier.wait(timeout=3)
                def transition(snapshot):
                    executions.append(threading.get_ident())
                    time.sleep(0.03)
                    return {"count": 1}, {"count": 1}, [{"count": 1}]
                return journal.apply("robot", "same-request", {"command": "start"}, transition)
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(worker) for _ in range(2)]
            self.assertEqual([future.result(timeout=5) for future in futures], [{"count": 1}, {"count": 1}])
        self.assertEqual(len(executions), 1)
        with SQLiteJournal(self.path) as journal:
            self.assertEqual(journal.read_snapshot("robot"), {"count": 1})
            self.assertEqual(len(journal.events("robot")), 1)

    def test_lock_timeout_is_bounded_and_connection_recovers(self):
        with SQLiteJournal(self.path, busy_timeout_ms=30) as journal:
            with sqlite3.connect(self.path) as lock:
                lock.execute("BEGIN IMMEDIATE")
                started = time.monotonic()
                with self.assertRaisesRegex(StorageError, "locked"):
                    journal.apply("robot", "prepare", {}, accepted)
                self.assertLess(time.monotonic() - started, 1)
                lock.rollback()
            self.assertEqual(journal.apply("robot", "prepare", {}, accepted), {"accepted": True})

    def test_nonfinite_request_json_never_executes_callback(self):
        for number in [math.nan, math.inf, -math.inf]:
            with self.subTest(number=number), SQLiteJournal(":memory:") as journal:
                with self.assertRaisesRegex(ValueError, "finite"):
                    journal.apply("robot", "bad", {"nested": [number]}, lambda state: self.fail("executed"))
                self.assertIsNone(journal.read_snapshot("robot"))
                self.assertEqual(journal.events("robot"), [])

    def test_invalid_transition_json_rolls_back(self):
        for bad_part in ["snapshot", "result", "events"]:
            with self.subTest(part=bad_part), SQLiteJournal(":memory:") as journal:
                journal.apply("robot", "prepare", {}, accepted)
                def invalid(snapshot):
                    new_snapshot = {"state": "PRECHECK"}
                    result = {"ok": True}
                    events = [{"state": "PRECHECK"}]
                    {"snapshot": new_snapshot, "result": result, "events": events[0]}[bad_part]["bad"] = math.nan
                    return new_snapshot, result, events
                with self.assertRaises(ValueError):
                    journal.apply("robot", "bad", {}, invalid)
                self.assertEqual(journal.read_snapshot("robot")["state"], "READY")
                self.assertEqual(len(journal.events("robot")), 1)

    def test_canonical_objects_require_string_keys(self):
        with SQLiteJournal(":memory:") as journal:
            with self.assertRaisesRegex(TypeError, "keys"):
                journal.apply("robot", "bad", {1: "value"}, accepted)

    def test_results_and_snapshots_are_detached_copies(self):
        with SQLiteJournal(":memory:") as journal:
            journal.apply("robot", "prepare", {}, accepted)
            snapshot = journal.read_snapshot("robot")
            snapshot["state"] = "MODIFIED"
            self.assertEqual(journal.read_snapshot("robot")["state"], "READY")

    def test_robot_namespaces_isolate_requests_and_keep_event_order(self):
        with SQLiteJournal(":memory:") as journal:
            journal.apply("one", "same-id", {"x": 1}, accepted)
            journal.apply("two", "same-id", {"x": 2}, lambda state: (
                {"state": "SECOND"}, {"accepted": True}, [{"state": "SECOND"}]))
            journal.apply("one", "next", {}, lambda state: (state, {"ok": True}, [{"next": True}]))
            self.assertEqual(journal.read_snapshot("two")["state"], "SECOND")
            self.assertEqual(journal.read_snapshot("one")["state"], "READY")
            self.assertEqual([event["event_seq"] for event in journal.events("one")], [1, 3])
            self.assertEqual([event["event_seq"] for event in journal.events("two")], [2])

    def test_storage_configuration_and_closed_connection_errors(self):
        journal = SQLiteJournal(self.path)
        self.assertEqual(journal._one("PRAGMA synchronous")[0], 2)
        self.assertEqual(journal._one("PRAGMA journal_mode")[0], "wal")
        self.assertLessEqual(journal._one("PRAGMA busy_timeout")[0], 1000)
        journal.close()
        with self.assertRaises(StorageError):
            journal.read_snapshot("robot")
        with self.assertRaises(StorageError):
            journal.apply("robot", "one", {}, accepted)
        with self.assertRaises(StorageError):
            journal.events("robot")

    def test_missing_parent_is_not_implicitly_created(self):
        parent = self.root / "missing"
        with self.assertRaises(StorageError):
            SQLiteJournal(parent / "journal.sqlite")
        self.assertFalse(parent.exists())

    def test_unknown_schema_version_is_rejected(self):
        with SQLiteJournal(self.path):
            pass
        with sqlite3.connect(self.path) as connection:
            connection.execute("UPDATE journal_meta SET value='999' WHERE key='schema_version'")
        with self.assertRaisesRegex(StorageError, "schema"):
            SQLiteJournal(self.path)

    def test_corrupt_object_type_is_reported_as_storage_failure(self):
        for table, column in [("command_receipts", "result_json"), ("events", "payload_json")]:
            with self.subTest(table=table), SQLiteJournal(self.root / f"{table}.sqlite") as journal:
                journal.apply("robot", "prepare", {}, accepted)
                with sqlite3.connect(self.root / f"{table}.sqlite") as connection:
                    connection.execute(f"UPDATE {table} SET {column}='[]'")
                with self.assertRaisesRegex(StorageError, "object"):
                    if table == "events":
                        journal.events("robot")
                    else:
                        journal.apply("robot", "prepare", {}, accepted)

    def test_process_exit_at_commit_boundary_keeps_all_or_none(self):
        for crash_before_commit in [True, False]:
            path = self.root / f"crash-{crash_before_commit}.sqlite"
            with self.subTest(before_commit=crash_before_commit):
                with SQLiteJournal(path) as journal:
                    journal.apply("robot", "prepare", {}, accepted)
                script = '''
import os, sys
from a2_storage import SQLiteJournal
journal = SQLiteJournal(sys.argv[1])
if sys.argv[2] == "before":
    def trace(statement):
        if statement == "COMMIT":
            os._exit(23)
    journal._connection.set_trace_callback(trace)
journal.apply("robot", "advance", {}, lambda snapshot: (
    {**snapshot, "state": "PRECHECK"}, {"advanced": True}, [{"state": "PRECHECK"}]))
os._exit(23)
'''
                env = os.environ.copy()
                env["PYTHONPATH"] = os.pathsep.join(sys.path)
                result = subprocess.run(
                    [sys.executable, "-c", script, str(path), "before" if crash_before_commit else "after"],
                    env=env, timeout=5, check=False,
                )
                self.assertEqual(result.returncode, 23)
                with SQLiteJournal(path) as journal:
                    self.assertEqual(journal.read_snapshot("robot")["state"],
                                     "READY" if crash_before_commit else "PRECHECK")
                    self.assertEqual(len(journal.events("robot")), 1 if crash_before_commit else 2)
                    callbacks = []
                    def retry(snapshot):
                        callbacks.append(True)
                        return {**snapshot, "state": "PRECHECK"}, {"advanced": True}, [{"state": "PRECHECK"}]
                    self.assertEqual(journal.apply("robot", "advance", {}, retry), {"advanced": True})
                    self.assertEqual(len(callbacks), 1 if crash_before_commit else 0)
                    self.assertEqual(len(journal.events("robot")), 2)


if __name__ == "__main__":
    unittest.main()
