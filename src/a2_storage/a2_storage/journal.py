"""Atomic mission snapshots, command receipts and events using SQLite.

Each thread or process must own its own journal connection. Transition callbacks
run under a database write transaction and must be short, deterministic and free
of device/network I/O. This class does not send motion commands.
"""

import hashlib
import json
import math
import sqlite3
from os import PathLike
from typing import Callable


class RequestConflict(ValueError):
    """A persisted request ID was reused with different command content."""


class StorageError(RuntimeError):
    """The journal could not read or commit its durable state."""


Transition = Callable[[dict | None], tuple[dict | None, dict, list[dict]]]


def _validate_json(value):
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("JSON values must be finite")
        return
    if isinstance(value, list):
        for item in value:
            _validate_json(item)
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError("JSON object keys must be strings")
            _validate_json(item)
        return
    raise TypeError(f"Unsupported JSON value: {type(value).__name__}")


def _canonical(value):
    _validate_json(value)
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False)


def _decode(value):
    try:
        result = json.loads(value)
        _validate_json(result)
        return result
    except (ValueError, TypeError) as exc:
        raise StorageError("Invalid JSON in persisted journal") from exc


class SQLiteJournal:
    """Durable, idempotent journal. Parent directories must already exist.

    ``apply`` atomically stores the returned snapshot, result and events. A
    repeated request returns the original result without invoking its callback.
    Snapshots retain all revisions; ``read_snapshot`` returns the latest one.
    ``None`` is a valid empty snapshot, including an explicit reset revision.
    """

    def __init__(self, path: str | PathLike, *, busy_timeout_ms: int = 1000):
        if type(busy_timeout_ms) is not int or not 0 <= busy_timeout_ms <= 1000:
            raise ValueError("busy_timeout_ms must be an integer between 0 and 1000")
        self._connection = None
        try:
            self._connection = sqlite3.connect(
                path, isolation_level=None, timeout=busy_timeout_ms / 1000,
            )
            self._execute(f"PRAGMA busy_timeout={busy_timeout_ms}")
            mode = self._one("PRAGMA journal_mode=WAL")[0]
            expected_mode = "memory" if str(path) == ":memory:" else "wal"
            if mode != expected_mode:
                raise StorageError(f"Required journal mode {expected_mode} is unavailable: {mode}")
            self._execute("PRAGMA synchronous=FULL")
            self._execute("BEGIN IMMEDIATE")
            self._execute(
                "CREATE TABLE IF NOT EXISTS journal_meta "
                "(key TEXT PRIMARY KEY, value TEXT NOT NULL)"
            )
            version = self._one("SELECT value FROM journal_meta WHERE key='schema_version'")
            if version is not None and version[0] != "1":
                raise StorageError("Unsupported journal schema version")
            self._execute("INSERT OR IGNORE INTO journal_meta VALUES ('schema_version', '1')")
            self._execute(
                "CREATE TABLE IF NOT EXISTS command_receipts ("
                "robot_id TEXT NOT NULL, request_id TEXT NOT NULL, "
                "request_digest TEXT NOT NULL, result_json TEXT NOT NULL, "
                "PRIMARY KEY (robot_id, request_id))"
            )
            self._execute(
                "CREATE TABLE IF NOT EXISTS snapshots ("
                "revision INTEGER PRIMARY KEY AUTOINCREMENT, robot_id TEXT NOT NULL, "
                "snapshot_json TEXT NOT NULL)"
            )
            self._execute(
                "CREATE INDEX IF NOT EXISTS snapshots_robot_revision "
                "ON snapshots(robot_id, revision DESC)"
            )
            self._execute(
                "CREATE TABLE IF NOT EXISTS events ("
                "event_seq INTEGER PRIMARY KEY AUTOINCREMENT, robot_id TEXT NOT NULL, "
                "request_id TEXT NOT NULL, payload_json TEXT NOT NULL)"
            )
            self._execute("COMMIT")
        except BaseException as exc:
            self._rollback()
            if self._connection is not None:
                try:
                    self._connection.close()
                except sqlite3.Error:
                    pass
            if isinstance(exc, sqlite3.Error):
                raise StorageError(str(exc)) from exc
            raise

    def _execute(self, sql, parameters=()):
        try:
            return self._connection.execute(sql, parameters)
        except sqlite3.Error as exc:
            raise StorageError(str(exc)) from exc

    def _one(self, sql, parameters=()):
        try:
            return self._connection.execute(sql, parameters).fetchone()
        except sqlite3.Error as exc:
            raise StorageError(str(exc)) from exc

    def _all(self, sql, parameters=()):
        try:
            return self._connection.execute(sql, parameters).fetchall()
        except sqlite3.Error as exc:
            raise StorageError(str(exc)) from exc

    def _rollback(self):
        # Preserve the original failure even if rollback/connection access fails.
        if self._connection is not None:
            try:
                if self._connection.in_transaction:
                    self._connection.execute("ROLLBACK")
            except sqlite3.Error:
                pass

    @staticmethod
    def _identifier(value, label):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{label} must be a nonempty string")

    def read_snapshot(self, robot_id: str) -> dict | None:
        self._identifier(robot_id, "robot_id")
        row = self._one(
            "SELECT snapshot_json FROM snapshots WHERE robot_id=? "
            "ORDER BY revision DESC LIMIT 1", (robot_id,),
        )
        snapshot = None if row is None else _decode(row[0])
        if snapshot is not None and not isinstance(snapshot, dict):
            raise StorageError("Persisted snapshot must be an object or null")
        return snapshot

    def events(self, robot_id: str) -> list[dict]:
        self._identifier(robot_id, "robot_id")
        rows = self._all(
            "SELECT event_seq, request_id, payload_json FROM events "
            "WHERE robot_id=? ORDER BY event_seq", (robot_id,),
        )
        events = []
        for seq, request, payload in rows:
            value = _decode(payload)
            if not isinstance(value, dict):
                raise StorageError("Persisted event must be an object")
            events.append({"event_seq": seq, "request_id": request, "payload": value})
        return events

    def apply(self, robot_id: str, request_id: str, payload: dict,
              transition: Transition) -> dict:
        self._identifier(robot_id, "robot_id")
        self._identifier(request_id, "request_id")
        if not isinstance(payload, dict):
            raise TypeError("payload must be a JSON object")
        digest = hashlib.sha256(_canonical(payload).encode("utf-8")).hexdigest()
        try:
            self._execute("BEGIN IMMEDIATE")
            receipt = self._one(
                "SELECT request_digest, result_json FROM command_receipts "
                "WHERE robot_id=? AND request_id=?", (robot_id, request_id),
            )
            if receipt is not None:
                if receipt[0] != digest:
                    raise RequestConflict("request_id was already used with different content")
                result = _decode(receipt[1])
                if not isinstance(result, dict):
                    raise StorageError("Persisted receipt result must be an object")
                self._execute("COMMIT")
                return result

            previous = self.read_snapshot(robot_id)
            previous_json = _canonical(previous)
            snapshot, result, events = transition(previous)
            if snapshot is not None and not isinstance(snapshot, dict):
                raise TypeError("transition snapshot must be an object or null")
            if not isinstance(result, dict):
                raise TypeError("transition result must be an object")
            if not isinstance(events, list) or any(not isinstance(event, dict) for event in events):
                raise TypeError("transition events must be a list of objects")
            snapshot_json = _canonical(snapshot)
            result_json = _canonical(result)
            event_json = [_canonical(event) for event in events]
            if snapshot_json != previous_json:
                self._execute("INSERT INTO snapshots (robot_id, snapshot_json) VALUES (?, ?)",
                              (robot_id, snapshot_json))
            for event in event_json:
                self._execute(
                    "INSERT INTO events (robot_id, request_id, payload_json) VALUES (?, ?, ?)",
                    (robot_id, request_id, event),
                )
            self._execute(
                "INSERT INTO command_receipts VALUES (?, ?, ?, ?)",
                (robot_id, request_id, digest, result_json),
            )
            self._execute("COMMIT")
            return json.loads(result_json)
        except BaseException:
            self._rollback()
            raise

    def close(self):
        try:
            self._connection.close()
        except sqlite3.Error as exc:
            raise StorageError(str(exc)) from exc

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type is None:
            self.close()
        else:
            try:
                self.close()
            except StorageError:
                pass
