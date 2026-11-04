"""Mock-only task adapter. It never authenticates users or controls devices."""

import hashlib
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from .controller import MissionController, PRECHECK_FIELDS


class AdapterError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class MockRequest:
    schema_version: int = 1
    request_id: str = ""
    command: str = ""
    mission_id: str = ""
    operator_id: str = ""
    local_confirmation: bool = False
    authorization_id: str = ""
    bundle_id: str = ""
    route_id: str = ""
    map_version: str = ""
    config_version: str = ""
    visit_id: str = ""
    fault_reason: str = ""


PLAN_FIELDS = ("bundle_id", "route_id", "map_version", "config_version")
REQUEST_FIELDS = {
    "prepare": set(PLAN_FIELDS),
    "authorize": {"operator_id", "local_confirmation"},
    "start": {"authorization_id"},
    "pause": set(),
    "resume": {"authorization_id"},
    "cancel": set(),
    "clear_fault": {"operator_id", "local_confirmation"},
    "record_visit": {"visit_id"},
    "inject_fault": {"fault_reason"},
}


def load_mock_plan(path):
    """Load synthetic metadata, not a deployable or signed navigation bundle."""
    plan = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(plan, dict) or set(plan) != {
        "schema_version", "simulated", "visit_ids", *PLAN_FIELDS,
    }:
        raise ValueError("mock plan has unknown or missing fields")
    if type(plan["schema_version"]) is not int or plan["schema_version"] != 1 or plan["simulated"] is not True:
        raise ValueError("only schema 1 synthetic mock plans are supported")
    for field in PLAN_FIELDS:
        if not isinstance(plan[field], str) or not plan[field].strip() or len(plan[field]) > 256:
            raise ValueError(f"invalid mock plan {field}")
    visits = plan["visit_ids"]
    if (not isinstance(visits, list) or not visits or len(visits) > 1000 or
            any(not isinstance(value, str) or not value.strip() or len(value) > 256 for value in visits) or
            len(set(visits)) != len(visits)):
        raise ValueError("mock visit_ids must be unique, nonempty identifiers")
    return plan


class MockPlatformTracker:
    """Require advancing synthetic samples using the consumer's own clock.

    A repeated sample cannot renew freshness. Source monotonic stamps are only
    compared for ordering inside their source session, never with this clock.
    This is mock health, not a sensor or localization health implementation.
    """

    def __init__(self, robot_id, timeout_s=1.0, clock=time.monotonic):
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("platform timeout must be finite and positive")
        self.robot_id = robot_id
        self.timeout_s = timeout_s
        self.clock = clock
        self._session = None
        self._seq = None
        self._stamp = None
        self._deadline = None
        self._last_clock = None
        self._valid = False
        self._retired_sessions = set()

    def _now(self):
        now = self.clock()
        if (not isinstance(now, (int, float)) or isinstance(now, bool) or
                not math.isfinite(now) or now < 0 or
                (self._last_clock is not None and now < self._last_clock)):
            self._valid = False
            return None
        self._last_clock = now
        return now

    def observe(self, *, robot_id, source, simulated, motion_capable, valid,
                source_session_id, sample_seq, received_mono_ns, age_ms,
                battery_valid, battery_percent, velocity_valid, velocity,
                mode_valid, reason):
        if robot_id != self.robot_id:
            return
        if isinstance(source_session_id, str) and source_session_id in self._retired_sessions:
            return
        now = self._now()
        if now is None:
            return
        numbers_valid = (
            type(sample_seq) is int and sample_seq > 0 and
            type(received_mono_ns) is int and received_mono_ns >= 0 and
            type(age_ms) is int and age_ms >= 0 and
            isinstance(source_session_id, str) and bool(source_session_id)
        )
        if not numbers_valid:
            self._valid = False
            return
        try:
            numeric_fields_valid = (
                math.isfinite(battery_percent) and 0 <= battery_percent <= 100 and
                len(velocity) == 3 and all(math.isfinite(value) for value in velocity)
            )
        except (TypeError, ValueError):
            numeric_fields_valid = False
        fields_valid = (
            source == "mock" and simulated is True and motion_capable is False and
            valid is True and battery_valid is True and velocity_valid is True and
            mode_valid is True and reason == "VALID" and numeric_fields_valid
        )
        if not fields_valid:
            self._valid = False
            return
        if source_session_id == self._session:
            if sample_seq < self._seq or received_mono_ns < self._stamp:
                self._valid = False
                return
            if sample_seq == self._seq:
                # Neither a duplicate nor a forged revised duplicate renews it.
                if received_mono_ns != self._stamp:
                    self._valid = False
                return
            if received_mono_ns == self._stamp:
                self._valid = False
                return
        if self._session is not None and source_session_id != self._session:
            self._retired_sessions.add(self._session)
        self._session = source_session_id
        self._seq = sample_seq
        self._stamp = received_mono_ns
        self._deadline = now + max(0.0, self.timeout_s - age_ms / 1000.0)
        self._valid = age_ms < self.timeout_s * 1000

    def healthy(self):
        now = self._now()
        return (now is not None and self._valid and self._deadline is not None and
                now < self._deadline)


class MockMissionAdapter:
    """Expose only mock business commands; simulation acknowledgments stay here."""

    def __init__(self, controller: MissionController, plan, tracker: MockPlatformTracker):
        if controller.execution_mode != "mock":
            raise ValueError("mock adapter requires an isolated mock controller journal")
        self.controller = controller
        self.plan = plan
        self.tracker = tracker
        self._health_fault_sequence = 0

    def _matches(self, mission):
        return not mission or all(mission[field] == self.plan[field] for field in PLAN_FIELDS)

    def healthy(self):
        observation = self.controller.current()
        snapshot = observation["snapshot"]
        return (not observation["storage_fault_latched"] and snapshot is not None and
                snapshot["controller_session_id"] == self.controller.session_id and
                self._matches(snapshot["mission"]) and self.tracker.healthy())

    def _internal(self, stage, parent, command, mission_id, **params):
        digest = hashlib.sha256(
            f"{self.controller.session_id}:{stage}:{parent}".encode("utf-8")).hexdigest()
        return self.controller.command(f"sim-internal:{digest}", command, mission_id, **params)

    def execute(self, request: MockRequest):
        if type(request.schema_version) is not int or request.schema_version != 1:
            raise AdapterError("VERSION_MISMATCH", "only mock schema version 1 is supported")
        if request.command not in REQUEST_FIELDS:
            raise AdapterError("INVALID_COMMAND", "command is not in the mock allowlist")
        if request.request_id.startswith("sim-internal:"):
            raise AdapterError("INVALID_REQUEST_ID", "internal request prefix is reserved")
        defaults = asdict(MockRequest())
        envelope = {"schema_version", "request_id", "command", "mission_id"}
        for key, value in asdict(request).items():
            if key not in envelope | REQUEST_FIELDS[request.command] and value != defaults[key]:
                raise AdapterError("INVALID_FIELDS", f"field {key} is not valid for this command")
        params = {key: getattr(request, key) for key in REQUEST_FIELDS[request.command]}
        kind = request.command
        if kind == "prepare":
            if any(params[field] != self.plan[field] for field in PLAN_FIELDS):
                raise AdapterError("MOCK_PLAN_MISMATCH", "only the loaded synthetic plan may be prepared")
            params["bundle_verified"] = True  # Synthetic metadata, not a signature verdict.
        elif kind in {"pause", "cancel"}:
            params["stopped_confirmed"] = True  # No motion is ever produced by this process.
        elif kind == "resume":
            params.update(health_confirmed=self.healthy(), stopped_confirmed=True)
        elif kind == "clear_fault":
            params.update(health_confirmed=self.healthy(), stopped_confirmed=True)
        elif kind == "record_visit" and params["visit_id"] not in self.plan["visit_ids"]:
            raise AdapterError("UNKNOWN_VISIT", "visit is not in the synthetic plan")
        elif kind == "inject_fault":
            kind = "fault"
            params = {"reason": request.fault_reason, "stopped_confirmed": True}

        reply = self.controller.command(
            request.request_id, kind, request.mission_id,
            _request_identity={"adapter": "mock_ros_v1", "request": asdict(request)}, **params)
        receipt = reply["receipt"]
        if (receipt["accepted"] and receipt["code"] in {"STARTED", "RESUMED"} and
                receipt["controller_session_id"] == self.controller.session_id):
            self._advance(request.request_id, request.mission_id)
        # A database wait may have exhausted the health lease. Recheck before
        # returning a current-state observation, independently of receipt replay.
        self.monitor()
        return {"receipt": receipt, "current": self.controller.current()}

    def _advance(self, parent, mission_id):
        mission = self.controller.current()["snapshot"]["mission"]
        if mission["state"] == "PRECHECK":
            healthy = self.healthy()
            self._internal("precheck", parent, "precheck", mission_id,
                           checks={field: healthy for field in PRECHECK_FIELDS})
            mission = self.controller.current()["snapshot"]["mission"]
        if mission["state"] == "LOCALIZING":
            self._internal("localized", parent, "localized", mission_id, valid=self.healthy())

    def monitor(self):
        observation = self.controller.current()
        snapshot = observation["snapshot"]
        if not snapshot or snapshot["controller_session_id"] != self.controller.session_id:
            return
        mission = snapshot["mission"]
        if mission and mission["state"] in {"PRECHECK", "LOCALIZING", "INSPECTING"} and not self.healthy():
            self._health_fault_sequence += 1
            self._internal("health", f"{mission['mission_id']}:{self._health_fault_sequence}", "fault", mission["mission_id"],
                           reason="MOCK_PLATFORM_UNAVAILABLE", stopped_confirmed=True)
