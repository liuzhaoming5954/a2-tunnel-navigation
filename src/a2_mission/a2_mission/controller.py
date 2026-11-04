"""Transactional offline mission rules for trusted local adapters.

This library does not issue motion permits or stop commands. A production
adapter must revoke/stop through a2_safety *before* calling stop-related rules.
Boolean observations below are trusted adapter inputs, never Web assertions.
"""

import copy
import math
import time
import uuid
from typing import Callable

from a2_storage import StorageError

from .model import Journal, State


PRECHECK_FIELDS = frozenset({
    "bundle_valid", "sensors_healthy", "storage_ready", "energy_sufficient",
    "control_released", "stop_guarantee_verified",
})
COMMAND_FIELDS = {
    "prepare": {"bundle_id", "route_id", "map_version", "config_version", "bundle_verified"},
    "authorize": {"operator_id", "local_confirmation"},
    "start": {"authorization_id"},
    "precheck": {"checks"},
    "localized": {"valid"},
    "record_visit": {"visit_id"},
    "pause": {"stopped_confirmed"},
    "resume": {"authorization_id", "health_confirmed", "stopped_confirmed"},
    "fault": {"reason", "stopped_confirmed"},
    "clear_fault": {"operator_id", "local_confirmation", "health_confirmed", "stopped_confirmed"},
    "cancel": {"stopped_confirmed"},
}


def _identifier(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 256:
        raise ValueError(f"{name} must be a nonempty string of at most 256 characters")
    return value


class MissionController:
    """Single-owner mission core backed by a transactional journal.

    Construct once per controller process. Construction fences any previous
    controller and clears all admission credentials, even on the same OS boot.
    Admission credentials are distinct from a2_safety motion permissions.
    """

    def __init__(self, journal: Journal, robot_id: str, boot_id: str, *,
                 authorization_ttl_s: float,
                 execution_mode: str = "core",
                 clock: Callable[[], float] = time.monotonic):
        if (isinstance(authorization_ttl_s, bool) or
                not isinstance(authorization_ttl_s, (int, float)) or
                not math.isfinite(authorization_ttl_s) or authorization_ttl_s <= 0):
            raise ValueError("authorization_ttl_s must be finite and positive")
        self.journal = journal
        if execution_mode not in {"core", "mock"}:
            raise ValueError("unsupported execution mode")
        self.execution_mode = execution_mode
        self.robot_id = _identifier(robot_id, "robot_id")
        self.boot_id = _identifier(boot_id, "boot_id")
        self.session_id = uuid.uuid4().hex
        self.ttl = float(authorization_ttl_s)
        self.clock = clock
        self.storage_fault_latched = False
        self._last_clock = None
        self._recover()

    def _now(self):
        now = self.clock()
        if (isinstance(now, bool) or not isinstance(now, (int, float)) or
                not math.isfinite(now) or now < 0 or
                (self._last_clock is not None and now < self._last_clock)):
            raise ValueError("clock must be finite, nonnegative and monotonic")
        self._last_clock = now
        return now

    def _recover(self):
        def transition(snapshot):
            snapshot = copy.deepcopy(snapshot) if snapshot else {
                "schema_version": 1, "robot_id": self.robot_id,
                "used_mission_ids": [], "mission": None,
            }
            if snapshot.get("schema_version") != 1:
                raise ValueError("unsupported mission snapshot schema")
            if snapshot.get("execution_mode", "core") != self.execution_mode and snapshot["mission"] is not None:
                raise ValueError("journal mission belongs to another execution mode")
            snapshot["execution_mode"] = self.execution_mode
            snapshot.update(boot_id=self.boot_id, controller_session_id=self.session_id)
            mission = snapshot["mission"]
            if mission:
                mission["authorization"] = None
                if mission["state"] not in {State.READY.value, State.CANCELLED.value}:
                    if mission["state"] in {State.PRECHECK.value, State.LOCALIZING.value, State.INSPECTING.value}:
                        mission["resume_state"] = mission["state"]
                    mission["state"] = State.PAUSED_FAULT.value
                    mission["reason"] = "CONTROLLER_RESTART"
            return snapshot, {"recovered": True}, [{
                "kind": "controller_recovered", "session_id": self.session_id,
                "boot_id": self.boot_id,
                "mission_id": mission["mission_id"] if mission else None,
            }]

        self.journal.apply(self.robot_id, f"__recover__:{self.session_id}",
                           {"session_id": self.session_id, "boot_id": self.boot_id}, transition)

    def current(self) -> dict:
        """Return current observation, excluding reusable credentials."""
        snapshot = self.journal.read_snapshot(self.robot_id)
        if snapshot and snapshot.get("mission"):
            snapshot = copy.deepcopy(snapshot)
            snapshot["mission"].pop("authorization", None)
        return {"snapshot": snapshot, "storage_fault_latched": self.storage_fault_latched}

    def command(self, request_id: str, command: str, mission_id: str, *,
                _request_identity=None, **parameters) -> dict:
        """Persist a command once; return historical receipt AND current state.

        request_id is stable across retries. A trusted adapter may provide a
        stable wire identity separately from its freshly computed observations;
        replay then retrieves the original result even if health has changed.
        The deduplication payload excludes
        the current session, so a retry after restart retrieves its old receipt
        without executing it again. Clients must inspect current separately.
        """
        _identifier(request_id, "request_id")
        if request_id.startswith("__"):
            raise ValueError("request_id prefix '__' is reserved")
        _identifier(command, "command")
        _identifier(mission_id, "mission_id")
        if self.storage_fault_latched:
            raise RuntimeError("storage fault latched; create a new controller after diagnosis")
        payload = {"command": command, "mission_id": mission_id, "parameters": parameters}
        if _request_identity is not None:
            if not isinstance(_request_identity, dict):
                raise TypeError("request identity must be a JSON object")
            payload = {"command": command, "mission_id": mission_id,
                       "adapter_identity": _request_identity}

        def transition(snapshot):
            # Lock acquisition may take time; validate expiry at execution time.
            return self._transition(snapshot, command, mission_id, parameters, self._now())

        try:
            receipt = self.journal.apply(self.robot_id, request_id, payload, transition)
            current = self.current()
        except StorageError:
            self.storage_fault_latched = True
            raise
        return {"receipt": receipt, "current": current}

    def _transition(self, snapshot, command, mission_id, params, now):
        snapshot = copy.deepcopy(snapshot)
        mission = snapshot.get("mission") if snapshot else None

        def result(accepted, code, **extra):
            current_mission = snapshot.get("mission") if snapshot else None
            receipt = {
                "accepted": accepted, "code": code, "mission_id": mission_id,
                "state": current_mission["state"] if current_mission else None,
                "controller_session_id": self.session_id, **extra,
            }
            event = {"kind": "mission_command", "command": command,
                     "mission_id": mission_id, "accepted": accepted, "code": code,
                     "state": receipt["state"]}
            return snapshot, receipt, [event]

        if not snapshot or snapshot.get("controller_session_id") != self.session_id:
            return result(False, "CONTROLLER_REPLACED")
        if command not in COMMAND_FIELDS or set(params) != COMMAND_FIELDS.get(command):
            return result(False, "INVALID_COMMAND")
        if command == "prepare":
            if mission and mission["state"] != State.CANCELLED.value:
                return result(False, "ACTIVE_MISSION")
            if mission_id in snapshot["used_mission_ids"]:
                return result(False, "MISSION_ID_REUSED")
            for field in ("bundle_id", "route_id", "map_version", "config_version"):
                _identifier(params[field], field)
            if params["bundle_verified"] is not True:
                return result(False, "BUNDLE_UNVERIFIED")
            mission = {"mission_id": mission_id, "state": State.READY.value,
                       "resume_state": State.PRECHECK.value,
                       "authorization": None, "completed_visits": [], "reason": None,
                       **{key: params[key] for key in
                          ("bundle_id", "route_id", "map_version", "config_version")}}
            snapshot["mission"] = mission
            snapshot["used_mission_ids"].append(mission_id)
            return result(True, "PREPARED")
        if not mission or mission["mission_id"] != mission_id:
            return result(False, "MISSION_MISMATCH")
        state = State(mission["state"])
        if command == "authorize":
            if state not in {State.READY, State.PAUSED_OPERATOR}:
                return result(False, "INVALID_STATE")
            if params["local_confirmation"] is not True:
                return result(False, "LOCAL_CONFIRMATION_REQUIRED")
            _identifier(params["operator_id"], "operator_id")
            credential = uuid.uuid4().hex
            mission["authorization"] = {
                "id": credential, "boot_id": self.boot_id, "session_id": self.session_id,
                "operator_id": params["operator_id"], "expires_at": now + self.ttl,
                "consumed": False,
            }
            return result(True, "AUTHORIZED", authorization_id=credential)
        if command in {"start", "resume"}:
            expected = State.READY if command == "start" else State.PAUSED_OPERATOR
            if state != expected:
                return result(False, "INVALID_STATE")
            credential = mission.get("authorization")
            if (not credential or credential["id"] != params["authorization_id"] or
                    credential["session_id"] != self.session_id or
                    credential["boot_id"] != self.boot_id or credential["consumed"] or
                    now >= credential["expires_at"]):
                return result(False, "AUTHORIZATION_INVALID")
            if command == "resume" and (params["health_confirmed"] is not True or
                                         params["stopped_confirmed"] is not True):
                return result(False, "RESUME_CHECK_FAILED")
            credential["consumed"] = True
            mission["state"] = State.PRECHECK.value if command == "start" else mission["resume_state"]
            mission["reason"] = None
            return result(True, "STARTED" if command == "start" else "RESUMED")
        if command == "precheck":
            if state != State.PRECHECK:
                return result(False, "INVALID_STATE")
            checks = params["checks"]
            if (not isinstance(checks, dict) or set(checks) != PRECHECK_FIELDS or
                    any(value is not True for value in checks.values())):
                mission.update(state=State.PAUSED_FAULT.value, reason="PRECHECK_FAILED", authorization=None)
                return result(False, "PRECHECK_FAILED")
            mission["state"] = State.LOCALIZING.value
            mission["resume_state"] = State.LOCALIZING.value
            return result(True, "PRECHECK_PASSED")
        if command == "localized":
            if state != State.LOCALIZING:
                return result(False, "INVALID_STATE")
            if params["valid"] is not True:
                mission.update(state=State.PAUSED_FAULT.value, reason="LOCALIZATION_INVALID", authorization=None)
                return result(False, "LOCALIZATION_INVALID")
            mission["state"] = State.INSPECTING.value
            mission["resume_state"] = State.INSPECTING.value
            return result(True, "LOCALIZED")
        if command == "record_visit":
            if state != State.INSPECTING:
                return result(False, "INVALID_STATE")
            visit_id = _identifier(params["visit_id"], "visit_id")
            if visit_id not in mission["completed_visits"]:
                mission["completed_visits"].append(visit_id)
            return result(True, "VISIT_RECORDED")
        if command == "clear_fault":
            if state != State.PAUSED_FAULT:
                return result(False, "INVALID_STATE")
            _identifier(params["operator_id"], "operator_id")
            if any(params[field] is not True for field in
                   ("local_confirmation", "health_confirmed", "stopped_confirmed")):
                return result(False, "RECOVERY_CHECK_FAILED")
            mission.update(state=State.PAUSED_OPERATOR.value, reason=None, authorization=None)
            return result(True, "FAULT_CLEARED")
        if command in {"pause", "fault", "cancel"}:
            if state == State.CANCELLED:
                return result(False, "INVALID_STATE")
            # A pause can never unlock an existing fault.
            if command == "pause" and state == State.PAUSED_FAULT:
                return result(False, "FAULT_LATCHED")
            if command == "pause" and state != State.INSPECTING:
                return result(False, "INVALID_STATE")
            mission["authorization"] = None
            if params["stopped_confirmed"] is not True:
                mission.update(state=State.PAUSED_FAULT.value, reason="STOP_UNCONFIRMED")
                return result(False, "STOP_UNCONFIRMED")
            if command == "fault":
                _identifier(params["reason"], "reason")
                mission.update(state=State.PAUSED_FAULT.value, reason=params["reason"])
                return result(True, "FAULT_LATCHED")
            mission.update(state=(State.CANCELLED if command == "cancel" else State.PAUSED_OPERATOR).value,
                           reason="OPERATOR_CANCEL" if command == "cancel" else "OPERATOR_PAUSE")
            return result(True, "CANCELLED" if command == "cancel" else "PAUSED")
        raise AssertionError("unhandled validated mission command")
