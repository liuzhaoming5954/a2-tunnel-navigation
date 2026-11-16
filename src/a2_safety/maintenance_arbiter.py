"""Local maintenance authority and watchdog, with no chassis transport of its own."""

from dataclasses import dataclass
from enum import Enum
import math
import secrets
from threading import Event, Lock, Thread
import time
from typing import Callable, Generic, Protocol, TypeVar


class Owner(Enum):
    RELEASED = "released"
    OWNED = "owned"
    UNKNOWN = "unknown"


class Direction(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"


class Feedback(Enum):
    STOPPED = "stopped"
    MOVING = "moving"


T = TypeVar("T")


@dataclass(frozen=True)
class Sample(Generic[T]):
    value: T
    observed_at: float


@dataclass(frozen=True)
class LocalGrant:
    serial: int
    issued_at: float
    expires_at: float
    directions: frozenset[Direction]


@dataclass(frozen=True)
class ReceiverStopEvidence:
    receiver_id: str
    report_id: str
    expires_at: float


@dataclass(frozen=True)
class Inputs:
    owner: Sample[Owner] | None = None
    guard: Sample[frozenset[Direction]] | None = None
    feedback: Sample[Feedback] | None = None
    grant: LocalGrant | None = None
    receiver_stop: ReceiverStopEvidence | None = None


@dataclass(frozen=True)
class Limits:
    linear_mps: float
    angular_radps: float
    lease_s: float
    deadman_s: float
    max_sample_age_s: float
    tick_s: float

    def __post_init__(self):
        values = (self.linear_mps, self.angular_radps, self.lease_s, self.deadman_s,
                  self.max_sample_age_s, self.tick_s)
        if not all(_finite(value) and value > 0 for value in values):
            raise ValueError("Positive finite limits are required")
        if (not .001 <= self.lease_s <= 60 or not self.tick_s < self.deadman_s <= self.lease_s
                or self.tick_s >= self.max_sample_age_s):
            raise ValueError("Inconsistent lease or watchdog timing")


@dataclass(frozen=True)
class VelocityRequest:
    session_id: str
    sequence: int
    grant_serial: int
    generated_at: float
    expires_at: float
    linear_mps: float
    angular_radps: float


class Output(Protocol):
    """Bounded local handoff to the collision monitor and chassis bridge.

    Acknowledging either method means the request was accepted. A separate
    feedback observation must establish actual standstill.
    """

    receiver_id: str

    def velocity(self, request: VelocityRequest) -> bool: ...

    def stop(self, session_id: str, reason: str) -> bool: ...


@dataclass
class _Lease:
    session_id: str
    grant: LocalGrant
    expires_at: float
    sequence: int = 0
    command_deadline: float | None = None
    direction: Direction | None = None


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _reject(status, reason):
    return status, {"ok": False, "reason": reason}


class MaintenanceArbiter:
    """A single manual lease backed by fresh, independently acquired local evidence.

    Input samples and requests use the same machine's monotonic clock. The
    provider and output callbacks must be bounded and must not call this object.
    The output still requires its own downstream watchdog. No real output or
    field evidence provider is included here.
    """

    def __init__(self, limits: Limits, observe: Callable[[], Inputs], output: Output,
                 *, clock: Callable[[], float] = time.monotonic):
        if not isinstance(output.receiver_id, str) or not output.receiver_id:
            raise ValueError("A specific output receiver ID is required")
        self._limits, self._observe, self._output, self._clock = limits, observe, output, clock
        self._receiver_id = output.receiver_id
        self._started_at = clock()
        self._lock, self._closed = Lock(), Event()
        self._lease = None
        self._last_grant = 0
        self._last_revoked = None
        self._stop_requested_at = None
        self._stop_accepted = False
        self._worker = Thread(target=self._watchdog, name="a2-maintenance-watchdog", daemon=True)
        self._worker.start()

    def _watchdog(self):
        while not self._closed.wait(self._limits.tick_s):
            self.tick()

    def close(self):
        self._closed.set()
        with self._lock:
            self._advance()
        self._worker.join()

    def _sample(self, sample, now):
        if (isinstance(sample, Sample) and _finite(sample.observed_at)
                and 0 <= now - sample.observed_at <= self._limits.max_sample_age_s):
            return sample.value
        return None

    def _receiver_verified(self, inputs, now):
        evidence = inputs.receiver_stop
        return (isinstance(evidence, ReceiverStopEvidence) and evidence.receiver_id == self._receiver_id
                and isinstance(evidence.report_id, str) and bool(evidence.report_id)
                and _finite(evidence.expires_at) and now < evidence.expires_at)

    def _granted(self, grant, now):
        return (isinstance(grant, LocalGrant) and type(grant.serial) is int and grant.serial > 0
                and _finite(grant.issued_at) and _finite(grant.expires_at)
                and self._started_at < grant.issued_at <= now < grant.expires_at
                and isinstance(grant.directions, frozenset) and bool(grant.directions)
                and all(isinstance(direction, Direction) for direction in grant.directions))

    def _deny_reason(self, inputs, now):
        if self._closed.is_set():
            return "arbiter_closed"
        owner = self._sample(inputs.owner, now)
        if owner is not Owner.RELEASED:
            return "control_owned" if owner is Owner.OWNED else "control_unknown"
        if not self._receiver_verified(inputs, now):
            return "receiver_stop_unverified"
        if self._stop_requested_at is not None:
            return "stop_unconfirmed"
        feedback = self._sample(inputs.feedback, now)
        if feedback not in (Feedback.STOPPED, Feedback.MOVING):
            return "feedback_stale"
        if not self._lease or self._lease.direction is None:
            if feedback is not Feedback.STOPPED:
                return "standstill_required"
        directions = self._sample(inputs.guard, now)
        if not isinstance(directions, frozenset) or not all(isinstance(direction, Direction) for direction in directions):
            return "guard_stale"
        if not self._granted(inputs.grant, now):
            return "local_authorization_required"
        allowed = directions & inputs.grant.directions
        if not allowed or (self._lease and self._lease.direction and self._lease.direction not in allowed):
            return "obstacle"
        if self._lease:
            if inputs.grant != self._lease.grant:
                return "authorization_changed"
            if now >= self._lease.expires_at:
                return "lease_expired"
            if self._lease.command_deadline is not None and now >= self._lease.command_deadline:
                return "deadman_expired"
        elif inputs.grant.serial <= self._last_grant:
            return "local_authorization_required"
        return None

    def _revoke(self, reason, inputs, now):
        lease, self._lease = self._lease, None
        self._last_revoked = lease.session_id
        self._stop_requested_at = now
        self._stop_accepted = False
        if self._sample(inputs.owner, now) is Owner.RELEASED:
            try:
                self._stop_accepted = self._output.stop(lease.session_id, reason) is True
            except Exception:
                # Feedback is still required even if the output failed to acknowledge.
                pass

    def _advance(self):
        try:
            inputs = self._observe()
        except Exception:
            inputs = None
        if not isinstance(inputs, Inputs):
            inputs = Inputs()
        now = self._clock()
        if (self._stop_requested_at is not None and self._sample(inputs.feedback, now) is Feedback.STOPPED
                and inputs.feedback.observed_at > self._stop_requested_at):
            self._stop_requested_at = None
            self._stop_accepted = True
        reason = self._deny_reason(inputs, now)
        if reason and self._lease:
            self._revoke(reason, inputs, now)
        return inputs, now, reason

    def tick(self):
        with self._lock:
            self._advance()

    def state(self):
        with self._lock:
            inputs, now, reason = self._advance()
            return {"available": reason is None, "armed": self._lease is not None,
                    "receiver_stop_verified": self._receiver_verified(inputs, now),
                    "reason": reason or ("armed" if self._lease else "ready"),
                    "limits": {"linear_mps": self._limits.linear_mps, "angular_radps": self._limits.angular_radps}}

    def acquire(self, payload):
        if payload != {}:
            return _reject(400, "invalid_request")
        with self._lock:
            inputs, now, reason = self._advance()
            if reason:
                return _reject(409, reason)
            if self._lease:
                return _reject(409, "session_busy")
            expires = min(now + self._limits.lease_s, inputs.grant.expires_at, inputs.receiver_stop.expires_at)
            expires_in_ms = math.floor((expires - now) * 1000)
            if expires_in_ms < 1:
                return _reject(409, "local_authorization_required")
            self._lease = _Lease(secrets.token_urlsafe(24), inputs.grant, expires)
            self._last_grant = inputs.grant.serial
            return 200, {"ok": True, "session_id": self._lease.session_id, "expires_in_ms": expires_in_ms}

    def velocity(self, payload):
        if not isinstance(payload, dict) or set(payload) != {"session_id", "sequence", "linear_mps", "angular_radps"}:
            return _reject(400, "invalid_velocity")
        seq, linear, angular = (payload[key] for key in ("sequence", "linear_mps", "angular_radps"))
        if (not isinstance(payload["session_id"], str) or type(seq) is not int or not 1 <= seq < 2**53
                or not _finite(linear) or not _finite(angular)
                or abs(linear) > self._limits.linear_mps or abs(angular) > self._limits.angular_radps
                or bool(linear) == bool(angular)):
            return _reject(400, "invalid_velocity")
        direction = (Direction.FORWARD if linear > 0 else Direction.BACKWARD) if linear else (
            Direction.LEFT if angular > 0 else Direction.RIGHT)
        with self._lock:
            inputs, now, _ = self._advance()
            lease = self._lease
            if not lease or payload["session_id"] != lease.session_id:
                return _reject(409, "invalid_session")
            if seq <= lease.sequence:
                return _reject(409, "stale_sequence")
            if direction not in inputs.guard.value or direction not in inputs.grant.directions:
                self._revoke("obstacle", inputs, now)
                return _reject(409, "obstacle")
            deadline = min(now + self._limits.deadman_s, lease.expires_at, inputs.grant.expires_at)
            request = VelocityRequest(lease.session_id, seq, lease.grant.serial, now, deadline, linear, angular)
            try:
                accepted = self._output.velocity(request) is True
            except Exception:
                accepted = False
            if not accepted:
                self._revoke("output_rejected", inputs, self._clock())
                return _reject(409, "output_rejected")
            lease.sequence, lease.command_deadline, lease.direction = seq, deadline, direction
            return 200, {"ok": True}

    def stop(self, payload):
        if not isinstance(payload, dict) or set(payload) != {"session_id"} or not isinstance(payload["session_id"], str):
            return _reject(400, "invalid_session")
        with self._lock:
            inputs, now, _ = self._advance()
            session = payload["session_id"]
            if self._lease:
                if session != self._lease.session_id:
                    return _reject(409, "invalid_session")
                self._revoke("operator_stop", inputs, now)
            if session != self._last_revoked:
                return _reject(409, "invalid_session")
            if not self._stop_accepted:
                return _reject(409, "stop_unconfirmed")
            return 200, {"ok": True}
