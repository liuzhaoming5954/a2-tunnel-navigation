"""Client for the local, independently authorized maintenance motion service."""

import json
import math
import os
from pathlib import Path
import re
import socket
import struct
import time


MAX_MESSAGE_BYTES = 4096
REQUEST_TIMEOUT_S = 0.18
SESSION_PATTERN = re.compile(r"[A-Za-z0-9_-]{16,128}\Z")
MAX_SEQUENCE = (1 << 53) - 1


def _response_error(status, reason):
    return status, {"ok": False, "reason": reason}


def _session_id(value):
    return isinstance(value, str) and bool(SESSION_PATTERN.fullmatch(value))


def _finite_number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


class LocalMotionClient:
    """Relay a fixed command set. The receiver owns motion authority and timeouts."""

    def __init__(self, socket_path):
        path = Path(socket_path)
        if not path.is_absolute():
            raise ValueError("The motion socket path must be absolute")
        self._path = str(path)

    def _request(self, operation, payload):
        message = json.dumps({"schema_version": 1, "op": operation, **payload},
                             allow_nan=False, separators=(",", ":")).encode() + b"\n"
        deadline = time.monotonic() + REQUEST_TIMEOUT_S

        def set_remaining_timeout(connection):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Motion service deadline exceeded")
            connection.settimeout(remaining)

        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            set_remaining_timeout(connection)
            connection.connect(self._path)
            # The socket is provided by the operator or administrator. Verify
            # the configured peer runs under the same account or as root.
            credentials = connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
            if struct.unpack("3i", credentials)[1] not in (0, os.geteuid()):
                raise PermissionError("Unexpected motion service account")
            set_remaining_timeout(connection)
            connection.sendall(message)
            reply = bytearray()
            while b"\n" not in reply and len(reply) <= MAX_MESSAGE_BYTES:
                set_remaining_timeout(connection)
                chunk = connection.recv(MAX_MESSAGE_BYTES + 1 - len(reply))
                if not chunk:
                    break
                reply.extend(chunk)
        line, terminator, remainder = reply.partition(b"\n")
        if not terminator or remainder or len(line) + 1 > MAX_MESSAGE_BYTES:
            raise ValueError("Invalid motion service response size")
        response = json.loads(line)
        if not isinstance(response, dict) or type(response.get("ok")) is not bool:
            raise ValueError("Invalid motion service response")
        return response

    @staticmethod
    def _unavailable(reason):
        return {"available": False, "armed": False, "reason": reason,
                "limits": {"linear_mps": 0.0, "angular_radps": 0.0}}

    def state(self):
        try:
            response = self._request("status", {})
            if not response["ok"]:
                return self._unavailable("motion_arbiter_rejected")
            control = response.get("control")
            if (not isinstance(control, dict) or type(control.get("available")) is not bool
                    or type(control.get("armed")) is not bool):
                raise ValueError("Invalid motion service state")
            if control.get("receiver_stop_verified") is not True:
                return self._unavailable("receiver_stop_unverified")
            if not control["available"]:
                return self._unavailable(self._reason(control, "motion_arbiter_unavailable"))
            limits = control.get("limits")
            if (not isinstance(limits, dict) or not all(
                _finite_number(limits.get(key)) and limits[key] > 0
                for key in ("linear_mps", "angular_radps")
            )):
                raise ValueError("Invalid motion service limits")
            return {"available": True, "armed": control["armed"],
                    "reason": self._reason(control, "ready"),
                    "limits": {key: limits[key] for key in ("linear_mps", "angular_radps")}}
        except (AttributeError, OSError, ValueError):
            return self._unavailable("motion_arbiter_unavailable")

    @staticmethod
    def _reason(response, fallback):
        reason = response.get("reason")
        return reason if isinstance(reason, str) and 0 < len(reason) <= 160 else fallback

    def acquire(self, payload):
        if payload != {}:
            return _response_error(400, "invalid_request")
        control = self.state()
        if not control["available"]:
            return _response_error(409, control["reason"])
        return self._relay("acquire", {})

    def velocity(self, payload):
        if (not isinstance(payload, dict)
                or set(payload) != {"session_id", "sequence", "linear_mps", "angular_radps"}
                or not _session_id(payload["session_id"])
                or type(payload["sequence"]) is not int or not 1 <= payload["sequence"] <= MAX_SEQUENCE
                or not all(_finite_number(payload[key]) for key in ("linear_mps", "angular_radps"))):
            return _response_error(400, "invalid_velocity")
        return self._relay("velocity", payload)

    def stop(self, payload):
        if not isinstance(payload, dict) or set(payload) != {"session_id"} or not _session_id(payload["session_id"]):
            return _response_error(400, "invalid_session")
        return self._relay("stop", payload)

    def _relay(self, operation, payload):
        try:
            response = self._request(operation, payload)
            if not response["ok"]:
                return _response_error(409, self._reason(response, "motion_arbiter_rejected"))
            if operation == "acquire":
                session = response.get("session_id")
                expires = response.get("expires_in_ms")
                if not _session_id(session) or type(expires) is not int or not 1 <= expires <= 60_000:
                    raise ValueError("Invalid session issued by the motion service")
                return 200, {"ok": True, "session_id": session, "expires_in_ms": expires}
            return 200, {"ok": True}
        except (AttributeError, OSError, ValueError):
            return _response_error(503, "motion_arbiter_unavailable")
