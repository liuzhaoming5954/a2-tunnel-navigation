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
MAX_ACTION_MS = 30_000
ACTION_DIRECTIONS = frozenset(("forward", "backward", "left", "right"))
ACTION_STATES = frozenset(("ready", "active", "stopping", "release_required"))


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


def _sequence_id(value):
    return type(value) is int and 1 <= value <= MAX_SEQUENCE


def _velocity_request(payload):
    if not isinstance(payload, dict):
        return False
    fields = {"session_id", "sequence", "linear_mps", "angular_radps"}
    if (set(payload) not in (fields, fields | {"action_id"})
            or not _session_id(payload["session_id"]) or not _sequence_id(payload["sequence"])
            or not all(_finite_number(payload[key]) for key in ("linear_mps", "angular_radps"))):
        return False
    # 动作编号来自已确认的起步请求，零速心跳独立于动作编号。
    return ("action_id" not in payload
            or (_sequence_id(payload["action_id"]) and payload["action_id"] < payload["sequence"]
                and bool(payload["linear_mps"] or payload["angular_radps"])))


def _action_request(operation, payload):
    if (not isinstance(payload, dict) or not _session_id(payload.get("session_id"))
            or not _sequence_id(payload.get("sequence"))):
        return False
    if operation == "action_start":
        return (set(payload) == {"session_id", "sequence", "direction"}
                and isinstance(payload["direction"], str) and payload["direction"] in ACTION_DIRECTIONS)
    return (operation == "action_release" and set(payload) == {"session_id", "sequence", "action_id"}
            and _sequence_id(payload["action_id"]) and payload["action_id"] < payload["sequence"])


def _control_action_fields(control):
    if "action_state" not in control and "action_id" not in control:
        return {}
    state, identifier = control.get("action_state"), control.get("action_id")
    valid_id = identifier is None if state == "ready" else _sequence_id(identifier)
    if (control.get("armed") is not True or not isinstance(state, str) or state not in ACTION_STATES
            or not valid_id):
        raise ValueError("Invalid motion action state")
    return {"action_state": state, "action_id": identifier}


def _command_receipt(operation, payload, reply):
    # 同一套回执校验用于机载套接字与站端 HTTP，两个边界都拒绝异常动作编号和时限。
    if operation == "acquire":
        session = reply.get("session_id")
        if not _session_id(session):
            raise ValueError("Invalid session issued by the motion service")
        if "continuous_hold" in reply:
            # 持续按压与限时、连续点动使用互斥的回执格式。含糊的回执不能放行。
            if reply["continuous_hold"] is not True or set(reply) != {"ok", "session_id", "continuous_hold"}:
                raise ValueError("Invalid continuous hold session issued by the motion service")
            return {"ok": True, "session_id": session, "continuous_hold": True}
        if "repeated_actions" in reply:
            limit = reply.get("action_limit_ms")
            if (reply["repeated_actions"] is not True or "expires_in_ms" in reply
                    or type(limit) is not int or not 1 <= limit <= MAX_ACTION_MS):
                raise ValueError("Invalid action limit issued by the motion service")
            return {"ok": True, "session_id": session, "repeated_actions": True, "action_limit_ms": limit}
        expires = reply.get("expires_in_ms")
        if type(expires) is not int or not 1 <= expires <= 60_000:
            raise ValueError("Invalid session lifetime issued by the motion service")
        return {"ok": True, "session_id": session, "expires_in_ms": expires}
    if operation == "action_start":
        identifier, expires = reply.get("action_id"), reply.get("expires_in_ms")
        if (not _sequence_id(identifier) or identifier != payload["sequence"]
                or type(expires) is not int or not 1 <= expires <= MAX_ACTION_MS):
            raise ValueError("Invalid action receipt issued by the motion service")
        return {"ok": True, "action_id": identifier, "expires_in_ms": expires}
    return {"ok": True}


class LocalMotionClient:
    """Relay a fixed command set. The receiver owns motion authority and timeouts."""

    def __init__(self, socket_path, *, peer_user=None, require_source=False):
        path = Path(socket_path)
        if not path.is_absolute():
            raise ValueError("The motion socket path must be absolute")
        self._path = str(path)
        self._require_source = require_source
        if peer_user is None:
            # 既有本机模拟允许当前用户或管理员提供套接字。
            self._peer_uids = {0, os.geteuid()}
        else:
            # 正式机载网关只信受管发令账号。账户缺失必须修正部署，不能退回普通网关账号。
            import pwd
            try:
                self._peer_uids = {pwd.getpwnam(peer_user).pw_uid}
            except KeyError as error:
                raise ValueError(f"未找到配置的遥控接收器账户 {peer_user}") from error

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
            # 对端 UID 由内核提供；身份不符时不会向这个套接字发送会话或指令。
            credentials = connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
            if struct.unpack("3i", credentials)[1] not in self._peer_uids:
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
            source = control.get("source")
            origin = {"source": source} if source in ("mock", "unitree") else {}
            if self._require_source and not origin:
                return self._unavailable("motion_arbiter_unavailable")
            if not control["available"]:
                return {**self._unavailable(self._reason(control, "motion_arbiter_unavailable")), **origin}
            limits = control.get("limits")
            if not isinstance(limits, dict):
                raise ValueError("Invalid motion service limits")
            # 显式前向限速随套接字状态透传。旧接收器仍只报告线速度和角速度。
            keys = ("linear_mps", "angular_radps")
            if "forward_mps" in limits:
                keys += ("forward_mps",)
            if not all(_finite_number(limits.get(key)) and limits[key] > 0 for key in keys):
                raise ValueError("Invalid motion service limits")
            return {**origin, "available": True, "armed": control["armed"],
                    "reason": self._reason(control, "ready"),
                    "limits": {key: limits[key] for key in keys}, **_control_action_fields(control)}
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
        if not _velocity_request(payload):
            return _response_error(400, "invalid_velocity")
        return self._relay("velocity", payload)

    def action_start(self, payload):
        if not _action_request("action_start", payload):
            return _response_error(400, "invalid_action")
        return self._relay("action_start", payload)

    def action_release(self, payload):
        if not _action_request("action_release", payload):
            return _response_error(400, "invalid_action")
        return self._relay("action_release", payload)

    def stop(self, payload):
        if not isinstance(payload, dict) or set(payload) != {"session_id"} or not _session_id(payload["session_id"]):
            return _response_error(400, "invalid_session")
        return self._relay("stop", payload)

    def _relay(self, operation, payload):
        try:
            response = self._request(operation, payload)
            if not response["ok"]:
                return _response_error(409, self._reason(response, "motion_arbiter_rejected"))
            return 200, _command_receipt(operation, payload, response)
        except (AttributeError, OSError, ValueError):
            return _response_error(503, "motion_arbiter_unavailable")
