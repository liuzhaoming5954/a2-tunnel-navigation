"""Synthetic A2 telemetry and a server-owned simulation control lease."""

from dataclasses import dataclass
import math
import secrets
import threading
import time


LINEAR_LIMIT_MPS = 0.45
ANGULAR_LIMIT_RADPS = 1.20
LEASE_SECONDS = 30.0
DEADMAN_SECONDS = 0.40
TICK_SECONDS = 0.02
MAX_SEQUENCE = (1 << 53) - 1
ROBOT_RADIUS_M = 0.32

WIDTH = 104
HEIGHT = 72
RESOLUTION_M = 0.25
ORIGIN_X_M = -13.0
ORIGIN_Y_M = -8.0
MAP_ID = "sim-tunnel"


def _base_grid():
    free = set()

    def floor(left, bottom, right, top):
        free.update((x, y) for y in range(bottom, top) for x in range(left, right))

    floor(10, 22, 96, 34)
    floor(20, 29, 32, 58)
    floor(71, 29, 82, 58)
    floor(24, 49, 78, 59)
    floor(42, 34, 53, 50)
    floor(33, 58, 43, 66)
    floor(77, 55, 93, 63)
    floor(8, 36, 22, 44)
    floor(16, 33, 23, 39)
    cells = [-1] * (WIDTH * HEIGHT)
    for x, y in free:
        cells[y * WIDTH + x] = 0
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            xx, yy = x + dx, y + dy
            if (xx, yy) not in free:
                cells[yy * WIDTH + xx] = 100
    for x, y in ((42, 33), (43, 33), (44, 33), (28, 51), (73, 43), (74, 43)):
        cells[y * WIDTH + x] = 100
    # Parked equipment cart on the remote spur.
    for y in range(58, 61):
        for x in range(83, 86):
            cells[y * WIDTH + x] = 100
    return tuple(cells)


BASE_GRID = _base_grid()


def _velocity(value, limit):
    if type(value) not in (float, int) or not -limit <= value <= limit:
        return None
    return float(value)


def _error(status, reason):
    return status, {"ok": False, "reason": reason}


@dataclass
class Lease:
    session_id: str
    expires_at: float
    sequence: int = -1
    command_deadline: float | None = None


class Simulator:
    """Integrate simulated velocities and enforce lease/deadman independently of HTTP."""

    def __init__(self):
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._last_tick = time.monotonic()
        self._lease = None
        self._reason = "idle"
        self._x, self._y, self._yaw = -4.85, -1.1, 0.0
        self._battery = 86.0
        self._request_linear = self._request_angular = 0.0
        self._linear = self._angular = 0.0
        self._worker = threading.Thread(target=self._run, name="a2-simulator", daemon=True)
        self._worker.start()

    def _run(self):
        while not self._closed.wait(TICK_SECONDS):
            with self._lock:
                self._advance()

    def close(self):
        self._closed.set()
        with self._lock:
            self._revoke("simulator_closed")
        self._worker.join()

    def _revoke(self, reason):
        self._lease = None
        self._request_linear = self._request_angular = 0.0
        self._linear = self._angular = 0.0
        self._reason = reason

    def _can_occupy(self, x, y):
        radius = ROBOT_RADIUS_M / RESOLUTION_M
        column = (x - ORIGIN_X_M) / RESOLUTION_M
        row = (y - ORIGIN_Y_M) / RESOLUTION_M
        for yy in range(math.floor(row - radius), math.floor(row + radius) + 1):
            for xx in range(math.floor(column - radius), math.floor(column + radius) + 1):
                if not (0 <= xx < WIDTH and 0 <= yy < HEIGHT):
                    return False
                if BASE_GRID[yy * WIDTH + xx] == 0:
                    continue
                dx = max(xx - column, 0.0, column - xx - 1)
                dy = max(yy - row, 0.0, row - yy - 1)
                if dx * dx + dy * dy <= radius * radius:
                    return False
        return True

    def _step(self, elapsed):
        heading = self._yaw + self._request_angular * elapsed / 2.0
        distance = self._request_linear * elapsed
        next_x = self._x + distance * math.cos(heading)
        next_y = self._y + distance * math.sin(heading)
        self._angular = self._request_angular
        self._yaw = (self._yaw + self._angular * elapsed + math.pi) % (2.0 * math.pi) - math.pi
        if not distance or self._can_occupy(next_x, next_y):
            self._x, self._y = next_x, next_y
            self._linear = self._request_linear
            self._reason = "armed"
        else:
            self._linear = 0.0
            self._reason = "obstacle"
        load = abs(self._linear) / LINEAR_LIMIT_MPS + abs(self._angular) / ANGULAR_LIMIT_RADPS
        self._battery = max(0.0, self._battery - elapsed * (0.0008 + 0.006 * load))

    def _advance(self):
        now = time.monotonic()
        if self._lease is None:
            self._last_tick = now
            return now

        cutoff = self._lease.expires_at
        reason = "lease_expired"
        if self._lease.command_deadline is not None and self._lease.command_deadline < cutoff:
            cutoff = self._lease.command_deadline
            reason = "deadman_expired"
        elapsed = max(0.0, min(now, cutoff) - self._last_tick)
        if self._request_linear or self._request_angular:
            steps = max(1, math.ceil(elapsed / TICK_SECONDS))
            for _ in range(steps):
                self._step(elapsed / steps)
        if now >= cutoff:
            self._revoke(reason)
        self._last_tick = now
        return now

    def _map_metadata(self):
        return {
            "id": MAP_ID,
            "frame_id": "map",
            "width": WIDTH,
            "height": HEIGHT,
            "resolution_m": RESOLUTION_M,
            "origin_x_m": ORIGIN_X_M,
            "origin_y_m": ORIGIN_Y_M,
            "origin_yaw_rad": 0.0,
        }

    def snapshot(self):
        with self._lock:
            self._advance()
            stamp_ms = time.time_ns() // 1_000_000
            grid = self._map_metadata()
            return {
                "robot": {
                    "id": "a2-simulator",
                    "model": "Unitree A2-PRO",
                    "online": not self._closed.is_set(),
                    "mode": "walking" if self._linear or self._angular else "standing",
                    "battery_percent": round(self._battery, 2),
                    "linear_mps": self._linear,
                    "angular_radps": self._angular,
                    "telemetry_stamp_ms": stamp_ms,
                    "telemetry_age_ms": 0,
                },
                "pose": {
                    "x_m": self._x,
                    "y_m": self._y,
                    "yaw_rad": self._yaw,
                    "stamp_ms": stamp_ms,
                    "age_ms": 0,
                    "map_id": grid["id"],
                },
                "map": grid,
                "control": self._control_state(),
            }

    def map_data(self):
        return {**self._map_metadata(), "cells": list(BASE_GRID)}

    def _control_state(self):
        return {
            "available": not self._closed.is_set(),
            "armed": self._lease is not None,
            "reason": self._reason,
            "limits": {"linear_mps": LINEAR_LIMIT_MPS, "angular_radps": ANGULAR_LIMIT_RADPS},
        }

    def state(self):
        with self._lock:
            self._advance()
            return self._control_state()

    def acquire(self, payload):
        if payload != {}:
            return _error(400, "invalid_session_request")
        with self._lock:
            now = self._advance()
            if self._closed.is_set():
                return _error(503, "simulator_closed")
            if self._lease is not None:
                return _error(409, "session_busy")
            self._lease = Lease(secrets.token_urlsafe(24), now + LEASE_SECONDS)
            self._reason = "armed"
            return 200, {
                "ok": True,
                "session_id": self._lease.session_id,
                "expires_in_ms": round(LEASE_SECONDS * 1000),
            }

    def _valid_session(self, payload):
        token = payload.get("session_id")
        return self._lease is not None and isinstance(token, str) and token == self._lease.session_id

    def velocity(self, payload):
        if set(payload) != {"session_id", "sequence", "linear_mps", "angular_radps"}:
            return _error(400, "invalid_velocity_request")
        sequence = payload["sequence"]
        linear = _velocity(payload["linear_mps"], LINEAR_LIMIT_MPS)
        angular = _velocity(payload["angular_radps"], ANGULAR_LIMIT_RADPS)
        if type(sequence) is not int or not 0 <= sequence <= MAX_SEQUENCE or linear is None or angular is None:
            return _error(400, "invalid_velocity_request")
        with self._lock:
            now = self._advance()
            if not self._valid_session(payload):
                return _error(409, "invalid_session")
            if sequence <= self._lease.sequence:
                return _error(409, "stale_sequence")
            self._lease.sequence = sequence
            self._lease.command_deadline = now + DEADMAN_SECONDS
            self._request_linear, self._request_angular = linear, angular
            if not linear:
                self._linear = 0.0
            if not angular:
                self._angular = 0.0
            return 200, {"ok": True}

    def stop(self, payload):
        if set(payload) != {"session_id"}:
            return _error(400, "invalid_stop_request")
        with self._lock:
            self._advance()
            if not self._valid_session(payload):
                return _error(409, "invalid_session")
            self._revoke("operator_stop")
            return 200, {"ok": True}
