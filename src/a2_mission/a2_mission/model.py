"""States implemented by the first mission-core increment."""

from enum import Enum
from typing import Callable, Optional, Protocol


class State(str, Enum):
    READY = "READY"
    PRECHECK = "PRECHECK"
    LOCALIZING = "LOCALIZING"
    INSPECTING = "INSPECTING"
    PAUSED_OPERATOR = "PAUSED_OPERATOR"
    PAUSED_FAULT = "PAUSED_FAULT"
    CANCELLED = "CANCELLED"


class Journal(Protocol):
    def read_snapshot(self, robot_id: str) -> Optional[dict]: ...

    def apply(self, robot_id: str, request_id: str, payload: dict,
              transition: Callable) -> dict: ...
