"""Validate mock launch inputs without importing ROS or starting processes."""

import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class MockConfiguration:
    robot_id: str
    database_path: str
    mock_plan_path: str
    platform_drop_after_s: float


def parse_mock_configuration(*, profile: str, robot_id: str, database_path: str,
                             mock_plan_path: str,
                             platform_drop_after_s: str) -> MockConfiguration:
    if profile != "mock":
        raise ValueError("This launch only supports profile=mock; hardware profiles are unavailable")
    if not isinstance(robot_id, str) or not robot_id.strip() or len(robot_id) > 256:
        raise ValueError("robot_id must be a nonempty string of at most 256 characters")
    database = Path(database_path)
    if not database.is_absolute() or (database.exists() and not database.is_file()):
        raise ValueError("database_path must be an absolute file path")
    plan = Path(mock_plan_path)
    if not plan.is_absolute() or not plan.is_file():
        raise ValueError("mock_plan_path must name an existing absolute file")
    try:
        drop = float(platform_drop_after_s)
    except (TypeError, ValueError) as error:
        raise ValueError("platform_drop_after_s must be numeric") from error
    if not math.isfinite(drop) or (drop < 0 and drop != -1) or drop > 86400:
        raise ValueError("platform_drop_after_s must be -1 or between 0 and 86400")
    return MockConfiguration(robot_id, str(database), str(plan), drop)
