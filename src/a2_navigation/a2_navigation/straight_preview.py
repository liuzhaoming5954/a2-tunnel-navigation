"""Preview a straight route from map-frame poses without any motion I/O."""

import math
from dataclasses import dataclass

from .offline_planner import finite_number


GEOMETRY_TOLERANCE_M = 1e-6


def _point(value, field: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"{field} must contain two coordinates in metres")
    return finite_number(value[0], field), finite_number(value[1], field)


def _wrap_angle(angle: float) -> float:
    return math.remainder(angle, math.tau)


@dataclass(frozen=True)
class StraightRoute:
    start_m: tuple[float, float]
    goal_m: tuple[float, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "start_m", _point(self.start_m, "start_m"))
        object.__setattr__(self, "goal_m", _point(self.goal_m, "goal_m"))
        if not math.isfinite(self.length_m) or self.length_m <= GEOMETRY_TOLERANCE_M:
            raise ValueError("The straight route needs distinct finite endpoints")

    @property
    def length_m(self) -> float:
        return math.dist(self.start_m, self.goal_m)

    @property
    def heading_rad(self) -> float:
        return math.atan2(self.goal_m[1] - self.start_m[1], self.goal_m[0] - self.start_m[0])

    def _project(self, x_m: float, y_m: float) -> tuple[float, float]:
        """Return progress along the route and signed cross-track displacement."""
        dx = x_m - self.start_m[0]
        dy = y_m - self.start_m[1]
        cosine = math.cos(self.heading_rad)
        sine = math.sin(self.heading_rad)
        return dx * cosine + dy * sine, -dx * sine + dy * cosine


def load_straight_route(plan: dict) -> StraightRoute:
    """Read the file planner's map-frame result and check its straight geometry."""
    if not isinstance(plan, dict) or plan.get("status") != "planned" or plan.get("frame_id") != "map":
        raise ValueError("A planned map-frame route is required")
    route = StraightRoute(plan.get("start_m"), plan.get("goal_m"))
    length_m = route.length_m
    reported_length_m = finite_number(plan.get("path_length_m"), "path_length_m")
    if not math.isclose(length_m, reported_length_m, rel_tol=0, abs_tol=GEOMETRY_TOLERANCE_M):
        raise ValueError("The planned length does not match a straight route")

    path = plan.get("path_m")
    if not isinstance(path, (list, tuple)) or len(path) < 2:
        raise ValueError("path_m must contain the start and goal")
    points = [_point(value, "path_m") for value in path]
    if (math.dist(points[0], route.start_m) > GEOMETRY_TOLERANCE_M
            or math.dist(points[-1], route.goal_m) > GEOMETRY_TOLERANCE_M):
        raise ValueError("The path must start and finish at the declared endpoints")

    previous_progress_m = 0.0
    for x_m, y_m in points:
        progress_m, cross_track_m = route._project(x_m, y_m)
        if (not math.isfinite(progress_m) or not math.isfinite(cross_track_m)
                or abs(cross_track_m) > GEOMETRY_TOLERANCE_M
                or progress_m < previous_progress_m - GEOMETRY_TOLERANCE_M
                or progress_m > length_m + GEOMETRY_TOLERANCE_M):
            raise ValueError("The path must stay on the straight segment and move toward its goal")
        previous_progress_m = max(previous_progress_m, progress_m)
    return route


@dataclass(frozen=True)
class PreviewCommand:
    state: str
    linear_x_mps: float
    angular_z_radps: float
    remaining_m: float | None
    reason: str | None = None


class StraightPreview:
    """Compute bounded diagnostic velocity values and latch arrival or stop."""

    def __init__(
        self,
        route: StraightRoute,
        *,
        max_speed_mps: float = 0.15,
        max_yaw_rate_radps: float = 0.40,
        goal_tolerance_m: float = 0.05,
        start_tolerance_m: float = 0.20,
        cross_track_tolerance_m: float = 0.15,
        heading_tolerance_rad: float = 0.35,
        max_pose_age_ns: int = 250_000_000,
    ) -> None:
        self._route = route
        self._max_speed_mps = finite_number(max_speed_mps, "max_speed_mps")
        self._max_yaw_rate_radps = finite_number(max_yaw_rate_radps, "max_yaw_rate_radps")
        self._goal_tolerance_m = finite_number(goal_tolerance_m, "goal_tolerance_m")
        self._start_tolerance_m = finite_number(start_tolerance_m, "start_tolerance_m")
        self._cross_track_tolerance_m = finite_number(cross_track_tolerance_m, "cross_track_tolerance_m")
        self._heading_tolerance_rad = finite_number(heading_tolerance_rad, "heading_tolerance_rad")
        if self._max_speed_mps <= 0 or self._max_yaw_rate_radps <= 0:
            raise ValueError("Preview speed limits must be positive")
        if not 0 < self._goal_tolerance_m < route.length_m:
            raise ValueError("Goal tolerance must be positive and shorter than the route")
        if self._start_tolerance_m <= 0 or self._cross_track_tolerance_m <= 0:
            raise ValueError("Start and cross-track tolerances must be positive")
        if not 0 < self._heading_tolerance_rad < math.pi / 2:
            raise ValueError("Heading tolerance must be between zero and pi/2 radians")
        if type(max_pose_age_ns) is not int or max_pose_age_ns <= 0:
            raise ValueError("max_pose_age_ns must be a positive integer")
        self._max_pose_age_ns = max_pose_age_ns
        self._terminal: PreviewCommand | None = None
        self._previous_stamp_ns: int | None = None
        self._previous_now_ns: int | None = None
        self._remaining_m: float | None = None

    def stop(self, reason: str) -> PreviewCommand:
        """Latch a zero-velocity result, for example when a pose lookup fails."""
        if self._terminal is not None:
            return self._terminal
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("A stop reason is required")
        self._terminal = PreviewCommand("stopped", 0.0, 0.0, self._remaining_m, reason)
        return self._terminal

    def step(
        self,
        *,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        pose_stamp_ns: int,
        now_ns: int,
    ) -> PreviewCommand:
        """Check pose freshness in one clock domain and advance the preview."""
        if self._terminal is not None:
            return self._terminal
        if (type(pose_stamp_ns) is not int or type(now_ns) is not int
                or pose_stamp_ns < 0 or now_ns < 0):
            return self.stop("invalid_timestamp")
        if pose_stamp_ns > now_ns:
            return self.stop("future_pose")
        if now_ns - pose_stamp_ns > self._max_pose_age_ns:
            return self.stop("stale_pose")
        if self._previous_now_ns is not None and now_ns < self._previous_now_ns:
            return self.stop("clock_regressed")
        if self._previous_stamp_ns is not None and pose_stamp_ns < self._previous_stamp_ns:
            return self.stop("pose_time_regressed")

        try:
            x_m = finite_number(x_m, "x_m")
            y_m = finite_number(y_m, "y_m")
            yaw_rad = finite_number(yaw_rad, "yaw_rad")
        except ValueError:
            return self.stop("invalid_pose")
        progress_m, cross_track_m = self._route._project(x_m, y_m)
        remaining_m = self._route.length_m - progress_m
        if not math.isfinite(remaining_m) or not math.isfinite(cross_track_m):
            return self.stop("invalid_pose")
        self._remaining_m = max(0.0, remaining_m)
        if self._previous_stamp_ns is None:
            if math.dist((x_m, y_m), self._route.start_m) > self._start_tolerance_m:
                return self.stop("not_at_start")
        elif progress_m < -self._start_tolerance_m:
            return self.stop("before_start")
        if remaining_m < -GEOMETRY_TOLERANCE_M:
            return self.stop("overshoot")
        if abs(cross_track_m) > self._cross_track_tolerance_m:
            return self.stop("off_path")
        heading_error_rad = _wrap_angle(_wrap_angle(yaw_rad) - self._route.heading_rad)
        if abs(heading_error_rad) > self._heading_tolerance_rad:
            return self.stop("wrong_heading")
        if math.dist((x_m, y_m), self._route.goal_m) <= self._goal_tolerance_m:
            self._terminal = PreviewCommand("arrived", 0.0, 0.0, self._remaining_m)
            return self._terminal
        if remaining_m <= GEOMETRY_TOLERANCE_M:
            return self.stop("outside_goal_tolerance")

        self._previous_stamp_ns = pose_stamp_ns
        self._previous_now_ns = now_ns
        desired_error_rad = -heading_error_rad - math.atan2(cross_track_m, 0.50)
        angular_z_radps = max(-self._max_yaw_rate_radps, min(self._max_yaw_rate_radps, 2.0 * desired_error_rad))
        linear_x_mps = min(self._max_speed_mps, remaining_m) * max(0.0, math.cos(desired_error_rad))
        return PreviewCommand("tracking", linear_x_mps, angular_z_radps, self._remaining_m)
