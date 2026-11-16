"""Convert a timestamped front LiDAR pose to a planar body pose."""

from collections.abc import Sequence
from dataclasses import dataclass
import math
from numbers import Real


_RIGID_TOLERANCE = 1e-6
_MIN_HORIZONTAL_FORWARD = 1e-6


@dataclass(frozen=True, slots=True)
class BodyPose:
    """Body origin and forward heading projected onto the map plane."""

    x_m: float
    y_m: float
    yaw_rad: float
    stamp_ns: int


def _finite_components(values: Sequence[float], count: int, name: str) -> tuple[float, ...]:
    try:
        components = tuple(values)
        if len(components) != count or any(
            isinstance(value, bool) or not isinstance(value, Real) for value in components
        ):
            raise ValueError
        result = tuple(float(value) for value in components)
        if not all(map(math.isfinite, result)):
            raise ValueError
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain {count} finite real numbers") from exc
    return result


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(a * b for a, b in zip(left, right))


def _near(value: float, target: float) -> bool:
    return math.isclose(value, target, rel_tol=0.0, abs_tol=_RIGID_TOLERANCE)


def _rigid_matrix(value: Sequence[Sequence[float]], name: str) -> tuple[tuple[float, ...], ...]:
    try:
        rows = tuple(value)
    except TypeError as exc:
        raise ValueError(f"{name} must be a 4x4 matrix") from exc
    if len(rows) != 4:
        raise ValueError(f"{name} must be a 4x4 matrix")
    matrix = tuple(_finite_components(row, 4, f"{name} row") for row in rows)
    if any(not _near(actual, expected) for actual, expected in zip(matrix[3], (0, 0, 0, 1))):
        raise ValueError(f"{name} must have a homogeneous last row")

    rotation = tuple(row[:3] for row in matrix[:3])
    for i, left in enumerate(rotation):
        for j, right in enumerate(rotation[: i + 1]):
            if not _near(_dot(left, right), float(i == j)):
                raise ValueError(f"{name} rotation must be orthonormal")

    (a, b, c), (d, e, f), (g, h, i) = rotation
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if not _near(determinant, 1.0):
        raise ValueError(f"{name} rotation must be right-handed")
    return matrix


def pose_matrix(
    translation_xyz: Sequence[float], quaternion_xyzw: Sequence[float]
) -> tuple[tuple[float, ...], ...]:
    """Build a homogeneous transform from a translation and unit XYZW quaternion."""
    tx, ty, tz = _finite_components(translation_xyz, 3, "translation")
    quaternion = _finite_components(quaternion_xyzw, 4, "quaternion")
    norm = math.hypot(*quaternion)
    if not _near(norm, 1.0):
        raise ValueError("quaternion must have unit norm")
    x, y, z, w = (value / norm for value in quaternion)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), tx),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), ty),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), tz),
        (0.0, 0.0, 0.0, 1.0),
    )


def body_pose_from_lidar(
    transform: Sequence[Sequence[float]],
    base_from_lidar: Sequence[Sequence[float]],
    stamp_ns: int,
) -> BodyPose:
    """Compose map-from-LiDAR with LiDAR-from-base, preserving sensor time."""
    if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns < 0:
        raise ValueError("stamp_ns must be a nonnegative integer")
    map_from_lidar = _rigid_matrix(transform, "map-from-LiDAR")
    base_from_lidar = _rigid_matrix(base_from_lidar, "base-from-LiDAR")

    # The inverse extrinsic uses the transpose of its rotation. Compose in 3D
    # so roll, pitch and the mounting offset contribute before projection.
    map_from_base_rotation = tuple(
        tuple(_dot(map_row[:3], base_row[:3]) for base_row in base_from_lidar[:3])
        for map_row in map_from_lidar[:3]
    )
    base_from_lidar_translation = tuple(row[3] for row in base_from_lidar[:3])
    position = tuple(
        map_row[3] - _dot(base_row, base_from_lidar_translation)
        for map_row, base_row in zip(map_from_lidar[:3], map_from_base_rotation)
    )
    if not all(map(math.isfinite, position)):
        raise ValueError("composed body position must be finite")
    forward_x, forward_y = (row[0] for row in map_from_base_rotation[:2])
    if math.hypot(forward_x, forward_y) <= _MIN_HORIZONTAL_FORWARD:
        raise ValueError("body forward axis has no usable horizontal projection")
    return BodyPose(position[0], position[1], math.atan2(forward_y, forward_x), stamp_ns)
