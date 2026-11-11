"""Plan a position-only path on a ROS trinary occupancy map."""

import hashlib
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path


MAX_CELLS = 4_000_000
PGM_WHITESPACE = b" \t\n\r\v\f"
DIAGONAL_COST = math.sqrt(2.0)
NEIGHBORS = (
    (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
    (1, 1, DIAGONAL_COST), (-1, 1, DIAGONAL_COST),
    (-1, -1, DIAGONAL_COST), (1, -1, DIAGONAL_COST),
)

Cell = tuple[int, int]
Point = tuple[float, float]


def finite_number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a finite number")
    try:
        number = float(value)
    except OverflowError as error:
        raise ValueError(f"{field} must be a finite number") from error
    if not math.isfinite(number):
        raise ValueError(f"{field} must be a finite number")
    return number


def load_metadata(content):
    try:
        metadata = json.loads(content)
    except json.JSONDecodeError:
        try:
            import yaml
        except ImportError as error:
            raise ValueError("PyYAML is required to read non-JSON map YAML") from error
        try:
            metadata = yaml.safe_load(content)
        except yaml.YAMLError as error:
            raise ValueError(f"Invalid map YAML: {error}") from error
    if not isinstance(metadata, dict):
        raise ValueError("Map metadata must be an object")
    return metadata


def read_pgm(content):
    position = 0

    def token():
        nonlocal position
        while position < len(content):
            character = content[position]
            if character in PGM_WHITESPACE:
                position += 1
            elif character == ord("#"):
                newline = content.find(b"\n", position + 1)
                position = len(content) if newline < 0 else newline + 1
            else:
                break
        start = position
        while position < len(content) and content[position] not in PGM_WHITESPACE + b"#":
            position += 1
        return content[start:position]

    magic = token()
    if magic not in (b"P2", b"P5"):
        raise ValueError("Map image must be a P2 or P5 PGM")
    try:
        width, height, maximum = (int(token()) for _ in range(3))
    except ValueError as error:
        raise ValueError("Invalid PGM header") from error
    count = width * height
    if width <= 0 or height <= 0 or count > MAX_CELLS or not 1 <= maximum <= 255:
        raise ValueError("Expected positive PGM dimensions and 8-bit pixels within the map size limit")
    if magic == b"P5":
        if position == len(content) or content[position] not in PGM_WHITESPACE:
            raise ValueError("P5 header must end with a whitespace delimiter")
        crlf = content[position:position + 2] == b"\r\n" and len(content) - position - 2 == count
        position += 2 if crlf else 1
        pixels = content[position:]
    else:
        try:
            pixels = bytes(int(token()) for _ in range(count))
        except ValueError as error:
            raise ValueError("Invalid P2 raster") from error
        if token():
            raise ValueError("Unexpected data after the P2 raster")
    if len(pixels) != count or max(pixels) > maximum:
        raise ValueError("PGM raster does not match its header")
    return width, height, maximum, pixels


@dataclass(frozen=True)
class OccupancyMap:
    width: int
    height: int
    resolution: float
    origin: tuple[float, float, float]
    known_free: tuple[int, ...]
    metadata_sha256: str
    image_sha256: str

    def contains(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def world_to_cell(self, point: Point) -> Cell | None:
        x, y, yaw = self.origin
        dx, dy = point[0] - x, point[1] - y
        cosine, sine = math.cos(yaw), math.sin(yaw)
        local_x = (cosine * dx + sine * dy) / self.resolution
        local_y = (-sine * dx + cosine * dy) / self.resolution
        if not (0 <= local_x < self.width and 0 <= local_y < self.height):
            return None
        return math.floor(local_x), math.floor(local_y)

    def cell_center(self, cell: Cell) -> Point:
        x, y, yaw = self.origin
        local_x = (cell[0] + 0.5) * self.resolution
        local_y = (cell[1] + 0.5) * self.resolution
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return (x + cosine * local_x - sine * local_y,
                y + sine * local_x + cosine * local_y)


def load_map(path: Path) -> OccupancyMap:
    metadata_bytes = path.read_bytes()
    metadata = load_metadata(metadata_bytes.decode("utf-8"))
    image = metadata.get("image")
    if not isinstance(image, str) or not image:
        raise ValueError("Map metadata needs an image path")
    if metadata.get("mode", "trinary") != "trinary":
        raise ValueError("Only trinary occupancy maps are supported")
    resolution = finite_number(metadata.get("resolution"), "resolution")
    origin = metadata.get("origin")
    if resolution <= 0 or not isinstance(origin, list) or len(origin) != 3:
        raise ValueError("Map needs a positive resolution and origin [x, y, yaw]")
    origin = tuple(finite_number(value, "origin") for value in origin)
    negate = metadata.get("negate", 0)
    occupied = finite_number(metadata.get("occupied_thresh"), "occupied_thresh")
    free = finite_number(metadata.get("free_thresh"), "free_thresh")
    if type(negate) is not int or negate not in (0, 1) or not 0 <= free < occupied <= 1:
        raise ValueError("Expected negate 0 or 1 and 0 <= free_thresh < occupied_thresh <= 1")
    image_path = path.parent / image
    if image_path.stat().st_size > 4 * MAX_CELLS + 65_536:
        raise ValueError("PGM exceeds the map size limit")
    image_bytes = image_path.read_bytes()
    width, height, maximum, pixels = read_pgm(image_bytes)
    if not math.isfinite(math.hypot(*origin[:2]) + resolution * (width + height)):
        raise ValueError("Map coordinate extent must be finite")
    shades = [pixel / maximum for pixel in range(maximum + 1)]
    occupancy = shades if negate else [1.0 - shade for shade in shades]
    # PGM's first row is the upper edge. Planner row zero is the map origin side.
    known_free = []
    for y in range(height - 1, -1, -1):
        row = pixels[y * width:(y + 1) * width]
        packed = bytearray((width + 7) // 8)
        for x, pixel in enumerate(row):
            if occupancy[pixel] < free:
                packed[x // 8] |= 1 << (x % 8)
        known_free.append(int.from_bytes(packed, "little"))
    return OccupancyMap(
        width, height, resolution, origin, tuple(known_free),
        hashlib.sha256(metadata_bytes).hexdigest(), hashlib.sha256(image_bytes).hexdigest(),
    )


def traversable_rows(grid: OccupancyMap, radius_m: float) -> tuple[int, ...]:
    """Keep cells whose entire square is farther than the radius from obstacles."""
    radius_cells = radius_m / grid.resolution
    if not math.isfinite(radius_cells) or radius_cells * 2 >= min(grid.width, grid.height):
        return (0,) * grid.height
    full_row = (1 << grid.width) - 1
    blocked = tuple(full_row ^ row for row in grid.known_free)
    edge = math.floor(radius_cells) + 1
    interior = (full_row >> (2 * edge)) << edge
    if not interior or grid.height <= 2 * edge:
        return (0,) * grid.height
    forbidden = [0] * grid.height
    # For square cells, center indices that differ by 1 still touch. For each
    # vertical displacement compute the maximum horizontal displacement whose
    # two cell squares are within radius_m.
    for dy in range(edge + 1):
        separation = max(0, dy - 1)
        dx = math.floor(math.sqrt(max(0, radius_cells ** 2 - separation ** 2))) + 1
        for y, row in enumerate(blocked):
            if not row:
                continue
            widened = row
            span = 0
            while span < dx:
                shift = min(2 * span + 1, dx - span)
                widened |= (widened << shift) | (widened >> shift)
                span += shift
            if y + dy < grid.height:
                forbidden[y + dy] |= widened
            if dy and y >= dy:
                forbidden[y - dy] |= widened
    return tuple(interior & ~forbidden[y] if edge <= y < grid.height - edge else 0
                 for y in range(grid.height))


def is_traversable(grid: OccupancyMap, rows: tuple[int, ...], cell: Cell) -> bool:
    return grid.contains(cell) and bool(rows[cell[1]] & (1 << cell[0]))


def search(grid: OccupancyMap, rows: tuple[int, ...], start: Cell, goal: Cell) -> list[Cell]:
    def heuristic(cell):
        dx, dy = abs(goal[0] - cell[0]), abs(goal[1] - cell[1])
        return max(dx, dy) + (DIAGONAL_COST - 1.0) * min(dx, dy)

    queue = [(heuristic(start), 0.0, start)]
    best_cost = {start: 0.0}
    previous = {}
    while queue:
        _, cost, cell = heapq.heappop(queue)
        if cost != best_cost[cell]:
            continue
        if cell == goal:
            path = [cell]
            while cell != start:
                cell = previous[cell]
                path.append(cell)
            return list(reversed(path))
        x, y = cell
        for dx, dy, step_cost in NEIGHBORS:
            neighbor = x + dx, y + dy
            if not is_traversable(grid, rows, neighbor):
                continue
            if dx and dy and (not is_traversable(grid, rows, (x + dx, y)) or
                              not is_traversable(grid, rows, (x, y + dy))):
                continue
            new_cost = cost + step_cost
            if new_cost < best_cost.get(neighbor, math.inf):
                best_cost[neighbor] = new_cost
                previous[neighbor] = cell
                heapq.heappush(queue, (new_cost + heuristic(neighbor), new_cost, neighbor))
    return []


def plan(grid: OccupancyMap, start: Point, goal: Point, radius_m: float, via=()) -> dict:
    radius_m = finite_number(radius_m, "footprint radius")
    if radius_m <= 0:
        raise ValueError("Footprint radius must be positive")
    endpoints = tuple(tuple(finite_number(value, "position") for value in point)
                      for point in (start, *via, goal))
    if any(len(point) != 2 for point in endpoints):
        raise ValueError("Every position must contain two coordinates in metres")
    report = {
        "frame_id": "map",
        "map": {
            "width": grid.width,
            "height": grid.height,
            "resolution_m": grid.resolution,
            "origin": grid.origin,
            "metadata_sha256": grid.metadata_sha256,
            "image_sha256": grid.image_sha256,
        },
        "start_m": endpoints[0],
        "goal_m": endpoints[-1],
        "via_m": endpoints[1:-1],
        "footprint_radius_m": radius_m,
    }
    rows = traversable_rows(grid, radius_m)
    cells = []
    for index, point in enumerate(endpoints):
        cell = grid.world_to_cell(point)
        if cell is None:
            reason = "outside_map"
        elif not is_traversable(grid, grid.known_free, cell):
            reason = "not_known_free"
        elif not is_traversable(grid, rows, cell):
            reason = "insufficient_clearance"
        else:
            cells.append(cell)
            continue
        status = "invalid_start" if index == 0 else "invalid_goal" if index == len(endpoints) - 1 else "invalid_via"
        return {**report, "status": status, "reason": reason, "endpoint_index": index}

    cell_path = []
    world_path = []

    def append_point(point):
        if not world_path or world_path[-1] != point:
            world_path.append(point)

    for index, (left, right) in enumerate(zip(cells, cells[1:])):
        segment = search(grid, rows, left, right)
        if not segment:
            return {**report, "status": "unreachable", "segment_index": index}
        cell_path.extend(segment[1:] if cell_path else segment)
        append_point(endpoints[index])
        if left != right:
            for cell in segment:
                append_point(grid.cell_center(cell))
        append_point(endpoints[index + 1])
    return {
        **report, "status": "planned", "path_cells": cell_path, "path_m": world_path,
        "path_length_m": sum(math.dist(left, right) for left, right in zip(world_path, world_path[1:])),
    }
