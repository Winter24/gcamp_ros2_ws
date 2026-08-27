"""Pure-Python lane-graph loading, snapping, and forward-only A* routing."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import heapq
import math
from pathlib import Path
from typing import Callable, Mapping, Sequence

import yaml


Point = tuple[float, float]
_HARD_MAX_TURN_DEG = 150.0


class SnapError(ValueError):
    """A start or goal cannot be projected onto a valid road corridor."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


class RouteError(ValueError):
    """A route cannot be produced, with a stable planner failure reason."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


@dataclass(frozen=True)
class Node:
    id: str
    x: float
    y: float


@dataclass(frozen=True)
class Edge:
    id: str
    from_node: str
    to_node: str
    width: float
    polyline: tuple[Point, ...]
    successors: tuple[str, ...]
    length: float
    heading: float
    end_heading: float


@dataclass(frozen=True)
class ParkedObstacle:
    id: str
    x: float
    y: float
    yaw: float
    half_length: float
    half_width: float


@dataclass(frozen=True)
class LaneGraph:
    map_frame: str
    minimum_turning_radius: float
    waypoint_spacing: float
    max_turn_deg: float
    start_heading_limit_deg: float
    snap_max_distance: float
    turn_penalty: float
    lateral_offsets: tuple[float, ...]
    nodes: Mapping[str, Node]
    edges: Mapping[str, Edge]
    parked_obstacles: tuple[ParkedObstacle, ...]


@dataclass(frozen=True)
class Snap:
    edge_id: str
    x: float
    y: float
    lateral_distance: float
    along: float
    heading: float

    @property
    def distance(self) -> float:
        """Compatibility-friendly name for the lateral projection distance."""
        return self.lateral_distance


@dataclass(frozen=True)
class PathPoint:
    """A ROS-independent path sample in the map frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class OccupancyGrid:
    """Immutable map-server-style raster; unknown and out-of-map are unsafe."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    pixels: bytes
    negate: bool
    occupied_threshold: float
    free_threshold: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("occupancy resolution must be positive and finite")
        if self.width <= 0 or self.height <= 0 or len(self.pixels) != self.width * self.height:
            raise ValueError("occupancy raster dimensions do not match pixel data")
        if not 0.0 <= self.free_threshold < self.occupied_threshold <= 1.0:
            raise ValueError("occupancy thresholds are invalid")

    def is_occupied(self, x: float, y: float) -> bool:
        if not math.isfinite(x) or not math.isfinite(y):
            return True
        column = math.floor((x - self.origin_x) / self.resolution)
        row_from_bottom = math.floor((y - self.origin_y) / self.resolution)
        if column < 0 or column >= self.width or row_from_bottom < 0 or row_from_bottom >= self.height:
            return True
        row = self.height - 1 - row_from_bottom
        pixel = self.pixels[row * self.width + column]
        occupancy = pixel / 255.0 if self.negate else (255 - pixel) / 255.0
        # Trinary cells between the two thresholds are unknown.  Treating them
        # as occupied is the required fail-closed behavior.
        return occupancy > self.free_threshold


@dataclass(frozen=True)
class ClearanceResult:
    path: tuple[PathPoint, ...]
    offset: float
    minimum_clearance: float


@dataclass(frozen=True)
class AuditResult:
    valid: bool
    reason: str | None = None
    detail: str = ""


def _finite_float(value: object, field: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field} must be a finite number") from error
    if not math.isfinite(number):
        raise ValueError(f"{field} must be a finite number")
    return number


def _pgm_token(data: bytes, index: int) -> tuple[bytes, int]:
    while index < len(data):
        if data[index] in b" \t\r\n":
            index += 1
            continue
        if data[index] == ord("#"):
            newline = data.find(b"\n", index)
            if newline < 0:
                raise ValueError("unterminated PGM comment")
            index = newline + 1
            continue
        break
    start = index
    while index < len(data) and data[index] not in b" \t\r\n#":
        index += 1
    if start == index:
        raise ValueError("missing PGM token")
    return data[start:index], index


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    magic, index = _pgm_token(data, 0)
    width_token, index = _pgm_token(data, index)
    height_token, index = _pgm_token(data, index)
    max_value_token, index = _pgm_token(data, index)
    try:
        width = int(width_token)
        height = int(height_token)
        max_value = int(max_value_token)
    except ValueError as error:
        raise ValueError("invalid PGM header") from error
    if width <= 0 or height <= 0 or max_value != 255:
        raise ValueError("PGM must have positive dimensions and max value 255")
    if magic == b"P5":
        if index >= len(data) or data[index] not in b" \t\r\n":
            raise ValueError("PGM header is missing its raster separator")
        if data[index:index + 2] == b"\r\n":
            index += 2
        else:
            index += 1
        pixels = data[index:]
    elif magic == b"P2":
        values: list[int] = []
        while len(values) < width * height:
            token, index = _pgm_token(data, index)
            try:
                value = int(token)
            except ValueError as error:
                raise ValueError("invalid PGM pixel") from error
            if not 0 <= value <= 255:
                raise ValueError("PGM pixel is outside 0..255")
            values.append(value)
        pixels = bytes(values)
    else:
        raise ValueError("occupancy image must be a P5 or P2 PGM")
    if len(pixels) != width * height:
        raise ValueError("PGM raster size does not match its dimensions")
    return width, height, pixels


@lru_cache(maxsize=None)
def _load_occupancy_map_cached(resolved_yaml_path: str) -> OccupancyGrid:
    yaml_path = Path(resolved_yaml_path)
    with yaml_path.open(encoding="utf-8") as stream:
        raw = _mapping(yaml.safe_load(stream), "occupancy metadata")
    image = raw.get("image")
    origin = raw.get("origin")
    if not isinstance(image, str) or not image:
        raise ValueError("occupancy image must be a non-empty path")
    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise ValueError("occupancy origin must contain x and y")
    resolution = _finite_float(raw.get("resolution"), "occupancy resolution")
    origin_x = _finite_float(origin[0], "occupancy origin x")
    origin_y = _finite_float(origin[1], "occupancy origin y")
    occupied_threshold = _finite_float(raw.get("occupied_thresh"), "occupied_thresh")
    free_threshold = _finite_float(raw.get("free_thresh"), "free_thresh")
    negate_raw = raw.get("negate", 0)
    if negate_raw not in (0, 1, False, True):
        raise ValueError("occupancy negate must be 0 or 1")
    image_path = Path(image)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    width, height, pixels = _read_pgm(image_path.resolve())
    return OccupancyGrid(
        resolution,
        origin_x,
        origin_y,
        width,
        height,
        pixels,
        bool(negate_raw),
        occupied_threshold,
        free_threshold,
    )


def load_occupancy_map(path: str | Path) -> OccupancyGrid:
    """Load map metadata and PGM once per resolved YAML path."""
    return _load_occupancy_map_cached(str(Path(path).resolve()))


def _mapping(value: object, field: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be a mapping")
    return value


def _records(value: object, field: str) -> Sequence[Mapping[str, object]]:
    if not isinstance(value, list):
        raise ValueError(f"{field} must be a list")
    return tuple(_mapping(item, f"{field}[{index}]") for index, item in enumerate(value))


def _wrapped_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _polyline_length_and_heading(polyline: Sequence[Point], edge_id: str) -> tuple[float, float, float]:
    if len(polyline) < 2:
        raise ValueError(f"edge {edge_id} polyline must contain at least two points")
    length = 0.0
    first_heading: float | None = None
    last_heading: float | None = None
    for start, end in zip(polyline, polyline[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        if segment_length <= 0.0:
            raise ValueError(f"edge {edge_id} polyline has a zero-length segment")
        if first_heading is None:
            first_heading = math.atan2(dy, dx)
        last_heading = math.atan2(dy, dx)
        length += segment_length
    assert first_heading is not None
    assert last_heading is not None
    return length, first_heading, last_heading


def _read_polyline(raw: Mapping[str, object], nodes: Mapping[str, Node], edge_id: str) -> tuple[Point, ...]:
    raw_polyline = raw.get("polyline")
    if raw_polyline is None:
        return (
            (nodes[str(raw["from"])].x, nodes[str(raw["from"])].y),
            (nodes[str(raw["to"])].x, nodes[str(raw["to"])].y),
        )
    if not isinstance(raw_polyline, list):
        raise ValueError(f"edge {edge_id} polyline must be a list")
    points: list[Point] = []
    for index, raw_point in enumerate(raw_polyline):
        if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 2:
            raise ValueError(f"edge {edge_id} polyline point {index} must contain x and y")
        points.append(
            (
                _finite_float(raw_point[0], f"edge {edge_id} point {index} x"),
                _finite_float(raw_point[1], f"edge {edge_id} point {index} y"),
            )
        )
    return tuple(points)


def _points_match(first: Point, second: Point) -> bool:
    return math.isclose(first[0], second[0], abs_tol=1e-6) and math.isclose(first[1], second[1], abs_tol=1e-6)


def load_graph(path: str | Path) -> LaneGraph:
    """Load a strict, fixed-world graph and reject invalid topology early."""
    with Path(path).open(encoding="utf-8") as stream:
        raw = _mapping(yaml.safe_load(stream), "lane graph")

    required_scalars = (
        "minimum_turning_radius",
        "waypoint_spacing",
        "max_turn_deg",
        "start_heading_limit_deg",
    )
    for field in required_scalars:
        if field not in raw:
            raise ValueError(f"missing {field}")
    map_frame = raw.get("map_frame")
    if not isinstance(map_frame, str) or not map_frame:
        raise ValueError("map_frame must be a non-empty string")
    minimum_turning_radius = _finite_float(raw["minimum_turning_radius"], "minimum_turning_radius")
    waypoint_spacing = _finite_float(raw["waypoint_spacing"], "waypoint_spacing")
    max_turn_deg = _finite_float(raw["max_turn_deg"], "max_turn_deg")
    start_heading_limit_deg = _finite_float(raw["start_heading_limit_deg"], "start_heading_limit_deg")
    snap_max_distance = _finite_float(raw.get("snap_max_distance", 4.2), "snap_max_distance")
    turn_penalty = _finite_float(raw.get("turn_penalty", 1.0), "turn_penalty")
    if min(minimum_turning_radius, waypoint_spacing, max_turn_deg, start_heading_limit_deg, snap_max_distance) <= 0.0:
        raise ValueError("graph dimensions and angle limits must be positive")
    if max_turn_deg > 180.0 or start_heading_limit_deg > 180.0 or turn_penalty < 0.0:
        raise ValueError("graph angle limits or turn_penalty are invalid")
    max_turn_deg = min(max_turn_deg, _HARD_MAX_TURN_DEG)

    raw_offsets = raw.get("lateral_offsets")
    if not isinstance(raw_offsets, list) or not raw_offsets:
        raise ValueError("lateral_offsets must be a non-empty list")
    lateral_offsets = tuple(_finite_float(value, "lateral offset") for value in raw_offsets)

    nodes: dict[str, Node] = {}
    for item in _records(raw.get("nodes"), "nodes"):
        node_id = item.get("id")
        if not isinstance(node_id, str) or not node_id:
            raise ValueError("node id must be a non-empty string")
        if node_id in nodes:
            raise ValueError(f"duplicate node id {node_id}")
        nodes[node_id] = Node(
            node_id,
            _finite_float(item.get("x"), f"node {node_id} x"),
            _finite_float(item.get("y"), f"node {node_id} y"),
        )

    preliminary: list[tuple[str, str, str, float, tuple[Point, ...], float, float, float, object]] = []
    edge_ids: set[str] = set()
    for item in _records(raw.get("edges"), "edges"):
        edge_id = item.get("id")
        from_node = item.get("from")
        to_node = item.get("to")
        if not isinstance(edge_id, str) or not edge_id:
            raise ValueError("edge id must be a non-empty string")
        if edge_id in edge_ids:
            raise ValueError(f"duplicate edge id {edge_id}")
        if not isinstance(from_node, str) or from_node not in nodes:
            raise ValueError(f"edge {edge_id} has unknown from node")
        if not isinstance(to_node, str) or to_node not in nodes:
            raise ValueError(f"edge {edge_id} has unknown to node")
        width = _finite_float(item.get("width"), f"edge {edge_id} width")
        if width <= 0.0:
            raise ValueError(f"edge {edge_id} width must be positive")
        polyline = _read_polyline(item, nodes, edge_id)
        if not _points_match(polyline[0], (nodes[from_node].x, nodes[from_node].y)):
            raise ValueError(f"edge {edge_id} polyline endpoint does not match from node {from_node}")
        if not _points_match(polyline[-1], (nodes[to_node].x, nodes[to_node].y)):
            raise ValueError(f"edge {edge_id} polyline endpoint does not match to node {to_node}")
        length, heading, end_heading = _polyline_length_and_heading(polyline, edge_id)
        preliminary.append((edge_id, from_node, to_node, width, polyline, length, heading, end_heading, item.get("successors")))
        edge_ids.add(edge_id)

    if not preliminary:
        raise ValueError("edges must not be empty")
    outgoing: dict[str, list[tuple[str, float]]] = {}
    for edge_id, from_node, _, _, _, _, heading, _, _ in preliminary:
        outgoing.setdefault(from_node, []).append((edge_id, heading))

    edges: dict[str, Edge] = {}
    for edge_id, from_node, to_node, width, polyline, length, heading, end_heading, raw_successors in preliminary:
        if raw_successors == "auto":
            successor_ids = tuple(
                candidate_id
                for candidate_id, candidate_heading in outgoing.get(to_node, [])
                if abs(math.degrees(_wrapped_angle(candidate_heading - end_heading))) < max_turn_deg
            )
        else:
            if not isinstance(raw_successors, list):
                raise ValueError(f"edge {edge_id} successors must be a list or 'auto'")
            if len(set(raw_successors)) != len(raw_successors):
                raise ValueError(f"edge {edge_id} has duplicate successors")
            successor_ids = tuple(raw_successors)
            for successor_id in successor_ids:
                if not isinstance(successor_id, str) or successor_id not in edge_ids:
                    raise ValueError(f"edge {edge_id} has unknown successor {successor_id}")
                successor = next(candidate for candidate in preliminary if candidate[0] == successor_id)
                if successor[1] != to_node:
                    raise ValueError(f"edge {edge_id} successor {successor_id} is not connected")
                if abs(math.degrees(_wrapped_angle(successor[6] - end_heading))) >= max_turn_deg:
                    raise ValueError(f"edge {edge_id} successor {successor_id} is a forbidden turn")
        edges[edge_id] = Edge(edge_id, from_node, to_node, width, polyline, successor_ids, length, heading, end_heading)

    obstacles: list[ParkedObstacle] = []
    obstacle_ids: set[str] = set()
    for item in _records(raw.get("parked_obstacles"), "parked_obstacles"):
        obstacle_id = item.get("id")
        if not isinstance(obstacle_id, str) or not obstacle_id:
            raise ValueError("parked obstacle id must be a non-empty string")
        if obstacle_id in obstacle_ids:
            raise ValueError(f"duplicate parked obstacle id {obstacle_id}")
        obstacle = ParkedObstacle(
            obstacle_id,
            _finite_float(item.get("x"), f"parked obstacle {obstacle_id} x"),
            _finite_float(item.get("y"), f"parked obstacle {obstacle_id} y"),
            _finite_float(item.get("yaw"), f"parked obstacle {obstacle_id} yaw"),
            _finite_float(item.get("half_length"), f"parked obstacle {obstacle_id} half_length"),
            _finite_float(item.get("half_width"), f"parked obstacle {obstacle_id} half_width"),
        )
        if obstacle.half_length <= 0.0 or obstacle.half_width <= 0.0:
            raise ValueError(f"parked obstacle {obstacle_id} dimensions must be positive")
        obstacles.append(obstacle)
        obstacle_ids.add(obstacle_id)

    return LaneGraph(
        map_frame,
        minimum_turning_radius,
        waypoint_spacing,
        max_turn_deg,
        start_heading_limit_deg,
        snap_max_distance,
        turn_penalty,
        lateral_offsets,
        nodes,
        edges,
        tuple(obstacles),
    )


def _project(edge: Edge, x: float, y: float) -> tuple[float, float, float, float, float]:
    best: tuple[float, float, float, float, float] | None = None
    traversed = 0.0
    for start, end in zip(edge.polyline, edge.polyline[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length = math.hypot(dx, dy)
        fraction = max(0.0, min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / (segment_length**2)))
        projected_x = start[0] + fraction * dx
        projected_y = start[1] + fraction * dy
        candidate = (
            math.hypot(x - projected_x, y - projected_y),
            projected_x,
            projected_y,
            traversed + fraction * segment_length,
            math.atan2(dy, dx),
        )
        if best is None or candidate[0] < best[0]:
            best = candidate
        traversed += segment_length
    assert best is not None
    return best


def _snap(graph: LaneGraph, x: float, y: float, yaw: float | None) -> Snap:
    x = _finite_float(x, "snap x")
    y = _finite_float(y, "snap y")
    if yaw is not None:
        yaw = _finite_float(yaw, "snap yaw")
    candidates: list[tuple[float, str, float, float, float, float, float]] = []
    heading_limit = math.radians(graph.start_heading_limit_deg)
    for edge in graph.edges.values():
        lateral_distance, projected_x, projected_y, along, local_heading = _project(edge, x, y)
        if lateral_distance > graph.snap_max_distance:
            continue
        heading_error = 0.0 if yaw is None else abs(_wrapped_angle(local_heading - yaw))
        if yaw is not None and heading_error > heading_limit:
            continue
        candidates.append((lateral_distance + heading_error, edge.id, lateral_distance, projected_x, projected_y, along, local_heading))
    if not candidates:
        raise SnapError("START_OUTSIDE_ROAD" if yaw is not None else "GOAL_OUTSIDE_ROAD")
    _, edge_id, lateral_distance, projected_x, projected_y, along, local_heading = min(candidates)
    return Snap(edge_id, projected_x, projected_y, lateral_distance, along, local_heading)


def snap_start(graph: LaneGraph, x: float, y: float, yaw: float) -> Snap:
    """Snap a vehicle pose only to corridors it can enter moving forward."""
    return _snap(graph, x, y, yaw)


def snap_goal(graph: LaneGraph, x: float, y: float) -> Snap:
    """Snap a goal position, deliberately ignoring any HMI goal orientation."""
    return _snap(graph, x, y, None)


def astar_route(
    graph: LaneGraph,
    start_snap: Snap,
    goal_snap: Snap,
    blocked_transitions: frozenset[tuple[str, str]] = frozenset(),
    is_cancelled: Callable[[], bool] | None = None,
) -> list[str]:
    """Return forward-only edge IDs, or raise ``RouteError`` with a stable reason.

    ``is_cancelled`` is polled before each expansion, allowing the ROS wrapper
    to stop stale action requests without importing ROS into this core module.
    """
    if start_snap.edge_id not in graph.edges or goal_snap.edge_id not in graph.edges:
        raise RouteError("NO_GRAPH_ROUTE")
    start_edge = graph.edges[start_snap.edge_id]
    goal_edge = graph.edges[goal_snap.edge_id]

    def heuristic(edge_id: str) -> float:
        end = graph.nodes[graph.edges[edge_id].to_node]
        return math.hypot(end.x - goal_snap.x, end.y - goal_snap.y)

    start_state = (start_edge.id, False)
    frontier: list[tuple[float, int, str, bool]] = [(heuristic(start_edge.id), 0, start_edge.id, False)]
    costs = {start_state: 0.0}
    parents: dict[tuple[str, bool], tuple[str, bool] | None] = {start_state: None}
    sequence = 1
    while frontier:
        if is_cancelled is not None and is_cancelled():
            raise RouteError("CANCELED")
        _, _, current_id, has_departed_start = heapq.heappop(frontier)
        current_state = (current_id, has_departed_start)
        current = graph.edges[current_id]
        if current_id == goal_edge.id and (
            current_id != start_edge.id or start_snap.along <= goal_snap.along or has_departed_start
        ):
            route: list[str] = []
            while current_state is not None:
                route.append(current_state[0])
                current_state = parents[current_state]
            return list(reversed(route))
        for successor_id in current.successors:
            if (current.id, successor_id) in blocked_transitions:
                continue
            successor = graph.edges[successor_id]
            turn = abs(_wrapped_angle(successor.heading - current.end_heading))
            successor_state = (successor_id, True)
            candidate_cost = costs[current_state] + successor.length + (graph.turn_penalty if turn > 1e-9 else 0.0)
            if candidate_cost >= costs.get(successor_state, math.inf):
                continue
            costs[successor_state] = candidate_cost
            parents[successor_state] = current_state
            heapq.heappush(frontier, (candidate_cost + heuristic(successor_id), sequence, successor_id, True))
            sequence += 1
    raise RouteError("NO_GRAPH_ROUTE")


def _point_at_along(edge: Edge, along: float) -> Point:
    remaining = max(0.0, min(edge.length, along))
    for start, end in zip(edge.polyline, edge.polyline[1:]):
        segment_length = math.hypot(end[0] - start[0], end[1] - start[1])
        if remaining <= segment_length:
            fraction = remaining / segment_length
            return (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            )
        remaining -= segment_length
    return edge.polyline[-1]


def _heading_at_along(edge: Edge, along: float) -> float:
    remaining = max(0.0, min(edge.length, along))
    for start, end in zip(edge.polyline, edge.polyline[1:]):
        segment_length = math.hypot(end[0] - start[0], end[1] - start[1])
        if remaining <= segment_length + 1e-12:
            return math.atan2(end[1] - start[1], end[0] - start[0])
        remaining -= segment_length
    return edge.end_heading


def _slice_edge(edge: Edge, start_along: float, end_along: float) -> list[Point]:
    if end_along < start_along - 1e-9:
        raise RouteError("NO_VALID_CORNER")
    start_along = max(0.0, min(edge.length, start_along))
    end_along = max(0.0, min(edge.length, end_along))
    result = [_point_at_along(edge, start_along)]
    traversed = 0.0
    for point, following in zip(edge.polyline, edge.polyline[1:]):
        traversed += math.hypot(following[0] - point[0], following[1] - point[1])
        if start_along + 1e-9 < traversed < end_along - 1e-9:
            result.append(following)
    end = _point_at_along(edge, end_along)
    if not _points_match(result[-1], end):
        result.append(end)
    return result


def _unit_vector(start: Point, end: Point) -> Point:
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    if length <= 1e-9:
        raise RouteError("NO_VALID_CORNER")
    return ((end[0] - start[0]) / length, (end[1] - start[1]) / length)


def _route_vertices(
    graph: LaneGraph,
    route_edge_ids: Sequence[str],
    start_snap: Snap,
    goal_snap: Snap,
) -> list[Point]:
    if not route_edge_ids or route_edge_ids[0] != start_snap.edge_id or route_edge_ids[-1] != goal_snap.edge_id:
        raise RouteError("NO_VALID_CORNER")
    if any(edge_id not in graph.edges for edge_id in route_edge_ids):
        raise RouteError("NO_VALID_CORNER")
    vertices: list[Point] = []
    last_index = len(route_edge_ids) - 1
    for index, edge_id in enumerate(route_edge_ids):
        edge = graph.edges[edge_id]
        if index and graph.edges[route_edge_ids[index - 1]].to_node != edge.from_node:
            raise RouteError("NO_VALID_CORNER")
        start_along = start_snap.along if index == 0 else 0.0
        end_along = goal_snap.along if index == last_index else edge.length
        for point in _slice_edge(edge, start_along, end_along):
            if not vertices or not _points_match(vertices[-1], point):
                vertices.append(point)
    if len(vertices) < 2:
        raise RouteError("NO_VALID_CORNER")

    simplified = [vertices[0]]
    for point in vertices[1:]:
        if len(simplified) < 2:
            simplified.append(point)
            continue
        incoming = _unit_vector(simplified[-2], simplified[-1])
        outgoing = _unit_vector(simplified[-1], point)
        cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
        dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
        if abs(cross) <= 1e-9 and dot > 0.0:
            simplified[-1] = point
        else:
            simplified.append(point)
    return simplified


def _filleted_polyline(vertices: Sequence[Point], radius: float, dense_spacing: float) -> list[Point]:
    corner_data: list[tuple[Point, Point, Point, float, float] | None] = [None] * len(vertices)
    for index in range(1, len(vertices) - 1):
        incoming = _unit_vector(vertices[index - 1], vertices[index])
        outgoing = _unit_vector(vertices[index], vertices[index + 1])
        turn = _wrapped_angle(math.atan2(outgoing[1], outgoing[0]) - math.atan2(incoming[1], incoming[0]))
        if abs(math.degrees(turn)) >= _HARD_MAX_TURN_DEG - 1e-9:
            raise RouteError("NO_VALID_CORNER")
        if abs(turn) <= 1e-9:
            continue
        tangent_distance = radius * math.tan(abs(turn) / 2.0)
        tangent_in = (
            vertices[index][0] - incoming[0] * tangent_distance,
            vertices[index][1] - incoming[1] * tangent_distance,
        )
        tangent_out = (
            vertices[index][0] + outgoing[0] * tangent_distance,
            vertices[index][1] + outgoing[1] * tangent_distance,
        )
        sign = 1.0 if turn > 0.0 else -1.0
        center = (
            tangent_in[0] - incoming[1] * sign * radius,
            tangent_in[1] + incoming[0] * sign * radius,
        )
        corner_data[index] = (tangent_in, tangent_out, center, turn, tangent_distance)

    for index in range(len(vertices) - 1):
        consumed_at_start = corner_data[index][4] if corner_data[index] is not None else 0.0
        consumed_at_end = corner_data[index + 1][4] if corner_data[index + 1] is not None else 0.0
        segment_length = math.hypot(
            vertices[index + 1][0] - vertices[index][0],
            vertices[index + 1][1] - vertices[index][1],
        )
        if consumed_at_start + consumed_at_end > segment_length + 1e-7:
            raise RouteError("NO_VALID_CORNER")

    dense: list[Point] = [vertices[0]]

    def append_line(end: Point) -> None:
        start = dense[-1]
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if length <= 1e-9:
            return
        steps = max(1, math.ceil(length / dense_spacing))
        for step in range(1, steps + 1):
            fraction = step / steps
            dense.append((start[0] + fraction * (end[0] - start[0]), start[1] + fraction * (end[1] - start[1])))

    for index in range(1, len(vertices) - 1):
        corner = corner_data[index]
        if corner is None:
            append_line(vertices[index])
            continue
        tangent_in, tangent_out, center, turn, _ = corner
        append_line(tangent_in)
        start_angle = math.atan2(tangent_in[1] - center[1], tangent_in[0] - center[0])
        steps = max(2, math.ceil(radius * abs(turn) / dense_spacing))
        for step in range(1, steps + 1):
            angle = start_angle + turn * step / steps
            dense.append((center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle)))
        dense[-1] = tangent_out
    append_line(vertices[-1])
    return dense


def _resample_path(points: Sequence[Point], spacing: float) -> list[PathPoint]:
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.hypot(second[0] - first[0], second[1] - first[1]))
    total_length = cumulative[-1]
    if total_length <= 1e-9:
        raise RouteError("NO_VALID_CORNER")
    sample_distances = [index * spacing for index in range(math.floor(total_length / spacing) + 1)]
    if total_length - sample_distances[-1] > 1e-8:
        sample_distances.append(total_length)
    else:
        sample_distances[-1] = total_length

    sampled: list[Point] = []
    segment_index = 0
    for distance in sample_distances:
        while segment_index + 1 < len(cumulative) - 1 and cumulative[segment_index + 1] < distance - 1e-12:
            segment_index += 1
        segment_length = cumulative[segment_index + 1] - cumulative[segment_index]
        fraction = 0.0 if segment_length <= 1e-12 else (distance - cumulative[segment_index]) / segment_length
        first = points[segment_index]
        second = points[segment_index + 1]
        sampled.append((first[0] + fraction * (second[0] - first[0]), first[1] + fraction * (second[1] - first[1])))

    result: list[PathPoint] = []
    for index, point in enumerate(sampled):
        if index == 0:
            tangent_start, tangent_end = sampled[0], sampled[1]
        elif index == len(sampled) - 1:
            tangent_start, tangent_end = sampled[-2], sampled[-1]
        else:
            tangent_start, tangent_end = sampled[index - 1], sampled[index + 1]
        result.append(PathPoint(point[0], point[1], math.atan2(tangent_end[1] - tangent_start[1], tangent_end[0] - tangent_start[0])))
    return result


def build_path_geometry(
    graph: LaneGraph,
    route_edge_ids: Sequence[str],
    start_snap: Snap,
    goal_snap: Snap,
) -> list[PathPoint]:
    """Trim a graph route and return tangent, forward-only path samples.

    The base fillet reserves the largest configured lateral offset so an inner
    offset candidate can still satisfy the graph's minimum turning radius.
    """
    vertices = _route_vertices(graph, route_edge_ids, start_snap, goal_snap)
    offset_reserve = max((abs(offset) for offset in graph.lateral_offsets), default=0.0)
    fillet_radius = graph.minimum_turning_radius + offset_reserve
    dense = _filleted_polyline(vertices, fillet_radius, min(graph.waypoint_spacing / 5.0, 0.05))
    path = _resample_path(dense, graph.waypoint_spacing)
    if abs(_wrapped_angle(path[0].yaw - start_snap.heading)) > math.radians(1.0):
        raise RouteError("NO_VALID_CORNER")
    if abs(_wrapped_angle(path[-1].yaw - goal_snap.heading)) > math.radians(1.0):
        raise RouteError("NO_VALID_CORNER")
    return path


_PRIUS_HALF_LENGTH = 2.4
_PRIUS_HALF_WIDTH = 0.9
_CURVATURE_TOLERANCE = 0.01


def _path_length(path: Sequence[PathPoint]) -> float:
    return sum(math.hypot(second.x - first.x, second.y - first.y) for first, second in zip(path, path[1:]))


def _offset_path(path: Sequence[PathPoint], offset: float, spacing: float) -> list[PathPoint]:
    if abs(offset) <= 1e-12:
        return list(path)
    cumulative = [0.0]
    for first, second in zip(path, path[1:]):
        cumulative.append(cumulative[-1] + math.hypot(second.x - first.x, second.y - first.y))
    total = cumulative[-1]
    transition = min(10.0, total / 2.0)
    shifted: list[Point] = []
    for point, distance in zip(path, cumulative):
        if transition <= 1e-9:
            factor = 0.0
        elif distance < transition:
            factor = 0.5 - 0.5 * math.cos(math.pi * distance / transition)
        elif distance > total - transition:
            factor = 0.5 - 0.5 * math.cos(math.pi * (total - distance) / transition)
        else:
            factor = 1.0
        local_offset = offset * factor
        shifted.append(
            (
                point.x - math.sin(point.yaw) * local_offset,
                point.y + math.cos(point.yaw) * local_offset,
            )
        )
    return _resample_path(shifted, spacing)


def _rectangle_corners(x: float, y: float, yaw: float, half_length: float, half_width: float) -> tuple[Point, ...]:
    forward = (math.cos(yaw), math.sin(yaw))
    left = (-forward[1], forward[0])
    return tuple(
        (
            x + forward[0] * longitudinal + left[0] * lateral,
            y + forward[1] * longitudinal + left[1] * lateral,
        )
        for longitudinal, lateral in (
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width),
        )
    )


def _project_polygon(polygon: Sequence[Point], axis: Point) -> tuple[float, float]:
    projections = [point[0] * axis[0] + point[1] * axis[1] for point in polygon]
    return min(projections), max(projections)


def _polygons_intersect(first: Sequence[Point], second: Sequence[Point]) -> bool:
    for polygon in (first, second):
        for start, end in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (-(end[1] - start[1]), end[0] - start[0])
            axis_length = math.hypot(axis[0], axis[1])
            axis = (axis[0] / axis_length, axis[1] / axis_length)
            first_min, first_max = _project_polygon(first, axis)
            second_min, second_max = _project_polygon(second, axis)
            if first_max < second_min - 1e-9 or second_max < first_min - 1e-9:
                return False
    return True


def _point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared))
    return math.hypot(point[0] - start[0] - fraction * dx, point[1] - start[1] - fraction * dy)


def _polygon_distance(first: Sequence[Point], second: Sequence[Point]) -> float:
    if _polygons_intersect(first, second):
        return 0.0
    distances: list[float] = []
    for point in first:
        distances.extend(_point_segment_distance(point, start, end) for start, end in zip(second, second[1:] + second[:1]))
    for point in second:
        distances.extend(_point_segment_distance(point, start, end) for start, end in zip(first, first[1:] + first[:1]))
    return min(distances)


def _footprint_hits_occupancy(
    occupancy: OccupancyGrid,
    point: PathPoint,
    half_length: float,
    half_width: float,
) -> bool:
    return (
        _footprint_occupancy_clearance(
            occupancy,
            point,
            half_length,
            half_width,
            occupancy.resolution,
        )
        <= 1e-9
    )


def _footprint_occupancy_clearance(
    occupancy: OccupancyGrid,
    point: PathPoint,
    half_length: float,
    half_width: float,
    upper_bound: float,
) -> float:
    """Exact rectangle-to-cell clearance, capped by an existing safer bound."""
    corners = _rectangle_corners(point.x, point.y, point.yaw, half_length, half_width)
    min_x = min(corner[0] for corner in corners)
    max_x = max(corner[0] for corner in corners)
    min_y = min(corner[1] for corner in corners)
    max_y = max(corner[1] for corner in corners)
    if (
        min_x < occupancy.origin_x
        or max_x >= occupancy.origin_x + occupancy.width * occupancy.resolution
        or min_y < occupancy.origin_y
        or max_y >= occupancy.origin_y + occupancy.height * occupancy.resolution
    ):
        return 0.0
    search_distance = max(0.0, upper_bound)
    first_column = max(0, math.floor((min_x - search_distance - occupancy.origin_x) / occupancy.resolution))
    last_column = min(occupancy.width - 1, math.floor((max_x + search_distance - occupancy.origin_x) / occupancy.resolution))
    first_row = max(0, math.floor((min_y - search_distance - occupancy.origin_y) / occupancy.resolution))
    last_row = min(occupancy.height - 1, math.floor((max_y + search_distance - occupancy.origin_y) / occupancy.resolution))
    minimum = search_distance
    for row_from_bottom in range(first_row, last_row + 1):
        for column in range(first_column, last_column + 1):
            pixel_row = occupancy.height - 1 - row_from_bottom
            pixel = occupancy.pixels[pixel_row * occupancy.width + column]
            probability = pixel / 255.0 if occupancy.negate else (255 - pixel) / 255.0
            if probability <= occupancy.free_threshold:
                continue
            cell_min_x = occupancy.origin_x + column * occupancy.resolution
            cell_min_y = occupancy.origin_y + row_from_bottom * occupancy.resolution
            cell_max_x = cell_min_x + occupancy.resolution
            cell_max_y = cell_min_y + occupancy.resolution
            cell = (
                (cell_min_x, cell_min_y),
                (cell_max_x, cell_min_y),
                (cell_max_x, cell_max_y),
                (cell_min_x, cell_max_y),
            )
            minimum = min(minimum, _polygon_distance(corners, cell))
            if minimum <= 1e-9:
                return 0.0
    return minimum


def _distance_to_edge(point: Point, edge: Edge) -> float:
    return min(_point_segment_distance(point, start, end) for start, end in zip(edge.polyline, edge.polyline[1:]))


def _road_clearance(graph: LaneGraph, route_edge_ids: Sequence[str], footprint: Sequence[Point]) -> float:
    edges = [graph.edges[edge_id] for edge_id in route_edge_ids]
    longitudinal_length = math.hypot(footprint[3][0] - footprint[0][0], footprint[3][1] - footprint[0][1])
    lateral_length = math.hypot(footprint[1][0] - footprint[0][0], footprint[1][1] - footprint[0][1])
    longitudinal_steps = max(1, math.ceil(longitudinal_length / 0.25))
    lateral_steps = max(1, math.ceil(lateral_length / 0.25))
    minimum = math.inf
    for longitudinal_index in range(longitudinal_steps + 1):
        longitudinal_fraction = longitudinal_index / longitudinal_steps
        left = (
            footprint[0][0] + longitudinal_fraction * (footprint[3][0] - footprint[0][0]),
            footprint[0][1] + longitudinal_fraction * (footprint[3][1] - footprint[0][1]),
        )
        right = (
            footprint[1][0] + longitudinal_fraction * (footprint[2][0] - footprint[1][0]),
            footprint[1][1] + longitudinal_fraction * (footprint[2][1] - footprint[1][1]),
        )
        for lateral_index in range(lateral_steps + 1):
            lateral_fraction = lateral_index / lateral_steps
            sample = (
                left[0] + lateral_fraction * (right[0] - left[0]),
                left[1] + lateral_fraction * (right[1] - left[1]),
            )
            clearance = max(edge.width / 2.0 - _distance_to_edge(sample, edge) for edge in edges)
            minimum = min(minimum, clearance)
    # Clearance to a corridor union is 1-Lipschitz.  Every footprint point is
    # within half a lattice-cell diagonal of a sample, so subtracting that
    # covering radius turns the sampled value into a conservative lower bound.
    covering_radius = 0.5 * math.hypot(
        longitudinal_length / longitudinal_steps,
        lateral_length / lateral_steps,
    )
    return minimum - covering_radius


def _measured_curvature(first: PathPoint, middle: PathPoint, last: PathPoint) -> float:
    a = math.hypot(middle.x - first.x, middle.y - first.y)
    b = math.hypot(last.x - middle.x, last.y - middle.y)
    c = math.hypot(last.x - first.x, last.y - first.y)
    if min(a, b, c) <= 1e-9:
        return math.inf
    twice_area = abs((middle.x - first.x) * (last.y - first.y) - (middle.y - first.y) * (last.x - first.x))
    return 2.0 * twice_area / (a * b * c)


def _candidate_clearance(
    graph: LaneGraph,
    route_edge_ids: Sequence[str],
    path: Sequence[PathPoint],
    occupancy: OccupancyGrid,
    half_length: float,
    half_width: float,
) -> float | None:
    curvature_limit = 1.0 / graph.minimum_turning_radius + _CURVATURE_TOLERANCE
    if any(_measured_curvature(*points) > curvature_limit for points in zip(path, path[1:], path[2:])):
        return None
    obstacle_polygons = [
        _rectangle_corners(obstacle.x, obstacle.y, obstacle.yaw, obstacle.half_length, obstacle.half_width)
        for obstacle in graph.parked_obstacles
    ]
    minimum_clearance = math.inf
    for point in path:
        footprint = _rectangle_corners(point.x, point.y, point.yaw, half_length, half_width)
        road_clearance = _road_clearance(graph, route_edge_ids, footprint)
        if road_clearance < -1e-7:
            return None
        minimum_clearance = min(minimum_clearance, max(0.0, road_clearance))
        for obstacle_polygon in obstacle_polygons:
            distance = _polygon_distance(footprint, obstacle_polygon)
            if distance <= 1e-7:
                return None
            minimum_clearance = min(minimum_clearance, distance)
        occupancy_clearance = _footprint_occupancy_clearance(
            occupancy,
            point,
            half_length,
            half_width,
            minimum_clearance,
        )
        if occupancy_clearance <= 1e-7:
            return None
        minimum_clearance = min(minimum_clearance, occupancy_clearance)
    return minimum_clearance


def choose_clear_offset(
    graph: LaneGraph,
    route_edge_ids: Sequence[str],
    start_snap: Snap,
    goal_snap: Snap,
    occupancy: OccupancyGrid,
    *,
    vehicle_half_length: float = _PRIUS_HALF_LENGTH,
    vehicle_half_width: float = _PRIUS_HALF_WIDTH,
) -> ClearanceResult:
    """Select the valid smooth lateral candidate with greatest clearance."""
    if not isinstance(occupancy, OccupancyGrid):
        raise RouteError("NO_CLEAR_CORRIDOR")
    base_path = build_path_geometry(graph, route_edge_ids, start_snap, goal_snap)
    candidates: list[tuple[float, float, float, float, tuple[PathPoint, ...]]] = []
    for offset in graph.lateral_offsets:
        path = tuple(_offset_path(base_path, offset, graph.waypoint_spacing))
        clearance = _candidate_clearance(
            graph,
            route_edge_ids,
            path,
            occupancy,
            vehicle_half_length,
            vehicle_half_width,
        )
        if clearance is None:
            continue
        length = _path_length(path)
        candidates.append((clearance, -length, -abs(offset), offset, path))
    if not candidates:
        raise RouteError("NO_CLEAR_CORRIDOR")
    clearance, _, _, offset, path = max(candidates, key=lambda candidate: candidate[:3])
    return ClearanceResult(path, offset, clearance)


def _audit_failure(detail: str) -> AuditResult:
    return AuditResult(False, "FINAL_PATH_AUDIT_FAILED", detail)


def _path_follows_ordered_route(
    graph: LaneGraph,
    route_edge_ids: Sequence[str],
    path: Sequence[PathPoint],
    start_snap: Snap,
    goal_snap: Snap,
) -> bool:
    """Associate samples monotonically with the route, without skipping edges."""
    progress_tolerance = 1e-6
    association_tolerance = 1e-6
    states: dict[int, float] = {0: start_snap.along}
    for point in path:
        following_states: dict[int, float] = {}
        for route_index, previous_along in states.items():
            for candidate_index in (route_index, route_index + 1):
                if candidate_index >= len(route_edge_ids):
                    continue
                edge = graph.edges[route_edge_ids[candidate_index]]
                current_edge = graph.edges[route_edge_ids[route_index]]
                lateral_distance, _, _, along, _ = _project(edge, point.x, point.y)
                if (
                    lateral_distance > edge.width / 2.0 + association_tolerance
                    or candidate_index == route_index and along < previous_along - progress_tolerance
                    or (
                        candidate_index == route_index + 1
                        and previous_along < current_edge.length - current_edge.width / 2.0 - association_tolerance
                    )
                ):
                    continue
                following_states[candidate_index] = max(
                    following_states.get(candidate_index, -math.inf), along
                )
        if not following_states:
            return False
        states = following_states
    final_along = states.get(len(route_edge_ids) - 1)
    return final_along is not None and abs(final_along - goal_snap.along) <= progress_tolerance


def audit_path(
    graph: LaneGraph,
    route_edge_ids: Sequence[str],
    path: Sequence[PathPoint],
    start_snap: Snap,
    goal_snap: Snap,
    occupancy: OccupancyGrid,
    *,
    vehicle_half_length: float = _PRIUS_HALF_LENGTH,
    vehicle_half_width: float = _PRIUS_HALF_WIDTH,
) -> AuditResult:
    """Run the independent final safety gate; never approve a partial path."""
    if (
        len(path) < 2
        or not route_edge_ids
        or any(edge_id not in graph.edges for edge_id in route_edge_ids)
        or not isinstance(occupancy, OccupancyGrid)
    ):
        return _audit_failure("INVALID_INPUT")
    if not all(
        math.isfinite(value)
        for value in (
            start_snap.x,
            start_snap.y,
            start_snap.along,
            start_snap.heading,
            goal_snap.x,
            goal_snap.y,
            goal_snap.along,
            goal_snap.heading,
        )
    ):
        return _audit_failure("NON_FINITE")
    if any(not all(math.isfinite(value) for value in (point.x, point.y, point.yaw)) for point in path):
        return _audit_failure("NON_FINITE")
    if route_edge_ids[0] != start_snap.edge_id or route_edge_ids[-1] != goal_snap.edge_id:
        return _audit_failure("ROUTE_TRANSITION")
    start_edge = graph.edges[start_snap.edge_id]
    goal_edge = graph.edges[goal_snap.edge_id]
    if (
        start_snap.along < -1e-9
        or start_snap.along > start_edge.length + 1e-9
        or goal_snap.along < -1e-9
        or goal_snap.along > goal_edge.length + 1e-9
    ):
        return _audit_failure("INVALID_INPUT")
    expected_start_position = _point_at_along(start_edge, start_snap.along)
    expected_start_heading = _heading_at_along(start_edge, start_snap.along)
    expected_goal_position = _point_at_along(goal_edge, goal_snap.along)
    expected_goal_heading = _heading_at_along(goal_edge, goal_snap.along)
    if math.hypot(start_snap.x - expected_start_position[0], start_snap.y - expected_start_position[1]) > 1e-6:
        return _audit_failure("START_POSITION")
    if math.hypot(goal_snap.x - expected_goal_position[0], goal_snap.y - expected_goal_position[1]) > 1e-6:
        return _audit_failure("TERMINAL_POSITION")
    for current_id, next_id in zip(route_edge_ids, route_edge_ids[1:]):
        current = graph.edges[current_id]
        following = graph.edges[next_id]
        turn = abs(_wrapped_angle(following.heading - current.end_heading))
        if (
            current.to_node != following.from_node
            or next_id not in current.successors
            or turn >= math.radians(min(graph.max_turn_deg, _HARD_MAX_TURN_DEG)) - 1e-9
        ):
            return _audit_failure("ROUTE_TRANSITION")

    maximum_step = graph.waypoint_spacing * 1.5 + 1e-6
    endpoint_position_tolerance = max(0.05, graph.waypoint_spacing * 0.2)
    if math.hypot(path[0].x - expected_start_position[0], path[0].y - expected_start_position[1]) > endpoint_position_tolerance:
        return _audit_failure("START_POSITION")
    for first, second in zip(path, path[1:]):
        dx = second.x - first.x
        dy = second.y - first.y
        distance = math.hypot(dx, dy)
        if distance <= 1e-9 or distance > maximum_step:
            return _audit_failure("WAYPOINT_SPACING")
        if dx * math.cos(first.yaw) + dy * math.sin(first.yaw) < -1e-7:
            return _audit_failure("REVERSE_PROJECTION")
        heading_change = abs(_wrapped_angle(second.yaw - first.yaw))
        if heading_change >= math.radians(min(graph.max_turn_deg, _HARD_MAX_TURN_DEG)) - 1e-9:
            return _audit_failure("HEADING_CHANGE")

    curvature_limit = 1.0 / graph.minimum_turning_radius + _CURVATURE_TOLERANCE
    if any(_measured_curvature(*points) > curvature_limit for points in zip(path, path[1:], path[2:])):
        return _audit_failure("CURVATURE")
    if not _path_follows_ordered_route(graph, route_edge_ids, path, start_snap, goal_snap):
        return _audit_failure("ROUTE_ORDER")

    obstacle_polygons = [
        _rectangle_corners(obstacle.x, obstacle.y, obstacle.yaw, obstacle.half_length, obstacle.half_width)
        for obstacle in graph.parked_obstacles
    ]
    for point in path:
        footprint = _rectangle_corners(point.x, point.y, point.yaw, vehicle_half_length, vehicle_half_width)
        if _road_clearance(graph, route_edge_ids, footprint) < -1e-7:
            return _audit_failure("ROAD_BOUNDARY")
        if _footprint_hits_occupancy(occupancy, point, vehicle_half_length, vehicle_half_width):
            return _audit_failure("COLLISION")
        if any(_polygons_intersect(footprint, obstacle_polygon) for obstacle_polygon in obstacle_polygons):
            return _audit_failure("COLLISION")

    terminal = path[-1]
    if math.hypot(terminal.x - expected_goal_position[0], terminal.y - expected_goal_position[1]) > endpoint_position_tolerance:
        return _audit_failure("TERMINAL_POSITION")
    if abs(_wrapped_angle(terminal.yaw - expected_goal_heading)) > math.radians(1.0):
        return _audit_failure("TERMINAL_TANGENT")
    if abs(_wrapped_angle(path[0].yaw - expected_start_heading)) > math.radians(1.0):
        return _audit_failure("START_TANGENT")
    for index, point in enumerate(path):
        if index == 0:
            tangent_start, tangent_end = path[0], path[1]
        elif index == len(path) - 1:
            tangent_start, tangent_end = path[-2], path[-1]
        else:
            tangent_start, tangent_end = path[index - 1], path[index + 1]
        geometric_tangent = math.atan2(tangent_end.y - tangent_start.y, tangent_end.x - tangent_start.x)
        if abs(_wrapped_angle(point.yaw - geometric_tangent)) > math.radians(1.0):
            return _audit_failure("PATH_TANGENT")
    return AuditResult(True)
