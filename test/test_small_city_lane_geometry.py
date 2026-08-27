import dataclasses
import importlib.util
import math
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "launch" / "small_city_lane_planner_core.py"
GRAPH_PATH = REPO_ROOT / "config" / "small_city_lane_graph.yaml"


def load_module():
    spec = importlib.util.spec_from_file_location("small_city_lane_planner_geometry_core", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def real_graph():
    module = load_module()
    return module, module.load_graph(GRAPH_PATH)


def curvature(first, middle, last):
    a = math.hypot(middle.x - first.x, middle.y - first.y)
    b = math.hypot(last.x - middle.x, last.y - middle.y)
    c = math.hypot(last.x - first.x, last.y - first.y)
    if min(a, b, c) < 1e-9:
        return math.inf
    twice_area = abs(
        (middle.x - first.x) * (last.y - first.y)
        - (middle.y - first.y) * (last.x - first.x)
    )
    return 2.0 * twice_area / (a * b * c)


def assert_path_geometry(path, spacing=0.25, minimum_radius=3.7):
    assert len(path) > 2
    distances = [
        math.hypot(second.x - first.x, second.y - first.y)
        for first, second in zip(path, path[1:])
    ]
    assert max(distances) <= spacing + 1e-6
    assert all(distance == pytest.approx(spacing, abs=2e-3) for distance in distances[:-1])
    assert all(
        (second.x - first.x) * math.cos(first.yaw)
        + (second.y - first.y) * math.sin(first.yaw)
        >= -1e-6
        for first, second in zip(path, path[1:])
    )
    assert max(curvature(*points) for points in zip(path, path[1:], path[2:])) <= 1.0 / minimum_radius + 0.01


def test_build_path_geometry_trims_straight_route_and_uses_tangent_yaw():
    module, graph = real_graph()
    start = module.Snap("v_m15_n_1", -15.0, -30.0, 0.0, 15.0, math.pi / 2.0)
    goal = module.Snap("v_m15_n_2", -15.0, 30.0, 0.0, 30.0, math.pi / 2.0)

    path = module.build_path_geometry(graph, [start.edge_id, goal.edge_id], start, goal)

    assert (path[0].x, path[0].y) == pytest.approx((start.x, start.y))
    assert (path[-1].x, path[-1].y) == pytest.approx((goal.x, goal.y))
    assert all(point.yaw == pytest.approx(math.pi / 2.0) for point in path)
    assert_path_geometry(path)


@pytest.mark.parametrize(
    ("outgoing_edge", "goal", "expected_turn_sign"),
    [
        ("h_0_w_0", (-30.0, 0.0, math.pi), 1.0),
        ("h_0_e_1", (20.0, 0.0, 0.0), -1.0),
    ],
)
def test_build_path_geometry_makes_continuous_left_and_right_fillets(
    outgoing_edge, goal, expected_turn_sign
):
    module, graph = real_graph()
    start = module.Snap("v_m15_n_1", -15.0, -30.0, 0.0, 15.0, math.pi / 2.0)
    goal_snap = module.Snap(
        outgoing_edge,
        goal[0],
        goal[1],
        0.0,
        math.hypot(goal[0] + 15.0, goal[1]),
        goal[2],
    )

    path = module.build_path_geometry(graph, [start.edge_id, outgoing_edge], start, goal_snap)

    assert (path[0].x, path[0].y) == pytest.approx((start.x, start.y))
    assert (path[-1].x, path[-1].y) == pytest.approx((goal_snap.x, goal_snap.y))
    signed_yaw_change = sum(
        math.atan2(math.sin(second.yaw - first.yaw), math.cos(second.yaw - first.yaw))
        for first, second in zip(path, path[1:])
    )
    assert math.copysign(1.0, signed_yaw_change) == expected_turn_sign
    assert abs(signed_yaw_change) == pytest.approx(math.pi / 2.0, abs=0.03)
    assert_path_geometry(path)


def test_build_path_geometry_fails_closed_when_corner_cannot_fit_minimum_radius():
    module = load_module()
    nodes = {
        "a": module.Node("a", 0.0, 0.0),
        "b": module.Node("b", 2.0, 0.0),
        "c": module.Node("c", 2.0, 2.0),
    }
    east = module.Edge("east", "a", "b", 7.4, ((0.0, 0.0), (2.0, 0.0)), ("north",), 2.0, 0.0, 0.0)
    north = module.Edge("north", "b", "c", 7.4, ((2.0, 0.0), (2.0, 2.0)), (), 2.0, math.pi / 2.0, math.pi / 2.0)
    graph = module.LaneGraph(
        "map", 3.7, 0.25, 150.0, 90.0, 4.2, 1.0, (0.0,), nodes, {"east": east, "north": north}, ()
    )
    start = module.Snap("east", 0.0, 0.0, 0.0, 0.0, 0.0)
    goal = module.Snap("north", 2.0, 2.0, 0.0, 2.0, math.pi / 2.0)

    with pytest.raises(module.RouteError, match="NO_VALID_CORNER") as error:
        module.build_path_geometry(graph, ["east", "north"], start, goal)

    assert error.value.reason == "NO_VALID_CORNER"


def make_straight_graph(module, offsets=(-0.75, 0.0, 0.75)):
    nodes = {"a": module.Node("a", 0.0, 0.0), "b": module.Node("b", 60.0, 0.0)}
    edge = module.Edge("east", "a", "b", 8.4, ((0.0, 0.0), (60.0, 0.0)), (), 60.0, 0.0, 0.0)
    graph = module.LaneGraph(
        "map", 3.7, 0.25, 150.0, 90.0, 4.2, 1.0, tuple(offsets), nodes, {"east": edge}, ()
    )
    start = module.Snap("east", 0.0, 0.0, 0.0, 0.0, 0.0)
    goal = module.Snap("east", 60.0, 0.0, 0.0, 60.0, 0.0)
    return graph, ["east"], start, goal


def occupancy_grid(module, occupied=(), *, resolution=0.1, width=700, height=100, origin=(-5.0, -5.0)):
    pixels = bytearray([255]) * (width * height)
    for x, y in occupied:
        column = math.floor((x - origin[0]) / resolution)
        row_from_bottom = math.floor((y - origin[1]) / resolution)
        row = height - 1 - row_from_bottom
        pixels[row * width + column] = 0
    return module.OccupancyGrid(
        resolution=resolution,
        origin_x=origin[0],
        origin_y=origin[1],
        width=width,
        height=height,
        pixels=bytes(pixels),
        negate=False,
        occupied_threshold=0.65,
        free_threshold=0.25,
    )


def test_load_occupancy_map_reads_pgm_yaml_once(tmp_path):
    module = load_module()
    pgm_path = tmp_path / "tiny.pgm"
    pgm_path.write_bytes(b"P5\n# synthetic\n2 2\n255\n" + bytes([255, 0, 127, 255]))
    yaml_path = tmp_path / "tiny.yaml"
    yaml_path.write_text(
        "image: tiny.pgm\nresolution: 0.5\norigin: [-1.0, -2.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )

    first = module.load_occupancy_map(yaml_path)
    second = module.load_occupancy_map(yaml_path)

    assert first is second
    assert not first.is_occupied(-0.75, -1.25)
    assert first.is_occupied(-0.25, -1.25)
    assert first.is_occupied(-0.75, -1.75)
    assert first.is_occupied(100.0, 100.0)


def test_choose_clear_offset_prefers_center_between_all_real_parked_cars():
    module, graph = real_graph()
    assert len(graph.parked_obstacles) == 16
    start = module.Snap("h_0_e_1", -14.5, 0.0, 0.0, 0.5, 0.0)
    goal = module.Snap("h_0_e_1", 44.5, 0.0, 0.0, 59.5, 0.0)
    occupancy = occupancy_grid(module, resolution=0.5, width=500, height=400, origin=(-100.0, -100.0))

    result = module.choose_clear_offset(graph, ["h_0_e_1"], start, goal, occupancy)

    assert result.offset == 0.0
    assert result.minimum_clearance > 0.0


def test_choose_clear_offset_rejects_blocked_candidates_and_transitions_smoothly():
    module = load_module()
    graph, route, start, goal = make_straight_graph(module)
    occupancy = occupancy_grid(module, occupied=((30.0, -0.95),))

    result = module.choose_clear_offset(graph, route, start, goal, occupancy)

    assert result.offset == 0.75
    assert (result.path[0].x, result.path[0].y) == pytest.approx((start.x, start.y))
    assert (result.path[-1].x, result.path[-1].y) == pytest.approx((goal.x, goal.y))
    lateral_steps = [abs(second.y - first.y) for first, second in zip(result.path, result.path[1:])]
    assert max(lateral_steps) < 0.03
    assert max(abs(point.y) for point in result.path) == pytest.approx(0.75, abs=0.01)
    assert max(abs(point.y) for point in result.path) + 0.9 < graph.edges["east"].width / 2.0


def test_choose_clear_offset_scores_nearby_occupied_cells_not_only_collisions():
    module = load_module()
    graph, route, start, goal = make_straight_graph(module)
    occupancy = occupancy_grid(module, occupied=((30.0, 1.5),))

    result = module.choose_clear_offset(graph, route, start, goal, occupancy)

    assert result.offset == -0.75
    assert 0.0 < result.minimum_clearance < 1.5


def test_choose_clear_offset_fails_closed_when_every_candidate_is_blocked():
    module = load_module()
    graph, route, start, goal = make_straight_graph(module)
    occupancy = occupancy_grid(module)
    occupancy = dataclasses.replace(occupancy, pixels=bytes([0]) * len(occupancy.pixels))

    with pytest.raises(module.RouteError, match="NO_CLEAR_CORRIDOR") as error:
        module.choose_clear_offset(graph, route, start, goal, occupancy)

    assert error.value.reason == "NO_CLEAR_CORRIDOR"


def audit_fixture():
    module = load_module()
    graph, route, start, goal = make_straight_graph(module, offsets=(0.0,))
    occupancy = occupancy_grid(module)
    path = module.build_path_geometry(graph, route, start, goal)
    return module, graph, route, start, goal, occupancy, path


def test_audit_path_rejects_a_valid_suffix_that_omits_the_start_anchor():
    module, graph, route, start, goal, occupancy, path = audit_fixture()

    result = module.audit_path(graph, route, path[len(path) // 2 :], start, goal, occupancy)

    assert not result.valid
    assert result.detail == "START_POSITION"


def test_audit_path_rejects_samples_that_skip_an_ordered_route_segment():
    module = load_module()
    nodes = {
        "a": module.Node("a", 0.0, 0.0),
        "b": module.Node("b", 10.0, 0.0),
        "c": module.Node("c", 10.0, 10.0),
        "d": module.Node("d", 19.75, 0.0),
    }
    east = module.Edge("east", "a", "b", 8.4, ((0.0, 0.0), (10.0, 0.0)), ("north",), 10.0, 0.0, 0.0)
    north = module.Edge("north", "b", "c", 8.4, ((10.0, 0.0), (10.0, 10.0)), ("late",), 10.0, math.pi / 2.0, math.pi / 2.0)
    late = module.Edge(
        "late",
        "c",
        "d",
        8.4,
        ((10.0, 10.0), (20.0, 10.0), (20.0, -5.0), (10.0, -5.0), (10.0, 0.0), (19.75, 0.0)),
        (),
        49.75,
        0.0,
        0.0,
    )
    graph = module.LaneGraph(
        "map", 3.7, 0.25, 150.0, 90.0, 4.2, 1.0, (0.0,), nodes,
        {"east": east, "north": north, "late": late}, ()
    )
    start = module.Snap("east", 0.0, 0.0, 0.0, 0.0, 0.0)
    goal = module.Snap("late", 19.75, 0.0, 0.0, 49.75, 0.0)
    path = [module.PathPoint(index * 0.25, 0.0, 0.0) for index in range(80)]
    occupancy = occupancy_grid(module, resolution=0.5, width=100, height=100, origin=(-20.0, -20.0))

    result = module.audit_path(graph, ["east", "north", "late"], path, start, goal, occupancy)

    assert not result.valid
    assert result.detail == "ROUTE_ORDER"


@pytest.mark.parametrize(("field", "value"), [("x", math.nan), ("yaw", math.inf)])
def test_audit_path_rejects_non_finite_samples(field, value):
    module, graph, route, start, goal, occupancy, path = audit_fixture()
    path[10] = dataclasses.replace(path[10], **{field: value})

    result = module.audit_path(graph, route, path, start, goal, occupancy)

    assert not result.valid
    assert result.reason == "FINAL_PATH_AUDIT_FAILED"
    assert result.detail == "NON_FINITE"


def test_audit_path_rejects_reverse_projection():
    module, graph, route, _, _, occupancy, _ = audit_fixture()
    path = [module.PathPoint(0.25, 0.0, 0.0), module.PathPoint(0.0, 0.0, 0.0)]
    start = module.Snap("east", 0.25, 0.0, 0.0, 0.25, 0.0)
    goal = module.Snap("east", 0.0, 0.0, 0.0, 0.0, 0.0)

    result = module.audit_path(graph, route, path, start, goal, occupancy)

    assert not result.valid
    assert result.detail == "REVERSE_PROJECTION"


def test_audit_path_rejects_local_heading_change_at_150_degrees():
    module, graph, route, start, _, occupancy, _ = audit_fixture()
    path = [
        module.PathPoint(0.0, 0.0, 0.0),
        module.PathPoint(0.25, 0.0, 0.0),
        module.PathPoint(0.50, 0.0, math.radians(150.0)),
    ]
    goal = module.Snap("east", 0.5, 0.0, 0.0, 0.5, math.radians(150.0))

    result = module.audit_path(graph, route, path, start, goal, occupancy)

    assert not result.valid
    assert result.detail == "HEADING_CHANGE"


def test_audit_path_rejects_excessive_curvature():
    module, graph, route, _, _, occupancy, _ = audit_fixture()
    path = [
        module.PathPoint(10.0, 0.0, 0.0),
        module.PathPoint(10.25, 0.0, math.pi / 4.0),
        module.PathPoint(10.25, 0.25, math.pi / 2.0),
    ]
    start = module.Snap("east", 10.0, 0.0, 0.0, 10.0, 0.0)
    goal = module.Snap("east", 10.25, 0.0, 0.0, 10.25, 0.0)

    result = module.audit_path(graph, route, path, start, goal, occupancy)

    assert not result.valid
    assert result.detail == "CURVATURE"


def test_audit_path_rejects_occupancy_collision():
    module, graph, route, start, goal, _, path = audit_fixture()
    occupancy = occupancy_grid(module, occupied=((30.0, 0.0),))

    result = module.audit_path(graph, route, path, start, goal, occupancy)

    assert not result.valid
    assert result.detail == "COLLISION"


def test_audit_path_rejects_road_boundary_violation():
    module, graph, route, start, goal, occupancy, _ = audit_fixture()
    dense = [
        (x / 4.0, 3.5 * math.sin(math.pi * (x / 4.0) / 60.0))
        for x in range(241)
    ]
    shifted = module._resample_path(dense, graph.waypoint_spacing)

    result = module.audit_path(graph, route, shifted, start, goal, occupancy)

    assert not result.valid
    assert result.detail == "ROAD_BOUNDARY"


def test_audit_path_rejects_wrong_terminal_tangent():
    module, graph, route, start, goal, occupancy, path = audit_fixture()
    path[-1] = dataclasses.replace(path[-1], yaw=0.2)

    result = module.audit_path(graph, route, path, start, goal, occupancy)

    assert not result.valid
    assert result.detail == "TERMINAL_TANGENT"


def test_audit_path_derives_terminal_tangent_instead_of_trusting_goal_snap():
    module, graph, route, start, goal, occupancy, path = audit_fixture()
    forged_yaw = math.radians(60.0)
    forged_path = [dataclasses.replace(point, yaw=forged_yaw) for point in path]
    forged_goal = dataclasses.replace(goal, heading=forged_yaw)

    result = module.audit_path(graph, route, forged_path, start, forged_goal, occupancy)

    assert not result.valid
    assert result.detail == "TERMINAL_TANGENT"


def test_road_clearance_checks_footprint_interior_against_nonconvex_corridor_union():
    module = load_module()
    nodes = {
        "a": module.Node("a", 110.0, -10.0),
        "b": module.Node("b", 110.0, 10.0),
        "c": module.Node("c", 120.0, 10.0),
        "d": module.Node("d", 120.0, -10.0),
    }
    north = module.Edge("north", "a", "b", 7.4, ((110.0, -10.0), (110.0, 10.0)), ("east",), 20.0, math.pi / 2.0, math.pi / 2.0)
    east = module.Edge("east", "b", "c", 7.4, ((110.0, 10.0), (120.0, 10.0)), ("south",), 10.0, 0.0, 0.0)
    south = module.Edge("south", "c", "d", 7.4, ((120.0, 10.0), (120.0, -10.0)), (), 20.0, -math.pi / 2.0, -math.pi / 2.0)
    graph = module.LaneGraph(
        "map", 3.7, 0.25, 150.0, 90.0, 4.2, 1.0, (0.0,), nodes,
        {"north": north, "east": east, "south": south}, ()
    )
    footprint = module._rectangle_corners(115.0, 0.0, 0.0, 2.4, 0.9)

    assert module._road_clearance(graph, ["north", "east", "south"], footprint) < 0.0


def test_road_clearance_lattice_uses_a_fail_closed_covering_bound():
    module, graph = real_graph()
    footprint = module._rectangle_corners(
        -11.6937316,
        -3.3778619,
        0.1708606,
        2.4,
        0.9,
    )

    clearance = module._road_clearance(graph, ["v_m15_n_1", "h_0_e_1"], footprint)

    assert clearance < 0.0


@pytest.mark.parametrize(
    ("route", "start", "goal"),
    [
        (
            ["v_m15_n_1", "v_m15_n_2"],
            ("v_m15_n_1", -15.0, -30.0, 15.0, math.pi / 2.0),
            ("v_m15_n_2", -15.0, 30.0, 30.0, math.pi / 2.0),
        ),
        (
            ["v_m15_n_1", "h_0_w_0"],
            ("v_m15_n_1", -15.0, -30.0, 15.0, math.pi / 2.0),
            ("h_0_w_0", -30.0, 0.0, 15.0, math.pi),
        ),
        (
            ["v_m15_n_1", "h_0_e_1"],
            ("v_m15_n_1", -15.0, -30.0, 15.0, math.pi / 2.0),
            ("h_0_e_1", 20.0, 0.0, 35.0, 0.0),
        ),
    ],
)
def test_audit_path_accepts_representative_straight_left_and_right(route, start, goal):
    module, graph = real_graph()
    graph = dataclasses.replace(graph, parked_obstacles=(), lateral_offsets=(0.0,))
    start_snap = module.Snap(start[0], start[1], start[2], 0.0, start[3], start[4])
    goal_snap = module.Snap(goal[0], goal[1], goal[2], 0.0, goal[3], goal[4])
    path = module.build_path_geometry(graph, route, start_snap, goal_snap)
    occupancy = occupancy_grid(module, resolution=0.5, width=500, height=400, origin=(-100.0, -100.0))

    result = module.audit_path(graph, route, path, start_snap, goal_snap, occupancy)

    assert result.valid, result.detail
    assert result.reason is None
