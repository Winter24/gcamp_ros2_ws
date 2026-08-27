import importlib.util
import math
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "launch" / "small_city_lane_planner_core.py"
GRAPH_PATH = REPO_ROOT / "config" / "small_city_lane_graph.yaml"


def load_module():
    spec = importlib.util.spec_from_file_location("small_city_lane_planner_core", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_graph():
    module = load_module()
    return module, module.load_graph(GRAPH_PATH)


def wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def write_graph(tmp_path, *, nodes, edges, max_turn_deg=150.0):
    graph_path = tmp_path / "graph.yaml"
    graph_path.write_text(
        f"""
map_frame: map
minimum_turning_radius: 3.7
waypoint_spacing: 0.25
max_turn_deg: {max_turn_deg}
start_heading_limit_deg: 90.0
lateral_offsets: [0.0]
nodes:
{nodes}
edges:
{edges}
parked_obstacles: []
"""
    )
    return graph_path


def test_real_graph_contains_documented_road_centerlines_and_parked_cars():
    _, graph = load_graph()

    vertical = {
        round(edge.polyline[0][0], 6)
        for edge in graph.edges.values()
        if all(math.isclose(point[0], edge.polyline[0][0]) for point in edge.polyline)
    }
    horizontal = {
        round(edge.polyline[0][1], 6)
        for edge in graph.edges.values()
        if all(math.isclose(point[1], edge.polyline[0][1]) for point in edge.polyline)
    }

    assert {-45.0, -15.0, 45.0, 110.0, 120.0} <= vertical
    assert {-45.0, 0.0, 45.0} <= horizontal
    assert graph.map_frame == "map"
    assert graph.minimum_turning_radius == 3.7
    assert graph.waypoint_spacing == 0.25
    assert graph.lateral_offsets == (-0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75)
    assert len(graph.parked_obstacles) == 16
    parked_by_id = {obstacle.id: obstacle for obstacle in graph.parked_obstacles}
    assert (parked_by_id["pickup_253"].x, parked_by_id["pickup_253"].y) == pytest.approx((81.52668900386297, -2.55))
    assert (parked_by_id["suv_254"].x, parked_by_id["suv_254"].y) == pytest.approx((91.14288982830527, 2.55))


def test_real_graph_successors_exist_connect_and_never_u_turn():
    _, graph = load_graph()

    for edge in graph.edges.values():
        for successor_id in edge.successors:
            successor = graph.edges[successor_id]
            assert edge.to_node == successor.from_node
            turn_degrees = abs(math.degrees(wrapped_angle(successor.heading - edge.heading)))
            assert turn_degrees < 150.0


def test_loader_rejects_duplicate_node_ids(tmp_path):
    module = load_module()
    graph_path = write_graph(
        tmp_path,
        nodes="  - {id: a, x: 0.0, y: 0.0}\n  - {id: a, x: 1.0, y: 0.0}",
        edges="  - {id: edge, from: a, to: a, width: 7.4, polyline: [[0.0, 0.0], [1.0, 0.0]], successors: []}",
    )

    with pytest.raises(ValueError, match="duplicate node id a"):
        module.load_graph(graph_path)


def test_loader_rejects_non_finite_node_coordinates(tmp_path):
    module = load_module()
    graph_path = write_graph(
        tmp_path,
        nodes="  - {id: a, x: .nan, y: 0.0}\n  - {id: b, x: 1.0, y: 0.0}",
        edges="  - {id: edge, from: a, to: b, width: 7.4, successors: []}",
    )

    with pytest.raises(ValueError, match="node a x must be a finite number"):
        module.load_graph(graph_path)


def test_loader_rejects_nonpositive_edge_width(tmp_path):
    module = load_module()
    graph_path = write_graph(
        tmp_path,
        nodes="  - {id: a, x: 0.0, y: 0.0}\n  - {id: b, x: 1.0, y: 0.0}",
        edges="  - {id: edge, from: a, to: b, width: 0.0, successors: []}",
    )

    with pytest.raises(ValueError, match="edge edge width must be positive"):
        module.load_graph(graph_path)


def test_loader_rejects_missing_successor_id(tmp_path):
    module = load_module()
    graph_path = write_graph(
        tmp_path,
        nodes="  - {id: a, x: 0.0, y: 0.0}\n  - {id: b, x: 1.0, y: 0.0}",
        edges="  - {id: edge, from: a, to: b, width: 7.4, successors: [missing]}",
    )

    with pytest.raises(ValueError, match="edge edge has unknown successor missing"):
        module.load_graph(graph_path)


def test_loader_rejects_direct_turn_at_or_above_150_degrees_even_if_yaml_relaxes_limit(tmp_path):
    module = load_module()
    graph_path = write_graph(
        tmp_path,
        max_turn_deg=180.0,
        nodes=(
            "  - {id: a, x: 0.0, y: 0.0}\n"
            "  - {id: b, x: 10.0, y: 0.0}\n"
            "  - {id: c, x: 0.15192246987792046, y: 1.7364817766693041}"
        ),
        edges=(
            "  - {id: east, from: a, to: b, width: 7.4, successors: [backish]}\n"
            "  - {id: backish, from: b, to: c, width: 7.4, successors: []}"
        ),
    )

    with pytest.raises(ValueError, match="edge east successor backish is a forbidden turn"):
        module.load_graph(graph_path)


def test_loader_rejects_polyline_that_does_not_end_at_declared_to_node(tmp_path):
    module = load_module()
    graph_path = write_graph(
        tmp_path,
        nodes="  - {id: a, x: 0.0, y: 0.0}\n  - {id: b, x: 10.0, y: 0.0}",
        edges="  - {id: edge, from: a, to: b, width: 7.4, polyline: [[0.0, 0.0], [9.0, 0.0]], successors: []}",
    )

    with pytest.raises(ValueError, match="edge edge polyline endpoint does not match to node b"):
        module.load_graph(graph_path)


def test_snap_uses_local_selected_segment_tangent_for_heading_filter_and_result(tmp_path):
    module = load_module()
    graph_path = write_graph(
        tmp_path,
        nodes="  - {id: a, x: 0.0, y: 0.0}\n  - {id: b, x: 10.0, y: 10.0}",
        edges="  - {id: corner, from: a, to: b, width: 7.4, polyline: [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]], successors: []}",
    )
    graph = module.load_graph(graph_path)

    start = module.snap_start(graph, 10.0, 5.0, math.pi / 2.0)
    goal = module.snap_goal(graph, 10.0, 5.0)

    assert start.heading == pytest.approx(math.pi / 2.0)
    assert goal.heading == pytest.approx(math.pi / 2.0)
    with pytest.raises(module.SnapError, match="START_OUTSIDE_ROAD"):
        module.snap_start(graph, 10.0, 5.0, -math.pi / 2.0)


def test_start_snap_requires_forward_heading_and_goal_ignores_yaw():
    module, graph = load_graph()

    start = module.snap_start(graph, -45.0, -30.0, math.pi / 2.0)
    goal = module.snap_goal(graph, -45.0, 20.0)

    assert graph.edges[start.edge_id].heading == pytest.approx(math.pi / 2.0)
    assert goal.x == pytest.approx(-45.0)
    assert goal.y == pytest.approx(20.0)
    assert graph.edges[goal.edge_id].heading in {math.pi / 2.0, -math.pi / 2.0}
    with pytest.raises(module.SnapError, match="START_OUTSIDE_ROAD"):
        module.snap_start(graph, -37.0, -30.0, 0.0)


def test_astar_routes_straight_left_and_right_turns():
    module, graph = load_graph()

    straight = module.astar_route(
        graph,
        module.snap_start(graph, -45.0, -30.0, math.pi / 2.0),
        module.snap_goal(graph, -45.0, 30.0),
    )
    left = module.astar_route(
        graph,
        module.snap_start(graph, -15.0, -30.0, math.pi / 2.0),
        module.snap_goal(graph, -30.0, 0.0),
    )
    right = module.astar_route(
        graph,
        module.snap_start(graph, -15.0, -30.0, math.pi / 2.0),
        module.snap_goal(graph, 20.0, 0.0),
    )

    assert all(graph.edges[edge_id].heading == pytest.approx(math.pi / 2.0) for edge_id in straight)
    assert any(graph.edges[edge_id].heading == pytest.approx(math.pi) for edge_id in left)
    assert any(graph.edges[edge_id].heading == pytest.approx(0.0) for edge_id in right)


def test_goal_behind_vehicle_routes_around_a_block_without_reverse_transition():
    module, graph = load_graph()
    start = module.snap_start(graph, -45.0, -20.0, math.pi / 2.0)
    goal = module.snap_goal(graph, -45.0, -30.0)

    route = module.astar_route(graph, start, goal)

    assert len(route) > 2
    assert all(
        graph.edges[next_edge].id in graph.edges[current_edge].successors
        for current_edge, next_edge in zip(route, route[1:])
    )
    assert all(
        abs(math.degrees(wrapped_angle(graph.edges[next_edge].heading - graph.edges[current_edge].heading))) < 150.0
        for current_edge, next_edge in zip(route, route[1:])
    )


def test_astar_honors_cancellation_callback_before_expanding():
    module, graph = load_graph()
    start = module.snap_start(graph, -45.0, -30.0, math.pi / 2.0)
    goal = module.snap_goal(graph, 45.0, 30.0)

    with pytest.raises(module.RouteError, match="CANCELED") as error:
        module.astar_route(graph, start, goal, is_cancelled=lambda: True)

    assert error.value.reason == "CANCELED"


def test_astar_reports_no_graph_route_for_disconnected_components(tmp_path):
    module = load_module()
    graph_path = tmp_path / "disconnected.yaml"
    graph_path.write_text(
        """
map_frame: map
minimum_turning_radius: 3.7
waypoint_spacing: 0.25
max_turn_deg: 150.0
start_heading_limit_deg: 90.0
lateral_offsets: [0.0]
nodes:
  - {id: a, x: 0.0, y: 0.0}
  - {id: b, x: 10.0, y: 0.0}
  - {id: c, x: 0.0, y: 10.0}
  - {id: d, x: 10.0, y: 10.0}
edges:
  - {id: east, from: a, to: b, width: 7.4, polyline: [[0.0, 0.0], [10.0, 0.0]], successors: []}
  - {id: north, from: c, to: d, width: 7.4, polyline: [[0.0, 10.0], [10.0, 10.0]], successors: []}
parked_obstacles: []
"""
    )
    graph = module.load_graph(graph_path)

    with pytest.raises(module.RouteError, match="NO_GRAPH_ROUTE") as error:
        module.astar_route(
            graph,
            module.snap_start(graph, 1.0, 0.0, 0.0),
            module.snap_goal(graph, 1.0, 10.0),
        )

    assert error.value.reason == "NO_GRAPH_ROUTE"
