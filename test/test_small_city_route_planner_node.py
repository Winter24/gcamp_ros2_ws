import importlib.util
import math
from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "launch" / "small_city_route_planner.py"


class Header:
    def __init__(self):
        self.frame_id = ""
        self.stamp = None


class Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(),
        )


class PathMessage:
    def __init__(self):
        self.header = Header()
        self.poses = []


class ResultMessage:
    def __init__(self):
        self.path = PathMessage()
        self.planning_time = SimpleNamespace(sec=0, nanosec=0)


class ComputePathToPose:
    class Goal:
        pass

    Result = ResultMessage


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)

    warn = warning

    def error(self, message):
        self.messages.append(message)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeNode:
    parameter_values = {
        "graph_path": "/real/config/small_city_lane_graph.yaml",
        "map_path": "/real/maps/small_city.yaml",
    }

    def __init__(self, name):
        self.node_name = name
        self.logger = FakeLogger()

    def declare_parameter(self, name, default):
        return FakeParameter(self.parameter_values.get(name, default))

    def create_publisher(self, message_type, topic, depth):
        self.publisher_args = (message_type, topic, depth)
        self.publisher = FakePublisher()
        return self.publisher

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=7, nanosec=9))
        )

    def destroy_node(self):
        pass


class FakeActionServer:
    def __init__(
        self,
        node,
        action_type,
        action_name,
        execute_callback,
        *,
        goal_callback,
        cancel_callback,
        handle_accepted_callback,
        callback_group,
    ):
        self.action_type = action_type
        self.action_name = action_name
        self.execute_callback = execute_callback
        self.goal_callback = goal_callback
        self.cancel_callback = cancel_callback
        self.handle_accepted_callback = handle_accepted_callback
        self.callback_group = callback_group
        node.captured_action_server = self

    def destroy(self):
        pass


class FakeBuffer:
    def __init__(self):
        self.transform = SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=12.0, y=-4.0),
                rotation=Quaternion(),
            )
        )
        self.lookups = []

    def lookup_transform(self, target, source, when):
        self.lookups.append((target, source, when))
        return self.transform


class FakeGoalHandle:
    def __init__(self, request):
        self.request = request
        self.is_cancel_requested = False
        self.is_active = True
        self.execute_count = 0
        self.succeed_count = 0
        self.abort_count = 0
        self.canceled_count = 0

    def execute(self):
        self.execute_count += 1

    def succeed(self):
        self.succeed_count += 1
        self.is_active = False

    def abort(self):
        self.abort_count += 1
        self.is_active = False

    def canceled(self):
        self.canceled_count += 1
        self.is_active = False


def quaternion(yaw):
    value = Quaternion()
    value.z = math.sin(yaw / 2.0)
    value.w = math.cos(yaw / 2.0)
    return value


def request(*, use_start=True, start_yaw=0.4, goal_yaw=-2.2):
    start = PoseStamped()
    start.pose.position.x = -15.0
    start.pose.position.y = -30.0
    start.pose.orientation = quaternion(start_yaw)
    goal = PoseStamped()
    goal.pose.position.x = 20.0
    goal.pose.position.y = 0.0
    goal.pose.orientation = quaternion(goal_yaw)
    return SimpleNamespace(use_start=use_start, start=start, goal=goal)


def fake_core_module():
    core = ModuleType("small_city_lane_planner_core")
    core.calls = []

    class PlannerError(ValueError):
        def __init__(self, reason):
            super().__init__(reason)
            self.reason = reason

    class PathPoint:
        def __init__(self, x, y, yaw):
            self.x = x
            self.y = y
            self.yaw = yaw

    core.SnapError = PlannerError
    core.RouteError = PlannerError
    core.PathPoint = PathPoint
    core.load_graph = lambda path: (
        core.calls.append(("load_graph", path))
        or SimpleNamespace(map_frame="map")
    )
    core.load_occupancy_map = lambda path: (
        core.calls.append(("load_occupancy_map", path)) or object()
    )

    def snap_start(graph, x, y, yaw):
        core.calls.append(("snap_start", x, y, yaw))
        return SimpleNamespace(edge_id="start", x=x, y=y, heading=yaw)

    def snap_goal(graph, x, y):
        core.calls.append(("snap_goal", x, y))
        return SimpleNamespace(edge_id="goal", x=x, y=y, heading=0.0)

    def astar_route(graph, start_snap, goal_snap, *, is_cancelled):
        core.calls.append(("astar_route", start_snap, goal_snap, is_cancelled))
        if is_cancelled():
            raise PlannerError("CANCELED")
        return ["start", "goal"]

    def choose_clear_offset(graph, route, start_snap, goal_snap, occupancy):
        core.calls.append(
            ("choose_clear_offset", graph, route, start_snap, goal_snap, occupancy)
        )
        return SimpleNamespace(
            path=(PathPoint(-15.0, -30.0, math.pi / 2.0), PathPoint(20.0, 0.0, 0.0)),
            offset=0.0,
            minimum_clearance=1.25,
        )

    def audit_path(graph, route, path, start_snap, goal_snap, occupancy):
        core.calls.append(
            ("audit_path", graph, route, path, start_snap, goal_snap, occupancy)
        )
        return SimpleNamespace(valid=True, reason=None, detail="")

    core.snap_start = snap_start
    core.snap_goal = snap_goal
    core.astar_route = astar_route
    core.choose_clear_offset = choose_clear_offset
    core.audit_path = audit_path
    return core


def install_fake_modules(monkeypatch):
    rclpy = ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node, executor=None: None
    rclpy.shutdown = lambda: None
    rclpy.ok = lambda: True
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)

    rclpy_action = ModuleType("rclpy.action")
    rclpy_action.ActionServer = FakeActionServer
    rclpy_action.GoalResponse = SimpleNamespace(ACCEPT="accept")
    rclpy_action.CancelResponse = SimpleNamespace(ACCEPT="accept")
    monkeypatch.setitem(sys.modules, "rclpy.action", rclpy_action)

    callback_groups = ModuleType("rclpy.callback_groups")
    callback_groups.ReentrantCallbackGroup = type("ReentrantCallbackGroup", (), {})
    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", callback_groups)

    executors = ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = type(
        "MultiThreadedExecutor",
        (),
        {"add_node": lambda self, node: None, "spin": lambda self: None, "shutdown": lambda self: None},
    )
    monkeypatch.setitem(sys.modules, "rclpy.executors", executors)

    node_module = ModuleType("rclpy.node")
    node_module.Node = FakeNode
    monkeypatch.setitem(sys.modules, "rclpy.node", node_module)

    time_module = ModuleType("rclpy.time")
    time_module.Time = type("Time", (), {})
    monkeypatch.setitem(sys.modules, "rclpy.time", time_module)

    nav2_action = ModuleType("nav2_msgs.action")
    nav2_action.ComputePathToPose = ComputePathToPose
    monkeypatch.setitem(sys.modules, "nav2_msgs", ModuleType("nav2_msgs"))
    monkeypatch.setitem(sys.modules, "nav2_msgs.action", nav2_action)

    nav_msgs = ModuleType("nav_msgs.msg")
    nav_msgs.Path = PathMessage
    monkeypatch.setitem(sys.modules, "nav_msgs", ModuleType("nav_msgs"))
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", nav_msgs)

    geometry_msgs = ModuleType("geometry_msgs.msg")
    geometry_msgs.PoseStamped = PoseStamped
    monkeypatch.setitem(sys.modules, "geometry_msgs", ModuleType("geometry_msgs"))
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs)

    tf2_ros = ModuleType("tf2_ros")
    tf2_ros.Buffer = FakeBuffer
    tf2_ros.TransformListener = lambda buffer, node: SimpleNamespace()
    monkeypatch.setitem(sys.modules, "tf2_ros", tf2_ros)

    core = fake_core_module()
    monkeypatch.setitem(sys.modules, "small_city_lane_planner_core", core)
    return core


@pytest.fixture
def loaded(monkeypatch):
    core = install_fake_modules(monkeypatch)
    spec = importlib.util.spec_from_file_location("small_city_route_planner_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module, core


def activate(node, handle):
    node.handle_accepted_callback(handle)
    assert handle.execute_count == 1


def test_constructs_dedicated_compute_path_action_and_plan_publisher(loaded):
    module, core = loaded
    node = module.SmallCityRoutePlanner()

    assert node.captured_action_server.action_type is ComputePathToPose
    assert node.captured_action_server.action_name == "/compute_lane_path_to_pose"
    assert node.publisher_args == (PathMessage, "/plan", 10)
    assert ("load_graph", "/real/config/small_city_lane_graph.yaml") in core.calls
    assert ("load_occupancy_map", "/real/maps/small_city.yaml") in core.calls


def test_explicit_start_uses_request_yaw_and_goal_orientation_is_ignored(loaded):
    module, core = loaded
    node = module.SmallCityRoutePlanner()
    handle = FakeGoalHandle(request(use_start=True, start_yaw=0.7, goal_yaw=-1.9))
    activate(node, handle)

    node.execute_callback(handle)

    start_call = next(call for call in core.calls if call[0] == "snap_start")
    goal_call = next(call for call in core.calls if call[0] == "snap_goal")
    assert start_call[1:] == pytest.approx((-15.0, -30.0, 0.7))
    assert goal_call == ("snap_goal", 20.0, 0.0)
    audit_call = next(call for call in core.calls if call[0] == "audit_path")
    assert audit_call[5].heading == 0.0


def test_tf_start_converts_prius_chassis_yaw_to_forward_axis(loaded):
    module, core = loaded
    node = module.SmallCityRoutePlanner()
    node.tf_buffer.transform.transform.rotation = quaternion(0.25)
    handle = FakeGoalHandle(request(use_start=False))
    activate(node, handle)

    node.execute_callback(handle)

    start_call = next(call for call in core.calls if call[0] == "snap_start")
    assert start_call[1:] == pytest.approx((12.0, -4.0, 0.25 - math.pi / 2.0))
    assert node.tf_buffer.lookups[0][:2] == ("map", "chassis")


def test_cancel_request_returns_empty_path_without_publishing(loaded):
    module, _ = loaded
    node = module.SmallCityRoutePlanner()
    handle = FakeGoalHandle(request())
    activate(node, handle)
    handle.is_cancel_requested = True

    result = node.execute_callback(handle)

    assert result.path.header.frame_id == "map"
    assert result.path.poses == []
    assert handle.canceled_count == 1
    assert node.publisher.messages == []
    assert result.planning_time.sec > 0 or result.planning_time.nanosec > 0
    assert any("LANE_PLAN_RESULT" in message and "reason=CANCELED" in message for message in node.logger.messages)


def test_new_goal_invalidates_old_callback_and_old_result_cannot_publish(loaded):
    module, _ = loaded
    node = module.SmallCityRoutePlanner()
    old_handle = FakeGoalHandle(request())
    new_handle = FakeGoalHandle(request(start_yaw=1.0))
    activate(node, old_handle)

    activate(node, new_handle)
    stale_result = node.execute_callback(old_handle)

    assert old_handle.abort_count == 1
    assert stale_result.path.poses == []
    assert node.publisher.messages == []
    assert any("LANE_PLAN_RESULT" in message and "reason=CANCELED" in message for message in node.logger.messages)


def test_empty_core_path_aborts_fail_closed_with_duration(loaded):
    module, core = loaded
    node = module.SmallCityRoutePlanner()
    core.choose_clear_offset = lambda *args: SimpleNamespace(
        path=(), offset=0.0, minimum_clearance=0.0
    )
    handle = FakeGoalHandle(request())
    activate(node, handle)

    result = node.execute_callback(handle)

    assert result.path.poses == []
    assert handle.abort_count == 1
    assert handle.succeed_count == 0
    assert node.publisher.messages == []
    assert result.planning_time.sec > 0 or result.planning_time.nanosec > 0
    assert any(
        "LANE_PLAN_RESULT" in message and "reason=FINAL_PATH_AUDIT_FAILED" in message
        for message in node.logger.messages
    )


def test_success_returns_and_publishes_map_path_with_tangent_quaternions(loaded):
    module, _ = loaded
    node = module.SmallCityRoutePlanner()
    handle = FakeGoalHandle(request(goal_yaw=math.pi))
    activate(node, handle)

    result = node.execute_callback(handle)

    assert handle.succeed_count == 1
    assert handle.abort_count == 0
    assert result.path.header.frame_id == "map"
    assert [pose.header.frame_id for pose in result.path.poses] == ["map", "map"]
    assert result.path.poses[0].pose.orientation.z == pytest.approx(math.sin(math.pi / 4.0))
    assert result.path.poses[0].pose.orientation.w == pytest.approx(math.cos(math.pi / 4.0))
    assert result.path.poses[-1].pose.orientation.z == pytest.approx(0.0)
    assert result.path.poses[-1].pose.orientation.w == pytest.approx(1.0)
    assert node.publisher.messages == [result.path]
    assert result.planning_time.sec > 0 or result.planning_time.nanosec > 0
    stages = [
        "LANE_PLAN_REQUEST",
        "LANE_SNAP_START",
        "LANE_SNAP_GOAL",
        "LANE_ASTAR_RESULT",
        "LANE_GEOMETRY_RESULT",
        "LANE_CLEARANCE_RESULT",
        "LANE_PLAN_RESULT",
    ]
    assert all(any(message.startswith(stage) for message in node.logger.messages) for stage in stages)


def test_runtime_install_and_dependency_declarations_are_additive():
    cmake = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (REPO_ROOT / "package.xml").read_text(encoding="utf-8")

    assert "launch/small_city_route_planner.py" in cmake
    assert "launch/small_city_lane_planner_core.py" in cmake
    assert "DIRECTORY config" in cmake
    assert (REPO_ROOT / "config" / "small_city_lane_graph.yaml").is_file()
    for dependency in (
        "rclpy",
        "nav2_msgs",
        "nav_msgs",
        "geometry_msgs",
        "tf2_ros",
        "python3-yaml",
        "python3-numpy",
    ):
        assert f"<exec_depend>{dependency}</exec_depend>" in package
