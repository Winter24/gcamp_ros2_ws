#!/usr/bin/env python3
"""ROS 2 action wrapper for the fixed small-city lane-graph planner."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

import small_city_lane_planner_core as core


ACTION_NAME = "/compute_lane_path_to_pose"
PLAN_TOPIC = "/plan"
MAP_FRAME = "map"
CHASSIS_FRAME = "chassis"


def _wrapped_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion) -> float:
    """Return planar yaw from a geometry quaternion."""
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def prius_forward_yaw(chassis_yaw: float) -> float:
    """Convert the Prius chassis-axis yaw to its physical forward-axis yaw."""
    return _wrapped_angle(chassis_yaw - math.pi / 2.0)


def _set_planning_duration(result, elapsed_seconds: float) -> None:
    # A non-zero duration makes both fast unit-test aborts and real results
    # distinguishable from an unpopulated default message.
    nanoseconds = max(1, int(max(0.0, elapsed_seconds) * 1_000_000_000))
    result.planning_time.sec = nanoseconds // 1_000_000_000
    result.planning_time.nanosec = nanoseconds % 1_000_000_000


class SmallCityRoutePlanner(Node):
    """Serve fail-closed, forward-only lane paths for the small-city map."""

    def __init__(self):
        super().__init__("small_city_route_planner")

        graph_path = str(self.declare_parameter("graph_path", "").value).strip()
        map_path = str(self.declare_parameter("map_path", "").value).strip()
        if not graph_path or not map_path:
            raise ValueError("graph_path and map_path parameters are required")

        self.graph = core.load_graph(graph_path)
        if self.graph.map_frame != MAP_FRAME:
            raise ValueError("small-city lane graph must use the map frame")
        self.occupancy = core.load_occupancy_map(map_path)
        self.map_frame = MAP_FRAME

        self.plan_publisher = self.create_publisher(Path, PLAN_TOPIC, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._request_lock = threading.Lock()
        self._next_request_id = 0
        self._current_request_id = 0
        self._current_goal_handle = None
        self._request_ids: dict[int, int] = {}

        callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            ComputePathToPose,
            ACTION_NAME,
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=callback_group,
        )
        self.get_logger().info(
            f"Small-city lane planner ready action={ACTION_NAME} frame={self.map_frame}"
        )

    def goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle) -> None:
        """Make the accepted goal current before allowing it to execute."""
        with self._request_lock:
            previous = self._current_goal_handle
            self._next_request_id += 1
            request_id = self._next_request_id
            self._current_request_id = request_id
            self._current_goal_handle = goal_handle
            self._request_ids[id(goal_handle)] = request_id
            if previous is not None and previous is not goal_handle and previous.is_active:
                previous.abort()
        goal_handle.execute()

    def _request_id(self, goal_handle) -> int:
        with self._request_lock:
            return self._request_ids.get(id(goal_handle), 0)

    def _is_current(self, request_id: int, goal_handle) -> bool:
        with self._request_lock:
            return (
                request_id != 0
                and request_id == self._current_request_id
                and goal_handle is self._current_goal_handle
            )

    def _is_cancelled(self, request_id: int, goal_handle) -> bool:
        return (
            bool(goal_handle.is_cancel_requested)
            or not rclpy.ok()
            or not self._is_current(request_id, goal_handle)
        )

    def _empty_result(self):
        result = ComputePathToPose.Result()
        result.path.header.frame_id = self.map_frame
        result.path.header.stamp = self.get_clock().now().to_msg()
        return result

    def _finish_failure(
        self,
        goal_handle,
        request_id: int,
        result,
        reason: str,
        started: float,
        detail: str = "",
    ):
        _set_planning_duration(result, time.perf_counter() - started)
        if goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        with self._request_lock:
            if (
                request_id == self._current_request_id
                and goal_handle is self._current_goal_handle
            ):
                self._current_goal_handle = None
            self._request_ids.pop(id(goal_handle), None)
        detail_field = f" detail={detail}" if detail else ""
        elapsed = result.planning_time.sec + result.planning_time.nanosec * 1e-9
        self.get_logger().warning(
            f"LANE_PLAN_RESULT id={request_id} status=ABORTED reason={reason} "
            f"points=0 planning_time={elapsed:.6f}s{detail_field}"
        )
        return result

    def _start_pose(self, request):
        if request.use_start:
            frame = request.start.header.frame_id
            if frame and frame != self.map_frame:
                raise core.SnapError("START_OUTSIDE_ROAD")
            pose = request.start.pose
            return (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
                "request",
            )

        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            CHASSIS_FRAME,
            Time(),
        )
        translation = transform.transform.translation
        yaw = prius_forward_yaw(
            yaw_from_quaternion(transform.transform.rotation)
        )
        return translation.x, translation.y, yaw, "tf"

    def _path_message(self, points) -> Path:
        path = Path()
        stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        path.header.stamp = stamp
        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = stamp
            pose.pose.position.x = float(point.x)
            pose.pose.position.y = float(point.y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(point.yaw / 2.0)
            pose.pose.orientation.w = math.cos(point.yaw / 2.0)
            path.poses.append(pose)
        return path

    def execute_callback(self, goal_handle):
        """Run the pure planning pipeline and publish only a current audited path."""
        started = time.perf_counter()
        result = self._empty_result()
        request_id = self._request_id(goal_handle)
        request = goal_handle.request
        self.get_logger().info(
            f"LANE_PLAN_REQUEST id={request_id} use_start={bool(request.use_start)}"
        )

        if self._is_cancelled(request_id, goal_handle):
            return self._finish_failure(
                goal_handle, request_id, result, "CANCELED", started
            )

        try:
            try:
                start_x, start_y, start_yaw, start_source = self._start_pose(request)
            except core.SnapError:
                raise
            except Exception as error:
                self.get_logger().error(
                    f"LANE_SNAP_START id={request_id} reason=START_OUTSIDE_ROAD "
                    f"detail={error}"
                )
                raise core.SnapError("START_OUTSIDE_ROAD") from error

            start_snap = core.snap_start(
                self.graph, start_x, start_y, start_yaw
            )
            self.get_logger().info(
                f"LANE_SNAP_START id={request_id} status=OK source={start_source} "
                f"edge={start_snap.edge_id} position=({start_snap.x:.3f},{start_snap.y:.3f})"
            )
            if self._is_cancelled(request_id, goal_handle):
                raise core.RouteError("CANCELED")

            goal_frame = request.goal.header.frame_id
            if goal_frame and goal_frame != self.map_frame:
                raise core.SnapError("GOAL_OUTSIDE_ROAD")
            # Goal orientation is intentionally never read. The terminal yaw
            # is the tangent recorded by snap_goal and the generated geometry.
            goal_snap = core.snap_goal(
                self.graph,
                request.goal.pose.position.x,
                request.goal.pose.position.y,
            )
            self.get_logger().info(
                f"LANE_SNAP_GOAL id={request_id} status=OK "
                f"edge={goal_snap.edge_id} position=({goal_snap.x:.3f},{goal_snap.y:.3f})"
            )
            if self._is_cancelled(request_id, goal_handle):
                raise core.RouteError("CANCELED")

            route = core.astar_route(
                self.graph,
                start_snap,
                goal_snap,
                is_cancelled=lambda: self._is_cancelled(
                    request_id, goal_handle
                ),
            )
            self.get_logger().info(
                f"LANE_ASTAR_RESULT id={request_id} status=OK edges={len(route)}"
            )
            if self._is_cancelled(request_id, goal_handle):
                raise core.RouteError("CANCELED")

            clearance = core.choose_clear_offset(
                self.graph,
                route,
                start_snap,
                goal_snap,
                self.occupancy,
            )
            path_points = tuple(clearance.path)
            self.get_logger().info(
                f"LANE_GEOMETRY_RESULT id={request_id} status=OK points={len(path_points)}"
            )
            self.get_logger().info(
                f"LANE_CLEARANCE_RESULT id={request_id} status=OK "
                f"offset={clearance.offset:.3f} minimum={clearance.minimum_clearance:.3f}"
            )
            if not path_points:
                return self._finish_failure(
                    goal_handle,
                    request_id,
                    result,
                    "FINAL_PATH_AUDIT_FAILED",
                    started,
                    "EMPTY_PATH",
                )
            if self._is_cancelled(request_id, goal_handle):
                raise core.RouteError("CANCELED")

            audit = core.audit_path(
                self.graph,
                route,
                path_points,
                start_snap,
                goal_snap,
                self.occupancy,
            )
            if not audit.valid:
                return self._finish_failure(
                    goal_handle,
                    request_id,
                    result,
                    audit.reason or "FINAL_PATH_AUDIT_FAILED",
                    started,
                    audit.detail,
                )
            path = self._path_message(path_points)
        except (core.SnapError, core.RouteError) as error:
            reason = getattr(error, "reason", str(error))
            if reason == "CANCELED" or self._is_cancelled(
                request_id, goal_handle
            ):
                reason = "CANCELED"
            return self._finish_failure(
                goal_handle, request_id, result, reason, started
            )
        except Exception as error:
            self.get_logger().error(
                f"LANE_PLAN_RESULT id={request_id} internal_error={error}"
            )
            return self._finish_failure(
                goal_handle,
                request_id,
                result,
                "FINAL_PATH_AUDIT_FAILED",
                started,
                type(error).__name__,
            )

        # Hold the same lock used by accepted-goal replacement across the
        # last stale check and publication. Thus an older callback can never
        # publish after a newer request has become current.
        with self._request_lock:
            if (
                request_id != self._current_request_id
                or goal_handle is not self._current_goal_handle
                or goal_handle.is_cancel_requested
                or not rclpy.ok()
            ):
                stale = True
            else:
                stale = False
                result.path = path
                _set_planning_duration(result, time.perf_counter() - started)
                self.plan_publisher.publish(path)
                goal_handle.succeed()
                self._current_goal_handle = None
                self._request_ids.pop(id(goal_handle), None)
        if stale:
            return self._finish_failure(
                goal_handle, request_id, result, "CANCELED", started
            )

        elapsed = result.planning_time.sec + result.planning_time.nanosec * 1e-9
        self.get_logger().info(
            f"LANE_PLAN_RESULT id={request_id} status=SUCCEEDED reason=NONE "
            f"points={len(path.poses)} planning_time={elapsed:.6f}s"
        )
        return result

    def destroy_node(self):
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SmallCityRoutePlanner()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
