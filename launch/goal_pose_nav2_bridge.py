#!/usr/bin/env python3
import copy

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalRequestTracker:
    """Keep only the newest asynchronous NavigateToPose request active."""

    def __init__(self):
        self.request_id = 0
        self.active_goal = None

    def begin_request(self):
        self.request_id += 1
        if self.active_goal is not None:
            self.active_goal.cancel_goal_async()
            self.active_goal = None
        return self.request_id

    def is_current(self, request_id):
        return request_id == self.request_id

    def accept_goal(self, request_id, goal_handle):
        if not self.is_current(request_id):
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return False
        if not goal_handle.accepted:
            return False
        self.active_goal = goal_handle
        return True

    def finish_request(self, request_id, goal_handle):
        if self.is_current(request_id) and self.active_goal is goal_handle:
            self.active_goal = None
            return True
        return False


class GoalPoseNav2Bridge(Node):
    def __init__(self):
        super().__init__("goal_pose_nav2_bridge")
        self.tracker = GoalRequestTracker()
        self.navigate_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.get_logger().info(
            "HMI /goal_pose -> Nav2 NavigateToPose bridge ready"
        )

    def goal_callback(self, pose):
        request_id = self.tracker.begin_request()
        self.stop_pub.publish(Twist())

        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server is unavailable")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = copy.deepcopy(pose)
        if not goal_msg.pose.header.frame_id:
            goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"NAV_GOAL_REQUEST id={request_id} "
            f"position=({pose.pose.position.x:.2f},{pose.pose.position.y:.2f})"
        )
        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda done: self.goal_response_callback(done, request_id)
        )

    def goal_response_callback(self, future, request_id):
        try:
            goal_handle = future.result()
        except Exception as error:
            if self.tracker.is_current(request_id):
                self.get_logger().error(
                    f"NAV_GOAL_RESPONSE id={request_id} error={error}"
                )
            return

        if not self.tracker.accept_goal(request_id, goal_handle):
            if self.tracker.is_current(request_id):
                self.get_logger().warn(f"NAV_GOAL_REJECTED id={request_id}")
            return

        self.get_logger().info(f"NAV_GOAL_ACCEPTED id={request_id}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self.result_callback(done, request_id, goal_handle)
        )

    def result_callback(self, future, request_id, goal_handle):
        if not self.tracker.finish_request(request_id, goal_handle):
            return
        try:
            status = future.result().status
        except Exception as error:
            self.get_logger().error(
                f"NAV_GOAL_RESULT id={request_id} error={error}"
            )
            self.stop_pub.publish(Twist())
            return

        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(status, str(status))
        self.get_logger().info(
            f"NAV_GOAL_RESULT id={request_id} status={status_name}"
        )
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.stop_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseNav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
