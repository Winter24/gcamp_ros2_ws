#!/usr/bin/env python3
import math
import copy
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def corner_safe_limits(steer, alpha, max_steer):
    """Limit full-lock corner cutting without changing gentle turns."""
    if abs(alpha) <= 0.6:
        return clamp(steer, -max_steer, max_steer), 2.0

    severity = clamp((abs(alpha) - 0.6) / 0.5, 0.0, 1.0)
    sharp_steer_limit = 0.48
    steer_limit = max_steer - severity * (max_steer - sharp_steer_limit)
    speed_limit = 0.8 - severity * (0.8 - 0.18)
    return clamp(steer, -steer_limit, steer_limit), speed_limit


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def finite(*vals):
    return all(math.isfinite(v) for v in vals)


def signed_command_speed(speed, direction):
    if not math.isfinite(speed) or direction not in (-1, 1):
        return 0.0
    return abs(speed) * direction


def goal_pose_reached(
    car_x,
    car_y,
    car_yaw,
    goal_x,
    goal_y,
    goal_yaw,
    xy_tolerance,
    yaw_tolerance,
):
    if not finite(
        car_x, car_y, car_yaw, goal_x, goal_y, goal_yaw,
        xy_tolerance, yaw_tolerance,
    ):
        return False
    if xy_tolerance < 0.0 or yaw_tolerance < 0.0:
        return False
    position_error = math.hypot(goal_x - car_x, goal_y - car_y)
    yaw_error = abs(norm_angle(goal_yaw - car_yaw))
    return position_error <= xy_tolerance and yaw_error <= yaw_tolerance


def pose_from_xy_yaw(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def forward_anchor_pose(x, y, yaw, distance):
    return pose_from_xy_yaw(
        x + distance * math.cos(yaw),
        y + distance * math.sin(yaw),
        yaw,
    )


def merge_path_poses(first, second, duplicate_tolerance=1e-3):
    merged = list(first)
    remaining = list(second)
    if merged and remaining:
        dx = merged[-1].pose.position.x - remaining[0].pose.position.x
        dy = merged[-1].pose.position.y - remaining[0].pose.position.y
        if math.hypot(dx, dy) <= duplicate_tolerance:
            remaining = remaining[1:]
    merged.extend(remaining)
    return merged


def annotate_path_directions(poses, projection_tolerance=1e-6):
    """Annotate each Reeds-Shepp waypoint as forward (+1) or reverse (-1)."""
    if not poses:
        return []
    if not math.isfinite(projection_tolerance) or projection_tolerance < 0.0:
        raise ValueError("INVALID_PATH_DIRECTION projection tolerance")

    annotated = []
    last_direction = 1
    for index, current in enumerate(poses[:-1]):
        following = poses[index + 1]
        x = current.pose.position.x
        y = current.pose.position.y
        dx = following.pose.position.x - x
        dy = following.pose.position.y - y
        heading = yaw_from_quat(current.pose.orientation)
        if not finite(x, y, dx, dy, heading):
            raise ValueError("INVALID_PATH_DIRECTION non-finite path geometry")
        projection = dx * math.cos(heading) + dy * math.sin(heading)
        if math.hypot(dx, dy) > 1e-9:
            if projection < -projection_tolerance:
                last_direction = -1
            elif projection > projection_tolerance:
                last_direction = 1
        annotated.append((x, y, last_direction))

    final = poses[-1]
    final_x = final.pose.position.x
    final_y = final.pose.position.y
    if not finite(final_x, final_y, yaw_from_quat(final.pose.orientation)):
        raise ValueError("INVALID_PATH_DIRECTION non-finite terminal pose")
    annotated.append((final_x, final_y, last_direction))
    return annotated


def downsample_directional_path(points, min_dist=0.5):
    """Downsample a directional path without deleting gear-change cusps."""
    if not points:
        return []
    if any(point[2] not in (-1, 1) for point in points):
        raise ValueError("INVALID_PATH_DIRECTION waypoint")

    output = [points[0]]
    for index, point in enumerate(points[1:], start=1):
        last_kept = output[-1]
        direction_changed = point[2] != points[index - 1][2]
        far_enough = math.hypot(
            point[0] - last_kept[0], point[1] - last_kept[1]
        ) >= min_dist
        if direction_changed or far_enough:
            output.append(point)

    if output[-1] != points[-1]:
        output.append(points[-1])
    return output


class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        self.last_output = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        self.last_output = 0.0

    def update(self, error, now_sec):
        if self.prev_time is None:
            self.prev_time = now_sec
            self.prev_error = error
            self.last_output = clamp(self.kp * error, self.out_min, self.out_max)
            return self.last_output

        dt = now_sec - self.prev_time
        # Clock chua nhich (vd odom_cb 71Hz > sim /clock 9.5Hz => dt=0 phan lon
        # cac lan goi). KHONG duoc tra 0 => giu lenh cu, cho lan co dt>0 that su.
        if dt <= 0.0 or not math.isfinite(dt):
            return self.last_output

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        u_sat = clamp(u, self.out_min, self.out_max)

        if u != u_sat:
            self.integral -= error * dt

        self.prev_error = error
        self.prev_time = now_sec
        self.last_output = u_sat
        return u_sat


class Nav2PurePursuitPID(Node):
    def __init__(self):
        super().__init__("nav2_pure_pursuit_pid")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.planner_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.plan_request_id = 0
        self.active_plan_goal = None
        self.goal_pose = None

        # GIỮ NGUYÊN
        self.wheelbase = 2.70

        # GIỮ NGUYÊN
        self.kdd = 0.7
        self.min_ld = 0.8
        self.max_ld = 2.0
        self.goal_tolerance = 1.5
        self.goal_yaw_tolerance = 0.3
        self.waypoint_reach_dist = 1.0

        # GIỮ NGUYÊN
        self.target_speed = 2.0
        self.max_speed = 3.0
        self.max_omega = 0.8

        self.max_steer = 0.6458

        self.speed_pid = PID(
            kp=0.8,
            ki=0.05,
            kd=0.02,
            out_min=0.0,
            out_max=self.max_speed
        )

        self.current_speed = 0.0

        # ADDED: ngưỡng để bỏ waypoint đã nằm phía sau xe
        self.behind_wp_threshold = -0.3

        self.get_logger().info("✅ Nav2 forward-only Pure Pursuit PID sẵn sàng.")

    def stop(self):
        self.cmd_pub.publish(Twist())

    def goal_callback(self, msg):
        self.get_logger().info("Đang gọi Nav2 planner...")
        goal = self.normalize_goal_to_map(msg)
        if goal is None:
            return

        self.plan_request_id += 1
        request_id = self.plan_request_id

        if self.active_plan_goal is not None:
            self.active_plan_goal.cancel_goal_async()
            self.active_plan_goal = None

        self.stop()
        self.waypoints = []
        self.goal_pose = None
        self.speed_pid.reset()

        self.request_path(
            goal,
            request_id,
            planner_id="SmacPlannerHybrid",
            stage="direct",
        )

    def normalize_goal_to_map(self, msg):
        goal = copy.deepcopy(msg)
        source_frame = goal.header.frame_id or "map"
        if source_frame != "map":
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", source_frame, rclpy.time.Time()
                )
                goal = do_transform_pose_stamped(goal, transform)
            except Exception as error:
                self.get_logger().warn(
                    f"Không đổi được goal {source_frame} -> map: {error}"
                )
                return None

        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        return goal

    def request_path(self, goal, request_id, planner_id, stage="direct", start=None):
        if not self.planner_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 compute_path_to_pose chưa sẵn sàng.")
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.planner_id = planner_id
        goal_msg.use_start = start is not None
        if start is not None:
            start.header.stamp = self.get_clock().now().to_msg()
            goal_msg.start = start

        self.get_logger().info(
            f"PLAN_REQUEST id={request_id} planner={planner_id} stage={stage} "
            f"goal=({goal.pose.position.x:.2f},{goal.pose.position.y:.2f}) "
            f"frame={goal.header.frame_id}"
        )
        future = self.planner_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda done: self.goal_response_callback(
                done, request_id, goal, planner_id, stage
            )
        )

    def goal_response_callback(self, future, request_id, goal, planner_id, stage):
        goal_handle = future.result()
        if request_id != self.plan_request_id:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"Nav2 planner {planner_id} từ chối goal id={request_id} stage={stage}."
            )
            return

        self.active_plan_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self.path_result_callback(
                done, request_id, goal, planner_id, stage
            )
        )

    def path_result_callback(self, future, request_id, goal, planner_id, stage):
        response = future.result()
        if request_id != self.plan_request_id:
            return

        self.active_plan_goal = None
        goal_status = response.status
        result = response.result
        path = result.path
        planning_time = (
            result.planning_time.sec + result.planning_time.nanosec * 1e-9
        )

        self.get_logger().info(
            f"PLAN_RESULT id={request_id} planner={planner_id} stage={stage} "
            f"status={goal_status} planning_time={planning_time:.3f}s "
            f"poses={len(path.poses)}"
        )

        if not path.poses:
            self.get_logger().warn(
                f"{planner_id} không trả path; status={goal_status}."
            )
            return

        self.set_waypoints_from_poses(
            path.poses,
            frame_id=path.header.frame_id,
            goal_pose=goal.pose,
        )

    def set_waypoints_from_poses(self, poses, frame_id="map", goal_pose=None):
        if not poses:
            self.get_logger().warn("Không có pose hợp lệ để follow.")
            self.goal_pose = None
            return False

        try:
            directional_path = annotate_path_directions(poses)
        except ValueError as error:
            self.get_logger().error(str(error))
            self.waypoints = []
            self.goal_pose = None
            self.stop()
            return False
        self.waypoints = [
            waypoint for waypoint in directional_path
            if finite(waypoint[0], waypoint[1])
        ]
        self.waypoints = downsample_directional_path(
            self.waypoints, min_dist=0.1
        )

        terminal = goal_pose if goal_pose is not None else poses[-1].pose
        terminal_yaw = yaw_from_quat(terminal.orientation)
        if not finite(terminal.position.x, terminal.position.y, terminal_yaw):
            self.get_logger().error("Goal pose chứa NaN/Inf; dừng xe.")
            self.waypoints = []
            self.goal_pose = None
            self.stop()
            return False
        self.goal_pose = (
            terminal.position.x,
            terminal.position.y,
            terminal_yaw,
        )

        self.get_logger().info(
            f"🗺️ Nhận path {len(self.waypoints)} waypoint trong frame: {frame_id}"
        )
        return True

    def downsample_path(self, pts, min_dist=0.5):
        return downsample_directional_path(pts, min_dist)

    def get_robot_pose_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "chassis",
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Không lấy được TF map -> chassis: {e}")
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw_chassis = yaw_from_quat(q)

        # Prius model forward axis is -Y of chassis
        yaw = yaw_chassis - math.pi / 2.0
        yaw = norm_angle(yaw)

        if not finite(x, y, yaw):
            return None

        return x, y, yaw

    def forward_projection(self, car_x, car_y, yaw, x, y):
        dx = x - car_x
        dy = y - car_y
        return dx * math.cos(yaw) + dy * math.sin(yaw)

    def prune_waypoints(self, car_x, car_y, yaw):
        if len(self.waypoints) <= 1:
            return

        # ADDED: cắt path về waypoint gần xe nhất trước
        closest_idx = 0
        closest_dist = float("inf")

        current_direction = self.waypoints[0][2]
        for i, (x, y, direction) in enumerate(self.waypoints):
            if direction != current_direction:
                break
            d = math.hypot(x - car_x, y - car_y)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        if closest_idx > 0:
            self.get_logger().info(
                f"PRUNE_CLOSEST: remove {closest_idx} old waypoints, closest_dist={closest_dist:.2f}"
            )
            self.waypoints = self.waypoints[closest_idx:]

        # ADDED: xoá waypoint đầu nếu đã gần hoặc đã nằm sau xe rõ ràng
        removed = 0
        while len(self.waypoints) > 1:
            wp_x, wp_y, direction = self.waypoints[0]

            dx = wp_x - car_x
            dy = wp_y - car_y
            d0 = math.hypot(dx, dy)
            fp = direction * self.forward_projection(
                car_x, car_y, yaw, wp_x, wp_y
            )

            if d0 < self.waypoint_reach_dist or fp < self.behind_wp_threshold:
                self.waypoints.pop(0)
                removed += 1
            else:
                break

        if removed > 0:
            self.get_logger().info(f"PRUNE_BEHIND: remove {removed} waypoints")

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        if finite(vx, vy):
            self.current_speed = math.hypot(vx, vy)
        else:
            self.current_speed = 0.0

        if not self.waypoints:
            return

        pose = self.get_robot_pose_map()
        if pose is None:
            self.stop()
            return

        car_x, car_y, yaw = pose

        # CHANGED: thay block xoá waypoint cũ bằng prune thông minh hơn
        self.prune_waypoints(car_x, car_y, yaw)

        if not self.waypoints:
            self.stop()
            return

        if self.goal_pose is None:
            self.get_logger().warn("Thiếu goal pose có yaw; dừng xe.")
            self.waypoints = []
            self.stop()
            return

        goal_x, goal_y, goal_yaw = self.goal_pose
        dist_to_goal = math.hypot(goal_x - car_x, goal_y - car_y)

        if goal_pose_reached(
            car_x,
            car_y,
            yaw,
            goal_x,
            goal_y,
            goal_yaw,
            self.goal_tolerance,
            self.goal_yaw_tolerance,
        ):
            self.get_logger().info("🎉 Đã đến goal.")
            self.waypoints = []
            self.goal_pose = None
            self.stop()
            return

        ld = clamp(self.kdd * max(self.current_speed, 0.1), self.min_ld, self.max_ld)

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        desired_direction = self.waypoints[0][2]
        target_x, target_y, _ = self.waypoints[0]

        for x, y, direction in self.waypoints:
            if direction != desired_direction:
                break
            target_x, target_y = x, y
            dx = x - car_x
            dy = y - car_y
            dist = math.hypot(dx, dy)
            fp = desired_direction * self.forward_projection(
                car_x, car_y, yaw, x, y
            )

            if fp < self.behind_wp_threshold:
                continue

            if dist >= ld:
                target_x, target_y = x, y
                break

        travel_yaw = yaw if desired_direction == 1 else yaw + math.pi
        alpha = math.atan2(target_y - car_y, target_x - car_x) - travel_yaw
        alpha = norm_angle(alpha)

        target_dist = math.hypot(target_x - car_x, target_y - car_y)
        target_fp = desired_direction * self.forward_projection(
            car_x, car_y, yaw, target_x, target_y
        )

        if abs(alpha) > 1.0 or target_fp < 0.0:
            self.get_logger().info(
                f"POSE=({car_x:.2f},{car_y:.2f}) "
                f"YAW={math.degrees(yaw):.1f}deg "
                f"TARGET=({target_x:.2f},{target_y:.2f}) "
                f"D={target_dist:.2f} "
                f"FP={target_fp:.2f} "
                f"ALPHA={math.degrees(alpha):.1f}deg "
                f"N_WP={len(self.waypoints)}"
            )

        if not finite(alpha, ld) or ld <= 0.01:
            self.get_logger().warn("alpha/ld lỗi, dừng xe.")
            self.stop()
            return

        curvature = 2.0 * math.sin(alpha) / ld

        steer = math.atan(self.wheelbase * curvature)
        steer *= desired_direction
        steer = clamp(steer, -self.max_steer, self.max_steer)

        # =========================
        # FOLLOW_PATH bình thường
        # =========================
        speed_error = self.target_speed - self.current_speed
        cmd_speed_magnitude = self.speed_pid.update(speed_error, now_sec)

        steer, corner_speed_limit = corner_safe_limits(
            steer, alpha, self.max_steer
        )
        cmd_speed_magnitude = min(cmd_speed_magnitude, corner_speed_limit)

        if dist_to_goal < 4.0:
            cmd_speed_magnitude = min(cmd_speed_magnitude, 0.5)

        cmd_speed = signed_command_speed(cmd_speed_magnitude, desired_direction)

        if not finite(cmd_speed, steer):
            self.get_logger().warn("cmd bị NaN/Inf, dừng xe.")
            self.stop()
            return

        if abs(alpha) > 1.0 or target_fp < 0.0:
            self.get_logger().info(
                f"CMD_SPEED={cmd_speed:.2f} "
                f"STEER={math.degrees(steer):.1f}deg "
                "STATE=FOLLOW_PATH"
            )

        cmd = Twist()
        cmd.linear.x = cmd_speed
        cmd.angular.z = steer
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PurePursuitPID()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
