#!/usr/bin/env python3
import math
import rclpy
from enum import Enum

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose

from tf2_ros import Buffer, TransformListener


class DriveState(Enum):
    FOLLOW_PATH = 0
    SHORT_REVERSE = 1


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def finite(*vals):
    return all(math.isfinite(v) for v in vals)


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

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def update(self, error, now_sec):
        if self.prev_time is None:
            self.prev_time = now_sec
            self.prev_error = error
            return clamp(self.kp * error, self.out_min, self.out_max)

        dt = now_sec - self.prev_time
        if dt <= 0.0 or not math.isfinite(dt):
            return 0.0

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        u_sat = clamp(u, self.out_min, self.out_max)

        if u != u_sat:
            self.integral -= error * dt

        self.prev_error = error
        self.prev_time = now_sec
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

        # GIỮ NGUYÊN
        self.wheelbase = 2.70

        # GIỮ NGUYÊN
        self.kdd = 0.7
        self.min_ld = 1.5
        self.max_ld = 8.0
        self.goal_tolerance = 1.5
        self.waypoint_reach_dist = 1.0

        # GIỮ NGUYÊN
        self.target_speed = 6.0
        self.max_speed = 7.0
        self.max_omega = 0.8

        self.speed_pid = PID(
            kp=0.8,
            ki=0.05,
            kd=0.02,
            out_min=0.0,
            out_max=self.max_speed
        )

        self.current_speed = 0.0

        # ADDED: state lùi khi thật sự kẹt ở cua gắt
        self.state = DriveState.FOLLOW_PATH
        self.state_start_time = None
        self.max_steer = 0.6458

        # GIỮ THEO CODE HIỆN TẠI CỦA BẠN
        self.reverse_trigger_alpha = 1.1
        self.reverse_time = 10.0
        self.reverse_speed = 0.18

        # GIỮ THEO CODE HIỆN TẠI CỦA BẠN
        self.stuck_check_time = 10.0
        self.stuck_min_move = 0.5
        self.stuck_ref_time = None
        self.stuck_ref_pos = None

        # GIỮ THEO CODE HIỆN TẠI CỦA BẠN
        self.last_reverse_time = -999.0
        self.reverse_cooldown = 12.0
        self.reverse_steer = 0.0

        # ADDED: ngưỡng để bỏ waypoint đã nằm phía sau xe
        self.behind_wp_threshold = -0.3

        self.get_logger().info("✅ Nav2 Pure Pursuit PID + Stuck Reverse + Waypoint Pruning sẵn sàng.")

    def stop(self):
        self.cmd_pub.publish(Twist())

    def goal_callback(self, msg):
        self.get_logger().info("Đang gọi Nav2 planner...")
        self.stop()
        self.waypoints = []
        self.speed_pid.reset()

        self.state = DriveState.FOLLOW_PATH
        self.state_start_time = None
        self.last_reverse_time = -999.0
        self.reverse_steer = 0.0
        self.stuck_ref_time = None
        self.stuck_ref_pos = None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = msg
        goal_msg.planner_id = "SmacPlannerHybrid"
        goal_msg.use_start = False

        self.planner_client.wait_for_server()
        future = self.planner_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 planner từ chối goal.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.path_result_callback)

    def path_result_callback(self, future):
        result = future.result().result
        path = result.path

        if not path.poses:
            self.get_logger().warn("Nav2 không trả path.")
            return

        self.waypoints = []
        for ps in path.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            if finite(x, y):
                self.waypoints.append((x, y))

        self.waypoints = self.downsample_path(self.waypoints, min_dist=0.1)

        self.get_logger().info(
            f"🗺️ Nhận path {len(self.waypoints)} waypoint trong frame: {path.header.frame_id}"
        )

    def downsample_path(self, pts, min_dist=0.5):
        if not pts:
            return []

        out = [pts[0]]
        last_x, last_y = pts[0]

        for x, y in pts[1:]:
            if math.hypot(x - last_x, y - last_y) >= min_dist:
                out.append((x, y))
                last_x, last_y = x, y

        if out[-1] != pts[-1]:
            out.append(pts[-1])

        return out

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

    def update_stuck_detector(self, car_x, car_y, now_sec, sharp_turn):
        if not sharp_turn:
            self.stuck_ref_time = None
            self.stuck_ref_pos = None
            return False

        if self.stuck_ref_time is None or self.stuck_ref_pos is None:
            self.stuck_ref_time = now_sec
            self.stuck_ref_pos = (car_x, car_y)
            return False

        elapsed = now_sec - self.stuck_ref_time
        moved = math.hypot(
            car_x - self.stuck_ref_pos[0],
            car_y - self.stuck_ref_pos[1]
        )

        if elapsed >= self.stuck_check_time:
            if moved < self.stuck_min_move:
                return True

            self.stuck_ref_time = now_sec
            self.stuck_ref_pos = (car_x, car_y)

        return False

    def prune_waypoints(self, car_x, car_y, yaw):
        if len(self.waypoints) <= 1:
            return

        # ADDED: cắt path về waypoint gần xe nhất trước
        closest_idx = 0
        closest_dist = float("inf")

        for i, (x, y) in enumerate(self.waypoints):
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
            wp_x, wp_y = self.waypoints[0]

            dx = wp_x - car_x
            dy = wp_y - car_y
            d0 = math.hypot(dx, dy)
            fp = self.forward_projection(car_x, car_y, yaw, wp_x, wp_y)

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

        goal_x, goal_y = self.waypoints[-1]
        dist_to_goal = math.hypot(goal_x - car_x, goal_y - car_y)

        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info("🎉 Đã đến goal.")
            self.waypoints = []
            self.state = DriveState.FOLLOW_PATH
            self.state_start_time = None
            self.stuck_ref_time = None
            self.stuck_ref_pos = None
            self.stop()
            return

        ld = clamp(self.kdd * max(self.current_speed, 0.1), self.min_ld, self.max_ld)

        # CHANGED: chọn target cho phép waypoint bên hông,
        # nhưng bỏ waypoint đã nằm sau xe rõ ràng
        target_x, target_y = self.waypoints[-1]

        for x, y in self.waypoints:
            dx = x - car_x
            dy = y - car_y
            dist = math.hypot(dx, dy)
            fp = self.forward_projection(car_x, car_y, yaw, x, y)

            if fp < self.behind_wp_threshold:
                continue

            if dist >= ld:
                target_x, target_y = x, y
                break

        alpha = math.atan2(target_y - car_y, target_x - car_x) - yaw
        alpha = norm_angle(alpha)

        target_dist = math.hypot(target_x - car_x, target_y - car_y)
        target_fp = self.forward_projection(car_x, car_y, yaw, target_x, target_y)

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

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.state_start_time is None:
            self.state_start_time = now_sec

        steer = math.atan(self.wheelbase * curvature)
        steer = clamp(steer, -self.max_steer, self.max_steer)

        # =========================
        # SHORT_REVERSE
        # =========================
        if self.state == DriveState.SHORT_REVERSE:
            elapsed = now_sec - self.state_start_time

            cmd = Twist()
            cmd.linear.x = -self.reverse_speed

            # Lùi thẳng, trả bánh về 0
            cmd.angular.z = 0.0

            if elapsed >= self.reverse_time:
                self.get_logger().warn("Lùi xong -> FOLLOW_PATH")
                self.state = DriveState.FOLLOW_PATH
                self.state_start_time = now_sec
                self.speed_pid.reset()
                self.stuck_ref_time = None
                self.stuck_ref_pos = None
                self.stop()
                return

            if not finite(cmd.linear.x, cmd.angular.z):
                self.stop()
                return

            self.cmd_pub.publish(cmd)
            return

        # =========================
        # FOLLOW_PATH bình thường
        # =========================
        speed_error = self.target_speed - self.current_speed
        cmd_speed = self.speed_pid.update(speed_error, now_sec)

        # GIỮ NGUYÊN logic giảm tốc cũ
        if abs(alpha) > 0.6:
            cmd_speed = min(cmd_speed, 1.2)
        else:
            cmd_speed = min(cmd_speed, 6.0)

        if dist_to_goal < 8.0:
            cmd_speed = min(cmd_speed, 1.2)

        sharp_turn = abs(alpha) > self.reverse_trigger_alpha

        # GIỮ NGUYÊN logic cua gắt
        if sharp_turn:
            steer = self.max_steer if steer > 0.0 else -self.max_steer
            cmd_speed = min(cmd_speed, 0.18)

        stuck_in_corner = self.update_stuck_detector(
            car_x,
            car_y,
            now_sec,
            sharp_turn
        )

        can_reverse = (now_sec - self.last_reverse_time) > self.reverse_cooldown

        if sharp_turn and can_reverse and stuck_in_corner:
            self.get_logger().warn("Cua gắt và vị trí gần như không đổi -> SHORT_REVERSE")
            self.state = DriveState.SHORT_REVERSE
            self.state_start_time = now_sec
            self.last_reverse_time = now_sec
            self.reverse_steer = steer
            self.stuck_ref_time = None
            self.stuck_ref_pos = None
            self.speed_pid.reset()
            self.stop()
            return

        if not finite(cmd_speed, steer):
            self.get_logger().warn("cmd bị NaN/Inf, dừng xe.")
            self.stop()
            return

        if abs(alpha) > 1.0 or target_fp < 0.0:
            self.get_logger().info(
                f"CMD_SPEED={cmd_speed:.2f} "
                f"STEER={math.degrees(steer):.1f}deg "
                f"STATE={self.state.name}"
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