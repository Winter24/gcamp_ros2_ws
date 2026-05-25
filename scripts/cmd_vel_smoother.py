#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def step_towards(current, target, max_delta):
    delta = clamp(target - current, -max_delta, max_delta)
    return current + delta


class CmdVelSmoother(Node):
    def __init__(self):
        super().__init__('cmd_vel_smoother')

        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', 'cmd_vel_smoothed')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('max_linear_speed', 1.5)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('linear_accel', 0.7)
        self.declare_parameter('angular_accel', 1.0)
        self.declare_parameter('command_timeout', 0.3)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.rate = float(self.get_parameter('rate').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.linear_accel = float(self.get_parameter('linear_accel').value)
        self.angular_accel = float(self.get_parameter('angular_accel').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_cmd_time = time.monotonic()

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self.on_cmd, 10)
        self.running = True
        self.worker = threading.Thread(target=self.publish_loop, daemon=True)
        self.worker.start()

    def on_cmd(self, msg):
        self.target_linear = clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        self.target_angular = clamp(msg.angular.z, -self.max_angular_speed, self.max_angular_speed)
        self.last_cmd_time = time.monotonic()

    def publish_loop(self):
        period = 1.0 / self.rate
        while self.running and rclpy.ok():
            self.publish_smoothed(period)
            time.sleep(period)

    def publish_smoothed(self, dt):
        if time.monotonic() - self.last_cmd_time > self.command_timeout:
            target_linear = 0.0
            target_angular = 0.0
        else:
            target_linear = self.target_linear
            target_angular = self.target_angular

        self.current_linear = step_towards(
            self.current_linear, target_linear, self.linear_accel * dt)
        self.current_angular = step_towards(
            self.current_angular, target_angular, self.angular_accel * dt)

        if math.isclose(self.current_linear, 0.0, abs_tol=1e-4):
            self.current_linear = 0.0
        if math.isclose(self.current_angular, 0.0, abs_tol=1e-4):
            self.current_angular = 0.0

        out = Twist()
        out.linear.x = self.current_linear
        out.angular.z = self.current_angular
        self.pub.publish(out)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'worker') and self.worker.is_alive():
            self.worker.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
