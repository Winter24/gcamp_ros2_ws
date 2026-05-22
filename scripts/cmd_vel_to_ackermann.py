#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

WHEELBASE = 2.86  # metres (front_axle_x + |rear_axle_x|)

class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cont/reference', 10)
        self.create_subscription(Twist, '/cmd_vel', self.callback, 10)

    def callback(self, msg: Twist):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = msg.linear.x
        if abs(msg.linear.x) > 0.01:
            out.drive.steering_angle = math.atan(msg.angular.z * WHEELBASE / msg.linear.x)
        else:
            out.drive.steering_angle = 0.0
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(CmdVelToAckermann())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
