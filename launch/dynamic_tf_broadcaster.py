#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_map_to_odom_broadcaster')

        self.br = TransformBroadcaster(self)

        # Initial map->odom offset; updated by /initialpose clicks in RViz.
        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_yaw = 0.0

        # Cached odom pose so we can reproject when the user re-localises.
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.init_pose_callback, 10)

        # Publish at 50 Hz so Nav2 always finds a fresh map->odom transform.
        self.create_timer(0.02, self.publish_tf)
        self.get_logger().info('=== map->odom broadcaster ready ===')

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def init_pose_callback(self, msg):
        click_x = msg.pose.pose.position.x
        click_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        click_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.map_to_odom_yaw = click_yaw - self.odom_yaw
        c = math.cos(self.map_to_odom_yaw)
        s = math.sin(self.map_to_odom_yaw)
        rotated_odom_x = self.odom_x * c - self.odom_y * s
        rotated_odom_y = self.odom_x * s + self.odom_y * c

        self.map_to_odom_x = click_x - rotated_odom_x
        self.map_to_odom_y = click_y - rotated_odom_y
        self.get_logger().info(
            f'New map->odom offset: x={self.map_to_odom_x:.2f}, '
            f'y={self.map_to_odom_y:.2f}, yaw={self.map_to_odom_yaw:.2f}')

    def publish_tf(self):
        t = TransformStamped()
        # Always stamp with the node clock (sim time when use_sim_time=True).
        # Using the latest /odom stamp can be older than "now" by a few ms,
        # which makes Nav2 controllers extrapolate and complain.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = self.map_to_odom_x
        t.transform.translation.y = self.map_to_odom_y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.map_to_odom_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.map_to_odom_yaw / 2.0)
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
