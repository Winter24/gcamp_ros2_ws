import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class DummySubscriber(Node):
    def __init__(self):
        super().__init__('dummy_camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    dummy_subscriber = DummySubscriber()
    rclpy.spin(dummy_subscriber)
    dummy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
