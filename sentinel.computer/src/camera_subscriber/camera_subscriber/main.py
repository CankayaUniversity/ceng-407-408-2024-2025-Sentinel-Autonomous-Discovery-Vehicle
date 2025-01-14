import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.subscription = self.create_subscription(Image, "camera", self.listener_callback, 10)
        self.get_logger().info("Camera subscriber started")

    def listener_callback(self, msg):
        self.get_logger().info("Frame received")


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
