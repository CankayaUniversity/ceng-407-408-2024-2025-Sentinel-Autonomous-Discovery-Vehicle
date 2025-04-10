from rclpy.node import Node
from geometry_msgs.msg import Twist
from movement.joystick import Joystick
from rclpy.qos import QoSProfile
import rclpy


class TwistPublisher(Node):
    def __init__(self):
        super().__init__("movement")
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, "movement", qos_profile)
        self.timer = self.create_timer(0.1, self.publish_twist)
        self.is_published = False
        self.joystick = Joystick()
        self.get_logger().info("Manual Movement Started")

    def publish_twist(self):
        result = self.joystick.event()
        if result is None:
            return
        linear, angular = result

        twist = Twist()
        twist.angular.z = angular
        twist.linear.x = linear
        self.publisher.publish(twist)
        self.get_logger().info("Command Published")


def main(args=None):
    rclpy.init(args=args)

    publisher = TwistPublisher()
    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
