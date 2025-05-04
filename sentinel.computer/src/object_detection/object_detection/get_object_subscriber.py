import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from .get_object_publisher import GetObjectPublisher

class GetObjectSubscriber(Node):

    def __init__(self):
        super().__init__('get_object_subscriber')
        self.subscription = self.create_subscription(
            String,
            'get_object_request',
            self.listener_callback,
            10)
        self.object_publisher = GetObjectPublisher()
    def listener_callback(self, msg):
        self.get_logger().info('Get request: "%s"' % msg.data)
        if msg.data == "all":
            self.object_publisher.publish_links()
        else:
            self.object_publisher.publish_one(msg.data)
