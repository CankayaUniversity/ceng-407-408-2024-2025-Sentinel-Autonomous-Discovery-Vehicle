import rclpy
import json
from rclpy.node import Node

from std_msgs.msg import String


class MovementSubscriber(Node):

    def __init__(self):
        super().__init__('movement_subscriber')
        self.subscription = self.create_subscription(
            String,
            'movement',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = -1
        self.data_sub = []
        self.read_speed_angle_data('src/simulation/simulation/movement.json')
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        data = json.loads(msg.data)
        self.data_sub.append(data)
        self.i = self.i + 1



    def read_speed_angle_data(self,filename):
        with open(filename, "r") as file:
            self.data = json.load(file)

    def get_movement_data(self):
        if self.i > -1:
            return self.data_sub[self.i]
        return None
        # json test
        # self.i = self.i + 1
        # if self.i >= len(self.data['data']):
        #     return None
        # return self.data['data'][self.i]
    

def main(args=None):
    # rclpy.init(args=args)

    # movement_subscriber = MovementSubscriber()

    # rclpy.spin(movement_subscriber)

    # movement_subscriber.destroy_node()
    # rclpy.shutdown()
    pass


if __name__ == '__main__':
    main()