import rclpy
import json
from rclpy.node import Node
from collections import deque
from std_msgs.msg import String


class MovementSubscriber(Node):

    def __init__(self):
        super().__init__('movement_subscriber')
        self.subscription = self.create_subscription(
            String,
            'movement',
            self.listener_callback,
            10)
        
        self.data_sub = deque(maxlen=20)
        self.subscription  # prevent unused variable warning
        self.data_sub.append({"left_speed": 0, "right_speed": 0, "angle": 0})
        self.read_speed_angle_data('src/simulation/simulation/movement.json')

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        data = json.loads(msg.data)
        self.data_sub.append(data)



    def read_speed_angle_data(self,filename):
        with open(filename, "r") as file:
            self.data = json.load(file)

    def get_movement_data(self):
        return self.data_sub[-1]
    
        # json test
        # self.i = self.i + 1
        # if self.i >= len(self.data['data']):
        #     return None
        # return self.data['data'][self.i]
    
