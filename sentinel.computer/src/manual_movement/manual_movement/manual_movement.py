from rclpy.node import Node
from std_msgs.msg import String
import json
import rclpy
import pygame
import os
import math

class ManualMovement(Node):
    def __init__(self):
        super().__init__('movement')
        self.publisher = self.create_publisher(String,'movement',10)
        self.get_logger().info("Manual Movement Publisher Started")

        os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"

        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.timer = self.create_timer(0.1, self.publish_data)
    def get_angle(self, axis_0, axis_1, threshold=0.2):

        if axis_0 == None or axis_1 == None:
            return None
        if abs(axis_0) < threshold and abs(axis_1) < threshold:
            return None

        angle_radians = math.atan2(-axis_1, axis_0)
        angle_degrees = math.degrees(angle_radians)

        if angle_degrees < 0:
            angle_degrees += 360

        return angle_degrees
    
    def calc_speed(self, angle, axis_5):
        if angle == None:
            return None,None
        speed_factor = 1
        speed = (axis_5 + 1) / 2
        speed *= speed_factor

        angle = (angle - 45) % 360
        angle_rad = math.radians(angle)

        left_speed = speed * math.cos(angle_rad)
        right_speed = speed * math.sin(angle_rad)

        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1:
            left_speed /= max_speed
            right_speed /= max_speed

        return left_speed, right_speed
    
    def create_dictionary(self, left_speed, right_speed, angle):
        return {
            "left_speed": left_speed,
            "right_speed": right_speed,
            "angle": angle
        }

    def publish_data(self):
        pygame.event.pump()
        axis_0 = self.joystick.get_axis(0)
        axis_1 = self.joystick.get_axis(1)
        axis_5 = self.joystick.get_axis(5)

        angle = self.get_angle(axis_0, axis_1)

        left_speed, right_speed = self.calc_speed(angle, axis_5)

        data = json.dumps(self.create_dictionary(left_speed, right_speed, angle))
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info("Commands published")

def main(args=None):
    rclpy.init(args=args)

    publisher = ManualMovement()
    rclpy.spin(publisher)
        

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()