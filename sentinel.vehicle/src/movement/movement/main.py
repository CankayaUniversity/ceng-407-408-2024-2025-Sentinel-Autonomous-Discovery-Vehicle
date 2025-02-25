# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import json
from rclpy.node import Node
from gpiozero import Robot
from std_msgs.msg import String


class MovementSubscriber(Node):

    def __init__(self, left, right):
        super().__init__("Movement")
        self.robot = Robot(left=left, right=right)
        self.subscription = self.create_subscription(
            String, "movement", self.listener_callback, 10
        )

        self.get_logger().info("Movement Subscriber Started")

    def listener_callback(self, msg):

        data = json.loads(msg.data)
        left_speed = data["left_speed"]
        right_speed = data["right_speed"]
        angle = data["angle"]

        if angle is None:
            self.robot.stop()
            return

        self.robot.left_motor.value = left_speed
        self.robot.right_motor.value = right_speed
        # self.get_logger().info(f"Left Speed: {left_speed}, Right Speed: {right_speed}")


def main(args=None):
    rclpy.init(args=args)

    left = (20, 21)
    right = (27, 22)
    movement_subscriber = MovementSubscriber(left=left, right=right)
    rclpy.spin(movement_subscriber)
    movement_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
