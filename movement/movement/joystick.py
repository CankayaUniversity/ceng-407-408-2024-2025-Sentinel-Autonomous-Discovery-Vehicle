import pygame
import math
import os
from rclpy import logging
from typing import Tuple, Optional


class Joystick:
    def __init__(self):
        os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick connected!")

        self.logger = logging.get_logger("movement")
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.is_sent = False

        self.motor_rpm = 200
        self.wheel_radius = 0.0325

        self.max_wheel_angular_speed = (self.motor_rpm / 60.0) * 2 * math.pi  # rad/s
        self.max_linear_speed = (
            self.wheel_radius * self.max_wheel_angular_speed / 2
        )  # m/s

        self.logger.info(f"Maximum Linear Speed:{self.max_linear_speed}")

    def __del__(self):
        self.joystick.quit()
        pygame.joystick.quit()
        pygame.quit()

    def event(self) -> Optional[Tuple[float, float]]:
        pygame.event.pump()
        forward_axis = -self.joystick.get_axis(1)
        turn_axis = -self.joystick.get_axis(0)
        throttle = self.joystick.get_axis(5)
        scale = (throttle + 1.0) / 2.0

        if not self.check_is_pressed(forward_axis, throttle):
            if not self.is_sent:
                self.is_sent = True
                return 0.0, 0.0
            return None

        self.is_sent = False

        linear_speed = forward_axis * scale * self.max_linear_speed
        angular_speed = turn_axis * scale * self.max_linear_speed

        return linear_speed, angular_speed

    def check_is_pressed(self, linear, throttle, threshold=0.2) -> bool:
        if throttle < threshold:
            return False
        if abs(linear) < threshold * 0.01:
            return False
        return True
