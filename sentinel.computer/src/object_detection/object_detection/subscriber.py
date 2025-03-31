import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from .model import YoloModel


class YoloSubscriber(Node):
    def __init__(self):
        super().__init__("yolo_ros2_subscriber")
        self.subscription = self.create_subscription(
            CompressedImage, "raspicam/compressed", self.image_callback, 10
        )
        self.bridge = CvBridge()
        self.model = YoloModel()

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = self.model(frame)
        annotated_frame = results[0].plot()

        cv2.imshow("YOLOv8 ROS2 Live Detection", annotated_frame)
        cv2.waitKey(1)
