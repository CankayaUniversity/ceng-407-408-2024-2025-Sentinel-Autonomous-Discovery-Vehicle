import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from .model import YoloModel


class YoloSubscriber(Node):
    def __init__(self):
        self.detected_objects = []
        super().__init__("yolo_ros2_subscriber")
        self.subscription = self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10
        )
        self.bridge = CvBridge()
        self.model = YoloModel(model="yolo11x.pt")

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = self.model(frame, verbose = False)

        boxes = results[0].boxes
        for box in boxes:
            cls_id = int(box.cls[0])
            class_name = results[0].names[cls_id]
            if class_name not in self.detected_objects:
                self.detected_objects.append(class_name)
                print(f"Detected class: {class_name}")

        annotated_frame = results[0].plot()

        cv2.imshow("YOLOv8 ROS2 Live Detection", annotated_frame)
        cv2.waitKey(1)