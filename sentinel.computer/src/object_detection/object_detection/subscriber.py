import cv2
import numpy as np
import json
import uuid
from datetime import datetime
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .model import YoloModel
from .notification_msg import NotificationMsg


class YoloSubscriber(Node):
    def __init__(self):
        self.detected_objects = []
        super().__init__("yolo_ros2_subscriber")
        self.subscription = self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10
        )
        self.notification_publisher = self.create_publisher(
            String, "/notifications", 10
        )        
        self.bridge = CvBridge()
        self.model = YoloModel(model="yolo11x.pt")        
        self.publish_notification("Object Detection Started.")
        
    def publish_notification(self, data, msg_type="INFO"):
        try:
            notification = NotificationMsg.create(data, msg_type)
            json_msg = json.dumps(notification.__dict__)    
            msg = String()
            msg.data = json_msg
        
            self.notification_publisher.publish(msg)
            self.get_logger().info(f"Published notification: {json_msg}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish notification: {str(e)}")

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            results = self.model(frame, verbose=False)

            boxes = results[0].boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                class_name = results[0].names[cls_id]
                if class_name not in self.detected_objects:
                    self.detected_objects.append(class_name)
                    print(f"Detected class: {class_name}")
                    self.publish_notification(f"New object detected: {class_name}")

            annotated_frame = results[0].plot()

            cv2.imshow("YOLOv8 ROS2 Live Detection", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            error_msg = f"Object Detection Failed: {str(e)}"
            self.get_logger().error(error_msg)
            self.publish_notification(error_msg, "ERROR")
    
    def destroy_node(self):
        self.publish_notification("Object Detection Stopped")
        super().destroy_node()