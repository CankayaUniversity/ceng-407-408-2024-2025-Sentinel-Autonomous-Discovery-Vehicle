import cv2
import numpy as np
import json
from datetime import datetime
from uuid import uuid4
import uuid
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from .model import YoloModel
from .publisher import ObcejtDetectionFramePublisher
from .notification_msg import NotificationMsg
from .minio_uploader import MinioUploader
from .id import UNIQUE_ID
from std_msgs.msg import String

class YoloSubscriber(Node):
    def __init__(self):
        super().__init__("yolo_ros2_subscriber")
        self.subscription = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10)
        self.detected_objects = []
        self.bridge = CvBridge()
        self.model = YoloModel(model="yolov8x.pt")
        self.frame_publisher = ObcejtDetectionFramePublisher()
        self.notification_publisher = self.create_publisher(String, "/notifications", 10)
        self.minio = MinioUploader()

        self.publish_notification("Object Detection Started.")
        
    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            results = self.model.process_video_stream(frame)
            while self.model.object_count > self.model.last_object_idx:
                self.get_logger().info(f"{self.model.known_objects[self.model.last_object_idx][3]}")
                id = self.publish_notification(f"New object detected {self.model.known_objects[self.model.last_object_idx][3]}.")
                self.upload_image(self.model.known_objects[self.model.last_object_idx][2], self.model.known_objects[self.model.last_object_idx][3], id)
                self.model.last_object_idx +=1

            success, buffer = cv2.imencode('.jpg', results)
            if not success:
                self.get_logger().error("Failed to encode annotated frame.")
                return
            
            image_bytes = buffer.tobytes()
            self.frame_publisher.publish(image_bytes)
                
        except Exception as e:
            error_msg = f"Object Detection Failed: {str(e)}"
            self.get_logger().error(error_msg)

    def publish_notification(self, data, msg_type="INFO"):
        try:
            notification = NotificationMsg.create(data, msg_type)
            json_msg = json.dumps(notification.__dict__)
            
            # Publish on topic
            msg = String()
            msg.data = json_msg
            self.notification_publisher.publish(msg)


            # self.get_logger().info(f"Published notification: {data}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish notification: {str(e)}")
        return notification.id

    def upload_image(self, cropped_object, class_name, id):
        try:

            # Encode cropped image as JPEG
            success, buffer = cv2.imencode('.jpg', cropped_object)
            if not success:
                self.get_logger().error("Failed to encode cropped object image.")
                return

            image_bytes = buffer.tobytes()

            # Generate a unique filename
            filename = f"objects/{class_name}_{id}.jpg"
            # Upload to MinIO
            self.minio.upload_bytes(
                data_bytes=image_bytes,
                bucket_name=f"{UNIQUE_ID}-ros2-bucket",
                object_name=filename,
                content_type="image/jpeg"
            )


        except Exception as e:
            self.get_logger().error(f"Failed to upload object image: {str(e)}")
