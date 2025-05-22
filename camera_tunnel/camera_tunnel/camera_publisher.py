import base64
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import numpy as np
import cv2

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.compressed_publisher = self.create_publisher(CompressedImage,'raspicam/compressed',10)
        self.raw_publisher = self.create_publisher(Image, 'raspicam/raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'raspicam/camera_info', 10)
        self.get_logger().info("Camera Publisher Started")
        self.image = bytearray()
        self.current_id = 0
        self.clock = self.get_clock()

        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = "camera_link"
        self.camera_info.height = 480
        self.camera_info.width = 640
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        self.camera_info.p = self.camera_info.k

    def publish(self):
        if not self.image:
            self.get_logger().warn("Empty Image")
            return
        
        compressed_image = CompressedImage()
        raw_image = Image()

        time = self.clock.now().to_msg()

        compressed_image.header.frame_id = 'camera_link'
        compressed_image.header.stamp = time

        raw_image.header.stamp = time
        raw_image.header.frame_id = 'camera_link'
        compressed_image.data = bytes(self.image)
        compressed_image.format = "jpeg"

        raw_image.data = bytes(self.image)
        nparr = np.frombuffer(raw_image.data, np.uint8)
        decoded_image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if hasattr(decoded_image, 'shape') == False:
            self.get_logger().warn("Image can not be decoded")
            return
        
        decoded_image = cv2.cvtColor(decoded_image, cv2.COLOR_BGR2RGB)
        
        raw_image.height, raw_image.width, _ = decoded_image.shape
        raw_image.encoding = "rgb8"
        raw_image.step = raw_image.width * 3
        raw_image.data = decoded_image.tobytes()

        self.camera_info.header.stamp = time
        self.camera_info.height = raw_image.height
        self.camera_info.width = raw_image.width

        self.compressed_publisher.publish(compressed_image)
        self.raw_publisher.publish(raw_image)
        self.camera_info_publisher.publish(self.camera_info)
        
        self.get_logger().debug(f"Image Published")

    def configure_params(self):
        self.image = bytearray() 
        self.current_size = 0

    def handle_frame(self, json_data):
        frame_id = json_data["frame_id"]
        temp_frame = json_data["data"]

        if frame_id == self.current_id:
            self.image.extend(base64.b64decode(temp_frame))
        else:
            self.publish()
            self.configure_params()
            self.current_id = frame_id
            self.image.extend(base64.b64decode(temp_frame))