import base64
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(CompressedImage,'raspicam',10)
        self.get_logger().info("Camera Publisher Started")
        self.image = bytearray()
        self.current_id = 0

    def publish(self):
        image = CompressedImage()
        image.data = bytes(self.image)
        image.format = "jpeg"
        self.publisher.publish(image)
        self.get_logger().info("Image Published")

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