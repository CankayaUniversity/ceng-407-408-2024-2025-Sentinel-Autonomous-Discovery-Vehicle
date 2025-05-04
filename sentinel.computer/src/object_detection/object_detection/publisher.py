from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class ObcejtDetectionFramePublisher(Node):

    def __init__(self):
        super().__init__('object_detection_frame_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'object_detection_frame', 10)
        self.i = 0

    def publish(self, image):
        msg = CompressedImage()
        msg.format = "jpg"  # or "png", depending on your image encoding
        msg.data = image  # Make sure this is a bytes object, e.g., from cv2.imencode()[1].tobytes()

        self.publisher_.publish(msg)