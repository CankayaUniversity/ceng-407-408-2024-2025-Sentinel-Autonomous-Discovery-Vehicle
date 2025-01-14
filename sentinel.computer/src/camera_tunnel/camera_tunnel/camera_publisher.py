from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image,'camera',10)
        self.get_logger().info("Camera Publisher Started")
        
    def publish(self,data):
        image = Image()
        image.data = data
        image.width = 640
        image.height = 480
        image.step = 640 * 3
        image.encoding = "jpeg"

        self.publisher.publish(image)
        self.get_logger().info("Image Published")


    

    
