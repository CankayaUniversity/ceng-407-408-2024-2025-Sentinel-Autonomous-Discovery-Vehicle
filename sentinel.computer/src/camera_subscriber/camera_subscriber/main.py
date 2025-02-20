import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.subscription = self.create_subscription(CompressedImage, "raspicam/compressed", self.listener_callback, 10)
        self.get_logger().info("Camera subscriber")

    def listener_callback(self, msg):
        try:
            nparr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            cv2.imshow("Video", image)
            
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return

            self.get_logger().info("Frame received and displayed")

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()