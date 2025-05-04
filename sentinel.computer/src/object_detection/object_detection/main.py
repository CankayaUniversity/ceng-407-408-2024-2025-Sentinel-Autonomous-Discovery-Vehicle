# pip install ultralytics opencv-python torchvision
import rclpy
from .subscriber import YoloSubscriber
from .get_object_subscriber import GetObjectSubscriber
import numpy as np
from rclpy.executors import MultiThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    node1 = YoloSubscriber()
    node2 = GetObjectSubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
