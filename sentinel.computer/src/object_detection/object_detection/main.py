import rclpy
from .subscriber import YoloSubscriber


def main(args=None):
    rclpy.init(args=args)
    
    node = YoloSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()