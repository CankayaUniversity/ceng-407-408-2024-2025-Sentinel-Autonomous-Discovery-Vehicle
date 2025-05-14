import rclpy
from .server import PathFinder

def main(args=None):
    rclpy.init(args=args)

    try:
        path_finder = PathFinder()
        rclpy.spin(path_finder)
    except KeyboardInterrupt:
        pass
    finally:
        path_finder.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()