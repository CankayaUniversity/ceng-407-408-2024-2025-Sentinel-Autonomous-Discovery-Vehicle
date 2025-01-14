import rclpy
from .udp_socket import UdpSocket
from .camera_publisher import CameraPublisher


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()
    socket = UdpSocket(computer_host="0.0.0.0", computer_port=9000)

    try:
        while True:
            data = socket.receive()
            camera_publisher.publish(data)
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        socket.close()
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
