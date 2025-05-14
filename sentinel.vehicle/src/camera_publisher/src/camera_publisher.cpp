#include <iostream>
#include "camera/camera_publisher.hpp"
#include "camera/constants.hpp"

CameraPublisher::CameraPublisher() : Node{"camera_publisher"}
{
    this->declare_udp_params();
    this->declare_camera_params();
    this->socket = this->create_socket();
}

CameraPublisher::~CameraPublisher()
{
    if (this->socket)
    {
        delete this->socket;
    }
}

void CameraPublisher::completed(void *frame, size_t frame_size, int id)
{
    this->socket->send(frame, frame_size, id);
    RCLCPP_DEBUG(this->get_logger(), "Camera Frame Send");
}

CameraConfiguration CameraPublisher::camera_configuration()
{
    int width = this->get_parameter("camera_width").as_int();
    int height = this->get_parameter("camera_height").as_int();
    int fps = this->get_parameter("camera_fps").as_int();
    std::string encoding = this->get_parameter("camera_encoding").as_string();
#ifdef LIBCAMERA
    return CameraConfiguration(width, height, fps, encoding, Constants::camera_pixel_format);
#else
    return CameraConfiguration(width, height, fps, encoding);
#endif
}

void CameraPublisher::declare_udp_params()
{
    this->declare_parameter<std::string>("vehicle_host", std::string(Constants::vehicle_host));
    this->declare_parameter<int>("vehicle_port", Constants::vehicle_port);

    this->declare_parameter<std::string>("computer_host", std::string("192.168.171.94"));
    this->declare_parameter<int>("computer_port", Constants::computer_port);

    this->declare_parameter<int>("chunk_size", Constants::chunk_size);
}

void CameraPublisher::declare_camera_params()
{
    this->declare_parameter<int>("camera_width", Constants::camera_width);
    this->declare_parameter<int>("camera_height", Constants::camera_heigh);
    this->declare_parameter<int>("camera_fps", Constants::camera_fps);
    this->declare_parameter<std::string>("camera_encoding", Constants::camera_encoding);
}

UdpSocket *CameraPublisher::create_socket()
{
    std::string vehicle_host = this->get_parameter("vehicle_host").as_string();
    int vehicle_port = this->get_parameter("vehicle_port").as_int();

    std::string computer_host = this->get_parameter("computer_host").as_string();
    int computer_port = this->get_parameter("computer_port").as_int();
    int chunk_size = this->get_parameter("chunk_size").as_int();

    if (computer_host.empty())
    {
        throw std::runtime_error("The 'computer_host' parameter is required.");
    }
    return new UdpSocket(vehicle_host, vehicle_port, computer_host, computer_port, chunk_size);
}
