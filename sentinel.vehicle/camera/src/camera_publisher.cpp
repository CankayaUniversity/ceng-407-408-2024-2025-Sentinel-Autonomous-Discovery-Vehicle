#include <iostream>
#include "camera_publisher.hpp"

CameraPublisher::CameraPublisher(const std::string &source_host, int source_port, const std::string &destination_host, int destination_port)
    : socket(source_host, source_port, destination_host, destination_port)
{
}

CameraPublisher::~CameraPublisher()
{
}

void CameraPublisher::completed(void *frame, size_t frame_size)
{
    this->socket.send(frame, frame_size);
    std::cout << "Camera frame sent\n";
}
