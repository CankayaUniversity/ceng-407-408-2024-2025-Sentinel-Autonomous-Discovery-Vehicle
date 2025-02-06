#include <iostream>
#include "camera_publisher.hpp"

CameraPublisher::CameraPublisher(const std::string &source_host, int source_port, const std::string &destination_host, int destination_port, int chunk_size)
    : socket(source_host, source_port, destination_host, destination_port, chunk_size)
{
}

CameraPublisher::~CameraPublisher()
{
}

void CameraPublisher::completed(void *frame, size_t frame_size, int id)
{
    this->socket.send(frame, frame_size, id);
    std::cout << "Camera frame sent\n";
}
