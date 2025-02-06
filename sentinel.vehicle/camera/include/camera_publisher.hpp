#pragma once

#include "udp_socket.hpp"
#include "observer.hpp"

class CameraPublisher : public CameraRequestObserver
{
public:
  using SharedPtr = std::shared_ptr<CameraPublisher>;

public:
  CameraPublisher(const std::string &source_host, int source_port, const std::string &destination_host, int destination_port, int chunk_size);
  ~CameraPublisher();

  void completed(void *frame, size_t frame_size, int id) override;

private:
  UdpSocket socket;
};
