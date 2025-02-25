#pragma once

#include <rclcpp/rclcpp.hpp>
#include "udp_socket.hpp"
#include "observer.hpp"
#include "camera/camera.hpp"

class CameraPublisher : public CameraRequestObserver, public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<CameraPublisher>;

public:
  CameraPublisher();
  ~CameraPublisher();

  void completed(void *frame, size_t frame_size, int id) override;
  CameraConfiguration camera_configuration();

private:
  void declare_udp_params();
  void declare_camera_params();
  UdpSocket *create_socket();

private:
  UdpSocket *socket;
};
