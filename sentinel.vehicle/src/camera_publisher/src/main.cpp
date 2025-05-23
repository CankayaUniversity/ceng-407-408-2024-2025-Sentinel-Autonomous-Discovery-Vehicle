#include <iostream>
#include <thread>
#include <csignal>
#include <rclcpp/rclcpp.hpp>

#include "camera/camera_publisher.hpp"
#include "camera/camera.hpp"

Camera *camera;
void quit_handler(int signal);

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT | SIGKILL, quit_handler);
  try
  {
    CameraPublisher::SharedPtr publisher = std::make_shared<CameraPublisher>();
    CameraConfiguration camera_configuration = publisher->camera_configuration();

    camera = new Camera{camera_configuration};
    camera->attach(publisher);
    camera->start();

    while (true)
    {
      std::this_thread::sleep_for(std::chrono::hours::max());
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    quit_handler(SIGKILL);
  }
  rclcpp::shutdown();
  return 0;
}

void quit_handler(int signal)
{
  UdpSocket::quit_handler(signal);
  if (camera)
  {
    delete camera;
  }
}
