#include <iostream>
#include <thread>
#include <chrono>

#include "camera_publisher.hpp"
#include "camera.hpp"

int main(int argc, char const *argv[])
{
  try
  {
    std::string vehicle_host = "0.0.0.0";
    int vehicle_port = 8000;

    std::string computer_host = "192.168.1.20";
    int computer_port = 9000;

    CameraPublisher::SharedPtr publisher = std::make_shared<CameraPublisher>(vehicle_host, vehicle_port, computer_host, computer_port);
    CameraConfiguration camera_configuration = {
        .width = 640,
        .height = 480,
        .fps = 24,
        .encoding = ".jpeg",
    };

    Camera camera{camera_configuration};
    camera.attach(publisher);
    camera.start();
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }
  return 0;
}
