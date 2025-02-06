#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

#include "camera_publisher.hpp"
#include "camera.hpp"

#define SLEEP_DURATION 1000000000000
#define CHUNK_SIZE 60000

Camera *camera;
void quit_handler(int signal);

int main(int argc, char const *argv[])
{
  std::signal(SIGINT | SIGKILL, quit_handler);
  try
  {
    std::string vehicle_host = "0.0.0.0";
    int vehicle_port = 8000;

    std::string computer_host = "192.168.93.94";
    int computer_port = 9000;

    CameraPublisher::SharedPtr publisher = std::make_shared<CameraPublisher>(vehicle_host, vehicle_port, computer_host, computer_port, CHUNK_SIZE);
    CameraConfiguration camera_configuration = {
        .width = 640,
        .height = 480,
        .fps = 30,
        .pixel_format = libcamera::formats::RGB888,
        .encoding = ".jpeg",
    };

    camera = new Camera{camera_configuration};
    camera->attach(publisher);
    camera->start();
    while (true)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_DURATION));
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    if (camera)
    {
      delete camera;
    }
  }
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