#include <iostream>
#include <chrono>
#include <thread>
#include <time.h>

#include "camera.hpp"
#include "image_publisher.hpp"

int main(int argc, const char *argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    std::shared_ptr<ImagePublisher> publisher = std::make_shared<ImagePublisher>();

    Camera camera;
    camera.attach(publisher);
    camera.start();

    while (true)
    {
      rclcpp::spin_some(publisher);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }

  rclcpp::shutdown();
  return 0;
}
