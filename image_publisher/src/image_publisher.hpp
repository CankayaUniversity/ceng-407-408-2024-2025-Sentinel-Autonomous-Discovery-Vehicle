#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "observer.hpp"

class ImagePublisher : public rclcpp::Node, public RequestObserver
{
public:
  ImagePublisher();
  ~ImagePublisher();
  void onCompleted(void *mem, size_t byteused) override;

private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
};