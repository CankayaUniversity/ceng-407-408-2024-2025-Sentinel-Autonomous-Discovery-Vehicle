#include "image_publisher.hpp"
#include <iostream>

ImagePublisher::ImagePublisher() : rclcpp::Node("image_publisher")
{
  this->publisher = this->create_publisher<sensor_msgs::msg::Image>("image_publisher", 10);
}

ImagePublisher::~ImagePublisher()
{
}

void ImagePublisher::onCompleted(void *mem, size_t byteused)
{
  sensor_msgs::msg::Image::SharedPtr ros_image = std::make_shared<sensor_msgs::msg::Image>();

  ros_image->header.frame_id = "camera_frame";
  ros_image->width = 640;
  ros_image->height = 480;
  ros_image->encoding = "rgb8";
  ros_image->step = 640 * 3;

  ros_image->data.resize(480 * ros_image->step);

  memcpy(ros_image->data.data(), mem, byteused);

  this->publisher->publish(*ros_image);
  RCLCPP_INFO(this->get_logger(), "Image published");
  free(mem);
}
