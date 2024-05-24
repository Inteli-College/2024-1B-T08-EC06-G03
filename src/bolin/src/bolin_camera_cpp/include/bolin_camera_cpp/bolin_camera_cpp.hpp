#ifndef BOLIN_CAMERA_CPP_HPP_
#define BOLIN_CAMERA_CPP_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher();
  void spin();

private:
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  cv::VideoCapture cap_;
  std::vector<int> encode_param_;
};

#endif // BOLIN_CAMERA_CPP_HPP_
