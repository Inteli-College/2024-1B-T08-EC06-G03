#include "bolin_camera_cpp/bolin_camera_cpp.hpp"

CameraPublisher::CameraPublisher() : Node("bolin_camera_cpp"), cap_(0) {
  publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera_feed", 10);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  encode_param_ = {cv::IMWRITE_JPEG_QUALITY, 90};
}

void CameraPublisher::spin() {
  rclcpp::Rate loop_rate(30);
  while (rclcpp::ok()) {
    auto time_before = this->get_clock()->now();
    cv::Mat frame;
    if (cap_.read(frame)) {
      std::vector<uchar> buffer;
      cv::imencode(".jpg", frame, buffer, encode_param_);

      auto msg = sensor_msgs::msg::CompressedImage();
      msg.header.stamp = this->get_clock()->now();
      msg.format = "jpeg";
      msg.data = std::move(buffer);

      publisher_->publish(msg);
      auto time_after = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(), "Latency: %.2f ms",
                  (time_after - time_before).nanoseconds() / 1e6);
    }
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}
