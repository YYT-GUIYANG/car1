// 订阅相机原始图话题，单独 OpenCV 窗口预览（与 trace_calculator 的「Received Image」形成双画面）
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ProcessedImagePreviewNode : public rclcpp::Node
{
public:
  ProcessedImagePreviewNode()
  : Node("processed_image_preview")
  {
    topic_ = this->declare_parameter<std::string>("image_topic", "/processed_image");
    window_name_ = this->declare_parameter<std::string>(
      "window_name", "Camera /processed_image (raw)");
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_,
      rclcpp::QoS(5).best_effort(),
      std::bind(&ProcessedImagePreviewNode::onImage, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(), "订阅 %s ；预览窗口标题：%s",
      topic_.c_str(), window_name_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (gui_broken_) {
      return;
    }
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      if (cv_ptr->image.empty()) {
        return;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "cv_bridge: %s", e.what());
      return;
    }
    try {
      cv::imshow(window_name_, cv_ptr->image);
      cv::waitKey(1);
    } catch (const cv::Exception & e) {
      if (!gui_broken_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "OpenCV 预览窗口失败（无 DISPLAY 或 GTK 不可用）：%s — 之后不再弹此窗",
          e.what());
        gui_broken_ = true;
        try {
          cv::destroyWindow(window_name_);
        } catch (...) {
        }
      }
    }
  }

  std::string topic_;
  std::string window_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  bool gui_broken_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessedImagePreviewNode>());
  rclcpp::shutdown();
  return 0;
}
