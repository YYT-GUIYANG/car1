#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {
int clamp_int(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }
using RefColor = std::pair<int, cv::Vec3b>;  // id, bgr
const std::vector<RefColor>& reference_bgr_table() {
  static const std::vector<RefColor> kRef = {
    {0, cv::Vec3b(116, 5, 202)},   {3, cv::Vec3b(167, 1, 98)},   {4, cv::Vec3b(7, 237, 19)},
    {5, cv::Vec3b(23, 51, 215)},   {6, cv::Vec3b(241, 132, 251)}, {7, cv::Vec3b(17, 168, 214)},
    {8, cv::Vec3b(135, 199, 246)}, {9, cv::Vec3b(221, 66, 76)},   {10, cv::Vec3b(216, 199, 167)},
    {11, cv::Vec3b(85, 152, 55)},  {12, cv::Vec3b(57, 244, 231)}, {16, cv::Vec3b(23, 3, 23)},
  };
  return kRef;
}
}  // namespace

class SimpleBlobVisionNode final : public rclcpp::Node {
public:
  SimpleBlobVisionNode() : Node("simple_blob_vision") {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/processed_image");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/blob_xy");
    publish_debug_mask_ = this->declare_parameter<bool>("publish_debug_mask", false);
    debug_mask_topic_ = this->declare_parameter<std::string>("debug_mask_topic", "/blob_mask");
    publish_debug_overlay_ = this->declare_parameter<bool>("publish_debug_overlay", true);
    debug_overlay_topic_ = this->declare_parameter<std::string>("debug_overlay_topic", "/simple_track/debug_image");

    // 只保留方块追踪：默认 HSV 色块（颜色 id 参照 调试参数/offline_trace.py）
    vision_mode_ = this->declare_parameter<std::string>("vision_mode", "hsv_blob");
    // 默认红色：id=5（见 offline_trace REFERENCE_BGR）
    hsv_color_id_ = clamp_int(this->declare_parameter<int>("hsv_color_id", 5), 0, 16);
    hsv_h_min_ = clamp_int(this->declare_parameter<int>("hsv_h_min", 0), 0, 180);
    hsv_h_max_ = clamp_int(this->declare_parameter<int>("hsv_h_max", 180), 0, 180);
    hsv_s_min_ = clamp_int(this->declare_parameter<int>("hsv_s_min", 80), 0, 255);
    hsv_s_max_ = clamp_int(this->declare_parameter<int>("hsv_s_max", 255), 0, 255);
    hsv_v_min_ = clamp_int(this->declare_parameter<int>("hsv_v_min", 80), 0, 255);
    hsv_v_max_ = clamp_int(this->declare_parameter<int>("hsv_v_max", 255), 0, 255);
    blob_morph_open_ = clamp_int(this->declare_parameter<int>("blob_morph_open", 3), 0, 31);  // 0=关闭
    blob_morph_close_ = clamp_int(this->declare_parameter<int>("blob_morph_close", 5), 0, 31);
    blob_square_aspect_max_ =
      std::clamp(this->declare_parameter<double>("blob_square_aspect_max", 1.45), 1.0, 5.0);
    hsv_id_dist_max_ = std::max(10.0, this->declare_parameter<double>("hsv_id_dist_max", 5200.0));

    // 纯圆检测：不依赖颜色阈值，直接灰度 + Hough。
    hough_blur_ksize_ = clamp_int(this->declare_parameter<int>("hough_blur_ksize", 9), 3, 41) | 1;
    hough_dp_ = std::clamp(this->declare_parameter<double>("hough_dp", 1.2), 1.0, 3.0);
    hough_min_dist_ratio_ = std::clamp(this->declare_parameter<double>("hough_min_dist_ratio", 0.10), 0.01, 0.90);
    hough_param1_ = std::clamp(this->declare_parameter<double>("hough_param1", 120.0), 1.0, 500.0);
    hough_param2_ = std::clamp(this->declare_parameter<double>("hough_param2", 22.0), 1.0, 300.0);

    circle_min_radius_px_ = std::max(1.0, this->declare_parameter<double>("circle_min_radius_px", 8.0));
    circle_max_radius_ratio_ = std::clamp(this->declare_parameter<double>("circle_max_radius_ratio", 0.45), 0.02, 0.95);
    circle_min_area_ = std::max(1.0, this->declare_parameter<double>("circle_min_area", 120.0));
    circle_max_area_ratio_ = std::clamp(this->declare_parameter<double>("circle_max_area_ratio", 0.80), 0.01, 0.99);

    prefer_center_ = this->declare_parameter<bool>("prefer_center", true);
    center_bias_ = std::clamp(this->declare_parameter<double>("center_bias", 0.002), 0.0, 0.02);
    circle_prefer_last_enable_ = this->declare_parameter<bool>("circle_prefer_last_enable", true);
    circle_prefer_last_bias_ = std::clamp(this->declare_parameter<double>("circle_prefer_last_bias", 0.003), 0.0, 0.05);
    circle_hold_frames_ = std::clamp(static_cast<int>(this->declare_parameter<int>("circle_hold_frames", 2)), 0, 20);

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, rclcpp::QoS(10));
    if (publish_debug_mask_) {
      debug_mask_pub_ = image_transport::create_publisher(this, debug_mask_topic_);
    }
    if (publish_debug_overlay_) {
      overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_overlay_topic_, rclcpp::QoS(2));
    }

    sub_ = image_transport::create_subscription(
      this,
      input_topic_,
      std::bind(&SimpleBlobVisionNode::on_image, this, std::placeholders::_1),
      "raw",
      rclcpp::QoS(1).get_rmw_qos_profile());

    RCLCPP_INFO(
      this->get_logger(),
      "[simple_blob_vision] mode=%s in=%s blob=%s overlay=%s(%s) mask=%s hsv_color_id=%d",
      "hsv_blob",
      input_topic_.c_str(),
      output_topic_.c_str(),
      publish_debug_overlay_ ? "on" : "off",
      debug_overlay_topic_.c_str(),
      publish_debug_mask_ ? "on" : "off",
      hsv_color_id_);

    // 预计算参考色的 HSV（与 offline_trace REFERENCE_BGR 对齐）
    for (const auto& kv : reference_bgr_table()) {
      const cv::Mat3b bgr_px(1, 1, kv.second);
      cv::Mat3b hsv_px;
      cv::cvtColor(bgr_px, hsv_px, cv::COLOR_BGR2HSV);
      const auto& h = hsv_px(0, 0);
      ref_hsv_.push_back({kv.first, cv::Vec3d(static_cast<double>(h[0]), static_cast<double>(h[1]), static_cast<double>(h[2]))});
    }
  }

private:
  bool hsv_mode() {
    if (!(vision_mode_ == "hsv_blob" || vision_mode_ == "color_blob" || vision_mode_ == "hsv")) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "vision_mode=%s 已忽略；当前仅支持 hsv_blob 方块追踪",
                           vision_mode_.c_str());
    }
    return true;
  }

  void hsv_color_mask(const cv::Mat& hsv, cv::Mat& mask) {
    if (hsv_color_id_ == 0) {
      cv::inRange(
        hsv,
        cv::Scalar(hsv_h_min_, hsv_s_min_, hsv_v_min_),
        cv::Scalar(hsv_h_max_, hsv_s_max_, hsv_v_max_),
        mask);
      return;
    }
    // 颜色 ID 模式：先做宽松高饱和度前景分割，再按轮廓均值 HSV 与 reference id 精确匹配
    cv::inRange(hsv, cv::Scalar(0, 40, 35), cv::Scalar(180, 255, 255), mask);
  }

  static double hsv_dist_sq(const cv::Vec3d& a, const cv::Vec3d& b) {
    // OpenCV H:0..180（等价 0..360 度的一半）
    const double dh_raw = std::abs(a[0] - b[0]);
    const double dh = std::min(dh_raw, 180.0 - dh_raw) * 2.0;
    const double ds = a[1] - b[1];
    const double dv = a[2] - b[2];
    return dh * dh + ds * ds + dv * dv;
  }

  int classify_hsv_id(const cv::Vec3d& hsv_mean) const {
    int best_id = -1;
    double best_d = 1e18;
    for (const auto& kv : ref_hsv_) {
      const double d = hsv_dist_sq(hsv_mean, kv.second);
      if (d < best_d) {
        best_d = d;
        best_id = kv.first;
      }
    }
    if (best_d > hsv_id_dist_max_) return -1;
    return best_id;
  }

  static int odd_kernel(int v) {
    if (v <= 0) return 0;
    int k = v | 1;
    return k < 3 ? 3 : k;
  }

  void morph_mask(cv::Mat& mask) const {
    const int ko = odd_kernel(blob_morph_open_);
    if (ko > 0) {
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                       cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ko, ko)));
    }
    const int kc = odd_kernel(blob_morph_close_);
    if (kc > 0) {
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                       cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kc, kc)));
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if (!msg) return;
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN(this->get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    const cv::Mat& bgr = cv_ptr->image;
    if (bgr.empty()) return;

    const double frame_area = static_cast<double>(bgr.cols) * static_cast<double>(bgr.rows);
    const cv::Point2f opt(static_cast<float>(bgr.cols) * 0.5f, static_cast<float>(bgr.rows) * 0.5f);

    bool ok = false;
    bool detected_this_frame = false;
    double best_score = -1e18;
    cv::Point2f best_c(0.f, 0.f);
    double best_area = 0.0;
    float best_radius = 0.0f;
    cv::Rect best_bbox;

    (void)hsv_mode();
    run_hsv_blob_(bgr, opt, frame_area, ok, best_score, best_c, best_area, best_radius, best_bbox);
    detected_this_frame = ok;

    if (!ok && has_last_circle_ && lost_hold_count_ < circle_hold_frames_) {
      ok = true;
      best_c = last_circle_center_;
      best_radius = last_circle_radius_;
      best_area = CV_PI * static_cast<double>(best_radius) * static_cast<double>(best_radius);
      lost_hold_count_ += 1;
    }
    if (ok) {
      if (detected_this_frame) {
        lost_hold_count_ = 0;
      }
      last_circle_center_ = best_c;
      last_circle_radius_ = best_radius;
      has_last_circle_ = true;
      if (detected_this_frame) {
        last_blob_bbox_ = best_bbox;
      }
    } else {
      lost_hold_count_ = std::min(lost_hold_count_ + 1, circle_hold_frames_);
    }

    std_msgs::msg::Float32MultiArray out;
    out.data.resize(6);
    out.data[0] = ok ? 1.0f : 0.0f;
    out.data[1] = ok ? best_c.x : 0.0f;
    out.data[2] = ok ? best_c.y : 0.0f;
    out.data[3] = ok ? static_cast<float>(best_area) : 0.0f;
    out.data[4] = static_cast<float>(bgr.cols);
    out.data[5] = static_cast<float>(bgr.rows);
    pub_->publish(out);

    if (publish_debug_mask_ && debug_mask_pub_) {
      cv_bridge::CvImage dbg;
      dbg.encoding = "mono8";
      if (!last_debug_mask_.empty()) {
        dbg.image = last_debug_mask_;
      } else {
        cv::Mat gray;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        dbg.image = gray;
      }
      dbg.header = msg->header;
      debug_mask_pub_.publish(dbg.toImageMsg());
    }

    if (publish_debug_overlay_ && overlay_pub_) {
      cv::Mat vis = bgr.clone();
      const cv::Point center_i(static_cast<int>(std::lround(opt.x)), static_cast<int>(std::lround(opt.y)));
      cv::drawMarker(vis, center_i, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 24, 2, cv::LINE_AA);
      if (ok) {
        const cv::Point2i bc(static_cast<int>(std::lround(best_c.x)), static_cast<int>(std::lround(best_c.y)));
        cv::line(vis, center_i, bc, cv::Scalar(255, 128, 0), 1, cv::LINE_AA);
        char buf[160];
        const cv::Rect& r = last_blob_bbox_;
        if (r.area() > 0) {
          cv::rectangle(vis, r, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
        }
        cv::circle(vis, bc, 8, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        const double asp =
          r.width > 0 && r.height > 0
            ? static_cast<double>(std::max(r.width, r.height)) /
                static_cast<double>(std::max(1, std::min(r.width, r.height)))
            : 0.0;
        std::snprintf(
          buf,
          sizeof(buf),
          "HSV square id=%d area=%.0f asp=%.2f cx=%.0f cy=%.0f",
          hsv_color_id_,
          best_area,
          asp,
          best_c.x,
          best_c.y);
        cv::putText(vis, buf, cv::Point(8, 28), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      } else {
        const char* hint = "NO SQUARE (hsv/morph/area/aspect)";
        cv::putText(
          vis,
          hint,
          cv::Point(8, 28),
          cv::FONT_HERSHEY_SIMPLEX,
          0.65,
          cv::Scalar(0, 0, 255),
          2,
          cv::LINE_AA);
      }
      cv_bridge::CvImage out_img;
      out_img.header = msg->header;
      out_img.encoding = "bgr8";
      out_img.image = vis;
      overlay_pub_->publish(*out_img.toImageMsg());
    }
  }

  void run_hough_(
    const cv::Mat& bgr,
    const cv::Point2f& opt,
    double frame_area,
    bool& ok,
    double& best_score,
    cv::Point2f& best_c,
    double& best_area,
    float& best_radius) {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(hough_blur_ksize_, hough_blur_ksize_), 0.0);

    const double a_cap = circle_max_area_ratio_ * frame_area;
    const float max_radius_px =
      static_cast<float>(std::max(2.0, circle_max_radius_ratio_ * std::min(bgr.cols, bgr.rows)));
    const double min_dist_px = std::max(8.0, hough_min_dist_ratio_ * std::min(bgr.cols, bgr.rows));

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(
      gray,
      circles,
      cv::HOUGH_GRADIENT,
      hough_dp_,
      min_dist_px,
      hough_param1_,
      hough_param2_,
      static_cast<int>(std::lround(circle_min_radius_px_)),
      static_cast<int>(std::lround(max_radius_px)));

    for (const auto& c : circles) {
      const float cx = c[0];
      const float cy = c[1];
      const float r = c[2];
      if (r < static_cast<float>(circle_min_radius_px_) || r > max_radius_px) continue;
      const double area = CV_PI * static_cast<double>(r) * static_cast<double>(r);
      if (area < circle_min_area_ || area > a_cap) continue;

      const double dist = std::hypot(static_cast<double>(cx) - opt.x, static_cast<double>(cy) - opt.y);
      double score = area;
      if (prefer_center_) {
        score -= center_bias_ * dist * dist;
      }
      if (circle_prefer_last_enable_ && has_last_circle_) {
        const double d_last =
          std::hypot(static_cast<double>(cx - last_circle_center_.x), static_cast<double>(cy - last_circle_center_.y));
        score -= circle_prefer_last_bias_ * d_last * d_last;
      }
      if (score > best_score) {
        best_score = score;
        best_c = cv::Point2f(cx, cy);
        best_area = area;
        best_radius = r;
        ok = true;
      }
    }
  }

  void run_hsv_blob_(
    const cv::Mat& bgr,
    const cv::Point2f& opt,
    double frame_area,
    bool& ok,
    double& best_score,
    cv::Point2f& best_c,
    double& best_area,
    float& best_radius,
    cv::Rect& best_bbox) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    hsv_color_mask(hsv, mask);
    morph_mask(mask);
    last_debug_mask_ = mask.clone();

    const double a_cap = circle_max_area_ratio_ * frame_area;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& cnt : contours) {
      const double area = std::abs(cv::contourArea(cnt));
      if (area < circle_min_area_ || area > a_cap) continue;
      const cv::Rect bbox = cv::boundingRect(cnt);
      if (bbox.width < 2 || bbox.height < 2) continue;
      const double asp =
        static_cast<double>(std::max(bbox.width, bbox.height)) /
        static_cast<double>(std::max(1, std::min(bbox.width, bbox.height)));
      if (asp > blob_square_aspect_max_) continue;
      // 以轮廓均值 HSV 对齐调试参数中的颜色 id
      cv::Mat cnt_mask = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);
      std::vector<std::vector<cv::Point>> one_cnt{cnt};
      cv::drawContours(cnt_mask, one_cnt, 0, cv::Scalar(255), cv::FILLED);
      const cv::Scalar mean_hsv = cv::mean(hsv, cnt_mask);
      const cv::Vec3d hsv_mean(mean_hsv[0], mean_hsv[1], mean_hsv[2]);
      const int cid = classify_hsv_id(hsv_mean);
      if (hsv_color_id_ != 0 && cid != hsv_color_id_) continue;

      cv::Moments mu = cv::moments(cnt);
      if (mu.m00 < 1e-6) continue;
      const float cx = static_cast<float>(mu.m10 / mu.m00);
      const float cy = static_cast<float>(mu.m01 / mu.m00);

      const double dist = std::hypot(static_cast<double>(cx) - opt.x, static_cast<double>(cy) - opt.y);
      double score = area;
      if (prefer_center_) {
        score -= center_bias_ * dist * dist;
      }
      if (circle_prefer_last_enable_ && has_last_circle_) {
        const double d_last = std::hypot(
          static_cast<double>(cx - last_circle_center_.x), static_cast<double>(cy - last_circle_center_.y));
        score -= circle_prefer_last_bias_ * d_last * d_last;
      }
      if (score > best_score) {
        best_score = score;
        best_c = cv::Point2f(cx, cy);
        best_area = area;
        best_bbox = bbox;
        best_radius = static_cast<float>(std::sqrt(best_area / CV_PI));
        ok = true;
      }
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  bool publish_debug_mask_{false};
  std::string debug_mask_topic_;
  bool publish_debug_overlay_{true};
  std::string debug_overlay_topic_;

  std::string vision_mode_;
  int hsv_color_id_{0};
  int hsv_h_min_{0};
  int hsv_h_max_{180};
  int hsv_s_min_{80};
  int hsv_s_max_{255};
  int hsv_v_min_{80};
  int hsv_v_max_{255};
  int blob_morph_open_{3};
  int blob_morph_close_{5};
  double blob_square_aspect_max_{1.45};
  double hsv_id_dist_max_{5200.0};

  int hough_blur_ksize_{9};
  double hough_dp_{1.2};
  double hough_min_dist_ratio_{0.10};
  double hough_param1_{120.0};
  double hough_param2_{22.0};
  double circle_min_radius_px_{8.0};
  double circle_max_radius_ratio_{0.45};
  double circle_min_area_{120.0};
  double circle_max_area_ratio_{0.80};

  bool prefer_center_{true};
  double center_bias_{0.002};
  bool circle_prefer_last_enable_{true};
  double circle_prefer_last_bias_{0.003};
  int circle_hold_frames_{2};

  bool has_last_circle_{false};
  cv::Point2f last_circle_center_{0.f, 0.f};
  float last_circle_radius_{0.0f};
  int lost_hold_count_{0};
  cv::Rect last_blob_bbox_;
  std::vector<std::pair<int, cv::Vec3d>> ref_hsv_;

  cv::Mat last_debug_mask_;

  image_transport::Subscriber sub_;
  image_transport::Publisher debug_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBlobVisionNode>());
  rclcpp::shutdown();
  return 0;
}
