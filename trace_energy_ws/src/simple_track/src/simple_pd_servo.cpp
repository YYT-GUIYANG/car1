#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <servo_message/msg/servo_message.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

using ServoMsg = servo_message::msg::ServoMessage;

namespace {

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

}  // namespace

/**
 * PD 跟踪 -> 发布 ServoMessage 到 /servo_control，由 servo_uart_node 经串口驱动舵机。
 */
class SimplePdServoNode final : public rclcpp::Node {
public:
  SimplePdServoNode() : Node("simple_pd_servo") {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/blob_xy");
    servo_output_topic_ = this->declare_parameter<std::string>("servo_output_topic", "servo_control");
    control_rate_hz_ = std::clamp(this->declare_parameter<double>("control_rate_hz", 25.0), 5.0, 120.0);

    servo_id_pitch_ = static_cast<int>(this->declare_parameter<int>("servo_id_pitch", 11));
    servo_id_yaw_ = static_cast<int>(this->declare_parameter<int>("servo_id_yaw", 8));

    init_pitch_deg_ = this->declare_parameter<double>("init_pitch_deg", 335.0);
    init_yaw_deg_ = this->declare_parameter<double>("init_yaw_deg", 340.0);

    servo_pitch_travel_pos_deg_ = std::max(0.0, this->declare_parameter<double>("servo_pitch_travel_pos_deg", 10.0));
    servo_pitch_travel_neg_deg_ = std::max(0.0, this->declare_parameter<double>("servo_pitch_travel_neg_deg", 10.0));
    servo_yaw_travel_pos_deg_ = std::max(0.0, this->declare_parameter<double>("servo_yaw_travel_pos_deg", 20.0));
    servo_yaw_travel_neg_deg_ = std::max(0.0, this->declare_parameter<double>("servo_yaw_travel_neg_deg", 20.0));

    kp_yaw_ = this->declare_parameter<double>("kp_yaw", 0.010);
    kd_yaw_ = this->declare_parameter<double>("kd_yaw", 0.006);
    kp_pitch_ = this->declare_parameter<double>("kp_pitch", 0.010);
    kd_pitch_ = this->declare_parameter<double>("kd_pitch", 0.006);

    deadband_px_ = std::max(0.0, this->declare_parameter<double>("deadband_px", 4.0));
    max_step_deg_ = std::clamp(this->declare_parameter<double>("max_step_deg", 0.8), 0.01, 8.0);
    lost_hold_frames_ = std::clamp(static_cast<int>(this->declare_parameter<int>("lost_hold_frames", 2)), 0, 20);

    pitch_dir_sign_ = (this->declare_parameter<int>("pitch_dir_sign", 1) >= 0) ? 1.0 : -1.0;
    yaw_dir_sign_ = (this->declare_parameter<int>("yaw_dir_sign", 1) >= 0) ? 1.0 : -1.0;

    aim_offset_x_px_ = this->declare_parameter<double>("aim_offset_x_px", 0.0);
    aim_offset_y_px_ = this->declare_parameter<double>("aim_offset_y_px", 0.0);

    control_enabled_ = this->declare_parameter<bool>("control_enabled", true);
    publish_track_status_ = this->declare_parameter<bool>("publish_track_status", true);
    track_status_topic_ = this->declare_parameter<std::string>("track_status_topic", "/simple_track/status");

    pub_ = this->create_publisher<ServoMsg>(servo_output_topic_, rclcpp::QoS(10));
    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      input_topic_, rclcpp::QoS(1),
      std::bind(&SimplePdServoNode::on_target, this, std::placeholders::_1));

    pitch_cmd_deg_ = init_pitch_deg_;
    yaw_cmd_deg_ = init_yaw_deg_;

    if (publish_track_status_) {
      status_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(track_status_topic_, rclcpp::QoS(10));
    }

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimplePdServoNode::on_tick, this));

    RCLCPP_INFO(this->get_logger(),
                "[simple_pd_servo] subscribe=%s publish=%s (pitch_id=%d yaw_id=%d) @ %.1fHz init pitch/yaw=%.1f/%.1f",
                input_topic_.c_str(), servo_output_topic_.c_str(), servo_id_pitch_, servo_id_yaw_, control_rate_hz_,
                init_pitch_deg_, init_yaw_deg_);
  }

private:
  void on_target(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (!msg || msg->data.size() < 6) return;
    last_valid_ = (msg->data[0] > 0.5f);
    last_cx_ = msg->data[1];
    last_cy_ = msg->data[2];
    last_w_ = msg->data[4];
    last_h_ = msg->data[5];
    if (last_valid_) {
      lost_count_ = 0;
      last_good_cx_ = last_cx_;
      last_good_cy_ = last_cy_;
      last_good_w_ = last_w_;
      last_good_h_ = last_h_;
      has_last_good_ = true;
    }
  }

  void publish_servo_(int sid, int ang_deg) {
    ServoMsg m;
    m.timestamp = this->get_clock()->now();
    m.servo_id = sid;
    m.servo_angle = std::clamp(ang_deg, 0, 360);
    pub_->publish(m);
  }

  void on_tick() {
    const double dt = 1.0 / std::max(1e-3, control_rate_hz_);

    const double yaw_lo = init_yaw_deg_ - servo_yaw_travel_neg_deg_;
    const double yaw_hi = init_yaw_deg_ + servo_yaw_travel_pos_deg_;
    const double pitch_lo = init_pitch_deg_ - servo_pitch_travel_neg_deg_;
    const double pitch_hi = init_pitch_deg_ + servo_pitch_travel_pos_deg_;

    double ex_pub = 0.0;
    double ey_pub = 0.0;
    float cx_pub = 0.f;
    float cy_pub = 0.f;

    if (!control_enabled_) {
      yaw_cmd_deg_ = init_yaw_deg_;
      pitch_cmd_deg_ = init_pitch_deg_;
      prev_ex_ = 0.0;
      prev_ey_ = 0.0;
      cx_pub = last_cx_;
      cy_pub = last_cy_;
    } else {
      float cx = last_cx_, cy = last_cy_, ww = last_w_, hh = last_h_;
      bool use = last_valid_ && (ww >= 10.0f) && (hh >= 10.0f);
      if (!use) {
        if (has_last_good_ && lost_count_ < lost_hold_frames_) {
          cx = last_good_cx_;
          cy = last_good_cy_;
          ww = last_good_w_;
          hh = last_good_h_;
          use = true;
          lost_count_ += 1;
        }
      }

      cx_pub = cx;
      cy_pub = cy;

      if (use) {
        const double cx0 = 0.5 * static_cast<double>(ww) + aim_offset_x_px_;
        const double cy0 = 0.5 * static_cast<double>(hh) + aim_offset_y_px_;
        const double ex = static_cast<double>(cx) - cx0;
        const double ey = static_cast<double>(cy) - cy0;

        const double ex_f = (std::abs(ex) < deadband_px_) ? 0.0 : ex;
        const double ey_f = (std::abs(ey) < deadband_px_) ? 0.0 : ey;
        ex_pub = ex_f;
        ey_pub = ey_f;

        const double dex = (ex_f - prev_ex_) / dt;
        const double dey = (ey_f - prev_ey_) / dt;
        prev_ex_ = ex_f;
        prev_ey_ = ey_f;

        double u_yaw = yaw_dir_sign_ * (kp_yaw_ * ex_f + kd_yaw_ * dex);
        double u_pitch = pitch_dir_sign_ * (kp_pitch_ * ey_f + kd_pitch_ * dey);

        u_yaw = clampd(u_yaw, -max_step_deg_, max_step_deg_);
        u_pitch = clampd(u_pitch, -max_step_deg_, max_step_deg_);

        yaw_cmd_deg_ = clampd(yaw_cmd_deg_ + u_yaw, yaw_lo, yaw_hi);
        pitch_cmd_deg_ = clampd(pitch_cmd_deg_ + u_pitch, pitch_lo, pitch_hi);
      }
    }

    publish_servo_(servo_id_yaw_, static_cast<int>(std::lround(yaw_cmd_deg_)));
    publish_servo_(servo_id_pitch_, static_cast<int>(std::lround(pitch_cmd_deg_)));

    if (publish_track_status_ && status_pub_) {
      std_msgs::msg::Float32MultiArray st;
      st.data.resize(8);
      st.data[0] = last_valid_ ? 1.f : 0.f;
      st.data[1] = control_enabled_ ? 1.f : 0.f;
      st.data[2] = cx_pub;
      st.data[3] = cy_pub;
      st.data[4] = static_cast<float>(ex_pub);
      st.data[5] = static_cast<float>(ey_pub);
      st.data[6] = static_cast<float>(yaw_cmd_deg_);
      st.data[7] = static_cast<float>(pitch_cmd_deg_);
      status_pub_->publish(st);
    }
  }

  std::string input_topic_;
  std::string servo_output_topic_;
  double control_rate_hz_{25.0};

  int servo_id_pitch_{11};
  int servo_id_yaw_{8};

  double init_pitch_deg_{335.0};
  double init_yaw_deg_{340.0};
  double servo_pitch_travel_pos_deg_{10.0};
  double servo_pitch_travel_neg_deg_{10.0};
  double servo_yaw_travel_pos_deg_{20.0};
  double servo_yaw_travel_neg_deg_{20.0};

  double kp_yaw_{0.01};
  double kd_yaw_{0.006};
  double kp_pitch_{0.01};
  double kd_pitch_{0.006};
  double deadband_px_{4.0};
  double max_step_deg_{0.8};
  int lost_hold_frames_{2};

  double pitch_dir_sign_{1.0};
  double yaw_dir_sign_{1.0};
  double aim_offset_x_px_{0.0};
  double aim_offset_y_px_{0.0};

  bool control_enabled_{true};
  bool publish_track_status_{true};
  std::string track_status_topic_;

  rclcpp::Publisher<ServoMsg>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool last_valid_{false};
  float last_cx_{0.f}, last_cy_{0.f}, last_w_{0.f}, last_h_{0.f};
  bool has_last_good_{false};
  int lost_count_{0};
  float last_good_cx_{0.f}, last_good_cy_{0.f}, last_good_w_{0.f}, last_good_h_{0.f};
  double prev_ex_{0.0}, prev_ey_{0.0};

  double pitch_cmd_deg_{335.0};
  double yaw_cmd_deg_{340.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePdServoNode>());
  rclcpp::shutdown();
  return 0;
}
