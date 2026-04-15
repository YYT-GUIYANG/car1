// 通用 12 色 Lab 追踪节点（自 trace_calculator 提取，无能量机关专用逻辑）
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/utility.hpp"
#include "image_transport/image_transport.hpp"
#include "servo_message/msg/servo_message.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <string>
#include <vector>

using ServoMsg = servo_message::msg::ServoMessage;

namespace {

const std::vector<std::pair<int, cv::Vec3b>> kReferenceBgr = {
    {0, cv::Vec3b(116, 5, 202)},   {3, cv::Vec3b(167, 1, 98)},   {4, cv::Vec3b(7, 237, 19)},
    {5, cv::Vec3b(23, 51, 215)},   {6, cv::Vec3b(241, 132, 251)}, {7, cv::Vec3b(17, 168, 214)},
    {8, cv::Vec3b(135, 199, 246)}, {9, cv::Vec3b(221, 66, 76)},   {10, cv::Vec3b(216, 199, 167)},
    {11, cv::Vec3b(85, 152, 55)},  {12, cv::Vec3b(57, 244, 231)}, {16, cv::Vec3b(23, 3, 23)},
};

struct Blob {
    int cid = -1;
    double area = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double circularity = 0.0;
    std::string shape = "other";
    int approx_vert = 0;
    std::vector<cv::Point> contour;
};

struct TrackState {
    bool has_smooth_target = false;
    cv::Point2f smooth_target{0.f, 0.f};
    bool has_prev_raw = false;
    cv::Point2f prev_raw{0.f, 0.f};
    cv::Point2f vel_ema{0.f, 0.f};
    int miss_count = 0;
};

double contour_circularity(const std::vector<cv::Point>& c) {
    const double a = cv::contourArea(c);
    const double p = cv::arcLength(c, true);
    if (a < 1.0 || p < 1e-6) return 0.0;
    return 4.0 * CV_PI * a / (p * p);
}

double min_area_rect_aspect_ratio(const std::vector<cv::Point>& c) {
    if (c.size() < 3) return 0.0;
    const cv::RotatedRect r = cv::minAreaRect(c);
    const float w = r.size.width, h = r.size.height;
    if (w < 1e-6f || h < 1e-6f) return 0.0;
    return static_cast<double>(std::min(w, h) / std::max(w, h));
}

std::string classify_shape(double circ, const std::vector<cv::Point>& c) {
    constexpr double kCircleMin = 0.78;
    constexpr double kSquareLo = 0.68;
    constexpr double kSquareHi = 0.84;
    if (kSquareLo <= circ && circ <= kSquareHi) return "square";
    if (circ >= kCircleMin) return "circle";
    const double asp = min_area_rect_aspect_ratio(c);
    if (circ >= 0.62 && circ < kCircleMin && asp >= 0.45) return "square";
    return "other";
}

static int contour_approx_vertex_count(const std::vector<cv::Point>& c, double eps_frac) {
    if (c.size() < 3) return 0;
    const double per = cv::arcLength(c, true);
    if (per < 1e-6) return 0;
    const double ef = std::clamp(eps_frac, 0.005, 0.08);
    std::vector<cv::Point> ap;
    cv::approxPolyDP(c, ap, ef * per, true);
    return static_cast<int>(ap.size());
}

cv::Mat refine_mask(const cv::Mat& mask, int ksize) {
    int k = std::max(3, ksize | 1);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
    cv::Mat out;
    cv::morphologyEx(mask, out, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(out, out, cv::MORPH_OPEN, kernel);
    return out;
}

static void apply_gamma_bgr_inplace(cv::Mat& bgr, double gamma) {
    if (bgr.empty() || bgr.type() != CV_8UC3 || std::abs(gamma - 1.0) < 0.02) return;
    const double g = 1.0 / std::clamp(gamma, 0.35, 3.5);
    cv::Mat lut(1, 256, CV_8U);
    uchar* pl = lut.ptr<uchar>(0);
    for (int i = 0; i < 256; ++i) {
        pl[i] = cv::saturate_cast<uchar>(std::pow(static_cast<double>(i) / 255.0, g) * 255.0);
    }
    cv::LUT(bgr, lut, bgr);
}

static void apply_clahe_lab_l_inplace(cv::Mat& bgr, double clip_limit) {
    if (bgr.empty() || bgr.type() != CV_8UC3 || !(clip_limit > 0.05)) return;
    cv::Mat lab;
    cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> ch(3);
    cv::split(lab, ch);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(std::clamp(clip_limit, 0.1, 12.0), cv::Size(8, 8));
    clahe->apply(ch[0], ch[0]);
    cv::merge(ch, lab);
    cv::cvtColor(lab, bgr, cv::COLOR_Lab2BGR);
}

static int suppress_bright_lights_inplace(
    cv::Mat& bgr, int hsv_v_min, int hsv_s_max, int bgr_ch_min, int dilate_ksize, int inpaint_radius,
    double max_area_frac) {
    if (bgr.empty() || bgr.type() != CV_8UC3) return 0;
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hch;
    cv::split(hsv, hch);
    cv::Mat m_hsv;
    cv::bitwise_and(hch[2] >= hsv_v_min, hch[1] <= hsv_s_max, m_hsv);
    std::vector<cv::Mat> bc;
    cv::split(bgr, bc);
    cv::Mat m_bgr;
    cv::bitwise_and(bc[0] >= bgr_ch_min, bc[1] >= bgr_ch_min, m_bgr);
    cv::bitwise_and(m_bgr, bc[2] >= bgr_ch_min, m_bgr);
    cv::Mat mask;
    cv::bitwise_or(m_hsv, m_bgr, mask);
    if (dilate_ksize >= 3) {
        const int dk = dilate_ksize | 1;
        cv::Mat ker = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dk, dk));
        cv::dilate(mask, mask, ker);
    }
    const int nz = cv::countNonZero(mask);
    if (nz < 16) return 0;
    const double cov = static_cast<double>(nz) / static_cast<double>(std::max(1, bgr.rows * bgr.cols));
    if (cov > max_area_frac) return 0;
    cv::Mat inpainted;
    cv::inpaint(bgr, mask, inpainted, std::max(1, inpaint_radius), cv::INPAINT_NS);
    inpainted.copyTo(bgr);
    return nz;
}

static void center_crop_zoom_inplace(cv::Mat& bgr, float zoom) {
    if (bgr.empty() || zoom <= 1.001f) return;
    const int vw = bgr.cols, vh = bgr.rows;
    const int cw = std::max(8, static_cast<int>(static_cast<float>(vw) / zoom));
    const int ch = std::max(8, static_cast<int>(static_cast<float>(vh) / zoom));
    cv::Rect zr((vw - cw) / 2, (vh - ch) / 2, cw, ch);
    zr &= cv::Rect(0, 0, vw, vh);
    if (zr.width < 8 || zr.height < 8) return;
    cv::Mat crop = bgr(zr).clone();
    cv::resize(crop, bgr, cv::Size(vw, vh), 0, 0, cv::INTER_LINEAR);
}

static int wrap_deg_0_359(int deg) {
    int x = deg % 360;
    if (x < 0) x += 360;
    return x;
}

static int safe_round_servo_deg(float v) {
    if (!std::isfinite(v)) return 0;
    return static_cast<int>(std::lround(std::clamp(v, -720.0f, 1080.0f)));
}

static int clamp_pitch_dual_window_deg(int cmd_deg, int main_lo_deg, int wrap_hi_deg) {
    const int cmd = wrap_deg_0_359(cmd_deg);
    const int main_lo = std::clamp(main_lo_deg, 0, 359);
    const int wrap_hi = std::clamp(wrap_hi_deg, 0, 359);
    if (wrap_hi >= main_lo) return cmd;
    if (cmd >= main_lo || cmd <= wrap_hi) return cmd;
    const int d_to_wrap_hi = cmd - wrap_hi;
    const int d_to_main_lo = main_lo - cmd;
    return (d_to_wrap_hi <= d_to_main_lo) ? wrap_hi : main_lo;
}

/** 指定 Lab 颜色 ID，取 approx 顶点为 4~5 且面积最大的轮廓（四边形类目标）。 */
static Blob* pick_largest_quad_for_color(
    std::vector<Blob>& blobs,
    int target_cid,
    double sector_min_area,
    double sector_max_area_ratio,
    double frame_area,
    double approx_eps) {
    if (target_cid < 0) return nullptr;
    Blob* best = nullptr;
    double best_area = -1.0;
    for (auto& b : blobs) {
        if (b.cid != target_cid) continue;
        if (b.area < sector_min_area || b.area > sector_max_area_ratio * frame_area) continue;
        const int nv = contour_approx_vertex_count(b.contour, approx_eps);
        if (nv != 4 && nv != 5) continue;
        if (b.area > best_area) {
            best_area = b.area;
            best = &b;
        }
    }
    return best;
}

/** 同色最大连通域（不强制四边形近似），用于 quad 未检出时仍能对色块质心追踪。 */
static Blob* pick_largest_blob_for_color(
    std::vector<Blob>& blobs,
    int target_cid,
    double sector_min_area,
    double sector_max_area_ratio,
    double frame_area) {
    if (target_cid < 0) return nullptr;
    Blob* best = nullptr;
    double best_area = -1.0;
    for (auto& b : blobs) {
        if (b.cid != target_cid) continue;
        if (b.area < sector_min_area || b.area > sector_max_area_ratio * frame_area) continue;
        if (b.area > best_area) {
            best_area = b.area;
            best = &b;
        }
    }
    return best;
}

static std::vector<cv::Point> contour_scaled_to_vis(const std::vector<cv::Point>& c, float inv_scale) {
    std::vector<cv::Point> out;
    out.reserve(c.size());
    for (const auto& p : c) {
        out.emplace_back(
            static_cast<int>(std::lround(p.x * inv_scale)), static_cast<int>(std::lround(p.y * inv_scale)));
    }
    return out;
}

constexpr const char* kPidTuneWinBase = "1-PID_BASE";
constexpr const char* kPidTuneWinFilter = "2-FILTER";
constexpr const char* kPidTuneWinDeadband = "3-DEADBAND";
constexpr const char* kPidTuneWinOutput = "4-OUTPUT";

static int bar_from_scaled(double v, double unit, int lo, int hi) {
    const int p = static_cast<int>(std::lround(v / unit));
    return std::clamp(p, lo, hi);
}

}  // namespace

class PidTestNode : public rclcpp::Node {
public:
    PidTestNode() : Node("pidtest_node") {
        declare_and_load_params_();
        [[maybe_unused]] const auto param_cb_unused = this->add_on_set_parameters_callback(
            std::bind(&PidTestNode::on_set_parameters_cb, this, std::placeholders::_1));

        image_sub_ = image_transport::create_subscription(
            this,
            image_topic_,
            std::bind(&PidTestNode::image_callback, this, std::placeholders::_1),
            "raw",
            rclcpp::QoS(10).get_rmw_qos_profile());
        servo_pub_ = this->create_publisher<ServoMsg>("/servo_control", 10);
        tracking_debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tracking_debug", 10);
        center_color_pub_ = this->create_publisher<std_msgs::msg::Int32>("/center_color_id", 10);
        trace_debug_pub_ = image_transport::create_publisher(
            this, trace_debug_image_topic_, rclcpp::QoS(1).get_rmw_qos_profile());

        last_frame_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "[pidtest] 订阅图像话题: %s", image_topic_.c_str());
        if (show_vis_window_ || pid_tune_sliders_) {
            const char* disp = std::getenv("DISPLAY");
            if (disp == nullptr || disp[0] == '\0') {
                RCLCPP_WARN(
                    this->get_logger(),
                    "未检测到 DISPLAY，OpenCV 窗口（Square Tracker / PID 滑块）可能无法显示。"
                    "调试图像可用: ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/trace_debug_image");
            }
        }
    }

    ~PidTestNode() override { cv::destroyAllWindows(); }

private:
    void declare_and_load_params_() {
        control_enabled_ = declare_parameter<bool>("control_enabled", true);
        image_topic_ = declare_parameter<std::string>("image_topic", "/processed_image");
        {
            rcl_interfaces::msg::ParameterDescriptor d;
            d.description = "追踪的 Lab 颜色 ID（与 kReferenceBgr 表一致），仅匹配该 ID 的四边形轮廓";
            target_color_id_ = declare_parameter<int>("target_color_id", 0, d);
        }
        track_fallback_to_largest_color_blob_ =
            declare_parameter<bool>("track_fallback_to_largest_color_blob", true);
        init_pitch_deg_ = declare_parameter<double>("init_pitch_deg", 340.0);
        init_yaw_deg_ = declare_parameter<double>("init_yaw_deg", 170.0);
        startup_hold_frames_ = declare_parameter<int>("startup_hold_frames", 15);
        target_hold_frames_ = static_cast<int>(
            std::clamp(declare_parameter<int>("target_hold_frames", 5), int64_t{0}, int64_t{300}));
        servo_id_pitch_ = declare_parameter<int>("servo_id_pitch", 8);
        servo_id_yaw_ = declare_parameter<int>("servo_id_yaw", 11);
        servo_pitch_min_deg_ = declare_parameter<double>("servo_pitch_min_deg", 320.0);
        servo_pitch_max_deg_ = declare_parameter<double>("servo_pitch_max_deg", 360.0);
        servo_yaw_min_deg_ = declare_parameter<double>("servo_yaw_min_deg", 110.0);
        servo_yaw_max_deg_ = declare_parameter<double>("servo_yaw_max_deg", 225.0);
        servo_pitch_travel_half_span_deg_ = declare_parameter<double>("servo_pitch_travel_half_span_deg", 20.0);
        servo_yaw_travel_half_span_deg_ = declare_parameter<double>("servo_yaw_travel_half_span_deg", 60.0);
        pitch_main_band_low_deg_ = static_cast<int>(
            std::clamp(declare_parameter<int>("pitch_main_band_low_deg", 320), int64_t{0}, int64_t{359}));
        pitch_wrap_band_high_deg_ = static_cast<int>(
            std::clamp(declare_parameter<int>("pitch_wrap_band_high_deg", 359), int64_t{0}, int64_t{359}));
        trace_proc_width_cap_ = static_cast<int>(
            std::clamp(declare_parameter<int>("trace_proc_width_cap", 480), int64_t{320}, int64_t{1920}));
        lab_thresh_ = declare_parameter<double>("lab_thresh", 71.5);
        lab_preprocess_blur_ksize_ = declare_parameter<int>("lab_preprocess_blur_ksize", 5);
        lab_mask_morph_ksize_ = declare_parameter<int>("lab_mask_morph_ksize", 3);
        vis_input_gamma_ = std::clamp(declare_parameter<double>("vis_input_gamma", 0.76), 0.35, 3.5);
        vis_clahe_l_clip_limit_ = std::clamp(declare_parameter<double>("vis_clahe_l_clip_limit", 0.0), 0.0, 12.0);
        lab_mask_canny_bridge_ = declare_parameter<bool>("lab_mask_canny_bridge", false);
        lab_canny_thresh1_ = static_cast<int>(
            std::clamp(declare_parameter<int>("lab_canny_thresh1", 80), int64_t{1}, int64_t{254}));
        lab_canny_thresh2_ = static_cast<int>(
            std::clamp(declare_parameter<int>("lab_canny_thresh2", 160), int64_t{1}, int64_t{255}));
        lab_canny_dilate_px_ = static_cast<int>(
            std::clamp(declare_parameter<int>("lab_canny_dilate_px", 5), int64_t{1}, int64_t{15}));
        lab_canny_bridge_mask_dilate_ = static_cast<int>(
            std::clamp(declare_parameter<int>("lab_canny_bridge_mask_dilate", 2), int64_t{1}, int64_t{5}));
        ctrl_integ_max_px_s_ = std::clamp(declare_parameter<double>("ctrl_integ_max_px_s", 60.0), 10.0, 800.0);
        kp_ = declare_parameter<double>("kp", 0.022);
        kd_ = declare_parameter<double>("kd", 0.006);
        ki_yaw_ = declare_parameter<double>("ki_yaw", 0.00002);
        ki_pitch_ = declare_parameter<double>("ki_pitch", 0.00001);
        err_lp_beta_yaw_ = declare_parameter<double>("err_lp_beta_yaw", 0.62);
        err_lp_beta_pitch_ = declare_parameter<double>("err_lp_beta_pitch", 0.68);
        deadband_px_ = declare_parameter<int>("deadband_px", 3);
        deadband_yaw_px_ = declare_parameter<int>("deadband_yaw_px", 2);
        max_delta_deg_ = declare_parameter<double>("max_delta_deg", 1.2);
        yaw_gain_ = declare_parameter<double>("yaw_gain", 2.45);
        yaw_kp_mult_ = declare_parameter<double>("yaw_kp_mult", 3.0);
        yaw_min_step_deg_ = declare_parameter<double>("yaw_min_step_deg", 0.35);
        near_tgt_yaw_kp_scale_ = declare_parameter<double>("near_tgt_yaw_kp_scale", 0.80);
        // 目标已在画面中心附近时，量化舵机角易与 ±1°「去粘连」微推形成极限环；默认关闭。
        yaw_quantize_nudge_enable_ = declare_parameter<bool>("yaw_quantize_nudge_enable", false);
        yaw_still_zone_hit_px_ =
            std::clamp(declare_parameter<double>("yaw_still_zone_hit_px", 30.0), 8.0, 120.0);
        yaw_still_zone_deadband_extra_px_ =
            std::clamp(declare_parameter<double>("yaw_still_zone_deadband_extra_px", 2.5), 0.0, 15.0);
        yaw_still_zone_kd_scale_ =
            std::clamp(declare_parameter<double>("yaw_still_zone_kd_scale", 0.32), 0.0, 1.0);
        yaw_min_step_min_hit_px_ =
            std::clamp(declare_parameter<double>("yaw_min_step_min_hit_px", 22.0), 0.0, 80.0);
        // 静止区 yaw 死区滞回：|filt| 进入 db 内则冻结（不跟），需超过 db+hysteresis 才重新跟，减轻「大死区慢 / 小死区抖」两难。
        yaw_still_hysteresis_px_ =
            std::clamp(declare_parameter<double>("yaw_still_hysteresis_px", 5.0), 0.0, 28.0);
        d_lp_beta_yaw_ = declare_parameter<double>("d_lp_beta_yaw", 0.24);
        d_lp_beta_pitch_ = declare_parameter<double>("d_lp_beta_pitch", 0.18);
        antiwindup_backcalc_gain_yaw_ = declare_parameter<double>("antiwindup_backcalc_gain_yaw", 0.35);
        antiwindup_backcalc_gain_pitch_ = declare_parameter<double>("antiwindup_backcalc_gain_pitch", 0.35);
        search_yaw_amp_deg_ = declare_parameter<double>("search_yaw_amp_deg", 32.0);
        search_pitch_amp_deg_ = declare_parameter<double>("search_pitch_amp_deg", 12.0);
        search_phase_step_ = declare_parameter<double>("search_phase_step", 0.10);
        search_pitch_phase_step_ = declare_parameter<double>("search_pitch_phase_step", 0.035);
        search_enter_frames_ = declare_parameter<int>("search_enter_frames", 2);
        settle_px_ = declare_parameter<double>("settle_px", 12.0);
        settle_frames_ = declare_parameter<int>("settle_frames", 6);
        servo_angle_smooth_beta_ = declare_parameter<double>("servo_angle_smooth_beta", 0.88);
        publish_tracking_debug_ = declare_parameter<bool>("publish_tracking_debug", true);
        show_vis_window_ = declare_parameter<bool>("show_vis_window", false);
        publish_trace_debug_image_ = declare_parameter<bool>("publish_trace_debug_image", true);
        trace_debug_image_topic_ = declare_parameter<std::string>("trace_debug_image_topic", "/trace_debug_image");
        roi_frac_ = declare_parameter<double>("roi_frac", 0.60);
        center_roi_zoom_ = declare_parameter<double>("center_roi_zoom", 1.0);
        suppress_bright_lights_ = declare_parameter<bool>("suppress_bright_lights", false);
        bright_hsv_v_min_ = declare_parameter<int>("bright_hsv_v_min", 252);
        bright_hsv_s_max_ = declare_parameter<int>("bright_hsv_s_max", 58);
        bright_bgr_channel_min_ = declare_parameter<int>("bright_bgr_channel_min", 244);
        bright_mask_dilate_ = declare_parameter<int>("bright_mask_dilate", 5);
        bright_inpaint_radius_ = declare_parameter<int>("bright_inpaint_radius", 5);
        bright_inpaint_max_area_frac_ = declare_parameter<double>("bright_inpaint_max_area_frac", 0.012);
        smooth_target_alpha_scale_ = declare_parameter<double>("smooth_target_alpha_scale", 1.0);
        target_step_max_px_ = declare_parameter<double>("target_step_max_px", 88.0);
        predict_coast_frames_ = declare_parameter<int>("predict_coast_frames", 12);
        predict_lead_ms_ = declare_parameter<double>("predict_lead_ms", 62.0);
        predict_arm_frames_ = declare_parameter<int>("predict_arm_frames", 8);
        predict_vel_beta_ = declare_parameter<double>("predict_vel_beta", 0.42);
        lab_min_area_seg_ = declare_parameter<double>("lab_min_area_seg", 50.0);
        sector_min_area_ = declare_parameter<double>("sector_min_area", 220.0);
        sector_max_area_ratio_ =
            std::clamp(declare_parameter<double>("sector_max_area_ratio", 0.52), 0.05, 0.95);
        square_approx_poly_eps_ =
            std::clamp(declare_parameter<double>("square_approx_poly_eps", 0.024), 0.005, 0.08);
        publish_center_color_id_ = declare_parameter<bool>("publish_center_color_id", true);
        pid_tune_sliders_ = declare_parameter<bool>("pid_tune_sliders", false);
        {
            rcl_interfaces::msg::ParameterDescriptor d;
            d.description = "按 s 保存滑块/当前 PID 参数到此路径；空则 ~/.ros/pidtest_pid_tune_snapshot.yaml";
            pid_tune_save_path_ = declare_parameter<std::string>("pid_tune_save_path", "", d);
        }
    }

    rcl_interfaces::msg::SetParametersResult on_set_parameters_cb(
        const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : parameters) {
            const std::string& n = p.get_name();
            try {
                if (n == "control_enabled") control_enabled_ = p.as_bool();
                else if (n == "image_topic") {
                    image_topic_ = p.as_string();
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[pidtest] image_topic 已更新为 %s，需重启节点后订阅才会切换", image_topic_.c_str());
                } else if (n == "target_color_id") target_color_id_ = p.as_int();
                else if (n == "track_fallback_to_largest_color_blob")
                    track_fallback_to_largest_color_blob_ = p.as_bool();
                else if (n == "init_pitch_deg") init_pitch_deg_ = p.as_double();
                else if (n == "init_yaw_deg") init_yaw_deg_ = p.as_double();
                else if (n == "startup_hold_frames") startup_hold_frames_ = p.as_int();
                else if (n == "target_hold_frames")
                    target_hold_frames_ = static_cast<int>(std::clamp(p.as_int(), int64_t{0}, int64_t{300}));
                else if (n == "servo_id_pitch") servo_id_pitch_ = p.as_int();
                else if (n == "servo_id_yaw") servo_id_yaw_ = p.as_int();
                else if (n == "servo_pitch_min_deg") servo_pitch_min_deg_ = p.as_double();
                else if (n == "servo_pitch_max_deg") servo_pitch_max_deg_ = p.as_double();
                else if (n == "servo_yaw_min_deg") servo_yaw_min_deg_ = p.as_double();
                else if (n == "servo_yaw_max_deg") servo_yaw_max_deg_ = p.as_double();
                else if (n == "servo_pitch_travel_half_span_deg")
                    servo_pitch_travel_half_span_deg_ = p.as_double();
                else if (n == "servo_yaw_travel_half_span_deg")
                    servo_yaw_travel_half_span_deg_ = p.as_double();
                else if (n == "pitch_main_band_low_deg")
                    pitch_main_band_low_deg_ = static_cast<int>(std::clamp(p.as_int(), int64_t{0}, int64_t{359}));
                else if (n == "pitch_wrap_band_high_deg")
                    pitch_wrap_band_high_deg_ = static_cast<int>(std::clamp(p.as_int(), int64_t{0}, int64_t{359}));
                else if (n == "trace_proc_width_cap")
                    trace_proc_width_cap_ = static_cast<int>(
                        std::clamp(p.as_int(), int64_t{320}, int64_t{1920}));
                else if (n == "lab_thresh") lab_thresh_ = p.as_double();
                else if (n == "lab_preprocess_blur_ksize") lab_preprocess_blur_ksize_ = p.as_int();
                else if (n == "lab_mask_morph_ksize") lab_mask_morph_ksize_ = p.as_int();
                else if (n == "vis_input_gamma")
                    vis_input_gamma_ = std::clamp(p.as_double(), 0.35, 3.5);
                else if (n == "vis_clahe_l_clip_limit")
                    vis_clahe_l_clip_limit_ = std::clamp(p.as_double(), 0.0, 12.0);
                else if (n == "lab_mask_canny_bridge") lab_mask_canny_bridge_ = p.as_bool();
                else if (n == "lab_canny_thresh1")
                    lab_canny_thresh1_ = static_cast<int>(std::clamp(p.as_int(), int64_t{1}, int64_t{254}));
                else if (n == "lab_canny_thresh2")
                    lab_canny_thresh2_ = static_cast<int>(std::clamp(p.as_int(), int64_t{1}, int64_t{255}));
                else if (n == "lab_canny_dilate_px")
                    lab_canny_dilate_px_ = static_cast<int>(std::clamp(p.as_int(), int64_t{1}, int64_t{15}));
                else if (n == "lab_canny_bridge_mask_dilate")
                    lab_canny_bridge_mask_dilate_ =
                        static_cast<int>(std::clamp(p.as_int(), int64_t{1}, int64_t{5}));
                else if (n == "ctrl_integ_max_px_s")
                    ctrl_integ_max_px_s_ = std::clamp(p.as_double(), 10.0, 800.0);
                else if (n == "kp") kp_ = p.as_double();
                else if (n == "kd") kd_ = p.as_double();
                else if (n == "ki_yaw") ki_yaw_ = p.as_double();
                else if (n == "ki_pitch") ki_pitch_ = p.as_double();
                else if (n == "err_lp_beta_yaw") err_lp_beta_yaw_ = p.as_double();
                else if (n == "err_lp_beta_pitch") err_lp_beta_pitch_ = p.as_double();
                else if (n == "deadband_px") deadband_px_ = p.as_int();
                else if (n == "deadband_yaw_px") deadband_yaw_px_ = p.as_int();
                else if (n == "max_delta_deg") max_delta_deg_ = p.as_double();
                else if (n == "yaw_gain") yaw_gain_ = p.as_double();
                else if (n == "yaw_kp_mult") yaw_kp_mult_ = p.as_double();
                else if (n == "yaw_min_step_deg") yaw_min_step_deg_ = p.as_double();
                else if (n == "near_tgt_yaw_kp_scale") near_tgt_yaw_kp_scale_ = p.as_double();
                else if (n == "yaw_quantize_nudge_enable") yaw_quantize_nudge_enable_ = p.as_bool();
                else if (n == "yaw_still_zone_hit_px")
                    yaw_still_zone_hit_px_ = std::clamp(p.as_double(), 8.0, 120.0);
                else if (n == "yaw_still_zone_deadband_extra_px")
                    yaw_still_zone_deadband_extra_px_ = std::clamp(p.as_double(), 0.0, 15.0);
                else if (n == "yaw_still_zone_kd_scale")
                    yaw_still_zone_kd_scale_ = std::clamp(p.as_double(), 0.0, 1.0);
                else if (n == "yaw_min_step_min_hit_px")
                    yaw_min_step_min_hit_px_ = std::clamp(p.as_double(), 0.0, 80.0);
                else if (n == "yaw_still_hysteresis_px")
                    yaw_still_hysteresis_px_ = std::clamp(p.as_double(), 0.0, 28.0);
                else if (n == "d_lp_beta_yaw")
                    d_lp_beta_yaw_ = std::clamp(p.as_double(), 0.0, 1.0);
                else if (n == "d_lp_beta_pitch")
                    d_lp_beta_pitch_ = std::clamp(p.as_double(), 0.0, 1.0);
                else if (n == "antiwindup_backcalc_gain_yaw")
                    antiwindup_backcalc_gain_yaw_ = std::clamp(p.as_double(), 0.0, 2.0);
                else if (n == "antiwindup_backcalc_gain_pitch")
                    antiwindup_backcalc_gain_pitch_ = std::clamp(p.as_double(), 0.0, 2.0);
                else if (n == "search_yaw_amp_deg") search_yaw_amp_deg_ = p.as_double();
                else if (n == "search_pitch_amp_deg") search_pitch_amp_deg_ = p.as_double();
                else if (n == "search_phase_step") search_phase_step_ = p.as_double();
                else if (n == "search_pitch_phase_step") search_pitch_phase_step_ = p.as_double();
                else if (n == "search_enter_frames") search_enter_frames_ = p.as_int();
                else if (n == "settle_px") settle_px_ = p.as_double();
                else if (n == "settle_frames") settle_frames_ = p.as_int();
                else if (n == "servo_angle_smooth_beta")
                    servo_angle_smooth_beta_ = p.as_double();
                else if (n == "publish_tracking_debug") publish_tracking_debug_ = p.as_bool();
                else if (n == "show_vis_window") show_vis_window_ = p.as_bool();
                else if (n == "publish_trace_debug_image") publish_trace_debug_image_ = p.as_bool();
                else if (n == "trace_debug_image_topic") trace_debug_image_topic_ = p.as_string();
                else if (n == "roi_frac") roi_frac_ = p.as_double();
                else if (n == "center_roi_zoom") center_roi_zoom_ = p.as_double();
                else if (n == "suppress_bright_lights") suppress_bright_lights_ = p.as_bool();
                else if (n == "bright_hsv_v_min") bright_hsv_v_min_ = p.as_int();
                else if (n == "bright_hsv_s_max") bright_hsv_s_max_ = p.as_int();
                else if (n == "bright_bgr_channel_min") bright_bgr_channel_min_ = p.as_int();
                else if (n == "bright_mask_dilate") bright_mask_dilate_ = p.as_int();
                else if (n == "bright_inpaint_radius") bright_inpaint_radius_ = p.as_int();
                else if (n == "bright_inpaint_max_area_frac")
                    bright_inpaint_max_area_frac_ = p.as_double();
                else if (n == "smooth_target_alpha_scale")
                    smooth_target_alpha_scale_ = p.as_double();
                else if (n == "target_step_max_px") target_step_max_px_ = p.as_double();
                else if (n == "predict_coast_frames") predict_coast_frames_ = p.as_int();
                else if (n == "predict_lead_ms") predict_lead_ms_ = p.as_double();
                else if (n == "predict_arm_frames") predict_arm_frames_ = p.as_int();
                else if (n == "predict_vel_beta") predict_vel_beta_ = p.as_double();
                else if (n == "lab_min_area_seg") lab_min_area_seg_ = p.as_double();
                else if (n == "sector_min_area") sector_min_area_ = p.as_double();
                else if (n == "sector_max_area_ratio")
                    sector_max_area_ratio_ = std::clamp(p.as_double(), 0.05, 0.95);
                else if (n == "square_approx_poly_eps")
                    square_approx_poly_eps_ = std::clamp(p.as_double(), 0.005, 0.08);
                else if (n == "publish_center_color_id") publish_center_color_id_ = p.as_bool();
                else if (n == "pid_tune_sliders") pid_tune_sliders_ = p.as_bool();
                else if (n == "pid_tune_save_path") pid_tune_save_path_ = p.as_string();
                else {
                    RCLCPP_DEBUG(this->get_logger(), "[pidtest] 忽略未声明参数: %s", n.c_str());
                }
            } catch (const rclcpp::ParameterTypeException& e) {
                result.successful = false;
                result.reason = e.what();
                return result;
            }
        }
        return result;
    }

    void warn_no_camera_frame_() {
        if (!control_enabled_) return;
        const double t = this->get_clock()->now().seconds();
        if (t - last_no_image_warn_sec_ < 5.0) return;
        last_no_image_warn_sec_ = t;
        RCLCPP_WARN(this->get_logger(), "[pidtest] 无有效相机帧");
    }

    /** 从滑块读入内存（每帧在 process_image 开头调用，再算 PID，保证本帧控制用当前滑块值）。 */
    void pid_tune_apply_trackbars_to_members_() {
        if (!pid_tune_sliders_ready_) return;
        int p = cv::getTrackbarPos("kp_", kPidTuneWinBase);
        kp_ = static_cast<double>(std::clamp(p, 0, 400)) * 0.0001;
        p = cv::getTrackbarPos("kd_", kPidTuneWinBase);
        kd_ = static_cast<double>(std::clamp(p, 0, 120)) * 0.0001;
        p = cv::getTrackbarPos("ki_yaw_", kPidTuneWinBase);
        ki_yaw_ = static_cast<double>(std::clamp(p, 0, 100)) * 0.000001;
        p = cv::getTrackbarPos("ki_pitch_", kPidTuneWinBase);
        ki_pitch_ = static_cast<double>(std::clamp(p, 0, 100)) * 0.000001;

        p = cv::getTrackbarPos("err_lp_beta_yaw_", kPidTuneWinFilter);
        err_lp_beta_yaw_ = static_cast<double>(std::clamp(p, 50, 95)) * 0.01;
        p = cv::getTrackbarPos("err_lp_beta_pitch_", kPidTuneWinFilter);
        err_lp_beta_pitch_ = static_cast<double>(std::clamp(p, 50, 95)) * 0.01;
        p = cv::getTrackbarPos("d_lp_beta_yaw_", kPidTuneWinFilter);
        d_lp_beta_yaw_ = static_cast<double>(std::clamp(p, 10, 80)) * 0.01;
        p = cv::getTrackbarPos("d_lp_beta_pitch_", kPidTuneWinFilter);
        d_lp_beta_pitch_ = static_cast<double>(std::clamp(p, 10, 80)) * 0.01;

        p = cv::getTrackbarPos("deadband_px_", kPidTuneWinDeadband);
        deadband_px_ = std::clamp(p, 0, 10);
        p = cv::getTrackbarPos("deadband_yaw_px_", kPidTuneWinDeadband);
        deadband_yaw_px_ = std::clamp(p, 0, 10);
        p = cv::getTrackbarPos("settle_px_", kPidTuneWinDeadband);
        settle_px_ = static_cast<double>(std::clamp(p, 2, 30));
        p = cv::getTrackbarPos("settle_frames_", kPidTuneWinDeadband);
        settle_frames_ = std::clamp(p, 1, 15);

        p = cv::getTrackbarPos("servo_angle_smooth_beta_", kPidTuneWinOutput);
        servo_angle_smooth_beta_ = static_cast<double>(std::clamp(p, 70, 99)) * 0.01;
        p = cv::getTrackbarPos("max_delta_deg_", kPidTuneWinOutput);
        max_delta_deg_ = static_cast<double>(std::clamp(p, 30, 200)) * 0.01;
        p = cv::getTrackbarPos("yaw_gain_", kPidTuneWinOutput);
        yaw_gain_ = static_cast<double>(std::clamp(p, 50, 400)) * 0.01;
        p = cv::getTrackbarPos("yaw_kp_mult_", kPidTuneWinOutput);
        yaw_kp_mult_ = static_cast<double>(std::clamp(p, 50, 400)) * 0.01;
    }

    void save_pid_tune_params_to_file_() {
        const double t = this->get_clock()->now().seconds();
        if (t - last_pid_tune_save_sec_ < 0.35) return;
        last_pid_tune_save_sec_ = t;

        try {
            if (pid_tune_sliders_ready_) {
                pid_tune_apply_trackbars_to_members_();
            }
        } catch (const cv::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "[pidtest] 保存前同步滑块失败: %s", e.what());
        }

        std::string path = pid_tune_save_path_;
        if (path.empty()) {
            if (const char* home = std::getenv("HOME")) {
                path = std::string(home) + "/.ros/pidtest_pid_tune_snapshot.yaml";
            } else {
                path = "pidtest_pid_tune_snapshot.yaml";
            }
        }
        {
            namespace fs = std::filesystem;
            const fs::path fp(path);
            if (fp.has_parent_path()) {
                std::error_code ec;
                fs::create_directories(fp.parent_path(), ec);
            }
        }

        std::ofstream out(path);
        if (!out) {
            RCLCPP_ERROR(this->get_logger(), "[pidtest] 无法写入 PID 参数文件: %s", path.c_str());
            return;
        }
        out << std::fixed;
        out << "# pidtest_node — 滑块调参快照（ROS2 参数名）。可合并进 launch 或 ros2 param load。\n";
        out << "# 按 s 保存时的内存值（若已开滑块窗口，保存前会再同步一次滑块）。\n";
        out << "pidtest_node:\n";
        out << "  ros__parameters:\n";
        auto w = [&out](const char* name, double v) {
            out << "    " << name << ": " << std::setprecision(12) << v << "\n";
        };
        auto wi = [&out](const char* name, int v) { out << "    " << name << ": " << v << "\n"; };
        auto wb = [&out](const char* name, bool v) {
            out << "    " << name << ": " << (v ? "true" : "false") << "\n";
        };
        w("kp", kp_);
        w("kd", kd_);
        w("ki_yaw", ki_yaw_);
        w("ki_pitch", ki_pitch_);
        w("err_lp_beta_yaw", err_lp_beta_yaw_);
        w("err_lp_beta_pitch", err_lp_beta_pitch_);
        w("d_lp_beta_yaw", d_lp_beta_yaw_);
        w("d_lp_beta_pitch", d_lp_beta_pitch_);
        wi("deadband_px", deadband_px_);
        wi("deadband_yaw_px", deadband_yaw_px_);
        w("settle_px", settle_px_);
        wi("settle_frames", settle_frames_);
        w("servo_angle_smooth_beta", servo_angle_smooth_beta_);
        w("max_delta_deg", max_delta_deg_);
        w("yaw_gain", yaw_gain_);
        w("yaw_kp_mult", yaw_kp_mult_);
        wb("yaw_quantize_nudge_enable", yaw_quantize_nudge_enable_);
        w("yaw_still_zone_hit_px", yaw_still_zone_hit_px_);
        w("yaw_still_zone_deadband_extra_px", yaw_still_zone_deadband_extra_px_);
        w("yaw_still_zone_kd_scale", yaw_still_zone_kd_scale_);
        w("yaw_min_step_min_hit_px", yaw_min_step_min_hit_px_);
        w("yaw_still_hysteresis_px", yaw_still_hysteresis_px_);
        out.flush();
        RCLCPP_INFO(
            this->get_logger(), "[pidtest] 已保存当前 PID 调参到 %s （可按此文件恢复参数）", path.c_str());
    }

    /** OpenCV 高 GUI 事件泵；Esc 退出；s 保存当前 PID 参数。 */
    void pump_opencv_highgui_() {
        if (!show_vis_window_ && !pid_tune_sliders_) return;
        try {
            const int key = cv::waitKey(1);
            if (key < 0) return;
            const unsigned char c = static_cast<unsigned char>(key & 0xff);
            if (c == 27) {
                rclcpp::shutdown();
                return;
            }
            if (c == 's' || c == 'S') save_pid_tune_params_to_file_();
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000, "OpenCV GUI (waitKey): %s", e.what());
        }
    }

    void pid_tune_sliders_init_() {
        if (!pid_tune_sliders_ || pid_tune_sliders_ready_) return;
        try {
            if (const char* disp = std::getenv("DISPLAY")) {
                RCLCPP_INFO(this->get_logger(), "[pidtest] OpenCV 窗口 DISPLAY=%s", disp);
            } else {
                RCLCPP_WARN(
                    this->get_logger(),
                    "[pidtest] 未设置 DISPLAY，PID 滑块/Square Tracker 可能无法显示；"
                    "请在有桌面的终端 export DISPLAY=:0（或 echo $DISPLAY）后启动");
            }
            static std::once_flag k_win_thread;
            std::call_once(k_win_thread, [] { cv::startWindowThread(); });
            pid_tune_blank_ = cv::Mat(100, 420, CV_8UC3, cv::Scalar(40, 40, 40));
            cv::namedWindow(kPidTuneWinBase, cv::WINDOW_NORMAL);
            cv::namedWindow(kPidTuneWinFilter, cv::WINDOW_NORMAL);
            cv::namedWindow(kPidTuneWinDeadband, cv::WINDOW_NORMAL);
            cv::namedWindow(kPidTuneWinOutput, cv::WINDOW_NORMAL);
            cv::createTrackbar("kp_", kPidTuneWinBase, nullptr, 400, nullptr);
            cv::createTrackbar("kd_", kPidTuneWinBase, nullptr, 120, nullptr);
            cv::createTrackbar("ki_yaw_", kPidTuneWinBase, nullptr, 100, nullptr);
            cv::createTrackbar("ki_pitch_", kPidTuneWinBase, nullptr, 100, nullptr);
            cv::setTrackbarPos("kp_", kPidTuneWinBase, bar_from_scaled(kp_, 0.0001, 0, 400));
            cv::setTrackbarPos("kd_", kPidTuneWinBase, bar_from_scaled(kd_, 0.0001, 0, 120));
            cv::setTrackbarPos("ki_yaw_", kPidTuneWinBase, bar_from_scaled(ki_yaw_, 0.000001, 0, 100));
            cv::setTrackbarPos("ki_pitch_", kPidTuneWinBase, bar_from_scaled(ki_pitch_, 0.000001, 0, 100));

            cv::createTrackbar("err_lp_beta_yaw_", kPidTuneWinFilter, nullptr, 95, nullptr);
            cv::createTrackbar("err_lp_beta_pitch_", kPidTuneWinFilter, nullptr, 95, nullptr);
            cv::createTrackbar("d_lp_beta_yaw_", kPidTuneWinFilter, nullptr, 80, nullptr);
            cv::createTrackbar("d_lp_beta_pitch_", kPidTuneWinFilter, nullptr, 80, nullptr);
            cv::setTrackbarPos(
                "err_lp_beta_yaw_", kPidTuneWinFilter, bar_from_scaled(err_lp_beta_yaw_, 0.01, 50, 95));
            cv::setTrackbarPos(
                "err_lp_beta_pitch_", kPidTuneWinFilter, bar_from_scaled(err_lp_beta_pitch_, 0.01, 50, 95));
            cv::setTrackbarPos("d_lp_beta_yaw_", kPidTuneWinFilter, bar_from_scaled(d_lp_beta_yaw_, 0.01, 10, 80));
            cv::setTrackbarPos(
                "d_lp_beta_pitch_", kPidTuneWinFilter, bar_from_scaled(d_lp_beta_pitch_, 0.01, 10, 80));

            cv::createTrackbar("deadband_px_", kPidTuneWinDeadband, nullptr, 10, nullptr);
            cv::createTrackbar("deadband_yaw_px_", kPidTuneWinDeadband, nullptr, 10, nullptr);
            cv::createTrackbar("settle_px_", kPidTuneWinDeadband, nullptr, 30, nullptr);
            cv::createTrackbar("settle_frames_", kPidTuneWinDeadband, nullptr, 15, nullptr);
            cv::setTrackbarPos("deadband_px_", kPidTuneWinDeadband, std::clamp(deadband_px_, 0, 10));
            cv::setTrackbarPos("deadband_yaw_px_", kPidTuneWinDeadband, std::clamp(deadband_yaw_px_, 0, 10));
            cv::setTrackbarPos("settle_px_", kPidTuneWinDeadband, bar_from_scaled(settle_px_, 1.0, 2, 30));
            cv::setTrackbarPos("settle_frames_", kPidTuneWinDeadband, std::clamp(settle_frames_, 1, 15));

            cv::createTrackbar("servo_angle_smooth_beta_", kPidTuneWinOutput, nullptr, 99, nullptr);
            cv::createTrackbar("max_delta_deg_", kPidTuneWinOutput, nullptr, 200, nullptr);
            cv::createTrackbar("yaw_gain_", kPidTuneWinOutput, nullptr, 400, nullptr);
            cv::createTrackbar("yaw_kp_mult_", kPidTuneWinOutput, nullptr, 400, nullptr);
            cv::setTrackbarPos(
                "servo_angle_smooth_beta_", kPidTuneWinOutput,
                bar_from_scaled(servo_angle_smooth_beta_, 0.01, 70, 99));
            cv::setTrackbarPos(
                "max_delta_deg_", kPidTuneWinOutput, bar_from_scaled(max_delta_deg_, 0.01, 30, 200));
            cv::setTrackbarPos("yaw_gain_", kPidTuneWinOutput, bar_from_scaled(yaw_gain_, 0.01, 50, 400));
            cv::setTrackbarPos("yaw_kp_mult_", kPidTuneWinOutput, bar_from_scaled(yaw_kp_mult_, 0.01, 50, 400));

            pid_tune_sliders_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "[pidtest] PID 调参滑块窗口已创建（4 类）");
            RCLCPP_INFO(
                this->get_logger(),
                "[pidtest] 滑块值每帧在控制计算前读入；按 **s** 保存当前参数到文件（默认 ~/.ros/pidtest_pid_tune_snapshot.yaml）");
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[pidtest] pid_tune_sliders_init_: %s", e.what());
        }
    }

    void pid_tune_sliders_update_() {
        if (!pid_tune_sliders_ || !pid_tune_sliders_ready_) return;
        try {
            pid_tune_apply_trackbars_to_members_();

            cv::imshow(kPidTuneWinBase, pid_tune_blank_);
            cv::imshow(kPidTuneWinFilter, pid_tune_blank_);
            cv::imshow(kPidTuneWinDeadband, pid_tune_blank_);
            cv::imshow(kPidTuneWinOutput, pid_tune_blank_);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 3000, "[pidtest] pid_tune_sliders_update_: %s", e.what());
        }
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge failed: %s", e.what());
            warn_no_camera_frame_();
            return;
        }
        if (cv_ptr->image.empty()) {
            warn_no_camera_frame_();
            return;
        }
        frame_count_++;
        const rclcpp::Time now = this->get_clock()->now();
        const double dt_print = (now - last_frame_time_).seconds();
        if (dt_print >= 1.0) {
            current_fps_ = frame_count_ / dt_print;
            frame_count_ = 0;
            last_frame_time_ = now;
            RCLCPP_INFO(this->get_logger(), "FPS=%.1f", current_fps_);
        }
        process_image(cv_ptr->image);
    }

    std::vector<std::pair<int, int>> process_image(const cv::Mat& src) {
        std::vector<std::pair<int, int>> out;
        frame_center_lab_cid_ = -1;

        const double t_wall = this->get_clock()->now().seconds();
        float dt = 0.033f;
        if (last_proc_wall_sec_ > 1e-6) {
            dt = static_cast<float>(t_wall - last_proc_wall_sec_);
            dt = std::clamp(dt, 1.0f / 120.0f, 0.15f);
        }
        last_proc_wall_sec_ = t_wall;

        // 滑块→成员变量在本帧最前完成，后续整帧 PID/舵机均使用当前滑块值。
        if (pid_tune_sliders_) {
            if (!pid_tune_sliders_ready_) pid_tune_sliders_init_();
            pid_tune_sliders_update_();
        }

        static float servo_pitch_f = 90.f;
        static float servo_yaw_f = 90.f;
        static bool servo_init_once = false;
        if (!servo_init_once) {
            servo_yaw_f = static_cast<float>(std::clamp(init_yaw_deg_, servo_yaw_min_deg_, servo_yaw_max_deg_));
            servo_pitch_f =
                static_cast<float>(std::clamp(init_pitch_deg_, servo_pitch_min_deg_, servo_pitch_max_deg_));
            startup_countdown_ = std::max(0, startup_hold_frames_);
            servo_init_once = true;
        }

        const float roi_f = static_cast<float>(std::clamp(roi_frac_, 0.55, 0.99));
        const int roi_w = static_cast<int>(src.cols * roi_f);
        const int roi_h = static_cast<int>(src.rows * roi_f);
        const cv::Rect roi_rect((src.cols - roi_w) / 2, (src.rows - roi_h) / 2, roi_w, roi_h);
        cv::Rect safe = roi_rect & cv::Rect(0, 0, src.cols, src.rows);
        if (safe.width <= 20 || safe.height <= 20) {
            out.push_back({servo_id_pitch_, wrap_deg_0_359(safe_round_servo_deg(servo_pitch_f))});
            out.push_back({servo_id_yaw_, wrap_deg_0_359(safe_round_servo_deg(servo_yaw_f))});
            send_servo_angle(out);
            pump_opencv_highgui_();
            return out;
        }

        cv::Mat vis = src(safe).clone();
        const float zoom_use = static_cast<float>(std::clamp(center_roi_zoom_, 1.0, 1.6));
        center_crop_zoom_inplace(vis, zoom_use);
        apply_gamma_bgr_inplace(vis, vis_input_gamma_);
        apply_clahe_lab_l_inplace(vis, vis_clahe_l_clip_limit_);

        if (suppress_bright_lights_) {
            (void)suppress_bright_lights_inplace(
                vis, std::clamp(bright_hsv_v_min_, 1, 255), std::clamp(bright_hsv_s_max_, 0, 255),
                std::clamp(bright_bgr_channel_min_, 1, 255), std::max(0, bright_mask_dilate_),
                std::clamp(bright_inpaint_radius_, 1, 21),
                std::clamp(bright_inpaint_max_area_frac_, 0.003, 0.08));
        }

        const cv::Point2f aim_center(
            static_cast<float>(vis.cols) * 0.5f, static_cast<float>(vis.rows) * 0.5f);

        cv::Mat work = vis;
        float scale = 1.0f;
        if (vis.cols > trace_proc_width_cap_) {
            scale = static_cast<float>(trace_proc_width_cap_) / static_cast<float>(vis.cols);
            cv::resize(vis, work, cv::Size(), scale, scale, cv::INTER_AREA);
        }

        const float lab_t = static_cast<float>(lab_thresh_);
        cv::Mat lab_src = work;
        cv::Mat blur_store;
        if (lab_preprocess_blur_ksize_ >= 3) {
            const int kb = std::max(3, lab_preprocess_blur_ksize_ | 1);
            cv::GaussianBlur(work, blur_store, cv::Size(kb, kb), 0.0);
            lab_src = blur_store;
        }
        cv::Mat lab;
        cv::cvtColor(lab_src, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Vec3f> ref_lab;
        std::vector<int> ref_ids;
        for (const auto& kv : kReferenceBgr) {
            cv::Mat px(1, 1, CV_8UC3, kv.second);
            cv::Mat lpx;
            cv::cvtColor(px, lpx, cv::COLOR_BGR2Lab);
            cv::Vec3b lv = lpx.at<cv::Vec3b>(0, 0);
            ref_lab.emplace_back(lv[0], lv[1], lv[2]);
            ref_ids.push_back(kv.first);
        }
        cv::Mat labels(lab.rows, lab.cols, CV_32S, cv::Scalar(-1));
        const float lab_t2 = lab_t * lab_t;
        const size_t nref = ref_lab.size();
        cv::parallel_for_(cv::Range(0, lab.rows), [&](const cv::Range& range) {
            for (int y = range.start; y < range.end; ++y) {
                const cv::Vec3b* pr = lab.ptr<cv::Vec3b>(y);
                int* ow = labels.ptr<int>(y);
                for (int x = 0; x < lab.cols; ++x) {
                    const float c0 = static_cast<float>(pr[x][0]);
                    const float c1 = static_cast<float>(pr[x][1]);
                    const float c2 = static_cast<float>(pr[x][2]);
                    float best_d2 = 1e18f;
                    int best_id = -1;
                    for (size_t i = 0; i < nref; ++i) {
                        const cv::Vec3f& r = ref_lab[i];
                        const float d0 = c0 - r[0];
                        const float d1 = c1 - r[1];
                        const float d2c = c2 - r[2];
                        const float d2 = d0 * d0 + d1 * d1 + d2c * d2c;
                        if (d2 < best_d2) {
                            best_d2 = d2;
                            best_id = ref_ids[i];
                        }
                    }
                    if (best_d2 <= lab_t2) ow[x] = best_id;
                }
            }
        });

        cv::Mat canny_edges;
        if (lab_mask_canny_bridge_ && work.cols > 8 && work.rows > 8) {
            cv::Mat gray;
            cv::cvtColor(work, gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.0);
            const double t1 = static_cast<double>(lab_canny_thresh1_);
            const double t2 = static_cast<double>(std::max(lab_canny_thresh2_, lab_canny_thresh1_ + 1));
            cv::Mat ed;
            cv::Canny(gray, ed, t1, t2);
            const int dpx = lab_canny_dilate_px_ | 1;
            const cv::Mat dk = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dpx, dpx));
            cv::dilate(ed, canny_edges, dk);
        }

        const double min_area = std::max(10.0, lab_min_area_seg_ * 0.45);
        const int mask_k = std::max(3, lab_mask_morph_ksize_ | 1);
        constexpr int kMaxBlobsPerId = 6;

        std::vector<Blob> blobs;
        blobs.reserve(64);
        for (const auto& kv : kReferenceBgr) {
            const int cid = kv.first;
            cv::Mat mask = (labels == cid);
            mask.convertTo(mask, CV_8U, 255.0);
            mask = refine_mask(mask, mask_k);
            if (!canny_edges.empty()) {
                const int br = lab_canny_bridge_mask_dilate_;
                cv::Mat k5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
                cv::Mat md;
                cv::dilate(mask, md, k5, cv::Point(-1, -1), br);
                cv::Mat add;
                cv::bitwise_and(canny_edges, md, add);
                cv::bitwise_or(mask, add, mask);
            }
            std::vector<std::vector<cv::Point>> cnts;
            cv::findContours(mask, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            std::vector<std::pair<double, std::vector<cv::Point>>> scored;
            for (const auto& c : cnts) {
                double a = cv::contourArea(c);
                if (a < min_area) continue;
                scored.push_back({a, c});
            }
            std::sort(scored.begin(), scored.end(), [](const auto& l, const auto& r) { return l.first > r.first; });
            const int keep = std::min(kMaxBlobsPerId, static_cast<int>(scored.size()));
            for (int i = 0; i < keep; ++i) {
                const auto& c = scored[i].second;
                const double a = scored[i].first;
                cv::Moments m = cv::moments(c);
                if (std::abs(m.m00) < 1e-6) continue;
                Blob b;
                b.cid = cid;
                b.area = a;
                b.cx = m.m10 / m.m00;
                b.cy = m.m01 / m.m00;
                b.circularity = contour_circularity(c);
                b.shape = classify_shape(b.circularity, c);
                b.approx_vert = contour_approx_vertex_count(c, square_approx_poly_eps_);
                b.contour = c;
                blobs.push_back(std::move(b));
            }
        }

        const double frame_area = static_cast<double>(work.cols) * static_cast<double>(work.rows);

        Blob* target = pick_largest_quad_for_color(
            blobs, target_color_id_, sector_min_area_, sector_max_area_ratio_, frame_area, square_approx_poly_eps_);
        if (!target && track_fallback_to_largest_color_blob_) {
            target = pick_largest_blob_for_color(
                blobs, target_color_id_, sector_min_area_, sector_max_area_ratio_, frame_area);
        }
        frame_center_lab_cid_ = target ? target_color_id_ : -1;

        cv::Point2f target_raw(-1, -1);
        if (target) {
            target_raw = cv::Point2f(static_cast<float>(target->cx / scale), static_cast<float>(target->cy / scale));
        }
        const bool detect_ok_raw = (target_raw.x >= 0.0f);
        if (detect_ok_raw) {
            held_target_ = target_raw;
            held_target_valid_ = true;
            target_hold_lost_count_ = 0;
        } else if (held_target_valid_ && target_hold_lost_count_ < target_hold_frames_) {
            // 丢目标短窗口内沿用上次目标，避免瞬时误检导致抖动和频繁切搜索状态。
            target_raw = held_target_;
            target_hold_lost_count_ += 1;
        } else {
            held_target_valid_ = false;
        }

        const float alpha_track = 0.52f;
        const float alpha_recover = 0.46f;
        float target_step_max = static_cast<float>(target_step_max_px_);
        if (target_raw.x >= 0.0f) {
            const int lost_before = track_.miss_count;
            track_.miss_count = 0;
            cv::Point2f pred = target_raw;
            cv::Point2f inst{(target_raw.x - track_.prev_raw.x) / std::max(1e-4f, dt),
                (target_raw.y - track_.prev_raw.y) / std::max(1e-4f, dt)};
            inst.x = std::clamp(inst.x, -950.0f, 950.0f);
            inst.y = std::clamp(inst.y, -950.0f, 950.0f);
            const float vb = static_cast<float>(predict_vel_beta_);
            vel_ema_.x = vb * inst.x + (1.0f - vb) * vel_ema_.x;
            vel_ema_.y = vb * inst.y + (1.0f - vb) * vel_ema_.y;

            if (track_.has_smooth_target) {
                const cv::Point2f dp = pred - track_.smooth_target;
                const float dpm = std::hypot(dp.x, dp.y);
                if (dpm > target_step_max && dpm > 1e-3f) {
                    const float s = target_step_max / dpm;
                    pred = track_.smooth_target + dp * s;
                }
            }
            track_.prev_raw = target_raw;
            track_.has_prev_raw = true;

            if (!track_.has_smooth_target) {
                track_.smooth_target = pred;
                track_.has_smooth_target = true;
            } else {
                float a = (lost_before > 0) ? alpha_recover : alpha_track;
                const float el = std::hypot(target_raw.x - aim_center.x, target_raw.y - aim_center.y);
                const float jump_thresh = 82.0f;
                const cv::Point2f djump = pred - track_.smooth_target;
                const float ej = std::hypot(djump.x, djump.y);
                if (ej > jump_thresh) {
                    a = std::min(0.80f, alpha_track + 0.18f);
                } else if (el < 24.0f) {
                    a = 0.38f;
                } else if (el < 52.0f) {
                    a = 0.44f;
                } else if (el < 100.0f) {
                    a = 0.50f;
                } else {
                    a = 0.56f;
                }
                if (lost_before > 0) a = std::max(a, alpha_recover);
                a *= static_cast<float>(smooth_target_alpha_scale_);
                a = std::clamp(a, 0.18f, 0.68f);
                track_.smooth_target = a * pred + (1.0f - a) * track_.smooth_target;
            }
            predict_good_streak_ += 1;
            if (predict_good_streak_ >= predict_arm_frames_) predict_armed_ = true;
        } else if (track_.has_smooth_target) {
            track_.miss_count += 1;
            const float vel_m = std::hypot(vel_ema_.x, vel_ema_.y);
            const bool in_miss_window =
                predict_armed_ && track_.miss_count <= predict_coast_frames_;
            const bool sq_coast = in_miss_window && vel_m > 0.22f;
            if (sq_coast) {
                const float ple = static_cast<float>(predict_lead_ms_) / 1000.0f;
                cv::Point2f st = track_.smooth_target + vel_ema_ * ple;
                st.x = std::clamp(st.x, 0.0f, static_cast<float>(vis.cols - 1));
                st.y = std::clamp(st.y, 0.0f, static_cast<float>(vis.rows - 1));
                track_.smooth_target = 0.58f * st + 0.42f * track_.smooth_target;
                vel_ema_.x *= 0.988f;
                vel_ema_.y *= 0.988f;
            } else if (!in_miss_window) {
                track_.has_prev_raw = false;
                predict_armed_ = false;
                predict_good_streak_ = 0;
                vel_ema_.x = vel_ema_.y = 0.f;
            }
        }

        const bool allow_predict_coast =
            predict_armed_ && target_raw.x < 0.0f && track_.has_smooth_target && track_.miss_count > 0 &&
            track_.miss_count <= predict_coast_frames_;

        const cv::Point2f target_point = track_.has_smooth_target ? track_.smooth_target : aim_center;
        const int x_error = static_cast<int>(std::lround(target_point.x - aim_center.x));
        const int y_error = static_cast<int>(std::lround(target_point.y - aim_center.y));
        const float hit_error_px = std::sqrt(static_cast<float>(x_error * x_error + y_error * y_error));

        const bool detect_ok_control = (target_raw.x >= 0.0f);
        if (detect_ok_control) {
            acquire_lost_frames_ = 0;
        } else if (!allow_predict_coast) {
            acquire_lost_frames_++;
        }

        const bool search_mode =
            !allow_predict_coast && ((track_.miss_count >= 3) || (acquire_lost_frames_ >= search_enter_frames_));

        settle_ok_count_ =
            (detect_ok_control && hit_error_px <= static_cast<float>(settle_px_)) ? (settle_ok_count_ + 1) : 0;
        const bool should_hold = (settle_ok_count_ >= settle_frames_);

        float kp_pitch = static_cast<float>(kp_);
        float kp_yaw = kp_pitch * static_cast<float>(yaw_kp_mult_);
        float kd_yaw = static_cast<float>(kd_);
        float kd_pitch = kd_yaw;
        // [MOD-2] 误差一阶低通参数（可配置），用于平滑误差信号后再进入 PID。
        float beta_ex = static_cast<float>(std::clamp(err_lp_beta_yaw_, 0.0, 1.0));
        float beta_ey = static_cast<float>(std::clamp(err_lp_beta_pitch_, 0.0, 1.0));
        float max_delta_pitch_deg = static_cast<float>(max_delta_deg_);
        float max_delta_yaw_deg = max_delta_pitch_deg;
        int deadband_pitch = deadband_px_;
        int deadband_yaw = std::max(1, std::min(deadband_pitch, deadband_yaw_px_));

        float yaw_lo = static_cast<float>(servo_yaw_min_deg_);
        float yaw_hi = static_cast<float>(servo_yaw_max_deg_);
        float pitch_lo = static_cast<float>(servo_pitch_min_deg_);
        float pitch_hi = static_cast<float>(servo_pitch_max_deg_);
        if (servo_yaw_travel_half_span_deg_ > 0.5) {
            const float c = static_cast<float>(init_yaw_deg_);
            const float h = static_cast<float>(servo_yaw_travel_half_span_deg_);
            yaw_lo = std::max(yaw_lo, c - h);
            yaw_hi = std::min(yaw_hi, c + h);
        }
        if (servo_pitch_travel_half_span_deg_ > 0.5) {
            const float c = static_cast<float>(init_pitch_deg_);
            const float h = static_cast<float>(servo_pitch_travel_half_span_deg_);
            pitch_lo = std::max(pitch_lo, c - h);
            pitch_hi = std::min(pitch_hi, c + h);
        }
        if (yaw_lo > yaw_hi) std::swap(yaw_lo, yaw_hi);
        if (pitch_lo > pitch_hi) std::swap(pitch_lo, pitch_hi);

        if (hit_error_px < 40.0f) {
            kp_yaw *= static_cast<float>(near_tgt_yaw_kp_scale_);
            kd_yaw *= 1.08f;
            float lp_near = 0.88f;
            if (std::abs(static_cast<float>(x_error)) > 18.0f) lp_near = 0.97f;
            beta_ex *= lp_near;
            beta_ey *= lp_near;
            max_delta_yaw_deg *= 0.90f;
            deadband_pitch = std::max(2, deadband_pitch - 1);
            deadband_yaw = std::max(1, deadband_yaw - 1);
        }
        // [MOD-2] 对误差做一阶低通滤波，后续 PID 全部使用滤波后的误差。
        ctrl_filt_ex_ = beta_ex * static_cast<float>(x_error) + (1.0f - beta_ex) * ctrl_filt_ex_;
        ctrl_filt_ey_ = beta_ey * static_cast<float>(y_error) + (1.0f - beta_ey) * ctrl_filt_ey_;

        const float yaw_dead_extra =
            (hit_error_px < static_cast<float>(yaw_still_zone_hit_px_))
                ? static_cast<float>(yaw_still_zone_deadband_extra_px_)
                : 0.f;
        const float yaw_db_total = static_cast<float>(deadband_yaw) + yaw_dead_extra;
        const bool freeze_was = yaw_deadband_freeze_;
        if (search_mode || !detect_ok_control) {
            yaw_deadband_freeze_ = false;
        } else {
            const float yaw_db_release = yaw_db_total + static_cast<float>(yaw_still_hysteresis_px_);
            if (!yaw_deadband_freeze_ && std::abs(ctrl_filt_ex_) < yaw_db_total) {
                yaw_deadband_freeze_ = true;
            }
            if (yaw_deadband_freeze_ && std::abs(ctrl_filt_ex_) > yaw_db_release) {
                yaw_deadband_freeze_ = false;
            }
        }
        float err_used_x = 0.f;
        if (!yaw_deadband_freeze_ && std::abs(ctrl_filt_ex_) > yaw_db_total) {
            err_used_x = ctrl_filt_ex_;
        }
        if (freeze_was && !yaw_deadband_freeze_) {
            ctrl_prev_ex_ = err_used_x;
        }
        const bool freeze_enter_edge = !freeze_was && yaw_deadband_freeze_;
        if (freeze_enter_edge) {
            ctrl_prev_ex_ = err_used_x;
            ctrl_dfilt_ex_ *= 0.15f;
        }
        const float err_used_y = (std::abs(ctrl_filt_ey_) > static_cast<float>(deadband_pitch)) ? ctrl_filt_ey_ : 0.0f;
        float de_x_raw = (err_used_x - ctrl_prev_ex_) / std::max(1e-3f, dt);
        float de_y_raw = (err_used_y - ctrl_prev_ey_) / std::max(1e-3f, dt);
        const float d_beta_x = static_cast<float>(std::clamp(d_lp_beta_yaw_, 0.0, 1.0));
        const float d_beta_y = static_cast<float>(std::clamp(d_lp_beta_pitch_, 0.0, 1.0));
        ctrl_dfilt_ex_ = d_beta_x * de_x_raw + (1.0f - d_beta_x) * ctrl_dfilt_ex_;
        ctrl_dfilt_ey_ = d_beta_y * de_y_raw + (1.0f - d_beta_y) * ctrl_dfilt_ey_;
        const float de_x = ctrl_dfilt_ex_;
        const float de_y = ctrl_dfilt_ey_;
        ctrl_prev_ex_ = err_used_x;
        ctrl_prev_ey_ = err_used_y;

        float ki_yaw = static_cast<float>(ki_yaw_) * static_cast<float>(yaw_kp_mult_);
        float ki_pitch = static_cast<float>(ki_pitch_);
        const bool integ_run =
            detect_ok_control && !search_mode && !should_hold && !yaw_deadband_freeze_;
        if (integ_run) {
            if (std::abs(err_used_x) > 1e-6f) ctrl_integ_ex_ += err_used_x * dt;
            if (std::abs(err_used_y) > 1e-6f) ctrl_integ_ey_ += err_used_y * dt;
            const float lim = static_cast<float>(ctrl_integ_max_px_s_);
            ctrl_integ_ex_ = std::clamp(ctrl_integ_ex_, -lim, lim);
            ctrl_integ_ey_ = std::clamp(ctrl_integ_ey_, -lim, lim);
        } else {
            ctrl_integ_ex_ *= 0.88f;
            ctrl_integ_ey_ *= 0.88f;
        }

        const float kd_yaw_eff =
            kd_yaw * ((hit_error_px < static_cast<float>(yaw_still_zone_hit_px_))
                          ? static_cast<float>(yaw_still_zone_kd_scale_)
                          : 1.f);

        float u_yaw = 0.f;
        float u_pitch = 0.f;
        if (std::abs(err_used_x) > 1e-6f) {
            u_yaw = -(kp_yaw * err_used_x + kd_yaw_eff * de_x + ki_yaw * ctrl_integ_ex_);
        } else if (std::abs(ki_yaw * ctrl_integ_ex_) > 1e-6f) {
            u_yaw = -(ki_yaw * ctrl_integ_ex_);
        }
        if (std::abs(err_used_y) > 1e-6f) {
            u_pitch = +(kp_pitch * err_used_y + kd_pitch * de_y + ki_pitch * ctrl_integ_ey_);
        } else if (std::abs(ki_pitch * ctrl_integ_ey_) > 1e-6f) {
            u_pitch = +(ki_pitch * ctrl_integ_ey_);
        }
        u_yaw *= static_cast<float>(yaw_gain_);
        const float u_yaw_pre_limit = u_yaw;
        const float u_pitch_pre_limit = u_pitch;
        u_yaw = std::clamp(u_yaw, -max_delta_yaw_deg, max_delta_yaw_deg);
        u_pitch = std::clamp(u_pitch, -max_delta_pitch_deg, max_delta_pitch_deg);
        // 供 /tracking_debug：PID 限幅后、模式分支前的 yaw 增量（度/帧）
        const float dbg_u_yaw_after_pid_rate_limit = u_yaw;
        const float g_yaw = static_cast<float>(yaw_gain_);
        float dbg_yaw_p = 0.f, dbg_yaw_d = 0.f, dbg_yaw_i = 0.f;
        if (std::abs(err_used_x) > 1e-6f) {
            dbg_yaw_p = -(kp_yaw * err_used_x) * g_yaw;
            dbg_yaw_d = -(kd_yaw_eff * de_x) * g_yaw;
            dbg_yaw_i = -(ki_yaw * ctrl_integ_ex_) * g_yaw;
        } else if (std::abs(ki_yaw * ctrl_integ_ex_) > 1e-6f) {
            dbg_yaw_i = -(ki_yaw * ctrl_integ_ex_) * g_yaw;
        }
        if (integ_run) {
            ctrl_integ_ex_ += static_cast<float>(antiwindup_backcalc_gain_yaw_) * (u_yaw - u_yaw_pre_limit);
            ctrl_integ_ey_ += static_cast<float>(antiwindup_backcalc_gain_pitch_) * (u_pitch - u_pitch_pre_limit);
        }

        static float search_phase_yaw = 0.f;
        static float search_phase_pitch = 0.f;
        // 本帧实际叠到 servo_yaw_f 上的 yaw 增量（search/hold 为 0；供调试与 trace_calculator 区分）
        float dbg_u_yaw_applied_delta = 0.f;
        if (should_hold) {
            u_yaw = u_pitch = 0.f;
        } else if (search_mode) {
            const float yaw_c = std::clamp(static_cast<float>(init_yaw_deg_), yaw_lo, yaw_hi);
            const float pitch_c = std::clamp(static_cast<float>(init_pitch_deg_), pitch_lo, pitch_hi);
            const float yaw_amp = static_cast<float>(search_yaw_amp_deg_);
            const float pitch_amp = static_cast<float>(search_pitch_amp_deg_);
            search_phase_yaw += static_cast<float>(search_phase_step_);
            search_phase_pitch += static_cast<float>(search_pitch_phase_step_);
            const float two_pi = 2.0f * static_cast<float>(CV_PI);
            if (search_phase_yaw > two_pi) search_phase_yaw -= two_pi;
            if (search_phase_pitch > two_pi) search_phase_pitch -= two_pi;
            servo_yaw_f = yaw_c + yaw_amp * std::sin(search_phase_yaw);
            servo_pitch_f = pitch_c + pitch_amp * std::sin(search_phase_pitch);
            if (target_color_id_ >= 0) {
                Blob* aim = nullptr;
                double best_s = -1e9;
                for (auto& b : blobs) {
                    if (b.cid != target_color_id_) continue;
                    if (b.area < 42.0) continue;
                    double s = std::sqrt(std::max(1.0, b.area));
                    if (b.approx_vert >= 4 && b.approx_vert <= 10) s += 0.55;
                    if (s > best_s) {
                        best_s = s;
                        aim = &b;
                    }
                }
                if (aim) {
                    const float bx = static_cast<float>(aim->cx / scale);
                    const float by = static_cast<float>(aim->cy / scale);
                    const float ex_vis = bx - aim_center.x;
                    const float ey_vis = by - aim_center.y;
                    servo_yaw_f += std::clamp(-0.38f * ex_vis, -9.0f, 9.0f);
                    servo_pitch_f += std::clamp(0.09f * ey_vis, -8.0f, 8.0f);
                }
            }
        } else {
            const float min_step_y = static_cast<float>(yaw_min_step_deg_);
            if (!yaw_deadband_freeze_ && std::abs(u_yaw) < min_step_y &&
                std::abs(ctrl_filt_ex_) > yaw_db_total + 1e-3f &&
                hit_error_px > static_cast<float>(yaw_min_step_min_hit_px_)) {
                u_yaw = (ctrl_filt_ex_ > 0.0f) ? -min_step_y : min_step_y;
            }
            if (std::abs(u_pitch) < 0.35f && std::abs(ctrl_filt_ey_) > (deadband_pitch + 8)) {
                u_pitch = (u_pitch >= 0.0f) ? 0.35f : -0.35f;
            }
            const float yaw_next = servo_yaw_f + u_yaw;
            if (yaw_next < yaw_lo || yaw_next > yaw_hi) {
                const float u_yaw_sat = std::clamp(u_yaw, yaw_lo - servo_yaw_f, yaw_hi - servo_yaw_f);
                if (integ_run) {
                    ctrl_integ_ex_ +=
                        static_cast<float>(antiwindup_backcalc_gain_yaw_) * (u_yaw_sat - u_yaw);
                }
                u_yaw = u_yaw_sat;
            }
            const float pitch_next = servo_pitch_f + u_pitch;
            if (pitch_next < pitch_lo || pitch_next > pitch_hi) {
                const float u_pitch_sat = std::clamp(u_pitch, pitch_lo - servo_pitch_f, pitch_hi - servo_pitch_f);
                if (integ_run) {
                    ctrl_integ_ey_ +=
                        static_cast<float>(antiwindup_backcalc_gain_pitch_) * (u_pitch_sat - u_pitch);
                }
                u_pitch = u_pitch_sat;
            }
            dbg_u_yaw_applied_delta = u_yaw;
            servo_yaw_f += u_yaw;
            servo_pitch_f += u_pitch;
        }

        if (startup_countdown_ > 0) {
            if (!detect_ok_control && !search_mode) {
                servo_yaw_f = std::clamp(static_cast<float>(init_yaw_deg_), yaw_lo, yaw_hi);
                servo_pitch_f = std::clamp(static_cast<float>(init_pitch_deg_), pitch_lo, pitch_hi);
                servo_out_lp_inited_ = false;
            }
            startup_countdown_ -= 1;
        }
        servo_yaw_f = std::clamp(servo_yaw_f, yaw_lo, yaw_hi);
        servo_pitch_f = std::clamp(servo_pitch_f, pitch_lo, pitch_hi);
        if (!std::isfinite(servo_pitch_f)) servo_pitch_f = static_cast<float>(init_pitch_deg_);
        if (!std::isfinite(servo_yaw_f)) servo_yaw_f = static_cast<float>(init_yaw_deg_);

        {
            const float sb = search_mode ? 1.0f : static_cast<float>(std::clamp(servo_angle_smooth_beta_, 0.05, 0.95));
            if (!servo_out_lp_inited_) {
                servo_out_lp_pitch_ = servo_pitch_f;
                servo_out_lp_yaw_ = servo_yaw_f;
                servo_out_lp_inited_ = true;
            } else {
                servo_out_lp_pitch_ = sb * servo_pitch_f + (1.0f - sb) * servo_out_lp_pitch_;
                servo_out_lp_yaw_ = sb * servo_yaw_f + (1.0f - sb) * servo_out_lp_yaw_;
                servo_pitch_f = servo_out_lp_pitch_;
                servo_yaw_f = servo_out_lp_yaw_;
            }
        }

        const int center_id = target_color_id_;
        const int target_id = (target && detect_ok_raw) ? target->cid : -1;
        const bool detect_ok = detect_ok_raw;

        // 叠加层：仅画面中心十字（摄像头/瞄准中心）+ 目标轴对齐矩形（无额外文字与重复几何）
        {
            const int ccx = vis.cols / 2;
            const int ccy = vis.rows / 2;
            const int arm = std::max(16, std::min(vis.cols, vis.rows) / 18);
            const cv::Scalar cross_color(0, 255, 0);
            cv::line(vis, cv::Point(ccx - arm, ccy), cv::Point(ccx + arm, ccy), cross_color, 2, cv::LINE_AA);
            cv::line(vis, cv::Point(ccx, ccy - arm), cv::Point(ccx, ccy + arm), cross_color, 2, cv::LINE_AA);
        }
        {
            const float inv_s = (scale > 1e-6f) ? (1.0f / scale) : 1.0f;
            if (target && target->contour.size() >= 3) {
                const std::vector<cv::Point> tpts = contour_scaled_to_vis(target->contour, inv_s);
                const cv::Rect br = cv::boundingRect(tpts);
                cv::rectangle(vis, br, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            }
        }

        int pitch_cmd = wrap_deg_0_359(safe_round_servo_deg(servo_pitch_f));
        int yaw_cmd = wrap_deg_0_359(safe_round_servo_deg(servo_yaw_f));
        pitch_cmd = clamp_pitch_dual_window_deg(pitch_cmd, pitch_main_band_low_deg_, pitch_wrap_band_high_deg_);
        yaw_cmd = std::clamp(yaw_cmd, static_cast<int>(std::floor(yaw_lo)), static_cast<int>(std::ceil(yaw_hi)));

        if (control_enabled_ && detect_ok_control && !search_mode && !should_hold && yaw_quantize_nudge_enable_) {
            if (yaw_cmd == last_pub_yaw_cmd_) {
                yaw_cmd_stale_frames_ += 1;
            } else {
                yaw_cmd_stale_frames_ = 0;
            }
            // 仅在大偏差时微推，避免中心附近 ±1° 来回（极限环）。
            if (yaw_cmd_stale_frames_ >= 4 && std::abs(x_error) > 16 &&
                hit_error_px > static_cast<float>(yaw_still_zone_hit_px_) + 6.f &&
                std::abs(ctrl_filt_ex_) > yaw_db_total + 1.2f) {
                yaw_cmd += (ctrl_filt_ex_ > 0.0f) ? -1 : 1;
                yaw_cmd = std::clamp(yaw_cmd, static_cast<int>(std::floor(yaw_lo)), static_cast<int>(std::ceil(yaw_hi)));
                servo_yaw_f = static_cast<float>(yaw_cmd);
                yaw_cmd_stale_frames_ = 0;
            }
            last_pub_yaw_cmd_ = yaw_cmd;
        } else {
            yaw_cmd_stale_frames_ = 0;
            last_pub_yaw_cmd_ = yaw_cmd;
        }

        if (publish_trace_debug_image_) {
            std_msgs::msg::Header hdr;
            hdr.stamp = this->now();
            hdr.frame_id = "camera_optical_frame";
            try {
                trace_debug_pub_.publish(
                    cv_bridge::CvImage(hdr, sensor_msgs::image_encodings::BGR8, vis).toImageMsg());
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 3000, "trace_debug publish failed: %s", e.what());
            }
        }

        out.push_back({servo_id_pitch_, pitch_cmd});
        out.push_back({servo_id_yaw_, yaw_cmd});
        send_servo_angle(out);

        if (publish_tracking_debug_) {
            /* /tracking_debug Float64 布局（pidtest_node）：
             *  0 hit_error_px   1 x_error   2 y_error
             *  3 detect_ok(0/1) 4 should_hold  5 track_.miss_count
             *  6 servo_pitch_f  7 servo_yaw_f（平滑后、发角前）
             *  8 fps  9 control_enabled(0/1)  10 保留(0)
             * 11 ctrl_filt_ex_
             * 12 u_yaw：hold/search 时为 0（避免误用上一帧 PID 残量）
             * 13 pitch_cmd  14 yaw_cmd  15 pitch_cmd  16 yaw_cmd（历史重复，兼容旧脚本）
             * 17 center_id  18 target_id  19 frame_center_lab_cid_
             * 20 search_mode  21 yaw_deadband_freeze
             * 22 err_used_x  23 yaw_db_total  24 kp_yaw  25 kd_yaw_eff
             * 26 de_x  27 max_delta_yaw_deg
             * 28 dbg_u_yaw_after_pid_rate_limit（yaw_gain+每帧 max 限幅后）
             * 29 dbg_u_yaw_applied_delta（本帧 servo_yaw_f += 的量）
             * 30 dbg_yaw_p  31 dbg_yaw_d  32 dbg_yaw_i（均已乘 yaw_gain）
             */
            std_msgs::msg::Float64MultiArray dbg;
            dbg.data.resize(33);
            dbg.data[0] = static_cast<double>(hit_error_px);
            dbg.data[1] = static_cast<double>(x_error);
            dbg.data[2] = static_cast<double>(y_error);
            dbg.data[3] = detect_ok ? 1.0 : 0.0;
            dbg.data[4] = should_hold ? 1.0 : 0.0;
            dbg.data[5] = static_cast<double>(track_.miss_count);
            dbg.data[6] = static_cast<double>(servo_pitch_f);
            dbg.data[7] = static_cast<double>(servo_yaw_f);
            dbg.data[8] = static_cast<double>(current_fps_);
            dbg.data[9] = static_cast<double>(control_enabled_ ? 1.0 : 0.0);
            dbg.data[10] = 0.0;
            dbg.data[11] = static_cast<double>(ctrl_filt_ex_);
            float u_yaw_dbg_tail = u_yaw;
            if (should_hold || search_mode) u_yaw_dbg_tail = 0.f;
            dbg.data[12] = static_cast<double>(u_yaw_dbg_tail);
            dbg.data[13] = static_cast<double>(pitch_cmd);
            dbg.data[14] = static_cast<double>(yaw_cmd);
            dbg.data[15] = static_cast<double>(pitch_cmd);
            dbg.data[16] = static_cast<double>(yaw_cmd);
            dbg.data[17] = static_cast<double>(center_id);
            dbg.data[18] = static_cast<double>(target_id);
            dbg.data[19] = static_cast<double>(frame_center_lab_cid_);
            dbg.data[20] = search_mode ? 1.0 : 0.0;
            dbg.data[21] = yaw_deadband_freeze_ ? 1.0 : 0.0;
            dbg.data[22] = static_cast<double>(err_used_x);
            dbg.data[23] = static_cast<double>(yaw_db_total);
            dbg.data[24] = static_cast<double>(kp_yaw);
            dbg.data[25] = static_cast<double>(kd_yaw_eff);
            dbg.data[26] = static_cast<double>(de_x);
            dbg.data[27] = static_cast<double>(max_delta_yaw_deg);
            dbg.data[28] = static_cast<double>(dbg_u_yaw_after_pid_rate_limit);
            dbg.data[29] = static_cast<double>(dbg_u_yaw_applied_delta);
            dbg.data[30] = static_cast<double>(dbg_yaw_p);
            dbg.data[31] = static_cast<double>(dbg_yaw_d);
            dbg.data[32] = static_cast<double>(dbg_yaw_i);
            tracking_debug_pub_->publish(dbg);
        }
        if (publish_center_color_id_) {
            std_msgs::msg::Int32 cc;
            cc.data = center_id;
            center_color_pub_->publish(cc);
        }

        if (show_vis_window_) {
            try {
                cv::imshow("Square Tracker", vis);
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000, "OpenCV GUI (Square Tracker): %s", e.what());
            }
        }
        pump_opencv_highgui_();

        return out;
    }

    void send_servo_angle(const std::vector<std::pair<int, int>>& servo_controller) {
        if (!control_enabled_) return;
        for (const auto& servo : servo_controller) {
            if (servo.first != servo_id_pitch_ && servo.first != servo_id_yaw_) continue;
            int lo = 0, hi = 360;
            if (servo.first == servo_id_pitch_) {
                lo = static_cast<int>(std::lround(servo_pitch_min_deg_));
                hi = static_cast<int>(std::lround(servo_pitch_max_deg_));
            } else if (servo.first == servo_id_yaw_) {
                lo = static_cast<int>(std::lround(servo_yaw_min_deg_));
                hi = static_cast<int>(std::lround(servo_yaw_max_deg_));
            }
            if (lo > hi) std::swap(lo, hi);
            ServoMsg msg;
            msg.timestamp = this->now();
            msg.servo_id = servo.first;
            int cmd = wrap_deg_0_359(servo.second);
            if (servo.first == servo_id_pitch_) {
                cmd = clamp_pitch_dual_window_deg(cmd, pitch_main_band_low_deg_, pitch_wrap_band_high_deg_);
            }
            msg.servo_angle = std::clamp(cmd, lo, hi);
            servo_pub_->publish(msg);
        }
    }

    image_transport::Subscriber image_sub_;
    image_transport::Publisher trace_debug_pub_;
    std::string trace_debug_image_topic_;
    std::string image_topic_;
    bool publish_trace_debug_image_ = true;
    rclcpp::Publisher<ServoMsg>::SharedPtr servo_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr center_color_pub_;
    bool publish_tracking_debug_ = true;
    bool show_vis_window_ = false;
    bool pid_tune_sliders_ = false;
    bool pid_tune_sliders_ready_ = false;
    std::string pid_tune_save_path_;
    double last_pid_tune_save_sec_{-1e9};
    cv::Mat pid_tune_blank_;
    rclcpp::Time last_frame_time_;
    int frame_count_ = 0;
    double current_fps_ = 30.0;
    bool control_enabled_ = true;
    double last_no_image_warn_sec_{-1e9};
    int target_color_id_ = 0;
    bool track_fallback_to_largest_color_blob_ = true;
    double init_pitch_deg_ = 340.0;
    double init_yaw_deg_ = 170.0;
    int startup_hold_frames_ = 15;
    int target_hold_frames_ = 5;
    int startup_countdown_ = 0;
    int servo_id_pitch_ = 8;
    int servo_id_yaw_ = 11;
    double servo_pitch_min_deg_ = 320.0;
    double servo_pitch_max_deg_ = 360.0;
    double servo_yaw_min_deg_ = 110.0;
    double servo_yaw_max_deg_ = 225.0;
    double servo_pitch_travel_half_span_deg_ = 20.0;
    double servo_yaw_travel_half_span_deg_ = 60.0;
    int pitch_main_band_low_deg_ = 320;
    int pitch_wrap_band_high_deg_ = 359;
    int trace_proc_width_cap_ = 480;
    double lab_thresh_ = 71.5;
    int lab_preprocess_blur_ksize_ = 5;
    int lab_mask_morph_ksize_ = 3;
    double vis_input_gamma_ = 0.76;
    double vis_clahe_l_clip_limit_ = 0.0;
    bool lab_mask_canny_bridge_ = false;
    int lab_canny_thresh1_ = 80;
    int lab_canny_thresh2_ = 160;
    int lab_canny_dilate_px_ = 5;
    int lab_canny_bridge_mask_dilate_ = 2;
    double ctrl_integ_max_px_s_ = 60.0;
    double kp_ = 0.022;
    double kd_ = 0.006;
    double ki_yaw_ = 0.00002;
    double ki_pitch_ = 0.00001;
    // [MOD-2] 与 declare_parameter 默认值保持一致，确保动态参数未加载时也按预期滤波。
    double err_lp_beta_yaw_ = 0.62;
    double err_lp_beta_pitch_ = 0.68;
    int deadband_px_ = 3;
    int deadband_yaw_px_ = 2;
    double max_delta_deg_ = 1.2;
    double yaw_gain_ = 2.45;
    double yaw_kp_mult_ = 3.0;
    double yaw_min_step_deg_ = 0.35;
    double near_tgt_yaw_kp_scale_ = 0.80;
    bool yaw_quantize_nudge_enable_ = false;
    double yaw_still_zone_hit_px_ = 30.0;
    double yaw_still_zone_deadband_extra_px_ = 2.5;
    double yaw_still_zone_kd_scale_ = 0.32;
    double yaw_min_step_min_hit_px_ = 22.0;
    double yaw_still_hysteresis_px_ = 5.0;
    bool yaw_deadband_freeze_ = false;
    double d_lp_beta_yaw_ = 0.24;
    double d_lp_beta_pitch_ = 0.18;
    double antiwindup_backcalc_gain_yaw_ = 0.35;
    double antiwindup_backcalc_gain_pitch_ = 0.35;
    double search_yaw_amp_deg_ = 32.0;
    double search_pitch_amp_deg_ = 12.0;
    double search_phase_step_ = 0.10;
    double search_pitch_phase_step_ = 0.035;
    int search_enter_frames_ = 2;
    double settle_px_ = 12.0;
    int settle_frames_ = 6;
    int settle_ok_count_ = 0;
    double servo_angle_smooth_beta_ = 0.88;
    double roi_frac_ = 0.60;
    double center_roi_zoom_ = 1.0;
    bool suppress_bright_lights_ = false;
    int bright_hsv_v_min_ = 252;
    int bright_hsv_s_max_ = 58;
    int bright_bgr_channel_min_ = 244;
    int bright_mask_dilate_ = 5;
    int bright_inpaint_radius_ = 5;
    double bright_inpaint_max_area_frac_ = 0.012;
    double smooth_target_alpha_scale_ = 1.0;
    double target_step_max_px_ = 88.0;
    int predict_coast_frames_ = 12;
    double predict_lead_ms_ = 62.0;
    int predict_arm_frames_ = 8;
    double predict_vel_beta_ = 0.42;
    bool predict_armed_ = false;
    int predict_good_streak_ = 0;
    double lab_min_area_seg_ = 50.0;
    double sector_min_area_ = 220.0;
    double sector_max_area_ratio_ = 0.52;
    double square_approx_poly_eps_ = 0.024;
    bool publish_center_color_id_ = true;
    int frame_center_lab_cid_ = -1;
    int acquire_lost_frames_ = 0;
    TrackState track_;
    cv::Point2f vel_ema_{0.f, 0.f};
    float ctrl_prev_ex_ = 0.f;
    float ctrl_prev_ey_ = 0.f;
    float ctrl_dfilt_ex_ = 0.f;
    float ctrl_dfilt_ey_ = 0.f;
    float ctrl_filt_ex_ = 0.f;
    float ctrl_filt_ey_ = 0.f;
    float ctrl_integ_ex_ = 0.f;
    float ctrl_integ_ey_ = 0.f;
    int last_pub_yaw_cmd_ = -999;
    int yaw_cmd_stale_frames_ = 0;
    cv::Point2f held_target_{-1.0f, -1.0f};
    bool held_target_valid_ = false;
    int target_hold_lost_count_ = 0;
    bool servo_out_lp_inited_ = false;
    float servo_out_lp_pitch_ = 0.f;
    float servo_out_lp_yaw_ = 0.f;
    double last_proc_wall_sec_{0.0};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidTestNode>());
    rclcpp::shutdown();
    return 0;
}
