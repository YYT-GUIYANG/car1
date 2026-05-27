/**
 * trace_calculator.cpp — 能量机关视觉跟踪主节点（可执行名 trace_calculator，类名 ImageSubscriberNode）
 *
 * 我们这套的思路可以概括成四步：
 *   1. 订阅 /processed_image，在画面中心 ROI 里做 Lab 分色 + 轮廓分析；
 *   2. 先找「中心圆盘」，再用 HSV 把颜色 id 读准，然后在扇区里找同色方块；
 *   3. 算目标中心与激光准星之间的像素误差 x_error / y_error；
 *   4. PD（可选 I）算出俯仰/偏航舵机角，发布到 /servo_control。
 *
 * 调参建议：先在 调试参数/offline_trace.py 里用滑条把识别调稳，再把数值写到 launch 或本节点参数里。
 * 参数含义与「调大/调小会怎样」见仓库根目录 README.md 的「调试参数说明」一节。
 */
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include "servo_message/msg/servo_message.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using ServoMsg = servo_message::msg::ServoMessage;

namespace {

// 现场 12 种能量块颜色的 BGR 参考表（id 与赛场色块编号对应，Lab 分色就是跟这张表比距离）
const std::vector<std::pair<int, cv::Vec3b>> kReferenceBgr = {
    {0, cv::Vec3b(116, 5, 202)},   {3, cv::Vec3b(167, 1, 98)},   {4, cv::Vec3b(7, 237, 19)},
    {5, cv::Vec3b(23, 51, 215)},   {6, cv::Vec3b(241, 132, 251)}, {7, cv::Vec3b(17, 168, 214)},
    {8, cv::Vec3b(135, 199, 246)}, {9, cv::Vec3b(221, 66, 76)},   {10, cv::Vec3b(216, 199, 167)},
    {11, cv::Vec3b(85, 152, 55)},  {12, cv::Vec3b(57, 244, 231)}, {16, cv::Vec3b(23, 3, 23)},
};

// 连通域打包：后面选中心圆、选扇区方块都靠这些字段打分
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

// 跨帧记忆：平滑目标点、速度 EMA、丢检计数，避免一帧没认出就云台乱甩
struct TrackState {
    bool has_smooth_target = false;
    cv::Point2f smooth_target{0.f, 0.f};
    bool has_center = false;
    cv::Point2f smooth_center{0.f, 0.f};
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

static void contour_area_circle_rect_ratios(
    const std::vector<cv::Point>& c, double& r_circ, double& r_rect) {
    const double a = cv::contourArea(c);
    if (a < 1.0 || c.size() < 3) {
        r_circ = 0.0;
        r_rect = 1.0;
        return;
    }
    cv::Point2f cc;
    float rad = 0.f;
    cv::minEnclosingCircle(c, cc, rad);
    const double rr = std::max(static_cast<double>(rad), 1e-6);
    r_circ = a / (CV_PI * rr * rr);
    const cv::RotatedRect mr = cv::minAreaRect(c);
    const double ar = std::max(static_cast<double>(mr.size.width) * static_cast<double>(mr.size.height), 1e-6);
    r_rect = a / ar;
}

struct CenterDiskGeomParams {
    double circle_area_ratio_min = 0.8;
    double rect_area_ratio_max = 0.9;
    double center_max_dist_ratio = 0.38;
    double min_center_area = 25.0;
};

// 根据圆度 + 长宽比粗分：circle / square / other（中心圆和扇区方块筛选会用到）
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

static void bgr_scalar_to_hsv(double B, double G, double R, float& h, float& s, float& v) {
    cv::Mat px(1, 1, CV_8UC3);
    px.at<cv::Vec3b>(0, 0) = cv::Vec3b(
        static_cast<uchar>(std::clamp(B, 0.0, 255.0)),
        static_cast<uchar>(std::clamp(G, 0.0, 255.0)),
        static_cast<uchar>(std::clamp(R, 0.0, 255.0)));
    cv::Mat hsv;
    cv::cvtColor(px, hsv, cv::COLOR_BGR2HSV);
    const auto t = hsv.at<cv::Vec3b>(0, 0);
    h = static_cast<float>(t[0]);
    s = static_cast<float>(t[1]);
    v = static_cast<float>(t[2]);
}

static double hsv_dist_sq_color_table(float h1, float s1, float v1, float h2, float s2, float v2) {
    float dh = std::abs(h1 - h2);
    dh = std::min(dh, 180.0f - dh);
    const float ds = std::abs(s1 - s2);
    const float dv = std::abs(v1 - v2);
    const double a = static_cast<double>(dh);
    const double b = 0.3 * static_cast<double>(ds);
    const double c = 0.3 * static_cast<double>(dv);
    return a * a + b * b + c * c;
}

static int classify_blob_mean_bgr_hsv_nearest(const cv::Mat& work_bgr, const Blob& b, double max_dist_sq) {
    if (b.contour.size() < 3 || work_bgr.empty() || work_bgr.type() != CV_8UC3) return -1;
    cv::Mat mask = cv::Mat::zeros(work_bgr.rows, work_bgr.cols, CV_8U);
    const std::vector<std::vector<cv::Point>> ch = {b.contour};
    cv::drawContours(mask, ch, 0, cv::Scalar(255), cv::FILLED);
    const cv::Scalar mu = cv::mean(work_bgr, mask);
    float h0 = 0, s0 = 0, v0 = 0;
    bgr_scalar_to_hsv(mu[0], mu[1], mu[2], h0, s0, v0);
    int best_id = -1;
    double best_d = 1e18;
    for (const auto& kv : kReferenceBgr) {
        float rh = 0, rs = 0, rv = 0;
        bgr_scalar_to_hsv(kv.second[0], kv.second[1], kv.second[2], rh, rs, rv);
        const double d = hsv_dist_sq_color_table(h0, s0, v0, rh, rs, rv);
        if (d < best_d) {
            best_d = d;
            best_id = kv.first;
        }
    }
    // id=10 在现场光照下波动较大，给一点轻微容差，减少漏检。
    const double accept_th = (best_id == 10) ? (max_dist_sq * 1.18) : max_dist_sq;
    if (best_id < 0 || best_d > accept_th) return -1;
    return best_id;
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

static Blob* pick_center_disk_shape_first(
    std::vector<Blob>& blobs,
    const cv::Point2f& opt_center,
    double frame_area,
    int work_w,
    int work_h,
    double center_circularity_min,
    const CenterDiskGeomParams& geom) {
    constexpr double kMaxCenterFrac = 0.22;
    constexpr double kBigIdFrac = 0.42;
    const float denom = static_cast<float>(std::max(work_w, work_h));
    const float min_side = static_cast<float>(std::min(work_w, work_h));
    Blob* best = nullptr;
    double best_score = -1e9;
    for (auto& b : blobs) {
        if (b.area > kMaxCenterFrac * frame_area) continue;
        if ((b.cid == 8 || b.cid == 10) && b.area > kBigIdFrac * frame_area) continue;
        if (b.area < geom.min_center_area) continue;
        if (b.circularity < center_circularity_min) continue;
        const double d_opt = std::hypot(b.cx - opt_center.x, b.cy - opt_center.y);
        if (d_opt > geom.center_max_dist_ratio * static_cast<double>(min_side)) continue;
        double r_circ = 0, r_rect = 1;
        contour_area_circle_rect_ratios(b.contour, r_circ, r_rect);
        if (r_circ <= geom.circle_area_ratio_min) continue;
        if (r_rect >= geom.rect_area_ratio_max) continue;
        const double dx = b.cx - opt_center.x;
        const double dy = b.cy - opt_center.y;
        const double dist2 = dx * dx + dy * dy;
        double score = b.circularity - std::sqrt(dist2) / std::max(1.0f, denom);
        if (b.shape == "circle") score += 0.12;
        else if (b.shape == "square") score -= 0.08;
        if (score > best_score) {
            best_score = score;
            best = &b;
        }
    }
    return best;
}

static void orient_center_and_sector(Blob*& center_b, Blob*& sector_b, const cv::Point2f& opt) {
    if (!center_b || !sector_b) return;
    if (center_b == sector_b) {
        sector_b = nullptr;
        return;
    }
    Blob* a = center_b;
    Blob* b = sector_b;
    if (a->shape == "circle" && b->shape == "square") {
        center_b = a;
        sector_b = b;
        return;
    }
    if (b->shape == "circle" && a->shape == "square") {
        center_b = b;
        sector_b = a;
        return;
    }
    const double da = std::hypot(a->cx - opt.x, a->cy - opt.y);
    const double db = std::hypot(b->cx - opt.x, b->cy - opt.y);
    if (da <= db) {
        center_b = a;
        sector_b = b;
    } else {
        center_b = b;
        sector_b = a;
    }
}

static Blob* pick_sector_quad_same_hsv(
    std::vector<Blob>& blobs,
    Blob* center_blk,
    int ref_id,
    const cv::Mat& work_bgr,
    double sector_hsv_max_dist_sq,
    double sector_min_area,
    double sector_max_area_ratio,
    double frame_area,
    double approx_eps,
    double area_rel_tol,
    double diam_side_rel_tol,
    double max_dist_center_radius,
    bool has_prefer_pt,
    const cv::Point2f& prefer_pt,
    double prefer_radius_px,
    double prefer_bonus) {
    if (!center_blk || ref_id < 0) return nullptr;
    Blob* best = nullptr;
    double best_score = -1e18;
    const double center_area = std::max(1.0, center_blk->area);
    const double center_diam = 2.0 * std::sqrt(center_area / CV_PI);
    const double center_radius = std::max(1.0, center_diam * 0.5);
    const cv::Point2f cpt(static_cast<float>(center_blk->cx), static_cast<float>(center_blk->cy));
    for (auto& b : blobs) {
        if (&b == center_blk) continue;
        if (b.area < sector_min_area || b.area > sector_max_area_ratio * frame_area) continue;
        const int nv = contour_approx_vertex_count(b.contour, approx_eps);
        if (nv != 4 && nv != 5) continue;
        const int hid = classify_blob_mean_bgr_hsv_nearest(work_bgr, b, sector_hsv_max_dist_sq);
        if (hid != ref_id) continue;
        const double area_rel = std::abs(b.area - center_area) / center_area;
        if (area_rel > area_rel_tol) continue;
        const double side = std::sqrt(std::max(1.0, b.area));
        const double ds_rel = std::abs(center_diam - side) / std::max(1.0, side);
        if (ds_rel > diam_side_rel_tol) continue;
        const double dist = std::hypot(static_cast<double>(b.cx) - cpt.x, static_cast<double>(b.cy) - cpt.y);
                                                                                                // 距离门限做自适应下限：除了中心圆半径倍数，还要允许至少与候选方块边长同量级的距离
        const double dist_cap = std::max(std::max(0.8, max_dist_center_radius) * center_radius, 2.2 * side);
        if (dist > dist_cap) continue;
        // 单评分：优先几何一致(面积/边长)，其次距离中心圆适中，避免只按面积选到错误大块
        double score = -3.0 * area_rel - 2.0 * ds_rel - 0.0025 * dist + 0.0006 * b.area;
        if (has_prefer_pt && prefer_radius_px > 1.0) {
            const double dp =
                std::hypot(static_cast<double>(b.cx) - prefer_pt.x, static_cast<double>(b.cy) - prefer_pt.y);
            if (dp < prefer_radius_px) {
                const double k = 1.0 - dp / prefer_radius_px;
                score += std::max(0.0, prefer_bonus) * k;
            }
        }
        if (score > best_score) {
            best_score = score;
            best = &b;
        }
    }
    return best;
}

static Blob* pick_largest_quad_for_color_hsv(
    std::vector<Blob>& blobs,
    int ref_id,
    const cv::Mat& work_bgr,
    double sector_hsv_max_dist_sq,
    double sector_min_area,
    double sector_max_area_ratio,
    double frame_area,
    double approx_eps) {
    if (ref_id < 0) return nullptr;
    Blob* best = nullptr;
    double best_area = -1.0;
    const double a_cap = sector_max_area_ratio * frame_area;
    for (auto& b : blobs) {
        if (b.area < sector_min_area || b.area > a_cap) continue;
        const int nv = contour_approx_vertex_count(b.contour, approx_eps);
        if (nv != 4 && nv != 5) continue;
        const int hid = classify_blob_mean_bgr_hsv_nearest(work_bgr, b, sector_hsv_max_dist_sq);
        if (hid != ref_id) continue;
        if (b.area > best_area) {
            best_area = b.area;
            best = &b;
        }
    }
    return best;
}

static Blob* pick_largest_blob_for_color_hsv(
    std::vector<Blob>& blobs,
    int ref_id,
    const cv::Mat& work_bgr,
    double hsv_max_dist_sq,
    double min_area,
    double max_area_ratio,
    double frame_area,
    bool has_center_anchor,
    const cv::Point2f& center_anchor,
    double center_anchor_radius,
    double min_dist_center_radius,
    double max_dist_center_radius) {
    if (ref_id < 0) return nullptr;
    Blob* best = nullptr;
    double best_area = -1.0;
    const double area_cap = std::max(min_area, max_area_ratio * frame_area);
    const double c_rad = std::max(1.0, center_anchor_radius);
    for (auto& b : blobs) {
        if (b.area < min_area || b.area > area_cap) continue;
        const int hid = classify_blob_mean_bgr_hsv_nearest(work_bgr, b, hsv_max_dist_sq);
        if (hid != ref_id) continue;
        if (has_center_anchor) {
            const double d = std::hypot(static_cast<double>(b.cx) - center_anchor.x, static_cast<double>(b.cy) - center_anchor.y);
            const double d_min = std::max(0.0, min_dist_center_radius) * c_rad;
            const double d_max = std::max(d_min + 1.0, std::max(0.0, max_dist_center_radius) * c_rad);
            if (d < d_min || d > d_max) continue;
        }
        if (b.area > best_area) {
            best_area = b.area;
            best = &b;
        }
    }
    return best;
}

}  // namespace

// ROS2 节点本体：下面构造函数里 declare_parameter 的每一项，都能在 launch 里改
class ImageSubscriberNode : public rclcpp::Node {
public:
    ImageSubscriberNode() : Node("image_subscriber_node") {
        // ---------- 模式与瞄准基准 ----------
        energy_mode_ = this->declare_parameter<std::string>("energy_mode", "large");
        control_enabled_ = this->declare_parameter<bool>("control_enabled", true);
        laser_center_ratio_x_ = this->declare_parameter<double>("laser_center_ratio_x", 0.5);
        laser_center_ratio_y_ = this->declare_parameter<double>("laser_center_ratio_y", 0.5);
        laser_aim_offset_x_px_ = this->declare_parameter<double>("laser_aim_offset_x_px", 0.0);
        laser_aim_offset_y_px_ = this->declare_parameter<double>("laser_aim_offset_y_px", 34.0);
        init_pitch_deg_ = this->declare_parameter<double>("init_pitch_deg", 340.0);
        init_yaw_deg_ = this->declare_parameter<double>("init_yaw_deg", 170.0);
        startup_hold_frames_ = this->declare_parameter<int>("startup_hold_frames", 60);
        // ---------- 舵机 ID 与角度限位（要和实际控制板、串口节点一致）----------
        servo_id_pitch_ = this->declare_parameter<int>("servo_id_pitch", 11);
        servo_id_yaw_ = this->declare_parameter<int>("servo_id_yaw", 8);
        servo_pitch_min_deg_ = this->declare_parameter<double>("servo_pitch_min_deg", 0.0);
        servo_pitch_max_deg_ = this->declare_parameter<double>("servo_pitch_max_deg", 360.0);
        servo_yaw_min_deg_ = this->declare_parameter<double>("servo_yaw_min_deg", 0.0);
        servo_yaw_max_deg_ = this->declare_parameter<double>("servo_yaw_max_deg", 360.0);
        servo_pitch_travel_half_span_deg_ =
            this->declare_parameter<double>("servo_pitch_travel_half_span_deg", 40.0);
        servo_yaw_travel_half_span_deg_ =
            this->declare_parameter<double>("servo_yaw_travel_half_span_deg", 40.0);
        servo_pitch_travel_pos_deg_ = this->declare_parameter<double>("servo_pitch_travel_pos_deg", 30.0);
        servo_pitch_travel_neg_deg_ = this->declare_parameter<double>("servo_pitch_travel_neg_deg", 10.0);
        servo_yaw_travel_pos_deg_ = this->declare_parameter<double>("servo_yaw_travel_pos_deg", 20.0);
        servo_yaw_travel_neg_deg_ = this->declare_parameter<double>("servo_yaw_travel_neg_deg", 20.0);
        // ---------- 图像识别：Lab 阈值、模糊、形态学、处理宽度上限 ----------
        trace_proc_width_cap_ = static_cast<int>(std::clamp(
            this->declare_parameter<int>("trace_proc_width_cap_large", 960), int64_t{320}, int64_t{1920}));
        lab_thresh_ = this->declare_parameter<double>("large_lab_thresh", 71.5);
        trace_proc_width_cap_small_ = static_cast<int>(std::clamp(
            this->declare_parameter<int>("trace_proc_width_cap_small", 880), int64_t{320}, int64_t{1920}));
        small_lab_thresh_ = this->declare_parameter<double>("small_lab_thresh", 58.0);
        lab_preprocess_blur_ksize_ = this->declare_parameter<int>("lab_preprocess_blur_ksize", 15);
        lab_mask_morph_ksize_ = this->declare_parameter<int>("lab_mask_morph_ksize", 5);
        vis_input_gamma_ = std::clamp(this->declare_parameter<double>("vis_input_gamma", 0.76), 0.35, 3.5);
        vis_clahe_l_clip_limit_ =
            std::clamp(this->declare_parameter<double>("vis_clahe_l_clip_limit", 0.0), 0.0, 12.0);
        lab_mask_canny_bridge_ = this->declare_parameter<bool>("lab_mask_canny_bridge", false);
        lab_canny_thresh1_ = static_cast<int>(
            std::clamp(this->declare_parameter<int>("lab_canny_thresh1", 80), int64_t{1}, int64_t{254}));
        lab_canny_thresh2_ = static_cast<int>(
            std::clamp(this->declare_parameter<int>("lab_canny_thresh2", 160), int64_t{1}, int64_t{255}));
        lab_canny_dilate_px_ = static_cast<int>(
            std::clamp(this->declare_parameter<int>("lab_canny_dilate_px", 5), int64_t{1}, int64_t{15}));
        lab_canny_bridge_mask_dilate_ = static_cast<int>(
            std::clamp(this->declare_parameter<int>("lab_canny_bridge_mask_dilate", 2), int64_t{1}, int64_t{5}));
        // ---------- 控制环：大/小能量各自的 Kp Kd Ki、死区、低通、搜索摆幅 ----------
        ctrl_integ_max_px_s_ =
            std::clamp(this->declare_parameter<double>("ctrl_integ_max_px_s", 120.0), 10.0, 800.0);
        kp_ = this->declare_parameter<double>("large_kp", 0.026);
        kd_ = this->declare_parameter<double>("large_kd", 0.022);
        ki_yaw_ = this->declare_parameter<double>("large_ki_yaw", 0.0);
        ki_pitch_ = this->declare_parameter<double>("large_ki_pitch", 0.0);
        small_kp_ = this->declare_parameter<double>("small_kp", 0.013);
        small_kd_ = this->declare_parameter<double>("small_kd", 0.012);
        small_ki_yaw_ = this->declare_parameter<double>("small_ki_yaw", 0.0);
        small_ki_pitch_ = this->declare_parameter<double>("small_ki_pitch", 0.0);
        err_lp_beta_yaw_ = this->declare_parameter<double>("large_err_lp_beta_yaw", 0.52);
        err_lp_beta_pitch_ = this->declare_parameter<double>("large_err_lp_beta_pitch", 0.30);
        small_err_lp_beta_yaw_ = this->declare_parameter<double>("small_err_lp_beta_yaw", 0.46);
        small_err_lp_beta_pitch_ = this->declare_parameter<double>("small_err_lp_beta_pitch", 0.42);
        deadband_px_ = this->declare_parameter<int>("large_deadband_px", 2);
        deadband_yaw_px_ = this->declare_parameter<int>("large_deadband_yaw_px", 1);
        max_delta_deg_ = this->declare_parameter<double>("large_max_delta_deg", 1.05);
        small_deadband_ = this->declare_parameter<int>("small_deadband", 12);
        small_max_delta_deg_ = this->declare_parameter<double>("small_max_delta_deg", 0.50);
        yaw_gain_ = this->declare_parameter<double>("large_yaw_gain", 1.58);
        yaw_kp_mult_ = this->declare_parameter<double>("large_yaw_kp_mult", 1.75);
        yaw_min_step_deg_ = this->declare_parameter<double>("large_yaw_min_step_deg", 0.52);
        small_min_step_deg_ = this->declare_parameter<double>("small_min_step_deg", 0.18);
        small_lead_ms_ = this->declare_parameter<double>("small_lead_ms", 28.0);
        small_settle_px_ = this->declare_parameter<double>("small_settle_px", 14.0);
        small_settle_frames_ = this->declare_parameter<int>("small_settle_frames", 5);
        near_tgt_yaw_kp_scale_ = this->declare_parameter<double>("large_near_tgt_yaw_kp_scale", 0.92);
        search_yaw_amp_deg_ = this->declare_parameter<double>("large_square_search_yaw_amp_deg", 32.0);
        search_pitch_amp_deg_ = this->declare_parameter<double>("small_search_pitch_amp_deg", 12.0);
        search_phase_step_ = this->declare_parameter<double>("small_search_phase_step", 0.10);
        search_pitch_phase_step_ = this->declare_parameter<double>("small_search_pitch_phase_step", 0.035);
        search_enter_frames_ = this->declare_parameter<int>("small_search_enter_frames", 3);
        small_center_hub_dist_frac_ = this->declare_parameter<double>("small_center_hub_dist_frac", 0.32);
        settle_px_ = this->declare_parameter<double>("large_settle_px", 8.0);
        settle_frames_ = this->declare_parameter<int>("large_settle_frames", 4);
        servo_angle_smooth_beta_ = this->declare_parameter<double>("servo_angle_smooth_beta", 0.32);
        publish_tracking_debug_ = this->declare_parameter<bool>("publish_tracking_debug", true);
        show_vis_window_ = this->declare_parameter<bool>("show_vis_window", true);
        runtime_tuning_enable_ = this->declare_parameter<bool>("runtime_tuning_enable", true);
        publish_trace_debug_image_ = this->declare_parameter<bool>("publish_trace_debug_image", true);
        trace_debug_image_topic_ = this->declare_parameter<std::string>("trace_debug_image_topic", "/trace_debug_image");
        // ---------- ROI、强光抑制、目标平滑与丢检预测 ----------
        roi_frac_ = this->declare_parameter<double>("large_square_roi_frac", 0.77);
        center_roi_zoom_ = this->declare_parameter<double>("center_roi_zoom", 1.0);
        suppress_bright_lights_ = this->declare_parameter<bool>("suppress_bright_lights", false);
        bright_hsv_v_min_ = this->declare_parameter<int>("bright_hsv_v_min", 252);
        bright_hsv_s_max_ = this->declare_parameter<int>("bright_hsv_s_max", 58);
        bright_bgr_channel_min_ = this->declare_parameter<int>("bright_bgr_channel_min", 244);
        bright_mask_dilate_ = this->declare_parameter<int>("bright_mask_dilate", 5);
        bright_inpaint_radius_ = this->declare_parameter<int>("bright_inpaint_radius", 5);
        bright_inpaint_max_area_frac_ = this->declare_parameter<double>("bright_inpaint_max_area_frac", 0.012);
        pitch_laser_lift_px_ = this->declare_parameter<double>("large_pitch_laser_lift_px", 22.0);
        smooth_target_alpha_scale_ = this->declare_parameter<double>("large_smooth_target_alpha_scale", 1.0);
        target_step_max_px_ = this->declare_parameter<double>("large_square_target_step_max_px", 88.0);
        predict_coast_frames_ = this->declare_parameter<int>("large_square_predict_coast_frames", 12);
        predict_lead_ms_ = this->declare_parameter<double>("large_square_predict_lead_ms", 62.0);
        predict_arm_frames_ = this->declare_parameter<int>("large_square_predict_arm_frames", 8);
        predict_vel_beta_ = this->declare_parameter<double>("large_square_predict_vel_beta", 0.28);
        // ---------- 小能量：锁中心色、仅追同色块、已知转速预测 ----------
        small_known_speed_predict_enable_ = this->declare_parameter<bool>("small_known_speed_predict_enable", true);
        small_known_speed_deg_s_ = this->declare_parameter<double>("small_known_speed_deg_s", 60.0);
        small_known_speed_dir_sign_ = this->declare_parameter<int>("small_known_speed_dir_sign", 1);
        small_known_speed_blend_ = this->declare_parameter<double>("small_known_speed_blend", 0.68);
        color_lock_enable_ = this->declare_parameter<bool>("small_center_color_lock_enable", true);
        color_lock_stable_frames_ = this->declare_parameter<int>("small_center_color_lock_stable_frames", 4);
        color_lock_timeout_frames_ = this->declare_parameter<int>("small_center_color_lock_timeout_frames", 36);
        center_color_relock_after_miss_frames_ =
            this->declare_parameter<int>("center_color_relock_after_miss_frames", 10);
        color_only_after_lock_enable_ = this->declare_parameter<bool>("small_color_only_after_lock_enable", true);
        color_only_arm_frames_ = this->declare_parameter<int>("small_color_only_arm_frames", 6);
        disable_blob_fallback_ = this->declare_parameter<bool>("disable_blob_fallback", true);
        aim_workflow_enabled_ = this->declare_parameter<bool>("aim_workflow_enabled", false);
        stable_online_mode_ = this->declare_parameter<bool>("stable_online_mode", true);
        stable_target_hold_frames_ = this->declare_parameter<int>("stable_target_hold_frames", 3);
        stable_transition_blend_frames_ = this->declare_parameter<int>("stable_transition_blend_frames", 5);
        stable_disable_predict_ = this->declare_parameter<bool>("stable_disable_predict", true);
        stable_disable_search_ = this->declare_parameter<bool>("stable_disable_search", true);
        stable_fixed_dt_s_ = std::clamp(this->declare_parameter<double>("stable_fixed_dt_s", 0.04), 0.01, 0.1);
        // ---------- 几何选靶：中心圆、扇区方块、HSV 重标、回退 blob ----------
        rt_kp_ = small_kp_;
        rt_kd_ = small_kd_;
        rt_max_delta_deg_ = small_max_delta_deg_;
        rt_min_step_deg_ = small_min_step_deg_;
        rt_deadband_px_ = static_cast<double>(small_deadband_);
        rt_err_lp_beta_ = small_err_lp_beta_pitch_;
        rt_servo_smooth_beta_ = servo_angle_smooth_beta_;
        rt_hold_frames_ = stable_target_hold_frames_;
        rt_transition_frames_ = stable_transition_blend_frames_;
        center_square_area_rel_tol_ =
            std::clamp(this->declare_parameter<double>("center_square_area_rel_tol", 0.45), 0.05, 1.5);
        center_square_diam_side_rel_tol_ =
            std::clamp(this->declare_parameter<double>("center_square_diam_side_rel_tol", 0.35), 0.05, 1.5);
        square_max_dist_center_radius_ =
            std::clamp(this->declare_parameter<double>("square_max_dist_center_radius", 2.8), 0.5, 8.0);
        sector_prefer_last_enable_ = this->declare_parameter<bool>("sector_prefer_last_enable", true);
        sector_prefer_radius_px_ =
            std::clamp(this->declare_parameter<double>("sector_prefer_radius_px", 85.0), 10.0, 400.0);
        sector_prefer_bonus_ =
            std::clamp(this->declare_parameter<double>("sector_prefer_bonus", 0.35), 0.0, 2.0);
        fallback_blob_max_area_ratio_ =
            std::clamp(this->declare_parameter<double>("fallback_blob_max_area_ratio", 0.10), 0.01, 0.95);
        fallback_blob_min_dist_center_radius_ =
            std::clamp(this->declare_parameter<double>("fallback_blob_min_dist_center_radius", 0.35), 0.0, 8.0);
        fallback_blob_max_dist_center_radius_ =
            std::clamp(this->declare_parameter<double>("fallback_blob_max_dist_center_radius", 3.0), 0.2, 12.0);
        reacquire_blend_frames_ = this->declare_parameter<int>("small_reacquire_blend_frames", 2);
        lab_min_area_seg_ = this->declare_parameter<double>("large_lab_min_area_seg", 50.0);
        center_circle_area_ratio_min_ =
            std::clamp(this->declare_parameter<double>("center_circle_area_ratio_min", 0.8), 0.05, 0.999);
        center_rect_area_ratio_max_ =
            std::clamp(this->declare_parameter<double>("center_rect_area_ratio_max", 0.9), 0.05, 0.999);
        center_max_dist_ratio_ =
            std::clamp(this->declare_parameter<double>("center_max_dist_ratio", 0.38), 0.02, 0.95);
        sector_min_area_ = this->declare_parameter<double>("large_sector_min_area", 220.0);
        sector_max_area_ratio_ =
            std::clamp(this->declare_parameter<double>("large_sector_max_area_ratio", 0.52), 0.05, 0.95);
        square_approx_poly_eps_ =
            std::clamp(this->declare_parameter<double>("square_approx_poly_eps", 0.024), 0.005, 0.08);
        center_circularity_min_ = this->declare_parameter<double>("center_circularity_min", 0.74);
        center_hsv_max_dist_sq_ = this->declare_parameter<double>("large_center_hsv_max_dist_sq", 6200.0);
        sector_hsv_max_dist_sq_ = this->declare_parameter<double>("large_blob_hsv_max_dist_sq", 5200.0);
        publish_center_color_id_ = this->declare_parameter<bool>("publish_center_color_id", true);
        if (energy_mode_ != "small" && energy_mode_ != "large") energy_mode_ = "large";
        is_small_mode_ = (energy_mode_ == "small");

        image_sub_ = image_transport::create_subscription(
            this,
            "processed_image",
            std::bind(&ImageSubscriberNode::image_callback, this, std::placeholders::_1),
            "raw",
            rclcpp::QoS(10).get_rmw_qos_profile());
        servo_pub_ = this->create_publisher<ServoMsg>("servo_control", 10);
        tracking_debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tracking_debug", 10);
        center_color_pub_ = this->create_publisher<std_msgs::msg::Int32>("/center_color_id", 10);
        aim_command_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/aim_command", rclcpp::QoS(10),
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (!msg) return;
                if (msg->data == 0) {
                    aim_started_ = false;
                } else if (msg->data == 1) {
                    aim_started_ = true;
                    aim_relock_requested_ = false;
                } else if (msg->data == 2) {
                    aim_started_ = false;
                    aim_relock_requested_ = true;
                    locked_center_color_id_ = -1;
                    center_color_candidate_ = -1;
                    center_color_stable_count_ = 0;
                    color_only_active_ = false;
                }
            });
        trace_debug_pub_ = image_transport::create_publisher(this, trace_debug_image_topic_, rclcpp::QoS(1).get_rmw_qos_profile());

        last_frame_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "[TRACE] 通用12色追踪节点；订阅 /processed_image");
    }

    ~ImageSubscriberNode() override { cv::destroyAllWindows(); }

private:
    void warn_no_camera_frame_() {
        if (!control_enabled_) return;
        const double t = this->get_clock()->now().seconds();
        if (t - last_no_image_warn_sec_ < 5.0) return;
        last_no_image_warn_sec_ = t;
        RCLCPP_WARN(this->get_logger(), "[TRACE] 无有效相机帧");
    }

    // 每来一帧相机图就进 process_image；这里只负责格式转换和 FPS 统计
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

    // 单帧主流程：裁剪 ROI → 分色找 blob → 选中心圆和扇区目标 → 算误差 → PD 输出舵机角
    std::vector<std::pair<int, int>> process_image(const cv::Mat& src) {
        std::vector<std::pair<int, int>> out;
        frame_center_lab_cid_ = -1;

        static float servo_pitch_f = 90.f;
        static float servo_yaw_f = 90.f;
        static bool servo_init_once = false;
        if (!servo_init_once) {
            servo_yaw_f = static_cast<float>(std::clamp(init_yaw_deg_, servo_yaw_min_deg_, servo_yaw_max_deg_));
            servo_pitch_f = static_cast<float>(std::clamp(init_pitch_deg_, servo_pitch_min_deg_, servo_pitch_max_deg_));
            startup_countdown_ = std::max(0, startup_hold_frames_);
            servo_init_once = true;
        }

        // 【1】裁中心 ROI，减少边缘干扰；ROI 太小会把旋转扇区裁掉
        const float roi_f = static_cast<float>(std::clamp(roi_frac_, 0.55, 0.99));
        const int roi_w = static_cast<int>(src.cols * roi_f);
        const int roi_h = static_cast<int>(src.rows * roi_f);
        const cv::Rect roi_rect((src.cols - roi_w) / 2, (src.rows - roi_h) / 2, roi_w, roi_h);
        cv::Rect safe = roi_rect & cv::Rect(0, 0, src.cols, src.rows);
        if (safe.width <= 20 || safe.height <= 20) {
            out.push_back({servo_id_pitch_, wrap_deg_0_359(safe_round_servo_deg(servo_pitch_f))});
            out.push_back({servo_id_yaw_, wrap_deg_0_359(safe_round_servo_deg(servo_yaw_f))});
            send_servo_angle(out);
            return out;
        }

        // 【2】可选数字变焦 + 伽马/CLAHE，让暗部色块更好分
        cv::Mat vis = src(safe).clone();
        const float zoom_use = static_cast<float>(std::clamp(center_roi_zoom_, 1.0, 1.6));
        center_crop_zoom_inplace(vis, zoom_use);
        apply_gamma_bgr_inplace(vis, vis_input_gamma_);
        apply_clahe_lab_l_inplace(vis, vis_clahe_l_clip_limit_);

        int bright_px = 0;
        if (suppress_bright_lights_) {
            bright_px = suppress_bright_lights_inplace(
                vis, std::clamp(bright_hsv_v_min_, 1, 255), std::clamp(bright_hsv_s_max_, 0, 255),
                std::clamp(bright_bgr_channel_min_, 1, 255), std::max(0, bright_mask_dilate_),
                std::clamp(bright_inpaint_radius_, 1, 21),
                std::clamp(bright_inpaint_max_area_frac_, 0.003, 0.08));
        }

        const float lx = static_cast<float>(std::clamp(laser_center_ratio_x_, 0.0, 1.0));
        const float ly = static_cast<float>(std::clamp(laser_center_ratio_y_, 0.0, 1.0));
        const float lxs = static_cast<float>(vis.cols * lx + laser_aim_offset_x_px_);
        const float lys = static_cast<float>(vis.rows * ly + laser_aim_offset_y_px_);
        const cv::Point2f laser_center(
            std::clamp(lxs, 0.0f, static_cast<float>(std::max(1, vis.cols - 1))),
            std::clamp(lys, 0.0f, static_cast<float>(std::max(1, vis.rows - 1))));
        const float laser_y_for_track = laser_center.y - static_cast<float>(pitch_laser_lift_px_);

        const bool small_mode = is_small_mode_;
        const int trace_cap = small_mode ? trace_proc_width_cap_small_ : trace_proc_width_cap_;
        cv::Mat work = vis;
        float scale = 1.0f;
        if (vis.cols > trace_cap) {
            scale = static_cast<float>(trace_cap) / static_cast<float>(vis.cols);
            cv::resize(vis, work, cv::Size(), scale, scale, cv::INTER_AREA);
        }

        // 【3】Lab 逐像素分 12 色，再 findContours 得到 blobs（调 lab_thresh 最关键）
        const float lab_t = static_cast<float>(small_mode ? small_lab_thresh_ : lab_thresh_);
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
        for (int y = 0; y < lab.rows; ++y) {
            const cv::Vec3b* p = lab.ptr<cv::Vec3b>(y);
            int* o = labels.ptr<int>(y);
            for (int x = 0; x < lab.cols; ++x) {
                float best = 1e9f;
                int best_id = -1;
                const cv::Vec3f c(static_cast<float>(p[x][0]), static_cast<float>(p[x][1]), static_cast<float>(p[x][2]));
                for (size_t i = 0; i < ref_lab.size(); ++i) {
                    cv::Vec3f d = c - ref_lab[i];
                    float dist = std::sqrt(d.dot(d));
                    if (dist < best) {
                        best = dist;
                        best_id = ref_ids[i];
                    }
                }
                if (best <= lab_t) o[x] = best_id;
            }
        }

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

        const cv::Point2f opt_center(static_cast<float>(work.cols) * 0.5f, static_cast<float>(work.rows) * 0.5f);
        const double frame_area = static_cast<double>(work.cols) * static_cast<double>(work.rows);

        const double ccm = std::clamp(center_circularity_min_, 0.35, 0.85);
        CenterDiskGeomParams center_geom{
            center_circle_area_ratio_min_,
            center_rect_area_ratio_max_,
            center_max_dist_ratio_,
            std::max(25.0, lab_min_area_seg_ * 0.35)};
        if (small_mode) {
            center_geom.center_max_dist_ratio =
                std::clamp(small_center_hub_dist_frac_, 0.08, 0.60);
        }
        // 【4】几何 + 位置约束找中心圆，HSV 读准 track_id，再在扇区找同色方块
        Blob* disk = pick_center_disk_shape_first(blobs, opt_center, frame_area, work.cols, work.rows, ccm, center_geom);
        frame_center_lab_cid_ = disk ? disk->cid : -1;

        int track_id = -1;
        Blob* sector = nullptr;
        Blob* cen_pick = disk;
        if (disk) {
            track_id = classify_blob_mean_bgr_hsv_nearest(work, *disk, center_hsv_max_dist_sq_);
            if (track_id < 0) track_id = disk->cid;
            if (aim_workflow_enabled_ && aim_relock_requested_) {
                locked_center_color_id_ = track_id;
                center_color_candidate_ = track_id;
                center_color_stable_count_ = std::max(1, color_lock_stable_frames_);
                center_color_relock_miss_count_ = 0;
                aim_relock_requested_ = false;
            }
            if (color_lock_enable_) {
                if (locked_center_color_id_ < 0) {
                    if (track_id == center_color_candidate_) {
                        center_color_stable_count_++;
                    } else {
                        center_color_candidate_ = track_id;
                        center_color_stable_count_ = 1;
                    }
                    center_color_miss_count_ = 0;
                    if (center_color_stable_count_ >= std::max(1, color_lock_stable_frames_)) {
                        locked_center_color_id_ = center_color_candidate_;
                        center_color_relock_miss_count_ = 0;
                    }
                } else {
                    // 已锁色后不做每帧重判，只有连续不一致达到阈值才重锁
                    center_color_miss_count_ = 0;
                    if (track_id == locked_center_color_id_) {
                        center_color_relock_miss_count_ = 0;
                    } else {
                        center_color_relock_miss_count_++;
                        if (center_color_relock_miss_count_ >= std::max(1, center_color_relock_after_miss_frames_)) {
                            locked_center_color_id_ = track_id;
                            center_color_candidate_ = track_id;
                            center_color_stable_count_ = std::max(1, color_lock_stable_frames_);
                            center_color_relock_miss_count_ = 0;
                        }
                    }
                }
            }
        } else if (color_lock_enable_) {
            center_color_stable_count_ = 0;
            center_color_miss_count_ += 1;
            if (center_color_miss_count_ > std::max(1, color_lock_timeout_frames_)) {
                center_color_candidate_ = -1;
                locked_center_color_id_ = -1;
                center_color_relock_miss_count_ = 0;
                color_only_active_ = false;
            }
        }

        const int ref_track_id = (locked_center_color_id_ >= 0) ? locked_center_color_id_ : track_id;
        if (!aim_workflow_enabled_ && color_only_after_lock_enable_ && locked_center_color_id_ >= 0 &&
            center_color_stable_count_ >= std::max(1, color_only_arm_frames_)) {
            color_only_active_ = true;
        }
        if (locked_center_color_id_ < 0) color_only_active_ = false;
        bool has_center_anchor = false;
        cv::Point2f center_anchor = opt_center;
        double center_anchor_radius = 0.0;
        if (disk) {
            has_center_anchor = true;
            center_anchor = cv::Point2f(static_cast<float>(disk->cx), static_cast<float>(disk->cy));
            center_anchor_radius = std::sqrt(std::max(1.0, disk->area) / CV_PI);
        } else if (track_.has_center) {
            has_center_anchor = true;
            center_anchor = cv::Point2f(track_.smooth_center.x * scale, track_.smooth_center.y * scale);
            center_anchor_radius = std::sqrt(std::max(1.0, sector_min_area_) / CV_PI);
        }

        if (color_only_active_ && locked_center_color_id_ >= 0) {
            // 小能量常用：中心色锁死后只追同色方块，中心圆丢了也能跟
            sector = pick_largest_quad_for_color_hsv(
                blobs, locked_center_color_id_, work, sector_hsv_max_dist_sq_, sector_min_area_, sector_max_area_ratio_,
                frame_area, square_approx_poly_eps_);
            if (!sector) {
                if (!disable_blob_fallback_) {
                    sector = pick_largest_blob_for_color_hsv(
                        blobs, locked_center_color_id_, work, sector_hsv_max_dist_sq_, std::max(24.0, sector_min_area_ * 0.6),
                        fallback_blob_max_area_ratio_, frame_area, has_center_anchor, center_anchor, center_anchor_radius,
                        fallback_blob_min_dist_center_radius_, fallback_blob_max_dist_center_radius_);
                }
            }
            cen_pick = nullptr;
            if (sector) {
                color_only_miss_count_ = 0;
            } else {
                color_only_miss_count_ += 1;
                if (color_only_miss_count_ > std::max(1, color_lock_timeout_frames_)) {
                    color_only_active_ = false;
                    locked_center_color_id_ = -1;
                    center_color_candidate_ = -1;
                    center_color_stable_count_ = 0;
                }
            }
        } else {
            color_only_miss_count_ = 0;
            if (disk) {
                const bool has_prefer_pt = sector_prefer_last_enable_ && has_last_sector_pick_;
                sector = pick_sector_quad_same_hsv(
                    blobs, disk, ref_track_id, work, sector_hsv_max_dist_sq_, sector_min_area_, sector_max_area_ratio_,
                    frame_area, square_approx_poly_eps_, center_square_area_rel_tol_, center_square_diam_side_rel_tol_,
                    square_max_dist_center_radius_, has_prefer_pt, last_sector_pick_work_,
                    sector_prefer_radius_px_, sector_prefer_bonus_);
                orient_center_and_sector(cen_pick, sector, opt_center);
            } else if (locked_center_color_id_ >= 0 && !disable_blob_fallback_) {
                sector = pick_largest_blob_for_color_hsv(
                    blobs, locked_center_color_id_, work, sector_hsv_max_dist_sq_, std::max(24.0, sector_min_area_ * 0.6),
                    fallback_blob_max_area_ratio_, frame_area, has_center_anchor, center_anchor, center_anchor_radius,
                    fallback_blob_min_dist_center_radius_, fallback_blob_max_dist_center_radius_);
            }
        }

        bool center_ref_valid = (cen_pick != nullptr);
        float center_ref_x = cen_pick ? static_cast<float>(cen_pick->cx) : 0.f;
        float center_ref_y = cen_pick ? static_cast<float>(cen_pick->cy) : 0.f;
        int center_ref_cid = (locked_center_color_id_ >= 0) ? locked_center_color_id_ : track_id;
        float center_ref_rad =
            cen_pick ? std::max(10.0f, static_cast<float>(std::sqrt(std::max(1.0, cen_pick->area) / CV_PI))) : 0.f;

        Blob* target = sector;
        if (aim_workflow_enabled_ && !aim_started_) {
            // 未点击开始时只对准中心圆；同色方块继续识别用于可视化/锁色
            target = cen_pick;
            color_only_active_ = false;
        }
        const bool workflow_switched = (aim_started_ != last_aim_started_);
        if (workflow_switched && stable_online_mode_) {
            // 工作流切换时清控制状态，避免锁中心<->追方块切换瞬间冲击
            ctrl_integ_ex_ = ctrl_integ_ey_ = 0.f;
            ctrl_prev_ex_ = ctrl_prev_ey_ = 0.f;
            ctrl_filt_ex_ = ctrl_filt_ey_ = 0.f;
            stable_transition_countdown_ = std::max(0, stable_transition_blend_frames_);
            track_.has_prev_raw = false;
            vel_ema_ = cv::Point2f(0.f, 0.f);
            model_vel_ = cv::Point2f(0.f, 0.f);
            predict_armed_ = false;
            predict_good_streak_ = 0;
        }
        last_aim_started_ = aim_started_;
        cv::Point2f center_pt(-1, -1), target_raw(-1, -1);
        if (center_ref_valid) {
            center_pt = cv::Point2f(center_ref_x / scale, center_ref_y / scale);
        }
        if (target) {
            target_raw = cv::Point2f(static_cast<float>(target->cx / scale), static_cast<float>(target->cy / scale));
            stable_hold_count_ = 0;
            stable_last_valid_target_ = target_raw;
            stable_has_last_valid_target_ = true;
        } else if (stable_online_mode_ && stable_has_last_valid_target_ &&
                   stable_hold_count_ < std::max(0, stable_target_hold_frames_)) {
            // 短时丢检保持上一目标，减少识别抖动直接注入控制环
            target_raw = stable_last_valid_target_;
            stable_hold_count_ += 1;
        } else if (!target) {
            stable_hold_count_ = 0;
            stable_has_last_valid_target_ = false;
        }

        const float dt = stable_online_mode_
                             ? static_cast<float>(stable_fixed_dt_s_)
                             : ((current_fps_ > 1.0) ? (1.0f / static_cast<float>(current_fps_)) : (1.0f / 30.0f));
        const float alpha_track = 0.52f;
        const float alpha_recover = 0.46f;
        float target_step_max = static_cast<float>(target_step_max_px_);
        const float center_step_max = 42.0f;
        const float lead_ms = static_cast<float>(small_mode ? small_lead_ms_ : predict_lead_ms_);
        const float lead_s = std::clamp(lead_ms / 1000.0f, 0.0f, 0.35f);

        if (center_pt.x >= 0.0f) {
            if (!track_.has_center) {
                track_.smooth_center = center_pt;
                track_.has_center = true;
            } else {
                cv::Point2f c_meas = center_pt;
                const cv::Point2f dc = c_meas - track_.smooth_center;
                const float dcm = std::hypot(dc.x, dc.y);
                if (dcm > center_step_max && dcm > 1e-3f) {
                    const float s = center_step_max / dcm;
                    c_meas = track_.smooth_center + dc * s;
                }
                track_.smooth_center = 0.55f * c_meas + 0.45f * track_.smooth_center;
            }
        }

        if (target_raw.x >= 0.0f) {
            const int lost_before = track_.miss_count;
            track_.miss_count = 0;
            cv::Point2f inst{0.f, 0.f};
            if (track_.has_prev_raw) {
                inst.x = (target_raw.x - track_.prev_raw.x) / std::max(1e-4f, dt);
                inst.y = (target_raw.y - track_.prev_raw.y) / std::max(1e-4f, dt);
            }
            inst.x = std::clamp(inst.x, -950.0f, 950.0f);
            inst.y = std::clamp(inst.y, -950.0f, 950.0f);
            const float vb = static_cast<float>(predict_vel_beta_);
            vel_ema_.x = vb * inst.x + (1.0f - vb) * vel_ema_.x;
            vel_ema_.y = vb * inst.y + (1.0f - vb) * vel_ema_.y;

            cv::Point2f model_vel = vel_ema_;
            if (small_mode && small_known_speed_predict_enable_ && center_ref_valid) {
                cv::Point2f base_target = track_.has_smooth_target ? track_.smooth_target : target_raw;
                cv::Point2f radial = base_target - cv::Point2f(center_ref_x / scale, center_ref_y / scale);
                float rr = std::hypot(radial.x, radial.y);
                if (rr < 1e-3f) rr = 1.0f;
                cv::Point2f tang(-radial.y / rr, radial.x / rr);
                const float omega = static_cast<float>(small_known_speed_deg_s_ * CV_PI / 180.0);
                const float pix_speed = rr * omega;
                const float dir = (small_known_speed_dir_sign_ >= 0) ? 1.0f : -1.0f;
                cv::Point2f model_known = dir * pix_speed * tang;
                const float mb = static_cast<float>(std::clamp(small_known_speed_blend_, 0.0, 1.0));
                model_vel = (1.0f - mb) * model_vel + mb * model_known;
            }
            model_vel_ = model_vel;
            cv::Point2f model_predict = target_raw + model_vel * lead_s;
            model_predict.x = std::clamp(model_predict.x, 0.0f, static_cast<float>(vis.cols - 1));
            model_predict.y = std::clamp(model_predict.y, 0.0f, static_cast<float>(vis.rows - 1));
            last_model_predict_ = model_predict;

            cv::Point2f fused_meas = target_raw;
            if (track_state_ == TrackControlState::REACQUIRE || lost_before > 0) {
                fused_meas = 0.60f * target_raw + 0.40f * last_model_predict_;
            } else if (small_mode) {
                fused_meas = 0.70f * target_raw + 0.30f * model_predict;
            }
            vel_ema_ = 0.60f * vel_ema_ + 0.40f * model_vel;

            cv::Point2f pred = fused_meas;
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
                const float el = std::hypot(fused_meas.x - laser_center.x, fused_meas.y - laser_y_for_track);
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
            const float vel_m = std::hypot(model_vel_.x, model_vel_.y);
            const bool in_miss_window = predict_armed_ && track_.miss_count <= predict_coast_frames_;
            const bool sq_coast = in_miss_window && vel_m > 0.12f;
            if (sq_coast) {
                cv::Point2f st = track_.smooth_target + model_vel_ * lead_s;
                st.x = std::clamp(st.x, 0.0f, static_cast<float>(vis.cols - 1));
                st.y = std::clamp(st.y, 0.0f, static_cast<float>(vis.rows - 1));
                last_model_predict_ = st;
                track_.smooth_target = st;
                model_vel_.x *= 0.98f;
                model_vel_.y *= 0.98f;
            } else if (!in_miss_window) {
                track_.has_prev_raw = false;
                predict_armed_ = false;
                predict_good_streak_ = 0;
                vel_ema_.x = vel_ema_.y = 0.f;
                model_vel_.x = model_vel_.y = 0.f;
            }
        }

        const bool allow_predict_coast =
            (!stable_online_mode_ || !stable_disable_predict_) && predict_armed_ && target_raw.x < 0.0f &&
            track_.has_smooth_target && track_.miss_count > 0 && track_.miss_count <= predict_coast_frames_;

        const cv::Point2f target_point = track_.has_smooth_target ? track_.smooth_target : laser_center;
        const int x_error = static_cast<int>(std::lround(target_point.x - laser_center.x));
        const int y_error = static_cast<int>(std::lround(target_point.y - laser_y_for_track));
        const float hit_error_px = std::sqrt(static_cast<float>(x_error * x_error + y_error * y_error));

        const bool detect_ok_fresh = (target != nullptr);
        const bool detect_ok_control = (target_raw.x >= 0.0f);
        if (detect_ok_fresh) {
            acquire_lost_frames_ = 0;
        } else if (!allow_predict_coast) {
            acquire_lost_frames_++;
        }

        // 工作流约束：未开始追方块前，中心圆一旦锁定就冻结云台，不再旋转
        const bool hold_on_center_locked =
            aim_workflow_enabled_ && !aim_started_ && (locked_center_color_id_ >= 0) && center_ref_valid;

        bool search_mode =
            !allow_predict_coast && ((track_.miss_count >= 3) || (acquire_lost_frames_ >= search_enter_frames_));
        if (stable_online_mode_ && stable_disable_search_) {
            search_mode = false;
        }
        if (hold_on_center_locked) {
            search_mode = false;
        }
        if (detect_ok_control) {
            if (track_state_ == TrackControlState::COASTING || track_state_ == TrackControlState::SEARCH) {
                track_state_ = TrackControlState::REACQUIRE;
                reacquire_countdown_ = std::max(1, reacquire_blend_frames_);
            } else if (track_state_ == TrackControlState::REACQUIRE) {
                reacquire_countdown_ = std::max(0, reacquire_countdown_ - 1);
                if (reacquire_countdown_ <= 0) track_state_ = TrackControlState::TRACKING;
            } else {
                track_state_ = TrackControlState::TRACKING;
            }
        } else if (allow_predict_coast) {
            track_state_ = TrackControlState::COASTING;
        } else if (search_mode) {
            track_state_ = TrackControlState::SEARCH;
        }
        search_mode = (track_state_ == TrackControlState::SEARCH);

        const double settle_px_active = small_mode ? small_settle_px_ : settle_px_;
        const int settle_frames_active = small_mode ? small_settle_frames_ : settle_frames_;
        settle_ok_count_ = (detect_ok_control && hit_error_px <= static_cast<float>(settle_px_active))
                               ? (settle_ok_count_ + 1)
                               : 0;
        const bool should_hold = (settle_ok_count_ >= settle_frames_active) || hold_on_center_locked;

        float kp_pitch = static_cast<float>(small_mode ? small_kp_ : kp_);
        float kp_yaw = kp_pitch * static_cast<float>(yaw_kp_mult_);
        float kd_yaw = static_cast<float>(small_mode ? small_kd_ : kd_);
        float kd_pitch = kd_yaw;
        float beta_ex = static_cast<float>(small_mode ? small_err_lp_beta_yaw_ : err_lp_beta_yaw_);
        float beta_ey = static_cast<float>(small_mode ? small_err_lp_beta_pitch_ : err_lp_beta_pitch_);
        float max_delta_pitch_deg = static_cast<float>(small_mode ? small_max_delta_deg_ : max_delta_deg_);
        float max_delta_yaw_deg = max_delta_pitch_deg;
        int deadband_pitch = small_mode ? small_deadband_ : deadband_px_;
        int deadband_yaw = std::max(1, std::min(deadband_pitch, deadband_yaw_px_));
        float min_step_active = static_cast<float>(small_mode ? small_min_step_deg_ : yaw_min_step_deg_);
        if (runtime_tuning_enable_ && stable_online_mode_) {
            kp_pitch = static_cast<float>(rt_kp_);
            kp_yaw = kp_pitch * static_cast<float>(yaw_kp_mult_);
            kd_yaw = static_cast<float>(rt_kd_);
            kd_pitch = kd_yaw;
            beta_ex = static_cast<float>(rt_err_lp_beta_);
            beta_ey = static_cast<float>(rt_err_lp_beta_);
            max_delta_pitch_deg = static_cast<float>(rt_max_delta_deg_);
            max_delta_yaw_deg = max_delta_pitch_deg;
            deadband_pitch = std::max(0, static_cast<int>(std::lround(rt_deadband_px_)));
            deadband_yaw = std::max(1, std::min(deadband_pitch, deadband_yaw_px_));
            min_step_active = static_cast<float>(rt_min_step_deg_);
            servo_angle_smooth_beta_ = rt_servo_smooth_beta_;
            stable_target_hold_frames_ = rt_hold_frames_;
            stable_transition_blend_frames_ = rt_transition_frames_;
        }

        float yaw_lo = static_cast<float>(servo_yaw_min_deg_);
        float yaw_hi = static_cast<float>(servo_yaw_max_deg_);
        float pitch_lo = static_cast<float>(servo_pitch_min_deg_);
        float pitch_hi = static_cast<float>(servo_pitch_max_deg_);
        if (servo_yaw_travel_half_span_deg_ > 0.5) {
            const float c = static_cast<float>(init_yaw_deg_);
            const float h = static_cast<float>(servo_yaw_travel_half_span_deg_);
            yaw_lo = std::max(yaw_lo, c - h);
            yaw_hi = std::min(yaw_hi, c + h);
        } else {
            const float c = static_cast<float>(init_yaw_deg_);
            yaw_lo = std::max(yaw_lo, c - static_cast<float>(std::max(0.0, servo_yaw_travel_neg_deg_)));
            yaw_hi = std::min(yaw_hi, c + static_cast<float>(std::max(0.0, servo_yaw_travel_pos_deg_)));
        }
        if (servo_pitch_travel_half_span_deg_ > 0.5) {
            const float c = static_cast<float>(init_pitch_deg_);
            const float h = static_cast<float>(servo_pitch_travel_half_span_deg_);
            pitch_lo = std::max(pitch_lo, c - h);
            pitch_hi = std::min(pitch_hi, c + h);
        } else {
            const float c = static_cast<float>(init_pitch_deg_);
            pitch_lo = std::max(pitch_lo, c - static_cast<float>(std::max(0.0, servo_pitch_travel_neg_deg_)));
            pitch_hi = std::min(pitch_hi, c + static_cast<float>(std::max(0.0, servo_pitch_travel_pos_deg_)));
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
        ctrl_filt_ex_ = beta_ex * static_cast<float>(x_error) + (1.0f - beta_ex) * ctrl_filt_ex_;
        ctrl_filt_ey_ = beta_ey * static_cast<float>(y_error) + (1.0f - beta_ey) * ctrl_filt_ey_;

        float de_x = (ctrl_filt_ex_ - ctrl_prev_ex_) / std::max(1e-3f, dt);
        float de_y = (ctrl_filt_ey_ - ctrl_prev_ey_) / std::max(1e-3f, dt);
        ctrl_prev_ex_ = ctrl_filt_ex_;
        ctrl_prev_ey_ = ctrl_filt_ey_;

        float ki_yaw = static_cast<float>(small_mode ? small_ki_yaw_ : ki_yaw_) * static_cast<float>(yaw_kp_mult_);
        float ki_pitch = static_cast<float>(small_mode ? small_ki_pitch_ : ki_pitch_);
        const bool integ_run = detect_ok_control && !search_mode && !should_hold;
        if (integ_run) {
            ctrl_integ_ex_ += ctrl_filt_ex_ * dt;
            ctrl_integ_ey_ += ctrl_filt_ey_ * dt;
            const float lim = static_cast<float>(ctrl_integ_max_px_s_);
            ctrl_integ_ex_ = std::clamp(ctrl_integ_ex_, -lim, lim);
            ctrl_integ_ey_ = std::clamp(ctrl_integ_ey_, -lim, lim);
        } else {
            ctrl_integ_ex_ *= 0.88f;
            ctrl_integ_ey_ *= 0.88f;
        }

        float u_yaw = 0.f;
        float u_pitch = 0.f;
        if (std::abs(ctrl_filt_ex_) > deadband_yaw) {
            u_yaw = -(kp_yaw * ctrl_filt_ex_ + kd_yaw * de_x + ki_yaw * ctrl_integ_ex_);
        } else if (std::abs(ki_yaw * ctrl_integ_ex_) > 1e-6f) {
            u_yaw = -(ki_yaw * ctrl_integ_ex_);
        }
        if (std::abs(ctrl_filt_ey_) > deadband_pitch) {
            u_pitch = +(kp_pitch * ctrl_filt_ey_ + kd_pitch * de_y + ki_pitch * ctrl_integ_ey_);
        } else if (std::abs(ki_pitch * ctrl_integ_ey_) > 1e-6f) {
            u_pitch = +(ki_pitch * ctrl_integ_ey_);
        }
        u_yaw *= static_cast<float>(yaw_gain_);
        u_yaw = std::clamp(u_yaw, -max_delta_yaw_deg, max_delta_yaw_deg);
        u_pitch = std::clamp(u_pitch, -max_delta_pitch_deg, max_delta_pitch_deg);
        if (stable_online_mode_ && stable_transition_countdown_ > 0) {
            const float s = static_cast<float>(stable_transition_countdown_) /
                            static_cast<float>(std::max(1, stable_transition_blend_frames_));
            const float blend_scale = std::clamp(1.0f - 0.55f * s, 0.35f, 1.0f);
            u_yaw *= blend_scale;
            u_pitch *= blend_scale;
            stable_transition_countdown_ -= 1;
        }

        static float search_phase_yaw = 0.f;
        static float search_phase_pitch = 0.f;
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
            if (center_ref_cid >= 0) {
                Blob* aim = nullptr;
                double best_s = -1e9;
                for (auto& b : blobs) {
                    if (b.cid != center_ref_cid) continue;
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
                    const float ex_vis = bx - laser_center.x;
                    const float ey_vis = by - laser_y_for_track;
                    servo_yaw_f += std::clamp(-0.38f * ex_vis, -9.0f, 9.0f);
                    servo_pitch_f += std::clamp(0.09f * ey_vis, -8.0f, 8.0f);
                }
            }
        } else {
            const float min_step_y = min_step_active;
            if (std::abs(u_yaw) < min_step_y && std::abs(ctrl_filt_ex_) > static_cast<float>(deadband_yaw) + 1e-3f) {
                u_yaw = (ctrl_filt_ex_ > 0.0f) ? -min_step_y : min_step_y;
            }
            if (std::abs(u_pitch) < 0.35f && std::abs(ctrl_filt_ey_) > (deadband_pitch + 8)) {
                u_pitch = (u_pitch >= 0.0f) ? 0.35f : -0.35f;
            }
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

        const int center_id = center_ref_valid ? center_ref_cid : -1;
        const int target_id = (target && target_raw.x >= 0.0f) ? target->cid : -1;
        const bool detect_ok = (target_id >= 0);

        cv::putText(vis, "FPS: " + std::to_string(static_cast<int>(current_fps_)), cv::Point(10, 28),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::putText(vis, std::string("12-color track lab=") + std::to_string(static_cast<int>(lab_t)), cv::Point(10, 54),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        {
            char psz[96];
            std::snprintf(psz, sizeof(psz), "zoom=%.2f bright_px=%d", std::clamp(center_roi_zoom_, 1.0, 1.6), bright_px);
            cv::putText(vis, psz, cv::Point(10, 72), cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(200, 200, 255), 2);
        }
        cv::circle(vis, laser_center, 5, cv::Scalar(0, 0, 255), -1);
        {
            const float inv_s = (scale > 1e-6f) ? (1.0f / scale) : 1.0f;
            if (center_ref_valid && cen_pick) {
                const float cx = center_ref_x * inv_s;
                const float cy = center_ref_y * inv_s;
                const int r = std::max(6, static_cast<int>(std::lround(std::max(8.0f, center_ref_rad) * inv_s)));
                cv::circle(vis, cv::Point(static_cast<int>(std::lround(cx)), static_cast<int>(std::lround(cy))), r,
                    cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            }
            if (target && !target->contour.empty()) {
                std::vector<cv::Point> tpts;
                for (const auto& p : target->contour) {
                    tpts.emplace_back(static_cast<int>(std::lround(p.x * inv_s)), static_cast<int>(std::lround(p.y * inv_s)));
                }
                const cv::Rect br = cv::boundingRect(tpts);
                cv::rectangle(vis, br, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
            }
        }
        {
            char dbg_ctrl[120];
            std::snprintf(dbg_ctrl, sizeof(dbg_ctrl), "dx=%d dy=%d fex=%.1f uy=%.2f up=%.2f", x_error, y_error,
                static_cast<double>(ctrl_filt_ex_), static_cast<double>(u_yaw), static_cast<double>(u_pitch));
            cv::putText(vis, dbg_ctrl, cv::Point(10, 92), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
        cv::putText(vis, "hit_px=" + std::to_string(static_cast<int>(std::lround(hit_error_px))), cv::Point(10, 118),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 200, 255), 2);
        cv::putText(vis,
            std::string("detect=") + (detect_ok ? "OK" : "LOST") + " cid=" + std::to_string(center_id) +
                " tid=" + std::to_string(target_id),
            cv::Point(10, 144), cv::FONT_HERSHEY_SIMPLEX, 0.6,
            detect_ok ? cv::Scalar(0, 255, 120) : cv::Scalar(0, 140, 255), 2);
        if (aim_workflow_enabled_) {
            cv::putText(
                vis, aim_started_ ? "AIM:TRACK_SQUARE" : "AIM:LOCK_CENTER",
                cv::Point(10, 216), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        }
        if (color_only_active_) {
            cv::putText(vis, "COLOR_ONLY", cv::Point(10, 192), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 255), 2);
        }
        cv::putText(vis,
            "miss=" + std::to_string(track_.miss_count) + " acq=" + std::to_string(acquire_lost_frames_) +
                (allow_predict_coast ? " COAST" : "") + (search_mode ? " SEARCH" : "") +
                " blobs=" + std::to_string(static_cast<int>(blobs.size())),
            cv::Point(10, 168), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 220, 0), 2);

        // 【5】整度化后发到 /servo_control；终端里能看到「发布舵机指令」就说明链路通了
        int pitch_cmd = wrap_deg_0_359(safe_round_servo_deg(servo_pitch_f));
        int yaw_cmd = wrap_deg_0_359(safe_round_servo_deg(servo_yaw_f));
        pitch_cmd = std::clamp(pitch_cmd, static_cast<int>(std::floor(pitch_lo)), static_cast<int>(std::ceil(pitch_hi)));
        yaw_cmd = std::clamp(yaw_cmd, static_cast<int>(std::floor(yaw_lo)), static_cast<int>(std::ceil(yaw_hi)));

        if (control_enabled_ && detect_ok_control && !search_mode && !should_hold) {
            if (yaw_cmd == last_pub_yaw_cmd_) {
                yaw_cmd_stale_frames_ += 1;
            } else {
                yaw_cmd_stale_frames_ = 0;
            }
            if (yaw_cmd_stale_frames_ >= 2 && std::abs(x_error) > 8 &&
                std::abs(ctrl_filt_ex_) > static_cast<float>(deadband_yaw) + 0.5f) {
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
            std_msgs::msg::Float64MultiArray dbg;
            dbg.data.resize(20);
            dbg.data[0] = static_cast<double>(hit_error_px);
            dbg.data[1] = static_cast<double>(x_error);
            dbg.data[2] = static_cast<double>(y_error);
            dbg.data[3] = detect_ok ? 1.0 : 0.0;
            dbg.data[4] = should_hold ? 1.0 : 0.0;
            dbg.data[5] = static_cast<double>(track_.miss_count);
            dbg.data[6] = static_cast<double>(servo_pitch_f);
            dbg.data[7] = static_cast<double>(servo_yaw_f);
            dbg.data[8] = current_fps_;
            dbg.data[9] = control_enabled_ ? 1.0 : 0.0;
            dbg.data[10] = 0.0;
            dbg.data[11] = static_cast<double>(ctrl_filt_ex_);
            dbg.data[12] = static_cast<double>(u_yaw);
            dbg.data[13] = static_cast<double>(pitch_cmd);
            dbg.data[14] = static_cast<double>(yaw_cmd);
            dbg.data[15] = static_cast<double>(pitch_cmd);
            dbg.data[16] = static_cast<double>(yaw_cmd);
            dbg.data[17] = static_cast<double>(center_id);
            dbg.data[18] = static_cast<double>(target_id);
            dbg.data[19] = static_cast<double>(frame_center_lab_cid_);
            tracking_debug_pub_->publish(dbg);
        }
        if (publish_center_color_id_) {
            std_msgs::msg::Int32 cc;
            cc.data = center_id;
            center_color_pub_->publish(cc);
        }

        if (show_vis_window_) {
            static bool vis_window_ready = false;
            static bool highgui_disabled = false;
            if (!highgui_disabled) {
                try {
                    if (!vis_window_ready) {
                        cv::namedWindow("Received Image", cv::WINDOW_NORMAL);
                        cv::resizeWindow("Received Image", std::max(320, vis.cols), std::max(240, vis.rows));
                        if (runtime_tuning_enable_) init_runtime_tuning_panel_();
                        vis_window_ready = true;
                        RCLCPP_INFO(this->get_logger(), "[TRACE] OpenCV 窗口 Received Image");
                    }
                    if (runtime_tuning_enable_) refresh_runtime_tuning_from_panel_();
                    cv::imshow("Received Image", vis);
                    if (static_cast<char>(cv::waitKey(1)) == 27) rclcpp::shutdown();
                } catch (const cv::Exception& e) {
                    highgui_disabled = true;
                    RCLCPP_ERROR(this->get_logger(), "OpenCV GUI: %s", e.what());
                }
            }
        }
        if (sector) {
            last_sector_pick_work_ = cv::Point2f(static_cast<float>(sector->cx), static_cast<float>(sector->cy));
            has_last_sector_pick_ = true;
        } else if (!color_only_active_) {
            // 在普通路径下连续丢失时释放粘滞，避免长期锁死在旧区域。
            has_last_sector_pick_ = false;
        }

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
            msg.servo_id = servo.first;
            msg.servo_angle = std::clamp(wrap_deg_0_359(servo.second), lo, hi);
            servo_pub_->publish(msg);
        }
    }

    void init_runtime_tuning_panel_() {
        if (panel_inited_) return;
        cv::createTrackbar("rt_kp x1000", "Received Image", &tb_kp_x1000_, 120);
        cv::createTrackbar("rt_kd x1000", "Received Image", &tb_kd_x1000_, 120);
        cv::createTrackbar("rt_max_delta x100", "Received Image", &tb_max_delta_x100_, 300);
        cv::createTrackbar("rt_min_step x100", "Received Image", &tb_min_step_x100_, 100);
        cv::createTrackbar("rt_deadband px", "Received Image", &tb_deadband_px_, 40);
        cv::createTrackbar("rt_errlp x100", "Received Image", &tb_errlp_x100_, 100);
        cv::createTrackbar("rt_smooth x100", "Received Image", &tb_smooth_x100_, 100);
        cv::createTrackbar("rt_hold_frames", "Received Image", &tb_hold_frames_, 12);
        cv::createTrackbar("rt_trans_frames", "Received Image", &tb_trans_frames_, 20);
        tb_kp_x1000_ = std::clamp(static_cast<int>(std::lround(rt_kp_ * 1000.0)), 0, 120);
        tb_kd_x1000_ = std::clamp(static_cast<int>(std::lround(rt_kd_ * 1000.0)), 0, 120);
        tb_max_delta_x100_ = std::clamp(static_cast<int>(std::lround(rt_max_delta_deg_ * 100.0)), 0, 300);
        tb_min_step_x100_ = std::clamp(static_cast<int>(std::lround(rt_min_step_deg_ * 100.0)), 0, 100);
        tb_deadband_px_ = std::clamp(static_cast<int>(std::lround(rt_deadband_px_)), 0, 40);
        tb_errlp_x100_ = std::clamp(static_cast<int>(std::lround(rt_err_lp_beta_ * 100.0)), 0, 100);
        tb_smooth_x100_ = std::clamp(static_cast<int>(std::lround(rt_servo_smooth_beta_ * 100.0)), 5, 95);
        tb_hold_frames_ = std::clamp(rt_hold_frames_, 0, 12);
        tb_trans_frames_ = std::clamp(rt_transition_frames_, 0, 20);
        cv::setTrackbarPos("rt_kp x1000", "Received Image", tb_kp_x1000_);
        cv::setTrackbarPos("rt_kd x1000", "Received Image", tb_kd_x1000_);
        cv::setTrackbarPos("rt_max_delta x100", "Received Image", tb_max_delta_x100_);
        cv::setTrackbarPos("rt_min_step x100", "Received Image", tb_min_step_x100_);
        cv::setTrackbarPos("rt_deadband px", "Received Image", tb_deadband_px_);
        cv::setTrackbarPos("rt_errlp x100", "Received Image", tb_errlp_x100_);
        cv::setTrackbarPos("rt_smooth x100", "Received Image", tb_smooth_x100_);
        cv::setTrackbarPos("rt_hold_frames", "Received Image", tb_hold_frames_);
        cv::setTrackbarPos("rt_trans_frames", "Received Image", tb_trans_frames_);
        panel_inited_ = true;
    }

    void refresh_runtime_tuning_from_panel_() {
        if (!panel_inited_) return;
        rt_kp_ = std::clamp(tb_kp_x1000_ / 1000.0, 0.0, 0.12);
        rt_kd_ = std::clamp(tb_kd_x1000_ / 1000.0, 0.0, 0.12);
        rt_max_delta_deg_ = std::clamp(tb_max_delta_x100_ / 100.0, 0.0, 3.0);
        rt_min_step_deg_ = std::clamp(tb_min_step_x100_ / 100.0, 0.0, 1.0);
        rt_deadband_px_ = std::clamp(static_cast<double>(tb_deadband_px_), 0.0, 40.0);
        rt_err_lp_beta_ = std::clamp(tb_errlp_x100_ / 100.0, 0.0, 1.0);
        rt_servo_smooth_beta_ = std::clamp(tb_smooth_x100_ / 100.0, 0.05, 0.95);
        rt_hold_frames_ = std::clamp(tb_hold_frames_, 0, 12);
        rt_transition_frames_ = std::clamp(tb_trans_frames_, 0, 20);
    }

    image_transport::Subscriber image_sub_;
    image_transport::Publisher trace_debug_pub_;
    std::string trace_debug_image_topic_;
    bool publish_trace_debug_image_ = true;
    rclcpp::Publisher<ServoMsg>::SharedPtr servo_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr center_color_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr aim_command_sub_;
    bool publish_tracking_debug_ = true;
    bool show_vis_window_ = true;
    bool runtime_tuning_enable_ = true;
    bool panel_inited_ = false;
    int tb_kp_x1000_ = 13;
    int tb_kd_x1000_ = 12;
    int tb_max_delta_x100_ = 50;
    int tb_min_step_x100_ = 18;
    int tb_deadband_px_ = 12;
    int tb_errlp_x100_ = 42;
    int tb_smooth_x100_ = 32;
    int tb_hold_frames_ = 3;
    int tb_trans_frames_ = 5;
    double rt_kp_ = 0.013;
    double rt_kd_ = 0.012;
    double rt_max_delta_deg_ = 0.50;
    double rt_min_step_deg_ = 0.18;
    double rt_deadband_px_ = 12.0;
    double rt_err_lp_beta_ = 0.42;
    double rt_servo_smooth_beta_ = 0.32;
    int rt_hold_frames_ = 3;
    int rt_transition_frames_ = 5;
    rclcpp::Time last_frame_time_;
    int frame_count_ = 0;
    double current_fps_ = 30.0;
    std::string energy_mode_{"large"};
    bool is_small_mode_ = false;
    bool control_enabled_ = true;
    double last_no_image_warn_sec_{-1e9};
    double laser_center_ratio_x_ = 0.5;
    double laser_center_ratio_y_ = 0.5;
    double laser_aim_offset_x_px_ = 0.0;
    double laser_aim_offset_y_px_ = 0.0;
    double init_pitch_deg_ = 340.0;
    double init_yaw_deg_ = 170.0;
    int startup_hold_frames_ = 60;
    int startup_countdown_ = 0;
    int servo_id_pitch_ = 8;
    int servo_id_yaw_ = 11;
    double servo_pitch_min_deg_ = 0.0;
    double servo_pitch_max_deg_ = 360.0;
    double servo_yaw_min_deg_ = 0.0;
    double servo_yaw_max_deg_ = 360.0;
    double servo_pitch_travel_half_span_deg_ = 40.0;
    double servo_yaw_travel_half_span_deg_ = 40.0;
    double servo_pitch_travel_pos_deg_ = 30.0;
    double servo_pitch_travel_neg_deg_ = 10.0;
    double servo_yaw_travel_pos_deg_ = 20.0;
    double servo_yaw_travel_neg_deg_ = 20.0;
    int trace_proc_width_cap_ = 960;
    int trace_proc_width_cap_small_ = 880;
    double lab_thresh_ = 71.5;
    double small_lab_thresh_ = 58.0;
    int lab_preprocess_blur_ksize_ = 15;
    int lab_mask_morph_ksize_ = 5;
    double vis_input_gamma_ = 0.76;
    double vis_clahe_l_clip_limit_ = 0.0;
    bool lab_mask_canny_bridge_ = false;
    int lab_canny_thresh1_ = 80;
    int lab_canny_thresh2_ = 160;
    int lab_canny_dilate_px_ = 5;
    int lab_canny_bridge_mask_dilate_ = 2;
    double ctrl_integ_max_px_s_ = 120.0;
    double kp_ = 0.026;
    double kd_ = 0.022;
    double ki_yaw_ = 0.0;
    double ki_pitch_ = 0.0;
    double small_kp_ = 0.013;
    double small_kd_ = 0.012;
    double small_ki_yaw_ = 0.0;
    double small_ki_pitch_ = 0.0;
    double err_lp_beta_yaw_ = 0.52;
    double err_lp_beta_pitch_ = 0.30;
    double small_err_lp_beta_yaw_ = 0.46;
    double small_err_lp_beta_pitch_ = 0.42;
    int deadband_px_ = 2;
    int deadband_yaw_px_ = 1;
    double max_delta_deg_ = 1.20;
    int small_deadband_ = 12;
    double small_max_delta_deg_ = 0.50;
    double yaw_gain_ = 1.72;
    double yaw_kp_mult_ = 1.90;
    double yaw_min_step_deg_ = 0.52;
    double small_min_step_deg_ = 0.18;
    double small_lead_ms_ = 28.0;
    double small_settle_px_ = 14.0;
    int small_settle_frames_ = 5;
    double near_tgt_yaw_kp_scale_ = 0.92;
    double search_yaw_amp_deg_ = 32.0;
    double search_pitch_amp_deg_ = 12.0;
    double search_phase_step_ = 0.10;
    double search_pitch_phase_step_ = 0.035;
    int search_enter_frames_ = 2;
    double small_center_hub_dist_frac_ = 0.32;
    double settle_px_ = 8.0;
    int settle_frames_ = 4;
    int settle_ok_count_ = 0;
    double servo_angle_smooth_beta_ = 0.32;
    double roi_frac_ = 0.77;
    double center_roi_zoom_ = 1.0;
    bool suppress_bright_lights_ = false;
    int bright_hsv_v_min_ = 252;
    int bright_hsv_s_max_ = 58;
    int bright_bgr_channel_min_ = 244;
    int bright_mask_dilate_ = 5;
    int bright_inpaint_radius_ = 5;
    double bright_inpaint_max_area_frac_ = 0.012;
    double pitch_laser_lift_px_ = 22.0;
    double smooth_target_alpha_scale_ = 1.0;
    double target_step_max_px_ = 88.0;
    int predict_coast_frames_ = 12;
    double predict_lead_ms_ = 62.0;
    int predict_arm_frames_ = 8;
    double predict_vel_beta_ = 0.28;
    bool small_known_speed_predict_enable_ = true;
    double small_known_speed_deg_s_ = 60.0;
    int small_known_speed_dir_sign_ = 1;
    double small_known_speed_blend_ = 0.68;
    bool color_lock_enable_ = true;
    int color_lock_stable_frames_ = 4;
    int color_lock_timeout_frames_ = 36;
    int reacquire_blend_frames_ = 2;
    bool predict_armed_ = false;
    int predict_good_streak_ = 0;
    double lab_min_area_seg_ = 50.0;
    double center_circle_area_ratio_min_ = 0.8;
    double center_rect_area_ratio_max_ = 0.9;
    double center_max_dist_ratio_ = 0.38;
    double sector_min_area_ = 220.0;
    double sector_max_area_ratio_ = 0.52;
    double square_approx_poly_eps_ = 0.024;
    double center_circularity_min_ = 0.74;
    double center_hsv_max_dist_sq_ = 6200.0;
    double sector_hsv_max_dist_sq_ = 5200.0;
    bool publish_center_color_id_ = true;
    int frame_center_lab_cid_ = -1;
    int acquire_lost_frames_ = 0;
    int center_color_stable_count_ = 0;
    int center_color_miss_count_ = 0;
    int center_color_relock_miss_count_ = 0;
    int center_color_candidate_ = -1;
    int locked_center_color_id_ = -1;
    int center_color_relock_after_miss_frames_ = 10;
    bool color_only_after_lock_enable_ = true;
    int color_only_arm_frames_ = 6;
    bool disable_blob_fallback_ = true;
    bool color_only_active_ = false;
    int color_only_miss_count_ = 0;
    double fallback_blob_max_area_ratio_ = 0.10;
    double fallback_blob_min_dist_center_radius_ = 0.35;
    double fallback_blob_max_dist_center_radius_ = 3.0;
    bool aim_workflow_enabled_ = false;
    bool aim_started_ = false;
    bool aim_relock_requested_ = false;
    bool last_aim_started_ = false;
    bool stable_online_mode_ = true;
    int stable_target_hold_frames_ = 3;
    int stable_transition_blend_frames_ = 5;
    int stable_transition_countdown_ = 0;
    bool stable_disable_predict_ = true;
    bool stable_disable_search_ = true;
    double stable_fixed_dt_s_ = 0.04;
    bool stable_has_last_valid_target_ = false;
    int stable_hold_count_ = 0;
    cv::Point2f stable_last_valid_target_{0.f, 0.f};
    double center_square_area_rel_tol_ = 0.45;
    double center_square_diam_side_rel_tol_ = 0.35;
    double square_max_dist_center_radius_ = 2.8;
    bool sector_prefer_last_enable_ = true;
    double sector_prefer_radius_px_ = 85.0;
    double sector_prefer_bonus_ = 0.35;
    bool has_last_sector_pick_ = false;
    cv::Point2f last_sector_pick_work_{0.f, 0.f};
    int reacquire_countdown_ = 0;
    enum class TrackControlState { TRACKING = 0, COASTING = 1, REACQUIRE = 2, SEARCH = 3 };
    TrackControlState track_state_ = TrackControlState::SEARCH;
    cv::Point2f last_model_predict_{0.f, 0.f};
    cv::Point2f model_vel_{0.f, 0.f};
    TrackState track_;
    cv::Point2f vel_ema_{0.f, 0.f};
    float ctrl_prev_ex_ = 0.f;
    float ctrl_prev_ey_ = 0.f;
    float ctrl_filt_ex_ = 0.f;
    float ctrl_filt_ey_ = 0.f;
    float ctrl_integ_ex_ = 0.f;
    float ctrl_integ_ey_ = 0.f;
    int last_pub_yaw_cmd_ = -999;
    int yaw_cmd_stale_frames_ = 0;
    bool servo_out_lp_inited_ = false;
    float servo_out_lp_pitch_ = 0.f;
    float servo_out_lp_yaw_ = 0.f;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}

