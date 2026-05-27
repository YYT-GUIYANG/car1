/**
 * read_image.cpp — 相机采集节点
 *
 * 负责打开 USB 相机、做可选的 gamma/CLAHE/白平衡，然后把图发布到 /processed_image。
 * trace_calculator 只订阅这个话题，不直接碰相机，这样调视觉和调相机可以分开跑。
 */
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/header.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int video_index_from_dev_path(const std::filesystem::path& dev_path) {
    const std::string base = dev_path.filename().string();
    constexpr std::size_t kVideoPrefix = 5;  // "video"
    if (base.size() > kVideoPrefix && base.rfind("video", 0) == 0) {
        return std::atoi(base.c_str() + kVideoPrefix);
    }
    return -1;
}

std::vector<int> parse_comma_separated_ints(const std::string& s) {
    std::vector<int> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        while (!item.empty() && (item.front() == ' ' || item.front() == '\t')) {
            item.erase(0, 1);
        }
        while (!item.empty() && (item.back() == ' ' || item.back() == '\t')) {
            item.pop_back();
        }
        if (!item.empty()) {
            out.push_back(std::atoi(item.c_str()));
        }
    }
    return out;
}

/**
 * 枚举 /dev/v4l/by-id 下 usb-* 设备；解析符号链接到 /dev/videoN，
 * 跳过 skip_video_indices 中的节点（默认跳过 0、1，避免笔记本内置 Chicony 排在字母序前面误开）。
 */
std::vector<std::string> list_usb_v4l_by_id_paths_filtered(const std::vector<int>& skip_video_indices) {
    std::vector<std::string> out;
    namespace fs = std::filesystem;
    const fs::path base("/dev/v4l/by-id");
    std::error_code ec;
    if (!fs::exists(base, ec)) {
        return out;
    }
    for (const auto& entry : fs::directory_iterator(base, ec)) {
        if (ec) {
            break;
        }
        const std::string name = entry.path().filename().string();
        if (name.size() < 4 || name.compare(0, 4, "usb-") != 0) {
            continue;
        }
        std::error_code ec2;
        const fs::path can = fs::weakly_canonical(entry.path(), ec2);
        if (ec2) {
            continue;
        }
        const int vi = video_index_from_dev_path(can);
        if (vi >= 0 &&
            std::find(skip_video_indices.begin(), skip_video_indices.end(), vi) != skip_video_indices.end()) {
            continue;
        }
        out.push_back(entry.path().string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

void push_unique(std::vector<std::string>* v, const std::string& p) {
    if (std::find(v->begin(), v->end(), p) == v->end()) {
        v->push_back(p);
    }
}

/**
 * 仅使用 V4L2 打开摄像头。Linux 上不要用 CAP_ANY 回退，否则会走 GStreamer，
 * 对 /dev/video* 常产生 “no source element for URI” 且无法采集。
 * 路径失败时再尝试解析到的 /dev/videoN 的整数索引（部分机器上更稳）。
 */
bool open_capture_v4l_only(const std::string& device, cv::VideoCapture* cap) {
    if (!cap) {
        return false;
    }
    cap->release();
    if (!device.empty()) {
        *cap = cv::VideoCapture(device, cv::CAP_V4L2);
        if (cap->isOpened()) {
            return true;
        }
        cap->release();
    }
    namespace fs = std::filesystem;
    if (!device.empty() && device.front() == '/') {
        std::error_code ec;
        const fs::path can = fs::weakly_canonical(fs::path(device), ec);
        if (!ec) {
            const int idx = video_index_from_dev_path(can);
            if (idx >= 0) {
                *cap = cv::VideoCapture(idx, cv::CAP_V4L2);
                if (cap->isOpened()) {
                    return true;
                }
                cap->release();
            }
        }
    }
    if (!device.empty()) {
        bool all_digit = true;
        for (unsigned char ch : device) {
            if (!std::isdigit(static_cast<unsigned char>(ch))) {
                all_digit = false;
                break;
            }
        }
        if (all_digit) {
            const int idx = std::atoi(device.c_str());
            if (idx >= 0) {
                *cap = cv::VideoCapture(idx, cv::CAP_V4L2);
                if (cap->isOpened()) {
                    return true;
                }
                cap->release();
            }
        }
    }
    return false;
}

}  // namespace

class OpenCVTestNode : public rclcpp::Node
{
public:
    OpenCVTestNode() : Node("opencv_test_node")
    {
        brightness_adjust_enabled_ = this->declare_parameter<bool>("brightness_adjust_enabled", false);
        brightness_scale_ = this->declare_parameter<double>("brightness_scale", 1.0);
        publish_period_ms_ = static_cast<int>(std::clamp(
            this->declare_parameter<int>("publish_period_ms", 10),
            int64_t{5},
            int64_t{200}));
        input_gamma_ = std::clamp(this->declare_parameter<double>("input_gamma", 1.0), 0.35, 3.5);
        clahe_l_clip_limit_ = std::clamp(this->declare_parameter<double>("clahe_l_clip_limit", 0.0), 0.0, 12.0);
        gray_world_wb_enabled_ = this->declare_parameter<bool>("gray_world_wb_enabled", false);
        gray_world_wb_strength_ =
            std::clamp(this->declare_parameter<double>("gray_world_wb_strength", 0.72), 0.0, 1.0);

        // V4L2 / UVC：默认不改，避免破坏现有行为。颜色/亮度乱跳时可试：manual 曝光 + 关 AWB。
        // AUTO_EXPOSURE：OpenCV+V4L2 常用 1=手动曝光(MANUAL)、3=自动(AUTO)；-1=不设置。
        camera_v4l_auto_exposure_ = static_cast<int>(this->declare_parameter<int>("camera_v4l_auto_exposure", -1));
        camera_v4l_set_exposure_ = this->declare_parameter<bool>("camera_v4l_set_exposure", false);
        camera_v4l_exposure_ = this->declare_parameter<double>("camera_v4l_exposure", -6.0);
        // AUTO_WB：0=关、1=开；-1=不设置。关 AWB 后部分相机可再设 WB_TEMPERATURE。
        camera_v4l_auto_wb_ = static_cast<int>(this->declare_parameter<int>("camera_v4l_auto_wb", -1));
        camera_v4l_wb_temperature_ = static_cast<int>(this->declare_parameter<int>("camera_v4l_wb_temperature", -1));
        camera_v4l_gain_ = this->declare_parameter<double>("camera_v4l_gain", -1.0);

        const std::string camera_device = this->declare_parameter<std::string>("camera_device", "");
        const std::string skip_v4l_video_indices_str =
            this->declare_parameter<std::string>("skip_v4l_video_indices", "0,1");
        const std::vector<int> skip_v4l_video_indices = parse_comma_separated_ints(skip_v4l_video_indices_str);

        // 默认严格：仅用 USB by-id 或显式路径，避免误开笔记本内置 /dev/video0。
        const bool strict_external_camera = this->declare_parameter<bool>("strict_external_camera", true);
        std::vector<std::string> camera_candidates;
        if (!camera_device.empty()) {
            camera_candidates.push_back(camera_device);
        } else {
            for (const auto& p : list_usb_v4l_by_id_paths_filtered(skip_v4l_video_indices)) {
                push_unique(&camera_candidates, p);
            }
            // 兼容旧工程固定路径（若未出现在动态列表中）
            push_unique(
                &camera_candidates,
                "/dev/v4l/by-id/usb-DHZJ-250731-A_Nozzle_Alignment_Camera-video-index0");
            push_unique(
                &camera_candidates,
                "/dev/v4l/by-id/usb-DHZJ-250731-A_Nozzle_Alignment_Camera-video-index1");
            // 非严格：仅尝试较高编号 video 节点（不尝试 video0/1，一般为内置摄像头）
            if (!strict_external_camera) {
                push_unique(&camera_candidates, "/dev/video7");
                push_unique(&camera_candidates, "/dev/video6");
                push_unique(&camera_candidates, "/dev/video5");
                push_unique(&camera_candidates, "/dev/video4");
                push_unique(&camera_candidates, "/dev/video3");
                push_unique(&camera_candidates, "/dev/video2");
            }
            RCLCPP_INFO(
                this->get_logger(),
                "[CAMERA] 候选设备共 %zu 个（strict_external_camera=%s，skip_v4l_video_indices=%s）"
                "，将按顺序尝试",
                camera_candidates.size(),
                strict_external_camera ? "true" : "false",
                skip_v4l_video_indices_str.c_str());
        }
        std::string selected_device;
        for (const auto &device : camera_candidates)
        {
            if (open_capture_v4l_only(device, &cap_)) {
                selected_device = device;
                break;
            }
            RCLCPP_WARN(this->get_logger(), "摄像头设备打开失败：%s，尝试下一个设备", device.c_str());
        }

        // ============== 关键修改2：先不强制分辨率，避免摄像头不支持 ==============
        // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        // cap_.set(cv::CAP_PROP_FPS, 30);

        if (!cap_.isOpened())
        {
            if (strict_external_camera) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "摄像头打开失败！严格模式下仅允许外接USB摄像头。请检查外接相机连接/占用，"
                    "若需临时回退/dev/video*，启动时设置 strict_external_camera:=false"
                );
            } else {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "摄像头打开失败！请检查：设备是否插入、是否被占用、当前用户是否在 video 组；"
                    "可执行 v4l2-ctl --list-devices 确认采集节点；启动时可用 camera_device:=/dev/videoN 强制指定。"
                );
            }
            throw std::runtime_error("read_image: camera open failed (see log above)");
        }
        // 预读3帧丢弃（解决摄像头预热无效帧）
        cv::Mat dummy;
        for(int i=0; i<3; i++) cap_.read(dummy);

        apply_v4l_user_controls_();

        const int cap_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        const int cap_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        const double cap_fps = cap_.get(cv::CAP_PROP_FPS);

        // 便于 grep / 英文环境：同时打出 CAMERA_OPEN 与中文
        RCLCPP_INFO(
            this->get_logger(),
            "======== CAMERA_OPEN OK / 摄像头已打开 ========  device=%s",
            selected_device.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "[CAMERA] Opened successfully | 打开成功  path=%s  resolution=%dx%d  reported_fps=%.1f",
            selected_device.c_str(),
            cap_w,
            cap_h,
            cap_fps);
        RCLCPP_INFO(
            this->get_logger(),
            "亮度策略 brightness: %s (scale=%.2f)",
            brightness_adjust_enabled_ ? "software" : "passthrough",
            brightness_scale_);
        RCLCPP_INFO(
            this->get_logger(),
            "输入预处理 input_gamma=%.2f clahe_L_clip=%.2f(0=关) gray_world_wb=%s(strength=%.2f) publish_period_ms=%d",
            input_gamma_,
            clahe_l_clip_limit_,
            gray_world_wb_enabled_ ? "on" : "off",
            gray_world_wb_strength_,
            publish_period_ms_);
        if (camera_v4l_auto_exposure_ >= 0 || camera_v4l_set_exposure_ || camera_v4l_auto_wb_ >= 0 ||
            camera_v4l_wb_temperature_ >= 0 || camera_v4l_gain_ >= 0.0) {
            RCLCPP_INFO(
                this->get_logger(),
                "V4L 用户控制 camera_v4l_auto_exposure=%d set_exposure=%s exposure=%.4f auto_wb=%d wb_temp=%d gain=%.4f",
                camera_v4l_auto_exposure_,
                camera_v4l_set_exposure_ ? "true" : "false",
                camera_v4l_exposure_,
                camera_v4l_auto_wb_,
                camera_v4l_wb_temperature_,
                camera_v4l_gain_);
        }

        // 必须用绝对话题名，否则与 trace_calculator 各带节点前缀，二者永远对不上
        image_pub_ = image_transport::create_publisher(this, "/processed_image", rclcpp::QoS(10).get_rmw_qos_profile());
        RCLCPP_INFO(
            this->get_logger(),
            "[CAMERA] Publishing absolute topic=/processed_image  encoding=bgr8  timer=%dms",
            publish_period_ms_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms_),
            std::bind(&OpenCVTestNode::image_callback, this)
        );
    }

    ~OpenCVTestNode()
    {
        if (cap_.isOpened()) cap_.release();
    }

private:
    void apply_v4l_user_controls_()
    {
        if (!cap_.isOpened()) {
            return;
        }
        if (camera_v4l_auto_exposure_ >= 0) {
            const bool ok =
                cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, static_cast<double>(camera_v4l_auto_exposure_));
            const double rb = cap_.get(cv::CAP_PROP_AUTO_EXPOSURE);
            RCLCPP_INFO(
                this->get_logger(),
                "[CAMERA] V4L AUTO_EXPOSURE 请求=%d %s 读回=%.6f（1 常为手动、3 常为自动；以驱动为准）",
                camera_v4l_auto_exposure_,
                ok ? "set_ok" : "set_fail",
                rb);
        }
        if (camera_v4l_set_exposure_) {
            const bool ok = cap_.set(cv::CAP_PROP_EXPOSURE, camera_v4l_exposure_);
            const double rb = cap_.get(cv::CAP_PROP_EXPOSURE);
            RCLCPP_INFO(
                this->get_logger(),
                "[CAMERA] V4L EXPOSURE 请求=%.6f %s 读回=%.6f（UVC 常为负值档位，需实测）",
                camera_v4l_exposure_,
                ok ? "set_ok" : "set_fail",
                rb);
        }
        if (camera_v4l_auto_wb_ >= 0) {
            const bool ok = cap_.set(cv::CAP_PROP_AUTO_WB, static_cast<double>(camera_v4l_auto_wb_));
            const double rb = cap_.get(cv::CAP_PROP_AUTO_WB);
            RCLCPP_INFO(
                this->get_logger(),
                "[CAMERA] V4L AUTO_WB 请求=%d %s 读回=%.6f（0 关 / 1 开）",
                camera_v4l_auto_wb_,
                ok ? "set_ok" : "set_fail",
                rb);
        }
        if (camera_v4l_wb_temperature_ >= 0) {
            const bool ok =
                cap_.set(cv::CAP_PROP_WB_TEMPERATURE, static_cast<double>(camera_v4l_wb_temperature_));
            const double rb = cap_.get(cv::CAP_PROP_WB_TEMPERATURE);
            RCLCPP_INFO(
                this->get_logger(),
                "[CAMERA] V4L WB_TEMPERATURE 请求=%d %s 读回=%.6f",
                camera_v4l_wb_temperature_,
                ok ? "set_ok" : "set_fail",
                rb);
        }
        if (camera_v4l_gain_ >= 0.0) {
            const bool ok = cap_.set(cv::CAP_PROP_GAIN, camera_v4l_gain_);
            const double rb = cap_.get(cv::CAP_PROP_GAIN);
            RCLCPP_INFO(
                this->get_logger(),
                "[CAMERA] V4L GAIN 请求=%.4f %s 读回=%.6f",
                camera_v4l_gain_,
                ok ? "set_ok" : "set_fail",
                rb);
        }
    }

    static void apply_gamma_bgr_inplace(cv::Mat& bgr, double gamma)
    {
        if (bgr.empty() || bgr.type() != CV_8UC3) {
            return;
        }
        if (std::abs(gamma - 1.0) < 0.02) {
            return;
        }
        const double g = 1.0 / std::clamp(gamma, 0.35, 3.5);
        cv::Mat lut(1, 256, CV_8U);
        uchar* pl = lut.ptr<uchar>(0);
        for (int i = 0; i < 256; ++i) {
            pl[i] = cv::saturate_cast<uchar>(std::pow(static_cast<double>(i) / 255.0, g) * 255.0);
        }
        cv::LUT(bgr, lut, bgr);
    }

    // 灰世界白平衡：按通道均值拉齐，strength 与原始色按比例混合（减轻色偏，大光照变化时可试）
    static void apply_gray_world_wb_inplace(cv::Mat& bgr, double strength)
    {
        if (bgr.empty() || bgr.type() != CV_8UC3 || strength < 0.02) {
            return;
        }
        const cv::Scalar m = cv::mean(bgr);
        const double mb = m[0], mg = m[1], mr = m[2];
        const double gray = (mb + mg + mr) / 3.0;
        if (gray < 1.0) {
            return;
        }
        const double kb = gray / std::max(1.0, mb);
        const double kg = gray / std::max(1.0, mg);
        const double kr = gray / std::max(1.0, mr);
        const float s = static_cast<float>(std::clamp(strength, 0.0, 1.0));
        std::vector<cv::Mat> ch(3);
        cv::split(bgr, ch);
        for (int c = 0; c < 3; ++c) {
            const double k_raw = (c == 0) ? kb : (c == 1) ? kg : kr;
            const double k = 1.0 + static_cast<double>(s) * (k_raw - 1.0);
            ch[c].convertTo(ch[c], CV_8U, k, 0.0);
        }
        cv::merge(ch, bgr);
    }

    static void apply_clahe_lab_l_inplace(cv::Mat& bgr, double clip_limit)
    {
        if (bgr.empty() || bgr.type() != CV_8UC3 || !(clip_limit > 0.05)) {
            return;
        }
        cv::Mat lab;
        cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> ch(3);
        cv::split(lab, ch);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(std::clamp(clip_limit, 0.1, 12.0), cv::Size(8, 8));
        clahe->apply(ch[0], ch[0]);
        cv::merge(ch, lab);
        cv::cvtColor(lab, bgr, cv::COLOR_Lab2BGR);
    }

    // 软件调亮度：降低画面亮度（推荐用HSV的V通道，更自然）
    cv::Mat adjust_brightness(const cv::Mat& img, double brightness_scale)
    {
        cv::Mat hsv_img;
        // 转换到HSV色彩空间（H:色相, S:饱和度, V:亮度）
        cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
        
        // 拆分通道，只调整V通道（亮度）
        std::vector<cv::Mat> hsv_channels;
        cv::split(hsv_img, hsv_channels);
        
        // 亮度缩放：scale<1 变暗，scale=0.5 亮度减半（可自行调整）
        hsv_channels[2] = hsv_channels[2] * brightness_scale;
        
        // 合并通道，转回BGR
        cv::merge(hsv_channels, hsv_img);
        cv::Mat result_img;
        cv::cvtColor(hsv_img, result_img, cv::COLOR_HSV2BGR);
        
        return result_img;
    }




    void image_callback()
    {
        cv::Mat input_img;
        bool read_success = cap_.read(input_img);

        // ============== 关键修改3：加完整调试日志 ==============
        // RCLCPP_INFO(this->get_logger(), "read_success=%d, empty=%d, size=%dx%d", 
        //     read_success, input_img.empty(), input_img.cols, input_img.rows);

        if (!read_success || input_img.empty())
        {
            RCLCPP_WARN(this->get_logger(), "读取空帧");
            return;
        }

        // ---------- VISION[采集端]: 仅做图像预处理，不参与识别/舵机 ----------
        // 方法顺序：可选灰世界白平衡 → LUT 伽马 → Lab 的 L 通道 CLAHE；与 trace 内 vis_* 预处理可叠加调参。
        cv::Mat proc = input_img.clone();
        if (gray_world_wb_enabled_) {
            apply_gray_world_wb_inplace(proc, gray_world_wb_strength_);
        }
        apply_gamma_bgr_inplace(proc, input_gamma_);
        apply_clahe_lab_l_inplace(proc, clahe_l_clip_limit_);

        cv::Mat output_img;
        if (brightness_adjust_enabled_) {
            output_img = adjust_brightness(proc, brightness_scale_);
            RCLCPP_DEBUG(this->get_logger(), "已用软件调亮度，缩放系数：%.2f", brightness_scale_);
        } else {
            output_img = proc;
        }

        // ============== 关键修改4：彻底注释掉GUI！（子线程不能用imshow） ==============
        // cv::imshow("input_img", input_img);
        // cv::waitKey(1);

        // 发布图像
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_link";
        auto img_msg = cv_bridge::CvImage(header, "bgr8", output_img).toImageMsg();
        image_pub_.publish(img_msg);
    }

    cv::VideoCapture cap_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool brightness_adjust_enabled_{false};
    double brightness_scale_{1.0};
    int publish_period_ms_{10};
    double input_gamma_{1.0};
    double clahe_l_clip_limit_{0.0};
    bool gray_world_wb_enabled_{false};
    double gray_world_wb_strength_{0.72};

    int camera_v4l_auto_exposure_{-1};
    bool camera_v4l_set_exposure_{false};
    double camera_v4l_exposure_{-6.0};
    int camera_v4l_auto_wb_{-1};
    int camera_v4l_wb_temperature_{-1};
    double camera_v4l_gain_{-1.0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<OpenCVTestNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("read_image"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}