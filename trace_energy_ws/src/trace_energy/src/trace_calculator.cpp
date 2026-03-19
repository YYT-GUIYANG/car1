#include "rclcpp/rclcpp.hpp"
#include "servo_message/msg/servo_message.hpp"
#include <cmath>
#include <random>
#include <chrono>
#include <vector>
#include <string>

// 常量定义（与Python版本对齐）
const double WIDTH = 800.0, HEIGHT = 800.0;
const std::pair<double, double> CENTER = {WIDTH/2, HEIGHT/2};
const double R_PIXELS = 250.0;
const double F_PIXELS = 1000.0;  // 虚拟焦距(像素)
const double DEPTH_MM = 5000.0;  // 方块距离相机5000mm
const double PREDICT_DELAY = 0.5; // 预判时间(s)
const int YAW_SERVO_ID = 1;       // 偏航舵机编号（可根据实际修改）
const int PITCH_SERVO_ID = 2;     // 俯仰舵机编号（可根据实际修改）

// 能量机关类（移植Python的EnergyBuff）
class EnergyBuff {
public:
    EnergyBuff(std::string mode = "large") : mode_(mode) {
        // 初始化随机数生成器
        std::random_device rd;
        gen_ = std::mt19937(rd());
        
        // 随机初始化运动参数
        a_dist_ = std::uniform_real_distribution<double>(0.780, 1.045);
        w_dist_ = std::uniform_real_distribution<double>(1.884, 2.000);
        phi_dist_ = std::uniform_real_distribution<double>(0, 2 * M_PI);
        
        a_ = a_dist_(gen_);
        w_ = w_dist_(gen_);
        phi_ = phi_dist_(gen_);
        b_ = 2.090 - a_;
        
        // 随机旋转方向（1/-1）
        dir_dist_ = std::uniform_int_distribution<int>(0, 1);
        direction_ = dir_dist_(gen_) == 0 ? 1 : -1;
        
        base_angle_ = 0.0;
        
        // 颜色相关（仅保留目标颜色逻辑，绘图部分移除）
        target_color_idx_ = std::uniform_int_distribution<int>(0, 4)(gen_);
    }

    // 切换能量机关模式（大/小）
    void toggle_mode() {
        mode_ = (mode_ == "large") ? "small" : "large";
    }

    // 计算角度增量（预测核心）
    double get_delta_theta(double t, double delay) {
        if (mode_ == "small") {
            return (M_PI / 3.0) * delay;
        } else {
            // 原函数：∫(a*sin(wτ+phi) + b)dτ = -a/w * cos(wτ+phi) + b*τ
            auto integral = [this](double tau) {
                return (-a_ / w_) * cos(w_ * tau + phi_) + b_ * tau;
            };
            return integral(t + delay) - integral(t);
        }
    }

    // 更新物理位置
    void update(double t, double dt) {
        double speed = 0.0;
        if (mode_ == "small") {
            speed = M_PI / 3.0;
        } else {
            speed = a_ * sin(w_ * t + phi_) + b_;
        }
        base_angle_ += direction_ * speed * dt;
        // 角度归一化（避免数值过大）
        base_angle_ = fmod(base_angle_, 2 * M_PI);
    }

    // 获取当前基础角度
    double get_base_angle() const { return base_angle_; }
    // 获取旋转方向
    int get_direction() const { return direction_; }
    // 获取模式
    std::string get_mode() const { return mode_; }

private:
    std::string mode_;
    double a_, w_, phi_, b_;
    int direction_;
    double base_angle_;
    int target_color_idx_;  // 目标颜色索引（仅占位，无绘图逻辑）

    // 随机数相关
    std::mt19937 gen_;
    std::uniform_real_distribution<double> a_dist_;
    std::uniform_real_distribution<double> w_dist_;
    std::uniform_real_distribution<double> phi_dist_;
    std::uniform_int_distribution<int> dir_dist_;
};

// ROS2节点类
class PredictAngleNode : public rclcpp::Node {
public:
    PredictAngleNode() : Node("predict_angle_node") {
        // 创建舵机指令发布者（话题名与Python节点一致）
        servo_pub_ = this->create_publisher<servo_message::msg::ServoMessage>(
            "servo_control", 10);
        
        // 初始化能量机关
        buff_ = std::make_unique<EnergyBuff>("large");
        start_time_ = this->get_clock()->now();

        // 10Hz定时器（与摄像头节点频率对齐）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PredictAngleNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "预测角度节点已启动！");
    }

private:
    void timer_callback() {
        // 1. 计算当前时间（相对于节点启动）
        auto now = this->get_clock()->now();
        double current_t = (now - start_time_).seconds();
        double dt = 0.1;  // 定时器周期100ms

        // 2. 更新能量机关状态
        buff_->update(current_t, dt);
        double delta_theta = buff_->get_delta_theta(current_t, PREDICT_DELAY);

        // 3. 计算目标扇区的预测位置
        double target_sector_angle = buff_->get_base_angle();  // 目标扇区基础角度
        double pred_angle = target_sector_angle + buff_->get_direction() * delta_theta;

        // 4. 解算云台舵机角度（像素→毫米→弧度→角度）
        double px = CENTER.first + R_PIXELS * cos(pred_angle);
        double py = CENTER.second + R_PIXELS * sin(pred_angle);
        double dx_pixels = px - CENTER.first;
        double dy_pixels = py - CENTER.second;

        double target_x_mm = dx_pixels * (DEPTH_MM / F_PIXELS);
        double target_y_mm = dy_pixels * (DEPTH_MM / F_PIXELS);

        // 计算yaw/pitch弧度（负号适配舵机转向）
        double yaw_rad = -atan2(target_x_mm, DEPTH_MM);
        double pitch_rad = -atan2(target_y_mm, DEPTH_MM);

        // 转换为角度（舵机范围0-180，映射：-90~90 → 0~180）
        double yaw_deg = rad2deg(yaw_rad) + 90.0;
        double pitch_deg = rad2deg(pitch_rad) + 90.0;

        // 5. 校验舵机角度范围（0-180）
        yaw_deg = std::clamp(yaw_deg, 0.0, 180.0);
        pitch_deg = std::clamp(pitch_deg, 0.0, 180.0);

        // 6. 发布舵机控制指令
        publish_servo_cmd(YAW_SERVO_ID, static_cast<int>(round(yaw_deg)));
        publish_servo_cmd(PITCH_SERVO_ID, static_cast<int>(round(pitch_deg)));

        // 打印调试信息
        RCLCPP_INFO(this->get_logger(), 
            "模式：%s | Yaw舵机角度：%d° | Pitch舵机角度：%d°",
            buff_->get_mode().c_str(),
            static_cast<int>(round(yaw_deg)),
            static_cast<int>(round(pitch_deg)));
    }

    // 发布单个舵机指令
    void publish_servo_cmd(int servo_id, int angle) {
        auto msg = servo_message::msg::ServoMessage();
        msg.servo_id = servo_id;
        msg.servo_angle = angle;
        servo_pub_->publish(msg);
    }

    // 弧度转角度
    double rad2deg(double rad) {
        return rad * 180.0 / M_PI;
    }

    // 成员变量
    rclcpp::Publisher<servo_message::msg::ServoMessage>::SharedPtr servo_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<EnergyBuff> buff_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PredictAngleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}