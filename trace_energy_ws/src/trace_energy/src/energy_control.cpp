// #include "rclcpp/rclcpp.hpp"
// #include "cv_bridge/cv_bridge.h"
// #include "sensor_msgs/msg/image.hpp"
// #include "opencv2/opencv.hpp"
// #include "image_transport/image_transport.hpp"
// #include "servo_message/msg/servo_message.hpp"
// #include <random>
// #include <string>

// const double PI = 3.14159265358979323846;

// using ServoMsg = servo_message::msg::ServoMessage;
// using ImageMsg = sensor_msgs::msg::Image;

// class EnergyControlNode : public rclcpp::Node
// {
// public:
//     EnergyControlNode() : Node("energy_control_node")
//     {
//         servo_pub_ = this->create_publisher<ServoMsg>("servo_control", 10);

//         std::random_device rd;
//         std::mt19937 gen(rd());
//         std::uniform_int_distribution<int> dist_266(0, 266);
//         std::uniform_int_distribution<int> dist_117(0, 117);

//         a = dist_266(gen) / 1000.0f + 0.78;
//         w = dist_117(gen) / 1000.0f + 1.884;
//         b = 2.09 - a;

//         start_time = std::chrono::high_resolution_clock::now();


//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(50),
//             std::bind(&EnergyControlNode::energy_control, this));
//     }

//     ~EnergyControlNode()
//     {
//         cv::destroyAllWindows();
//         RCLCPP_INFO(this->get_logger(), "能量机关控制节点已关闭");
//     }

//     void set_high_or_low_energy(int energy)
//     {
//         high_or_low_energy = energy;
//     }

// private:
//     void energy_control()
//     {
//         // 控制能量机关
//         std::vector<std::pair<int, int>> servo_controller;
//         int servo_id_8 = 8;
//         int servo_angle_8 = 0;

//         auto current_time = std::chrono::high_resolution_clock::now();

//         // 计算已运行时间（单位：秒）
//         double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.0;

//         if (high_or_low_energy == 0) // 小能量机关
//         {
//             float v = 1 / 3 * PI;
//             int angle = (int)(v * elapsed_time * 180 / PI) % 360;

//             servo_angle_8 = angle;

//             servo_controller.push_back({servo_id_8, servo_angle_8});
//             send_servo_angle(servo_controller);
//             return;
//         }

//         // 大能量机关
//         int angle = (int)((b * elapsed_time - a / w * cos(elapsed_time * w)) * 180 / PI) % 360;

//         // servo_angle_8 = angle;
//         servo_angle_8 = 180;

//         RCLCPP_INFO(this->get_logger(), "当前能量机关：%d", high_or_low_energy);
//         RCLCPP_INFO(this->get_logger(), "当前参数：a=%f, w=%f, b=%f, elapsed_time=%f, angle=%d", a, w, b, elapsed_time, servo_angle_8);

//         servo_controller.push_back({servo_id_8, servo_angle_8});
//         send_servo_angle(servo_controller);
//     }

//     void send_servo_angle(std::vector<std::pair<int, int>> &servo_controller)
//     {
//         // 发布舵机控制消息
//         for (auto &servo : servo_controller)
//         {
//             servo.second = std::clamp(servo.second, 0, 180);
//             ServoMsg msg;
//             msg.servo_id = servo.first;
//             msg.servo_angle = servo.second;
//             servo_pub_->publish(msg);
//             RCLCPP_INFO(this->get_logger(), "发布舵机指令 → ID：%d，角度：%d", servo.first, servo.second);
//         }
//     }

//     // 成员变量
//     float a = 0;
//     float w = 0;
//     float b = 0;
//     int closewise_or_counterclockwise = 0;
//     int high_or_low_energy = 1;
//     std::chrono::_V2::system_clock::time_point start_time = std::chrono::high_resolution_clock::now();
//     rclcpp::Publisher<ServoMsg>::SharedPtr servo_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<EnergyControlNode>();

//     int energy = 1;

//     node->set_high_or_low_energy(energy);
//     rclcpp::spin(node); // 阻塞等待回调
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "servo_message/msg/servo_message.hpp"
#include <random>
#include <cmath>
#include <algorithm>

const double PI = 3.14159265358979323846;
using ServoMsg = servo_message::msg::ServoMessage;

// 舵机硬件参数配置（无需修改，适配你的MG90S）
const int STOP_CMD = 90;                // 停止位指令
const int MIN_CMD = 0;                    // 最小指令值
const int MAX_CMD = 180;                  // 最大指令值
const double MAX_SPEED = 600.0;           // 舵机最大转速(°/s)，对应0.12s/60°，在你的额定范围内
const double SPEED2CMD_COEFF = 90.0 / MAX_SPEED; // 转速→指令的映射系数

class EnergyControlNode : public rclcpp::Node
{
public:
    EnergyControlNode() : Node("energy_control_node")
    {
        // 发布者初始化（绝对话题，与 trace_calculator / 串口节点对齐）
        servo_pub_ = this->create_publisher<ServoMsg>("/servo_control", 10);
        
        // 启动时先发送停止指令，避免上电狂转
        send_servo_cmd(8, STOP_CMD);
        RCLCPP_INFO(this->get_logger(), "能量机关控制节点已启动，舵机默认停止");

        // 大能量机关随机参数初始化，完全保留原代码逻辑
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dist_266(0, 266);
        std::uniform_int_distribution<int> dist_117(0, 117);
        a = dist_266(gen) / 1000.0f + 0.78;   // 范围：0.78~1.046
        w = dist_117(gen) / 1000.0f + 1.884;  // 范围：1.884~2.001
        b = 2.09 - a;                           // 范围：1.044~1.31

        // 记录启动时间
        start_time = std::chrono::high_resolution_clock::now();
        
        // 50ms定时器，匹配原代码更新频率
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&EnergyControlNode::energy_control, this));
    }

    ~EnergyControlNode()
    {
        // 节点关闭时强制停止舵机
        send_servo_cmd(8, STOP_CMD);
        RCLCPP_INFO(this->get_logger(), "能量机关控制节点已关闭，舵机已停止");
    }

    // 大小能量机关切换接口，完全兼容原代码
    void set_high_or_low_energy(int energy)
    {
        high_or_low_energy = energy;
        RCLCPP_INFO(this->get_logger(), "切换能量机关模式：%s", 
            energy == 0 ? "小能量机关" : "大能量机关");
    }

private:
    void energy_control()
    {
        int servo_id = 8;
        int target_cmd = STOP_CMD; // 默认停止

        // 计算已运行时间（单位：秒）
        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();

        if (high_or_low_energy == 0) // 小能量机关：恒定60°/s转速
        {
            const double fixed_omega_rad = PI / 3.0; // 1/3π rad/s
            double fixed_omega_deg = fixed_omega_rad * 180.0 / PI; // 转换为60°/s
            // 映射为舵机指令
            if (clockwise_or_counterclockwise == 1) // 顺时针
            {
                // target_cmd = static_cast<int>(STOP_CMD + fixed_omega_deg * SPEED2CMD_COEFF);
                target_cmd = 91;
            }
            else // 逆时针
            {
                // target_cmd = static_cast<int>(STOP_CMD - fixed_omega_deg * SPEED2CMD_COEFF);
                target_cmd = 89;
            }
            RCLCPP_INFO(this->get_logger(), "小能量机关 | 运行时间：%.2fs | 目标转速：%.1f°/s | 指令值：%d | 方向：%s",
                elapsed_time, fixed_omega_deg, target_cmd, clockwise_or_counterclockwise == 1 ? "顺时针" : "逆时针");
        }
        else // 大能量机关：正弦变化角速度 v=asin(wt)+b
        {
            // 计算瞬时角速度（rad/s）
            double omega_rad = a * sin(w * elapsed_time) + b;
            // 转换为°/s
            double omega_deg = omega_rad * 180.0 / PI;
            // 映射为舵机指令
            if (clockwise_or_counterclockwise == 1) // 顺时针
            {
                target_cmd = static_cast<int>(STOP_CMD + omega_deg * SPEED2CMD_COEFF);
            }
            else // 逆时针
            {
                target_cmd = static_cast<int>(STOP_CMD - omega_deg * SPEED2CMD_COEFF);
            }
            RCLCPP_INFO(this->get_logger(), "大能量机关 | 运行时间：%.2fs | 角速度：%.2frad/s | 转速：%.1f°/s | 指令值：%d | 方向：%s",
                elapsed_time, omega_rad, omega_deg, target_cmd, clockwise_or_counterclockwise == 1 ? "顺时针" : "逆时针");
        }

        // 指令限幅保护，避免超范围损坏舵机
        target_cmd = std::clamp(target_cmd, MIN_CMD, MAX_CMD);
        // 发送舵机控制指令
        send_servo_cmd(servo_id, target_cmd);
    }

    // 舵机指令发送函数，兼容原有串口节点
    void send_servo_cmd(int servo_id, int cmd_value)
    {
        ServoMsg msg;
        msg.servo_id = servo_id;
        msg.servo_angle = cmd_value; // 用原有字段传递指令值，无需修改自定义消息
        servo_pub_->publish(msg);
    }

    // 成员变量，完全保留原代码定义
    float a = 0;
    float w = 0;
    float b = 0;
    int high_or_low_energy = 0; // 默认小能量机关
    int clockwise_or_counterclockwise = 1; // 默认顺时针
    std::chrono::high_resolution_clock::time_point start_time;
    rclcpp::Publisher<ServoMsg>::SharedPtr servo_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EnergyControlNode>();

    
    
    // // 启动时默认大能量机关，可通过传参修改为小能量机关
    // int energy_mode = 1;
    // if (argc >= 2)
    // {
    //     energy_mode = atoi(argv[1]);
    // }
    // node->set_high_or_low_energy(energy_mode);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}