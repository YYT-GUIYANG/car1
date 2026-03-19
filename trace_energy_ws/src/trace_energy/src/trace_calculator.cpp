// #include "rclcpp/rclcpp.hpp"
// #include "cv_bridge/cv_bridge.h"
// #include "sensor_msgs/msg/image.hpp"
// #include "opencv2/opencv.hpp"
// #include "image_transport/image_transport.hpp"
// #include "std_msgs/msg/header.hpp"
// // 引入自定义舵机消息（和Python端匹配）
// #include "servo_message/msg/servo_message.hpp"

// // 类型别名简化代码
// using ServoMsg = servo_message::msg::ServoMessage;
// using ImageMsg = sensor_msgs::msg::Image;

// class ImageProcessToServoNode : public rclcpp::Node
// {
// public:
//     ImageProcessToServoNode() : Node("image_process_to_servo_node")
//     {
//         // ========== 1. 初始化订阅者（订阅read_image.cpp发布的图像话题） ==========
//         image_sub_ = image_transport::create_subscription(
//             this,
//             "processed_image",
//             std::bind(&ImageProcessToServoNode::image_callback, this, std::placeholders::_1),
//             "raw",
//             rclcpp::QoS(10).get_rmw_qos_profile()
//         );



//         // ========== 2. 初始化舵机指令发布者（给Python的servo_uart_node.py） ==========
//         servo_pub_ = this->create_publisher<ServoMsg>("servo_control", 10);

//         RCLCPP_INFO(this->get_logger(), "图像处理转舵机指令节点已启动！");
//         RCLCPP_INFO(this->get_logger(), "正在订阅话题：processed_image");
//     }

// private:
//     // ========== 接口1：图像处理（你自定义逻辑） ==========
//     // 输入：订阅到的原始图像；输出：处理后的图像（供角度计算使用）
//     cv::Mat process_image(const cv::Mat& raw_img)
//     {
//         // ----------------------------------------------------测试代码：显示原始图像------------------------
        
//         if (raw_img.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "订阅到空图像，跳过本次处理");
//             return raw_img;
//         }
        
//         // ----------------------------------------------------测试代码：显示原始图像------------------------ END




//         // --------------------------
//         // 此处填写你的图像处理逻辑
//         // 示例：灰度化+边缘检测（可删除，替换为你的逻辑）
//         // cv::cvtColor(raw_img, processed_img, cv::COLOR_BGR2GRAY);
//         // cv::Canny(processed_img, processed_img, 50, 150);
//         // --------------------------



//         return raw_img;
//     }

//     // ========== 接口2：舵机角度计算（你自定义逻辑） ==========
//     // 输入：处理后的图像；输出：舵机ID、目标角度（0-25/0-180范围）
//     std::pair<int, int> calculate_servo_angle(const cv::Mat& processed_img)
//     {
//         int servo_id = 1;   // 默认舵机ID（1,10）
//         int servo_angle = 45; // 默认角度（你根据计算修改）


//         // --------------------------
//         // 此处填写你的数学计算/角度推导逻辑
//         // 示例：根据图像特征点计算角度（可删除，替换为你的逻辑）
//         // int target_x = 0; // 假设你检测到的目标横坐标
//         // servo_angle = std::min(std::max((target_x / (float)processed_img.cols) * 180, 0.0f), 180.0f);
//         // --------------------------





//         // 角度/ID范围校验（防止超出舵机量程）
//         servo_angle = std::clamp(servo_angle, 0, 180);
//         servo_id = std::clamp(servo_id, 0, 25);
//         return {servo_id, servo_angle};
//     }

//     // ========== 图像话题回调（核心逻辑：接收→处理→计算→发布） ==========
//     void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
//     {
//         cv_bridge::CvImageConstPtr cv_ptr;
//         try
//         {
//             // 1. 将ROS图像消息转换为OpenCV的Mat格式
//             cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
//             if (cv_ptr->image.empty())
//             {
//                 RCLCPP_WARN(this->get_logger(), "订阅到空图像，跳过本次处理");
//                 return;
//             }
//             RCLCPP_INFO(this->get_logger(), "成功接收图像：尺寸%dx%d", cv_ptr->image.cols, cv_ptr->image.rows);

//             cv::Mat show_img = cv_ptr->image.clone();
//             cv::imshow("Received Image", show_img);
//             // 必须加waitKey，否则窗口无法刷新；按ESC键退出节点
//             char key = (char)cv::waitKey(1);
//             if (key == 27)
//             {
//                 rclcpp::shutdown();
//             }

//             // 2. 执行自定义图像处理
//             cv::Mat processed_img = process_image(cv_ptr->image.clone());

//             // 3. 计算舵机目标角度
//             auto [servo_id, servo_angle] = calculate_servo_angle(processed_img);

//             // 4. 发布舵机控制指令
//             ServoMsg servo_control_msg;
//             servo_control_msg.servo_id = servo_id;
//             servo_control_msg.servo_angle = servo_angle;
//             servo_pub_->publish(servo_control_msg);

//             RCLCPP_INFO(this->get_logger(), 
//                 "发布舵机指令 → ID：%d，角度：%d",
//                 servo_id, servo_angle);
//         }
//         catch (cv_bridge::Exception& e)
//         {
//             // 捕获图像转换异常
//             RCLCPP_ERROR(this->get_logger(), "图像转换失败：%s", e.what());
//         }
//         catch (std::exception& e)
//         {
//             // 捕获其他异常
//             RCLCPP_ERROR(this->get_logger(), "处理图像时出错：%s", e.what());
//         }
//     }

//     // 成员变量
//     image_transport::Subscriber image_sub_;          // 图像话题订阅者
//     rclcpp::Publisher<ServoMsg>::SharedPtr servo_pub_; // 舵机指令发布者
// };

// int main(int argc, char * argv[])
// {
//     // ROS2初始化
//     rclcpp::init(argc, argv);
//     // 创建节点
//     auto node = std::make_shared<ImageProcessToServoNode>();
//     // 保持节点运行（阻塞，等待话题消息）
//     rclcpp::spin(node);
//     // 关闭ROS2
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include "servo_message/msg/servo_message.hpp"
#include <string>

using ServoMsg = servo_message::msg::ServoMessage;
using ImageMsg = sensor_msgs::msg::Image;

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode() : Node("image_subscriber_node")
    {
        // 1. 初始化image_transport订阅器，和发布端完全匹配
        // 话题名 /processed_image 必须和发布节点完全一致
        image_sub_ = image_transport::create_subscription(
            this,
            "processed_image",
            std::bind(&ImageSubscriberNode::image_callback, this, std::placeholders::_1),
            "raw",
            rclcpp::QoS(10).get_rmw_qos_profile()
        );

        servo_pub_ = this->create_publisher<ServoMsg>("servo_control", 10);

        // 初始化帧率统计变量
        last_frame_time_ = this->get_clock()->now();
        frame_count_ = 0;
        current_fps_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "图像订阅节点启动成功，正在订阅话题：/processed_image");
        RCLCPP_INFO(this->get_logger(), "按ESC键可关闭图像窗口，退出节点");
    }

    ~ImageSubscriberNode()
    {
        cv::destroyAllWindows();
        RCLCPP_INFO(this->get_logger(), "图像订阅节点已关闭");
    }

private:
    // 核心回调函数：收到图像消息后执行
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        // 1. 用cv_bridge转换ROS图像消息为OpenCV的cv::Mat
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            // 编码必须和发布端一致：bgr8（OpenCV默认格式）
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge转换失败：%s", e.what());
            return;
        }

        // 2. 校验转换后的图像是否有效
        if (cv_ptr->image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "收到空图像，跳过处理");
            return;
        }

        // 3. 帧率统计
        frame_count_++;
        rclcpp::Time current_time = this->get_clock()->now();
        double time_diff = (current_time - last_frame_time_).seconds();
        if (time_diff >= 1.0) // 每秒更新一次FPS
        {
            current_fps_ = frame_count_ / time_diff;
            frame_count_ = 0;
            last_frame_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "接收帧率：%.1f FPS | 图像分辨率：%dx%d", 
                current_fps_, cv_ptr->image.cols, cv_ptr->image.rows);
        }

        // 4. 图像显示（主线程回调，安全无崩溃）
        cv::Mat show_img = cv_ptr->image.clone();

        process_image(show_img);
    }

    void process_image(const cv::Mat& show_img)
    {
        // ----------------------------------------------------测试代码：显示原始图像------------------------
        
        if (show_img.empty())
        {
            RCLCPP_WARN(this->get_logger(), "订阅到空图像，跳过本次处理");
        }

        cv::putText(show_img, "FPS: " + std::to_string((int)current_fps_), 
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Received Image", show_img);

        // 必须加waitKey，否则窗口无法刷新；按ESC键退出节点
        char key = (char)cv::waitKey(1);
        if (key == 27)
        {
            rclcpp::shutdown();
        }

        
        // ----------------------------------------------------测试代码：显示原始图像------------------------ END




        // --------------------------
        // 此处填写你的图像处理逻辑
        // 示例：灰度化+边缘检测（可删除，替换为你的逻辑）
        // cv::cvtColor(raw_img, processed_img, cv::COLOR_BGR2GRAY);
        // cv::Canny(processed_img, processed_img, 50, 150);
        // --------------------------

    }


    void calculate_servo_angle(const cv::Mat& processed_img)
    {
        std::vector<std::pair<int, int>> servo_controller;

        int servo_id_1 = 1;   // 默认舵机ID（1,10）
        int servo_angle_1 = 45; // 默认角度（你根据计算修改）

        int servo_id_10 = 10;   // 默认舵机ID（1,10）
        int servo_angle_10 = 145; // 默认角度（你根据计算修改）


        // --------------------------
        // 此处填写你的数学计算/角度推导逻辑
        // 示例：根据图像特征点计算角度（可删除，替换为你的逻辑）
        // int target_x = 0; // 假设你检测到的目标横坐标
        // servo_angle = std::min(std::max((target_x / (float)processed_img.cols) * 180, 0.0f), 180.0f);
        // --------------------------





        // 角度/ID范围校验（防止超出舵机量程）
        servo_angle_1 = std::clamp(servo_angle_1, 0, 180);
        servo_controller.push_back({servo_id_1, servo_angle_1});

        servo_angle_10 = std::clamp(servo_angle_10, 0, 180);
        servo_controller.push_back({servo_id_10, servo_angle_10});

        // 发布舵机控制消息
        for(auto& servo : servo_controller)
        {
            ServoMsg msg;
            msg.servo_id = servo.first;
            msg.servo_angle = servo.second;
            servo_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "发布舵机指令 → ID：%d，角度：%d",servo.first, servo.second);
        }
    }


    // 成员变量
    image_transport::Subscriber image_sub_;
    rclcpp::Time last_frame_time_;
    int frame_count_;
    double current_fps_;
    rclcpp::Publisher<ServoMsg>::SharedPtr servo_pub_; 
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriberNode>();
    rclcpp::spin(node); // 阻塞等待回调
    rclcpp::shutdown();
    return 0;
}