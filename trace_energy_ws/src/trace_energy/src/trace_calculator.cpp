#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include "servo_message/msg/servo_message.hpp"
#include <string>

const double PI = 3.14159265358979323846;

using ServoMsg = servo_message::msg::ServoMessage;
using ImageMsg = sensor_msgs::msg::Image;

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode() : Node("image_subscriber_node")
    {
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

        std::vector<std::pair<int, int>> servo_controller = process_image(show_img);
        send_servo_angle(servo_controller);
    }

    std::vector<std::pair<int, int>> process_image(const cv::Mat& show_img)
    {
        std::vector<std::pair<int, int>> servo_controller;

        int servo_id_1 = 1;   // 舵机ID1：云台上面的舵机，控制舵机俯仰
        int servo_angle_1 = 0; // 默认角度（后续需要根据计算得出的角度修改）

        int servo_id_10 = 10;   // 舵机ID10：云台下面的舵机，控制舵机偏航（水平面上旋转）
        int servo_angle_10 = 0; // 默认角度（后续需要根据计算得出的角度修改）





















        
        // TODO: 从这里开始就是你需要填写的代码，用于处理图像并计算舵机运动角度-------------------------------------------------------------

        // 只保留摄像头中心（960*540）（400x400）区域的图像
        cv::Mat img_crop = show_img(cv::Rect(760, 340, 400, 400));
        // ----------------------------------------------------测试代码：显示原始图像------------------------
        // if (show_img.empty())
        // {
        //     RCLCPP_WARN(this->get_logger(), "订阅到空图像，跳过本次处理");
        // }

        // cv::putText(show_img, "FPS: " + std::to_string((int)current_fps_), 
        // cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        // cv::imshow("Received Image", show_img);

        // // 必须加waitKey，否则窗口无法刷新；按ESC键退出节点
        // char key = (char)cv::waitKey(1);
        // if (key == 27)
        // {
        //     rclcpp::shutdown();
        // }
        // ----------------------------------------------------测试代码：显示原始图像------------------------ END

        cv::cvtColor(img_crop, img_crop, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(img_crop, img_crop, cv::Size(7,7), 0);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
        cv::dilate(img_crop, img_crop, kernel);
        cv::erode(img_crop, img_crop, kernel);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img_crop, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> conPoly(contours.size());   
        std::vector<cv::Rect> rect(contours.size());

        for(int i=0; i<contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if(area < 100) continue;
            double peri = cv::arcLength(contours[i], true);
            cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
            rect[i] = cv::boundingRect(conPoly[i]);
            if(fabs(pow(peri,2)/area - 4 * PI) < 10)
            {
                RCLCPP_INFO(this->get_logger(), "找到一个圆形轮廓，%.2f, 面积：%.2f, 周长：%.2f,圆心坐标点位置(%d,%d)", fabs(pow(peri,2)/area - 4 * PI), area, peri, rect[i].x, rect[i].y);

            }
        }




        cv::imshow("Processed Image", img_crop);

        // 必须加waitKey，否则窗口无法刷新；按ESC键退出节点
        char key = (char)cv::waitKey(1);
        if (key == 27)
        {
            rclcpp::shutdown();
        }


        // 将计算得出的角度赋值给舵机角度变量
        servo_angle_1 = 0;  // 云台上面的舵机，控制舵机俯仰角度
        servo_angle_10 = 0; // 云台下面的舵机，控制舵机偏航（水平面上旋转）角度

        // TODO: END-------------------------------------------------------------





























        servo_controller.push_back({servo_id_1, servo_angle_1});
        servo_controller.push_back({servo_id_10, servo_angle_10});

        return servo_controller;
    }


    void send_servo_angle(std::vector<std::pair<int, int>>& servo_controller)
    {
        // 发布舵机控制消息
        for(auto& servo : servo_controller)
        {
            if(!(servo.first == 1 || servo.first == 10))
            {
                RCLCPP_ERROR(this->get_logger(), "无效的舵机ID：%d,有效ID为1和10", servo.first);
                continue;
            }

            servo.second = std::clamp(servo.second, 0, 180);
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