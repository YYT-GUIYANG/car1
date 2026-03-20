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

        int servo_id_1 = 1;   // 舵机ID1：俯仰
        int servo_angle_1 = 0; 

        int servo_id_10 = 10;   // 舵机ID10：偏航
        int servo_angle_10 = 0; 

        // TODO: 从这里开始就是你需要填写的代码-------------------------------------------------------------
        
        static std::vector<double> time_hist;
        static std::vector<double> angle_hist;
        static double current_pitch = 90.0; 
        static double current_yaw = 90.0;   
        static int rune_mode = 0;           // 0: 未知, 1: 小能量, 2: 大能量
        
        // 1. 区域限制：原本裁剪是 (760, 340, 400, 400)
        // 我们在 400x400 内部再做一个“安全区”，忽略最边缘的 50 像素，防止圆筒边框干扰
        int safe_padding = 50;
        cv::Rect roi_rect(760 + safe_padding, 340 + safe_padding, 400 - 2*safe_padding, 400 - 2*safe_padding);
        cv::Mat img_crop = show_img(roi_rect);
        
        cv::Mat gray_crop, bin_crop;
        cv::cvtColor(img_crop, gray_crop, cv::COLOR_BGR2GRAY);
        // 使用自适应阈值或较高的固定阈值来滤除暗部干扰（圆筒阴影）
        cv::threshold(gray_crop, bin_crop, 120, 255, cv::THRESH_BINARY); 
        cv::GaussianBlur(bin_crop, bin_crop, cv::Size(5,5), 0);
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bin_crop, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Point2f rune_center(960, 540); 
        cv::Vec3b target_color(0, 0, 0);   
        bool found_circle = false;

        // 2. 寻找真正的中心 R 标
        for(size_t i=0; i<contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            // 真正的中心圆在 300x300 区域内面积通常在 500~5000 之间
            if(area < 200 || area > 8000) continue; 

            double peri = cv::arcLength(contours[i], true);
            double circularity = pow(peri, 2) / (4 * PI * area); // 完美圆 = 1.0

            // 如果形状接近圆形 (1.0 ~ 2.5 之间)
            if(circularity < 2.5)
            {
                cv::Rect r = cv::boundingRect(contours[i]);
                // 计算在原始图像 1920x1080 中的位置
                rune_center.x = r.x + r.width / 2.0 + roi_rect.x;
                rune_center.y = r.y + r.height / 2.0 + roi_rect.y;

                // 提取颜色
                target_color = show_img.at<cv::Vec3b>((int)rune_center.y, (int)rune_center.x);
                
                // 打印调试信息到终端
                RCLCPP_INFO(this->get_logger(), "检测到中心圆! 坐标:(%.1f, %.1f), 面积:%.1f, 颜色: B=%d G=%d R=%d", 
                            rune_center.x, rune_center.y, area, target_color[0], target_color[1], target_color[2]);
                
                found_circle = true;
                break;
            }
        }

        cv::Point2f target_square(960, 540);
        bool found_target = false;
        double current_angle = 0.0;
        double rune_radius = 0.0;

        // 3. 寻找与中心圆颜色匹配的击打扇区
        if (found_circle) {
            cv::Mat gray_full, bin_full;
            cv::cvtColor(show_img, gray_full, cv::COLOR_BGR2GRAY);
            cv::threshold(gray_full, bin_full, 100, 255, cv::THRESH_BINARY);
            
            std::vector<std::vector<cv::Point>> full_contours;
            cv::findContours(bin_full, full_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& cnt : full_contours) {
                double area = cv::contourArea(cnt);
                if (area < 400 || area > 10000) continue; 
                
                cv::Rect bRect = cv::boundingRect(cnt);
                cv::Point2f sq_center(bRect.x + bRect.width/2.0, bRect.y + bRect.height/2.0);
                
                // 排除中心圆自身
                if (cv::norm(sq_center - rune_center) < 30) continue;

                cv::Vec3b sq_color = show_img.at<cv::Vec3b>((int)sq_center.y, (int)sq_center.x);
                
                // 颜色匹配逻辑（判断是否为同阵营颜色）
                double color_dist = pow(sq_color[0]-target_color[0], 2) + 
                                    pow(sq_color[1]-target_color[1], 2) + 
                                    pow(sq_color[2]-target_color[2], 2);
                
                if (color_dist < 4000) { 
                    target_square = sq_center;
                    found_target = true;
                    rune_radius = cv::norm(target_square - rune_center);
                    current_angle = atan2(target_square.y - rune_center.y, target_square.x - rune_center.x);
                    break;
                }
            }
        }

        // 4. 预测与控制逻辑（保留 MATLAB 融合部分的 OLS 拟合）
        double current_time = this->get_clock()->now().seconds();
        double pred_angle = current_angle;
        double predict_dt = 0.5; 

        if (found_target && found_circle) {
            if (!angle_hist.empty()) {
                while (current_angle - angle_hist.back() > PI) current_angle -= 2 * PI;
                while (current_angle - angle_hist.back() < -PI) current_angle += 2 * PI;
            }
            time_hist.push_back(current_time);
            angle_hist.push_back(current_angle);
            if (time_hist.size() > 200) {
                time_hist.erase(time_hist.begin());
                angle_hist.erase(angle_hist.begin());
            }

            if (time_hist.size() >= 20) {
                double dt_total = time_hist.back() - time_hist.front();
                double d_theta = angle_hist.back() - angle_hist.front();
                double speed = std::abs(d_theta / dt_total);

                if (std::abs(speed - PI/3.0) < 0.25) rune_mode = 1; 
                else rune_mode = 2;

                if (rune_mode == 1) {
                    double dir = (d_theta > 0) ? 1.0 : -1.0;
                    pred_angle = current_angle + dir * (PI/3.0) * predict_dt;
                } 
                else if (rune_mode == 2 && time_hist.size() >= 40) {
                    // MATLAB 风格的 OLS 拟合
                    int n = time_hist.size();
                    cv::Mat A(n, 4, CV_64F); cv::Mat Y(n, 1, CV_64F);
                    double t0 = time_hist.front();
                    for (int i=0; i<n; i++) {
                        double t = time_hist[i] - t0;
                        A.at<double>(i,0)=sin(1.942*t); A.at<double>(i,1)=cos(1.942*t); A.at<double>(i,2)=t; A.at<double>(i,3)=1.0;
                        Y.at<double>(i,0)=angle_hist[i];
                    }
                    cv::Mat X_coef;
                    cv::solve(A, Y, X_coef, cv::DECOMP_SVD);
                    double t_p = (current_time + predict_dt) - t0;
                    pred_angle = X_coef.at<double>(0,0)*sin(1.942*t_p) + X_coef.at<double>(1,0)*cos(1.942*t_p) + 
                                 X_coef.at<double>(2,0)*t_p + X_coef.at<double>(3,0);
                }
            }
        }

        // 5. 最终角度计算
        cv::Point2f predict_p(960, 540);
        if (found_target) {
            predict_p.x = rune_center.x + rune_radius * cos(pred_angle);
            predict_p.y = rune_center.y + rune_radius * sin(pred_angle);
            // 在图像上画出预测点辅助调试
            cv::circle(show_img, predict_p, 10, cv::Scalar(0, 255, 255), -1);
            cv::line(show_img, rune_center, predict_p, cv::Scalar(255, 0, 0), 2);
        }

        double kp = 0.015; // 稍微调低一点P增益，更平滑
        current_yaw -= kp * (predict_p.x - 960.0);
        current_pitch -= kp * (predict_p.y - 540.0);

        servo_angle_1 = std::clamp((int)current_pitch, 0, 180);
        servo_angle_10 = std::clamp((int)current_yaw, 0, 180);

        cv::imshow("Debug View", bin_crop); // 显示二值化后的安全区图像
        cv::waitKey(1);

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