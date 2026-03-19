#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include <string>

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
        // 在图像上叠加FPS信息
        cv::putText(show_img, "FPS: " + std::to_string((int)current_fps_), 
            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Received Image", show_img);

        // 必须加waitKey，否则窗口无法刷新；按ESC键退出节点
        char key = (char)cv::waitKey(1);
        if (key == 27)
        {
            rclcpp::shutdown();
        }

        // ======================
        // 这里可以添加你的图像处理代码
        // 例如：目标检测、图像滤波、颜色识别等
        // 处理后的图像存在cv_ptr->image里
        // ======================
    }

    // 成员变量
    image_transport::Subscriber image_sub_;
    rclcpp::Time last_frame_time_;
    int frame_count_;
    double current_fps_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriberNode>();
    rclcpp::spin(node); // 阻塞等待回调
    rclcpp::shutdown();
    return 0;
}