#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/header.hpp"

class OpenCVTestNode : public rclcpp::Node
{
public:
    OpenCVTestNode() : Node("opencv_test_node")
    {
        // ============== 关键修改1：换后端+设备号（优先0，Jetson USB默认是0） ==============
        cap_ = cv::VideoCapture(1, cv::CAP_ANY); // 用CAP_ANY自动适配，禁用V4L2

        // ============== 关键修改2：先不强制分辨率，避免摄像头不支持 ==============
        // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        // cap_.set(cv::CAP_PROP_FPS, 30);

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "摄像头打开失败！设备号0/1切换试试");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "摄像头打开成功");

        // 预读3帧丢弃（解决摄像头预热无效帧）
        cv::Mat dummy;
        for(int i=0; i<3; i++) cap_.read(dummy);

        image_pub_ = image_transport::create_publisher(this, "processed_image", rclcpp::QoS(10).get_rmw_qos_profile());
        
        // 10Hz定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OpenCVTestNode::image_callback, this)
        );
    }

    ~OpenCVTestNode()
    {
        if (cap_.isOpened()) cap_.release();
    }

private:
    void image_callback()
    {
        cv::Mat input_img;
        bool read_success = cap_.read(input_img);

        // ============== 关键修改3：加完整调试日志 ==============
        RCLCPP_INFO(this->get_logger(), "read_success=%d, empty=%d, size=%dx%d", 
            read_success, input_img.empty(), input_img.cols, input_img.rows);

        if (!read_success || input_img.empty())
        {
            RCLCPP_WARN(this->get_logger(), "读取空帧");
            return;
        }

        // ============== 关键修改4：彻底注释掉GUI！（子线程不能用imshow） ==============
        // cv::imshow("input_img", input_img);
        // cv::waitKey(1);

        // 发布图像
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_link";
        auto img_msg = cv_bridge::CvImage(header, "bgr8", input_img).toImageMsg();
        image_pub_.publish(img_msg);
    }

    cv::VideoCapture cap_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpenCVTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}