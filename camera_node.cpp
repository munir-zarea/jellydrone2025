#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
    : Node("camera_node")
    {
        this->declare_parameter("camera_index", 0);
        int camera_index = this->get_parameter("camera_index").as_int();

        cap_.open(camera_index);
        if (!cap_.isOpened()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open camera at index %d", camera_index);
            rclcpp::shutdown();
            return;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 fps
            std::bind(&CameraNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Camera node started at index %d", camera_index);
    }

private:
    void timerCallback()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        publisher_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
