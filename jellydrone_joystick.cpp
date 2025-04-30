#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>

class JoystickServoControl : public rclcpp::Node
{
public:
    JoystickServoControl() : Node("joystick_servo_control")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickServoControl::joyCallback, this, std::placeholders::_1));

        servo_pub_ = this->create_publisher<std_msgs::msg::String>("servo_commands", 10);
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        int s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0;

        // Left stick horizontal
        if (joy->axes[0] > 0.5) { s1 = 1; s3 = -1; }
        else if (joy->axes[0] < -0.5) { s1 = -1; s3 = 1; }

        // Left stick vertical
        if (joy->axes[1] > 0.5) { s2 = 1; s4 = -1; }
        else if (joy->axes[1] < -0.5) { s2 = -1; s4 = 1; }

        // RB button
        if (joy->buttons[5] == 1) { s5 = 1; }

        std::ostringstream oss;
        oss << s1 << "," << s2 << "," << s3 << "," << s4 << "," << s5;

        std_msgs::msg::String msg;
        msg.data = oss.str();

        servo_pub_->publish(msg);

        RCLCPP_DEBUG(this->get_logger(), "Published servo command: %s", msg.data.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr servo_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickServoControl>());
    rclcpp::shutdown();
    return 0;
}
