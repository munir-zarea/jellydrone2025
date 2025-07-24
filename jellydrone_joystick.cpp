#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <algorithm>
#include <cmath>

class JoystickServoControl : public rclcpp::Node
{
public:
  JoystickServoControl()
  : Node("joystick_servo_control"),
    servo5_last_(DEFAULT_POS)
  {
    for (int i = 0; i < 4; ++i)
      last_set_[i] = DEFAULT_POS;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoystickServoControl::joyCallback, this, std::placeholders::_1));

    servo_pub_ = create_publisher<std_msgs::msg::String>("servo_commands", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&JoystickServoControl::publishCommand, this));

    RCLCPP_INFO(get_logger(), "Joystick servo control running (0–270° range)");
  }

private:
  void joyCallback(sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_ = std::move(msg);
  }

  void publishCommand()
  {
    if (!last_joy_ || last_joy_->axes.size() < 2) return;

    float x = last_joy_->axes[0];  // left stick X
    float y = -last_joy_->axes[1]; // left stick Y

    float x_gain = 40.0f;  // expand full stick to ±135° from center
    float y_gain = 40.0f;

    float cross_boost = -0.5f;     // Boost to perpendicular pouches
    float deflate_gain = 1.3f;    // Amplified deflation on opposite pouch

    int s1 = DEFAULT_POS + x * x_gain                                  // X inflation
           + std::abs(y) * y_gain * cross_boost;                   // Y-axis boost

    int s3 = DEFAULT_POS - x * x_gain * deflate_gain                   // X opposite deflation (amplified)
           + std::abs(y) * y_gain * cross_boost;                   // Y-axis boost

    int s2 = DEFAULT_POS + y * y_gain                                  // Y inflation
           + std::abs(x) * x_gain * cross_boost;                   // X-axis boost

    int s4 = DEFAULT_POS - y * y_gain * deflate_gain                   // Y opposite deflation (amplified)
           + std::abs(x) * x_gain * cross_boost;                   // X-axis boost


    last_set_[0] = std::clamp(s1, 0, 270);
    last_set_[1] = std::clamp(s2, 0, 270);
    last_set_[2] = std::clamp(s3, 0, 270);
    last_set_[3] = std::clamp(s4, 0, 270);

    // RT controls servo5: fast down, slow reset
    float rt = (last_joy_->axes.size() > 5 ? last_joy_->axes[5] : 1.0f);
    if (rt < RT_THRESHOLD)
      servo5_last_ = std::max(servo5_last_ - FAST_STEP, SERVO5_MIN);
    else if (servo5_last_ < DEFAULT_POS)
      servo5_last_ = std::min(servo5_last_ + STEP_SIZE, DEFAULT_POS);

    std::ostringstream oss;
    oss << "SET:"
        << last_set_[0] << ',' << last_set_[1] << ','
        << last_set_[2] << ',' << last_set_[3] << ','
        << servo5_last_;

    auto msg = std_msgs::msg::String();
    msg.data = oss.str();
    servo_pub_->publish(msg);
  }

  // Constants
  static constexpr int   STEP_SIZE     = 1;
  static constexpr int   FAST_STEP     = 10;
  static constexpr float RT_THRESHOLD  = 0.9f;
  static constexpr int   DEFAULT_POS   = 40;  // midpoint of 0–270
  static constexpr int   SERVO5_MIN    = 0;

  int last_set_[4];
  int servo5_last_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr servo_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickServoControl>());
  rclcpp::shutdown();
  return 0;
}
