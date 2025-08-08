#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class JoystickServoControl : public rclcpp::Node
{
public:
  JoystickServoControl()
  : Node("joystick_servo_control"),
    base_pos_(DEFAULT_POS), // Start the "goal post" at 90 degrees
    servo5_last_(DEFAULT_POS)
  {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&JoystickServoControl::joyCallback, this, std::placeholders::_1));

    servo_pub_ = create_publisher<std_msgs::msg::String>("servo_commands", 10);

    timer_ = create_wall_timer(
      20ms,
      std::bind(&JoystickServoControl::publishCommand, this));

    RCLCPP_INFO(get_logger(), "Joystick servo control running");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_ = msg;
  }

  void publishCommand()
  {
    if (!last_joy_ || last_joy_->axes.size() < 6) return;

    // Adjust the base_pos_ when LB/RB held
    bool lb = last_joy_->buttons.size() > 4 && last_joy_->buttons[4];
    bool rb = last_joy_->buttons.size() > 5 && last_joy_->buttons[5];
    if (lb) {
      base_pos_ = std::min(base_pos_ + BASE_STEP, SERVO_MAX);
    } else if (rb) {
      base_pos_ = std::max(base_pos_ - BASE_STEP, SERVO_MIN);
    }
    
    // If neither rb or lb is held, base_pos_ stays where it was.

    // READ AND DEADZONE STICKS
    float x = last_joy_->axes[0];      // ←: –1 … →: +1
    float y = -last_joy_->axes[1];     // ↑: +1 … ↓: –1

    constexpr float deadzone = 0.1f;
    if (std::abs(x) < deadzone) x = 0.0f;
    if (std::abs(y) < deadzone) y = 0.0f;

    // INITIALIZE ALL POUCHES TO MIDPOINT AND CARRY OVER SERVO 5
    int s[5] = {
      base_pos_,   // pouch 1 (servo1)
      base_pos_,   // pouch 2 (servo2)
      base_pos_,   // pouch 3 (servo3)
      base_pos_,   // pouch 4 (servo4)
      servo5_last_   // propulsion (servo5)
    };

    // APPLY STICK OFFSETS AROUND NEW CENTER
    s[0] = std::clamp(int(base_pos_ + x * GAIN), SERVO_MIN, SERVO_MAX); // servo 1
    s[2] = std::clamp(int(base_pos_ - x * GAIN), SERVO_MIN, SERVO_MAX); // servo 3
    s[1] = std::clamp(int(base_pos_ + y * GAIN), SERVO_MIN, SERVO_MAX); // servo 2
    s[3] = std::clamp(int(base_pos_ - y * GAIN), SERVO_MIN, SERVO_MAX); // servo 4

    // RT LOGIC FOR SERVO 5 (PROPULSION)
    float rt = last_joy_->axes[5];
    if (rt < RT_THRESHOLD) {
      servo5_last_ = std::max(servo5_last_ - FAST_STEP, SERVO5_MIN);
    } else {
      servo5_last_ = std::min(servo5_last_ + STEP_SIZE, DEFAULT_POS);
    }
    s[4] = servo5_last_;


    // PUBLISH
    std_msgs::msg::String out;
    out.data = "SET:" +
      std::to_string(s[0]) + ',' +
      std::to_string(s[1]) + ',' +
      std::to_string(s[2]) + ',' +
      std::to_string(s[3]) + ',' +
      std::to_string(s[4]);
    servo_pub_->publish(out);
  }

  // CONSTANTS
  static constexpr int   DEFAULT_POS = 135; // Neutral angle
  static constexpr int   SERVO_MIN   = 0; // Minimum angle
  static constexpr int   SERVO_MAX   = 270; // Maximum angle
  static constexpr float GAIN        = 135.0f;  // How aggressively stick deflection is translated into angle offset. You'll move +-X degrees from current bas position
  static constexpr int   BASE_STEP   = 1;      // Change base_pos_ by 1° each tick
  static constexpr float RT_THRESHOLD= 0.9f; // Cuttoff on right trigger axis value
  static constexpr int   FAST_STEP   = 10; // The amount you subtract from servo5_last
  static constexpr int   STEP_SIZE   = 10; // The amount (in degrees) you subtract from servo5_last_ each tick
  static constexpr int   SERVO5_MIN  = 0; // Lower limit for propulsion (servo 5)
  
  int base_pos_;
  int servo5_last_;
  sensor_msgs::msg::Joy::SharedPtr              last_joy_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr   joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr      servo_pub_;
  rclcpp::TimerBase::SharedPtr                             timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickServoControl>());
  rclcpp::shutdown();
  return 0;
}
