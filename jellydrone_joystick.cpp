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
    wasHolding_(false),
    stickActive_(false)
  {
    // initialize all to true neutral
    for (int i = 0; i < 4; ++i) {
      base_[i]     = 90;
      last_set_[i] = 90;
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoystickServoControl::joyCallback, this, std::placeholders::_1));
    servo_pub_ = create_publisher<std_msgs::msg::String>("servo_commands", 10);
    timer_     = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&JoystickServoControl::publishCommand, this));

    RCLCPP_INFO(get_logger(), "JoystickServoControl (with re-center) running");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_ = msg;
  }

  void publishCommand()
  {
    if (!last_joy_) return;
    bool holdA = last_joy_->buttons[0];
    bool holdB = last_joy_->buttons[1];
    bool sent = false;

    // 1) Ramp in-node while holding A/B
    if (holdA || holdB) {
      for (int i = 0; i < 4; ++i) {
        if (holdA) last_set_[i] = std::min(last_set_[i] + STEP_SIZE, 180);
        else       last_set_[i] = std::max(last_set_[i] - STEP_SIZE,   0);
      }
      wasHolding_ = true;
      sent = true;
    }
    // 2) On release of A/B: snapshot base & still send final ramp
    else if (wasHolding_) {
      wasHolding_ = false;
      for (int i = 0; i < 4; ++i) {
        base_[i] = last_set_[i];
      }
      sent = true;
    }
    // 3) Joystick-driven
    else {
      float ax = last_joy_->axes[0];
      float ay = last_joy_->axes[1];
      bool moved = std::fabs(ax) > DEADZONE || std::fabs(ay) > DEADZONE;

      if (moved) {
        // compute relative ±90° around base
        int d1 = static_cast<int>(ax * 90.0f);
        int d2 = static_cast<int>(ay * 90.0f);
        last_set_[0] = std::clamp(base_[0] + d1, 0, 180);
        last_set_[1] = std::clamp(base_[1] + d2, 0, 180);
        last_set_[2] = std::clamp(base_[2] - d1, 0, 180);
        last_set_[3] = std::clamp(base_[3] - d2, 0, 180);

        sent = true;
        stickActive_ = true;
      }
      else if (stickActive_) {
        // stick just returned to neutral → re-center to base
        for (int i = 0; i < 4; ++i) {
          last_set_[i] = base_[i];
        }
        sent = true;
        stickActive_ = false;
      }
      // else: stick is neutral and not previously active → do nothing
    }

    if (sent) {
      // servo 5 remains absolute from its button
      int s5 = (last_joy_->buttons[5] ? 180 : 90);

      std::ostringstream oss;
      oss << "SET:"
          << last_set_[0] << ',' << last_set_[1] << ','
          << last_set_[2] << ',' << last_set_[3] << ','
          << s5;

      auto msg = std_msgs::msg::String();
      msg.data = oss.str();
      servo_pub_->publish(msg);
      RCLCPP_DEBUG(get_logger(), "Published SET: %s", msg.data.c_str());
    }
  }

  // constants
  static constexpr int STEP_SIZE = 1;
  static constexpr float DEADZONE = 0.05f;

  // state
  bool wasHolding_;
  bool stickActive_;
  int  base_[4];      // frozen center when A/B released
  int  last_set_[4];  // last angles sent for servos 1–4

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr servo_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickServoControl>());
  rclcpp::shutdown();
  return 0;
}
