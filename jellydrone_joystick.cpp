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
    base_pos_(DEFAULT_POS),
    servo5_target_cmd_(DEFAULT_POS)
  {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoystickServoControl::joyCallback, this, std::placeholders::_1));
    servo_pub_ = create_publisher<std_msgs::msg::String>("servo_commands", 10);
    timer_ = create_wall_timer(20ms, std::bind(&JoystickServoControl::publishCommand, this));
    RCLCPP_INFO(get_logger(), "Joystick servo control running (prepose + slow-retract jellyfish seq)");
  }

private:
  // ====== Tuning constants ======
  static constexpr int   DEFAULT_POS = 150;     // "original" angle to start each cycle from
  static constexpr int   SERVO_MIN   = 0;
  static constexpr int   SERVO_MAX   = 270;
  static constexpr float GAIN        = 135.0f;  // stick gain for servos 1–4
  static constexpr int   BASE_STEP   = 1;

  // Manual RT behavior (only when Idle)
  static constexpr float RT_THRESHOLD               = 0.9f;
  static constexpr int   MANUAL_FAST_CONTRACT_STEP  = 10; // deg/tick when RT pressed
  static constexpr int   MANUAL_RELAX_STEP          = 10; // deg/tick toward DEFAULT

  // Jellyfish sequence parameters
  static constexpr int   CONTRACT_DELTA_DEG = 400;  // contract this many degrees from original (clamped by safe min)
  static constexpr int   GLIDE_MS           = 1000; // glide duration
  static constexpr int   RETURN_HOLD_MS     = 150;  // settle at original before next cycle
  static constexpr int   CYCLES_PER_PRESS   = 3;

  // SAFE MIN for servo 5 to avoid mechanical hard-stop/buzzing
  static constexpr int   SERVO5_MIN_SAFE    = 2;    // raise if you hear buzzing at the extreme
  static constexpr int   SERVO5_MAX_SAFE    = 270;   // adjust to your tested safe upper limit

  // Arduino motion model (must match your Arduino sketch)
  static constexpr int   ARD_STEP_DEG         = 18;  // MatchArduino STEP_SIZE_5 (deg per tick)
  static constexpr int   ARD_MOVE_INTERVAL_MS = 2;  // Match Arduino MOVE_INTERVAL_5 (ms per tick)
  static constexpr int   SAFETY_BUFFER_MS     = 80; // extra settle time at each move

  // Slow retraction pacing (command side)
  static constexpr int   RETRACT_CMD_STEP_DEG    = 4;   // command increments (deg) toward original
  static constexpr int   RETRACT_CMD_INTERVAL_MS = 20;  // time between increments (ms)

  // ====== State ======
  enum class SeqState { Idle, PreposeTravel, ContractTravel, GlideHold, RetractRamp, ReturnHold };
  SeqState seq_state_ = SeqState::Idle;

  int  base_pos_;
  int  servo5_target_cmd_;               // last commanded absolute target (deg) for servo 5
  int  servo5_original_ = DEFAULT_POS;   // captured "original" for the current run (deg)
  int  contract_target_ = DEFAULT_POS;   // absolute contract target for this run
  int  cycles_remaining_ = 0;

  // Timing
  std::chrono::steady_clock::time_point phase_end_{};
  std::chrono::steady_clock::time_point next_retract_cmd_time_{};

  // Button edges
  bool prev_a_ = false;
  bool prev_b_ = false;

  // I/O
  sensor_msgs::msg::Joy::SharedPtr last_joy_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    servo_pub_;
  rclcpp::TimerBase::SharedPtr                           timer_;

  // ====== Helpers ======
  static int clampAngle(int a) { return std::max(SERVO_MIN, std::min(SERVO_MAX, a)); }

  // Time Arduino needs to move from 'from_deg' to 'to_deg' (based on STEP/INTERVAL)
  static int travelTimeMs(int from_deg, int to_deg) {
    int delta = std::abs(to_deg - from_deg);
    int steps = (delta + ARD_STEP_DEG - 1) / ARD_STEP_DEG; // ceil
    return steps * ARD_MOVE_INTERVAL_MS + SAFETY_BUFFER_MS;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) { last_joy_ = msg; }

  // Begin a run: ensure we start from DEFAULT_POS (prepose), then contract
  void startSequence()
  {
    cycles_remaining_ = CYCLES_PER_PRESS;

    // If not already at DEFAULT_POS, prepose first
    if (servo5_target_cmd_ != DEFAULT_POS) {
      int t_ms = travelTimeMs(servo5_target_cmd_, DEFAULT_POS);
      servo5_target_cmd_ = DEFAULT_POS;
      phase_end_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(t_ms);
      seq_state_ = SeqState::PreposeTravel;
    } else {
      // Already at default; start contracting immediately
      beginContractFromDefault();
    }
  }

  // After prepose completes (or if already at default), start the contract phase
  void beginContractFromDefault()
  {
    servo5_original_ = DEFAULT_POS;  // capture a consistent, high original
    // Contract target: original - delta, but never go below the safe min
    contract_target_ = std::min(SERVO5_MAX_SAFE, servo5_original_ + CONTRACT_DELTA_DEG);
    contract_target_ = clampAngle(contract_target_);

    int t_ms = travelTimeMs(servo5_target_cmd_, contract_target_);
    servo5_target_cmd_ = contract_target_;
    phase_end_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(t_ms);
    seq_state_ = SeqState::ContractTravel;
  }

  void stopSequence()
  {
    seq_state_ = SeqState::Idle;
    cycles_remaining_ = 0;
  }

  void advanceSequenceIfReady()
  {
    using clock = std::chrono::steady_clock;
    auto now = clock::now();

    switch (seq_state_) {
      case SeqState::PreposeTravel:
        if (now >= phase_end_) {
          beginContractFromDefault();
        }
        break;

      case SeqState::ContractTravel:
        if (now >= phase_end_) {
          // Glide at contracted position
          phase_end_ = now + std::chrono::milliseconds(GLIDE_MS);
          seq_state_ = SeqState::GlideHold;
        }
        break;

      case SeqState::GlideHold:
        if (now >= phase_end_) {
          // Begin slow retract ramp toward original
          next_retract_cmd_time_ = now;
          seq_state_ = SeqState::RetractRamp;
        }
        break;

      case SeqState::RetractRamp: {
        // Pace command updates for slower retraction
        if (now >= next_retract_cmd_time_) {
          if (servo5_target_cmd_ < servo5_original_) {
            int diff = servo5_original_ - servo5_target_cmd_;
            int step = std::min(RETRACT_CMD_STEP_DEG, diff);
            servo5_target_cmd_ += step;  // inch upward toward original
          } else if (servo5_target_cmd_ > servo5_original_) {
            int diff = servo5_target_cmd_ - servo5_original_;
            int step = std::min(RETRACT_CMD_STEP_DEG, diff);
            servo5_target_cmd_ -= step;  // inch downward if needed
          }

          if (servo5_target_cmd_ == servo5_original_) {
            // Reached original: brief hold before next cycle
            phase_end_ = now + std::chrono::milliseconds(RETURN_HOLD_MS);
            seq_state_ = SeqState::ReturnHold;
          } else {
            // Schedule next small step
            next_retract_cmd_time_ = now + std::chrono::milliseconds(RETRACT_CMD_INTERVAL_MS);
          }
        }
        break;
      }

      case SeqState::ReturnHold:
        if (now >= phase_end_) {
          if (--cycles_remaining_ > 0) {
            // Next cycle: go back down to the same contract target
            int t_ms = travelTimeMs(servo5_target_cmd_, contract_target_);
            servo5_target_cmd_ = contract_target_;
            phase_end_ = now + std::chrono::milliseconds(t_ms);
            seq_state_ = SeqState::ContractTravel;
          } else {
            seq_state_ = SeqState::Idle;
          }
        }
        break;

      case SeqState::Idle:
      default:
        break;
    }
  }

  void publishCommand()
  {
    if (!last_joy_ || last_joy_->axes.size() < 6) return;

    // Buttons (Xbox layout by default; adjust indices if needed)
    auto btn = [&](size_t i){ return last_joy_->buttons.size() > i && last_joy_->buttons[i]; };
    bool lb = btn(4), rb = btn(5), a = btn(0), b = btn(1);

    // Base trim for servos 1–4
    if (lb)       base_pos_ = clampAngle(base_pos_ + BASE_STEP);
    else if (rb)  base_pos_ = clampAngle(base_pos_ - BASE_STEP);

    // Sticks with deadzone (servos 1–4)
    float x = last_joy_->axes[0];
    float y = -last_joy_->axes[1];
    constexpr float deadzone = 0.1f;
    if (std::abs(x) < deadzone) x = 0.0f;
    if (std::abs(y) < deadzone) y = 0.0f;

    int s1 = clampAngle(int(base_pos_ + x * GAIN));
    int s3 = clampAngle(int(base_pos_ - x * GAIN));
    int s2 = clampAngle(int(base_pos_ + y * GAIN));
    int s4 = clampAngle(int(base_pos_ - y * GAIN));

    // A/B rising edges
    if (a && !prev_a_) startSequence();
    if (b && !prev_b_) stopSequence();

    // Sequence progression or manual RT for servo 5
    if (seq_state_ == SeqState::Idle) {
      // Manual: RT contracts, otherwise relax toward DEFAULT
      float rt = last_joy_->axes[5];
      if (rt < RT_THRESHOLD) {
        // Respect SAFE MIN in manual mode as well
        servo5_target_cmd_ = std::min(servo5_target_cmd_ + MANUAL_FAST_CONTRACT_STEP, SERVO5_MAX_SAFE);
      } else {
        if (servo5_target_cmd_ > DEFAULT_POS)
          servo5_target_cmd_ = std::max(servo5_target_cmd_ - MANUAL_RELAX_STEP, DEFAULT_POS);
        else if (servo5_target_cmd_ < DEFAULT_POS)
          servo5_target_cmd_ = std::min(servo5_target_cmd_ + MANUAL_RELAX_STEP, DEFAULT_POS);
      }
    } else {
      // Advance sequence state machine (handles prepose, glide, slow retract, etc.)
      advanceSequenceIfReady();
    }

    // Publish to Arduino (absolute targets)
    std_msgs::msg::String out;
    out.data = "SET:" +
      std::to_string(s1) + ',' +
      std::to_string(s2) + ',' +
      std::to_string(s3) + ',' +
      std::to_string(s4) + ',' +
      std::to_string(servo5_target_cmd_);
    servo_pub_->publish(out);

    prev_a_ = a; prev_b_ = b;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickServoControl>());
  rclcpp::shutdown();
  return 0;
}
