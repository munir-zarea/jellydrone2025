#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <glob.h>

using namespace std::chrono_literals;

class ImuSerialPublisher : public rclcpp::Node
{
public:
  ImuSerialPublisher()
  : Node("imu_serial_publisher")
  {
    // Declare and get parameters
    this->declare_parameter("baudrate", 115200);
    baudrate_ = this->get_parameter("baudrate").as_int();

    // Auto-detect port
    port_ = autoDetectPort(baudrate_);
    if (port_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Could not auto-detect IMU serial port.");
      rclcpp::shutdown();
      return;
    }

    // Open serial port
    try {
      serial_.setPort(port_);
      serial_.setBaudrate(baudrate_);
      serial::Timeout to = serial::Timeout::simpleTimeout(50);
      serial_.setTimeout(to);
      serial_.open();
    } catch (const std::exception &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to open serial [%s]: %s",
                   port_.c_str(), e.what());
      rclcpp::shutdown();
      return;
    }

    if (!serial_.isOpen()) {
      RCLCPP_FATAL(this->get_logger(),
                   "Serial port %s failed to open.", port_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Serial port %s @ %d ready", port_.c_str(), baudrate_);

    // Create publisher
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    // Timer for polling at 20Hz
    timer_ = this->create_wall_timer(
      50ms, std::bind(&ImuSerialPublisher::timerCallback, this));
  }

private:
  std::string autoDetectPort(int baudrate)
  {
    glob_t glob_result;
    glob("/dev/ttyACM*", GLOB_TILDE, NULL, &glob_result);

    for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
      std::string candidate = glob_result.gl_pathv[i];
      try {
        serial::Serial test_serial(candidate, baudrate, serial::Timeout::simpleTimeout(250));
        rclcpp::sleep_for(std::chrono::milliseconds(300));  // Give Arduino time

        if (test_serial.available()) {
          std::string line = test_serial.readline(256, "\n");
          if (line.rfind("IMU:", 0) == 0) {
            test_serial.close();
            globfree(&glob_result);
            return candidate;
          }
        }
      } catch (...) {
        continue; // Try next port
      }
    }

    globfree(&glob_result);
    return "";
  }

  void timerCallback()
  {
    try {
      serial_.write("R\n");  // Send request
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to write request: %s", e.what());
      return;
    }

    std::string line;
    try {
      line = serial_.readline(256, "\n");
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                  "Timeout or read error: %s", e.what());
      return;
    }

    if (line.rfind("IMU:", 0) != 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Unexpected reply: %s", line.c_str());
      return;
    }

    std::string data = line.substr(4);
    std::istringstream ss(data);
    std::string token;
    std::vector<float> vals;
    while (std::getline(ss, token, ',')) {
      try {
        vals.push_back(std::stof(token));
      } catch (...) {
        RCLCPP_WARN(this->get_logger(),
                    "Bad float token: %s", token.c_str());
        return;
      }
    }

    if (vals.size() != 6) {
      RCLCPP_WARN(this->get_logger(),
                  "Got %zu values, expected 6", vals.size());
      return;
    }

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    msg.linear_acceleration.x = vals[0];
    msg.linear_acceleration.y = vals[1];
    msg.linear_acceleration.z = vals[2];

    msg.angular_velocity.x    = vals[3];
    msg.angular_velocity.y    = vals[4];
    msg.angular_velocity.z    = vals[5];

    msg.orientation_covariance[0] = -1;

    imu_pub_->publish(msg);
  }

  std::string port_;
  int baudrate_;
  serial::Serial serial_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSerialPublisher>());
  rclcpp::shutdown();
  return 0;
}
