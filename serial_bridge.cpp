#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>
#include <sstream>
#include <vector>

class SerialBridge : public rclcpp::Node
{
public:
    SerialBridge()
        : Node("serial_bridge")
    {
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 115200);
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        try {
            serial_port_.setPort(port);
            serial_port_.setBaudrate(baudrate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
            serial_port_.setTimeout(timeout);
            serial_port_.open();

            rclcpp::sleep_for(std::chrono::seconds(2));  // allow Arduino reset
            RCLCPP_INFO(this->get_logger(), "Serial port opened at %s", port.c_str());

        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "Serial error: %s", e.what());
            rclcpp::shutdown();
        }

        if (!serial_port_.isOpen()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port.");
            rclcpp::shutdown();
        }

        // Sub to servo commands
        servo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "servo_commands", 10,
            std::bind(&SerialBridge::sendCommand, this, std::placeholders::_1));

        // Pub for IMU data
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);

        // Timer for polling serial reads
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SerialBridge::readSerial, this));
    }

private:
    void sendCommand(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            serial_port_.write(msg->data + "\n");
            RCLCPP_DEBUG(this->get_logger(), "Sent to serial: %s", msg->data.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", e.what());
        }
    }

    void readSerial()
    {
        while (serial_port_.available()) {
            std::string line = serial_port_.readline(100, "\n");

            // Check prefix
            if (line.rfind("IMU:", 0) != 0) {
                continue;  // skip non-IMU lines
            }

            line = line.substr(4);  // remove prefix
            std::istringstream ss(line);
            std::string token;
            std::vector<float> values;

            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stof(token));
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse float from: %s", token.c_str());
                    return;
                }
            }

            if (values.size() != 6) {
                RCLCPP_WARN(this->get_logger(), "Malformed IMU line: '%s'", line.c_str());
                return;
            }

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";

            // Linear accel (m/s²)
            imu_msg.linear_acceleration.x = values[0];
            imu_msg.linear_acceleration.y = values[1];
            imu_msg.linear_acceleration.z = values[2];

            // Angular velocity (rad/s)
            imu_msg.angular_velocity.x = values[3];
            imu_msg.angular_velocity.y = values[4];
            imu_msg.angular_velocity.z = values[5];

            // Orientation not provided
            imu_msg.orientation_covariance[0] = -1;

            imu_pub_->publish(imu_msg);
        }
    }

    serial::Serial serial_port_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridge>());
    rclcpp::shutdown();
    return 0;
}
