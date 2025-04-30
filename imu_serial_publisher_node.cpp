#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <vector>

class ImuSerialPublisher : public rclcpp::Node
{
public:
    ImuSerialPublisher()
    : Node("imu_serial_publisher")
    {
        // Declare and get parameters
        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 115200);
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        // Init serial
        try {
            serial_.setPort(port);
            serial_.setBaudrate(baudrate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(10);  // LOW timeout for fast reads
            serial_.setTimeout(timeout);
            serial_.open();
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial: %s", e.what());
            rclcpp::shutdown();
        }

        if (!serial_.isOpen()) {
            RCLCPP_FATAL(this->get_logger(), "Serial port failed to open.");
            rclcpp::shutdown();
        }

        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                                         std::bind(&ImuSerialPublisher::readSerial, this));
    }

private:
    void readSerial()
    {
        while (serial_.available()) {
            std::string line = serial_.readline(100, "\n");
            if (line.rfind("IMU:", 0) != 0) {
                continue;  // Skip non-IMU lines
            }

            // Remove "IMU:" prefix
            line = line.substr(4);

            // Split line into floats
            std::istringstream ss(line);
            std::string token;
            std::vector<float> values;

            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stof(token));
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse: %s", token.c_str());
                    return;
                }
            }

            if (values.size() != 6) {
                RCLCPP_WARN(this->get_logger(), "Malformed IMU line: '%s'", line.c_str());
                return;
            }

            auto msg = sensor_msgs::msg::Imu();
            msg.header.stamp = this->now();
            msg.header.frame_id = "imu_link";

            // Linear acceleration (m/s^2)
            msg.linear_acceleration.x = values[0];
            msg.linear_acceleration.y = values[1];
            msg.linear_acceleration.z = values[2];

            // Angular velocity (rad/s)
            msg.angular_velocity.x = values[3];
            msg.angular_velocity.y = values[4];
            msg.angular_velocity.z = values[5];

            // Orientation not estimated
            msg.orientation_covariance[0] = -1;

            pub_->publish(msg);
        }
    }

    serial::Serial serial_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuSerialPublisher>());
    rclcpp::shutdown();
    return 0;
}
