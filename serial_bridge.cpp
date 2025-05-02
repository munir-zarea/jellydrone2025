#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include <sstream>
#include <algorithm>

class SerialBridge : public rclcpp::Node
{
public:
    SerialBridge() : Node("serial_bridge")
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

            // Wait for Arduino reset (very important!)
            rclcpp::sleep_for(std::chrono::seconds(2));
            RCLCPP_INFO(this->get_logger(), "Serial port opened and Arduino is ready");
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
        }

        if (!serial_port_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port opened on %s at %d baud", port.c_str(), baudrate);
        }

        servo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/servo_commands", 10,
            std::bind(&SerialBridge::sendCommand, this, std::placeholders::_1));
    }

private:
    void sendCommand(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            serial_port_.write(msg->data + "\n");
            RCLCPP_INFO(this->get_logger(), "Sent to serial: %s", msg->data.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", e.what());
        }
    }

    serial::Serial serial_port_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridge>());
    rclcpp::shutdown();
    return 0;
}
