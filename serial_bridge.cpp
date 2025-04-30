#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial/serial.h>
#include <sstream>

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
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
        }

        if (!serial_port_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
        }

        servo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/servo_commands", 10,
            std::bind(&SerialBridge::sendCommand, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&SerialBridge::readSerial, this));
    }

private:
    void sendCommand(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            serial_port_.write(msg->data + "\n");
            RCLCPP_DEBUG(this->get_logger(), "Sent: %s", msg->data.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", e.what());
        }
    }

    void readSerial()
    {
        try {
            while (serial_port_.available()) {
                std::string line = serial_port_.readline(200, "\n");
                line.erase(std::remove(line.begin(), line.end(), '\r'), line.end()); // remove CR if present
                line.erase(std::remove(line.begin(), line.end(), '\n'), line.end()); // remove LF if present

                if (line.rfind("IMU:", 0) == 0) { // Starts with IMU:
                    parseImuLine(line.substr(4));
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Serial read failed: %s", e.what());
        }
    }

    void parseImuLine(const std::string &data)
    {
        double ax, ay, az, gx, gy, gz;
        char comma;  // dummy variable for commas

        std::istringstream ss(data);

        if ((ss >> ax >> comma >> ay >> comma >> az >> comma >> gx >> comma >> gy >> comma >> gz) && (ss.eof() || ss.peek() == '\n'))
        {
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";

            imu_msg.linear_acceleration.x = ax;
            imu_msg.linear_acceleration.y = ay;
            imu_msg.linear_acceleration.z = az;

            imu_msg.angular_velocity.x = gx;
            imu_msg.angular_velocity.y = gy;
            imu_msg.angular_velocity.z = gz;

            imu_pub_->publish(imu_msg);

            RCLCPP_DEBUG(this->get_logger(), "Published IMU: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f",
                         ax, ay, az, gx, gy, gz);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Malformed IMU line: %s", data.c_str());
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
