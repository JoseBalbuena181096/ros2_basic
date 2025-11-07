#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

using namespace std::chrono_literals;

class HardwareStatusPublisher : public rclcpp::Node
{
public:
    HardwareStatusPublisher() : Node("hardware_status_publisher")
    {
        hw_status_pub_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(
            1000ms, 
            std::bind(&HardwareStatusPublisher::publish_hw_status, this));
        RCLCPP_INFO(this->get_logger(), "Hardware Status Publisher has been started.");
    }

private:

    void publish_hw_status()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.are_motors_ready = true;
        msg.temperature = 45.0;
        msg.debug_message = "All systems operational.";
        hw_status_pub_->publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr hw_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareStatusPublisher>());
    rclcpp::shutdown();
    return 0;
}