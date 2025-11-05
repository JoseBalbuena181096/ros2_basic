#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::placeholders;

class SmartphoneNode : public rclcpp::Node
{   
    public:
        SmartphoneNode() : Node("smartphone")
        {
            RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
            subscriber_ = this->create_subscription<example_interfaces::msg::String>(
                "robot_news", 10,
                std::bind(&SmartphoneNode::callbackRobotNews, this, _1));   
        }
    private:
        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
        void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->data.c_str());
        }
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
/**
Build and run the node:
$ colcon build --packages-select my_cpp_pkg
$ ros2 run my_cpp_pkg smartphone
*/
