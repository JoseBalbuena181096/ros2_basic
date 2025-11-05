#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class RobotNewsStationNode : public rclcpp::Node
{   
    public:
        RobotNewsStationNode() : Node("robot_news_station_node")
        {
            RCLCPP_INFO(this->get_logger(), "Robot news station has been started.");
            publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
            timer_ = this->create_wall_timer(
                500ms,
                std::bind(&RobotNewsStationNode::publish_news, this));
        }

    private:
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        std::string robot_name_ = "Robot C30";
        rclcpp::TimerBase::SharedPtr timer_;
        int counter_ = 0;

        void publish_news()
        {
            auto  msg = example_interfaces::msg::String();
            msg.data = std::string("Hello, this is  ")
                + robot_name_ 
                + std::string(" from the robot news station ") 
                + std::to_string(counter_)
                + std::string(" !");
            publisher_->publish(msg);
            counter_++;
        }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/**
Build and run the node:
$ colcon build --packages-select my_cpp_pkg
$ ros2 run my_cpp_pkg robot_news_station
*/
