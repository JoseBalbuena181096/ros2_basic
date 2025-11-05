#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

const int64_t NUMBER = 200;

using namespace std::chrono_literals;

class NumberPublisherNode : public rclcpp::Node
{   
    public:
        NumberPublisherNode() : Node("number_publisher_node")
        {
            RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
            timer_ = this->create_wall_timer(
                500ms,
                std::bind(&NumberPublisherNode::publish_number, this));
        }

    private:
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void publish_number()
        {
            auto msg = example_interfaces::msg::Int64();
            msg.data = NUMBER;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published number: %ld", msg.data);
        }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
