#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::placeholders;

class NumberCounterNode : public rclcpp::Node
{   
    public:
        NumberCounterNode() : Node("number_counter")
        {
            RCLCPP_INFO(this->get_logger(), "Number counter has been started.");
            subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
                "number", 10,
                std::bind(&NumberCounterNode::callbackNumber, this, _1));
            
            publisher_ = this->create_publisher<example_interfaces::msg::String>("number_count", 10);

            server_ = this->create_service<example_interfaces::srv::SetBool>(
                "reset_counter",
                std::bind(&NumberCounterNode::resetCountCallback, this,
                _1, _2));
        }
        
    private:
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        int64_t count_ = 0;
        rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;

        void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
        {
            
            auto str_msg = example_interfaces::msg::String();
            count_ += msg->data;
            str_msg.data = std::string("data: ")
                + std::to_string(count_);
            publisher_->publish(str_msg);
            RCLCPP_INFO(this->get_logger(), "data: %ld", count_);
        }

        void resetCountCallback(
            const example_interfaces::srv::SetBool::Request::SharedPtr request,
            const example_interfaces::srv::SetBool::Response::SharedPtr response)
        {
            if (request->data) {
                this->count_ = 0;
                response->success = true;
                response->message = "Count has been reset to zero.";
                RCLCPP_INFO(this->get_logger(), "Count has been reset to zero.");
            } else {
                response->success = false;
                response->message = "Count reset not performed.";
                RCLCPP_INFO(this->get_logger(), "Count reset not performed.");
            }
        }
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}