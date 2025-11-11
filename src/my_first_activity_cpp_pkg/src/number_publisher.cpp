#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"


using namespace std::chrono_literals;

class NumberPublisherNode : public rclcpp::Node
{   
    public:
        NumberPublisherNode() : Node("number_publisher_node")
        {
            this->declare_parameter("number", 200);
            this->declare_parameter("timer_period", 0.5);

            number_ = this->get_parameter("number").as_int();
            timer_period_ = this->get_parameter("timer_period").as_double();

            param_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&NumberPublisherNode::parameters_callback, this, std::placeholders::_1));
            
            RCLCPP_INFO(this->get_logger(), "Number publisher has been started. Number: %ld, Timer period: %f", number_, timer_period_);
            publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(timer_period_),
                std::bind(&NumberPublisherNode::publish_number, this));
        }

    private:
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        int64_t number_;
        double timer_period_;


        rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
                {
                    for (const auto & param : parameters)
                    {
                        if (param.get_name() == "number")
                        {
                            number_ = param.as_int();
                            RCLCPP_INFO(this->get_logger(), "Parameter 'number' changed to: %ld", number_);
                        }
                    }
                    rcl_interfaces::msg::SetParametersResult result;
                    result.set__successful(true);
                    return result;
                }

        void publish_number()
        {
            auto msg = example_interfaces::msg::Int64();
            msg.data = number_;
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
