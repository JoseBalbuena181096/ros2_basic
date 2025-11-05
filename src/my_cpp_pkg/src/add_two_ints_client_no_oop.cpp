# include "rclcpp/rclcpp.hpp"
# include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("add_two_ints_client_no_oop");

    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while(!client->wait_for_service(1s)) {
        RCLCPP_WARN(
            node->get_logger(),
            "Interrupted while waiting for the service. Exiting.");
    }
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 5;
    request->b = 3;
    auto future = client->async_send_request(request);
    // Wait for the result.
    rclcpp::spin_until_future_complete(node, future);
    
    auto response = future.get();
    RCLCPP_INFO(
        node->get_logger(),
        "Result of add_two_ints: %d + %d = %d",
        (int)request->a,
        (int)request->b,
        (int)response->sum);

    rclcpp::shutdown();
    return 0;
}