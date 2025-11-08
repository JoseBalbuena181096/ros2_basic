#include "rclcpp/rclcpp.hpp"
#include "batery_led_interfaces/srv/set_led.hpp"
#include "batery_led_interfaces/msg/led_panel_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class LedNodePanel : public rclcpp::Node
{
public:
    LedNodePanel() : Node("led_node_panel"), led_panel_state_{false, false, false}
    {
        // Crear el servicio para manejar las solicitudes de encendido/apagado de LEDs          
        server_ = this->create_service<batery_led_interfaces::srv::SetLed>(
            "set_led",
            std::bind(&LedNodePanel::handle_set_led_request, this, _1, _2));
        
        // Crear el publisher para publicar el estado del panel LED
        publisher_ = this->create_publisher<batery_led_interfaces::msg::LedPanelState>(
            "led_panel_state", 10);
        
        // Timer para publicar el estado cada 1 segundo
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LedNodePanel::publish_led_state, this));
        
        RCLCPP_INFO(this->get_logger(), "LED Panel Node started");
    }

private:
    void handle_set_led_request(
        const std::shared_ptr<batery_led_interfaces::srv::SetLed::Request> request,
        std::shared_ptr<batery_led_interfaces::srv::SetLed::Response> response)
    {
        int led_index = request->led_index;
        bool turn_state = request->turn_state;
        
        // Verificar que el índice del LED sea válido (0, 1, o 2)
        if (led_index >= 0 && led_index < 3)
        {
            led_panel_state_[led_index] = turn_state;
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "LED %d set to %s", led_index, turn_state ? "ON" : "OFF");
        }
        else
        {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "Invalid LED index: %d", led_index);
        }
    }
    
    void publish_led_state()
    {
        auto msg = batery_led_interfaces::msg::LedPanelState();
        msg.led_panel_state = led_panel_state_;
        publisher_->publish(msg);
    }

    std::vector<bool> led_panel_state_;
    rclcpp::Service<batery_led_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::Publisher<batery_led_interfaces::msg::LedPanelState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};      

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedNodePanel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}