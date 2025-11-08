#include "rclcpp/rclcpp.hpp"
#include "batery_led_interfaces/srv/set_led.hpp"

using std::placeholders::_1;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery_node"), battery_state_(100.0)
    {
        client_ = this->create_client<batery_led_interfaces::srv::SetLed>("set_led");
        
        // Timer para simular el drenado de la batería (vacía en 4 segundos)
        timer_drain_ = this->create_wall_timer(
            std::chrono::seconds(4),
            std::bind(&BatteryNode::battery_empty, this));
        
        // Timer para simular la carga de la batería (llena en 6 segundos después de vacía)
        timer_charge_ = this->create_wall_timer(
            std::chrono::seconds(6),
            std::bind(&BatteryNode::battery_full, this));
        
        // Inicialmente solo el timer de drenado está activo
        timer_charge_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "Battery Node started - Battery is FULL");
    }

private:
    rclcpp::Client<batery_led_interfaces::srv::SetLed>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_drain_;
    rclcpp::TimerBase::SharedPtr timer_charge_;
    double battery_state_;
    
    void battery_empty()
    {
        battery_state_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Battery is now EMPTY");
        
        // Encender LED (índice 2) cuando la batería está vacía
        call_set_led_service(2, true);
        
        // Cancelar el timer de drenado y activar el de carga
        timer_drain_->cancel();
        timer_charge_->reset();
    }
    
    void battery_full()
    {
        battery_state_ = 100.0;
        RCLCPP_INFO(this->get_logger(), "Battery is now FULL");
        
        // Apagar LED (índice 2) cuando la batería está llena
        call_set_led_service(2, false);
        
        // Cancelar el timer de carga y activar el de drenado
        timer_charge_->cancel();
        timer_drain_->reset();
    }
    
    void call_set_led_service(int led_index, bool turn_state)
    {
        // Verificar si el servicio está disponible
        if (!client_->service_is_ready())
        {
            RCLCPP_WARN(this->get_logger(), "Service not available, skipping request");
            return;
        }
        
        auto request = std::make_shared<batery_led_interfaces::srv::SetLed::Request>();
        request->led_index = led_index;
        request->turn_state = turn_state;
        
        // Enviar la solicitud de forma asíncrona con un callback
        auto result_callback = [this, led_index, turn_state](
            rclcpp::Client<batery_led_interfaces::srv::SetLed>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "LED %d set to %s successfully", 
                           led_index, turn_state ? "ON" : "OFF");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set LED %d", led_index);
            }
        };
        
        client_->async_send_request(request, result_callback);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}