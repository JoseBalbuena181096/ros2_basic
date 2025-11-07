#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")
        self.hw_status_pub_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_hw_status)
        self.get_logger().info("Hardware Status publisher has been started.")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.are_motors_ready =  True  
        msg.temperature = 45.0  
        msg.debug_message = "All systems operational."
        self.hw_status_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()