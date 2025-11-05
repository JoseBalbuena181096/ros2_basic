#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool
from std_msgs.msg import String

class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_number, 10)
        self.publisher_ = self.create_publisher(String, "number_count", 10)

        self.service_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        self.get_logger().info("Number counter has been started.")
        self.count_ = 0

    def callback_reset_counter(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.count_ = 0
            response.success = True
            response.message = "Counter has been reset to zero."
            self.get_logger().info("Counter has been reset to zero.")
        else:
            response.success = False
            response.message = "Counter reset not performed."
            self.get_logger().info("Counter reset not performed.")
        return response

    def callback_number(self, msg: Int64):
        str_msg = String()
        self.count_ += msg.data
        str_msg.data = f"data: {self.count_} "
        self.publisher_.publish(str_msg)
        self.get_logger().info(f"data: {self.count_} ")

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()