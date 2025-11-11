#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.robot_name_ = "C30"
        
        self.declare_parameter("robot_name", "C110")
        self.robot_name_ = self.get_parameter("robot_name").value
        
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot News Station has been started.")        
        self.count_ = 0

    def publish_news(self):
        msg = String()
        msg.data = f"Hi, this is {self.robot_name_} from the Robot News Station {self.count_}!"
        self.publisher_.publish(msg)
        self.count_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

"""
If you create the workspace the incorret way, you can rebuild it but you need delete built, install and log folder:

$ rm -rf build/ install/ log/

Build my package:

$ colcon build --packages-select my_py_pkg

run robot_news_station:

$ ros2 run my_py_pkg robot_news_station
"""