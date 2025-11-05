#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node("py_test")
    node.get_logger().info("Hello word")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
nota al compilar el paquete se recomida hacer un source ~/.bashrc para que el nodo 
se encuentre en el path
"""
