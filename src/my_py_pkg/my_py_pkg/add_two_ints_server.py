#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 
                                       'add_two_ints', 
                                       self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service is ready.')

    def add_two_ints_callback(self, 
                              request: AddTwoInts.Request, 
                              response: AddTwoInts.Response):
        
        response.sum = request.a + request.b
        self.get_logger().info(f'Received request: a={request.a}, b={request.b}, sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()
    rclpy.spin(add_two_ints_server)
    add_two_ints_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()