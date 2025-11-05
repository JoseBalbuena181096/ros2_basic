#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')
        

    def call_add_two_ints(self, a: int, b: int) -> int:
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service add_two_ints...')
       
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_add_two_ints, request=request))

    def callback_call_add_two_ints(self, future, request):
        response = future.result()
        self.get_logger().info(f'Received request: a={request.a}, b={request.b}, sum={response.sum}')


def main(args=None):
    rclpy.init(args=args)
    add_two_ints_client = AddTwoIntsClient()
    for i in range(5):
        add_two_ints_client.call_add_two_ints(i, i + 1)
    rclpy.spin(add_two_ints_client)
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()      