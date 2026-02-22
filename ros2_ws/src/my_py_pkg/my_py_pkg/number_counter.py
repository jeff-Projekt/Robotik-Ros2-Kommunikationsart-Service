#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from my_tutorial_interfaces.srv import ResetCounter

class NumberCounterNode(Node):
    
    def __init__(self):
        super().__init__('number_counter') 
        self.counter = 0
        self.number_subscriber = self.create_subscription (Int64,"number", self.callback_number, 10)
        self.reset_counter_service = self.create_service(ResetCounter, "reset_counter",
        self.callback_reset_counter)
        self.get_logger().info("Number Counter has been started.")

    def callback_number(self, msg: Int64):
        self.counter += msg.data
        self.get_logger().info("Counter: " + str(self.counter))
    
    def callback_reset_counter(self, request: ResetCounter.Request, response: ResetCounter.Response):
        self.counter = request.reset_value
        self.get_logger().info("Reset counter to " + str(self.counter))
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()