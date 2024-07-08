#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool
 
 
class CounterReseterNode(Node):
    def __init__(self):
        super().__init__("counter_reseter")
        self.subscriber_ = self.create_subscription(Int64, "number_count", self.callback_number_count, 10)
        self.client_ = self.create_client(SetBool, "reset_counter")

    def callback_number_count(self, msg):
        current = msg.data

        if (current >= 100):
            request = SetBool.Request()

            request.data = True
            future = self.client_.call_async(request)
            future.add_done_callback(self.callback_call_done)

    def callback_call_done(self, future):
        response = future.result()

        if (response.success):
          self.get_logger().info(response.message)
        else:
          self.get_logger().error(response.message)            
 
 
def main(args=None):
    rclpy.init(args=args)
    node = CounterReseterNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()