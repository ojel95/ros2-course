#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool
 
 
class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")

        self.count_ = 0
        self.subscriber_ = self.create_subscription(Int64, "number", self.increase_counter, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        self.get_logger().info("The number counter has been started.")

    def increase_counter(self, msg):
        self.count_ += msg.data
        self.get_logger().info(str(self.count_))
        
        self.publish_counter()

    def publish_counter(self):
        msg = Int64()
        msg.data = self.count_
        
        self.publisher_.publish(msg)
    
    def callback_reset_counter(self, request, response):
        response.success = False
        response.message = "Received reset request in false. Counter was no reseted."
        
        if (request.data == True):
            self.get_logger().info("Resetting the counter.")
            self.count_ = 0
            response.success = True
            response.message = "The counter was succesfully reseted."

        return response
        
 
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()