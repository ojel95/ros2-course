#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
 
 
class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")

        self.count_ = 0
        self.subscriber_ = self.create_subscription(Int64, "number", self.increase_counter, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.get_logger().info("The number counter has been started.")

    def increase_counter(self, msg):
        self.count_ += msg.data
        print(f"New counter value: {self.count_}")
        
        self.publish_counter()

    def publish_counter(self):
        msg = Int64()
        msg.data = self.count_
        
        self.publisher_.publish(msg)
        
 
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()