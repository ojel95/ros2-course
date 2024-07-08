#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLedState
from my_robot_interfaces.msg import LedPanelState
 
 
class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_states", [False, False, False])

        self.led_panel_ = self.get_parameter("led_states").value
        self.server_ = self.create_service(SetLedState, "set_led", self.set_led_callback)
        self.led_states_publisher_ = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.get_logger().info("The led panel node has started.")

    def set_led_callback(self, request, response):
        led_number = request.led_number
        state = request.state

        if (led_number < len(self.led_panel_) and led_number >= 0):
            self.led_panel_[request.led_number] = state
            response.success = True
            self.get_logger().info("Setted led " + str(led_number) + ": " + str(state))
        else:
            response.success = False
            self.get_logger().error("Led panel could not be set. Invalid led number")

        self.publish_panel_state()
        
        return response
    
    def publish_panel_state(self):
        message = LedPanelState()
        message.led_states = self.led_panel_

        self.get_logger().info("Publishing led panel state")
        
        self.led_states_publisher_.publish(message)

 
def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()