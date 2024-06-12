#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial

from my_robot_interfaces.srv import SetLedState
 
 
class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")

        self.battery_state_ = False
        self.discharge_timer_ = None
        self.charge_timer_ = self.create_timer(6, self.on_full_battery)
        self.client_ = self.create_client(SetLedState, "set_led")

        self.get_logger().info("The battery node has started.")

    def on_full_battery(self):
        self.destroy_timer(self.charge_timer_)
        self.battery_state_ = True
        self.get_logger().info("Battery is full. Setting led state.")
        self.call_set_led_state()
        self.discharge_timer_ = self.create_timer(4, self.on_empty_battery)

    def on_empty_battery(self):
        self.destroy_timer(self.discharge_timer_)
        self.battery_state_ = False
        self.get_logger().info("Battery is empty. Setting led state.")
        self.call_set_led_state()
        self.charge_timer_ = self.create_timer(6, self.on_full_battery)

    def call_set_led_state(self):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server set_led")

        request = SetLedState.Request()
        request.led_number = 2
        request.state = self.battery_state_

        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_done, led_number=request.led_number, state=request.state))

    def callback_call_done(self, future, led_number, state):
        try:
            response = future.result()
            self.get_logger().info("Led: " + str(led_number) + ", state: " + str(state) + ", success: " + str(response.success))
        except Exception as e:
            self.get_logger().error("Service call error: %r" % (e,))

 
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()