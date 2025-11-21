#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class myNode(Node): 

    def __init__(self, name):
        super().__init__(name) # Calling the inherited class's ctor
        self.get_logger().info("Hello from ROS2!")
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello")

def main(args=None):
    rclpy.init(args=args) # Start ros2 communications 

    node = myNode("first_node")
    rclpy.spin(node)

    rclpy.shutdown() # Shut down ros2 communications


if __name__ == '__main__':
    main()