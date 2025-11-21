#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):

    def __init__(self):
        super().__init__('draw_circle_node')
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)  # Send command every 0.5 seconds
        self.get_logger().info('Draw Circle Node has been started.')

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0      # Forward velocity
        msg.angular.z = 1.0     # Angular velocity for circular motion
        self.cmd_vel_pub_.publish(msg)  # Publish the velocity command

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()