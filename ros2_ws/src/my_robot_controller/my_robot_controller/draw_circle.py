#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist     # Import the Twist message type for controlling the robot's velocity


class DrawCircle(Node):

    def __init__(self):
        super().__init__('draw_circle')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Create a publisher for the cmd_vel topic, 10 msgs queue size
        self.timer = self.create_timer(0.5, self.send_velocity_command)  # Create a timer that calls the timer_callback function every 0.1 seconds
        self.get_logger().info('Drawing a circle...')  # Log message to indicate that the node has started

    def send_velocity_command(self):
        msg = Twist()  # Create a new Twist message
        msg.linear.x = 2.0  # Set the linear velocity in the x direction
        msg.angular.z = 1.0  # Set the angular velocity around the z axis
        self.cmd_vel_pub.publish(msg)  # Publish the message to the cmd_vel topic

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircle()
    rclpy.spin(node)
    rclpy.shutdown()    