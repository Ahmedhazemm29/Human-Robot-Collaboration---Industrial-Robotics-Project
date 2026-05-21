#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.counter = 0
        self.create_timer(1.0, self.timer_callback)     #This will call the timer_callback function every 1 second  

    def timer_callback(self):
        self.get_logger().info("Hello" + str(self.counter))
        self.counter += 1 #This will increment the counter variable by 1 every time the timer_callback function is called, so the output will be "Hello0", "Hello1", "Hello2", and so on.

def main (args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)        #This will keep the node alive until it is killed, Also without this line the node will not be able to call the timer_callback function because the node will be destroyed immediately after the main function ends.
    rclpy.shutdown()

if  __name__=='__main__':
    main()