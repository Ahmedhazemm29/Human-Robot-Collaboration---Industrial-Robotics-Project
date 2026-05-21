#!/usr/bin/env python
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil                                     


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__('count_until_server')
        self.count_until_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            execute_callback=self.execute_callback)
        self.get_logger().info('CountUntil Action Server has been started.')
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        
        # Execute the action
        self.get_logger().info(f'Executing goal: target_number={target_number}, period={period}')
        counter = 0
        for i in range(target_number):
            counter += 1
            self.get_logger().info(f'Current count: {counter}')
            time.sleep(period)  # Simulate work by sleeping for the specified period
        
        # Once we are done, set goal final state
        goal_handle.succeed()
        
        # and send the result
        result = CountUntil.Result()
        result.reached_number = counter
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()