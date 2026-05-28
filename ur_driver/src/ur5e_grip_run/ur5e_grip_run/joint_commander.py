import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotiq_2f_urcap_adapter.action import GripperCommand
import time

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander_node')
        
        self.arm_pub = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10)

        self._gripper_client = ActionClient(self, GripperCommand, '/robotiq_2f_urcap_adapter/gripper_command')

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self.get_logger().info('Joint Commander Node is Online')
        self.run_sequence()

    def send_gripper(self, pos):
        """ pos: 0.08 for open, 0.0 for closed """
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper server not available!')
            return
            
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = pos
        # FIX: Explicitly setting speed and effort to valid values
        goal_msg.command.max_speed = 0.1  # Must be between 0.02 and 0.15
        goal_msg.command.max_effort = 50.0 # Standard gripping force
        
        self.get_logger().info(f'Sending Gripper to {pos} with speed 0.1')
        self._gripper_client.send_goal_async(goal_msg)
        time.sleep(1.5)

    def move_arm(self, positions, seconds=4.0):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(seconds)
        msg.points = [point]
        self.arm_pub.publish(msg)
        time.sleep(seconds + 0.5)

    def run_sequence(self):
        print("\n--- SAFETY: Keep hand on E-Stop ---")
        input("Press ENTER to start the loop...")

        # Update these with your real joint values from 'ros2 topic echo /joint_states'
        home_pose = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        point_1   = [0.2, -1.3, 1.4, -1.57, -1.57, 0.0]
        point_2   = [0.4, -1.1, 1.2, -1.57, -1.57, 0.0]
        point_3   = [0.6, -1.4, 1.5, -1.57, -1.57, 0.0]

        try:
            while rclpy.ok():
                self.get_logger().info('Loop Starting...')
                
                # Step 0: Home and Open
                self.move_arm(home_pose)
                self.send_gripper(0.025)

                # Step 1: Point 1 and Close
                self.move_arm(point_1)
                self.send_gripper(0.0)
                
                # Step 2: Point 2 and Open
                self.move_arm(point_2)
                self.send_gripper(0.085)

                # Step 3: Point 3 and Close
                self.move_arm(point_3)
                self.send_gripper(0.045)

        except KeyboardInterrupt:
            self.get_logger().info('Shutdown requested.')

def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
