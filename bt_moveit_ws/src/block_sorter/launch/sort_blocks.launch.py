from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='block_sorter',
            executable='sort_blocks',
            name='block_sorter',
            output='screen',
            # The node uses MoveGroupInterface, which loads robot_description
            # and the SRDF from parameters published by your move_group node.
            # Run your usual ur_moveit launch first, then launch this.
        ),
    ])
