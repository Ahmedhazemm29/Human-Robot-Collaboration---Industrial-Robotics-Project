from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tree_path = os.path.join(
        get_package_share_directory('bt_action_server'),
        'trees', 'bt_action.xml'
    )

    return LaunchDescription([
        Node(
            package='bt_action_server',
            executable='bt_action',
            name='bt_action',
            output='screen',
            parameters=[{'tree_xml_file': tree_path}],
        )
    ])
