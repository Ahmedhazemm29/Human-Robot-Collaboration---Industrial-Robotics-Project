from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess

def generate_launch_description():
    # Read robot_description from robot_state_publisher
    rd = subprocess.run(
        ["ros2", "param", "get", "/robot_state_publisher", "robot_description"],
        capture_output=True, text=True, timeout=10
    ).stdout.replace("String value is: ", "", 1).strip()

    # Read robot_description_semantic from move_group
    srdf = subprocess.run(
        ["ros2", "param", "get", "/move_group", "robot_description_semantic"],
        capture_output=True, text=True, timeout=10
    ).stdout.replace("String value is: ", "", 1).strip()

    # Fix robot name mismatch between URDF and SRDF
    #srdf = srdf.replace('robot name="ur"', 'robot name="ur5e_workcell"')

    return LaunchDescription([
        Node(
            package="bt_action_server",
            executable="reach_location_server",
            name="reach_location_action_server",
            parameters=[{
                "robot_description":          rd,
                "robot_description_semantic": srdf,
                "use_sim_time":               False,
                "robot_description_kinematics.ur_manipulator.kinematics_solver":
                    "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "robot_description_kinematics.ur_manipulator.kinematics_solver_attempts":
                    3,
                "robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution":
                    0.005,
                "robot_description_kinematics.ur_manipulator.kinematics_solver_timeout":
                    0.1,
            }],
            output="screen",
        )
    ])
