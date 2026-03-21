from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):

    # Build robot description — no simulation_controllers arg passed to xacro
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("lab_robot_description"),
                "urdf", "lab_robot_sim.urdf.xacro"
            ]),
            " name:=ur ur_type:=ur5e sim_ignition:=true",
        ]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # Publishes TF transforms for all joints
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # Ignition Gazebo (with GUI)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

    # Spawn the robot into Ignition from the robot_description topic
    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "ur", "-allow_renaming", "true"],
    )

    # Bridge /clock so ROS nodes use sim time
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # Controllers yaml provides joint names and control interfaces
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("lab_robot_description"), "config", "ur_controllers.yaml"
    ])

    # Broadcasts joint states to /joint_states
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Accepts trajectory goals from MoveIt
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # MoveIt — uses ur_moveit_config but with our description
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type":             "ur5e",
            "description_package": "lab_robot_description",
            "description_file":    "lab_robot_sim.urdf.xacro",
            "use_sim_time":        "true",
            "launch_rviz":         "true",
        }.items(),
    )

    return [
        robot_state_publisher,
        gz_sim,
        gz_spawn,
        gz_bridge,
        joint_state_broadcaster,
        joint_trajectory_controller,
        moveit_launch,
    ]


def generate_launch_description():
    from launch.actions import OpaqueFunction
    from launch import LaunchDescription
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
