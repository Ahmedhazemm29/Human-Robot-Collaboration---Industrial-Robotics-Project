# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# ============================================================================
# lab_real_moveit.launch.py
#
# Launches the physical UR5e with:
#   - UR ROS2 driver  (ros2_control + controller_manager + joint_state_broadcaster
#                      + scaled_joint_trajectory_controller)
#   - ur_moveit_config (MoveIt2 + RViz)
#
# Usage:
#   ros2 launch lab_robot_description lab_real_moveit.launch.py robot_ip:=<ROBOT_IP>
#
# Prerequisites:
#   - Robot is powered on and reachable at <robot_ip>
#   - Teach pendant is set to REMOTE CONTROL mode
#   - ur_robot_driver is installed:  sudo apt install ros-humble-ur-robot-driver
#   - ur_moveit_config  is installed: sudo apt install ros-humble-ur-moveit-config
# ============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


# ---------------------------------------------------------------------------
# launch_setup — resolved after argument evaluation
# ---------------------------------------------------------------------------
def launch_setup(context, *args, **kwargs):

    # ------------------------------------------------------------------
    # Resolved launch arguments
    # ------------------------------------------------------------------
    robot_ip                 = LaunchConfiguration("robot_ip")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz              = LaunchConfiguration("launch_rviz")

    # ------------------------------------------------------------------
    # 1. UR5e hardware driver
    #
    #    Internally starts:
    #      • ros2_control_node          (controller_manager)
    #      • robot_state_publisher      (TF from /joint_states)
    #      • joint_state_broadcaster    (publishes /joint_states)
    #      • scaled_joint_trajectory_controller  (accepts MoveIt goals)
    #      • ur_ros2_driver / io_and_status_controller
    #
    #    NOTE: Do NOT spawn joint_state_broadcaster or the trajectory
    #    controller separately — the driver already handles them.
    # ------------------------------------------------------------------
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_robot_driver"), "/launch/ur5e.launch.py"]
        ),
        launch_arguments={
            # ---- connection ----
            "robot_ip":         robot_ip,
            # ---- hardware mode ----
            "use_fake_hardware":         "false",   # real robot
            "fake_sensor_commands":      "false",   # no fake sensors
            # ---- controller selection ----
            # scaled_joint_trajectory_controller honours the robot's speed
            # scaling slider — always prefer this over the plain variant on
            # a real robot.
            "initial_joint_controller":   initial_joint_controller,
            "activate_joint_controller":  activate_joint_controller,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. MoveIt2  (move_group + RViz via ur_moveit_config)
    #
    #    Passes our custom description package so MoveIt builds its
    #    planning scene from the same URDF/SRDF the driver uses.
    #
    #    use_sim_time = false  → wall-clock, not /clock
    # ------------------------------------------------------------------
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type":             "ur5e",
            # Point MoveIt at your custom description package.
            # Make sure lab_robot_sim.urdf.xacro does NOT hardcode
            # sim_ignition:=true — add a conditional guard or create
            # a separate lab_robot_real.urdf.xacro if needed.
            "description_package": "lab_robot_description",
            "description_file":    "lab_robot_urdf_latest.xacro",
            # Real hardware → wall-clock time
            "use_sim_time":        "false",
            "launch_rviz":         launch_rviz,
        }.items(),
    )

    return [
        ur_driver,
        moveit_launch,
    ]


# ---------------------------------------------------------------------------
# generate_launch_description — declare args then defer to launch_setup
# ---------------------------------------------------------------------------
def generate_launch_description():

    declared_arguments = [

        # ------------------------------------------------------------------
        # REQUIRED
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            "robot_ip",
            description=(
                "IP address of the UR5e controller box. "
                "Example: 192.168.1.102"
            ),
        ),

        # ------------------------------------------------------------------
        # Controller selection  (keep default unless you have a reason to
        # switch — scaled variant is always preferred on real hardware)
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Controller loaded at startup.",
            choices=[
                "scaled_joint_trajectory_controller",   # ← default / recommended
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate the selected joint controller at startup.",
        ),

        # ------------------------------------------------------------------
        # Optional
        # ------------------------------------------------------------------
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with the MoveIt MotionPlanning panel.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
