#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
#  Full HRC Hardware Launcher  (real UR5e at 192.168.1.102)
#  Usage:  ~/launch_hw.sh
#
#  Opens 6 terminals in the correct boot order:
#    T1  Real UR5e driver        ← robot must be powered on and E-stop cleared
#    T2  MoveIt + RViz           ← wait for "You can start planning now!"
#    T3  HRC safety (5 sub-terminals via launch_hrc.sh)
#    T4  Reach location action server
#    T5  BT executor
#    T6  Robotiq gripper adapter ← BT Gripper node commands it
# ─────────────────────────────────────────────────────────────────────────────

# Strip VS Code snap vars that corrupt GTK/GIO library loading in new terminals
if [[ -n "${SNAP_NAME}" ]]; then
    for _v in GIO_MODULE_DIR GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE \
               GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GSETTINGS_SCHEMA_DIR \
               XDG_DATA_HOME LOCPATH SNAP_LIBRARY_PATH; do
        unset "$_v"
    done
    [[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG}" ]] && \
        export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG}"
fi

KINEMATICS_FILE="/home/hazem/ur_driver/src/Universal_Robots_ROS2_Description/config/ur5e/default_kinematics.yaml"
MOVEIT_KINEMATICS="/home/hazem/ur_driver/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml"

# ── T1: Real UR5e driver ─────────────────────────────────────────────────────
terminator -e "bash -c '
    cd ~/ur_driver &&
    colcon build --allow-overriding ur_moveit_config ur_controllers ur_description &&
    source /opt/ros/humble/setup.bash &&
    source install/setup.bash &&
    ros2 launch ur_robot_driver ur_control.launch.py \
        ur_type:=ur5e \
        robot_ip:=192.168.1.102 \
        kinematics_params_file:=${KINEMATICS_FILE} \
        use_fake_hardware:=false;
    exec bash'" &
echo "[T1] Real UR5e driver — waiting 15 s for build + robot handshake..."
sleep 15

# ── T2: MoveIt + RViz ────────────────────────────────────────────────────────
terminator -e "bash -c '
    cd ~/ur_driver &&
    colcon build --allow-overriding ur_moveit_config ur_controllers ur_description &&
    source /opt/ros/humble/setup.bash &&
    source install/setup.bash &&
    ros2 launch ur_moveit_config ur_moveit.launch.py \
        ur_type:=ur5e \
        robot_ip:=192.168.1.102 \
        name:=ur5e;
    exec bash'" &
echo "[T2] MoveIt — waiting 10 s (watch T2 for 'You can start planning now!')..."
sleep 10

# ── T3: HRC safety (Kinect + MediaPipe + collision pub + monitor) ─────────────
terminator -e "bash -c '
    cd ~/ros2_ws &&
    source install/setup.bash &&
    ~/launch_hrc.sh;
    exec bash'" &
echo "[T3] HRC safety sub-terminals — waiting 5 s..."
sleep 5

# ── T4: Reach location action server ────────────────────────────────────────
terminator -e "bash -c '
    source ~/.ros_env.sh &&
    ros2 run bt_action_server reach_location_server --ros-args \
        -p use_sim_time:=false \
        -p enable_waypoint_fallback:=false \
        -p caution_distance:=0.10 \
        --params-file ${MOVEIT_KINEMATICS};
    exec bash'" &
echo "[T4] Action server — waiting 3 s..."
sleep 3

# ── T5: BT executor ─────────────────────────────────────────────────────────
terminator -e "bash -c '
    source ~/.ros_env.sh &&
    ros2 launch bt_action_server bt_action.launch.py;
    exec bash'" &
sleep 2

# ── T6: Robotiq gripper adapter (BT Gripper node commands it) ────────────────
terminator -e "bash -c '
    source ~/.ros_env.sh &&
    ros2 launch robotiq_2f_urcap_adapter robotiq_2f85_urcap_adapter_launch.py \
        robot_ip:=192.168.1.102;
    exec bash'" &

echo ""
echo "All 6 terminals launched (HARDWARE mode — real UR5e)."
echo "  Caution zone : hand within 0.25 m → 8 % speed (vel_scale_caution=0.08)"
echo "  Normal speed : 25 % (vel_scale_normal=0.25)"
echo "  Ghost loop   : enabled, 0.20 s/step"
