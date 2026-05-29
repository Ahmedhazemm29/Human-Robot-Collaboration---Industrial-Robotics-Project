#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
#  HRC Safety Sub-Terminals
#  Opens 4 terminals: Kinect driver, MediaPipe hand tracking,
#  collision object publisher, and planning_scene monitor.
#  Called from launch_sim.sh (Terminal 3) or manually after MoveIt is up.
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

# Sub-terminal 1 — Kinect driver
terminator -e "bash -c '
    source /opt/ros/humble/setup.bash &&
    source ~/ros2_ws/install/setup.bash &&
    ros2 run kinect_ros2 kinect_ros2_node;
    exec bash'" &
sleep 3

# Sub-terminal 2 — MediaPipe hand tracking
terminator -e "bash -c '
    source /opt/ros/humble/setup.bash &&
    source ~/ros2_ws/install/setup.bash &&
    cd ~/mediapipe &&
    ./bazel-bin/mediapipe/examples/hand_tracking_custom/hand_tracking;
    exec bash'" &
sleep 2

# Sub-terminal 3 — Collision object publisher (hand OBB → /planning_scene)
terminator -e "bash -c '
    source /opt/ros/humble/setup.bash &&
    source ~/ros2_ws/install/setup.bash &&
    ros2 run human_robot_collab hand_to_collision;
    exec bash'" &
sleep 2

# Sub-terminal 4 — Live /planning_scene monitor
terminator -e "bash -c '
    source /opt/ros/humble/setup.bash &&
    ros2 topic echo /planning_scene;
    exec bash'" &

echo "[HRC] 4 sub-terminals launched (Kinect, hand tracking, collision pub, monitor)."
