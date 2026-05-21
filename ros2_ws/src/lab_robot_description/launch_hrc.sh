#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
#  HRC Full Pipeline Launcher
#  Usage: ~/launch_hrc.sh
# ─────────────────────────────────────────────────────────────────────────────

# Terminal 1 — Gazebo Ignition + MoveIt + RViz
terminator -e "bash -c '
    cd ~/ros2_ws &&
    source /opt/ros/humble/setup.bash &&
    source ~/ros2_ws/install/setup.bash &&
    ros2 launch lab_robot_description lab_sim_moveit.launch.py;
    exec bash'" &

echo "Waiting 20 seconds for MoveIt to initialize..."
sleep 20

# Terminal 2 — Hand tracking (alias expanded to full command)
terminator -e "bash -c '
    source /opt/ros/humble/setup.bash &&
    cd ~/mediapipe &&
    ./bazel-bin/mediapipe/examples/hand_tracking_custom/hand_tracking;
    exec bash'" &

sleep 2

# Terminal 3 — Collision object publisher
terminator -e "bash -c '
    source /opt/ros/humble/setup.bash &&
    source ~/ros2_ws/install/setup.bash &&
    ros2 run human_robot_collab hand_to_collision;
    exec bash'" &

sleep 2

# Terminal 4 — Monitoring
terminator -e "bash -c '
    source /opt/ros/humble/setup.bash &&
    ros2 topic echo /planning_scene;
    exec bash'" &

echo "All terminals launched. Add PlanningScene in RViz if not already visible."
