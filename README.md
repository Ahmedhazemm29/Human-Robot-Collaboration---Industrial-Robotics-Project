# Human-Robot Collaboration — Industrial Robotics Project

## Vision-Based Safe Workspace Sharing using UR5e, ROS2, and Computer Vision

🚧 **Project Status: Full Pipeline Working in Simulation | Physical UR5e End-to-End Testing In Progress**

A vision-driven human-robot collaboration system where a UR5e robot detects a human worker's hand in real time, dynamically defines it as a forbidden zone in the MoveIt2 planning scene, and replans its trajectory to avoid it — all without physical safety barriers. The system also includes a Behavior Tree-based pipeline for autonomous task execution with live human-aware collision avoidance.

---

## Demos

### Demo 1 — Full HRC Pipeline: BT Waypoint Execution with Real-Time Hand Avoidance (Simulation)

[![Full Pipeline Demo](https://img.youtube.com/vi/pzDRlCJcUsM/0.jpg)](https://youtu.be/pzDRlCJcUsM)

*The complete HRC pipeline running end-to-end in simulation. The robot executes a Behavior Tree navigating across 3 hardcoded waypoints — **Home**, **Pick**, and **Place** positions. When a hand enters the workspace mid-motion, the robot immediately stops, shifts to a safe position, and autonomously re-navigates back to whichever target it was originally heading to once the hand is cleared. Also showcases the updated URDF with full TF frame visualization, ceiling-mounted Kinect camera, and the complete lab environment setup.*

---

### Demo 2 — BT-Driven UR5e: Autonomous Navigation & Hand Collision Avoidance

[![BT Demo Video](https://img.youtube.com/vi/Y2fY03Ji3fM/0.jpg)](https://youtube.com/shorts/Y2fY03Ji3fM)

*Demonstration of a BehaviorTree.CPP v4 pipeline controlling a UR5e robot arm. The Behavior Tree sends the robot to a target waypoint and returns it to the home position automatically. If a human hand is detected in the planned path during replanning, the robot holds position and waits — only executing once the path is clear. Validated in Gazebo Ignition simulation and on the physical UR5e.*

---

### Demo 3 — HRC Collision Avoidance Pipeline (Early Milestone)

[![HRC Demo Video](https://img.youtube.com/vi/w9RsIq-Hb4k/0.jpg)](https://www.youtube.com/watch?v=w9RsIq-Hb4k)

*Early intermediate milestone — collision avoidance pipeline running in Gazebo Ignition simulation with webcam mode. The green box tracks the human hand in real time, MoveIt2 replans around it automatically.*

---

## What This System Does

In traditional industrial environments, robots are separated from humans using physical safety cages. This project removes that barrier. A Kinect camera monitors the shared workspace, detects the human worker's hand using MediaPipe, and publishes a padded 3D collision box around it to MoveIt2 at 10Hz. The UR5e robot treats this box as a live obstacle and automatically replans around it in real time.

The result is a robot that can share a workspace with a human worker safely — stopping mid-motion the moment a hand enters its planned path, shifting to a safe position, and autonomously resuming its task once the workspace is clear. A second pipeline built on BehaviorTree.CPP v4 handles autonomous waypoint execution (Home → Pick → Place), with dynamic replanning triggered by the live hand collision box.

---

## System Architecture

```
Kinect Camera (RGB + Depth)
              |
              v
  C++ MediaPipe Hand Tracking Node
  (30 FPS, publishes hand bounding box)
              |
              v
        /hand_bbox topic
              |
              v
   hand_to_collision.py (ROS2, 10Hz)
   - Builds padded 3D bounding box
   - Deprojects to 3D using Kinect intrinsics
   - TF transform to robot base_link frame
   - Atomic remove+add (no ghost boxes)
              |
              v
      /planning_scene topic
              |
              v
    MoveIt2 Motion Planner
    - Treats hand box as obstacle
    - Stops mid-motion if hand detected
    - Replans when path is clear
              |
              v
    BehaviorTree.CPP v4 (bt_action_server)
    - ReachLocation action server
    - 3 waypoints: Home, Pick, Place
    - allowReplanning(true) for live avoidance
              |
              v
         UR5e Robot Arm
```

---

## Key Features

- **Real-time hand detection** at 30 FPS using a custom C++ MediaPipe node
- **Mid-motion stop and replan** — robot stops the moment a hand is detected, moves to a safe position, then autonomously re-navigates to the original target
- **Live collision avoidance** — MoveIt2 replans around the hand box at 10Hz
- **Atomic scene updates** — old box and new box swap in one message, no ghost boxes
- **Table-constrained collision box** — box is clamped to the table's physical boundaries
- **Dual mode operation** — WEBCAM_MODE for development, Kinect mode for deployment
- **Behavior Tree pipeline** — BehaviorTree.CPP v4 executes 3 hardcoded waypoints (Home, Pick, Place) with dynamic replanning
- **Gazebo Ignition simulation** — full lab environment with ceiling-mounted UR5e, table, Kinect, and TF frames
- **Single-command launcher** — entire HRC pipeline starts with one command

---

## Simulation Environment

The simulation runs in **Gazebo Ignition (Fortress)** with a custom lab description matching the real physical setup:

- UR5e mounted **inverted** on a ceiling frame (matching real lab configuration)
- Ceiling-mounted **Kinect camera** for hand tracking
- Aluminium frame structure (2.0m × 1.5m × 1.88m)
- Work table (1.4m × 0.7m × 0.71m) centered below the robot
- Full MoveIt2 integration with `joint_state_broadcaster` and `joint_trajectory_controller`
- RViz with live PlanningScene display showing the green hand collision box and full TF frame tree

---

## Proven Results

- ✅ Hand detected in real time via Kinect + MediaPipe at 30 FPS
- ✅ Collision box published to MoveIt2 at 10Hz — appears as green box in RViz
- ✅ Robot **stops mid-motion** and shifts to a safe position when hand blocks the path
- ✅ Robot **autonomously re-navigates** to the original target the moment the hand is removed
- ✅ Consistent on/off behaviour confirmed across multiple test runs
- ✅ BT action server plans and executes across 3 waypoints (Home, Pick, Place)
- ✅ Robot returns to home position autonomously after each waypoint
- ✅ Robot replans around hand obstacle mid-execution (`allowReplanning(true)`)
- ✅ Full Kinect depth integration with TF transform to robot base frame
- 🔄 Physical UR5e end-to-end testing — in progress

---

## Project Status

| Component | Status |
|---|---|
| C++ MediaPipe hand tracking node | ✅ Complete |
| hand_to_collision.py ROS2 node | ✅ Complete |
| Gazebo Ignition simulation (lab description) | ✅ Complete |
| MoveIt2 collision avoidance in simulation | ✅ Complete |
| Webcam testing mode | ✅ Complete |
| Kinect depth integration | ✅ Complete |
| Mid-motion stop & autonomous replan | ✅ Complete |
| BT 3-waypoint pipeline (Home / Pick / Place) | ✅ Complete |
| Single-command pipeline launcher | ✅ Complete |
| Full end-to-end test on physical UR5e | 🔄 In Progress |

---

## Repository Structure

```
~/hrc_repo/
├── ros2_ws/
│   └── src/
│       ├── human_robot_collab/        # hand_to_collision.py — publishes hand box to /planning_scene
│       │                              # validate_corners.py, waypoint_manager.py
│       ├── kinect_ros2/               # Kinect ROS2 driver node
│       ├── actions_py/                # Python action server/client examples
│       └── cmake_pkg/                 # CMake package template
│
├── book_ws_modified/
│   └── src/
│       ├── bt_action_server/          # BehaviorTree.CPP v4 pipeline
│       │   ├── src/
│       │   │   ├── reach_location_server.cpp   # MoveIt2 action server with mid-motion stop
│       │   │   ├── bt_action.cpp               # BT client — ticks the tree
│       │   │   └── reach_location_client.cpp   # Manual action client
│       │   ├── trees/
│       │   │   └── bt_action.xml               # 3-waypoint Behavior Tree (Home/Pick/Place)
│       │   ├── action/
│       │   │   └── ReachLocation.action        # Custom action definition
│       │   └── launch/
│       │       ├── reach_server.launch.py
│       │       └── bt_action.launch.py
│       └── block_sorter/              # Sorting task package
│           ├── src/sort_blocks.cpp
│           └── launch/sort_blocks.launch.py
│
├── ur_driver/
│   └── src/
│       ├── Universal_Robots_ROS2_Driver/        # Modified UR driver
│       ├── Universal_Robots_ROS2_Description/   # Modified UR description (lab URDF, ceiling mount, Kinect)
│       ├── ur5e_grip_run/                       # Python joint commander
│       └── ur5e_grip_run_cpp/                   # C++ joint commander with IK
│
├── launch_hrc.sh                      # Single-command launcher for the full HRC pipeline
└── README.md
```

---

## Dependencies — Not Included in This Repo

The following packages must be installed separately before building. Note that `Universal_Robots_ROS2_Driver` and `Universal_Robots_ROS2_Description` ARE included in this repo because they have been modified for this project — install the others fresh from their official sources.

| Package | GitHub |
|---|---|
| `Universal_Robots_Client_Library` | [github.com/UniversalRobots/Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) |
| `Universal_Robots_ROS2_GZ_Simulation` | [github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation) |
| `robotiq_description` | [github.com/PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) |
| `robotiq_2f_urcap_adapter` | [github.com/PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) |
| `MediaPipe C++` | [github.com/google-ai-edge/mediapipe](https://github.com/google-ai-edge/mediapipe) |

> ⚠️ MediaPipe must be built from source using Bazel 7.4.1 and placed at `~/mediapipe`. Follow the official MediaPipe C++ build instructions from the repo above.

---

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Ignition Fortress (`ros-humble-ros-gz`)
- MoveIt2 (`ros-humble-moveit`)
- BehaviorTree.CPP v4 (`ros-humble-behaviortree-cpp`)
- Bazel 7.4.1
- MediaPipe C++ (built in `~/mediapipe`)
- UR ROS2 packages (`ros-humble-ur`)

### Installation

**1. Clone the repository:**
```bash
git clone https://github.com/Ahmedhazemm29/Human-Robot-Collaboration---Industrial-Robotics-Project.git
```

**2. Install all dependencies listed above**

**3. Build the hand tracking node:**
```bash
builtrack
```

**4. Build the UR driver workspace (includes modified UR packages):**
```bash
cd ~/ur_driver
colcon build --allow-overriding ur_moveit_config ur_controllers ur_description
source install/setup.bash
```

**5. Build the ROS2 workspace:**
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

**6. Build the BT workspace:**
```bash
cd ~/book_ws_modified
source ~/ur_driver/install/setup.bash
colcon build
source install/setup.bash
```

---

## Running the Full Pipeline — Simulation (5 Terminals)

Open 5 terminals in order. Wait for each step to fully initialize before moving to the next.

---

### Terminal 1 — Fake UR5e Driver

```bash
cd ~/ur_driver
colcon build --allow-overriding ur_moveit_config ur_controllers ur_description
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.102 \
  kinematics_params_file:="/home/hazem/ur_driver/src/Universal_Robots_ROS2_Description/config/ur5e/default_kinematics.yaml" \
  use_fake_hardware:=true
```

---

### Terminal 2 — MoveIt

```bash
cd ~/ur_driver
colcon build --allow-overriding ur_moveit_config ur_controllers ur_description
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.102 \
  name:="ur5e"
```

> ⚠️ Wait for `"You can start planning now!"` before launching the next terminals.

---

### Terminal 3 — HRC Safety

```bash
cd ~/ros2_ws
source install/setup.bash
~/launch_hrc.sh
```

This automatically opens sub-terminals for:

| Sub-terminal | What it runs |
|---|---|
| A | Kinect driver |
| B | MediaPipe hand tracking node |
| C | Collision object publisher (`hand_to_collision`) |
| D | Planning scene monitor |

---

### Terminal 4 — Action Server

```bash
source ~/.ros_env.sh
ros2 run bt_action_server reach_location_server --ros-args \
  -p use_sim_time:=true \
  --params-file /home/hazem/ur_driver/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml
```

---

### Terminal 5 — BT Executor

```bash
source ~/.ros_env.sh
ros2 launch bt_action_server bt_action.launch.py
```

---

## Running the Full Pipeline — Real Robot (5 Terminals)

Follow the same steps as simulation with the changes below.

---

### Terminal 1 — UR5e Driver

```bash
cd ~/ur_driver
colcon build --allow-overriding ur_moveit_config ur_controllers ur_description
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.102 \
  kinematics_params_file:="/home/hazem/ur_driver/src/Universal_Robots_ROS2_Description/config/ur5e/default_kinematics.yaml" \
  use_fake_hardware:=false
```

---

### Terminal 2 — MoveIt

```bash
cd ~/ur_driver
colcon build --allow-overriding ur_moveit_config ur_controllers ur_description
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.102 \
  name:="ur5e"
```

> ⚠️ Wait for `"You can start planning now!"` before launching the next terminals.

---

### Terminal 3 — HRC Safety

```bash
cd ~/ros2_ws
source install/setup.bash
~/launch_hrc.sh
```

---

### Terminal 4 — Action Server

```bash
source ~/.ros_env.sh
ros2 launch bt_action_server reach_server.launch.py
```

---

### Terminal 5 — BT Executor

```bash
source ~/.ros_env.sh
ros2 launch bt_action_server bt_action.launch.py
```

> ⚠️ Ensure the robot is at home position and the workspace is clear before launching terminals 4 and 5.

---

## Mode Switching

At the top of `ros2_ws/src/human_robot_collab/human_robot_collab/hand_to_collision.py`:

```python
WEBCAM_MODE = True   # Development — uses webcam + fixed depth
WEBCAM_MODE = False  # Deployment  — uses Kinect depth stream + TF transform
```

---

## Useful Commands

| Command | Description |
|---|---|
| `~/launch_hrc.sh` | Launch full HRC pipeline with one command |
| `handtrack` | Run MediaPipe hand tracking node |
| `builtrack` | Rebuild MediaPipe hand tracking node after changes |
| `ros2 topic echo /planning_scene` | Monitor live collision box updates |
| `ros2 control list_hardware_interfaces` | Verify robot controllers are active |
| `ros2 pkg executables bt_action_server` | List all BT executable names |
| `ros2 launch bt_action_server reach_server.launch.py` | Launch the ReachLocation action server |
| `ros2 launch bt_action_server bt_action.launch.py` | Launch the BT client with tree XML |
| `ros2 run tf2_ros tf2_echo base_link tool0` | Check robot end-effector position and orientation (use to define waypoints in `bt_action.xml`) |

---

## Technical Stack

| Layer | Technology |
|---|---|
| Hand detection | MediaPipe HandLandmarkTrackingCpu (C++, 30 FPS) |
| Robot middleware | ROS2 Humble |
| Motion planning | MoveIt2 |
| Behavior Trees | BehaviorTree.CPP v4 |
| Simulation | Gazebo Ignition Fortress |
| Visualization | RViz2 |
| Depth sensing | Kinect |
| Robot | Universal Robots UR5e (6-DOF, 5kg payload) |
| Build system | Bazel 7.4.1 (C++), colcon (Python/ROS2) |

---

## Future Work

- Full end-to-end deployment on physical UR5e hardware


---

## Team

- Ahmed Hazem
- Mohab Khaled
- Ali Loay
- Maya Hossam
- Haya Ayman
- Habiba Gad
- Nour Ramy
- Nour Kamel

**German International University — Industrial Robotics Course**
