# Human-Robot Collaboration — Industrial Robotics Project

## Vision-Based Safe Workspace Sharing using UR5e, ROS2, and Computer Vision

🚧 **Project Status: Core System Working — Kinect Integration In Progress | Physical UR5e Partially Tested**

A vision-driven human-robot collaboration system where a UR5e robot detects a human worker's hand in real time, dynamically defines it as a forbidden zone in the MoveIt2 planning scene, and replans its trajectory to avoid it — all without physical safety barriers. The system also includes a Behavior Tree-based pipeline for autonomous task execution with live human-aware collision avoidance.

---

## Demos

### Demo 1 — HRC Collision Avoidance Pipeline

> ⚠️ This demo shows an intermediate milestone — full Kinect integration and complete physical robot testing are the next steps.

[![HRC Demo Video](https://img.youtube.com/vi/w9RsIq-Hb4k/0.jpg)](https://www.youtube.com/watch?v=w9RsIq-Hb4k)

*Collision avoidance pipeline running in Gazebo Ignition simulation with webcam mode. The green box tracks the human hand in real time — MoveIt2 replans around it automatically.*

---

### Demo 2 — BT-Driven UR5e: Autonomous Navigation & Hand Collision Avoidance

[![BT Demo Video](https://img.youtube.com/vi/Y2fY03Ji3fM/0.jpg)](https://youtube.com/shorts/Y2fY03Ji3fM)

*Demonstration of a BehaviorTree.CPP v4 pipeline controlling a UR5e robot arm. The Behavior Tree sends the robot to a target waypoint and returns it to the home position automatically. If a human hand is detected in the planned path during replanning, the robot holds position and waits — only executing once the path is clear. Validated in Gazebo Ignition simulation and on the physical UR5e. Part of a vision-based Human-Robot Collaboration project at German International University.*

---

## What This System Does

In traditional industrial environments, robots are separated from humans using physical safety cages. This project removes that barrier. A camera monitors the shared workspace, detects the human worker's hand using MediaPipe, and publishes a padded 3D collision box around it to MoveIt2 at 10Hz. The UR5e robot treats this box as a live obstacle and automatically replans around it in real time.

The result is a robot that can share a workspace with a human worker safely — holding position and waiting whenever the human's hand enters the robot's intended path, and resuming normal operation the moment the hand moves away. Crucially, this avoidance happens at **planning time** — the robot will not attempt to move at all if the path is blocked, which is the safest possible behaviour.

A second pipeline built on BehaviorTree.CPP v4 handles autonomous waypoint execution, with dynamic replanning triggered by the live hand collision box.

---

## System Architecture

```
Webcam / Kinect Camera (RGB + Depth)
              |
              v
  C++ MediaPipe Hand Tracking Node
  (30 FPS, publishes 21 landmarks)
              |
              v
     /hand_landmarks/hand_0
              |
              v
   hand_to_collision.py (ROS2, 10Hz)
   - Builds padded 3D bounding box
   - Constrained to table surface
   - Atomic remove+add (no ghost boxes)
              |
              v
      /planning_scene topic
              |
              v
    MoveIt2 Motion Planner
    - Treats hand box as obstacle
    - Holds position if path blocked
    - Replans when path is clear
              |
              v
    BehaviorTree.CPP v4 (bt_action_server)
    - ReachLocation action server
    - BT client ticks the tree
    - allowReplanning(true) for live avoidance
              |
              v
         UR5e Robot Arm
```

---

## Key Features

- **Real-time hand detection** at 30 FPS using a custom C++ MediaPipe node
- **Planning-time collision avoidance** — robot holds position if hand blocks the path, moves only when clear
- **Live collision avoidance** — MoveIt2 replans around the hand box at 10Hz
- **Atomic scene updates** — old box and new box swap in one message, no ghost boxes
- **Table-constrained collision box** — box is clamped to the table's physical boundaries
- **Dual mode operation** — WEBCAM_MODE for development, Kinect mode for deployment
- **Behavior Tree pipeline** — BehaviorTree.CPP v4 waypoint execution with dynamic replanning
- **Gazebo Ignition simulation** — full lab environment with inverted UR5e, table, and frame
- **Single-command launcher** — entire HRC pipeline starts with one command

---

## Simulation Environment

The simulation runs in **Gazebo Ignition (Fortress)** with a custom lab description matching the real physical setup:

- UR5e mounted **inverted** on a ceiling frame (matching real lab configuration)
- Aluminium frame structure (2.0m × 1.5m × 1.88m)
- Work table (1.4m × 0.7m × 0.71m) centered below the robot
- Full MoveIt2 integration with `joint_state_broadcaster` and `joint_trajectory_controller`
- RViz with live PlanningScene display showing the green hand collision box

---

## Proven Results

- ✅ Hand detected in real time via webcam + MediaPipe at 30 FPS
- ✅ Collision box published to MoveIt2 at 10Hz — appears as green box in RViz
- ✅ Robot **holds position and waits** when hand box blocks the planned path
- ✅ Robot **plans and executes** the moment the hand is removed
- ✅ Consistent on/off behaviour confirmed across multiple test runs
- ✅ BT action server plans and executes to target waypoints
- ✅ Robot returns to home position autonomously after each waypoint
- ✅ Robot replans around hand obstacle mid-execution (`allowReplanning(true)`)
- 🔄 Physical UR5e end-to-end testing — partially complete

---

## Project Status

| Component | Status |
|---|---|
| C++ MediaPipe hand tracking node | ✅ Complete |
| hand_to_collision.py ROS2 node | ✅ Complete |
| Gazebo Ignition simulation (lab description) | ✅ Complete |
| MoveIt2 collision avoidance in simulation | ✅ Complete |
| Webcam testing mode | ✅ Complete |
| Single-command pipeline launcher | ✅ Complete |
| BT pick-and-place pipeline (bt_action_server) | ✅ Complete |
| Kinect depth integration | 🔄 In Progress |
| Full end-to-end test on physical UR5e | 🔄 Partially Working |

---

## Repository Structure

```
~/
├── ros2_ws/
│   └── src/
│       ├── human_robot_collab/       # hand_to_collision.py — publishes hand box to /planning_scene
│       ├── lab_robot_description/    # Custom URDF/xacro: inverted UR5e + table + frame + launch files
│       ├── kinect_ros2/              # Kinect ROS2 driver node
│       ├── actions_py/               # Python action server/client examples
│       ├── cmake_pkg/                # CMake package template
│       ├── my_robot_controller/      # Robot controller nodes
│       └── my_robot_interfaces/      # Custom action/service definitions
│
├── book_ws/
│   └── src/
│       └── bt_action_server/         # BehaviorTree.CPP v4 pipeline
│           ├── src/
│           │   ├── reach_location_server.cpp   # MoveIt2 action server
│           │   ├── bt_action.cpp               # BT client — ticks the tree
│           │   └── reach_location_client.cpp   # Manual action client
│           ├── trees/
│           │   └── bt_action.xml               # Behavior Tree XML definition
│           ├── action/
│           │   └── ReachLocation.action         # Custom action definition
│           └── launch/
│               ├── reach_server.launch.py
│               └── bt_action.launch.py
│
└── launch_hrc.sh                     # Single-command launcher for the full HRC pipeline
```

---

## Dependencies — Not Included in This Repo

The following packages must be installed separately before building:

| Package | GitHub |
|---|---|
| `Universal_Robots_ROS2_Driver` | [github.com/UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) |
| `Universal_Robots_ROS2_Description` | [github.com/UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) |
| `Universal_Robots_Client_Library` | [github.com/UniversalRobots/Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) |
| `Universal_Robots_ROS2_GZ_Simulation` | [github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation) |
| `robotiq_description` | [github.com/PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) |
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

**4. Build the ROS2 workspace:**
```bash
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

**5. Build the BT workspace:**
```bash
cd ~/book_ws
colcon build
source ~/book_ws/install/setup.bash
```

---

## Running the Full Pipeline

### Step 1 — Real Robot Setup (Skip for Simulation)

Launch the UR5e robot driver before anything else:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.102 \
  kinematics_params_file:="/home/hazem/ir_project/src/Universal_Robots_ROS2_Driver/ur_calibration/config/my_robot_calibration.yaml"
```

---

### Step 2 — Launch the HRC Stack

```bash
~/launch_hrc.sh
```

This automatically opens 5 terminals in sequence:

| Terminal | What it runs |
|---|---|
| 1 | Gazebo Ignition + MoveIt2 + RViz (waits 20s for full init) |
| 2 | Kinect driver |
| 3 | MediaPipe hand tracking node |
| 4 | Collision object publisher (`hand_to_collision`) |
| 5 | Planning scene monitor (`ros2 topic echo /planning_scene`) |

Once RViz is open: **Add → PlanningScene → OK** to see the live green collision box.

---

### Step 3 — Launch the BT Pipeline

Run these **after** the HRC stack is fully up:

```bash
# Terminal 6 — Action server
cd ~/book_ws && source install/setup.bash
ros2 launch bt_action_server reach_server.launch.py

# Terminal 7 — BT client
cd ~/book_ws && source install/setup.bash
ros2 launch bt_action_server bt_action.launch.py
```

> If running the BT client manually without the launch file, pass the tree path explicitly:
> ```bash
> ros2 run bt_action_server bt_action --ros-args \
>   -p tree_xml_file:=/home/hazem/book_ws/src/bt_action_server/trees/bt_action.xml
> ```

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

- Complete Kinect depth integration and TF2 frame calibration
- Sorting task logic — robot autonomously sorts objects around detected hand zones
- Predictive human motion modeling
- BT robustness — add `RetryNode`/`FallbackNode` error recovery

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
