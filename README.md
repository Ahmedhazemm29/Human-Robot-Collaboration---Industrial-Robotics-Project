# Human-Robot Collaboration — Industrial Robotics Project

## Vision-Based Safe Workspace Sharing using UR5e, ROS2, and Computer Vision

🚧 **Project Status: Core System Working — Kinect Integration In Progress | Physical UR5e Partially Tested**

A vision-driven human-robot collaboration system where a UR5e robot detects a human worker's hand in real time, dynamically defines it as a forbidden zone in the MoveIt2 planning scene, and replans its trajectory to avoid it — all without physical safety barriers. The system also includes a Behavior Tree-based pick-and-place pipeline for autonomous task execution.

---

## Demo

> ⚠️ This demo shows an intermediate milestone — full Kinect integration and complete physical robot testing are the next steps.

[![Demo Video](https://img.youtube.com/vi/w9RsIq-Hb4k/0.jpg)](https://www.youtube.com/watch?v=w9RsIq-Hb4k)

*Collision avoidance pipeline running in Gazebo Ignition simulation with webcam mode. The green box tracks the human hand in real time — MoveIt2 replans around it automatically.*

---

## What This System Does

In traditional industrial environments, robots are separated from humans using physical safety cages. This project removes that barrier. A camera monitors the shared workspace, detects the human worker's hand using MediaPipe, and publishes a padded 3D collision box around it to MoveIt2 at 10Hz. The UR5e robot treats this box as a live obstacle and automatically replans around it in real time.

The result is a robot that can share a workspace with a human worker safely — stopping and replanning whenever the human's hand enters the robot's intended path, and resuming normal operation the moment the hand moves away.

A second pipeline built on BehaviorTree.CPP v4 handles autonomous pick-and-place task execution, with dynamic replanning triggered by the hand collision box.

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
    - Replans trajectory around it
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
- **Live collision avoidance** — MoveIt2 replans around the hand box at 10Hz
- **Atomic scene updates** — old box and new box swap in one message, no ghost boxes
- **Table-constrained collision box** — box is clamped to the table's physical boundaries
- **Dual mode operation** — WEBCAM_MODE for development, Kinect mode for deployment
- **Behavior Tree pipeline** — BehaviorTree.CPP v4 pick-and-place with dynamic replanning
- **Gazebo Ignition simulation** — full lab environment with inverted UR5e, table, and frame
- **Single-command launcher** — entire pipeline starts with one command

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

The collision avoidance pipeline has been tested end-to-end in simulation:

- ✅ Hand detected in real time via webcam + MediaPipe at 30 FPS
- ✅ Collision box published to MoveIt2 at 10Hz — appears as green box in RViz
- ✅ Robot **fails to plan** when hand box is in the intended path
- ✅ Robot **plans successfully** the moment the hand is removed
- ✅ Consistent on/off behaviour confirmed across multiple test runs
- ✅ BT action server plans and executes to hardcoded waypoints
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
│       └── bt_action_server/         # BehaviorTree.CPP v4 pick-and-place pipeline
│           ├── src/
│           │   ├── reach_location_server.cpp   # MoveIt2 action server
│           │   ├── bt_action.cpp               # BT client — ticks the tree
│           │   └── reach_location_client.cpp   # Manual action client
│           ├── trees/
│           │   └── bt_action.xml               # Behavior Tree XML definition
│           ├── action/
│           │   └── ReachLocation.action        # Custom action definition
│           └── launch/
│               ├── reach_server.launch.py
│               └── bt_action.launch.py
│
└── launch_hrc.sh                     # Single-command launcher for the full HRC pipeline
```

> ⚠️ The following packages are **not included** in this repo — install them separately:
> - `Universal_Robots_ROS2_Driver`
> - `Universal_Robots_ROS2_Description`
> - `Universal_Robots_Client_Library`
> - `Universal_Robots_ROS2_Gazebo_Simulation`
> - `robotiq_description`

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
cd Human-Robot-Collaboration---Industrial-Robotics-Project
git checkout master
```

**2. Build the hand tracking node:**
```bash
builtrack
```

**3. Build the ROS2 workspace:**
```bash
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

**4. Build the BT workspace:**
```bash
cd ~/book_ws
colcon build
source ~/book_ws/install/setup.bash
```

---

## Running the Full Pipeline

### HRC Safety Stack

#### ⚙️ Real Robot Setup (Run First)

Before starting the pipeline, launch the UR5e robot driver in a separate terminal:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.102 \
  kinematics_params_file:="/home/hazem/ir_project/src/Universal_Robots_ROS2_Driver/ur_calibration/config/my_robot_calibration.yaml"
```

#### Single command (recommended)

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
| 5 | Planning scene monitor |

#### Manual (terminal by terminal)

```bash
# Terminal 1 — Simulation
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch lab_robot_description lab_sim_moveit.launch.py

# Terminal 2 — Hand tracking
handtrack

# Terminal 3 — Collision publisher
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run human_robot_collab hand_to_collision

# Terminal 4 — Monitor
source /opt/ros/humble/setup.bash
ros2 topic echo /planning_scene
```

Once RViz is open: **Add → PlanningScene → OK** to see the live green collision box.

---

### BT Pick-and-Place Pipeline

Run **after** the HRC stack is fully up:

```bash
# Terminal 6 — Action server
cd ~/book_ws && source install/setup.bash
ros2 launch bt_action_server reach_server.launch.py

# Terminal 7 — BT client
cd ~/book_ws && source install/setup.bash
ros2 launch bt_action_server bt_action.launch.py
```

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
| `handtrack` | Run hand tracking node |
| `builtrack` | Rebuild hand tracking node after changes |
| `ros2 topic echo /planning_scene` | Monitor live collision box updates |
| `ros2 control list_hardware_interfaces` | Verify robot controllers are active |
| `ros2 pkg executables bt_action_server` | List BT executable names |

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
- Full end-to-end testing on physical UR5e
- Sorting task logic — robot autonomously sorts objects around detected hand zones
- Predictive human motion modeling
- Multi-hand tracking support
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
