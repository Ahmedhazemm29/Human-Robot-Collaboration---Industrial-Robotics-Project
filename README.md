# Human-Robot Collaboration — Industrial Robotics Project

## Vision-Based Safe Workspace Sharing using UR5e, ROS2, and Computer Vision

🚧 **Project Status: Core System Working — RealSense Integration Pending**

A vision-driven human-robot collaboration system where a UR5e robot detects a human worker's hand in real time, dynamically defines it as a forbidden zone in the MoveIt2 planning scene, and replans its trajectory to avoid it — all without physical safety barriers.

---

## Demo

> ⚠️ This demo shows an intermediate milestone — full RealSense integration and physical robot testing are the next steps.

[![Demo Video](https://img.youtube.com/vi/w9RsIq-Hb4k/0.jpg)](https://www.youtube.com/watch?v=w9RsIq-Hb4k)

*Collision avoidance pipeline running in Gazebo Ignition simulation with webcam mode. The green box tracks the human hand in real time — MoveIt2 replans around it automatically.*

---

## What This System Does

In traditional industrial environments, robots are separated from humans using physical safety cages. This project removes that barrier. A camera monitors the shared workspace, detects the human worker's hand using MediaPipe, and publishes a padded 3D collision box around it to MoveIt2 at 10Hz. The UR5e robot treats this box as a live obstacle and automatically replans around it in real time.

The result is a robot that can share a workspace with a human worker safely — stopping and replanning whenever the human's hand enters the robot's intended path, and resuming normal operation the moment the hand moves away.

---

## System Architecture

```
Webcam / RealSense Camera (RGB + Depth)
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
         UR5e Robot Arm
```

---

## Key Features

- **Real-time hand detection** at 30 FPS using a custom C++ MediaPipe node
- **Live collision avoidance** — MoveIt2 replans around the hand box at 10Hz
- **Atomic scene updates** — old box and new box swap in one message, no ghost boxes
- **Table-constrained collision box** — box is clamped to the table's physical boundaries
- **Dual mode operation** — WEBCAM_MODE for development, RealSense mode for deployment
- **Gazebo Ignition simulation** — full lab environment with inverted UR5e, table, and frame
- **Single-command launcher** — entire 4-terminal pipeline starts with one command

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
| Intel RealSense depth integration | 🔄 In Progress |
| Full end-to-end test on physical UR5e | ⏳ Pending lab access |

---

## Repository Structure

```
Human-Robot-Collaboration---Industrial-Robotics-Project/
├── README.md
├── .gitignore
├── launch_hrc.sh                     # Single command to launch full pipeline
├── mediapipe/                        # C++ hand tracking node (Bazel)
│   └── examples/hand_tracking_custom/
│       ├── hand_tracking.cpp         # MediaPipe + ROS2 publisher (30 FPS)
│       └── BUILD                     # Bazel build file
├── src/                              # ROS2 Python nodes
│   └── hand_to_collision.py          # Hand landmarks → MoveIt2 collision box
├── ros2/                             # Robot description and launch files
│   ├── lab_robot_sim.urdf.xacro      # Lab robot: inverted UR5e + table + frame
│   └── lab_sim_moveit.launch.py      # Gazebo Ignition + MoveIt2 launch file
├── docs/                             # Documentation and diagrams
└── third_party/                      # MediaPipe source dependencies
```

---

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Ignition Fortress (`ros-humble-ros-gz`)
- MoveIt2 (`ros-humble-moveit`)
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

### Running the Full Pipeline

**Single command (recommended):**
```bash
~/launch_hrc.sh
```

This opens 4 terminals automatically in sequence:
- Terminal 1 — Gazebo Ignition + MoveIt2 + RViz (waits 20s for full initialization)
- Terminal 2 — MediaPipe hand tracking
- Terminal 3 — Collision object publisher
- Terminal 4 — Planning scene monitor

**Or run terminals manually:**

```bash
# Terminal 1 — Simulation
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch lab_robot_description lab_sim_moveit.launch.py

# Terminal 2 — Hand tracking
handtrack

# Terminal 3 — Collision publisher
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run human_robot_collab hand_to_collision

# Terminal 4 — Monitor
source /opt/ros/humble/setup.bash
ros2 topic echo /planning_scene
```

Then in RViz: **Add → PlanningScene → OK** to see the live green collision box.

### Mode Switching

At the top of `src/hand_to_collision.py`:

```python
WEBCAM_MODE = True   # Development — uses webcam + fixed depth, no RealSense needed
WEBCAM_MODE = False  # Deployment  — uses RealSense depth stream + TF transform
```

When using RealSense, connect the camera to a **USB 3.0 port** (blue ports) before launching, then run:
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
  depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30
```

---

## Useful Commands

| Command | Description |
|---|---|
| `~/launch_hrc.sh` | Launch full pipeline with one command |
| `handtrack` | Run hand tracking node |
| `builtrack` | Rebuild hand tracking node after changes |
| `ros2 topic echo /planning_scene` | Monitor live collision box updates |
| `ros2 control list_hardware_interfaces` | Verify robot controllers are active |

---

## Technical Stack

| Layer | Technology |
|---|---|
| Hand detection | MediaPipe HandLandmarkTrackingCpu (C++, 30 FPS) |
| Robot middleware | ROS2 Humble |
| Motion planning | MoveIt2 |
| Simulation | Gazebo Ignition Fortress |
| Visualization | RViz2 |
| Depth sensing | Intel RealSense D400 series |
| Robot | Universal Robots UR5e (6-DOF, 5kg payload) |
| Build system | Bazel 7.4.1 (C++), colcon (Python/ROS2) |

---

## Future Work

- RealSense depth integration and TF2 frame calibration
- Full end-to-end testing on physical UR5e
- Sorting task logic — robot autonomously sorts objects around detected hand zones
- Predictive human motion modeling
- Multi-hand tracking support

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

German International University — Industrial Robotics Course
