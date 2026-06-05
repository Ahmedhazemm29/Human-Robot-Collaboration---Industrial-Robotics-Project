# Human-Robot Collaboration — Industrial Robots Project

## Vision-Based Safe Workspace Sharing using UR5e, ROS2, and Computer Vision

✅ **Project Complete — Presented at GIU Engineering Exhibition 2026**

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
    reach_location_server (MoveIt2 Action Server)
    - Ingests hand OBB from /planning_scene
    - Tracks hand velocity (6-sample rolling window)
    - Predictive FK collision checker (runs every 100ms)
    - Lookahead: checks 1.5s ahead of current trajectory
    - Caution zone: slows to 8% speed when hand within 0.25m
    - Brakes + generates safe-point candidates on predicted collision
    - HOLD_AND_WAIT only when hand actively blocks planning
              |
              v
    BehaviorTree.CPP v4 (bt_action_server)
    - ReachLocation action server
    - Continuous loop: Home → (Pick + Open gripper in parallel) → Close gripper → Drop → Open gripper → repeat
    - Self-managed recovery (no allowReplanning)
              |
              v
         UR5e Robot Arm
```

---

## Key Features

- **Real-time hand detection** at 30 FPS using a custom C++ MediaPipe node
- **Predictive FK collision checking** — every 100ms, checks the next 1.5 seconds of trajectory waypoints using direct in-process Forward Kinematics (no service call, ~10× faster than RPC approach). Each waypoint's hand position is predicted using estimated hand velocity
- **Hand velocity prediction** — 6-sample rolling deque tracks hand motion; predicts where the hand will be when the robot reaches each upcoming waypoint
- **Mid-motion brake and recovery** — robot decelerates smoothly using a linear velocity ramp, then scores and moves to the best safe-point candidate; only holds if the hand is actively blocking the replanned path
- **Caution zone** — when the hand enters 0.25m from the end-effector, velocity drops to 8% before a full collision event
- **Live collision avoidance** — MoveIt2 (Pilz + OMPL + FCL) plans around the hand OBB; Pilz PTP for all joint moves, Pilz LIN for straight-line pick approach, OMPL as fallback
- **Atomic scene updates** — old box and new box swap in one message, no ghost boxes
- **Behavior Tree pipeline** — BehaviorTree.CPP v4 executes waypoints with a self-managed state machine (PLAN → EXECUTE → BRAKE → SAFE_POINT → RESUME)
- **Single-command launchers** — `~/launch_sim.sh` (simulation, 5 terminals) and `~/launch_hw.sh` (hardware, 6 terminals including Robotiq gripper adapter)

---

## Motion Planning Details

| Decision | Choice | Reason |
|---|---|---|
| IK solver | LMA (Levenberg-Marquardt) | Fast, robust near UR5e singularities; no extra dependencies |
| OMPL planner | RRTConnect (bidirectional) | Fallback only; Pilz PTP is tried first for all joint moves |
| Normal path | Pilz PTP (joint-space) | Deterministic smooth joint move; OMPL kept as fallback |
| Cartesian path | Pilz LIN | Straight-line EEF path (pick approach only); falls back to PTP if infeasible |
| Collision checking (planning) | FCL via MoveIt2 planning scene | Full mesh-level collision for all OMPL samples |
| Collision checking (execution) | Direct FK + OBB distance | In-process, microseconds per check, velocity-predicted hand position |

### Velocity Scaling

| Scenario | Velocity | Acceleration |
|---|---|---|
| Normal execution | 25% | 25% |
| Hand within 0.25m of EE | 8% | 8% |
| Moving to safe-point (post-brake) | 15% | 15% |

---

## Safety State Machine

```
PLAN_TO_GOAL ──► EXECUTE_TO_GOAL ──► DONE_OK
     ▲                  │ collision_predicted_
     │                  ▼
     │               BRAKE  (linear velocity ramp, 6 waypoints)
     │                  │
     │         zero safe candidates
     ├──────────────────┘
     │
     │          MOVE_TO_SAFE_POINT
     │                  │
     │         all candidates exhausted
     ├──────────────────┘
     │
     │          AT_SAFE_POINT ──► EXECUTE_TO_GOAL
     │
     └── (planning fails AND hand detected) ──► HOLD_AND_WAIT
```

HOLD_AND_WAIT is only entered when OMPL cannot find a path **and** the hand is currently detected in the scene — meaning the hand is physically blocking all available paths. OMPL timeouts alone do not trigger a hold.

---

## Simulation Environment

The simulation runs using the **fake UR5e hardware interface** (no Gazebo) with MoveIt2 + RViz:

- UR5e mounted **inverted** on a ceiling frame (matching real lab configuration)
- Ceiling-mounted **Kinect camera** for hand tracking
- Aluminium frame structure (2.0m × 1.5m × 1.88m)
- Work table (1.4m × 0.7m × 0.71m) centered below the robot
- Full MoveIt2 integration with `joint_state_broadcaster` and `joint_trajectory_controller`
- RViz with live PlanningScene display showing the green hand collision box and full TF frame tree
- Trajectory ghost display (one-shot, no loop replay)

---

## Proven Results

- ✅ Hand detected in real time via Kinect + MediaPipe at 30 FPS
- ✅ Collision box published to MoveIt2 at 10Hz — appears as green box in RViz
- ✅ Robot **stops mid-motion** and shifts to a safe position when hand blocks the path
- ✅ Robot **autonomously re-navigates** to the original target the moment the hand is removed
- ✅ Predictive FK checker detects approach before contact using velocity extrapolation
- ✅ Consistent on/off behaviour confirmed across multiple test runs
- ✅ BT action server plans and executes across 3 waypoints (Home, Pick, Place)
- ✅ Robot returns to home position autonomously after each waypoint
- ✅ Full Kinect depth integration with TF transform to robot base frame
- ✅ Full end-to-end validation on physical UR5e hardware

---

## Project Status

| Component | Status |
|---|---|
| C++ MediaPipe hand tracking node | ✅ Complete |
| hand_to_collision.py ROS2 node | ✅ Complete |
| Fake-hardware simulation (MoveIt2 + RViz) | ✅ Complete |
| MoveIt2 collision avoidance in simulation | ✅ Complete |
| Webcam testing mode | ✅ Complete |
| Kinect depth integration | ✅ Complete |
| Mid-motion brake & safe-point recovery | ✅ Complete |
| Predictive FK collision checker with velocity prediction | ✅ Complete |
| BT 3-waypoint pipeline (Home / Pick / Place) | ✅ Complete |
| Single-command pipeline launcher (`launch_sim.sh`) | ✅ Complete |
| Full end-to-end test on physical UR5e | ✅ Complete |

---

## Repository Structure

```
~/hrc_repo/
├── hrc_perception_ws/
│   └── src/
│       ├── human_robot_collab/        # hand_to_collision.py — publishes hand box to /planning_scene
│       │                              # validate_corners.py, waypoint_manager.py
│       ├── kinect_ros2/               # Kinect ROS2 driver node
│       ├── actions_py/                # Python action server/client examples
│       └── cmake_pkg/                 # CMake package template
│
├── bt_moveit_ws/
│   └── src/
│       ├── bt_action_server/          # BehaviorTree.CPP v4 pipeline
│       │   ├── src/
│       │   │   ├── reach_location_server.cpp   # MoveIt2 action server — predictive FK collision,
│       │   │   │                               #   velocity-predicted hand OBB, safe-point recovery
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
├── ur5e_driver_ws/
│   └── src/
│       ├── Universal_Robots_ROS2_Driver/        # Modified UR driver
│       │   └── ur_moveit_config/
│       │       └── rviz/view_robot.rviz         # RViz config (Loop Animation off, 0.20s ghost step)
│       ├── Universal_Robots_ROS2_Description/   # Modified UR description (lab URDF, ceiling mount, Kinect)
│       ├── ur5e_grip_run/                       # Python joint commander
│       └── ur5e_grip_run_cpp/                   # C++ joint commander with IK
│
├── launch_sim.sh                      # One-command launcher: opens all 5 terminals in boot order (simulation)
├── launch_hw.sh                       # One-command launcher: opens all 6 terminals in boot order (real UR5e + gripper)
├── launch_hrc.sh                      # HRC safety sub-terminals (Kinect, MediaPipe, collision pub, monitor)
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
- MoveIt2 (`ros-humble-moveit`)
- BehaviorTree.CPP v4 (`ros-humble-behaviortree-cpp`)
- Bazel 7.4.1
- MediaPipe C++ (built in `~/mediapipe`)
- UR ROS2 packages (`ros-humble-ur`)

### Installation

**1. Clone the repository:**
```bash
git clone https://github.com/Ahmedhazemm29/Human-Robot-Collaboration---Industrial-Robotics-Project.git ~/hrc_repo
```

**2. Install all dependencies listed above**

**3. Build the hand tracking node:**
```bash
builtrack
```

**4. Build the UR driver workspace (includes modified UR packages):**
```bash
cd ~/ur5e_driver_ws
colcon build --allow-overriding ur_moveit_config ur_controllers ur_description
source install/setup.bash
```

**5. Build the ROS2 workspace:**
```bash
cd ~/hrc_perception_ws
colcon build
source install/setup.bash
```

**6. Build the BT workspace:**
```bash
cd ~/bt_moveit_ws
source ~/ur5e_driver_ws/install/setup.bash
colcon build
source install/setup.bash
```

---

## Running the Full Pipeline — Simulation

### Option A — Single command (recommended)

```bash
~/launch_sim.sh
```

This opens all 5 terminals automatically in the correct boot order, with appropriate delays between each:

| Terminal | What it runs | Wait |
|---|---|---|
| T1 | Fake UR5e driver (builds + launches `ur_control.launch.py`) | 10s |
| T2 | MoveIt2 + RViz (builds + launches `ur_moveit.launch.py`) | 10s |
| T3 | HRC safety sub-terminals via `launch_hrc.sh` | 5s |
| T4 | ReachLocation action server | 3s |
| T5 | BT executor | — |

> ⚠️ T2 takes the longest. Watch its terminal for `"You can start planning now!"` — T3/T4/T5 will already be launching by then due to the fixed delays, which is fine.

---

### Option B — Manual (5 terminals)

Open 5 terminals in order. Wait for each step to fully initialize before moving to the next.

#### Terminal 1 — Fake UR5e Driver

```bash
cd ~/ur5e_driver_ws
colcon build --allow-overriding ur_moveit_config ur_controllers ur_description
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.102 \
  kinematics_params_file:="/home/hazem/ur5e_driver_ws/src/Universal_Robots_ROS2_Description/config/ur5e/default_kinematics.yaml" \
  use_fake_hardware:=true
```

#### Terminal 2 — MoveIt2 + RViz

```bash
cd ~/ur5e_driver_ws
colcon build --allow-overriding ur_moveit_config ur_controllers ur_description
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.102 \
  name:="ur5e"
```

> ⚠️ Wait for `"You can start planning now!"` before launching the next terminals.

#### Terminal 3 — HRC Safety

```bash
~/launch_hrc.sh
```

This opens 4 sub-terminals:

| Sub-terminal | What it runs |
|---|---|
| 1 | Kinect driver (`kinect_ros2_node`) |
| 2 | MediaPipe hand tracking node |
| 3 | Collision object publisher (`hand_to_collision`) |
| 4 | Live `/planning_scene` topic monitor |

#### Terminal 4 — Action Server

```bash
source ~/.ros_env.sh
ros2 run bt_action_server reach_location_server --ros-args \
  -p use_sim_time:=true \
  --params-file /home/hazem/ur5e_driver_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml
```

#### Terminal 5 — BT Executor

```bash
source ~/.ros_env.sh
ros2 launch bt_action_server bt_action.launch.py
```

---

## Running the Full Pipeline — Real Robot

### Option A — Single command (recommended)

```bash
~/launch_hw.sh
```

Opens 6 terminals automatically. Differences from simulation:

| Terminal | Change vs simulation |
|---|---|
| T1 | `use_fake_hardware:=false` — connects to real UR5e at 192.168.1.102 |
| T4 | `use_sim_time:=false` — uses wall clock, not `/clock` topic |
| T6 | **New** — Robotiq 2F-85 gripper adapter (`robotiq_2f85_urcap_adapter_launch.py`) |

> ⚠️ Ensure the robot is at home position and the E-stop is cleared before running.

### Option B — Manual

Follow the same terminal steps as simulation with these changes:

**Terminal 1** — set `use_fake_hardware:=false`:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.102 \
  kinematics_params_file:="/home/hazem/ur5e_driver_ws/src/Universal_Robots_ROS2_Description/config/ur5e/default_kinematics.yaml" \
  use_fake_hardware:=false
```

**Terminal 4** — set `use_sim_time:=false`:
```bash
source ~/.ros_env.sh
ros2 run bt_action_server reach_location_server --ros-args \
  -p use_sim_time:=false \
  --params-file /home/hazem/ur5e_driver_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml
```

**Terminal 6** — Robotiq gripper adapter (new, required for hardware):
```bash
source ~/.ros_env.sh
ros2 launch robotiq_2f_urcap_adapter robotiq_2f85_urcap_adapter_launch.py \
  robot_ip:=192.168.1.102
```

> ⚠️ Launch T6 before T5 (BT executor). The BT `Gripper` node will wait up to 5 s for the action server, but it must be running before the first gripper command arrives.

---

## Mode Switching

At the top of `hrc_perception_ws/src/human_robot_collab/human_robot_collab/hand_to_collision.py`:

```python
WEBCAM_MODE = True   # Development — uses webcam + fixed depth
WEBCAM_MODE = False  # Deployment  — uses Kinect depth stream + TF transform
```

---

## Useful Commands

| Command | Description |
|---|---|
| `~/launch_sim.sh` | Launch full simulation pipeline with one command (all 5 terminals) |
| `~/launch_hrc.sh` | Launch HRC safety sub-terminals only (Kinect, hand tracking, collision pub, monitor) |
| `handtrack` | Run MediaPipe hand tracking node |
| `builtrack` | Rebuild MediaPipe hand tracking node after changes |
| `ros2 topic echo /planning_scene` | Monitor live collision box updates |
| `ros2 control list_hardware_interfaces` | Verify robot controllers are active |
| `ros2 pkg executables bt_action_server` | List all BT executable names |
| `ros2 launch bt_action_server reach_server.launch.py` | Launch the ReachLocation action server |
| `ros2 launch bt_action_server bt_action.launch.py` | Launch the BT client with tree XML |
| `ros2 run tf2_ros tf2_echo base_link tool0` | Check robot end-effector position and orientation |

---

## Technical Stack

| Layer | Technology | Detail |
|---|---|---|
| Hand detection | MediaPipe HandLandmarkTrackingCpu (C++) | 30 FPS, publishes bounding box |
| Robot middleware | ROS2 Humble | DDS-based pub/sub, actions, services |
| Motion planning (paths) | MoveIt2 + Pilz PTP / OMPL fallback | Deterministic PTP primary; RRTConnect fallback |
| Motion planning (straight-line) | Pilz LIN | Straight-line EEF motion; pick approach only; falls back to PTP |
| IK solver | LMA kinematics plugin | Levenberg-Marquardt, fast convergence, robust near singularities |
| Collision checking (planning) | FCL via MoveIt2 | Full mesh-level, every OMPL sample |
| Collision checking (execution) | Direct FK + OBB distance | In-process, no RPC, velocity-predicted |
| Behavior Trees | BehaviorTree.CPP v4 | XML-defined tree, ReachLocation action |
| Visualization | RViz2 | Planning scene, trajectory ghost (one-shot) |
| Depth sensing | Kinect | RGB-D, TF-transformed to robot frame |
| Robot | Universal Robots UR5e | 6-DOF, 5kg payload, ceiling-mounted |
| Build system | Bazel 7.4.1 (C++), colcon (ROS2) | |

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

**Supervised by:**
- Dr. Khaled Tolba
- Eng. Moaz Fouda
- Eng. Ahmed El-Hemaly

**German International University — Industrial Robots Course**

*Presented at GIU Engineering Exhibition 2026*
