# Human-Robot Collaboration — Industrial Robotics Project
## Vision-Based Safe Workspace Sharing using UR5e, ROS2, and Computer Vision

🚧 **Project Status: Work in Progress**

A vision-driven human-robot collaboration system where a UR5e robot detects a human worker's active workspace and avoids collision zones while autonomously operating in the remaining safe areas.

---

# Introduction

This project explores **Human-Robot Collaboration (HRC)** in an industrial robotics setting using **ROS2 Humble**, **computer vision**, and a **UR5e collaborative robot**. The goal is to enable safe interaction between a human worker and a robot operating within the same workspace.

In traditional industrial environments, robots are often separated from humans using physical safety cages. While effective for safety, these barriers reduce flexibility and efficiency. Modern **collaborative robotics systems (cobots)** aim to remove these barriers and allow robots to work alongside humans safely.

In this project, a vision system using an **Intel RealSense camera** monitors the workspace and detects the **human worker's hand using MediaPipe hand tracking**. A **padded rectangular region around the detected hand** is dynamically defined as a **prohibited workspace zone**. The robot interprets this region as unsafe and avoids planning motions within it.

By combining **computer vision, ROS2 robotic middleware, motion planning with MoveIt2, and simulation tools such as RViz and Gazebo**, the system enables the robot to perform tasks only in areas that are not currently occupied by the human worker.

This demonstrates an intelligent **vision-based safety layer for collaborative robotics**, allowing robots to adapt to human activity in real time.

Keywords:  
Human-Robot Collaboration, ROS2, Industrial Robotics, Computer Vision, UR5e, MoveIt2, MediaPipe, Robot Safety, Collaborative Robots.

---

# Current Progress

## ✅ Completed

### C++ MediaPipe Hand Tracking Node (Bazel)
- Implemented real-time C++ hand tracking node using **MediaPipe HandLandmarkTrackingCpu** graph
- Achieves **30 FPS** in good lighting conditions on a laptop webcam
- Publishes hand bounding box as `geometry_msgs/PolygonStamped` on `/hand_bbox` topic
- Bounding box contains 4 corner points (top-left → top-right → bottom-right → bottom-left) in pixel coordinates
- Built using **Bazel 7.4.1** with **Clang 14** linked against **ROS2 Humble** libraries on Ubuntu 22.04
- Custom graph config (`hand_tracking_custom.pbtxt`) exposes the landmarks output stream

### Pipeline So Far
```
Webcam → MediaPipe (C++) → /hand_bbox topic (2D pixel coordinates)
```

## 🔄 In Progress

- `bbox_to_3d_node` — subscribes to `/hand_bbox` + RealSense depth image, back-projects pixel coordinates to 3D world coordinates using TF2, publishes MoveIt2 collision object on `/collision_object`
- `motion_planner_node` — subscribes to `/collision_object`, adds hand as obstacle in MoveIt2 planning scene, stops robot, replans around hand, executes new trajectory

## ⏳ Pending (requires lab access)

- RealSense camera integration and TF2 frame calibration
- MoveIt2 planning group verification on real UR5e
- Full end-to-end testing on physical robot

---

# System Demonstration

*(A system architecture diagram or demo video will be added here in the future.)*

---

# High-Level System Workflow

1. The **Intel RealSense camera** monitors the shared workspace.
2. The **C++ MediaPipe hand tracking node** detects the human hand at 30 FPS.
3. A **padded bounding box around the hand** is published to `/hand_bbox` as 2D pixel coordinates.
4. The **bbox_to_3d_node** uses RealSense depth data to convert the 2D bbox to a 3D collision object in the robot world frame.
5. The **motion_planner_node** adds the collision object to the MoveIt2 planning scene.
6. MoveIt2 replans the robot trajectory to avoid the hand.
7. The UR5e executes the new safe trajectory.

---

# Full Pipeline Architecture
```
RealSense Camera (RGB)
        |
        v
C++ MediaPipe Hand Tracking Node (Bazel)
        |
        v
/hand_bbox topic (2D pixel bounding box)
        |
RealSense Depth Image ──────────────────┐
                                        v
                            bbox_to_3d_node (ROS2)
                                        |
                                        v
                            /collision_object topic (3D MoveIt2 obstacle)
                                        |
                                        v
                            motion_planner_node (ROS2 + MoveIt2)
                                        |
                                        v
                                UR5e Robot Arm
```

---

# User Instructions

## System Requirements

### Hardware

- UR5e Industrial Robot Arm
- Intel RealSense Camera (mounted fixed to environment at ~45° downward angle)
- Robot gripper or end effector
- Workstation or laptop running Ubuntu

### Software

- Ubuntu 22.04
- ROS2 Humble
- Bazel 7.4.1
- Clang 14
- OpenCV 4.5.4
- MediaPipe (C++)
- MoveIt2
- RViz
- Gazebo

---

# Installation

**1. Clone the repository:**
```bash
git clone https://github.com/Ahmedhazemm29/Human-Robot-Collaboration---Industrial-Robotics-Project.git
cd Human-Robot-Collaboration---Industrial-Robotics-Project
git checkout master
```

**2. Install dependencies:**
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Bazel
sudo apt install bazel-7.4.1

# Clang
sudo apt install clang-14

# OpenCV
sudo apt install libopencv-dev
```

**3. Download required MediaPipe hand tracking models:**
```bash
python3 -c "
import urllib.request
models = [
    ('https://storage.googleapis.com/mediapipe-assets/hand_landmark_lite.tflite',
     'mediapipe/modules/hand_landmark/hand_landmark_lite.tflite'),
    ('https://storage.googleapis.com/mediapipe-assets/hand_landmark_full.tflite',
     'mediapipe/modules/hand_landmark/hand_landmark_full.tflite'),
    ('https://storage.googleapis.com/mediapipe-assets/palm_detection_lite.tflite',
     'mediapipe/modules/palm_detection/palm_detection_lite.tflite'),
    ('https://storage.googleapis.com/mediapipe-assets/palm_detection_full.tflite',
     'mediapipe/modules/palm_detection/palm_detection_full.tflite'),
]
for url, path in models:
    print(f'Downloading {path}...')
    urllib.request.urlretrieve(url, path)
    print('Done')
"
```

**4. Set up terminal aliases for convenience:**
```bash
echo "alias handtrack='source /opt/ros/humble/setup.bash && cd ~/mediapipe && ./bazel-bin/mediapipe/examples/hand_tracking_custom/hand_tracking'" >> ~/.bashrc

echo "alias builtrack='cd ~/mediapipe && bazel-7.4.1 build -c opt --define=xnn_enable_avx512fp16=false --define=xnn_enable_avxvnniint8=false --action_env=CC=clang --action_env=CXX=clang++ --action_env=CPLUS_INCLUDE_PATH=/usr/include/opencv4:\$(find /opt/ros/humble/include -maxdepth 1 -type d | tr \"\n\" \":\" | sed \"s/:\$//\") --action_env=LIBRARY_PATH=/opt/ros/humble/lib --action_env=LD_LIBRARY_PATH=/opt/ros/humble/lib //mediapipe/examples/hand_tracking_custom:hand_tracking'" >> ~/.bashrc

source ~/.bashrc
```

**5. Build the hand tracking node:**
```bash
builtrack
```

---

# Running the System

## Quick Commands

| Command | Description |
|---|---|
| `builtrack` | Rebuild the hand tracking node after code changes |
| `handtrack` | Run the hand tracking node |

## Step by Step

Run the hand tracking node:
```bash
handtrack
```

Verify the bounding box is being published (in a second terminal):
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /hand_bbox
```

---

# Technical Architecture

The system is implemented using modular **ROS2 nodes**.

Intel RealSense Camera  
↓  
C++ MediaPipe Hand Tracking Node (Bazel + ROS2)  
↓  
/hand_bbox topic (2D pixel bounding box)  
↓  
bbox_to_3d_node (ROS2) + RealSense Depth  
↓  
/collision_object topic (3D MoveIt2 obstacle)  
↓  
motion_planner_node (ROS2 + MoveIt2)  
↓  
UR5e Robot Execution

---

# Programming Languages

This project uses **C++** as the primary language.

C++ is used for:

- Real-time hand tracking with MediaPipe (30 FPS)
- ROS2 node communication
- 3D coordinate transformation with TF2
- MoveIt2 motion planning and collision avoidance

---

# Simulation Tools

**RViz**

Used for visualization of robot motion, workspace geometry, and sensor data.

**Gazebo**

Used for simulating the robot, environment, and collaborative workspace interactions before deploying to the physical robot.

---

# Repository Structure
```
Human-Robot-Collaboration---Industrial-Robotics-Project/
├── mediapipe/
│   ├── examples/
│   │   └── hand_tracking_custom/
│   │       ├── hand_tracking.cpp       # Main C++ hand tracking + ROS2 publisher
│   │       └── BUILD                   # Bazel build file
│   └── graphs/
│       └── hand_tracking/
│           └── hand_tracking_custom.pbtxt  # Custom MediaPipe graph config
├── ros2_ws/                            # ROS2 colcon workspace (coming soon)
│   └── src/
│       ├── bbox_to_3d_node/            # 2D bbox → 3D collision object
│       └── motion_planner_node/        # MoveIt2 replanning node
├── .gitignore
└── README.md
```

---

# Future Improvements

Potential extensions include:

- Dynamic 3D workspace mapping
- Depth-based human detection
- Predictive human motion modeling
- Multi-object task allocation between human and robot
- Improved collision avoidance strategies

---

# Team

- Ahmed Hazem
- Mohab Khaled
- Ali Loay
- Maya Hossam
- Haya Ayman
- Habiba Gad
- Nour Ramy
- Nour Kamel

German International University  
Industrial Robotics Course

---

# Project Status

This project is currently under active development.
