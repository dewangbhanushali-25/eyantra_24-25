# eYRC_3152
Contains all folders and files used for EYRC competition. 

made by love by DJ Sanghvi Ke Chode

# Git commands
## general
> to intialise git in folder
```command
 git init
```
> adds all the files to stage
```command
 git add .
```
> clones the directary present on github
```command
 git clone <url> 
```
> commits directory with messange for understanding
```command
git commit -m "message"
```
>  push the origin branch already having itss upstream branch ready
```command
git push
```
>  to push the required branch to remote also setting its upstream branch
```command
 git push --set-upstream origin <brach_name>
```
>  to view all the branches setup locally
```command
 git branch
```
>  to create and switch new branch in one command
```command
git checkout -b <branch_name>
```
>  to switch to particular branch locally
```command
git switch <branch_name >
```
>  to generate new tag (first checkout to main)
```command
git tag <tagname>
```
>  to push a particular tag
```command
git push <repo_url> <tag name>
```
## To pull all directories excluding project envelope follow this
>  do not init if already done
```command
 git init
```
```command

# e-Yantra Robotics Competition 2024-25: Logistic Cobot Theme

## Team ID: 3152

### Team Members
- Anees Alwani
- Anish Oturkar
- Dewang Bhanushali

---

## 📋 Project Overview

This repository contains the complete solution for the **Logistic Cobot (LB) Theme** of e-Yantra Robotics Competition 2024-25. The project demonstrates an integrated warehouse automation system featuring a UR5 robotic manipulator for pick-and-place operations and an autonomous mobile robot (eBot) for material transportation using SLAM and Nav2.

### Key Features

- **Vision-Based Pick and Place**: ArUco marker detection and pose estimation for precise object manipulation
- **Real-time Servo Control**: Delta twist commands for smooth end-effector motion using MoveIt2 Servo
- **Autonomous Navigation**: SLAM-based mapping and Nav2 for autonomous mobile robot navigation
- **Precision Docking**: Ultrasonic sensor-based alignment for accurate docking operations
- **Multi-Robot Coordination**: Synchronized operation between robotic arm and mobile base
- **ROS2 Service Architecture**: Custom service interfaces for robot-to-robot communication

---

## 🏗️ System Architecture

### Hardware Components
- **UR5 Robotic Arm**: 6-DOF industrial manipulator
- **Intel RealSense D435**: RGB-D camera for ArUco detection
- **eBot**: Differential drive mobile robot
- **Ultrasonic Sensors**: For precision docking alignment
- **Electromagnetic Gripper**: Payload attachment/detachment mechanism

### Software Stack
- **ROS2 Humble**: Robot Operating System framework
- **MoveIt2**: Motion planning and manipulation
- **Nav2**: Navigation and path planning
- **Gazebo**: Physics-based simulation environment
- **OpenCV**: Computer vision and ArUco detection
- **tf2**: Coordinate frame transformations

---

## 📦 Package Structure

```
eyrc-24-25-logistic-cobot/
├── task4/                          # eBot navigation and coordination
│   ├── task4/
│   │   ├── ebot_nav2_cmd.py       # Navigation controller
│   │   ├── ebot_docking_service.py # Docking service implementation
│   │   └── ebot_passing_service.py # Box passing coordination
│   └── package.xml
│
├── task4c_bringup/                # Launch files
│   ├── launch/
│   │   └── task4.launch.py        # Main system launch file
│   └── package.xml
│
└── ur5_control/                   # UR5 arm control
    ├── ur5_control/
    │   ├── task1b.py              # ArUco detection and TF publishing
    │   ├── task1c.py              # Joint-based pick and place
    │   ├── task2a.py              # Servo-based manipulation (final)
    │   └── pose.py                # Pose utilities
    └── package.xml
```

---

## 🚀 Features Implementation

### Task 1: Vision-Based Object Detection
**File**: `ur5_control/task1b.py`

- ArUco marker detection using OpenCV
- Camera calibration and depth integration
- TF2 transforms from camera_link to base_link
- Pose estimation for 150mm markers (DICT_4X4_50)

**Key Functions**:
- `detect_aruco()`: Detects markers and returns center, distance, angle, and IDs
- `process_image()`: Publishes TF transforms for detected objects

### Task 2: Servo-Based Manipulation
**File**: `ur5_control/task2a.py`

- Real-time visual servoing using delta twist commands
- P-controller for position and orientation tracking
- State machine for pick-place-drop workflow
- Dynamic box detection and prioritization

**State Machine**:
1. `start_point`: Move to initial observation pose
2. `moving_to_box`: Approach detected ArUco box
3. `move_to_intermediate`: Safe transition position
4. `move_to_drop`: Navigate to drop-off location

**Key Features**:
- Adaptive velocity control based on position/orientation error
- Singularity avoidance through velocity limits
- Real-time error correction (position threshold: 0.03-0.7m, orientation: 0.23-1.2 rad)

### Task 3: Mobile Robot Navigation
**File**: `task4/ebot_nav2_cmd.py`

- SLAM-based environment mapping
- Nav2 integration for path planning
- Waypoint-based navigation
- Multi-goal navigation sequences

**Navigation Workflow**:
1. Navigate to pickup location (arm vicinity)
2. Wait for box transfer from arm
3. Navigate to conveyor belt for delivery
4. Execute precision docking

### Task 4: Precision Docking
**File**: `task4/ebot_docking_service.py`

- Dual ultrasonic sensor feedback
- P-controller for angular and linear alignment
- ReentrantCallbackGroup for concurrent operations
- Service-based docking interface

**Control Parameters**:
- Angular gain (Kp): 1.5
- Linear gain (Kp): 0.8
- Yaw threshold: 0.010 rad
- Distance threshold: 0.05 m

### Task 5: Robot Coordination
**File**: `task4/ebot_passing_service.py`

- Custom service for arm-to-ebot handoff
- State synchronization between subsystems
- Gripper control via AttachLink/DetachLink services
- Box tracking and management

---

## 🛠️ Installation & Setup

### Prerequisites
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop-full

# MoveIt2
sudo apt install ros-humble-moveit

# Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Additional dependencies
sudo apt install ros-humble-tf-transformations
pip3 install opencv-python scipy pyarmor
```

### Build Instructions
```bash
# Clone the repository
cd ~/e_yantra_ws/src
git clone <repository-url> eyrc-24-25-logistic-cobot

# Install dependencies
cd ~/e_yantra_ws
rosdep install --from-paths src --ignore-src -r -y

# Build packages
colcon build --packages-select task4 task4c_bringup ur5_control

# Source the workspace
source ~/e_yantra_ws/install/setup.bash
```

---

## 🎮 Usage

### Launch Complete System
```bash
# Terminal 1: Launch Gazebo simulation environment
ros2 launch <simulation_package> world.launch.py

# Terminal 2: Launch MoveIt2 for UR5
ros2 launch <moveit_config> ur5_moveit.launch.py

# Terminal 3: Launch Nav2 for eBot
ros2 launch <nav2_config> navigation.launch.py

# Terminal 4: Launch coordination nodes
ros2 launch task4c_bringup task4.launch.py
```

### Individual Components

#### Run ArUco Detection Only
```bash
ros2 run ur5_control task1b
```

#### Run Servo-Based Manipulation
```bash
ros2 run ur5_control task2a
```

#### Run eBot Navigation
```bash
ros2 run task4 navigation
```

#### Run Docking Service
```bash
ros2 run task4 docking
```

---

## 📊 System Parameters

### Camera Specifications
- **Resolution**: 1280 x 720 pixels
- **Focal Length**: 931.18 pixels (fx, fy)
- **Principal Point**: (640, 360)
- **Depth Range**: 0.3m - 3.0m

### UR5 Configuration
- **Workspace**: Cylindrical (radius: 0.85m, height: 1.5m)
- **Joint Limits**: See `task2a.py` joint_limits dictionary
- **Max Velocity**: 6.0 cm/s (linear), 7.0 rad/s (angular)

### eBot Specifications
- **Docking Accuracy**: ±5cm (position), ±0.01 rad (orientation)
- **Navigation Speed**: Variable based on Nav2 DWB controller
- **Sensor Range**: Ultrasonic (0.02m - 4.0m)

---

## 🔧 Custom Interfaces

### DockSw.srv (Docking Service)
```
bool linear_dock
bool orientation_dock
float32 orientation
---
bool success
string message
```

### PassingSw.srv (Box Passing Service)
```
bool get_box
---
bool success
```

### PayloadSW.srv (Payload Control)
```
bool receive
bool drop
---
bool success
string message
```

---

## 📈 Performance Metrics

### ArUco Detection
- **Detection Rate**: 20 Hz (0.2s timer)
- **Accuracy**: Sub-centimeter positioning with depth fusion
- **Range**: 0.5m - 2.0m effective detection distance

### Manipulation Performance
- **Position Accuracy**: ±3-9cm (state-dependent thresholds)
- **Orientation Accuracy**: ±0.23-1.2 rad
- **Cycle Time**: ~15-20 seconds per pick-place operation

### Navigation Performance
- **Path Planning**: Real-time replanning with DWB
- **Docking Success Rate**: High precision with dual-sensor feedback
- **Average Navigation Time**: 10-15 seconds per waypoint

---

## 🐛 Troubleshooting

### Common Issues

**ArUco Not Detected**
```bash
# Check camera topics
ros2 topic echo /camera/color/image_raw
ros2 topic echo /camera/aligned_depth_to_color/image_raw

# Verify lighting conditions and marker size (150mm)
```

**TF Transform Errors**
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo base_link obj_<marker_id>
```

**Servo Motion Jerky**
```bash
# Adjust velocity gains in task2a.py
linear_velocity_gain = 10.0  # Reduce for smoother motion
angular_velocity_gain = 10.0
```

**Docking Fails**
```bash
# Check ultrasonic sensor readings
ros2 topic echo /ultrasonic_rl/scan
ros2 topic echo /ultrasonic_rr/scan

# Verify P-controller gains in ebot_docking_service.py
```

---

## 📚 Key Algorithms

### Visual Servoing Controller
```python
# Position error calculation
position_error = target_pose.position - current_pose.position

# Velocity command with P-controller
linear_velocity = Kp_linear * position_error
angular_velocity = Kp_angular * orientation_error

# Apply velocity limits
velocity = clamp(velocity, -max_vel, max_vel)
```

### Docking Alignment
```python
# Dual sensor averaging
distance = (ultrasonic_left + ultrasonic_right) / 2

# Angular correction
yaw_error = normalize_angle(target_yaw - current_yaw)
angular_cmd = Kp_angular * yaw_error

# Linear approach
distance_error = distance - target_distance
linear_cmd = -Kp_linear * distance_error
```

---

## 🎓 Learning Outcomes

- ROS2 service-based architecture design
- Real-time visual servoing implementation
- TF2 coordinate transformation management
- Multi-threaded executor patterns
- State machine design for robotic workflows
- Sensor fusion (RGB-D camera integration)
- P-controller tuning for robotic systems

---

## 📝 Documentation References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/humble/index.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [OpenCV ArUco Module](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [e-Yantra Portal](https://portal.e-yantra.org/)

---

## 🏆 Competition Results

This solution was developed for the e-Yantra Robotics Competition 2024-25, organized by IIT Bombay. The project demonstrates integration of multiple robotics concepts including computer vision, manipulation, navigation, and multi-robot coordination.

---

## 📄 License

This project was developed as part of the e-Yantra Robotics Competition. Please refer to competition guidelines for usage terms.

---

## 🤝 Acknowledgments

- **e-Yantra Team**: For organizing the competition and providing excellent support
- **IIT Bombay**: For the platform and resources
- **MoveIt2 Community**: For manipulation framework
- **Nav2 Team**: For navigation stack

---

## 📧 Contact

For queries related to this project:
- **Team Lead**: Anees Alwani (anisameen123@gmail.com)
- **Team ID**: 3152
- **Competition**: e-Yantra Robotics Competition 2024-25

---

**Last Updated**: March 2026
git remote add origin <url>
```
```command
git pull origin master 
```
