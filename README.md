# Vision-Based UR10e Pick and Place 

A complete ROS 2 workspace for operating and controlling a UR10e collaborative robot with Robotiq gripper, integrated vision pipeline, and motion planning for a vision-based pick-and-place task.

## 📋 Overview

This workspace provides:
- **Robot Control**: UR10e driver and hardware interface
- **Motion Planning**: MoveIt integration for path planning and manipulation
- **Perception**: Vision pipeline with FoundationPose for object detection and 6D pose estimation
- **Manipulation**: Pick-and-place application with integrated GUI
- **Hardware**: Robotiq gripper control and custom camera flange support

## ✅ Prerequisites

- **ROS 2 Humble** installed on your system
- **Git** installed for cloning repositories
- **Python 3.10+** for ROS 2 Humble
- **Docker** and **docker-compose** (for containerized setup)
- **RealSense SDK** ([installation instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages))
- **UR Robot Setup**: Robotiq Gripper URCap **must be uninstalled** from the UR teach pendant before running this workspace

## 📦 Workspace Packages

| Package | Purpose |
|---------|---------|
| `setup_launch` | Main launch files for the complete system |
| `setup_description` | URDF descriptions and robot configuration |
| `setup_moveit_config` | MoveIt configuration for motion planning |
| `pick_and_place` | Pick-and-place manipulation application (C++) |
| `pick_and_place_gui` | Web-based GUI for pick-and-place control |
| `vision_pipeline` | Object detection and pose estimation using FoundationPose |
| `robotiq_controllers` | Robotiq gripper control interface |
| `robotiq_driver` | Low-level Robotiq gripper communication |
| `cobra_camera_positioning` | Camera positioning and calibration |
| `cam_flange_support` | Custom camera mount for UR flange |
| `serial` | Serial communication utilities |
| `Universal_Robots_ROS2_Driver` | UR robot hardware driver and communication |
| `Universal_Robots_ROS2_Description` | UR robot URDF and descriptions |

## 🚀 Quick Start

### Option A: Docker Setup (Recommended)

Easiest option with all dependencies pre-configured:

```bash
# 1. Clone or navigate to the workspace
cd eurobots_ws

# 2. Configure X11 access for Docker
xhost +local:docker

# 3. Build and launch the container
docker-compose up --build
```

### Option B: Native Installation

For development or systems without Docker:

```bash
# 1. Create/navigate to workspace
cd ~/eurobots_ws

# 2. Clone dependencies
vcs import src < src/dependencies.repos

# 3. Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build --symlink-install

# 5. Source the environment
source install/setup.bash

# 6. Launch the Pick-and-Place task
ros2 launch pick_and_place pick_and_placep_launch.launch.py
```




## 🐛 Troubleshooting

### Connection Issues
- Verify the UR robot IP matches your network: `export UR_ROBOT_IP=<your_ip>`
- Check network connectivity: `ping $UR_ROBOT_IP`
- Ensure **External Control** program is active on UR teach pendant

### Build Failures
- Clean rebuild: `colcon build --symlink-install --cmake-clean-first`
- Check dependencies: `rosdep check --all --from-paths src`
- Verify ROS 2 Humble is sourced: `echo $ROS_DISTRO`

### Robotiq Gripper Issues
- **ERROR**: Ensure Robotiq Gripper URCap is **uninstalled** from the UR
- Check serial port: `ls /tmp/ttyUR` (should see device)

### Vision Pipeline Problems
- Intel RealSense camera not detected: `realsense-viewer`
- Ensure camera is properly connected via USB 3.0
- Check permissions: `sudo usermod -a -G video $USER`

### Zombie ROS/Python Processes
```bash
killall -9 ros2
killall -9 python*
pkill -f rclpy
```


## 👥 Contributors

- **Martin Grao** (mgrao@ikerlan.es) - Main development
- **Mikel Mujika** (mmujika@ikerlan.es) - Vision pipeline and integration
