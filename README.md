# Vision-Based UR10e Pick and Place 

A complete ROS 2 workspace for operating and controlling a UR10e collaborative robot with Robotiq gripper, integrated vision pipeline, and motion planning for a vision-based pick-and-place task.

## Overview

This workspace provides:
- **Robot Control**: UR10e driver and hardware interface
- **Motion Planning**: MoveIt integration for path planning and manipulation
- **Perception**: Vision pipeline with FoundationPose for object detection and 6D pose estimation
- **Manipulation**: Pick-and-place application with integrated GUI
- **Hardware**: Robotiq gripper control and custom camera flange support

## Prerequisites

- **Git** installed for cloning repositories
- **Python 3.10+** for ROS 2 Humble
- **Docker** and **docker-compose** (for containerized setup)
- **RealSense SDK** ([installation instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages))
- **UR Robot Setup**: Robotiq Gripper URCap **must be uninstalled** from the UR teach pendant before running this workspace

## Workspace Packages

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
| `camera_positioning` | Camera positioning and calibration |
| `cam_flange_support` | Custom camera mount for UR flange |
| `serial` | Serial communication utilities |
| `Universal_Robots_ROS2_Driver` | UR robot hardware driver and communication |
| `Universal_Robots_ROS2_Description` | UR robot URDF and descriptions |
## Configuration and Setup
### Camera Positioning
To position the camera, you first need a printed DICT_4X4 ChArUco board placed in a known and fixed location.

Adjust the pose of the board in `setup_description/urdf/robot.urdf.xacro`:

```xml
<joint name="charuco_board_joint" type="fixed">
    <parent link="robotiq_base_link"/>
    <child link="charuco_board"/>
    <origin xyz="-0.15 0 0.15" rpy="0 0 -1.5708"/>
</joint>
```

- **xyz**: Place location position in world frame (meters)
- **rpy**: Place orientation in world frame (radians)

Run the system as described in [Quick Start](#quick-start).

Then, enter the container interactively:

```bash
docker exec -it pick_and_place bash
```

Run the camera positioning algorithm:

```bash
ros2 run cobra_camera_positioning diamond_detector --ros-args -p camera_topic:=/camera/color/image_raw -p board_length:=0.225 -p camera_base_frame:=camera_link -p base_frame:=base_link -p board_frame:=charuco_board
```

NOTE: Adjust board_length to match the actual size of your ChArUco board.

The algorithm will output the camera's position relative to base_link. Use this information when configuring `robot.urdf.xacro` file in the next section.

For more information, refer to [the documentation of the package](camera_positioning/README.md).  
### File Configuration

Before running the system, configure the following files to match your hardware setup and environment. See the [Quick Start](#quick-start) section to begin execution after configuration:

### `pick_and_place/config/configuration.yaml`

This file defines the pick-and-place task parameters:

| Parameter | Purpose |
|-----------|---------|
| **arm_move_group** | MoveIt planning group name for the robotic arm (e.g., "ur_manipulator") |
| **gripper_move_group** | MoveIt planning group name for the gripper (e.g., "gripper") |
| **pre_pick_frame** | TF frame for the pre-approach position before picking |
| **pre_place_frame** | TF frame for the pre-approach position before placing |
| **home_position** | Named joint configuration for the rest/home position |
| **observation_position** | Named joint configuration for detection service position |
| **gripper_open_position** | Named gripper configuration for open state |
| **gripper_closed_position** | Named gripper configuration for grasping state |
| **detect_service_name** | ROS service name for object detection |
| **pick_approach_distance** | Distance (meters) for final approach along gripper Z-axis |
| **place_leave_distance** | Distance (meters) to retreat after placing |
| **lift_distance** | Distance (meters) to lift object after grasping |
| **drop_distance** | Distance (meters) to lower object before releasing |
| **object_id** | Identifier for collision object in MoveIt |
| **object_length/width/height** | Dimensions of the object being picked (meters) |

### `pick_and_place/config/obstacles.yaml`

This file defines collision objects in the planning scene:

- **id**: Unique identifier for each obstacle
- **type**: Geometry type (box, sphere, cylinder, or mesh)
- **frame**: Reference frame (typically "world")
- **size/radius/height**: Dimensions of the obstacle
- **position**: [x, y, z] coordinates in the reference frame
- **orientation**: [x, y, z, w] quaternion (rotation)

**Current obstacles include:**
- `boxes`: Main workspace boundary box
- `table`: Work surface with negative Z to avoid collision below table
- `obstacle`: Barrier in the workspace
- `cell`: Boundary constraint

### `setup_description/urdf/robot.urdf.xacro`

Key sections to configure:

**Camera Position & Extrinsics:**
```xml
<xacro:sensor_d435 parent="base_link" use_nominal_extrinsics="$(arg use_nominal_extrinsics)">
  <origin xyz="-1.090329 0.318755 0.635220" rpy="-0.015609 0.316050 0.494161"/> <!-- CALIBRATED CAMERA EXTRINSICS -->
</xacro:sensor_d435>
```
- **xyz**: Camera position relative to base_link (meters)
- **rpy**: Camera orientation in roll-pitch-yaw (radians)

**Place Frame:**
```xml
<joint name="object_place_joint" type="fixed">
  <parent link="world"/>
  <child link="object_place"/>
  <origin xyz="-0.220 0.587 0.152" rpy="-3.083 0.018 -1.727"/> <!-- SET OBJECT PLACE FRAME -->
</joint>
```
- **xyz**: Place location position in world frame (meters)
- **rpy**: Place orientation in world frame (radians)

**Related Frames:**
- `charuco_board`: Calibration frame attached to gripper
- `ee_link`: End-effector virtual link at fingertips (0.22m offset along Z-axis from tool0)

All coordinates must be calibrated and match your actual hardware setup.

## Quick Start

```bash
# 1. Clone or navigate to the workspace
cd eurobots_ws

# 2. Configure X11 access for Docker
xhost +local:docker

# 3. Build and launch the container
docker compose up --build
```

## Troubleshooting

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


## Contributors

- **Martin Grao** (mgrao@ikerlan.es) - Main development
- **Mikel Mujika** (mmujika@ikerlan.es) - Vision pipeline and integration

## License

This project is licensed under the [Apache License 2.0](LICENSE).

You can find the full license text in the `LICENSE` file.
