# UR10e DIGILAB Setup

This repository contains the necessary setup for operating the UR10e robot at DIGILAB.

## Prerequisites

- ROS 2 Humble installed on your system
- Basic knowledge of ROS workspaces
- Git installed
- RealSense SDK installed [(instructions)] (https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)
- Robotiq Gripper URCap uninstalled from UR

## Installation & Usage

Follow these steps to set up the workspace:

## Option A: Docker Installataion

For a containerized setup that includes all the dependencies:

#### 1. Clone the repository:
```bash
git clone http://kaugitea.ikerlan.es/mgrao/ur_bringup.git .
cd ur_bringup
```

#### 2. Configure xhost
```bash
xhost +local:docker
```
#### 3. Build and launch
```bash
docker compose up --build
```

## Option B: Native Installataion

#### 1. Create the workspace
```bash
mkdir -p ~/ur10e_ws/src
cd ~/ur10e_ws/src
```

#### 2. Clone the repository
```bash
git clone http://kaugitea.ikerlan.es/mgrao/ur_bringup.git .
cd ..
```

#### 3. Import dependencies
Use `vcs` to import all required dependencies from the `dependencies.repos` file:
```bash
vcs import src < src/<repository-name>/dependencies.repos
```

#### 4. Install dependencies with rosdep
```bash
rosdep install --from-paths src --ignore-src -r -y
```

#### 5. Build the workspace
```bash
colcon build
```

#### 6. Source the workspace
```bash
source install/setup.bash
```
#### 7. Launch the bringup file
```bash
ros2 launch setup_launch setup_launch.launch.py
```

### 8. Play External Control program from the UR Teach Pendant


## Troubleshooting

If you encounter any issues during installation, make sure:
- All ROS dependencies are properly installed
- You have the correct ROS distribution for this package
- All submodules are properly initialized
- Robotiq Grippers URCap is uninstalled from the UR
- Kill all ROS 2 and Python zombie nodes

## Contributors

Martin Grao & Ander Gonzalez