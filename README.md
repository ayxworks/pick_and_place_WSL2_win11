# UR10e DIGILAB Setup

This repository contains the necessary setup for operating the UR10e robot at DIGILAB.

## Prerequisites

- ROS 2 Humble installed on your system
- Basic knowledge of ROS workspaces
- Git installed

## Installation

Follow these steps to set up the workspace:

### 1. Create the workspace
```bash
mkdir -p ~/ur10e_ws/src
cd ~/ur10e_ws/src
```

### 2. Clone the repository
```bash
git clone http://kaugitea.ikerlan.es/mgrao/ur_bringup.git
cd ..
```

### 3. Import dependencies

Use `vcs` to import all required dependencies from the `dependencies.repos` file:
```bash
vcs import src < src/<repository-name>/dependencies.repos
```

### 4. Install dependencies with rosdep
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the workspace
```bash
colcon build
```

### 6. Source the workspace
```bash
source install/setup.bash
```
### 7. Launch the bringup file
```bash
ros2 launch setup_launch setup_launch.launch.py
```


## Usage

[Add specific instructions for launching and operating the UR10e robot]

## Troubleshooting

If you encounter any issues during installation, make sure:
- All ROS dependencies are properly installed
- You have the correct ROS distribution for this package
- All submodules are properly initialized

## Contributors

Martin Grao