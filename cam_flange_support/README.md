# Cam flange support

This repository integrates the steel camera base support in ROS2.

![steel_cam_base_support](./rsc/full_piece.png)

## Parts

It consists of two pieces, the support itself, and a steel ring to help mounting the piece. 

![cam_support](./rsc/camera_support.png)
![ring](./rsc/ring_flange.png)

These two pieces are supposed to be mounted between the robot flange and its tool. With setups like UR robot + Robotiq gripper, the camera support is located on the flange side of the mounting, while the ring is located on the gripper side.

![ur_robotiq_far](./rsc/ur_robotiq_far.png)
![ur_robotiq_close](./rsc/ur_robotiq_close.png)

## Auxiliary frames

The repository provides auxiliary TF frames for the different holes in which the camera can be mounted, as well as frames corresponding with the mounting frames, with the Z axis aligned with the robot flange longitudinal axis.

![tfs_holes](./rsc/tfs_holes.png)
![tfs_mounting](./rsc/tfs_mounting.png)

## TODO

- **Visual meshes (.obj) do not seem to be compatible with gazebo**. Simply applying color might fix it, otherwise we might need to export the stls again or use a different file format (.dae).

- Following the previous point, add Gazebo to debug launch file visualizer.launch.py.