import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    
    robot_name = LaunchConfiguration("robot_name", default="ur")
    robot_ip = LaunchConfiguration("robot_ip")
    use_sim = LaunchConfiguration("use_sim", default="false")
    include_digilab = LaunchConfiguration("include_digilab", default="false")
    controllers_file = LaunchConfiguration("controllers_file", default="ur_controllers.yaml")
    use_cam_flange_support = LaunchConfiguration("use_cam_flange_support", default="false")

    if use_sim.perform(context) == "true":
        controllers_file = str(controllers_file.perform(context)).replace(".yaml", "_gazebo.yaml")

    # Launch del driver del UR
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("setup_launch"),
                    "launch",
                    "ur_control.launch.py",
                ]
            )
        ),
        launch_arguments={
                    "robot_name": robot_name,
            "ur_type": "ur10e",
            "robot_ip": robot_ip,
            "launch_rviz": "false",
            "use_fake_hardware": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "activate_joint_controller": "true",
            "description_package": "setup_description",
            "description_file": "robot.urdf.xacro",
            "use_tool_communication": "true",
            "gripper_com_port": "/tmp/ttyUR",
            "tool_voltage": "24",
            "tool_parity": "0",
            "tool_baud_rate": "115200",
            "tool_stop_bits": "1",
            "tool_rx_idle_chars": "1.5",
            "tool_tx_idle_chars": "3.5",
            "tool_device_name": "/tmp/ttyUR",
            "use_sim": use_sim,
            "controllers_file": controllers_file,
            "use_cam_flange_support": use_cam_flange_support,
            "include_digilab": include_digilab,
        }.items(),
    )

    # Launch de MoveIt
    moveit_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("setup_moveit_config"),
                            "launch",
                            "ur_moveit.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "robot_name": robot_name,
                    "ur_type": "ur10e",
                    "launch_rviz": "true",
                    "robot_ip": robot_ip,
                    "description_package": "setup_description",
                    "description_file": "robot.urdf.xacro",
                    "moveit_config_package": "setup_moveit_config",
                    "moveit_config_file": "ur.srdf.xacro",
                    "gripper_com_port": "/tmp/ttyUR",
                    "use_sim_time": use_sim,
                    "use_cam_flange_support": use_cam_flange_support,
                    "include_digilab": include_digilab,
                }.items(),
            )
    
    # Launch de RealSense D435
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                ]
            )
        ),
        launch_arguments={
            "camera_name": "camera",
            "camera_namespace": "",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "depth_module.profile": "1280x720x30",
            "rgb_camera.profile": "1280x720x30",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "publish_tf": "true",
        }.items(),
    )

    
    nodes = [
        ur_control_launch, 
        moveit_launch,
        realsense_launch,
    ]
    
    return nodes

def generate_launch_description():

    
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="ur",
            description="Robot name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.16.7.75",
            description="IP del robot UR10e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo Ignition simulation.',
            choices=["true", "false"],
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'include_digilab',
            default_value='false',
            description='If true, digilab surroundings added to URDF.',
            choices=["true", "false"],
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_cam_flange_support',
            default_value='true',
            description='Whether to include cam flange support.',
            choices=["true", "false"],
        ),
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
