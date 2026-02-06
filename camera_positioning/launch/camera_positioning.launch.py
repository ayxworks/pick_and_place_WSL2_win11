from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    launch_args = []
    nodes = []

    # Launch arguments declaration
    launch_args.append(
        DeclareLaunchArgument(
            'board_length',
            default_value='0.225',
            description='The board length [m].',
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            'camera_base_frame',
            default_value='',
            description='The frame to give the transform from the board to. Should be a frame \
                         fixed wrt the camera optical frame. If not specified it will use the \
                         frame of the subscribed camera. Default is the camera optical frame',
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image',
            description='The camera topic to subscribe to.',
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            'sample_number',
            default_value='120',
            description='Number of samples recorded for the measurement.',
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            'axis_length',
            default_value='0.1',
            description='Axis length of the frame visualized in the OpenCV window.',
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            'base_frame',
            default_value='',
            description='The frame in which the transform to the camera_base_frame is given. \
                         Default is the calibration board itself.',
        )
    )

    launch_args.append(
        DeclareLaunchArgument(
            'board_frame',
            default_value='',
            description='Must be specified if base_frame is set. The TF frame of the calibration \
                         board wrt to the base_frame.',
        )
    )

    # Launch arguments definition
    board_length = LaunchConfiguration('board_length')
    camera_base_frame = LaunchConfiguration('camera_base_frame')
    camera_topic = LaunchConfiguration('camera_topic')
    sample_number = LaunchConfiguration('sample_number')
    axis_length = LaunchConfiguration('axis_length')
    base_frame = LaunchConfiguration('base_frame')
    board_frame = LaunchConfiguration('board_frame')

    # Add node, pass arguments to it
    nodes.append(
        Node(
            namespace="",
            name="camera_positioning_node",
            package="camera_positioning",
            executable="diamond_detector",
            output="both",
            parameters=[
                    {
                    "board_length" : board_length,
                    "camera_base_frame" : camera_base_frame,
                    "camera_topic" : camera_topic,
                    "sample_number" : sample_number,
                    "axis_length" : axis_length,
                    "base_frame" : base_frame,
                    "board_frame" : board_frame,
                    },
                ],
        )
    )

    return LaunchDescription(launch_args + nodes)