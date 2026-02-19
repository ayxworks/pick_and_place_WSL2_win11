#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# List of frames with their current position and orientation
transforms = [
    {
        "parent": "base_link",
        "child": "camera_bottom_screw_frame",
        "translation": [-1.094, 0.337, 0.632],
        "rotation": [-0.064, 0.147, 0.217, 0.963]
    },
    {
        "parent": "base_link",
        "child": "camera_color_frame",
        "translation": [-1.064, 0.306, 0.643],
        "rotation": [-0.064, 0.146, 0.217, 0.963]
    },
    {
        "parent": "base_link",
        "child": "camera_color_optical_frame",
        "translation": [-1.064, 0.306, 0.643],
        "rotation": [0.695, -0.414, 0.332, -0.485]
    },
    {
        "parent": "base_link",
        "child": "camera_depth_optical_frame",
        "translation": [-1.089, 0.359, 0.640],
        "rotation": [0.695, -0.414, 0.331, -0.485]
    },
    {
        "parent": "base_link",
        "child": "camera_link",
        "translation": [-1.089, 0.359, 0.640],
        "rotation": [-0.064, 0.147, 0.217, 0.963]
    },
]

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish_transforms)  # 20 Hz

    def publish_transforms(self):
        for t in transforms:
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = t["parent"]
            msg.child_frame_id = t["child"]
            msg.transform.translation.x = t["translation"][0]
            msg.transform.translation.y = t["translation"][1]
            msg.transform.translation.z = t["translation"][2]
            msg.transform.rotation.x = t["rotation"][0]
            msg.transform.rotation.y = t["rotation"][1]
            msg.transform.rotation.z = t["rotation"][2]
            msg.transform.rotation.w = t["rotation"][3]
            self.broadcaster.sendTransform(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
