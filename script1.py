#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

frames = [
    "cam_flange_ring_flange",
    "cam_flange_ring_mounting",
    "cam_flange_support",
    "cam_flange_support_mounting",
    "camera_bottom_screw_frame",
    "camera_color_frame",
    "camera_color_optical_frame",
    "camera_depth_optical_frame",
    "camera_link"
]

class TfReader(Node):
    def __init__(self):
        super().__init__('tf_reader')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.print_transforms)

    def print_transforms(self):
        for frame in frames:
            try:
                t = self.tf_buffer.lookup_transform('world', frame, rclpy.time.Time())
                print(f"Frame: {frame}")
                print(f"  Translation: x={t.transform.translation.x:.3f}, y={t.transform.translation.y:.3f}, z={t.transform.translation.z:.3f}")
                print(f"  Rotation (quat): x={t.transform.rotation.x:.3f}, y={t.transform.rotation.y:.3f}, z={t.transform.rotation.z:.3f}, w={t.transform.rotation.w:.3f}")
            except Exception as e:
                print(f"Could not get transform for {frame}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TfReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
