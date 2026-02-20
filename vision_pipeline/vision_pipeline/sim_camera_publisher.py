#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

from ament_index_python.packages import get_package_share_directory


class SimCameraNode(Node):

    def __init__(self):
        super().__init__('sim_camera_publisher')

        # -----------------------------------
        # Obtener path del paquete
        # -----------------------------------
        package_name = 'vision_pipeline'
        package_share = get_package_share_directory(package_name)

        rgb_path = os.path.join(package_share, 'sim_rsc', 'rgb.png')
        depth_path = os.path.join(package_share, 'sim_rsc', 'depth.png')

        self.get_logger().info(f"RGB path: {rgb_path}")
        self.get_logger().info(f"Depth path: {depth_path}")

        # -----------------------------------
        # Publishers
        # -----------------------------------
        self.rgb_pub = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            10
        )

        self.info_pub = self.create_publisher(
            CameraInfo,
            '/camera/color/camera_info',
            10
        )

        self.bridge = CvBridge()

        # -----------------------------------
        # Cargar imágenes
        # -----------------------------------
        self.rgb_img = cv2.imread(rgb_path, cv2.IMREAD_COLOR)

        if self.rgb_img is None:
            self.get_logger().error("RGB image not found!")
            raise RuntimeError("RGB image missing")

        self.depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

        if self.depth_img is None:
            self.get_logger().error("Depth image not found!")
            raise RuntimeError("Depth image missing")

        # Convertimos depth a float32 si no lo es
        if self.depth_img.dtype != np.float32:
            self.depth_img = self.depth_img.astype(np.float32)

        # -----------------------------------
        # Timer
        # -----------------------------------
        self.timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_data
        )

        self.get_logger().info("Sim Camera Node started.")


    # --------------------------------------------------
    # CameraInfo EXACTO
    # --------------------------------------------------
    def create_camera_info(self, stamp):

        msg = CameraInfo()

        msg.header.stamp = stamp
        msg.header.frame_id = "camera_color_optical_frame"

        msg.height = 720
        msg.width = 1280

        msg.distortion_model = "plumb_bob"

        msg.d = [
            -0.055543940514326096,
             0.0655515119433403,
            -0.00021648436086252332,
             0.0008187743951566517,
            -0.02100312151014805
        ]

        msg.k = [
            642.8826904296875, 0.0, 647.38623046875,
            0.0, 642.2827758789062, 352.8768615722656,
            0.0, 0.0, 1.0
        ]

        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        msg.p = [
            642.8826904296875, 0.0, 647.38623046875, 0.0,
            0.0, 642.2827758789062, 352.8768615722656, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        msg.binning_x = 0
        msg.binning_y = 0

        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi.height = 0
        msg.roi.width = 0
        msg.roi.do_rectify = False

        return msg


    # --------------------------------------------------
    # Publicación
    # --------------------------------------------------
    def publish_data(self):

        now = self.get_clock().now().to_msg()

        rgb_msg = self.bridge.cv2_to_imgmsg(
            self.rgb_img,
            encoding='bgr8'
        )

        depth_msg = self.bridge.cv2_to_imgmsg(
            self.depth_img,
            encoding='32FC1'
        )

        rgb_msg.header.stamp = now
        depth_msg.header.stamp = now

        rgb_msg.header.frame_id = "camera_color_optical_frame"
        depth_msg.header.frame_id = "camera_color_optical_frame"

        camera_info = self.create_camera_info(now)

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(camera_info)


def main(args=None):
    rclpy.init(args=args)
    node = SimCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
