# Copyright 2026, IKERLAN S. Coop.
# Copyright 2026, Industrial Machinery Export Bilbao S.L.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import os

# ------------------------------------------------------------
# Configure Python paths so FoundationPose modules can be found
# ------------------------------------------------------------
module_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose"))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose/learning"))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose/learning/training"))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose/learning/datasets"))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose/learning/models"))

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from threading import Lock
import tf2_ros
from vision_pipeline.FoundationPose.estimater import *
import cv2
import numpy as np
import trimesh
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R


# ------------------------------------------------------------
# Monkey-patching FoundationPose to track registration status
# ------------------------------------------------------------

# Save original methods
original_init = FoundationPose.__init__
original_register = FoundationPose.register


def modified_init(self, model_pts, model_normals, symmetry_tfs=None,
                  mesh=None, scorer=None, refiner=None,
                  glctx=None, debug=0, debug_dir="./FoundationPose"):
    """
    Extended constructor that adds a flag to know
    whether register() has been executed.
    """
    original_init(self, model_pts, model_normals, symmetry_tfs,
                  mesh, scorer, refiner, glctx, debug, debug_dir)
    self.is_register = False


def modified_register(self, K, rgb, depth, ob_mask, iteration):
    """
    Wrapper around the original register() method
    that sets a flag when pose estimation is performed.
    """
    pose = original_register(self, K, rgb, depth, ob_mask, iteration)
    self.is_register = True
    return pose


# Override FoundationPose methods
FoundationPose.__init__ = modified_init
FoundationPose.register = modified_register


# ============================================================
# ROS 2 Node providing a pose estimation service
# ============================================================
class PoseEstimatorService(Node):

    def __init__(self):
        super().__init__(
            "pose_estimator_service",
            automatically_declare_parameters_from_overrides=True
        )

        # ----------------------------------------------------
        # Load ROS parameters
        # ----------------------------------------------------
        self.object_frame = self.get_parameter("object_frame").value
        self.mesh_file = self.get_parameter("mesh_file").value
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.debug = self.get_parameter("debug").value
        self.debug_dir = self.get_parameter("debug_dir").value

        # Position offset applied after pose estimation
        self.position_offset = {
            "x": self.get_parameter("position_offset.x").value,
            "y": self.get_parameter("position_offset.y").value,
            "z": self.get_parameter("position_offset.z").value,
        }

        # Orientation offset (Euler angles)
        self.orientation_offset = {
            "roll": self.get_parameter("orientation_offset.roll").value,
            "pitch": self.get_parameter("orientation_offset.pitch").value,
            "yaw": self.get_parameter("orientation_offset.yaw").value,
        }

        # Number of refinement iterations
        self.est_refine_iter = self.get_parameter(
            "estimation_params.est_refine_iter"
        ).value

        # ----------------------------------------------------
        # ROS utilities
        # ----------------------------------------------------
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.lock = Lock()

        # Latest sensor data buffers
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_K = None

        # Flags to ensure all sensor data is available
        self.rgb_received = False
        self.depth_received = False
        self.camera_info_received = False

        # FoundationPose related objects
        self.pose_estimator = None
        self.mesh = None
        self.to_origin = None
        self.bbox = None

        self.window_name = "Pose Estimation"

        # Initialize FoundationPose
        self.initialize_foundationpose()

        # ----------------------------------------------------
        # ROS subscriptions
        # ----------------------------------------------------

        # Callback groups to allow concurrent callbacks
        self.sensor_cb_group = ReentrantCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Image, self.rgb_topic, self.rgb_callback, 10, callback_group=self.sensor_cb_group
        )
        self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10, callback_group=self.sensor_cb_group
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic,
            self.camera_info_callback, 10, callback_group=self.sensor_cb_group
        )

        # Pose estimation service
        self.create_service(
            Trigger, "estimate_pose", self.estimate_pose_callback, callback_group=self.service_cb_group
        )

        self.get_logger().info("Pose Estimator Service initialized")

    def initialize_foundationpose(self):
        """
        Loads the mesh, computes bounding box information
        and initializes the FoundationPose estimator.
        """
        set_logging_format()
        set_seed(0)

        self.mesh = trimesh.load(self.mesh_file)
        os.makedirs(self.debug_dir, exist_ok=True)

        # Compute transformation to center the mesh at origin
        self.to_origin, extents = trimesh.bounds.oriented_bounds(self.mesh)
        self.bbox = np.stack([-extents / 2, extents / 2]).reshape(2, 3)

        self.pose_estimator = FoundationPose(
            model_pts=self.mesh.vertices,
            model_normals=self.mesh.vertex_normals,
            mesh=self.mesh,
            scorer=ScorePredictor(),
            refiner=PoseRefinePredictor(),
            glctx=dr.RasterizeCudaContext(),
            debug=self.debug,
            debug_dir=self.debug_dir
        )

    # --------------------------------------------------------
    # Sensor callbacks
    # --------------------------------------------------------
    def camera_info_callback(self, msg):
        """Stores the camera intrinsic matrix."""
        self.camera_K = np.array(msg.k).reshape(3, 3)
        self.camera_info_received = True

    def rgb_callback(self, msg):
        """Stores the latest RGB image."""
        with self.lock:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="rgb8"
            )
            self.rgb_received = True

    def depth_callback(self, msg):
        """Stores the latest depth image (converted to meters)."""
        with self.lock:
            depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            self.latest_depth = depth.astype(np.float32) / 1000.0
            self.depth_received = True

    # --------------------------------------------------------
    # Pose utilities
    # --------------------------------------------------------
    def apply_offsets(self, pose):
        """
        Applies user-defined position and orientation offsets
        to the estimated pose.
        """
        t = np.array([
            self.position_offset["x"],
            self.position_offset["y"],
            self.position_offset["z"]
        ])

        rot = R.from_euler(
            "xyz",
            [
                self.orientation_offset["roll"],
                self.orientation_offset["pitch"],
                self.orientation_offset["yaw"],
            ],
        ).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = t

        return pose @ T

    def publish_tf(self, pose, stamp):
        """
        Publishes the pose as a static TF transform.
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.object_frame

        t.transform.translation.x = pose[0, 3]
        t.transform.translation.y = pose[1, 3]
        t.transform.translation.z = pose[2, 3]

        q = R.from_matrix(pose[:3, :3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    # --------------------------------------------------------
    # UI helpers (OpenCV)
    # --------------------------------------------------------
    def add_buttons_to_image(self, image):
        """
        Draws Accept / Retry / Reject buttons on the image.
        """
        h, w = image.shape[:2]
        bw, bh, m = 200, 60, 20
        img = image.copy()

        x0 = w // 2 - (3 * bw + 2 * m) // 2
        y = h - bh - m

        buttons = [
            ("ACCEPT (A)", (0, 255, 0), x0),
            ("RETRY (N)", (255, 0, 0), x0 + bw + m),
            ("REJECT (R)", (0, 0, 255), x0 + 2 * (bw + m)),
        ]

        for text, color, x in buttons:
            cv2.rectangle(img, (x, y), (x + bw, y + bh), color, -1)
            cv2.putText(
                img, text, (x + 30, y + 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (255, 255, 255), 2
            )

        cv2.putText(
            img,
            "A = Accept | N = Retry | R = Reject | ESC = Cancel",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (255, 255, 255), 2
        )
        return img

    def wait_for_user_decision(self, image):
        """
        Displays the visualization and waits for
        user keyboard input.
        """
        img = self.add_buttons_to_image(image)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        while True:
            cv2.imshow(self.window_name, img[..., ::-1])
            k = cv2.waitKey(100) & 0xFF

            if k in (ord("a"), ord("A")):
                cv2.destroyWindow(self.window_name)
                return "accept"
            if k in (ord("n"), ord("N")):
                return "retry"
            if k in (ord("r"), ord("R"), 27):
                cv2.destroyWindow(self.window_name)
                return "reject"

    # --------------------------------------------------------
    # Service callback
    # --------------------------------------------------------
    def estimate_pose_callback(self, request, response):
        """
        Estimates object pose, allows the user to
        accept, retry or reject the result.
        """
        
        while True:
            if not (self.rgb_received and self.depth_received and self.camera_info_received):
                response.success = False
                response.message = "Missing sensor data"
                return response

            with self.lock:
                rgb = self.latest_rgb.copy()
                depth = self.latest_depth.copy()
                K = self.camera_K.copy()

            stamp = self.get_clock().now().to_msg()

            # Remove invalid depth values
            depth[(depth < 0.1) | (depth > 3.0)] = 0
            mask = depth > 0

            # Run pose estimation
            pose = self.pose_estimator.register(
                K=K,
                rgb=rgb,
                depth=depth,
                ob_mask=mask,
                iteration=self.est_refine_iter,
            )

            # Center pose and apply offsets
            center_pose = pose @ np.linalg.inv(self.to_origin)
            pose_with_offset = self.apply_offsets(center_pose)

            # Visualization
            vis = draw_posed_3d_box(
                K, rgb, center_pose, self.bbox
            )

            decision = self.wait_for_user_decision(vis)

            if decision == "retry":
                continue

            if decision == "accept":
                self.publish_tf(pose_with_offset, stamp)
                response.success = True
                response.message = "Pose accepted"
                return response

            response.success = False
            response.message = "Pose rejected"
            return response


# ------------------------------------------------------------
# Main entry point
# ------------------------------------------------------------
def main():
    rclpy.init()
    node = PoseEstimatorService()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
