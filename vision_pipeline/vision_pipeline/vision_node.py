import sys
import os

# Configure path for FoundationPose
module_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose"))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose/learning"))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose/learning/training"))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose/learning/datasets"))
sys.path.insert(0, os.path.join(module_dir, "FoundationPose/learning/models"))


import yaml
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
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


# Save the original `__init__` and `register` methods
original_init = FoundationPose.__init__
original_register = FoundationPose.register


# Modify `__init__` to add `is_register` attribute
def modified_init(self, model_pts, model_normals, symmetry_tfs=None, mesh=None, scorer=None, refiner=None, glctx=None, debug=0, debug_dir="./FoundationPose"):
    original_init(self, model_pts, model_normals, symmetry_tfs, mesh, scorer, refiner, glctx, debug, debug_dir)
    self.is_register = False  # Initialize as False


# Modify `register` to set `is_register` to True when a pose is registered
def modified_register(self, K, rgb, depth, ob_mask, iteration):
    pose = original_register(self, K, rgb, depth, ob_mask, iteration)
    self.is_register = True  # Set to True after registration
    return pose


# Apply the modifications
FoundationPose.__init__ = modified_init
FoundationPose.register = modified_register


class PoseEstimatorService(Node):
    def __init__(self):
        super().__init__("pose_estimator_service", automatically_declare_parameters_from_overrides=True)

        self.object_frame = self.get_parameter("object_frame").value
        self.mesh_file = self.get_parameter("mesh_file").value
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.debug = self.get_parameter("debug").value
        self.debug_dir = self.get_parameter("debug_dir").value

        self.position_offset = {
            "x": self.get_parameter("position_offset.x").value,
            "y": self.get_parameter("position_offset.y").value,
            "z": self.get_parameter("position_offset.z").value,
        }
        self.orientation_offset = {
            "roll": self.get_parameter("orientation_offset.roll").value,
            "pitch": self.get_parameter("orientation_offset.pitch").value,
            "yaw": self.get_parameter("orientation_offset.yaw").value,
        }
        self.est_refine_iter = self.get_parameter("estimation_params.est_refine_iter").value
        self.track_refine_iter = self.get_parameter("estimation_params.track_refine_iter").value

        # Initialize variables
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.lock = Lock()

        # Variables to store images and camera data
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_K = None
        self.rgb_received = False
        self.depth_received = False
        self.camera_info_received = False

        # FoundationPose variables
        self.pose_estimator = None
        self.mesh = None
        self.to_origin = None
        self.bbox = None

        # Variables for user confirmation
        self.user_decision = None
        self.window_name = "Pose Estimation"

        # Initialize FoundationPose
        self.initialize_foundationpose()

        # Initialize cameras
        # Create subscribers
        self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

        if not self.camera_info_received:
            camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        # Create service
        self.srv = self.create_service(Trigger, "estimate_pose", self.estimate_pose_callback)

        self.get_logger().info("Pose Estimator Service initialized and ready")

    def initialize_foundationpose(self):
        """Initialize the FoundationPose estimator"""
        try:
            set_logging_format()
            set_seed(0)

            if not self.mesh_file or not os.path.exists(self.mesh_file):
                raise FileNotFoundError(f"Mesh file not found: {self.mesh_file}")

            # Load mesh
            self.mesh = trimesh.load(self.mesh_file)

            # Prepare debug directory
            os.makedirs(self.debug_dir, exist_ok=True)

            # Compute bounding box and transform to origin
            self.to_origin, extents = trimesh.bounds.oriented_bounds(self.mesh)
            self.bbox = np.stack([-extents / 2, extents / 2], axis=0).reshape(2, 3)

            # Initialize FoundationPose components
            scorer = ScorePredictor()
            refiner = PoseRefinePredictor()
            glctx = dr.RasterizeCudaContext()

            self.pose_estimator = FoundationPose(
                model_pts=self.mesh.vertices,
                model_normals=self.mesh.vertex_normals,
                mesh=self.mesh,
                scorer=scorer,
                refiner=refiner,
                debug_dir=self.debug_dir,
                debug=self.debug,
                glctx=glctx,
            )

            self.get_logger().info("FoundationPose estimator initialized successfully")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize FoundationPose: {str(e)}")
            raise

    def camera_info_callback(self, msg):
        """Callback for camera information"""
        try:
            # Extract intrinsic matrix K
            self.camera_K = np.array(msg.k).reshape(3, 3)
            self.camera_info_received = True
        except Exception as e:
            self.get_logger().error(f"Error processing camera info: {str(e)}")

    def rgb_callback(self, msg):
        """Callback for RGB image"""
        try:
            with self.lock:
                self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                self.rgb_received = True
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {str(e)}")

    def depth_callback(self, msg):
        """Callback for depth image"""
        try:
            with self.lock:
                # Convert depth to numpy array
                if msg.encoding == "16UC1":
                    depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    self.latest_depth = depth.astype(np.float32) / 1000.0  # mm to meters
                elif msg.encoding == "32FC1":
                    self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                else:
                    depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    self.latest_depth = depth_raw / 1000.0  # Assume mm

                self.depth_received = True
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")

    def apply_offsets(self, pose):
        """Apply position and orientation offsets to the pose"""
        # Position offset
        offset_translation = np.array([
            self.position_offset["x"],
            self.position_offset["y"],
            self.position_offset["z"]
        ])

        # Orientation offset (roll, pitch, yaw) -> rotation matrix
        cr = np.cos(self.orientation_offset["roll"])
        sr = np.sin(self.orientation_offset["roll"])
        cp = np.cos(self.orientation_offset["pitch"])
        sp = np.sin(self.orientation_offset["pitch"])
        cy = np.cos(self.orientation_offset["yaw"])
        sy = np.sin(self.orientation_offset["yaw"])

        # ZYX rotation matrix (yaw-pitch-roll)
        offset_rotation = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ])

        # Create 4x4 transformation matrix for the offset
        offset_transform = np.eye(4)
        offset_transform[:3, :3] = offset_rotation
        offset_transform[:3, 3] = offset_translation

        # Apply offset: final_pose = estimated_pose @ offset
        pose_with_offset = pose @ offset_transform

        return pose_with_offset

    def publish_tf(self, pose, timestamp):
        """Publish the pose as a TF"""
        try:
            t = TransformStamped()

            t.header.stamp = timestamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = self.object_frame

            # Extract translation
            t.transform.translation.x = float(pose[0, 3])
            t.transform.translation.y = float(pose[1, 3])
            t.transform.translation.z = float(pose[2, 3])

            # Extract rotation (convert matrix to quaternion)
            rotation_matrix = pose[:3, :3]
            quaternion = R.from_matrix(rotation_matrix).as_quat()

            t.transform.rotation.x = float(quaternion[0])
            t.transform.rotation.y = float(quaternion[1])
            t.transform.rotation.z = float(quaternion[2])
            t.transform.rotation.w = float(quaternion[3])

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Error publishing TF: {str(e)}")
            raise

    def visualize_pose(self, image, center_pose):
        """Visualize the estimated pose (debug only)"""
        vis = draw_posed_3d_box(self.camera_K, img=image, ob_in_cam=center_pose, bbox=self.bbox)
        vis = draw_xyz_axis(
            vis,
            ob_in_cam=center_pose,
            scale=0.1,
            K=self.camera_K,
            thickness=3,
            transparency=0,
            is_input_rgb=True
        )
        return vis

    def add_buttons_to_image(self, image):
        """Add Accept/Reject buttons to the image"""
        h, w = image.shape[:2]
        button_height = 60
        button_width = 200
        margin = 20

        # Create a copy of the image to draw the buttons
        img_with_buttons = image.copy()

        # Button positions (bottom area)
        accept_x = w // 2 - button_width - margin // 2
        reject_x = w // 2 + margin // 2
        button_y = h - button_height - margin

        # Accept button (green)
        cv2.rectangle(
            img_with_buttons,
            (accept_x, button_y),
            (accept_x + button_width, button_y + button_height),
            (0, 255, 0), -1
        )
        cv2.rectangle(
            img_with_buttons,
            (accept_x, button_y),
            (accept_x + button_width, button_y + button_height),
            (0, 200, 0), 3
        )

        # Reject button (red)
        cv2.rectangle(
            img_with_buttons,
            (reject_x, button_y),
            (reject_x + button_width, button_y + button_height),
            (0, 0, 255), -1
        )
        cv2.rectangle(
            img_with_buttons,
            (reject_x, button_y),
            (reject_x + button_width, button_y + button_height),
            (0, 0, 200), 3
        )

        # Button text
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        font_thickness = 2

        cv2.putText(
            img_with_buttons, "ACCEPT (A)",
            (accept_x + 25, button_y + 38),
            font, font_scale, (255, 255, 255), font_thickness
        )

        cv2.putText(
            img_with_buttons, "REJECT (R)",
            (reject_x + 20, button_y + 38),
            font, font_scale, (255, 255, 255), font_thickness
        )

        # Add instructions at the top
        cv2.putText(
            img_with_buttons, "Press 'A' to Accept or 'R' to Reject",
            (20, 30),
            font, 0.7, (255, 255, 255), 2
        )

        return img_with_buttons

    def wait_for_user_decision(self, vis_image):
        """Show image with buttons and wait for user decision"""
        self.user_decision = None

        # Add buttons to the image
        img_with_buttons = self.add_buttons_to_image(vis_image)

        # Create window if it does not exist
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        while self.user_decision is None:
            cv2.imshow(self.window_name, img_with_buttons[..., ::-1])
            key = cv2.waitKey(100) & 0xFF

            if key == ord('a') or key == ord('A'):
                self.user_decision = True
                self.get_logger().info("User accepted the detection")
            elif key == ord('r') or key == ord('R'):
                self.user_decision = False
                self.get_logger().info("User rejected the detection")
            elif key == 27:  # ESC
                self.user_decision = False
                self.get_logger().info("User cancelled (ESC)")

        # Close the window after decision
        cv2.destroyWindow(self.window_name)

        return self.user_decision

    def estimate_pose_callback(self, request, response):
        """Service callback to estimate pose"""
        try:
            self.get_logger().info("Pose estimation service called")

            if not (self.rgb_received and self.depth_received and self.camera_info_received):
                response.success = False
                response.message = "RGB, Depth or Camera Info not received yet"
                self.get_logger().warning(response.message)
                return response

            # Copy data for processing
            with self.lock:
                rgb = self.latest_rgb.copy()
                depth = self.latest_depth.copy()
                K = self.camera_K.copy()

            timestamp = self.get_clock().now().to_msg()

            # Process images
            H, W = rgb.shape[:2]
            color = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_NEAREST)
            depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_NEAREST)

            # Clean invalid depth values
            depth[(depth < 0.1) | (depth >= np.inf)] = 0

            # Estimate pose
            self.get_logger().info("Running pose estimation...")

            # Estimation: create automatic mask
            mask = (depth > 0.1) & (depth < 3.0)  # Valid depth range

            pose = self.pose_estimator.register(
                K=K,
                rgb=color,
                depth=depth,
                ob_mask=mask,
                iteration=self.est_refine_iter
            )

            self.get_logger().info("Pose registered")

            # Apply transformation to object center
            center_pose = pose @ np.linalg.inv(self.to_origin)

            # Apply offsets
            pose_with_offset = self.apply_offsets(center_pose)

            # Visualization and wait for user confirmation
            vis = self.visualize_pose(color, center_pose)
            user_accepted = self.wait_for_user_decision(vis)

            if user_accepted:
                # Publish TF only if user accepted
                self.publish_tf(pose_with_offset, timestamp)

                # Save result if debug is enabled
                if self.debug >= 2:
                    debug_file = os.path.join(self.debug_dir, "latest_pose.txt")
                    np.savetxt(debug_file, pose_with_offset)

                response.success = True
                response.message = "Pose accepted by user, TF published"
                self.get_logger().info(response.message)
            else:
                # User rejected the detection
                response.success = False
                response.message = "Pose rejected by user"
                self.get_logger().warning(response.message)

            return response

        except Exception as e:
            response.success = False
            response.message = f"Error during pose estimation: {str(e)}"
            self.get_logger().error(response.message)
            return response


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PoseEstimatorService()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()