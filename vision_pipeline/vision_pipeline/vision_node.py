# -----------------------------------------------------------------------------
# Copyright © 2026 NVIDIA Corporation
#
# This file is part of a software that integrates NVIDIA FoundationPose.
#
# FoundationPose and related components are provided under the
# NVIDIA Source Code License. Use, reproduction, modification, and
# distribution of this software is governed by the terms of that license.
# See the LICENSE file in the root of the repository or:
#
#   https://github.com/NVlabs/FoundationPose/blob/main/LICENSE
#
# Before distributing this software or creating derivative works,
# ensure that you have read and accepted the NVIDIA Source Code License.
# -----------------------------------------------------------------------------



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
from ament_index_python.packages import get_package_share_directory
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
from ultralytics import SAM
import open3d as o3d
from sklearn.linear_model import RANSACRegressor


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

        # Initialize SAM2 segmentation model
        package_share_dir = get_package_share_directory('vision_pipeline')
        model_path = os.path.join(package_share_dir, 'models', 'sam2.1_b.pt')
        self.get_logger().info(f"Loading SAM model from: {model_path}")
        self.seg_model = SAM(model_path)

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
    # Plane detection utility
    # --------------------------------------------------------            
    def detect_table_plane(self, depth, K, height_threshold=0.02):
        """
        Detecta el plano de la mesa y devuelve máscara de objetos sobre ella.
        
        Args:
            depth: imagen de profundidad
            K: matriz intrínseca de la cámara
            height_threshold: altura mínima sobre la mesa (metros)
        
        Returns:
            mask: máscara booleana de píxeles sobre la mesa
        """
        
        # Crear nube de puntos desde depth
        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        
        # Backproject a 3D
        z = depth
        x = (u - K[0, 2]) * z / K[0, 0]
        y = (v - K[1, 2]) * z / K[1, 1]
        
        # Filtrar puntos válidos
        valid = z > 0
        points_3d = np.stack([x[valid], y[valid], z[valid]], axis=-1)
        
        # RANSAC para detectar plano dominante
        ransac = RANSACRegressor(residual_threshold=0.01, max_trials=1000)
        X = points_3d[:, [0, 1]]  # x, y como features
        y_target = points_3d[:, 2]  # z como target
        
        ransac.fit(X, y_target)
        
        # Predecir altura del plano en cada píxel
        plane_z = ransac.predict(np.stack([x, y], axis=-1).reshape(-1, 2))
        plane_z = plane_z.reshape(h, w)
        
        # Máscara: puntos que están "height_threshold" por encima del plano
        above_plane = (z > 0) & (z < plane_z - height_threshold)
        
        return above_plane

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

            H, W = rgb.shape[:2]
            color = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_NEAREST)
            depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_NEAREST)

            # Remove invalid depth values
            depth[(depth < 0.1) | (depth > 3.0)] = 0

            # above_table_mask = self.detect_table_plane(depth, K, height_threshold=0.02)
            # color_masked = color.copy()
            # color_masked[~above_table_mask] = 0
            # color_masked = color.copy()
            # color_masked[depth == 0] = 0 

            # Test SAM2
            res = self.seg_model.predict(color)[0]
            res.save("masks.png")
            masks = res.masks.data.cpu().numpy()

            def get_bbox(mask):
                """Obtiene el bounding box de una máscara binaria"""
                rows = np.any(mask, axis=1)
                cols = np.any(mask, axis=0)
                
                if not rows.any() or not cols.any():
                    return None
                
                rmin, rmax = np.where(rows)[0][[0, -1]]
                cmin, cmax = np.where(cols)[0][[0, -1]]
                
                return (rmin, rmax, cmin, cmax)  # (y_min, y_max, x_min, x_max)

            def bbox_contains_center(bbox_outer, bbox_inner):
                """Verifica si el centro del bbox_inner está dentro del bbox_outer"""
                y_min_out, y_max_out, x_min_out, x_max_out = bbox_outer
                y_min_in, y_max_in, x_min_in, x_max_in = bbox_inner
                
                # Centro del bbox interior
                center_y = (y_min_in + y_max_in) / 2
                center_x = (x_min_in + x_max_in) / 2
                
                # Verificar si el centro está dentro del bbox exterior
                return (y_min_out <= center_y <= y_max_out and 
                        x_min_out <= center_x <= x_max_out)

            # PASO 1: Encontrar la máscara más grande (la mesa)
            areas = [mask.sum() for mask in masks]
            largest_idx = np.argmax(areas)
            table_mask = masks[largest_idx].astype(bool)
            table_bbox = get_bbox(table_mask)

            if table_bbox is None:
                self.get_logger().error("No se pudo calcular el bbox de la mesa")
                return

            # PASO 2: Encontrar máscaras cuyo centro está DENTRO del bbox de la mesa
            masks_inside_table = []
            for i, mask in enumerate(masks):
                if i == largest_idx:  # Saltar la propia mesa
                    continue
                
                mask_bool = mask.astype(bool)
                mask_bbox = get_bbox(mask_bool)
                
                if mask_bbox is None:
                    continue
                
                # Verificar si el centro de esta máscara está dentro del bbox de la mesa
                if bbox_contains_center(table_bbox, mask_bbox):
                    mask_area = mask_bool.sum()
                    masks_inside_table.append((mask_bool, mask_area, mask_bbox))

            # PASO 3: De las máscaras dentro de la mesa, quedarse con la más grande
            if len(masks_inside_table) > 0:
                # Ordenar por área y tomar la mayor
                masks_inside_table.sort(key=lambda x: x[1], reverse=True)
                object_mask = masks_inside_table[0][0]
                self.get_logger().info(f"Objeto encontrado con área: {masks_inside_table[0][1]}")
            else:
                self.get_logger().warn("No se encontró ningún objeto dentro de la mesa")
                # Fallback: usar la segunda máscara más grande
                sorted_indices = np.argsort(areas)[::-1]
                if len(sorted_indices) > 1:
                    object_mask = masks[sorted_indices[1]].astype(bool)
                else:
                    self.get_logger().error("No hay suficientes máscaras")
                    return
            # Combinar con depth válida
            obj_mask = object_mask & (depth > 0)

            # Debug: guardar ambas máscaras
            cv2.imwrite("table_mask.png", (table_mask * 255).astype(np.uint8))
            cv2.imwrite("selected_mask.png", (obj_mask * 255).astype(np.uint8))

            # h, w = masks.shape[1:]
            # center_y, center_x = h // 2, w // 2

            # best_score = -1
            # best_mask = None

            # for mask in masks:
            #     area = mask.sum()
                
            #     # Calcular centroide de la máscara
            #     y_indices, x_indices = np.where(mask)
            #     if len(y_indices) == 0:
            #         continue
                
            #     centroid_y = y_indices.mean()
            #     centroid_x = x_indices.mean()
                
            #     # Distancia al centro
            #     dist_to_center = np.sqrt((centroid_x - center_x)**2 + (centroid_y - center_y)**2)
                
            #     # Score combinado: área grande + cerca del centro
            #     score = area / (1 + dist_to_center * 0.01)  # Ajustar peso
                
            #     if score > best_score:
            #         best_score = score
            #         best_mask = mask

            # sam_mask = best_mask.astype(bool)
            # obj_mask = sam_mask & (depth > 0)

            # # Save mask for debugging
            # cv2.imwrite("selected_mask.png", (obj_mask * 255).astype(np.uint8)) 

            # Run pose estimation
            pose = self.pose_estimator.register(
                K=K,
                rgb=rgb,
                depth=depth,
                ob_mask=obj_mask,
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
