import sys
import os

# Configurar path para FoundationPose
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
        super().__init__("pose_estimator_service")

        self.declare_parameter("object_frame", "object")
        self.declare_parameter("mesh_file", "")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("debug", 0)
        self.declare_parameter("debug_dir", "/tmp/foundationpose_debug")

        self.declare_parameter("position_offset.x", 0.0)
        self.declare_parameter("position_offset.y", 0.0)
        self.declare_parameter("position_offset.z", 0.0)
        self.declare_parameter("orientation_offset.roll", 0.0)
        self.declare_parameter("orientation_offset.pitch", 0.0)
        self.declare_parameter("orientation_offset.yaw", 0.0)
        self.declare_parameter("estimation_params.est_refine_iter", 5)
        self.declare_parameter("estimation_params.track_refine_iter", 2)

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

        # Inicializar variables
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.lock = Lock()

        # Variables para almacenar imágenes y datos de cámara
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_K = None
        self.rgb_received = False
        self.depth_received = False
        self.camera_info_received = False

        # Variables de FoundationPose
        self.pose_estimator = None
        self.is_initialized = False
        self.mesh = None
        self.to_origin = None
        self.bbox = None

        # Inicializar FoundationPose
        self.initialize_foundationpose()

        # Crear servicio
        self.srv = self.create_service(Trigger, "estimate_pose", self.estimate_pose_callback)

        self.get_logger().info("Pose Estimator Service initialized and ready")

    def load_configuration(self, config_file):
        """Cargar configuración desde archivo YAML"""
        try:
            # Buscar el archivo en diferentes ubicaciones posibles
            possible_paths = [config_file, os.path.join(os.path.dirname(__file__), "..", config_file), os.path.join(os.getcwd(), config_file)]

            config_path = None
            for path in possible_paths:
                if os.path.exists(path):
                    config_path = path
                    break

            if config_path is None:
                self.get_logger().warn(f"Configuration file not found, using defaults")
                self.use_default_configuration()
                return

            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            # Cargar parámetros del YAML
            self.object_frame = config.get("object_frame", "object")

            # Offsets de posición (metros)
            position_offset = config.get("position_offset", {})
            self.pos_offset_x = position_offset.get("x", 0.0)
            self.pos_offset_y = position_offset.get("y", 0.0)
            self.pos_offset_z = position_offset.get("z", 0.0)

            # Offsets de orientación (grados)
            orientation_offset = config.get("orientation_offset", {})
            self.rot_offset_roll = np.deg2rad(orientation_offset.get("roll", 0.0))
            self.rot_offset_pitch = np.deg2rad(orientation_offset.get("pitch", 0.0))
            self.rot_offset_yaw = np.deg2rad(orientation_offset.get("yaw", 0.0))

            # Mesh file (puede sobreescribirse con parámetro ROS)
            if not self.mesh_file:
                self.mesh_file = config.get("mesh_file", "")

            # Parámetros de estimación (pueden sobreescribirse con parámetros ROS)
            estimation_params = config.get("estimation_params", {})
            if self.est_refine_iter == 5:  # Si es el valor por defecto
                self.est_refine_iter = estimation_params.get("est_refine_iter", 5)
            if self.track_refine_iter == 2:  # Si es el valor por defecto
                self.track_refine_iter = estimation_params.get("track_refine_iter", 2)

            self.get_logger().info(f"Configuration loaded from {config_path}")

        except Exception as e:
            self.get_logger().error(f"Error loading configuration: {str(e)}")
            exit(1)

    def initialize_foundationpose(self):
        """Inicializar el estimador FoundationPose"""
        try:
            set_logging_format()
            set_seed(0)

            if not self.mesh_file or not os.path.exists(self.mesh_file):
                raise FileNotFoundError(f"Mesh file not found: {self.mesh_file}")

            # Cargar mesh
            self.mesh = trimesh.load(self.mesh_file)

            # Preparar directorio de debug
            os.makedirs(self.debug_dir, exist_ok=True)

            # Calcular bbox y transformación al origen
            self.to_origin, extents = trimesh.bounds.oriented_bounds(self.mesh)
            self.bbox = np.stack([-extents / 2, extents / 2], axis=0).reshape(2, 3)

            # Inicializar componentes de FoundationPose
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
        """Callback para información de cámara"""
        try:
            # Extraer matriz intrínseca K
            self.camera_K = np.array(msg.k).reshape(3, 3)
            self.camera_info_received = True
        except Exception as e:
            self.get_logger().error(f"Error processing camera info: {str(e)}")

    def rgb_callback(self, msg):
        """Callback para imagen RGB"""
        try:
            with self.lock:
                self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                self.rgb_received = True
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {str(e)}")

    def depth_callback(self, msg):
        """Callback para imagen de profundidad"""
        try:
            with self.lock:
                # Convertir depth a numpy array
                if msg.encoding == "16UC1":
                    depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    self.latest_depth = depth.astype(np.float32) / 1000.0  # mm a metros
                elif msg.encoding == "32FC1":
                    self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                else:
                    depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    self.latest_depth = depth_raw / 1000.0  # Asumir mm

                self.depth_received = True
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")

    def apply_offsets(self, pose):
        """Aplicar offsets de posición y orientación a la pose"""
        # Offset de posición
        offset_translation = np.array([self.pos_offset_x, self.pos_offset_y, self.pos_offset_z])

        # Offset de orientación (roll, pitch, yaw) -> matriz de rotación
        cr = np.cos(self.rot_offset_roll)
        sr = np.sin(self.rot_offset_roll)
        cp = np.cos(self.rot_offset_pitch)
        sp = np.sin(self.rot_offset_pitch)
        cy = np.cos(self.rot_offset_yaw)
        sy = np.sin(self.rot_offset_yaw)

        # Matriz de rotación ZYX (yaw-pitch-roll)
        offset_rotation = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])

        # Crear matriz de transformación 4x4 para el offset
        offset_transform = np.eye(4)
        offset_transform[:3, :3] = offset_rotation
        offset_transform[:3, 3] = offset_translation

        # Aplicar offset: pose_final = pose_estimada @ offset
        pose_with_offset = pose @ offset_transform

        return pose_with_offset

    def publish_tf(self, pose, timestamp):
        """Publicar la pose como TF"""
        try:
            t = TransformStamped()

            t.header.stamp = timestamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = self.object_frame

            # Extraer traslación
            t.transform.translation.x = float(pose[0, 3])
            t.transform.translation.y = float(pose[1, 3])
            t.transform.translation.z = float(pose[2, 3])

            # Extraer rotación (convertir matriz a cuaternión)
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
        """Visualizar la pose estimada (solo para debug)"""
        vis = draw_posed_3d_box(self.camera_K, img=image, ob_in_cam=center_pose, bbox=self.bbox)
        vis = draw_xyz_axis(vis, ob_in_cam=center_pose, scale=0.1, K=self.camera_K, thickness=3, transparency=0, is_input_rgb=True)
        return vis

    def estimate_pose_callback(self, request, response):
        """Callback del servicio para estimar pose"""
        try:
            self.get_logger().info("Pose estimation service called")

            # Resetear flags
            with self.lock:
                self.rgb_received = False
                self.depth_received = False
                if not self.camera_info_received:
                    self.get_logger().info("Waiting for camera info...")

            # Crear suscriptores temporales
            rgb_sub = self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)

            depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

            if not self.camera_info_received:
                camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

            # Esperar a recibir imágenes (timeout de 5 segundos)
            timeout = 5.0
            start_time = self.get_clock().now()

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

                with self.lock:
                    if self.rgb_received and self.depth_received and self.camera_info_received:
                        break

                if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                    response.success = False
                    response.message = "Timeout waiting for camera data"
                    self.get_logger().error(response.message)
                    return response

            # Copiar datos para procesamiento
            with self.lock:
                rgb = self.latest_rgb.copy()
                depth = self.latest_depth.copy()
                K = self.camera_K.copy()

            # Destruir suscriptores temporales
            self.destroy_subscription(rgb_sub)
            self.destroy_subscription(depth_sub)

            timestamp = self.get_clock().now().to_msg()

            # Procesar imágenes
            H, W = rgb.shape[:2]
            color = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_NEAREST)
            depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_NEAREST)

            # Limpiar valores inválidos de profundidad
            depth[(depth < 0.1) | (depth >= np.inf)] = 0

            # Estimar pose
            self.get_logger().info("Running pose estimation...")

            if not self.is_initialized:
                # Primera estimación: crear máscara automática
                mask = (depth > 0.1) & (depth < 3.0)  # Rango de profundidad válida

                pose = self.pose_estimator.register(K=K, rgb=color, depth=depth, ob_mask=mask, iteration=self.est_refine_iter)

                self.is_initialized = True
                self.get_logger().info("Initial pose registered")
            else:
                # Tracking subsiguiente
                pose = self.pose_estimator.track_one(rgb=color, depth=depth, K=K, iteration=self.track_refine_iter)

                self.get_logger().info("Pose tracked")

            # Aplicar transformación al centro del objeto
            center_pose = pose @ np.linalg.inv(self.to_origin)

            # Aplicar offsets
            pose_with_offset = self.apply_offsets(center_pose)

            # Publicar TF
            self.publish_tf(pose_with_offset, timestamp)

            # Visualización para debug
            if self.debug >= 1:
                vis = self.visualize_pose(color, center_pose)
                cv2.imshow("Pose Estimation", vis[..., ::-1])
                cv2.waitKey(1)

            # Guardar resultado si debug está activo
            if self.debug >= 2:
                debug_file = os.path.join(self.debug_dir, "latest_pose.txt")
                np.savetxt(debug_file, pose_with_offset)

            response.success = True
            response.message = "Pose obtained and published"
            self.get_logger().info(response.message)

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
