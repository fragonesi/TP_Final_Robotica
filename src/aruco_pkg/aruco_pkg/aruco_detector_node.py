import csv
import os

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('image_topic', '/oakd/rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/camera_info')
        self.declare_parameter('use_camera_info_topic', True)
        self.declare_parameter('calib_yaml_path', '')
        self.declare_parameter('marker_length', 0.15)  # metros, lado del tag (¡MEDIRLO!)
        self.declare_parameter('aruco_dictionary', 'DICT_5X5_250')
        self.declare_parameter('log_csv_path', 'aruco_detections.csv')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('upscale_factor', 4.0)
        self.declare_parameter('use_clahe', False)

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.use_camera_info_topic = self.get_parameter('use_camera_info_topic').value
        calib_yaml_path = self.get_parameter('calib_yaml_path').value
        self.marker_length = float(self.get_parameter('marker_length').value)
        dict_name = self.get_parameter('aruco_dictionary').value
        self.log_csv_path = self.get_parameter('log_csv_path').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        self.upscale_factor = float(self.get_parameter('upscale_factor').value)
        self.use_clahe = self.get_parameter('use_clahe').value

        self.K_raw = None
        self.K_scaled = None
        self.dist_coeffs = None

        if not self.use_camera_info_topic:
            if not calib_yaml_path or not os.path.isfile(calib_yaml_path):
                self.get_logger().error(
                    'use_camera_info_topic=False pero calib_yaml_path no es válido. '
                    'Pasen -p calib_yaml_path:=/ruta/a/archivo.yaml con K y dist_coeffs.'
                )
            else:
                self._load_calibration_yaml(calib_yaml_path)

        try:
            dict_id = getattr(cv2.aruco, dict_name)
        except AttributeError:
            self.get_logger().warn(f'Diccionario {dict_name} no encontrado, uso DICT_5X5_250')
            dict_id = cv2.aruco.DICT_5X5_250

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.minMarkerPerimeterRate = 0.01
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.errorCorrectionRate = 0.8
        self.aruco_params.perspectiveRemovePixelPerCell = 8
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.20

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self.bridge = CvBridge()

        if self.use_camera_info_topic:
            self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_cb, 10)

        self.create_subscription(Image, image_topic, self.image_cb, 10)

        self.landmarks_pub = self.create_publisher(PoseArray, '/landmarks_camera_frame', 10)

        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, '/aruco_detections/image', 10)

        self._init_csv()

        self.frame_count = 0
        self.frames_with_candidates = 0
        self.frames_with_decoded = 0

        self.get_logger().info(
            f'aruco_detector_node listo. image_topic={image_topic}, '
            f'camera_info_topic={camera_info_topic if self.use_camera_info_topic else "(desactivado, uso YAML)"}, '
            f'marker_length={self.marker_length} m, upscale_factor={self.upscale_factor}, '
            f'use_clahe={self.use_clahe}'
        )

    def _load_calibration_yaml(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self.K_raw = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(
            data['distortion_coefficients']['data'], dtype=np.float64
        ).reshape(1, -1)
        self._update_scaled_K()
        self.get_logger().info(f'Calibración cargada desde {path}')

    def camera_info_cb(self, msg: CameraInfo):
        if self.K_raw is None:
            self.K_raw = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(1, -1)
            self._update_scaled_K()
            self.get_logger().info('Calibración recibida desde tópico camera_info.')

    def _update_scaled_K(self):
        if self.K_raw is None:
            return
        K = self.K_raw.copy()
        s = self.upscale_factor
        K[0, 0] *= s
        K[1, 1] *= s
        K[0, 2] *= s
        K[1, 2] *= s
        self.K_scaled = K

    def _init_csv(self):
        is_new = not os.path.isfile(self.log_csv_path)
        self.csv_file = open(self.log_csv_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if is_new:
            self.csv_writer.writerow(
                ['timestamp', 'marker_id', 'tx', 'ty', 'tz', 'distancia_m',
                 'rx', 'ry', 'rz']
            )

    def image_cb(self, msg: Image):
        if self.K_scaled is None:
            self.get_logger().warn(
                'Todavía no tengo K/dist_coeffs, descarto frame.', throttle_duration_sec=2.0
            )
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.upscale_factor != 1.0:
            frame_proc = cv2.resize(
                frame, None, fx=self.upscale_factor, fy=self.upscale_factor,
                interpolation=cv2.INTER_LANCZOS4
            )
        else:
            frame_proc = frame

        gray = cv2.cvtColor(frame_proc, cv2.COLOR_BGR2GRAY)

        if self.use_clahe:
            gray = self.clahe.apply(gray)

        corners, ids, rejected = self.detector.detectMarkers(gray)

        self.frame_count += 1
        n_candidates = len(rejected) if rejected is not None else 0
        n_decoded = len(ids) if ids is not None else 0
        if n_candidates > 0 or n_decoded > 0:
            self.frames_with_candidates += 1
        if n_decoded > 0:
            self.frames_with_decoded += 1

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'[DIAGNÓSTICO] frames procesados={self.frame_count}, '
                f'con candidatos (cuadrados detectados)={self.frames_with_candidates}, '
                f'con ID decodificado={self.frames_with_decoded}'
            )

        pose_array = PoseArray()
        pose_array.header = msg.header

        if ids is not None and len(ids) > 0:
            half = self.marker_length / 2.0
            obj_points = np.array([
                [-half,  half, 0],
                [ half,  half, 0],
                [ half, -half, 0],
                [-half, -half, 0],
            ], dtype=np.float64)

            for i, marker_id in enumerate(ids.flatten()):
                img_points = corners[i].reshape(4, 2).astype(np.float64)
                ok, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points, self.K_scaled, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not ok:
                    continue

                tvec = tvec.flatten()
                rvec = rvec.flatten()
                distancia = float(np.linalg.norm(tvec))

                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])
                pose_array.poses.append(pose)

                t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.csv_writer.writerow([
                    t_sec, int(marker_id),
                    tvec[0], tvec[1], tvec[2], distancia,
                    rvec[0], rvec[1], rvec[2],
                ])
                self.csv_file.flush()

                self.get_logger().info(
                    f'ArUco id={marker_id} detectado a {distancia:.3f} m',
                    throttle_duration_sec=0.5
                )

                if self.publish_annotated:
                    cv2.drawFrameAxes(
                        frame_proc, self.K_scaled, self.dist_coeffs, rvec, tvec,
                        self.marker_length * 0.5
                    )

            if self.publish_annotated:
                cv2.aruco.drawDetectedMarkers(frame_proc, corners, ids)

        self.landmarks_pub.publish(pose_array)

        if self.publish_annotated:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame_proc, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

    def destroy_node(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()