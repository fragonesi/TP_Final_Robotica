#!/usr/bin/env python3
"""
Diagnóstico: prueba TODOS los diccionarios ArUco estándar contra cada frame
del tópico de imagen, para encontrar cuál decodifica el marcador real.

Uso:
    ros2 run aruco_pkg aruco_dict_probe --ros-args -p image_topic:=/tb4_0/oakd/rgb/preview/image_raw

O como script suelto (si no lo metés al paquete):
    python3 aruco_dict_probe.py
(en ese caso necesita rclpy igual, correrlo en el mismo entorno ROS)
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# Todos los diccionarios predefinidos relevantes de cv2.aruco
DICT_NAMES = [
    'DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_4X4_1000',
    'DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250', 'DICT_5X5_1000',
    'DICT_6X6_50', 'DICT_6X6_100', 'DICT_6X6_250', 'DICT_6X6_1000',
    'DICT_7X7_50', 'DICT_7X7_100', 'DICT_7X7_250', 'DICT_7X7_1000',
    'DICT_ARUCO_ORIGINAL',
    'DICT_APRILTAG_16h5', 'DICT_APRILTAG_25h9',
    'DICT_APRILTAG_36h10', 'DICT_APRILTAG_36h11',
]


class ArucoDictProbe(Node):
    def __init__(self):
        super().__init__('aruco_dict_probe')
        self.declare_parameter('image_topic', '/oakd/rgb/image_raw')
        self.declare_parameter('upscale_factor', 4.0)

        image_topic = self.get_parameter('image_topic').value
        self.upscale_factor = float(self.get_parameter('upscale_factor').value)

        self.bridge = CvBridge()

        self.detectors = {}
        params = cv2.aruco.DetectorParameters()
        params.minMarkerPerimeterRate = 0.01
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 4
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.errorCorrectionRate = 0.8
        params.perspectiveRemovePixelPerCell = 8
        params.perspectiveRemoveIgnoredMarginPerCell = 0.20

        for name in DICT_NAMES:
            dict_id = getattr(cv2.aruco, name)
            d = cv2.aruco.getPredefinedDictionary(dict_id)
            self.detectors[name] = cv2.aruco.ArucoDetector(d, params)

        self.found = set()
        self.frame_count = 0

        self.create_subscription(Image, image_topic, self.image_cb, 10)
        self.get_logger().info(f'Probando {len(DICT_NAMES)} diccionarios sobre {image_topic}')

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.upscale_factor != 1.0:
            frame = cv2.resize(
                frame, None, fx=self.upscale_factor, fy=self.upscale_factor,
                interpolation=cv2.INTER_LANCZOS4
            )
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        self.frame_count += 1
        for name, detector in self.detectors.items():
            if name in self.found:
                continue
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                self.found.add(name)
                self.get_logger().info(
                    f'¡MATCH! diccionario={name}, ids={ids.flatten().tolist()}, '
                    f'frame={self.frame_count}'
                )

        if self.frame_count % 60 == 0:
            self.get_logger().info(
                f'frames procesados={self.frame_count}, diccionarios con match hasta ahora={sorted(self.found)}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDictProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'RESUMEN FINAL - diccionarios que matchearon: {sorted(node.found)}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()