"""Lanza los nodos de percepción/registro para procesar el bag del laberinto.

En otra terminal, reproducir el bag:   ros2 bag play <carpeta_del_bag>

Genera los CSV (odom_deltas.csv, laberinto_detections.csv, scans.csv) que luego
consume el pipeline de GraphSLAM. Los CSV se escriben en el directorio de trabajo.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Detección de ArUco. El diccionario correcto es DICT_4X4_50.
        Node(
            package='aruco_pkg',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[{
                'image_topic': '/tb4_0/oakd/rgb/preview/image_raw',
                'camera_info_topic': '/tb4_0/oakd/rgb/preview/camera_info',
                'aruco_dictionary': 'DICT_4X4_50',
                'upscale_factor': 4.0,
                'use_clahe': False,
                'log_csv_path': 'laberinto_detections.csv',
            }],
        ),
        # Odometría → deltas (rot1, trans, rot2).
        Node(
            package='aruco_pkg',
            executable='odom_delta_node',
            name='odom_delta_node',
            output='screen',
            parameters=[{'odom_topic': '/tb4_0/odom', 'log_csv_path': 'odom_deltas.csv'}],
        ),
        # LIDAR → CSV para la 2da pasada (grilla de ocupación).
        Node(
            package='TP_Final_Robotica',
            executable='scan_logger_node',
            name='scan_logger_node',
            output='screen',
            parameters=[{'scan_topic': '/tb4_0/scan', 'log_csv_path': 'scans.csv'}],
        ),
    ])
