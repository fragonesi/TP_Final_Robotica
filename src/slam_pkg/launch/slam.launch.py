"""Lanza el nodo de GraphSLAM desde los CSV y abre RViz con la config de la Parte A.

    ros2 launch slam_pkg slam.launch.py odom_csv:=/ruta/odom_deltas.csv \
        aruco_csv:=/ruta/laberinto_detections.csv \
        map_yaml_path:=/ruta/a/salida/mapa.yaml

Si no se pasa map_yaml_path (o el archivo todavía no existe, p.ej. antes de
correr slam_pipeline.py), el publicador de /map loguea un error y no publica
nada — el resto de RViz (/belief, /landmarks, /poses_guardadas) sigue andando.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom = LaunchConfiguration('odom_csv')
    aruco = LaunchConfiguration('aruco_csv')
    noise_model = LaunchConfiguration('noise_model_path')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    rviz_cfg = os.path.join(
        get_package_share_directory('slam_pkg'), 'rviz', 'slam.rviz')

    # Default: noise_model.json instalado junto con aruco_pkg (share/aruco_pkg/).
    _nm_default = os.path.join(
        get_package_share_directory('aruco_pkg'), 'noise_model.json')

    return LaunchDescription([
        DeclareLaunchArgument('odom_csv',
                              description='Ruta al CSV de odom_delta_node'),
        DeclareLaunchArgument('aruco_csv', default_value='',
                              description='Ruta al CSV de aruco_detector_node (opcional)'),
        DeclareLaunchArgument('noise_model_path', default_value=_nm_default,
                              description='JSON de modelo de ruido ArUco (fit_noise_model.py); '
                                          'vacío = usar covarianza por defecto'),
        DeclareLaunchArgument('map_yaml_path', default_value='',
                              description='mapa.yaml exportado por slam_pipeline.py (opcional; '
                                          'vacío = no publicar /map)'),
        Node(
            package='slam_pkg',
            executable='graph_slam_node',
            name='graph_slam_node',
            output='screen',
            parameters=[{'odom_csv': odom, 'aruco_csv': aruco,
                         'noise_model_path': noise_model}],
        ),
        Node(
            package='slam_pkg',
            executable='map_publisher_node',
            name='slam_map_publisher',
            output='screen',
            parameters=[{'map_yaml_path': map_yaml_path}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
    ])
