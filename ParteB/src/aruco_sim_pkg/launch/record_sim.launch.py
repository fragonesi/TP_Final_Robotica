"""Lanza los nodos de registro de Sistema 3 contra una simulacion corriendo.

Sensor virtual de landmarks + loggers de odom/scan, contra
custom_casa.launch.py o custom_casa_obs.launch.py ya lanzados.

Uso:
    ros2 launch turtlebot3_custom_simulation custom_casa.launch.py   # terminal 1
    ros2 launch aruco_sim_pkg record_sim.launch.py world_name:=casa  # terminal 2
    # (o world_name:=casa_o si se lanzo custom_casa_obs.launch.py)

Genera, en el directorio de trabajo, los mismos 3 CSV que el pipeline de
Parte A: aruco_detections_sim.csv, odom_deltas_sim.csv, scans_sim.csv --
listos para `slam_pipeline.py --odom ... --aruco ... --scans ...`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='casa',
        description="Layout de obstaculos para la oclusion: 'casa' o 'casa_o'")
    out_dir_arg = DeclareLaunchArgument(
        'out_dir', default_value='.',
        description='Directorio donde se escriben los CSV de salida')

    world_name = LaunchConfiguration('world_name')
    out_dir = LaunchConfiguration('out_dir')

    return LaunchDescription([
        world_name_arg,
        out_dir_arg,
        Node(
            package='aruco_sim_pkg',
            executable='virtual_aruco_sensor_node',
            name='virtual_aruco_sensor_node',
            output='screen',
            parameters=[{
                'world_name': world_name,
                'log_csv_path': [out_dir, '/aruco_detections_sim.csv'],
            }],
        ),
        Node(
            package='aruco_sim_pkg',
            executable='sim_odom_delta_node',
            name='sim_odom_delta_node',
            output='screen',
            parameters=[{
                'odom_topic': '/odom',
                'log_csv_path': [out_dir, '/odom_deltas_sim.csv'],
            }],
        ),
        Node(
            package='aruco_sim_pkg',
            executable='sim_scan_logger_node',
            name='sim_scan_logger_node',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'log_csv_path': [out_dir, '/scans_sim.csv'],
            }],
        ),
    ])
