"""
Launch file para Parte C — TurtleBot4 real.

Corre en paralelo con:
  - ros2 bag play laberinto_conos  (o el robot real)
  - ros2 run cono_detector_pkg detector_cono

Nodos lanzados:
  1. likelihood_field_node  — genera el mapa de likelihood a partir del /map
  2. localization_node_tb4  — filtro de partículas con topics /tb4_0/*
  3. map_publisher          — publica /map desde el archivo .pgm/.yaml
  4. robot_node_c           — FSM de Parte C con exploración + búsqueda de conos
  5. rviz2                  — visualización (misma config que Parte B)

El map_publisher se retrasa 5 s para asegurarse de que los demás nodos ya
están escuchando antes de publicar el mapa con QoS TRANSIENT_LOCAL.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('tpf')

    rviz_config = os.path.join(pkg, 'rviz', 'tp_final.rviz')

    likelihood_node = Node(
        package='tpf',
        executable='likelihood_field_node',
        name='likelihood_map_publisher',
        output='screen',
    )

    localization_node = Node(
        package='tpf',
        executable='localization_node_tb4',
        name='localization_node',
        output='screen',
    )

    robot_node_c = Node(
        package='tpf',
        executable='robot_node_c',
        name='robot_navigator_c',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    map_publisher_node = Node(
        package='tpf',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
    )

    return LaunchDescription([
        # Arranca inmediatamente: likelihood y localización necesitan estar
        # listos antes de que llegue el primer /map
        likelihood_node,
        localization_node,
        robot_node_c,
        rviz_node,

        # El map_publisher se retrasa para que los suscriptores TRANSIENT_LOCAL
        # ya existan cuando el mapa se publique por primera vez
        TimerAction(period=5.0, actions=[map_publisher_node]),
    ])
