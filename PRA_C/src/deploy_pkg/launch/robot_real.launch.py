"""
Launch file for Part C — Real TurtleBot4.

Runs in parallel with:
  - ros2 bag play laberinto_conos (or the real robot)
  - ros2 run cono_detector_pkg detector_cono

Launched nodes:
  1. likelihood_field_node  — generates the likelihood map from /map
  2. localization_node_tb4  — particle filter using /tb4_0/* topics
  3. map_publisher          — publishes /map from the .pgm/.yaml files
  4. robot_node_c           — Part C FSM with exploration and cone search
  5. rviz2                  — visualization (same configuration as Part B)

The map_publisher is delayed by 5 seconds to ensure that the other nodes
are already listening before publishing the map using TRANSIENT_LOCAL QoS.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches the nodes for Part C — Real TurtleBot4.
    """
    pkg = get_package_share_directory('deploy_pkg')
    rviz_config = os.path.join(pkg, 'rviz', 'tp_final.rviz')
    likelihood_node = Node(
        package='deploy_pkg',
        executable='likelihood_field_node',
        name='likelihood_map_publisher',
        output='screen',
    )

    localization_node = Node(
        package='deploy_pkg',
        executable='localization_node_tb4',
        name='localization_node',
        output='screen',
    )

    robot_node_c = Node(
        package='deploy_pkg',
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
        package='deploy_pkg',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
    )

    return LaunchDescription([
        # Arranca inmediatamente: likelihood y localización necesitan estar listos antes de que llegue el primer /map
        likelihood_node,
        localization_node,
        robot_node_c,
        rviz_node,

        # El map_publisher se retrasa para que los suscriptores TRANSIENT_LOCAL ya existan cuando el mapa se publique por primera vez
        TimerAction(period=5.0, actions=[map_publisher_node]),
    ])
