from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Stack de la Parte B contra el TurtleBot4 real: mismos nodos que la
    simulación pero remapeados a los tópicos /tb4_0/* (sin Gazebo)."""
    rviz_config = os.path.join(
        get_package_share_directory('navegacion_pkg'), 'rviz', 'tp_final.rviz'
    )

    map_publisher_node = Node(
        package='navegacion_pkg',
        executable='map_publisher',
        name='map_publisher',
        output='screen'
    )

    likelihood_node = Node(
        package='navegacion_pkg',
        executable='likelihood_field_node',
        name='likelihood_map_publisher',
        output='screen'
    )

    localization_node = Node(
        package='navegacion_pkg',
        executable='localization_node',
        name='localization_node',
        output='screen',
        remappings=[
            ('/calc_odom', '/tb4_0/odom'),
            ('/scan',      '/tb4_0/scan'),
        ]
    )

    robot_node = Node(
        package='navegacion_pkg',
        executable='robot_node',
        name='robot_navigator',
        output='screen',
        remappings=[
            ('/scan',    '/tb4_0/scan'),
            ('/cmd_vel', '/tb4_0/cmd_vel'),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        remappings=[
            ('/scan', '/tb4_0/scan'),
        ]
    )

    return LaunchDescription([
        map_publisher_node,
        TimerAction(period=3.0, actions=[
            likelihood_node,
            localization_node,
            robot_node,
            rviz_node,
        ])
    ])
