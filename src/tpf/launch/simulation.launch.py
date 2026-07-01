from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('tpf'),
        'rviz',
        'tp_final.rviz'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_custom_simulation'),
            'launch',
            'custom_casa.launch.py'
        ))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    likelihood_node = Node(
        package='tpf',
        executable='likelihood_field_node',
        name='likelihood_map_publisher',
        output='screen'
    )

    localization_node = Node(
        package='tpf',
        executable='localization_node',
        name='localization_node',
        output='screen'
    )

    # map_publisher va último con delay para que todos los subscribers estén listos
    map_publisher_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='tpf',
            executable='map_publisher',
            name='map_publisher',
            output='screen'
        )]
    )

    robot_node = Node(
        package='tpf',
        executable='robot_node',
        name='robot_navigator',
        output='screen'
    )

    return LaunchDescription([
    gazebo_launch,
    TimerAction(
        period=5.0,
        actions=[
            rviz_node,
            likelihood_node,
            localization_node,
            robot_node,
        ]
    ),
    TimerAction(
        period=10.0,
        actions=[map_publisher_node]
    )])