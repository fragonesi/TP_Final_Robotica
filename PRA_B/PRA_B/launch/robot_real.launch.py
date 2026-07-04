from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launches the nodes for Part C — Simulation.
    """
    robot_id_arg = DeclareLaunchArgument(
        'robot_id', default_value='0',
        description='Número del TB4 (coincide con el namespace /tb4_<id>)',
    )
    robot_id = LaunchConfiguration('robot_id')
    robot_params = [{'robot': 'tb4', 'robot_id': robot_id}]

    rviz_config = os.path.join(
        get_package_share_directory('tpf'), 'rviz', 'tp_final.rviz'
    )

    scan_topic = PythonExpression(["'/tb4_' + '", robot_id, "' + '/scan'"])
    odom_frame = PythonExpression(["'tb4_' + '", robot_id, "' + '/odom'"])

    map_publisher_node = Node(
        package='tpf',
        executable='map_publisher',
        name='map_publisher',
        output='screen'
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
        output='screen',
        parameters=robot_params,
    )

    robot_node = Node(
        package='tpf',
        executable='robot_node',
        name='robot_navigator',
        output='screen',
        parameters=robot_params,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        remappings=[
            ('/scan', scan_topic),
        ]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame],
    )

    return LaunchDescription([
        robot_id_arg,
        map_publisher_node,
        static_tf_node,
        TimerAction(period=3.0, actions=[
            likelihood_node,
            localization_node,
            robot_node,
            rviz_node,
        ])
    ])
