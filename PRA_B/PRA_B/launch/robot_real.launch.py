from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    robot_id_arg = DeclareLaunchArgument(
        'robot_id', default_value='0',
        description='Número del TB4 (coincide con el namespace /tb4_<id>)',
    )
    robot_id = LaunchConfiguration('robot_id')

    # Los nodos usan este parámetro para: hablar por /tb4_<id>/scan, /tb4_<id>/odom
    # y /tb4_<id>/cmd_vel, aplicar el offset de 90° del LIDAR del TB4 y filtrar
    # los rayos con intensidad 0. Sin esto corren con los defaults de TB3.
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

    # RViz no tiene el parámetro 'robot': su config trae hardcodeado '/scan',
    # así que hace falta remapearlo al tópico real del TB4.
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

    # map -> tb4_<id>/odom: sin este TF estático, RViz no puede ubicar el scan
    # ni el robot dentro del frame "map" (la localización usa /estimated_pose,
    # no TF, así que esto es solo para visualización).
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
