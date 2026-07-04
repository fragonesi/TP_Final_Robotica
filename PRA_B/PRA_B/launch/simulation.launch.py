from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launches the nodes for Part C — Simulation or Real TurtleBot4.
    """
    robot_arg = DeclareLaunchArgument('robot', default_value='tb3', description='Tipo de robot: tb3 (sim) o tb4 (real)')
    robot_id_arg = DeclareLaunchArgument('robot_id', default_value='0', description='Número del TB4: 0 o 1')
    world_arg = DeclareLaunchArgument('world', default_value='casa', description="Mundo de Gazebo: 'casa' (limpio) u 'obs' (casa_o.world, con obstáculos no mapeados para probar la evasión)")

    robot = LaunchConfiguration('robot')
    robot_id = LaunchConfiguration('robot_id')
    world = LaunchConfiguration('world')

    is_simulation = PythonExpression(["'", robot, "' == 'tb3'"])
    gazebo_launch_file = PythonExpression(["'custom_casa_obs.launch.py' if '", world, "' == 'obs' else 'custom_casa.launch.py'"])

    robot_params = [{'robot': robot, 'robot_id': robot_id}]

    rviz_config = os.path.join(
        get_package_share_directory('tpf'),
        'rviz',
        'tp_final.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('turtlebot3_custom_simulation'),
            'launch',
            gazebo_launch_file
        ])),
        condition=IfCondition(is_simulation),
    )

    likelihood_node = Node(
        package='tpf',
        executable='likelihood_field_node',
        name='likelihood_map_publisher',
        output='screen',
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

    map_publisher_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='tpf',
            executable='map_publisher',
            name='map_publisher',
            output='screen',
        )]
    )

    odom_frame = PythonExpression(["'tb4_' + '", robot_id, "' + '/odom'"])

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame],
        condition=UnlessCondition(is_simulation),
    )

    # Para TB3: se espera 5s a que Gazebo arranque antes de levantar los nodos.
    # Para TB4: no hay Gazebo; los 5s son un margen de arranque mínimo.
    return LaunchDescription([
        robot_arg,
        robot_id_arg,
        world_arg,
        gazebo_launch,
        static_tf_node,
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
        ),
    ])