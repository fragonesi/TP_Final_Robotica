# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():

#     rviz_config = os.path.join(
#         get_package_share_directory('tpf'),
#         'rviz',
#         'tp_final.rviz'
#     )

#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(
#             get_package_share_directory('turtlebot3_custom_simulation'),
#             'launch',
#             'custom_casa.launch.py'
#         ))
#     )

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         arguments=['-d', rviz_config]
#     )

#     likelihood_node = Node(
#         package='tpf',
#         executable='likelihood_field_node',
#         name='likelihood_map_publisher',
#         output='screen'
#     )

#     localization_node = Node(
#         package='tpf',
#         executable='localization_node',
#         name='localization_node',
#         output='screen'
#     )

#     # map_publisher va último con delay para que todos los subscribers estén listos
#     map_publisher_node = TimerAction(
#         period=5.0,
#         actions=[Node(
#             package='tpf',
#             executable='map_publisher',
#             name='map_publisher',
#             output='screen'
#         )]
#     )

#     robot_node = Node(
#         package='tpf',
#         executable='robot_node',
#         name='robot_navigator',
#         output='screen'
#     )

#     return LaunchDescription([
#     gazebo_launch,
#     TimerAction(
#         period=5.0,
#         actions=[
#             rviz_node,
#             likelihood_node,
#             localization_node,
#             robot_node,
#         ]
#     ),
#     TimerAction(
#         period=10.0,
#         actions=[map_publisher_node]
#     )])



from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---------------------------------------------------------------------------
    # Argumentos de lanzamiento
    # ---------------------------------------------------------------------------
    robot_arg    = DeclareLaunchArgument('robot',    default_value='tb3',
                                         description='Tipo de robot: tb3 (sim) o tb4 (real)')
    robot_id_arg = DeclareLaunchArgument('robot_id', default_value='0',
                                         description='Número del TB4: 0 o 1')

    robot    = LaunchConfiguration('robot')
    robot_id = LaunchConfiguration('robot_id')

    # Condición: True cuando es simulación (TB3)
    is_simulation = PythonExpression(["'", robot, "' == 'tb3'"])

    # Parámetros que se pasan a los nodos
    robot_params = [{'robot': robot, 'robot_id': robot_id}]

    # ---------------------------------------------------------------------------
    # RViz
    # ---------------------------------------------------------------------------
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

    # ---------------------------------------------------------------------------
    # Gazebo — solo para TB3
    # ---------------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_custom_simulation'),
            'launch',
            'custom_casa.launch.py'
        )),
        condition=IfCondition(is_simulation),
    )

    # ---------------------------------------------------------------------------
    # Nodos propios (mismos para TB3 y TB4)
    # ---------------------------------------------------------------------------
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

    # ---------------------------------------------------------------------------
    # Descripción de lanzamiento
    # ---------------------------------------------------------------------------
    # Para TB3: se espera 5s a que Gazebo arranque antes de levantar los nodos.
    # Para TB4: no hay Gazebo; los 5s son un margen de arranque mínimo.
    return LaunchDescription([
        robot_arg,
        robot_id_arg,
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