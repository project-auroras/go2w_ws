from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Navegación semántica en Gazebo: campo + Go2-W + modelo YOLO propio + mapa semántico.

    go2w_gis.launch.py (robot con lidar, RealSense y /odom, los topics del robot real) en
    el mundo campo + yolo_node con models/yolo26m_datasetFinal.onnx + semantic_mapper
    (/semantic/landmarks) + semantic_goal (/semantic/goal_request -> /goal_pose) +
    row_follower (/semantic/row_follow) + RViz.

    EasyNav va aparte, en otra terminal:
      ros2 run easynav_system system_main --ros-args \\
        --params-file config/easynav_campo_sim.params.yaml
    """
    gazebo = FindPackageShare('go2w_gazebo')
    perception = FindPackageShare('go2w_perception')
    sim_time = {'use_sim_time': True}
    semantic_params = PathJoinSubstitution([perception, 'config', 'semantic_nav.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='campo',
                              description='campo, objetos o la ruta a un .sdf'),
        DeclareLaunchArgument('x', default_value='', description='vacío: según el mundo'),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: Gazebo sin ventana'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('yolo_params', default_value=PathJoinSubstitution(
            [gazebo, 'config', 'yolo_semantic_gazebo.yaml']),
            description='yaml de yolo_ros (modelo, umbral, cámara)'),
        DeclareLaunchArgument('semantic_map', default_value='',
                              description='semantic_*.yaml para empezar con ese mapa'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [gazebo, 'launch', 'go2w_gis.launch.py'])),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('yolo_bringup'), 'launch', 'yolo.launch.py'])),
            launch_arguments={'params_file': LaunchConfiguration('yolo_params')}.items()),

        Node(package='go2w_perception', executable='semantic_mapper',
             parameters=[semantic_params, sim_time,
                         {'load_file': LaunchConfiguration('semantic_map')}],
             output='screen'),
        Node(package='go2w_perception', executable='semantic_goal',
             parameters=[semantic_params, sim_time], output='screen'),
        Node(package='go2w_perception', executable='row_follower',
             parameters=[semantic_params, sim_time], output='screen'),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([gazebo, 'rviz', 'semantica.rviz'])],
             parameters=[sim_time],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
