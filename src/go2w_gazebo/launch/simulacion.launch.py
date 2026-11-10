from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Gazebo + Go2-W con RealSense + YOLO26 (+ sigue-personas) + RViz.

    Es la misma cadena que en el robot real: cámara -> yolo_node -> person_filter
    (/people) -> person_follower (/cmd_vel). Solo cambia de dónde salen la imagen
    y a dónde va /cmd_vel (Gazebo en vez de la RealSense y go2w_driver).
    """
    gazebo = FindPackageShare('go2w_gazebo')
    perception = FindPackageShare('go2w_perception')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('follower', default_value='false',
                              description='true: el robot sigue a la persona más cercana'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: Gazebo sin ventana'),
        DeclareLaunchArgument('world', default_value='objetos',
                              description='objetos, campo o la ruta a un .sdf'),
        DeclareLaunchArgument('x', default_value='', description='vacío: según el mundo'),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [gazebo, 'launch', 'escena.launch.py'])),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'world': LaunchConfiguration('world'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
            }.items()),

        # YOLO26: detección de todas las clases + tracking + 3D en base_link
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('yolo_bringup'), 'launch', 'yolo.launch.py'])),
            launch_arguments={'params_file': PathJoinSubstitution(
                [gazebo, 'config', 'yolo_gazebo.yaml'])}.items()),

        # Solo las personas -> /people
        Node(package='go2w_perception', executable='person_filter',
             parameters=[sim_time], output='screen'),

        Node(package='go2w_perception', executable='person_follower',
             parameters=[PathJoinSubstitution([perception, 'config', 'person_follower.yaml']),
                         sim_time],
             output='screen',
             condition=IfCondition(LaunchConfiguration('follower'))),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([gazebo, 'rviz', 'simulacion.rviz'])],
             parameters=[sim_time],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
