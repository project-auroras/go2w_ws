from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from go2w_perception.camera_source import camera_and_yolo, declare_source_argument


def generate_launch_description():
    """Navegación semántica con la RealSense real: YOLO (modelo propio) + mapa semántico.

    YOLO usa models/yolo26m_datasetFinal.onnx. source:=camera (NX, por defecto) | wifi
    (portátil): ver go2w_perception/camera_source.py.
    semantic_mapper, semantic_goal y row_follower trabajan en el frame map, que publica
    EasyNav (en la NX). Sin EasyNav, map_tf:=true publica map = odom (lo mismo que hace
    EasyNav con easynav_real.params.yaml) para poder ver y guardar el mapa semántico.
    No lanzar map_tf:=true a la vez que EasyNav.
    """
    pkg = FindPackageShare('go2w_perception')
    semantic_params = PathJoinSubstitution([pkg, 'config', 'semantic_nav.yaml'])

    return LaunchDescription([
        declare_source_argument(),
        DeclareLaunchArgument('yolo_params', default_value=PathJoinSubstitution(
            [pkg, 'config', 'yolo_semantic.yaml'])),
        DeclareLaunchArgument('map_tf', default_value='false',
                              description='true: TF map = odom, para usarlo SIN EasyNav'),
        DeclareLaunchArgument('semantic_map', default_value='',
                              description='semantic_*.yaml para empezar con ese mapa'),
        DeclareLaunchArgument('rviz', default_value='true'),

        camera_and_yolo(LaunchConfiguration('yolo_params')),

        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             condition=IfCondition(LaunchConfiguration('map_tf'))),

        Node(package='go2w_perception', executable='semantic_mapper', output='screen',
             parameters=[semantic_params, {'load_file': LaunchConfiguration('semantic_map')}]),
        Node(package='go2w_perception', executable='semantic_goal', output='screen',
             parameters=[semantic_params]),
        Node(package='go2w_perception', executable='row_follower', output='screen',
             parameters=[semantic_params]),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'semantic.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
