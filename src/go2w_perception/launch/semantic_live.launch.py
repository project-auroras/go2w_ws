from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Navegación semántica con la RealSense real: YOLO (modelo propio) + mapa semántico.

    Como people_live.launch.py: la cámara está en la NX, que manda la imagen comprimida;
    aquí se descomprime y yolo_node la procesa con models/yolo26m_datasetFinal.onnx.
    semantic_mapper, semantic_goal y row_follower trabajan en el frame map, que publica
    EasyNav (en la NX). Sin EasyNav, map_tf:=true publica map = odom (lo mismo que hace
    EasyNav con easynav_real.params.yaml) para poder ver y guardar el mapa semántico.
    No lanzar map_tf:=true a la vez que EasyNav.
    """
    pkg = FindPackageShare('go2w_perception')
    cam = '/camera/camera'
    semantic_params = PathJoinSubstitution([pkg, 'config', 'semantic_nav.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('map_tf', default_value='false',
                              description='true: TF map = odom, para usarlo SIN EasyNav'),
        DeclareLaunchArgument('semantic_map', default_value='',
                              description='semantic_*.yaml para empezar con ese mapa'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('yolo_params', default_value=PathJoinSubstitution(
            [pkg, 'config', 'yolo_semantic_live.yaml'])),

        # JPEG -> imagen (color)
        Node(
            package='image_transport', executable='republish', name='color_decompress',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', f'{cam}/color/image_raw/compressed'),
                        ('out', '/camera_local/color/image_raw')],
        ),
        # PNG -> profundidad alineada al color (ver people_live.launch.py)
        Node(
            package='go2w_perception', executable='depth_decompress', output='screen',
            parameters=[{
                'input_topic': f'{cam}/aligned_depth_to_color/image_raw/compressedDepth',
                'output_topic': '/camera_local/aligned_depth/image_raw',
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('yolo_bringup'), 'launch', 'yolo.launch.py'])),
            launch_arguments={'params_file': LaunchConfiguration('yolo_params')}.items()),

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
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'semantic_live.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
