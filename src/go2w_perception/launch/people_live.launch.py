from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """YOLO26 con la RealSense real, ejecutado en el portátil (la cámara está en la NX).

    En la NX: go2w_bringup (TF del robot, con la cámara en el URDF) + realsense2_camera
    con align_depth. Por WiFi viajan la imagen en JPEG y la profundidad en PNG; aquí se
    descomprimen, YOLO detecta y person_filter publica las personas en /people
    (en base_link). Con follower:=true, person_follower manda /cmd_vel al robot.
    """
    pkg = FindPackageShare('go2w_perception')
    cam = '/camera/camera'

    return LaunchDescription([
        DeclareLaunchArgument('follower', default_value='false',
                              description='true: el robot sigue a la persona (se MUEVE)'),
        DeclareLaunchArgument('max_linear', default_value='0.6',
                              description='m/s máximos del sigue-personas'),
        DeclareLaunchArgument('obstacle_avoidance', default_value='true',
                              description='false: no frena ante obstáculos (sin /scan)'),
        DeclareLaunchArgument('max_angular', default_value='1.2',
                              description='rad/s máximos del sigue-personas'),

        # JPEG -> imagen (color)
        Node(
            package='image_transport', executable='republish', name='color_decompress',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', f'{cam}/color/image_raw/compressed'),
                        ('out', '/camera_local/color/image_raw')],
        ),
        # PNG -> profundidad (16UC1, mm), alineada al color. Nodo propio: el
        # 'republish compressedDepth raw' de Humble falla al suscribirse.
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
            launch_arguments={'params_file': PathJoinSubstitution(
                [pkg, 'config', 'yolo_live.yaml'])}.items()),

        Node(package='go2w_perception', executable='person_filter', output='screen'),

        Node(
            package='go2w_perception', executable='person_follower', output='screen',
            parameters=[PathJoinSubstitution([pkg, 'config', 'person_follower.yaml']),
                        {'max_linear': LaunchConfiguration('max_linear'),
                         'max_angular': LaunchConfiguration('max_angular'),
                         'obstacle_avoidance': ParameterValue(
                             LaunchConfiguration('obstacle_avoidance'), value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('follower')),
        ),
    ])
