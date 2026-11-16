from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from go2w_perception.camera_source import camera_and_yolo, declare_source_argument


def generate_launch_description():
    """Detección de personas con YOLO26: yolo_node + person_filter (-> /people).

    source:=camera (NX, por defecto) | wifi (portátil) | bag (rosbags antiguos): ver
    go2w_perception/camera_source.py. Con follower:=true, person_follower manda /cmd_vel
    al robot.
    """
    pkg = FindPackageShare('go2w_perception')

    return LaunchDescription([
        declare_source_argument(),
        DeclareLaunchArgument('yolo_params', default_value=PathJoinSubstitution(
            [pkg, 'config', 'yolo_people.yaml'])),
        DeclareLaunchArgument('follower', default_value='false',
                              description='true: el robot sigue a la persona (se MUEVE)'),
        DeclareLaunchArgument('max_linear', default_value='0.6',
                              description='m/s máximos del sigue-personas'),
        DeclareLaunchArgument('max_angular', default_value='0.8',
                              description='rad/s máximos del sigue-personas'),

        camera_and_yolo(LaunchConfiguration('yolo_params')),

        Node(package='go2w_perception', executable='person_filter', output='screen'),

        Node(
            package='go2w_perception', executable='person_follower', output='screen',
            parameters=[PathJoinSubstitution([pkg, 'config', 'person_follower.yaml']),
                        {'max_linear': LaunchConfiguration('max_linear'),
                         'max_angular': LaunchConfiguration('max_angular')}],
            condition=IfCondition(LaunchConfiguration('follower')),
        ),
    ])
