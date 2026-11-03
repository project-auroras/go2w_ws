from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# base_link -> camera_color_optical_frame. PROVISIONAL: posición estimada de la
# RealSense en el frontal; medir en el robot y pasarlo al URDF.
# x y z yaw pitch roll (la rotación es la de un frame óptico: z delante, x derecha)
CAMERA_TF = ['0.30', '0.0', '0.10', '-1.5708', '0.0', '-1.5708']


def generate_launch_description():
    """Sigue-personas simulado: sim_go2w + personas virtuales + person_follower + RViz."""
    pkg = FindPackageShare('go2w_perception')
    params = PathJoinSubstitution([pkg, 'config', 'person_follower.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('follower', default_value='true',
                              description='false: solo robot y personas, para lanzar el '
                                          'person_follower aparte'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('go2w_bringup'), 'launch', 'go2w.launch.py'])),
            launch_arguments={'sim': 'true', 'rviz': 'false'}.items()),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=[
                '--x', CAMERA_TF[0], '--y', CAMERA_TF[1], '--z', CAMERA_TF[2],
                '--yaw', CAMERA_TF[3], '--pitch', CAMERA_TF[4], '--roll', CAMERA_TF[5],
                '--frame-id', 'base_link', '--child-frame-id', 'camera_color_optical_frame'],
        ),

        Node(package='go2w_perception', executable='fake_person',
             parameters=[params], output='screen'),

        Node(package='go2w_perception', executable='person_follower',
             parameters=[params], output='screen',
             condition=IfCondition(LaunchConfiguration('follower'))),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'person_follower_sim.rviz'])],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
