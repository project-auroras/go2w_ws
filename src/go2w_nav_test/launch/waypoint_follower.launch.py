from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('go2w_nav_test'), 'config', 'waypoint_follower.yaml'])),

        Node(
            package='go2w_nav_test',
            executable='waypoint_follower_node',
            name='waypoint_follower',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
