"""Robot model in RViz with joint sliders, no robot needed."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('go2w_description')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'description.launch.py']))),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'model.rviz']),
                       '-f', 'base_link'],
        ),
    ])
