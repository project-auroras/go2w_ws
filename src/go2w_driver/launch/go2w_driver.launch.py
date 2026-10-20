from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('go2w_driver'), 'config', 'go2w_driver.yaml'])),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='Run sim_go2w instead of the real robot'),
        ComposableNodeContainer(
            name='go2w_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='go2w_driver',
                    plugin='go2w_driver::Go2WDriver',
                    name='go2w_driver',
                    parameters=[params],
                ),
                ComposableNode(
                    package='go2w_driver',
                    plugin='go2w_driver::SimGo2W',
                    name='sim_go2w',
                    condition=IfCondition(LaunchConfiguration('sim')),
                ),
            ],
        ),
    ])
