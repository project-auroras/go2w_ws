from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    sim = LaunchConfiguration('sim')
    # The simulation has no LiDAR: wheel odometry there
    odom_source = PythonExpression(
        ["'wheels' if '", sim, "' == 'true' else '", LaunchConfiguration('odom_source'), "'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('go2w_driver'), 'config', 'go2w_driver.yaml'])),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='Run sim_go2w instead of the real robot'),
        DeclareLaunchArgument(
            'odom_source', default_value='lio',
            description='lio, wheels, sportmodestate or robot_pose (always wheels in sim)'),
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
                    parameters=[params, {'odom_source': odom_source}],
                ),
                ComposableNode(
                    package='go2w_driver',
                    plugin='go2w_driver::SimGo2W',
                    name='sim_go2w',
                    # Same topic the driver reads on the robot (sport_state_topic)
                    remappings=[('sportmodestate', '/lf/sportmodestate')],
                    condition=IfCondition(sim),
                ),
            ],
        ),
    ])
