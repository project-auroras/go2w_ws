from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('go2w_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Simulated robot (sim_go2w), no LiDAR'),
        DeclareLaunchArgument('scan', default_value='true',
                              description='Convert /utlidar/cloud to /scan'),
        DeclareLaunchArgument('lio', default_value='true',
                              description='LiDAR odometry with FAST-LIO, else wheel odometry'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('go2w_description'), 'launch', 'description.launch.py']))),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('go2w_driver'), 'launch', 'go2w_driver.launch.py'])),
            launch_arguments={
                'sim': LaunchConfiguration('sim'),
                'odom_source': PythonExpression(
                    ["'lio' if '", LaunchConfiguration('lio'), "' == 'true' else 'wheels'"]),
            }.items()),

        # /Odometry for go2w_driver (odom_source: lio). Its own launch file is not used
        # because it adds odom -> camera_init and body -> base_link TFs.
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            output='screen',
            parameters=[PathJoinSubstitution([bringup, 'config', 'fast_lio.yaml'])],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('lio'), "' == 'true' and '",
                LaunchConfiguration('sim'), "' != 'true'"])),
        ),

        # /scan: with FAST-LIO, from its last second of clouds (one L1 cloud is too sparse)
        Node(
            package='go2w_bringup',
            executable='lio_scan.py',
            name='lio_scan',
            parameters=[PathJoinSubstitution([bringup, 'config', 'lio_scan.yaml'])],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('scan'), "' == 'true' and '",
                LaunchConfiguration('lio'), "' == 'true' and '",
                LaunchConfiguration('sim'), "' != 'true'"])),
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[PathJoinSubstitution(
                [bringup, 'config', 'pointcloud_to_laserscan.yaml'])],
            remappings=[('cloud_in', '/utlidar/cloud'), ('scan', '/scan')],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('scan'), "' == 'true' and '",
                LaunchConfiguration('lio'), "' != 'true' and '",
                LaunchConfiguration('sim'), "' != 'true'"])),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([bringup, 'config', 'go2w.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
