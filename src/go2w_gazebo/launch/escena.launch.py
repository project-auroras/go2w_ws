import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

WHEEL_RADIUS = 0.086
WHEEL_WIDTH = 0.04


def robot_description():
    """URDF del Go2-W para Gazebo, con las ruedas como cilindros.

    El URDF original usa la malla de la rueda como colisión: en Gazebo rueda a
    saltos, así que se cambia por un cilindro del mismo radio.
    """
    path = os.path.join(get_package_share_directory('go2w_gazebo'), 'urdf',
                        'go2w_gazebo.urdf.xacro')
    doc = xacro.process_file(path)
    for link in doc.getElementsByTagName('link'):
        if not link.getAttribute('name').endswith('_foot'):
            continue
        for collision in link.getElementsByTagName('collision'):
            origin = collision.getElementsByTagName('origin')[0]
            origin.setAttribute('rpy', '1.5707963 0 0')
            geometry = collision.getElementsByTagName('geometry')[0]
            for child in list(geometry.childNodes):
                geometry.removeChild(child)
            cylinder = doc.createElement('cylinder')
            cylinder.setAttribute('radius', str(WHEEL_RADIUS))
            cylinder.setAttribute('length', str(WHEEL_WIDTH))
            geometry.appendChild(cylinder)
    return doc.toxml()


def launch_setup(context):
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    world = LaunchConfiguration('world').perform(context)
    gz_args = f'-r {world}' if gui else f'-r -s --headless-rendering {world}'

    return [
        # Gazebo. -r: arranca sin pausa; -s: sin ventana
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': gz_args}.items()),

        # Publica /robot_description y las TF del robot a partir de /joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description(), 'use_sim_time': True}],
        ),

        # Mete el robot en Gazebo leyendo /robot_description
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'go2w',
                       '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                       '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
            output='screen',
        ),

        # Puente Gazebo <-> ROS: /clock, /cmd_vel, /odom, /tf, /joint_states, cámara
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': PathJoinSubstitution(
                    [FindPackageShare('go2w_gazebo'), 'config', 'bridge.yaml']).perform(
                        context),
                'use_sim_time': True,
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    """Gazebo con un mundo + el Go2-W (de pie, con ruedas y RealSense) + puente a ROS."""
    description_share = get_package_share_directory('go2w_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=PathJoinSubstitution(
                [FindPackageShare('go2w_gazebo'), 'worlds', 'objetos.sdf']),
            description='Ruta al .sdf del mundo'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: Gazebo sin ventana'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.45'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Para que Gazebo encuentre las mallas package://go2w_description/...
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                                  os.path.dirname(description_share)),

        OpaqueFunction(function=launch_setup),
    ])
