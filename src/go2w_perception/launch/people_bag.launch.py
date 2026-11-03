from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Depth -> color extrinsics of our RealSense, read from
# /camera/camera/extrinsics/depth_to_color in rosbag2_2026_10_28-10_44_19
# (the bags have no /tf_static). x y z qx qy qz qw.
DEPTH_TO_COLOR = ['0.01471', '0.00003', '0.00045',
                  '-0.000815', '0.00232', '0.008123', '0.999965']


def generate_launch_description():
    """People detection on a RealSense rosbag: depth aligned to color + yolo_ros + filter."""
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depth_to_color_tf',
            arguments=[
                '--x', DEPTH_TO_COLOR[0], '--y', DEPTH_TO_COLOR[1], '--z', DEPTH_TO_COLOR[2],
                '--qx', DEPTH_TO_COLOR[3], '--qy', DEPTH_TO_COLOR[4],
                '--qz', DEPTH_TO_COLOR[5], '--qw', DEPTH_TO_COLOR[6],
                '--frame-id', 'camera_color_optical_frame',
                '--child-frame-id', 'camera_depth_optical_frame'],
        ),

        # The bag only has unaligned depth (848x480): project it onto the color image
        ComposableNodeContainer(
            name='depth_register_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::RegisterNode',
                name='depth_register',
                remappings=[
                    ('depth/image_rect', '/camera/camera/depth/image_rect_raw'),
                    ('depth/camera_info', '/camera/camera/depth/camera_info'),
                    ('rgb/camera_info', '/camera/camera/color/camera_info'),
                    ('depth_registered/image_rect', '/camera/camera/aligned_depth/image'),
                    ('depth_registered/camera_info',
                     '/camera/camera/aligned_depth/camera_info'),
                ],
            )],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('yolo_bringup'), 'launch', 'yolo.launch.py'])),
            launch_arguments={'params_file': PathJoinSubstitution(
                [FindPackageShare('go2w_perception'), 'config', 'yolo_people.yaml'])}.items()),

        Node(
            package='go2w_perception',
            executable='person_filter',
            output='screen',
        ),
    ])
