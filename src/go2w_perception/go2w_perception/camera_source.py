"""De dónde lee YOLO la imagen: parte común de people.launch.py y semantic.launch.py.

source:=camera  YOLO en la NX, junto a la RealSense: lee los topics raw.
source:=wifi    YOLO en el portátil: la NX manda la imagen comprimida por WiFi y aquí se
                descomprime en /camera_local/...
source:=bag     rosbags antiguos (octubre) sin profundidad alineada ni TF: se alinea con
                depth_image_proc y las posiciones 3D salen en el frame óptico de la cámara.
                Un bag nuevo grabado con aligned_depth_to_color y /tf, /tf_static se
                reproduce con source:=camera.

Según source, el launch arranca los nodos que hacen falta y fija los topics de la cámara
y el target_frame de yolo_node (lo que pone el yaml se sobrescribe).
"""

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

CAM = '/camera/camera'
SOURCES = ('camera', 'wifi', 'bag')

# Extrínsecos depth -> color de nuestra RealSense, leídos de
# /camera/camera/extrinsics/depth_to_color en rosbag2_2026_10_28-10_44_19
# (los bags no tienen /tf_static). x y z qx qy qz qw.
DEPTH_TO_COLOR = ['0.01471', '0.00003', '0.00045',
                  '-0.000815', '0.00232', '0.008123', '0.999965']

TOPICS = {
    'camera': {
        'cam0.rgb_topic': f'{CAM}/color/image_raw',
        'cam0.depth_topic': f'{CAM}/aligned_depth_to_color/image_raw',
        'cam0.depth_info_topic': f'{CAM}/aligned_depth_to_color/camera_info',
        'detection3d.target_frame': 'base_link',
    },
    'wifi': {
        'cam0.rgb_topic': '/camera_local/color/image_raw',
        'cam0.depth_topic': '/camera_local/aligned_depth/image_raw',
        'cam0.depth_info_topic': f'{CAM}/aligned_depth_to_color/camera_info',
        'detection3d.target_frame': 'base_link',
    },
    'bag': {
        'cam0.rgb_topic': f'{CAM}/color/image_raw',
        'cam0.depth_topic': f'{CAM}/aligned_depth/image',
        'cam0.depth_info_topic': f'{CAM}/aligned_depth/camera_info',
        'detection3d.target_frame': 'camera_color_optical_frame',
    },
}


def declare_source_argument():
    return DeclareLaunchArgument(
        'source', default_value='camera', choices=list(SOURCES),
        description='camera: YOLO en la NX | wifi: YOLO en el portátil (descomprime) | '
                    'bag: rosbags antiguos sin profundidad alineada ni TF')


def _wifi_nodes():
    return [
        # JPEG -> imagen (color). republish tiene dos topics con nombre fijo, in y out;
        # el remapping los conecta con los topics reales.
        Node(
            package='image_transport', executable='republish', name='color_decompress',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', f'{CAM}/color/image_raw/compressed'),
                        ('out', '/camera_local/color/image_raw')],
        ),
        # PNG -> profundidad (16UC1, mm), alineada al color. Nodo propio: el
        # 'republish compressedDepth raw' de Humble falla al suscribirse.
        Node(
            package='go2w_perception', executable='depth_decompress', output='screen',
            parameters=[{
                'input_topic': f'{CAM}/aligned_depth_to_color/image_raw/compressedDepth',
                'output_topic': '/camera_local/aligned_depth/image_raw',
            }],
        ),
    ]


def _bag_nodes():
    d = DEPTH_TO_COLOR
    return [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='depth_to_color_tf',
            arguments=['--x', d[0], '--y', d[1], '--z', d[2],
                       '--qx', d[3], '--qy', d[4], '--qz', d[5], '--qw', d[6],
                       '--frame-id', 'camera_color_optical_frame',
                       '--child-frame-id', 'camera_depth_optical_frame'],
        ),
        # El bag solo tiene la profundidad sin alinear (848x480): se proyecta sobre el color
        ComposableNodeContainer(
            name='depth_register_container', namespace='',
            package='rclcpp_components', executable='component_container',
            composable_node_descriptions=[ComposableNode(
                package='depth_image_proc', plugin='depth_image_proc::RegisterNode',
                name='depth_register',
                remappings=[
                    ('depth/image_rect', f'{CAM}/depth/image_rect_raw'),
                    ('depth/camera_info', f'{CAM}/depth/camera_info'),
                    ('rgb/camera_info', f'{CAM}/color/camera_info'),
                    ('depth_registered/image_rect', f'{CAM}/aligned_depth/image'),
                    ('depth_registered/camera_info', f'{CAM}/aligned_depth/camera_info'),
                ],
            )],
            output='screen',
        ),
    ]


def _setup(context, yolo_params):
    source = LaunchConfiguration('source').perform(context)
    nodes = {'camera': [], 'wifi': _wifi_nodes(), 'bag': _bag_nodes()}[source]
    # Igual que yolo_bringup/yolo.launch.py (mismo nodo y namespace), pero con los topics
    # y el frame de este source por encima del yaml.
    nodes.append(Node(
        package='yolo_ros', executable='yolo_node', name='yolo_node', namespace='yolo',
        output='screen',
        parameters=[yolo_params.perform(context), TOPICS[source]],
    ))
    return nodes


def camera_and_yolo(yolo_params):
    """Nodos de la cámara según source + yolo_node con el yaml yolo_params."""
    return OpaqueFunction(function=_setup, args=[yolo_params])
