import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


class DepthDecompress(Node):
    """compressedDepth (PNG) -> sensor_msgs/Image 16UC1.

    Sustituye a 'image_transport republish compressedDepth raw', que en Humble falla con
    "SubscriberPlugin::subscribeImpl with five arguments has not been overridden".
    El mensaje compressedDepth lleva una cabecera de configuración seguida del PNG.
    """

    def __init__(self):
        super().__init__('depth_decompress')
        in_topic = self.declare_parameter(
            'input_topic',
            '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth').value
        out_topic = self.declare_parameter(
            'output_topic', '/camera_local/aligned_depth/image_raw').value
        self.pub = self.create_publisher(Image, out_topic, 10)
        self.create_subscription(CompressedImage, in_topic, self.on_image, 10)
        self.get_logger().info(f'{in_topic} -> {out_topic}')

    def on_image(self, msg):
        data = bytes(msg.data)
        start = data.find(PNG_SIGNATURE)
        if start < 0:
            self.get_logger().error(
                f'Formato no soportado ({msg.format}): solo compressedDepth PNG de 16UC1',
                throttle_duration_sec=5.0)
            return
        depth = cv2.imdecode(np.frombuffer(data[start:], np.uint8), cv2.IMREAD_UNCHANGED)
        if depth is None or depth.dtype != np.uint16 or depth.ndim != 2:
            self.get_logger().error('No se pudo decodificar la profundidad',
                                    throttle_duration_sec=5.0)
            return
        out = Image()
        out.header = msg.header
        out.height, out.width = depth.shape
        out.encoding = '16UC1'
        out.is_bigendian = 0
        out.step = out.width * 2
        out.data = depth.astype('<u2').tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthDecompress()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
