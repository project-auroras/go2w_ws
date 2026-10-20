import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Go2Camera(Node):
    """Publica la cámara frontal del Go2 (multicast H.264 230.1.1.1:1720) como sensor_msgs/Image."""

    def __init__(self):
        super().__init__('go2_camera')
        iface = self.declare_parameter('iface', 'enP8p1s0').value
        decoder = self.declare_parameter('decoder', 'avdec_h264').value
        self.frame_id = self.declare_parameter('frame_id', 'go2_front_camera').value
        self.pub = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)

        Gst.init(None)
        self.pipeline = Gst.parse_launch(
            f'udpsrc address=230.1.1.1 port=1720 multicast-iface={iface} '
            '! application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! h264parse '
            f'! {decoder} ! videoconvert ! video/x-raw,format=BGR '
            '! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false')
        self.pipeline.get_by_name('sink').connect('new-sample', self.on_sample)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f'Escuchando 230.1.1.1:1720 en {iface} ({decoder})')

    def on_sample(self, sink):
        sample = sink.emit('pull-sample')
        s = sample.get_caps().get_structure(0)
        w, h = s.get_value('width'), s.get_value('height')
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = h, w
        msg.encoding = 'bgr8'
        msg.step = w * 3
        msg.data = bytes(info.data)
        buf.unmap(info)
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = h, w
        msg.encoding = 'bgr8'
        msg.step = w * 3
        msg.data = bytes(info.data)
        buf.unmap(info)
        self.pub.publish(msg)
        return Gst.FlowReturn.OK


def main():
    rclpy.init()
    node = Go2Camera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.try_shutdown()
