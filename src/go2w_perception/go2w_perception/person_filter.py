import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


class PersonFilter(Node):
    """Republica solo las detecciones de una clase (por defecto 'person') de yolo_ros."""

    def __init__(self):
        super().__init__('person_filter')
        input_topic = self.declare_parameter('input_topic', '/yolo/cam0/detections_3d').value
        output_topic = self.declare_parameter('output_topic', '/people').value
        self.class_name = self.declare_parameter('class_name', 'person').value
        self.pub = self.create_publisher(DetectionArray, output_topic, 10)
        self.create_subscription(DetectionArray, input_topic, self.on_detections, 10)
        self.get_logger().info(f'{input_topic} -> {output_topic} ({self.class_name})')

    def on_detections(self, msg):
        out = DetectionArray()
        out.header = msg.header
        out.detections = [d for d in msg.detections if d.class_name == self.class_name]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PersonFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
