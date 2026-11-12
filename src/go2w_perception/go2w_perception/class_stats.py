import collections

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


class ClassStats(Node):
    """Cuenta las detecciones de yolo_ros por clase y las imprime cada `period` s.

    Para comprobar qué clases del modelo salen en la simulación (o en un rosbag) antes
    de usarlas para navegar: número de detecciones, score medio y máximo y distancia
    media (bbox3d) de cada clase en el último periodo.
    """

    def __init__(self):
        super().__init__('class_stats')
        topic = self.declare_parameter('input_topic', '/yolo/cam0/detections_3d').value
        period = self.declare_parameter('period', 5.0).value
        self.stats = collections.defaultdict(list)
        self.frames = 0
        self.create_subscription(DetectionArray, topic, self.on_detections, 10)
        self.create_timer(period, self.report)
        self.get_logger().info(f'Contando clases de {topic} cada {period} s')

    def on_detections(self, msg):
        self.frames += 1
        for d in msg.detections:
            c = d.bbox3d.center.position
            dist = (c.x * c.x + c.y * c.y + c.z * c.z) ** 0.5
            self.stats[d.class_name].append((d.score, dist))

    def report(self):
        if not self.stats:
            self.get_logger().info(f'{self.frames} imágenes, ninguna detección')
        else:
            lines = [f'{self.frames} imágenes:']
            for cls, vals in sorted(self.stats.items(), key=lambda kv: -len(kv[1])):
                scores = [s for s, _ in vals]
                dists = [d for _, d in vals if d > 0.0]
                mean_d = sum(dists) / len(dists) if dists else float('nan')
                mean_s = sum(scores) / len(scores)
                lines.append(f'  {cls:12s} n={len(vals):4d}  score medio {mean_s:.2f}'
                             f'  máx {max(scores):.2f}  dist media {mean_d:.1f} m')
            self.get_logger().info('\n'.join(lines))
        self.stats.clear()
        self.frames = 0


def main():
    rclpy.init()
    node = ClassStats()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
