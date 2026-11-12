import colorsys
import math
import os
import time
import zlib

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from yolo_msgs.msg import Detection, DetectionArray

from go2w_perception.semantic_common import (detection_frame, lookup, transform_point,
                                             valid_position)


class Landmark:
    """Un objeto del mapa: posición media de sus detecciones en el frame del mapa."""

    def __init__(self, lid, cls, pos, size, score, t):
        self.id, self.cls = lid, cls
        self.x, self.y, self.z = pos
        self.sx, self.sy, self.sz = size
        self.score = score
        self.hits = 1
        self.first = self.last = t

    def update(self, pos, size, score, alpha, t):
        # media acumulada al principio (alpha = 1/hits) y luego móvil con alpha
        a = max(alpha, 1.0 / (self.hits + 1))
        self.x += a * (pos[0] - self.x)
        self.y += a * (pos[1] - self.y)
        self.z += a * (pos[2] - self.z)
        self.sx += a * (size[0] - self.sx)
        self.sy += a * (size[1] - self.sy)
        self.sz += a * (size[2] - self.sz)
        self.score = max(self.score, score)
        self.hits += 1
        self.last = t

    def to_dict(self):
        return {'id': self.id, 'class': self.cls, 'x': round(self.x, 3), 'y': round(self.y, 3),
                'z': round(self.z, 3), 'size': [round(v, 3) for v in (self.sx, self.sy, self.sz)],
                'score': round(self.score, 3), 'hits': self.hits}


class SemanticMapper(Node):
    """Mapa semántico: acumula en map_frame los objetos estáticos que detecta yolo_ros.

    Cada detección 3D de una clase de `classes` se pasa a map_frame (TF en el instante
    de la imagen) y se asocia al objeto más cercano de su misma clase si está a menos
    de su radio (`radii`, en el mismo orden que `classes`); si no, crea uno nuevo. Un
    objeto se confirma con min_hits detecciones; los no confirmados se olvidan a los
    forget_s s. Las clases dinámicas (person, animales) no van en `classes`.

    Publica los confirmados en /semantic/landmarks (yolo_msgs/DetectionArray en
    map_frame, id = id del objeto) y /semantic/markers (RViz). ~/save guarda el mapa
    en un YAML y load_file lo carga al arrancar.
    """

    def __init__(self):
        super().__init__('semantic_mapper')
        p = self.declare_parameter
        self.input_topic = p('input_topic', '/yolo/cam0/detections_3d').value
        self.map_frame = p('map_frame', 'map').value
        self.classes = list(p('classes', ['Row', 'Trace', 'Trunk', 'Post', 'Pothole',
                                          'Puddle']).value)
        radii = list(p('radii', [1.0, 1.0, 0.8, 0.5, 0.6, 1.0]).value)
        if len(radii) != len(self.classes):
            raise ValueError('radii debe tener un valor por cada clase de classes')
        self.radius = dict(zip(self.classes, radii))
        self.min_score = p('min_score', 0.4).value
        self.max_range = p('max_range', 8.0).value
        self.min_hits = p('min_hits', 3).value
        self.forget_s = p('forget_s', 5.0).value
        self.alpha = p('alpha', 0.2).value
        self.save_dir = p('save_dir', '/ws/maps').value
        load_file = p('load_file', '').value

        self.landmarks = []
        self.next_id = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(DetectionArray, '/semantic/landmarks', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/semantic/markers', 10)
        self.create_subscription(DetectionArray, self.input_topic, self.on_detections, 10)
        self.create_service(Trigger, '~/save', self.on_save)
        self.create_timer(1.0 / p('publish_rate', 2.0).value, self.publish)

        if load_file:
            self.load(load_file)
        self.get_logger().info(
            f'{self.input_topic} -> /semantic/landmarks en {self.map_frame}: {self.classes}')

    # ---------- detecciones ----------

    def on_detections(self, msg):
        now = self.now_s()
        stamp = Time.from_msg(msg.header.stamp)
        tfs = {}
        for det in msg.detections:
            if det.class_name not in self.radius or det.score < self.min_score:
                continue
            c = det.bbox3d.center.position
            if not valid_position(c):
                continue
            frame = detection_frame(det, msg.header.frame_id)
            if math.sqrt(c.x * c.x + c.y * c.y + c.z * c.z) > self.max_range:
                continue   # lejos, la profundidad es poco fiable
            if frame not in tfs:
                tfs[frame] = lookup(self.tf_buffer, self.map_frame, frame, stamp)
            tf = tfs[frame]
            if tf is None:
                self.get_logger().warn(f'Sin TF {frame} -> {self.map_frame}',
                                       throttle_duration_sec=5.0)
                continue
            pos = transform_point(tf, c.x, c.y, c.z)
            s = det.bbox3d.size
            self.integrate(det.class_name, pos, (s.x, s.y, s.z), det.score, now)
        self.forget(now)

    def integrate(self, cls, pos, size, score, t):
        best, best_d = None, self.radius[cls]
        for lm in self.landmarks:
            if lm.cls != cls:
                continue
            d = math.hypot(lm.x - pos[0], lm.y - pos[1])
            if d < best_d:
                best, best_d = lm, d
        if best is not None:
            best.update(pos, size, score, self.alpha, t)
            return
        self.landmarks.append(Landmark(self.next_id, cls, pos, size, score, t))
        self.next_id += 1

    def forget(self, t):
        self.landmarks = [lm for lm in self.landmarks
                          if lm.hits >= self.min_hits or t - lm.last < self.forget_s]

    def confirmed(self):
        return [lm for lm in self.landmarks if lm.hits >= self.min_hits]

    # ---------- salida ----------

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        out = DetectionArray()
        out.header.stamp = stamp
        out.header.frame_id = self.map_frame
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for lm in self.confirmed():
            d = Detection()
            d.class_name = lm.cls
            d.class_id = self.classes.index(lm.cls) if lm.cls in self.classes else -1
            d.id = str(lm.id)
            d.score = float(lm.score)
            d.bbox3d.frame_id = self.map_frame
            d.bbox3d.center.position.x = lm.x
            d.bbox3d.center.position.y = lm.y
            d.bbox3d.center.position.z = lm.z
            d.bbox3d.center.orientation.w = 1.0
            d.bbox3d.size.x, d.bbox3d.size.y, d.bbox3d.size.z = lm.sx, lm.sy, lm.sz
            out.detections.append(d)
            markers.markers.extend(self.make_markers(lm, stamp))
        self.pub.publish(out)
        self.markers_pub.publish(markers)

    def make_markers(self, lm, stamp):
        r, g, b = class_color(lm.cls)
        box = Marker()
        box.header.frame_id = self.map_frame
        box.header.stamp = stamp
        box.ns = lm.cls
        box.id = lm.id
        box.type = Marker.CUBE
        box.pose.position.x, box.pose.position.y, box.pose.position.z = lm.x, lm.y, lm.z
        box.pose.orientation.w = 1.0
        box.scale.x = max(lm.sx, 0.1)
        box.scale.y = max(lm.sy, 0.1)
        box.scale.z = max(lm.sz, 0.1)
        box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, 0.5
        text = Marker()
        text.header = box.header
        text.ns = lm.cls + '_text'
        text.id = lm.id
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.x, text.pose.position.y = lm.x, lm.y
        text.pose.position.z = lm.z + box.scale.z / 2 + 0.3
        text.pose.orientation.w = 1.0
        text.scale.z = 0.3
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = f'{lm.cls}:{lm.id}'
        return [box, text]

    # ---------- guardar / cargar ----------

    def on_save(self, request, response):
        os.makedirs(self.save_dir, exist_ok=True)
        path = os.path.join(self.save_dir, time.strftime('semantic_%Y-%m-%d_%H-%M-%S.yaml'))
        data = {'frame_id': self.map_frame,
                'landmarks': [lm.to_dict() for lm in self.confirmed()]}
        with open(path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        response.success = True
        response.message = f'{len(data["landmarks"])} objetos en {path}'
        self.get_logger().info(response.message)
        return response

    def load(self, path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        if data.get('frame_id', self.map_frame) != self.map_frame:
            self.get_logger().warn(f'{path} está en {data["frame_id"]}, no en {self.map_frame}')
        t = self.now_s()
        for item in data.get('landmarks', []):
            lm = Landmark(item['id'], item['class'], (item['x'], item['y'], item['z']),
                          tuple(item.get('size', (0.2, 0.2, 0.2))), item.get('score', 1.0), t)
            lm.hits = max(item.get('hits', self.min_hits), self.min_hits)
            self.landmarks.append(lm)
            self.next_id = max(self.next_id, lm.id + 1)
        self.get_logger().info(f'Cargados {len(self.landmarks)} objetos de {path}')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


def class_color(cls):
    """Color fijo por clase (el mismo en todas las ejecuciones)."""
    h = (zlib.crc32(cls.encode()) % 360) / 360.0
    return colorsys.hsv_to_rgb(h, 0.8, 1.0)


def main():
    rclpy.init()
    node = SemanticMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
