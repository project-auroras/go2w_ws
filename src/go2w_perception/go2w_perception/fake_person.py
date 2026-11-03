import math
import random

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import Detection, DetectionArray

COLORS = [(0.1, 0.8, 0.2), (1.0, 0.5, 0.0), (0.2, 0.4, 1.0), (0.9, 0.1, 0.6)]


def rotate(q, v):
    """Gira el vector v con el cuaternión q = (x, y, z, w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v); v' = v + w * t + q_vec x t
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))


class Walker:
    """Persona que recorre en bucle una lista de puntos (x, y) en el frame del mundo."""

    def __init__(self, person_id, waypoints, speed, pause):
        self.id = person_id
        self.waypoints = waypoints
        self.speed = speed
        self.pause = pause
        self.x, self.y = waypoints[0]
        self.next = 1 % len(waypoints)
        self.wait = pause

    def step(self, dt):
        if self.wait > 0.0 or len(self.waypoints) < 2:
            self.wait -= dt
            return
        gx, gy = self.waypoints[self.next]
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        advance = self.speed * dt
        if dist <= advance:
            self.x, self.y = gx, gy
            self.next = (self.next + 1) % len(self.waypoints)
            self.wait = self.pause
        else:
            self.x += dx / dist * advance
            self.y += dy / dist * advance


def pairs(flat):
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]


class FakePerson(Node):
    """Simula lo que publicaría person_filter (/people) para personas virtuales.

    Las personas andan en el frame world_frame. En cada ciclo se pasan al frame de
    la cámara con la TF y solo se publican si caen dentro del campo de visión de la
    RealSense, con ruido y fallos de detección. También publica markers para RViz.
    """

    def __init__(self):
        super().__init__('fake_person')
        p = self.declare_parameter
        self.world_frame = p('world_frame', 'odom').value
        self.camera_frame = p('camera_frame', 'camera_color_optical_frame').value
        output_topic = p('output_topic', '/people').value
        rate = p('rate', 15.0).value
        self.hfov = math.radians(p('hfov_deg', 69.0).value)
        self.min_range = p('min_range', 0.3).value
        self.max_range = p('max_range', 6.0).value
        self.noise = p('noise', 0.03).value              # sigma relativa a la distancia
        self.dropout = p('dropout', 0.1).value           # prob. de no detectar en un frame
        self.height = p('person_height', 1.7).value
        speed = p('speed', 0.4).value
        pause = p('pause', 2.0).value
        # persona 1 (a seguir) y persona 2 (distracción), [x0, y0, x1, y1, ...]
        path1 = p('path', [3.0, 0.0, 7.0, 0.0, 7.0, 4.0, 2.0, 4.0, 2.0, 1.0]).value
        path2 = p('distractor_path', [6.0, -3.0, 6.0, -1.5]).value
        self.fx, self.cx, self.cy = 615.0, 320.0, 240.0  # intrínsecos aprox. RealSense 640x480

        self.walkers = [Walker('1', pairs(path1), speed, pause)]
        if len(path2) >= 2:
            self.walkers.append(Walker('2', pairs(path2), 0.3, pause))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(DetectionArray, output_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fake_people/markers', 10)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f'{len(self.walkers)} persona(s) en {self.world_frame} -> {output_topic}')

    def step(self):
        for w in self.walkers:
            w.step(self.dt)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, self.world_frame, Time(), timeout=Duration(seconds=0.0))
        except TransformException as e:
            self.get_logger().warn(f'Sin TF {self.world_frame} -> {self.camera_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        q = (r.x, r.y, r.z, r.w)
        out = DetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.camera_frame
        visible = set()

        for w in self.walkers:
            # centro de la persona a media altura, en el frame de la cámara
            cx, cy, cz = rotate(q, (w.x, w.y, self.height / 2.0))
            x, y, z = cx + t.x, cy + t.y, cz + t.z
            if not (self.min_range < z < self.max_range):
                continue
            if abs(math.atan2(x, z)) > self.hfov / 2.0:
                continue
            visible.add(w.id)
            if random.random() < self.dropout:
                continue
            sigma = self.noise * z
            x += random.gauss(0.0, sigma)
            z += random.gauss(0.0, sigma)
            out.detections.append(self.make_detection(w.id, x, y, z))

        self.pub.publish(out)
        self.publish_markers(visible)

    def make_detection(self, person_id, x, y, z):
        d = Detection()
        d.class_id = 0
        d.class_name = 'person'
        d.score = 0.85
        d.id = person_id
        d.bbox.center.position.x = self.fx * x / z + self.cx
        d.bbox.center.position.y = self.fx * y / z + self.cy
        d.bbox.size.x = self.fx * 0.5 / z
        d.bbox.size.y = self.fx * self.height / z
        d.bbox3d.frame_id = self.camera_frame
        d.bbox3d.center.position.x = x
        d.bbox3d.center.position.y = y
        d.bbox3d.center.position.z = z
        d.bbox3d.center.orientation.w = 1.0
        d.bbox3d.size.x = 0.5
        d.bbox3d.size.y = self.height
        d.bbox3d.size.z = 0.3
        return d

    def publish_markers(self, visible):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, w in enumerate(self.walkers):
            r, g, b = COLORS[i % len(COLORS)]
            body = Marker()
            body.header.frame_id = self.world_frame
            body.header.stamp = stamp
            body.ns = 'person'
            body.id = i
            body.type = Marker.CYLINDER
            body.pose.position.x = w.x
            body.pose.position.y = w.y
            body.pose.position.z = self.height / 2.0
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = 0.4
            body.scale.z = self.height
            body.color.r, body.color.g, body.color.b = r, g, b
            body.color.a = 1.0 if w.id in visible else 0.3   # translúcida si no la ve
            label = Marker()
            label.header = body.header
            label.ns = 'label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x = w.x
            label.pose.position.y = w.y
            label.pose.position.z = self.height + 0.3
            label.pose.orientation.w = 1.0
            label.scale.z = 0.3
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'persona {w.id}' + ('' if w.id in visible else ' (fuera de vista)')
            markers.markers += [body, label]

        # campo de visión de la cámara, en el plano horizontal
        fov = Marker()
        fov.header.frame_id = self.camera_frame
        fov.header.stamp = stamp
        fov.ns = 'fov'
        fov.type = Marker.LINE_STRIP
        fov.pose.orientation.w = 1.0
        fov.scale.x = 0.03
        fov.color.r = fov.color.g = 1.0
        fov.color.a = 0.8
        half = math.tan(self.hfov / 2.0) * self.max_range
        for px, pz in [(0.0, 0.0), (-half, self.max_range), (half, self.max_range), (0.0, 0.0)]:
            fov.points.append(Point(x=px, y=0.0, z=pz))
        markers.markers.append(fov)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = FakePerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
