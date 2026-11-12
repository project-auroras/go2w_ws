import math
import random
from collections import deque

import numpy as np

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
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

    def __init__(self, person_id, waypoints, speed, pause, speed_profile=()):
        self.id = person_id
        self.waypoints = waypoints
        self.speed = speed
        # velocidad de cada tramo (el i va del punto i al i+1); vacío = siempre speed
        self.speed_profile = list(speed_profile)
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
        speed = self.speed
        if self.speed_profile:
            speed = self.speed_profile[(self.next - 1) % len(self.speed_profile)]
        advance = speed * dt
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
    RealSense, con ruido y fallos de detección, y si no las tapa un obstáculo. También
    publica markers para RViz.

    Con publish_scan publica un /scan (LaserScan en base_link, 360 deg) que ve los
    obstáculos (columnas de obstacles) y las personas con la estela de los últimos
    scan_window s, como el /scan de lio_scan en el robot real (junta 1 s de nubes).
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
        # m/s de cada tramo de la persona 1 (para probar cambios de ritmo); [] = speed
        profile = p('speed_profile', [0.0]).value
        profile = [v for v in profile if v > 0.0]
        # como ByteTrack con la cámara real: nuevo ID al volver a entrar en la imagen
        self.new_id_on_reappear = p('new_id_on_reappear', True).value
        # retraso de /people como WiFi + YOLO; el stamp es el de cuando se "tomó la imagen"
        self.latency = p('latency', 0.0).value
        self.pending = deque()
        self.next_track_id = 100
        self.track_ids = {}
        self.was_visible = set()
        self.fx, self.cx, self.cy = 615.0, 320.0, 240.0  # intrínsecos aprox. RealSense 640x480

        # obstáculos: columnas [x, y, radio, ...] en world_frame
        flat = p('obstacles', [0.0]).value
        self.obstacles = [tuple(flat[i:i + 3]) for i in range(0, len(flat) - 2, 3)]
        publish_scan = p('publish_scan', False).value
        self.scan_window = p('scan_window', 1.0).value
        self.person_radius = p('person_radius', 0.25).value
        self.base_frame = p('base_frame', 'base_link').value

        self.walkers = [Walker('1', pairs(path1), speed, pause, profile)]
        if len(path2) >= 2:
            self.walkers.append(Walker('2', pairs(path2), 0.3, pause))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(DetectionArray, output_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fake_people/markers', 10)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.step)
        self.history = deque()          # (t, [(x, y) de cada persona]) para la estela del scan
        if publish_scan:
            self.scan_pub = self.create_publisher(
                LaserScan, p('scan_topic', '/scan').value, 10)
            self.create_timer(1.0 / p('scan_rate', 10.0).value, self.publish_scan)
        self.get_logger().info(
            f'{len(self.walkers)} persona(s) y {len(self.obstacles)} obstáculo(s) en '
            f'{self.world_frame} -> {output_topic}' + (' + /scan' if publish_scan else ''))

    def occluded(self, q, t, x, z):
        """True si un obstáculo tapa el punto (x, z) del plano de la cámara."""
        for ox, oy, r in self.obstacles:
            cx, _, cz = rotate(q, (ox, oy, self.height / 2.0))
            cx, cz = cx + t.x, cz + t.z
            seg = x * x + z * z
            u = max(0.0, min(1.0, (cx * x + cz * z) / seg)) if seg > 0.0 else 0.0
            if math.hypot(cx - u * x, cz - u * z) < r and u * math.sqrt(seg) < math.sqrt(seg):
                return True
        return False

    def publish_scan(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.world_frame, Time(), timeout=Duration(seconds=0.0))
        except TransformException:
            return
        r = tf.transform.rotation
        yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        circles = list(self.obstacles)
        for _, positions in self.history:
            circles += [(x, y, self.person_radius) for x, y in positions]
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.base_frame
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 0.0087
        scan.range_min, scan.range_max = 0.15, 15.0
        n = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment))
        ranges = np.full(n, np.inf)
        if circles:
            arr = np.array(circles)
            # centros en base_link
            cxb = c * arr[:, 0] - s * arr[:, 1] + tx
            cyb = s * arr[:, 0] + c * arr[:, 1] + ty
            ang = scan.angle_min + scan.angle_increment * np.arange(n)
            dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
            b = dx * cxb[None, :] + dy * cyb[None, :]
            disc = b * b - (cxb ** 2 + cyb ** 2 - arr[:, 2] ** 2)[None, :]
            hit = b - np.sqrt(np.maximum(disc, 0.0))
            hit[(disc < 0.0) | (hit <= scan.range_min)] = np.inf
            ranges = hit.min(axis=1)
        scan.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(scan)

    def step(self):
        for w in self.walkers:
            w.step(self.dt)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.history.append((now, [(w.x, w.y) for w in self.walkers]))
        while self.history and now - self.history[0][0] > self.scan_window:
            self.history.popleft()

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
            if self.occluded(q, t, x, z):
                continue
            visible.add(w.id)
            if random.random() < self.dropout:
                continue
            sigma = self.noise * z
            x += random.gauss(0.0, sigma)
            z += random.gauss(0.0, sigma)
            out.detections.append(self.make_detection(self.track_id(w.id), x, y, z))

        self.was_visible = visible
        self.pending.append(out)
        now_ns = self.get_clock().now().nanoseconds
        while self.pending and now_ns - Time.from_msg(self.pending[0].header.stamp).nanoseconds \
                >= self.latency * 1e9:
            self.pub.publish(self.pending.popleft())
        self.publish_markers(visible)

    def track_id(self, person_id):
        if not self.new_id_on_reappear:
            return person_id
        if person_id not in self.was_visible or person_id not in self.track_ids:
            self.track_ids[person_id] = str(self.next_track_id)
            self.next_track_id += 1
        return self.track_ids[person_id]

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

        for i, (ox, oy, r) in enumerate(self.obstacles):
            obs = Marker()
            obs.header.frame_id = self.world_frame
            obs.header.stamp = stamp
            obs.ns = 'obstacle'
            obs.id = i
            obs.type = Marker.CYLINDER
            obs.pose.position.x, obs.pose.position.y = ox, oy
            obs.pose.position.z = 0.5
            obs.pose.orientation.w = 1.0
            obs.scale.x = obs.scale.y = 2.0 * r
            obs.scale.z = 1.0
            obs.color.r = obs.color.g = obs.color.b = 0.5
            obs.color.a = 1.0
            markers.markers.append(obs)

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
