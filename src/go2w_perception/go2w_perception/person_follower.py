import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from yolo_msgs.msg import DetectionArray

FOLLOW, LOST, SEARCH, IDLE = 'siguiendo', 'perdida', 'buscando', 'parado'


def clamp(value, limit):
    return max(-limit, min(limit, value))


def ramp(target, last, accel, decel, dt):
    """Acerca last a target sin pasar de accel (al ganar velocidad) o decel (al perderla)."""
    limit = accel if abs(target) > abs(last) and target * last >= 0.0 else decel
    step = limit * dt
    return last + max(-step, min(step, target - last))


class PersonTrack:
    """Filtro alfa-beta (velocidad constante) de la persona en el frame de odometría.

    Trabajar en odom, y no relativo al robot, separa el movimiento de la persona del
    del robot: la velocidad que sale es la de la persona respecto al suelo.
    """

    def __init__(self, x, y, t, alpha, beta, max_speed):
        self.x, self.y = x, y
        self.vx = self.vy = 0.0
        self.t = t
        self.alpha, self.beta, self.max_speed = alpha, beta, max_speed

    def predict(self, t, max_dt=None):
        dt = t - self.t
        if max_dt is not None:
            dt = min(dt, max_dt)
        return self.x + self.vx * dt, self.y + self.vy * dt

    def update(self, x, y, t):
        dt = t - self.t
        if dt <= 1e-3:
            return
        px, py = self.predict(t)
        rx, ry = x - px, y - py
        self.x, self.y = px + self.alpha * rx, py + self.alpha * ry
        self.vx += self.beta / dt * rx
        self.vy += self.beta / dt * ry
        speed = math.hypot(self.vx, self.vy)
        if speed > self.max_speed:   # un salto de la profundidad no es una persona a 5 m/s
            self.vx *= self.max_speed / speed
            self.vy *= self.max_speed / speed
        self.t = t


class PersonFollower(Node):
    """Sigue a una persona de /people publicando /cmd_vel, sin chocar y buscándola si la pierde.

    Seguir: elige la persona más cercana (a menos de max_start_distance) y se queda con
    su ID de tracking. Gira para tenerla centrada y avanza para mantener la distancia.
    Para no quedarse atrás cuando la persona acelera, estima su velocidad respecto al
    suelo (posición en odom + filtro alfa-beta) y la usa como base de la orden:
        vx = v_persona + k_linear * (distancia - distancia_objetivo)
    La distancia objetivo crece con la velocidad (target_distance + time_gap * v). Los
    cambios de velocidad van con rampa (max_accel / max_decel / max_angular_accel). Si la
    persona se le acerca a menos de backoff_distance, retrocede despacio.

    Obstáculos (/scan, obstacle_avoidance): limita la velocidad para poder frenar siempre
    a stop_distance del primer obstáculo del pasillo por el que avanza (o retrocede), y
    se aparta de los que tiene a los lados a menos de avoid_distance. Los puntos del scan
    que caen sobre la persona seguida y su recorrido reciente (person_trail_time) no
    cuentan: /scan de lio_scan junta 1 s de nubes y deja una estela de la persona.
    Sin /scan reciente no se desplaza (solo gira). No planifica: si el camino está
    cortado, se queda parado delante del obstáculo.

    Perderla: si deja de verla, sigue hacia donde el filtro predice que está; a los
    stop_timeout s frena. A los search_delay s empieza a girar sobre sí mismo hacia el
    lado por el que la perdió (search_angular), durante search_timeout s como mucho.
    Hasta lost_timeout la recuerda y se reengancha a una detección a menos de
    reacquire_radius de donde debería estar, aunque tenga otro ID; después, mientras
    busca, acepta a la persona más cercana a menos de search_max_distance.

    Las posiciones pueden venir en un frame óptico de cámara (z delante, x derecha,
    nombre con 'optical') o en uno de robot como base_link (x delante, y izquierda).
    """

    def __init__(self):
        super().__init__('person_follower')
        p = self.declare_parameter
        input_topic = p('input_topic', '/people').value
        cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel').value
        odom_topic = p('odom_topic', '/odom').value
        self.target_id = p('target_id', '').value          # '' = la más cercana
        self.target_distance = p('target_distance', 1.5).value
        self.time_gap = p('time_gap', 0.5).value
        self.distance_deadband = p('distance_deadband', 0.15).value
        self.max_range = p('max_range', 8.0).value
        self.k_linear = p('k_linear', 1.0).value
        self.k_angular = p('k_angular', 1.5).value
        self.max_linear = p('max_linear', 0.8).value
        self.max_accel = p('max_accel', 1.0).value
        self.max_decel = p('max_decel', 1.5).value
        # marcha atrás si la persona se acerca a menos de backoff_distance (desde base_link)
        self.backoff_distance = p('backoff_distance', 1.3).value
        self.k_backoff = p('k_backoff', 1.0).value
        self.max_backward = p('max_backward', 0.3).value
        self.max_angular = p('max_angular', 1.2).value
        self.k_angular_ff = p('k_angular_ff', 1.0).value      # 0 = sin anticipar el giro
        self.latency_compensation = p('latency_compensation', True).value
        self.max_angular_accel = p('max_angular_accel', 3.0).value
        self.turn_in_place_angle = p('turn_in_place_angle', 0.6).value  # rad
        self.moving_speed = p('moving_speed', 0.2).value     # m/s: por debajo, "parada"
        self.alpha = p('track_alpha', 0.35).value
        self.beta = p('track_beta', 0.05).value
        self.max_person_speed = p('max_person_speed', 3.0).value
        self.stop_timeout = p('stop_timeout', 0.5).value
        self.lost_timeout = p('lost_timeout', 3.0).value
        self.reacquire_radius = p('reacquire_radius', 1.0).value
        self.max_start_distance = p('max_start_distance', 3.0).value
        # búsqueda
        self.search_enabled = p('search_enabled', True).value
        self.search_delay = p('search_delay', 1.5).value
        self.search_angular = p('search_angular', 0.6).value
        self.search_timeout = p('search_timeout', 20.0).value
        self.search_max_distance = p('search_max_distance', 5.0).value
        # obstáculos
        self.obstacle_avoidance = p('obstacle_avoidance', True).value
        scan_topic = p('scan_topic', '/scan').value
        self.scan_timeout = p('scan_timeout', 0.5).value
        self.robot_front = p('robot_front', 0.45).value      # m de base_link al morro
        self.robot_back = p('robot_back', -0.55).value       # m de base_link a la cola
        self.robot_half_width = p('robot_half_width', 0.35).value
        self.side_margin = p('side_margin', 0.20).value
        self.stop_distance = p('stop_distance', 0.4).value
        self.obstacle_decel = p('obstacle_decel', 1.0).value
        self.obstacle_latency = p('obstacle_latency', 0.3).value
        self.avoid_distance = p('avoid_distance', 0.5).value
        self.k_avoid = p('k_avoid', 1.0).value
        self.person_clearance = p('person_clearance', 0.4).value
        self.person_trail_time = p('person_trail_time', 1.5).value
        rate = p('rate', 10.0).value
        self.dt = 1.0 / rate

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(DetectionArray, input_topic, self.on_people, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        if self.obstacle_avoidance:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_timer(self.dt, self.control)

        self.state = IDLE
        self.current_id = None
        self.track = None           # PersonTrack en odom
        self.trail = deque()        # (t, x, y) en odom de las últimas detecciones seguidas
        self.odom = None            # (x, y, yaw) del robot
        self.odom_history = deque()  # (stamp, x, y, yaw) de los últimos 2 s de /odom
        self.stamp_warned = False
        self.last_seen = None
        self.last_bearing = 0.0
        self.search_start = None
        self.search_dir = 1.0
        self.scan_points = None     # N x 2 en base_link
        self.scan_time = None
        self.scan_tf = {}           # frame -> (x, y, yaw) hasta base_link
        self.last_vx = self.last_wz = 0.0
        self.get_logger().info(
            f'{input_topic} -> {cmd_vel_topic}, distancia objetivo {self.target_distance} m'
            f' + {self.time_gap} s x v_persona; obstáculos '
            + (f'con {scan_topic}' if self.obstacle_avoidance else 'DESACTIVADOS')
            + ('; busca al perderla' if self.search_enabled else ''))

    # ---------------------------------------------------------------- utilidades
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.odom_history and stamp <= self.odom_history[-1][0]:
            self.odom_history.clear()   # el reloj ha ido hacia atrás (p. ej. bag en bucle)
        self.odom_history.append((stamp,) + self.odom)
        while self.odom_history and stamp - self.odom_history[0][0] > 2.0:
            self.odom_history.popleft()

    def pose_at(self, stamp):
        """Pose del robot en odom en el instante stamp y cuánto hace de eso (s).

        Las detecciones llegan tarde (WiFi + YOLO): si el robot gira mientras tanto, pasar
        la detección a odom con la pose de ahora la coloca en otro sitio. Se usa la pose
        de cuando se tomó la imagen. Si stamp no cae en el historial de /odom (relojes
        distintos, sin stamp...), la pose de ahora.
        """
        h = self.odom_history
        if not self.latency_compensation or len(h) < 2 or stamp <= 0.0:
            return self.odom, 0.0
        if not h[0][0] <= stamp <= h[-1][0] + 0.05:
            if abs(stamp - h[-1][0]) > 0.5 and not self.stamp_warned:
                self.stamp_warned = True
                self.get_logger().warn(
                    f'El stamp de /people está {h[-1][0] - stamp:+.2f} s respecto a /odom: '
                    'no compenso el retraso de las detecciones (¿relojes sin sincronizar?)')
            return self.odom, 0.0
        if stamp >= h[-1][0]:
            return self.odom, 0.0
        i = next(k for k in range(len(h) - 1, 0, -1) if h[k - 1][0] <= stamp)
        (t0, x0, y0, a0), (t1, x1, y1, a1) = h[i - 1], h[i]
        u = (stamp - t0) / (t1 - t0) if t1 > t0 else 0.0
        da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        return (x0 + u * (x1 - x0), y0 + u * (y1 - y0), a0 + u * da), h[-1][0] - stamp

    def to_odom(self, forward, left, pose=None):
        x, y, yaw = pose or self.odom
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * forward - s * left, y + s * forward + c * left

    def to_robot(self, wx, wy):
        """Del frame odom a (delante, izquierda) del robot."""
        x, y, yaw = self.odom
        dx, dy = wx - x, wy - y
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy

    @staticmethod
    def to_robot_plane(detection, frame_id):
        """Devuelve (delante, izquierda) en metros."""
        pos = detection.bbox3d.center.position
        if 'optical' in frame_id:
            return pos.z, -pos.x
        return pos.x, pos.y

    def forget(self):
        self.current_id = None
        self.track = None
        self.trail.clear()

    # ---------------------------------------------------------------- scan
    def on_scan(self, msg):
        frame = msg.header.frame_id
        if frame not in self.scan_tf:
            if frame in ('base_link', ''):
                self.scan_tf[frame] = (0.0, 0.0, 0.0)
            else:
                try:   # el LiDAR va fijo al robot: basta con buscar la TF una vez
                    tf = self.tf_buffer.lookup_transform(
                        'base_link', frame, Time(), timeout=Duration(seconds=0.0))
                except TransformException as e:
                    self.get_logger().warn(f'Sin TF base_link -> {frame}: {e}',
                                           throttle_duration_sec=2.0)
                    return
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.scan_tf[frame] = (tf.transform.translation.x, tf.transform.translation.y,
                                       yaw)
        tx, ty, tyaw = self.scan_tf[frame]
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + tyaw + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        self.scan_points = np.stack([tx + r[ok] * np.cos(a[ok]), ty + r[ok] * np.sin(a[ok])],
                                    axis=1)
        self.scan_time = self.now()

    def obstacles(self, t):
        """Puntos del scan (base_link) que son obstáculos, o None si no hay scan reciente."""
        if self.scan_points is None or t - self.scan_time > self.scan_timeout:
            return None
        pts = self.scan_points
        if self.trail and len(pts):
            # fuera la persona seguida y su estela (posiciones recientes en odom), más dónde
            # está ahora según el filtro: las detecciones llegan tarde y el scan no
            positions = [(x, y) for _, x, y in self.trail]
            if self.track is not None:
                positions.append(self.track.predict(t, max_dt=1.0))
            people = np.array([self.to_robot(x, y) for x, y in positions])
            # distancia de cada punto al recorrido (segmentos entre posiciones seguidas): si
            # falta alguna detección (sale un momento de la imagen al girar), el hueco
            # también queda cubierto
            a = people[:-1] if len(people) > 1 else people
            b = people[1:] if len(people) > 1 else people
            ab = b - a
            len2 = np.maximum((ab ** 2).sum(axis=1), 1e-9)
            u = np.clip(((pts[:, None, :] - a[None]) * ab[None]).sum(axis=2) / len2, 0.0, 1.0)
            closest = a[None] + u[:, :, None] * ab[None]
            d2 = ((pts[:, None, :] - closest) ** 2).sum(axis=2)
            pts = pts[d2.min(axis=1) > self.person_clearance ** 2]
        return pts

    def obstacle_limits(self, pts):
        """(v máx. hacia delante, v máx. hacia atrás, giro de repulsión, hueco delante)."""
        hw = self.robot_half_width + self.side_margin
        x, y = pts[:, 0], pts[:, 1]
        in_lane = np.abs(y) < hw
        ahead = x[in_lane & (x > self.robot_front)]
        behind = x[in_lane & (x < self.robot_back)]
        gap_front = ahead.min() - self.robot_front if len(ahead) else math.inf
        gap_back = self.robot_back - behind.max() if len(behind) else math.inf

        def allowed(gap, v_now):
            # poder frenar antes de stop_distance, contando con el retraso del scan
            room = gap - self.stop_distance - abs(v_now) * self.obstacle_latency
            return math.sqrt(2.0 * self.obstacle_decel * room) if room > 0.0 else 0.0

        # repulsión lateral: distancia de cada punto al rectángulo del robot (sin la cola)
        dx = np.maximum(x - self.robot_front, 0.0)
        dy = np.maximum(np.abs(y) - self.robot_half_width, 0.0)
        near = (x > self.robot_back) & (np.hypot(dx, dy) < self.avoid_distance)
        w = (self.avoid_distance - np.hypot(dx[near], dy[near])) / self.avoid_distance
        left = w[y[near] > 0.0].max() if np.any(y[near] > 0.0) else 0.0
        right = w[y[near] <= 0.0].max() if np.any(y[near] <= 0.0) else 0.0
        wz_avoid = self.k_avoid * (right - left)   # obstáculo a la izquierda -> gira a la derecha
        return (allowed(gap_front, self.last_vx), allowed(gap_back, self.last_vx),
                wz_avoid, gap_front)

    # ---------------------------------------------------------------- detecciones
    def on_people(self, msg):
        if self.odom is None:
            self.get_logger().warn('Sin odometría: no sigo a nadie', throttle_duration_sec=2.0)
            return
        t = self.now()
        # pose del robot cuando se tomó la imagen y antigüedad de la detección
        pose, age = self.pose_at(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        t_det = t - age
        candidates = {}
        for det in msg.detections:
            frame = det.bbox3d.frame_id or msg.header.frame_id
            forward, left = self.to_robot_plane(det, frame)
            distance = math.hypot(forward, left)
            # sin profundidad válida, yolo_ros deja la posición a 0
            if forward <= 0.1 or not math.isfinite(distance) or distance > self.max_range:
                continue
            candidates[det.id] = (distance, self.to_odom(forward, left, pose))
        if not candidates:
            return

        chosen = None
        if self.target_id:
            chosen = self.target_id if self.target_id in candidates else None
        elif self.current_id is not None:
            chosen = self.current_id if self.current_id in candidates else None
            if chosen is None and self.track is not None:
                # el tracker le ha cambiado el ID: la que esté donde debería estar
                ex, ey = self.track.predict(t, max_dt=1.0)

                def gap(k):
                    wx, wy = candidates[k][1]
                    return math.hypot(wx - ex, wy - ey)
                nearest = min(candidates, key=gap)
                if gap(nearest) <= self.reacquire_radius:
                    chosen = nearest
                    self.get_logger().info(
                        f'Persona {self.current_id} reenganchada como {chosen} '
                        f'({gap(nearest):.2f} m de donde se la esperaba)')
        else:
            # al empezar, cerca; buscando (ya olvidada), algo más lejos
            limit = self.search_max_distance if self.state == SEARCH else self.max_start_distance
            near = {k: v for k, v in candidates.items() if v[0] <= limit}
            chosen = min(near, key=lambda k: near[k][0]) if near else None

        if chosen is None:
            return
        wx, wy = candidates[chosen][1]
        if self.current_id is None or self.track is None:
            self.get_logger().info(
                f'Siguiendo a la persona {chosen} a {candidates[chosen][0]:.2f} m')
            self.track = PersonTrack(wx, wy, t_det, self.alpha, self.beta,
                                     self.max_person_speed)
        else:
            self.track.update(wx, wy, t_det)
        if self.state in (LOST, SEARCH):
            self.get_logger().info(
                f'Persona {chosen} encontrada tras {t - self.last_seen:.1f} s')
        self.current_id = chosen
        self.last_seen = t
        self.state = FOLLOW
        self.trail.append((t_det, wx, wy))
        while self.trail and t - self.trail[0][0] > self.person_trail_time:
            self.trail.popleft()

    # ---------------------------------------------------------------- control
    def publish(self, vx, wz):
        vx = ramp(vx, self.last_vx, self.max_accel, self.max_decel, self.dt)
        wz = ramp(wz, self.last_wz, self.max_angular_accel, self.max_angular_accel, self.dt)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = vx, wz
        self.cmd_pub.publish(cmd)
        self.last_vx, self.last_wz = vx, wz
        return cmd

    def brake(self):
        """Frena con rampa; deja de publicar al llegar a 0 (el driver para solo)."""
        if self.last_vx != 0.0 or self.last_wz != 0.0:
            self.publish(0.0, 0.0)

    def control(self):
        if self.odom is None:
            return
        t = self.now()
        if self.track is not None and t - self.last_seen > self.lost_timeout:
            self.get_logger().warn(
                f'Persona {self.current_id} sin ver {t - self.last_seen:.1f} s: la olvido')
            self.forget()

        if self.state == FOLLOW:
            if t - self.last_seen <= self.stop_timeout:
                self.follow(t)
                return
            self.state = LOST
            self.get_logger().warn(f'Sin ver a la persona {self.current_id}: frenando')

        if self.state == LOST:
            self.brake()
            if self.search_enabled and t - self.last_seen > self.search_delay:
                self.start_search(t)
            elif t - self.last_seen > self.lost_timeout:
                self.state = IDLE
            return

        if self.state == SEARCH:
            if t - self.search_start > self.search_timeout:
                self.get_logger().warn(
                    f'No encuentro a nadie tras {self.search_timeout:.0f} s buscando: me paro')
                self.state = IDLE
            else:
                self.publish(0.0, self.search_dir * self.search_angular)
                return

        self.brake()   # IDLE

    def start_search(self, t):
        # hacia el lado donde se espera que esté (la predicción tiene en cuenta hacia dónde iba)
        if self.track is not None:
            forward, left = self.to_robot(*self.track.predict(t, max_dt=1.0))
            side = math.atan2(left, forward)
        else:
            side = self.last_bearing
        self.search_dir = 1.0 if side >= 0.0 else -1.0
        self.search_start = t
        self.state = SEARCH
        self.get_logger().warn(
            f'Buscando a la persona: giro a la {"izquierda" if side >= 0.0 else "derecha"}')

    def follow(self, t):
        # dónde está ahora (predicción si la última detección es de hace un rato)
        forward, left = self.to_robot(*self.track.predict(t))
        distance = math.hypot(forward, left)
        bearing = math.atan2(left, forward)   # > 0 = a la izquierda
        self.last_bearing = bearing
        # velocidad de la persona alejándose del robot (componente en la línea robot-persona)
        x, y, _ = self.odom
        ux, uy = (self.track.x - x), (self.track.y - y)
        norm = math.hypot(ux, uy) or 1.0
        v_away = (self.track.vx * ux + self.track.vy * uy) / norm
        v_person = max(0.0, v_away)
        moving = v_person > self.moving_speed

        # giro: corrige el ángulo y además anticipa el de la persona andando de lado
        # (velocidad perpendicular a la línea robot-persona / distancia)
        yaw = self.odom[2]
        c, s = math.cos(yaw), math.sin(yaw)
        v_fwd = c * self.track.vx + s * self.track.vy
        v_left = -s * self.track.vx + c * self.track.vy
        v_side = -math.sin(bearing) * v_fwd + math.cos(bearing) * v_left
        wz_ff = self.k_angular_ff * v_side / max(distance, 0.5)
        wz = clamp(self.k_angular * bearing + wz_ff, self.max_angular)

        target = self.target_distance + self.time_gap * v_person
        error = distance - target
        vx = 0.0
        if distance < self.backoff_distance:
            # la persona viene hacia el robot: retrocede (no hay cámara detrás: despacio)
            vx = -min(self.k_backoff * (self.backoff_distance - distance), self.max_backward)
            # sin rampa al frenar si veníamos hacia delante: hay que parar ya
            if self.last_vx > 0.0:
                self.last_vx = 0.0
        elif moving or error > self.distance_deadband:
            if moving or abs(bearing) < self.turn_in_place_angle:
                # base = velocidad de la persona; el término proporcional corrige la distancia
                vx = (v_person + self.k_linear * error) * max(0.0, math.cos(bearing))
                vx = max(0.0, min(vx, self.max_linear))

        note = ''
        if self.obstacle_avoidance:
            pts = self.obstacles(t)
            if pts is None:
                vx = 0.0
                self.get_logger().warn('Sin /scan reciente: no me desplazo (solo giro)',
                                       throttle_duration_sec=2.0)
                note = ', SIN SCAN'
            elif len(pts):
                v_fwd, v_back, wz_avoid, gap = self.obstacle_limits(pts)
                if vx > v_fwd:
                    vx = v_fwd
                    # frenar por un obstáculo no espera a la rampa
                    self.last_vx = min(self.last_vx, v_fwd)
                    note = f', obstáculo a {gap:.2f} m'
                if vx < -v_back:
                    vx = -v_back
                    self.last_vx = max(self.last_vx, -v_back)
                    note = ', obstáculo detrás'
                if vx > 0.05 and wz_avoid != 0.0:
                    wz = clamp(wz + wz_avoid, self.max_angular)
                    note += f', esquiva {wz_avoid:+.2f}'

        cmd = self.publish(vx, wz)
        self.get_logger().info(
            f'id {self.current_id}: d={distance:.2f} m (obj {target:.2f}), '
            f'ang={math.degrees(bearing):+.0f} deg, v_persona={v_person:.2f}'
            f' -> vx={cmd.linear.x:.2f}, wz={cmd.angular.z:+.2f}{note}',
            throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
