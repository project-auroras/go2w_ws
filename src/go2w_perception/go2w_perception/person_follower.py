import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


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

    def predict(self, t):
        dt = t - self.t
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
    """Sigue a una persona de /people publicando /cmd_vel.

    Elige la persona más cercana (a menos de max_start_distance) y se queda con su
    ID de tracking. Gira para tenerla centrada y avanza para mantener la distancia.

    Para no quedarse atrás cuando la persona acelera, estima su velocidad respecto
    al suelo (posición en odom + filtro alfa-beta) y la usa como base de la orden:
        vx = v_persona + k_linear * (distancia - distancia_objetivo)
    La distancia objetivo crece con la velocidad (target_distance + time_gap * v),
    para tener margen de frenada. Los cambios de velocidad van con rampa
    (max_accel / max_decel / max_angular_accel).

    Si la persona se le acerca a menos de backoff_distance, retrocede (despacio,
    max_backward) sin dejar de mirarla.

    Si deja de verla un momento, sigue hacia donde el filtro predice que está. A los
    stop_timeout s frena (con rampa), pero la sigue recordando hasta lost_timeout.
    Si mientras tanto aparece una detección a menos de reacquire_radius de donde
    debería estar, se engancha a ella aunque el tracker le haya dado otro ID.

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
        self.max_angular = p('max_angular', 0.8).value
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
        rate = p('rate', 10.0).value
        self.dt = 1.0 / rate

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(DetectionArray, input_topic, self.on_people, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_timer(self.dt, self.control)

        self.current_id = None
        self.track = None           # PersonTrack en odom
        self.odom = None            # (x, y, yaw) del robot
        self.last_seen = None
        self.last_vx = self.last_wz = 0.0
        self.stopped = True
        self.braking = False
        self.get_logger().info(
            f'{input_topic} -> {cmd_vel_topic}, distancia objetivo {self.target_distance} m'
            f' + {self.time_gap} s x v_persona')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def to_odom(self, forward, left):
        x, y, yaw = self.odom
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

    def on_people(self, msg):
        if self.odom is None:
            self.get_logger().warn('Sin odometría: no sigo a nadie', throttle_duration_sec=2.0)
            return
        t = self.now()
        candidates = {}
        for det in msg.detections:
            frame = det.bbox3d.frame_id or msg.header.frame_id
            forward, left = self.to_robot_plane(det, frame)
            distance = math.hypot(forward, left)
            # sin profundidad válida, yolo_ros deja la posición a 0
            if forward <= 0.1 or not math.isfinite(distance) or distance > self.max_range:
                continue
            candidates[det.id] = (distance, self.to_odom(forward, left))
        if not candidates:
            return

        if self.target_id:
            chosen = self.target_id if self.target_id in candidates else None
        elif self.current_id is not None:
            chosen = self.current_id if self.current_id in candidates else None
            if chosen is None and self.track is not None:
                # el tracker le ha cambiado el ID: la que esté donde debería estar
                ex, ey = self.track.predict(t)

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
            near = {k: v for k, v in candidates.items() if v[0] <= self.max_start_distance}
            chosen = min(near, key=lambda k: near[k][0]) if near else None

        if chosen is None:
            return
        wx, wy = candidates[chosen][1]
        if self.current_id is None or self.track is None:
            self.get_logger().info(
                f'Siguiendo a la persona {chosen} a {candidates[chosen][0]:.2f} m')
            self.track = PersonTrack(wx, wy, t, self.alpha, self.beta, self.max_person_speed)
        else:
            self.track.update(wx, wy, t)
        self.current_id = chosen
        self.last_seen = t

    def publish(self, vx, wz, ramp_vx=True):
        if ramp_vx:
            vx = ramp(vx, self.last_vx, self.max_accel, self.max_decel, self.dt)
        wz = ramp(wz, self.last_wz, self.max_angular_accel, self.max_angular_accel, self.dt)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = vx, wz
        self.cmd_pub.publish(cmd)
        self.last_vx, self.last_wz = vx, wz
        return cmd

    def control(self):
        if self.current_id is None or self.track is None or self.odom is None:
            return
        t = self.now()
        elapsed = t - self.last_seen
        if elapsed > self.lost_timeout:
            self.get_logger().warn(
                f'Persona {self.current_id} perdida ({elapsed:.1f} s): la olvido')
            self.forget()
            return
        if elapsed > self.stop_timeout:
            if not self.stopped and not self.braking:
                self.braking = True
                self.get_logger().warn(
                    f'Sin ver a la persona {self.current_id}: frenando, esperando '
                    f'{self.lost_timeout:.0f} s a que vuelva')
            # frena con rampa hasta pararse del todo
            cmd = self.publish(0.0, 0.0)
            self.stopped = cmd.linear.x == 0.0 and cmd.angular.z == 0.0
            return

        # dónde está ahora (predicción si la última detección es de hace un rato)
        forward, left = self.to_robot(*self.track.predict(t))
        distance = math.hypot(forward, left)
        bearing = math.atan2(left, forward)   # > 0 = a la izquierda
        # velocidad de la persona alejándose del robot (componente en la línea robot-persona)
        x, y, _ = self.odom
        ux, uy = (self.track.x - x), (self.track.y - y)
        norm = math.hypot(ux, uy) or 1.0
        v_away = (self.track.vx * ux + self.track.vy * uy) / norm
        v_person = max(0.0, v_away)
        moving = v_person > self.moving_speed

        wz = clamp(self.k_angular * bearing, self.max_angular)

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

        cmd = self.publish(vx, wz)
        self.stopped = self.braking = False
        self.get_logger().info(
            f'id {self.current_id}: d={distance:.2f} m (obj {target:.2f}), '
            f'ang={math.degrees(bearing):+.0f} deg, v_persona={v_person:.2f}'
            f' -> vx={cmd.linear.x:.2f}, wz={cmd.angular.z:+.2f}',
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
