import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


def clamp(value, limit):
    return max(-limit, min(limit, value))


class PersonFollower(Node):
    """Sigue a una persona de /people publicando /cmd_vel.

    Elige la persona más cercana y se queda con su ID de tracking hasta perderla.
    Avanza para mantener target_distance y gira para tenerla centrada. Nunca va
    marcha atrás. Si no la ve durante lost_timeout, se para y suelta el ID.

    Las posiciones pueden venir en un frame óptico de cámara (z delante, x derecha,
    nombre con 'optical') o en uno de robot como base_link (x delante, y izquierda).
    """

    def __init__(self):
        super().__init__('person_follower')
        p = self.declare_parameter
        input_topic = p('input_topic', '/people').value
        cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel').value
        self.target_id = p('target_id', '').value          # '' = la más cercana
        self.target_distance = p('target_distance', 1.5).value
        self.distance_deadband = p('distance_deadband', 0.15).value
        self.max_range = p('max_range', 8.0).value
        self.k_linear = p('k_linear', 0.6).value
        self.k_angular = p('k_angular', 1.5).value
        self.max_linear = p('max_linear', 0.5).value
        self.max_angular = p('max_angular', 0.8).value
        self.turn_in_place_angle = p('turn_in_place_angle', 0.6).value  # rad
        self.lost_timeout = p('lost_timeout', 1.0).value
        rate = p('rate', 10.0).value

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(DetectionArray, input_topic, self.on_people, 10)
        self.create_timer(1.0 / rate, self.control)

        self.current_id = None
        self.target = None          # (distance, bearing): bearing > 0 = a la izquierda
        self.last_seen = None
        self.stopped = True
        self.get_logger().info(
            f'{input_topic} -> {cmd_vel_topic}, distancia objetivo {self.target_distance} m')

    @staticmethod
    def to_robot_plane(detection, frame_id):
        """Devuelve (delante, izquierda) en metros."""
        pos = detection.bbox3d.center.position
        if 'optical' in frame_id:
            return pos.z, -pos.x
        return pos.x, pos.y

    def on_people(self, msg):
        candidates = {}
        for det in msg.detections:
            frame = det.bbox3d.frame_id or msg.header.frame_id
            forward, left = self.to_robot_plane(det, frame)
            distance = math.hypot(forward, left)
            # sin profundidad válida, yolo_ros deja la posición a 0
            if forward <= 0.1 or not math.isfinite(distance) or distance > self.max_range:
                continue
            candidates[det.id] = (distance, math.atan2(left, forward))
        if not candidates:
            return

        if self.target_id:
            chosen = self.target_id if self.target_id in candidates else None
        elif self.current_id is not None:
            # no cambia de persona por un frame sin detección: espera a lost_timeout
            chosen = self.current_id if self.current_id in candidates else None
        else:
            chosen = min(candidates, key=lambda k: candidates[k][0])

        if chosen is None:
            return
        if chosen != self.current_id:
            self.get_logger().info(
                f'Siguiendo a la persona {chosen} a {candidates[chosen][0]:.2f} m')
        self.current_id = chosen
        self.target = candidates[chosen]
        self.last_seen = self.get_clock().now()

    def control(self):
        if self.last_seen is None:
            return
        elapsed = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        if elapsed > self.lost_timeout:
            if not self.stopped:
                self.get_logger().warn(
                    f'Persona {self.current_id} perdida ({elapsed:.1f} s): parado')
                self.cmd_pub.publish(Twist())
                self.stopped = True
                self.current_id = None
            return

        distance, bearing = self.target
        cmd = Twist()
        cmd.angular.z = clamp(self.k_angular * bearing, self.max_angular)

        error = distance - self.target_distance
        if error > self.distance_deadband and abs(bearing) < self.turn_in_place_angle:
            # avanza menos cuanto más ladeada esté la persona
            cmd.linear.x = min(self.k_linear * error, self.max_linear) * math.cos(bearing)

        self.cmd_pub.publish(cmd)
        self.stopped = False
        self.get_logger().info(
            f'id {self.current_id}: d={distance:.2f} m, ang={math.degrees(bearing):+.0f} deg'
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
