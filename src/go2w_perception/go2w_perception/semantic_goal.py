import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from yolo_msgs.msg import DetectionArray

from go2w_perception.semantic_common import lookup, yaw_to_quaternion


class SemanticGoal(Node):
    """Ir a un objeto del mapa semántico: convierte "Post" o "Post:3" en una meta de EasyNav.

    Escucha /semantic/goal_request (std_msgs/String):
      - "Clase": el objeto confirmado de esa clase más cercano al robot.
      - "Clase:N": el objeto con id N (el número de la etiqueta en RViz).
      - "cancel": deja de vigilar la meta (EasyNav sigue con la suya; para pararlo,
        mandar otra meta o cancelar en EasyNav).
    Publica en /goal_pose (lo que lee EasyNav) un punto a `standoff` m del borde del
    objeto, en la recta robot -> objeto, mirando al objeto, en map_frame y con la z del
    robot (EasyNav compara también la altura). En /semantic/goal_status publica el estado.
    """

    def __init__(self):
        super().__init__('semantic_goal')
        p = self.declare_parameter
        self.map_frame = p('map_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_link').value
        self.standoff = p('standoff', 1.0).value
        self.arrive_tolerance = p('arrive_tolerance', 0.4).value
        landmarks_topic = p('landmarks_topic', '/semantic/landmarks').value
        goal_topic = p('goal_topic', '/goal_pose').value

        self.landmarks = []
        self.goal = None        # (x, y, descripción)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.status_pub = self.create_publisher(String, '/semantic/goal_status', 10)
        self.create_subscription(DetectionArray, landmarks_topic, self.on_landmarks, 10)
        self.create_subscription(String, '/semantic/goal_request', self.on_request, 10)
        self.create_timer(0.5, self.check_arrival)
        self.get_logger().info(f'/semantic/goal_request -> {goal_topic} '
                               f'(standoff {self.standoff} m)')

    def on_landmarks(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(f'Landmarks en {msg.header.frame_id}, se esperaba '
                                   f'{self.map_frame}', throttle_duration_sec=10.0)
            return
        self.landmarks = msg.detections

    def robot_pose(self):
        tf = lookup(self.tf_buffer, self.map_frame, self.robot_frame)
        if tf is None:
            return None
        t = tf.transform.translation
        return t.x, t.y, t.z

    def on_request(self, msg):
        text = msg.data.strip()
        if text.lower() == 'cancel':
            self.goal = None
            self.status('cancelada')
            return
        cls, _, wanted_id = text.partition(':')
        robot = self.robot_pose()
        if robot is None:
            self.status(f'sin TF {self.map_frame} -> {self.robot_frame}')
            return
        candidates = [d for d in self.landmarks if d.class_name.lower() == cls.lower()
                      and (not wanted_id or d.id == wanted_id)]
        if not candidates:
            known = sorted({f'{d.class_name}:{d.id}' for d in self.landmarks})
            self.status(f'"{text}" no está en el mapa. Hay: {", ".join(known) or "nada"}')
            return
        target = min(candidates, key=lambda d: math.hypot(
            d.bbox3d.center.position.x - robot[0], d.bbox3d.center.position.y - robot[1]))
        self.send_goal(target, robot)

    def send_goal(self, target, robot):
        c = target.bbox3d.center.position
        dx, dy = c.x - robot[0], c.y - robot[1]
        dist = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        # radio del objeto en planta: los Row/Trace son largos, se usa el lado menor
        radius = 0.5 * min(target.bbox3d.size.x, target.bbox3d.size.y)
        back = min(self.standoff + radius, dist)   # si ya está más cerca, solo gira
        gx, gy = c.x - back * math.cos(yaw), c.y - back * math.sin(yaw)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = gx, gy, robot[2]
        q = yaw_to_quaternion(yaw)
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = q
        self.goal_pub.publish(goal)
        name = f'{target.class_name}:{target.id}'
        self.goal = (gx, gy, name)
        self.status(f'yendo a {name} en ({c.x:.1f}, {c.y:.1f}); meta ({gx:.1f}, {gy:.1f}), '
                    f'a {dist:.1f} m')

    def check_arrival(self):
        if self.goal is None:
            return
        robot = self.robot_pose()
        if robot is None:
            return
        if math.hypot(self.goal[0] - robot[0], self.goal[1] - robot[1]) < self.arrive_tolerance:
            self.status(f'llegado a {self.goal[2]}')
            self.goal = None

    def status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = SemanticGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
