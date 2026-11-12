import math

from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray

from go2w_perception.semantic_common import (detection_frame, lookup, quaternion_to_yaw,
                                             transform_point, valid_position,
                                             yaw_to_quaternion)


def fit_line(points, min_spread):
    """Recta y = a + b x por mínimos cuadrados; con poca extensión en x, y = mediana."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    if len(points) >= 3 and max(xs) - min(xs) >= min_spread:
        mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
        sxx = sum((x - mx) ** 2 for x in xs)
        b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
        return my - b * mx, b
    ys.sort()
    return ys[len(ys) // 2], 0.0


class RowFollower(Node):
    """Recorre el pasillo entre dos surcos mandando metas a EasyNav.

    Se activa con True en /semantic/row_follow (False lo para). Guarda en odom las
    detecciones de `row_classes` (Row) de los últimos window_s s y, en cada ciclo, las
    pasa al robot (x delante, y izquierda). Las de la izquierda (y > 0) y las de la
    derecha son los dos surcos: ajusta una recta a cada uno y el eje del pasillo es la
    media. Si solo ve un surco, el eje está a aisle_half_width de él. Si no ve surcos
    pero sí `center_classes` (Trace, las rodadas del pasillo), usa su posición media.

    Cada goal_period s publica en /goal_pose un punto del eje a `lookahead` m, mirando
    a lo largo del pasillo (EasyNav reemplaza la meta anterior). Cuando lleva
    end_timeout s sin ver surcos delante (ha llegado a la cabecera), manda una última
    meta end_advance m más allá y se desactiva.
    """

    def __init__(self):
        super().__init__('row_follower')
        p = self.declare_parameter
        self.input_topic = p('input_topic', '/yolo/cam0/detections_3d').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.map_frame = p('map_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_link').value
        self.row_classes = set(p('row_classes', ['Row']).value)
        self.center_classes = set(p('center_classes', ['Trace']).value)
        self.min_score = p('min_score', 0.4).value
        self.window_s = p('window_s', 3.0).value
        self.min_ahead = p('min_ahead', 0.3).value
        self.max_ahead = p('max_ahead', 6.0).value
        self.max_lateral = p('max_lateral', 2.0).value
        self.aisle_half_width = p('aisle_half_width', 0.75).value
        self.min_spread = p('min_spread', 1.0).value
        self.max_slope = p('max_slope', 0.4).value
        self.smoothing = p('smoothing', 0.5).value
        self.lookahead = p('lookahead', 3.0).value
        self.goal_period = p('goal_period', 1.0).value
        self.end_timeout = p('end_timeout', 3.0).value
        self.end_advance = p('end_advance', 1.5).value
        goal_topic = p('goal_topic', '/goal_pose').value

        self.points = []          # (t, clase, x, y) en odom
        self.active = False
        self.line = None          # (a, b) suavizada, en el robot
        self.last_rows = 0.0      # última vez que vio surcos delante
        self.started = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.status_pub = self.create_publisher(String, '/semantic/row_status', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/semantic/row_markers', 10)
        self.create_subscription(DetectionArray, self.input_topic, self.on_detections, 10)
        self.create_subscription(Bool, '/semantic/row_follow', self.on_enable, 10)
        self.create_timer(self.goal_period, self.step)
        self.get_logger().info(f'Seguir surco: {sorted(self.row_classes)} (+ '
                               f'{sorted(self.center_classes)}) -> {goal_topic}. '
                               f'Activar con /semantic/row_follow')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_enable(self, msg):
        if msg.data and not self.active:
            self.active = True
            self.line = None
            self.started = self.last_rows = self.now_s()
            self.status('siguiendo el surco')
        elif not msg.data and self.active:
            self.active = False
            self.status('parado por /semantic/row_follow (EasyNav sigue con la última meta)')

    def on_detections(self, msg):
        classes = self.row_classes | self.center_classes
        stamp = Time.from_msg(msg.header.stamp)
        t = self.now_s()
        tfs = {}
        for det in msg.detections:
            if det.class_name not in classes or det.score < self.min_score:
                continue
            c = det.bbox3d.center.position
            if not valid_position(c):
                continue
            frame = detection_frame(det, msg.header.frame_id)
            if frame not in tfs:
                tfs[frame] = lookup(self.tf_buffer, self.odom_frame, frame, stamp)
            if tfs[frame] is None:
                continue
            x, y, _ = transform_point(tfs[frame], c.x, c.y, c.z)
            self.points.append((t, det.class_name, x, y))
        self.points = [p for p in self.points if t - p[0] < self.window_s]

    def step(self):
        if not self.active:
            return
        now = self.now_s()
        to_robot = lookup(self.tf_buffer, self.robot_frame, self.odom_frame)
        if to_robot is None:
            self.status(f'sin TF {self.odom_frame} -> {self.robot_frame}')
            return

        rows, centers = [], []
        for _, cls, ox, oy in self.points:
            x, y, _ = transform_point(to_robot, ox, oy, 0.0)
            if not (self.min_ahead < x < self.max_ahead and abs(y) < self.max_lateral):
                continue
            (rows if cls in self.row_classes else centers).append((x, y))

        line = self.estimate(rows, centers)
        if rows:
            self.last_rows = now
        if line is None or now - self.last_rows > self.end_timeout:
            if now - self.started < self.end_timeout + self.goal_period:
                self.status('no veo surcos delante: no empiezo')
            else:
                self.finish()
            self.active = False
            return

        if self.line is None:
            self.line = line
        else:
            k = self.smoothing
            self.line = tuple(k * old + (1 - k) * new for old, new in zip(self.line, line))
        self.publish_goal(self.lookahead)
        self.publish_markers(rows, centers)

    def estimate(self, rows, centers):
        """Calcula el eje del pasillo en el robot: (a, b) de y = a + b x."""
        left = [p for p in rows if p[1] > 0.0]
        right = [p for p in rows if p[1] <= 0.0]
        sides = []
        if left:
            a, b = fit_line(left, self.min_spread)
            sides.append((a - self.aisle_half_width, b))
        if right:
            a, b = fit_line(right, self.min_spread)
            sides.append((a + self.aisle_half_width, b))
        if len(sides) == 2:
            a = 0.5 * (sides[0][0] + sides[1][0])
            b = 0.5 * (sides[0][1] + sides[1][1])
        elif sides:
            a, b = sides[0]
        elif centers:
            a, b = sum(p[1] for p in centers) / len(centers), 0.0
        else:
            return None
        return a, max(-self.max_slope, min(self.max_slope, b))

    def publish_goal(self, distance):
        a, b = self.line
        x, y, yaw = distance, a + b * distance, math.atan(b)
        to_map = lookup(self.tf_buffer, self.map_frame, self.robot_frame)
        if to_map is None:
            self.status(f'sin TF {self.robot_frame} -> {self.map_frame}')
            return
        gx, gy, gz = transform_point(to_map, x, y, 0.0)
        robot_yaw = quaternion_to_yaw(to_map.transform.rotation)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = gx, gy, gz
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = yaw_to_quaternion(robot_yaw + yaw)
        self.goal_pub.publish(goal)
        self.get_logger().info(f'eje: y = {a:+.2f} {b:+.2f} x -> meta ({gx:.1f}, {gy:.1f})',
                               throttle_duration_sec=2.0)

    def finish(self):
        if self.line is not None:
            self.line = (self.line[0], 0.0)
            self.publish_goal(self.end_advance)
        self.status('fin del surco (cabecera)')

    def publish_markers(self, rows, centers):
        a, b = self.line
        line = Marker()
        line.header.frame_id = self.robot_frame
        line.ns = 'eje'
        line.type = Marker.LINE_STRIP
        line.scale.x = 0.05
        line.color.g = line.color.a = 1.0
        line.pose.orientation.w = 1.0
        for x in (0.0, self.lookahead):
            line.points.append(Point(x=x, y=a + b * x, z=-0.4))
        pts = Marker()
        pts.header.frame_id = self.robot_frame
        pts.ns = 'detecciones'
        pts.type = Marker.SPHERE_LIST
        pts.scale.x = pts.scale.y = pts.scale.z = 0.12
        pts.color.r = pts.color.a = 1.0
        pts.pose.orientation.w = 1.0
        pts.points = [Point(x=x, y=y, z=-0.4) for x, y in rows + centers]
        self.markers_pub.publish(MarkerArray(markers=[line, pts]))

    def status(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = RowFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
