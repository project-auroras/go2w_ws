"""Utilidades comunes de la navegación semántica: TF sin tf2_geometry_msgs y quaterniones."""

import math

from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException


def yaw_to_quaternion(yaw):
    """(x, y, z, w) de un giro yaw alrededor de z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def transform_point(transform, x, y, z):
    """Aplica un geometry_msgs/TransformStamped al punto (x, y, z)."""
    t = transform.transform.translation
    q = transform.transform.rotation
    # v' = q v q^-1, desarrollado
    ux, uy, uz, w = q.x, q.y, q.z, q.w
    cx = uy * z - uz * y
    cy = uz * x - ux * z
    cz = ux * y - uy * x
    ccx = uy * cz - uz * cy
    ccy = uz * cx - ux * cz
    ccz = ux * cy - uy * cx
    return (x + 2.0 * (w * cx + ccx) + t.x,
            y + 2.0 * (w * cy + ccy) + t.y,
            z + 2.0 * (w * cz + ccz) + t.z)


def lookup(buffer, target, source, stamp=None, timeout=0.05):
    """TF source -> target en stamp (o la última si no la hay). None si no existe."""
    try:
        if stamp is not None:
            return buffer.lookup_transform(target, source, stamp, Duration(seconds=timeout))
    except TransformException:
        pass
    try:
        return buffer.lookup_transform(target, source, Time())
    except TransformException:
        return None


def detection_frame(detection, header_frame):
    return detection.bbox3d.frame_id or header_frame


def valid_position(p):
    return all(math.isfinite(v) for v in (p.x, p.y, p.z)) and (p.x, p.y, p.z) != (0.0, 0.0, 0.0)
