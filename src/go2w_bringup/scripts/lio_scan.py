#!/usr/bin/env python3
"""/scan for slam_toolbox from the last second of FAST-LIO clouds.

One L1 cloud cut to a height band leaves ~60 valid beams of 723 (most L1 points are floor),
too few for slam_toolbox's scan matcher. This node keeps the deskewed clouds of FAST-LIO
(/cloud_registered, frame camera_init) for `window` seconds, moves them to the current
base_link with the /Odometry of the same stamp (not the odom TF: go2w_driver drops the z
drift of FAST-LIO there, which would shift the height band) and cuts the band.
The scan has the stamp of the cloud, the same as the odom -> base_footprint TF.
"""
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def matrix(q, t):
    x, y, z, w = q
    m = np.eye(4)
    m[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = t
    return m


def xyz(cloud):
    offsets = {f.name: f.offset for f in cloud.fields}
    raw = np.frombuffer(cloud.data, dtype=np.uint8).reshape(-1, cloud.point_step)
    cols = [raw[:, offsets[k]:offsets[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz']
    p = np.stack(cols, axis=1).astype(np.float64)
    return p[np.isfinite(p).all(axis=1)]


class LioScan(Node):
    def __init__(self):
        super().__init__('lio_scan')
        self.window = self.declare_parameter('window', 1.0).value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.lidar_frame = self.declare_parameter('lidar_frame', 'utlidar_lidar').value
        self.min_height = self.declare_parameter('min_height', -0.35).value
        self.max_height = self.declare_parameter('max_height', 1.0).value
        # Box around base_link with the robot itself (legs, wheels, back)
        self.self_box = self.declare_parameter('self_box', [-0.55, 0.45, -0.35, 0.35]).value
        self.angle_increment = self.declare_parameter('angle_increment', 0.0087).value
        self.range_min = self.declare_parameter('range_min', 0.3).value
        self.range_max = self.declare_parameter('range_max', 15.0).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_to_lidar = None
        self.poses = deque(maxlen=50)
        self.clouds = deque()

        self.n_beams = int(np.ceil(2.0 * np.pi / self.angle_increment))
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/Odometry', self.on_odometry, 20)
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_cloud, 20)

    def on_odometry(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.poses.append((Time.from_msg(msg.header.stamp).nanoseconds,
                           matrix((q.x, q.y, q.z, q.w), (p.x, p.y, p.z))))

    def on_cloud(self, msg):
        if self.base_to_lidar is None:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, self.lidar_frame, Time())
            except TransformException as e:
                self.get_logger().warn(f'Waiting for TF {self.base_frame} -> {self.lidar_frame}: {e}',
                                       throttle_duration_sec=2.0)
                return
            r, tr = t.transform.rotation, t.transform.translation
            self.base_to_lidar = matrix((r.x, r.y, r.z, r.w), (tr.x, tr.y, tr.z))

        # FAST-LIO publishes /Odometry before the cloud, both stamped with the scan end
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        pose = next((m for s, m in reversed(self.poses) if s == stamp), None)
        if pose is None:
            self.get_logger().warn('No /Odometry with the stamp of the cloud',
                                   throttle_duration_sec=2.0)
            return

        self.clouds.append((stamp, xyz(msg)))
        while self.clouds and stamp - self.clouds[0][0] > self.window * 1e9:
            self.clouds.popleft()

        # camera_init -> L1 now -> base_link
        world_to_base = self.base_to_lidar @ np.linalg.inv(pose)
        p = np.vstack([c for _, c in self.clouds])
        p = p @ world_to_base[:3, :3].T + world_to_base[:3, 3]

        x0, x1, y0, y1 = self.self_box
        keep = (p[:, 2] > self.min_height) & (p[:, 2] < self.max_height)
        keep &= ~((p[:, 0] > x0) & (p[:, 0] < x1) & (p[:, 1] > y0) & (p[:, 1] < y1))
        p = p[keep]
        r = np.hypot(p[:, 0], p[:, 1])
        ok = (r >= self.range_min) & (r <= self.range_max)
        p, r = p[ok], r[ok]
        beam = ((np.arctan2(p[:, 1], p[:, 0]) + np.pi) / self.angle_increment).astype(int)
        beam = np.clip(beam, 0, self.n_beams - 1)
        ranges = np.full(self.n_beams, np.inf, dtype=np.float32)
        np.minimum.at(ranges, beam, r.astype(np.float32))

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = -np.pi
        scan.angle_max = -np.pi + (self.n_beams - 1) * self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    rclpy.spin(LioScan())


if __name__ == '__main__':
    main()
