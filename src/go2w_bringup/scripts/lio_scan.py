#!/usr/bin/env python3
"""/scan for slam_toolbox from the last second of FAST-LIO clouds.

One L1 cloud cut to a height band leaves ~60 valid beams of 723 (most L1 points are floor),
too few for slam_toolbox's scan matcher. This node keeps the deskewed clouds of FAST-LIO
(/cloud_registered, frame camera_init) for `window` seconds, moves them to the current
base_link with the /Odometry of the same stamp (not the odom TF: go2w_driver drops the z
drift of FAST-LIO there, which would shift the height band) and cuts the band.
The scan has the stamp of the cloud, the same as the odom -> base_footprint TF.
"""
import array
import os
# One BLAS thread: the matrices are tiny, and a threaded OpenBLAS (aarch64) keeps its pool
# spinning between clouds. Must be set before numpy is imported.
for _var in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS'):
    os.environ.setdefault(_var, '1')
from collections import deque  # noqa: E402

import numpy as np  # noqa: E402
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
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                      'offsets': [offsets[k] for k in 'xyz'], 'itemsize': cloud.point_step})
    raw = np.frombuffer(cloud.data, dtype=dtype, count=cloud.width * cloud.height)
    p = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
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

        # camera_init -> L1 now -> base_link. Height first: most points are floor, so x, y,
        # range and angle are computed only for the points inside the band.
        world_to_base = self.base_to_lidar @ np.linalg.inv(pose)
        p = np.vstack([c for _, c in self.clouds])
        rot, tr = world_to_base[:3, :3], world_to_base[:3, 3]
        z = p @ rot[2] + tr[2]
        p = p[(z > self.min_height) & (z < self.max_height)]
        x = p @ rot[0] + tr[0]
        y = p @ rot[1] + tr[1]

        x0, x1, y0, y1 = self.self_box
        r = np.hypot(x, y)
        ok = ~((x > x0) & (x < x1) & (y > y0) & (y < y1))
        ok &= (r >= self.range_min) & (r <= self.range_max)
        x, y, r = x[ok], y[ok], r[ok].astype(np.float32)
        beam = ((np.arctan2(y, x) + np.pi) / self.angle_increment).astype(int)
        beam = np.clip(beam, 0, self.n_beams - 1)
        # minimum range per beam: sort by beam then range, keep the first of each beam
        # (same result as np.minimum.at, which is very slow in numpy < 1.25)
        ranges = np.full(self.n_beams, np.inf, dtype=np.float32)
        if len(r):
            order = np.lexsort((r, beam))
            b = beam[order]
            first = np.r_[True, b[1:] != b[:-1]]
            ranges[b[first]] = r[order][first]

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = -np.pi
        scan.angle_max = -np.pi + (self.n_beams - 1) * self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = array.array('f', ranges.tobytes())   # skips the per-element check of a list
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    rclpy.spin(LioScan())


if __name__ == '__main__':
    main()
