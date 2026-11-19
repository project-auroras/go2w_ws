#!/usr/bin/env python3
"""Checks that the robot inputs are alive before a validation test (run on the NX, bringup up).

  scripts/validation/preflight.py [--seconds 4]

Measures the rate of the sensor and odometry topics and the odom -> base_footprint TF.
Exit code 0 if everything is within limits, 1 otherwise (record.sh asks before going on).
"""
import argparse
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

# topic: (type, minimum Hz). /utlidar/cloud is 15.4 Hz and /utlidar/imu 250 Hz on the robot
CHECKS = {
    '/utlidar/cloud': (PointCloud2, 14.0),
    '/utlidar/imu': (Imu, 230.0),
    '/lidar_imu': (Imu, 230.0),
    '/Odometry': (Odometry, 14.0),
    '/odom': (Odometry, 14.0),
    '/scan': (LaserScan, 14.0),
}


class Preflight(Node):
    def __init__(self):
        super().__init__('validation_preflight')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.stamps = {t: [] for t in CHECKS}
        for topic, (msg_type, _) in CHECKS.items():
            self.create_subscription(msg_type, topic,
                                     lambda m, t=topic: self.stamps[t].append(time.monotonic()), qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--seconds', type=float, default=4.0)
    a = p.parse_args()

    rclpy.init()
    node = Preflight()
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    ok = True
    print(f'{"topic":16s} {"Hz":>7s} {"min":>6s}  gap max')
    for topic, (_, min_hz) in CHECKS.items():
        s = node.stamps[topic]
        hz = (len(s) - 1) / (s[-1] - s[0]) if len(s) > 2 else 0.0
        gap = max((b - a for a, b in zip(s, s[1:])), default=float('inf')) * 1e3
        good = hz >= min_hz
        ok &= good
        print(f'{topic:16s} {hz:7.1f} {min_hz:6.0f}  {gap:6.0f} ms  {"OK" if good else "FAIL"}')
    try:
        node.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        print('TF odom -> base_footprint       OK')
    except Exception as e:  # noqa: BLE001 - any TF error is a failed check
        ok = False
        print(f'TF odom -> base_footprint       FAIL ({type(e).__name__})')
    node.destroy_node()
    rclpy.shutdown()
    print('PREFLIGHT', 'OK' if ok else 'FAILED')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
