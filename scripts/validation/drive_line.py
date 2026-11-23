#!/usr/bin/env python3
"""Straight line at constant speed for the L2 `line` test (docs/validation/README.md).

  scripts/validation/drive_line.py               # 10 m at 0.3 m/s
  scripts/validation/drive_line.py 10 --speed 1.0

/cmd_vel at a steady 50 Hz on the NX (`ros2 topic pub` from a terminal is irregular, and the driver
stops the robot after 0.5 s without cmd_vel), with ramps at both ends (a step to full speed jerks
and slips the wheels). Only linear.x: the heading is never corrected, the wheel drift is part of
the test. It stops when /odom has moved `distance` from the start (the tape measures the real
distance, so using the odometry to stop does not bias the scale), or after a time limit.
Ctrl+C stops the robot.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

RATE = 50.0


class DriveLine(Node):
    def __init__(self, distance, speed, ramp):
        super().__init__('drive_line')
        self.distance, self.speed, self.ramp = distance, speed, ramp
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.start = self.pos = None
        self.t0 = None
        self.v = 0.0
        self.state = 'wait_odom'
        self.limit = distance / speed * 1.5 + 2 * ramp + 3.0
        self.create_timer(1.0 / RATE, self.tick)

    def on_odom(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.start is None:
            self.start = self.pos

    def travelled(self):
        return math.hypot(self.pos[0] - self.start[0], self.pos[1] - self.start[1])

    def tick(self):
        now = time.monotonic()
        if self.state == 'wait_odom':
            if self.pos is None:
                self.get_logger().info('waiting for /odom', throttle_duration_sec=2.0)
                return
            self.t0, self.state = now, 'drive'
            self.get_logger().info(f'{self.distance:.1f} m at {self.speed:.2f} m/s')
        d = self.travelled()
        dv = self.speed / (self.ramp * RATE)
        if self.state == 'drive':
            # start braking so that the ramp ends at `distance` (v^2 / 2a, a = speed / ramp)
            brake = self.v * self.v * self.ramp / (2 * self.speed)
            if d + brake >= self.distance or now - self.t0 > self.limit:
                if now - self.t0 > self.limit:
                    self.get_logger().warn('time limit reached, stopping')
                self.state = 'brake'
            else:
                self.v = min(self.speed, self.v + dv)
        if self.state == 'brake':
            self.v = max(0.0, self.v - dv)
            if self.v == 0.0:
                self.state = 'stopped'
                self.t_stop = now
        cmd = Twist()
        cmd.linear.x = self.v
        self.pub.publish(cmd)
        if self.state == 'stopped' and now - self.t_stop > 0.5:
            self.get_logger().info(f'stopped: {d:.2f} m by odometry in {now - self.t0:.1f} s')
            raise SystemExit


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('distance', nargs='?', type=float, default=10.0, help='m (default 10)')
    ap.add_argument('--speed', type=float, default=0.3, help='m/s (default 0.3)')
    ap.add_argument('--ramp', type=float, default=1.0, help='s from 0 to speed and back (default 1)')
    a = ap.parse_args()
    rclpy.init()
    node = DriveLine(a.distance, a.speed, a.ramp)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        for _ in range(5):   # zero velocity, also on Ctrl+C
            node.pub.publish(Twist())
            time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
