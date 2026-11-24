#!/usr/bin/env python3
"""Smooth open-loop motions for the validation tests (docs/validation/README.md).

  scripts/validation/drive_line.py                  # L2: 10 m straight at 0.3 m/s
  scripts/validation/drive_line.py 10 --speed 1.0   # L2 at 1 m/s
  scripts/validation/drive_line.py --spin 3         # L3: 3 turns in place to the left (-3: right)

/cmd_vel at a steady 50 Hz on the NX (`ros2 topic pub` from a terminal is irregular, and the driver
stops the robot after 0.5 s without cmd_vel), with ramps at both ends (a step to full speed jerks
and slips the wheels). Only linear.x or only angular.z: the motion is never corrected, the wheel
drift is part of the test. It stops when /odom has moved `distance` (or turned `spin` turns) from
the start; the tape gives the real value, so using the odometry to stop does not bias the result.

It does not start until /odom has been still for --rest seconds (a run that starts while the robot
is still moving is invalid), and after stopping it keeps sending zero for --hold seconds, so the
bag gets the rest windows that validate needs. Ctrl+C stops the robot.
"""
import argparse
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

RATE = 50.0
STILL_XY, STILL_YAW = 0.05, math.radians(6)   # above the jitter of the standing robot (L1)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Drive(Node):
    def __init__(self, a):
        super().__init__('drive_line')
        self.spin = a.spin is not None
        # target and cruise speed in m or rad, always positive; `sign` is the direction
        self.target = abs(a.spin) * 2 * math.pi if self.spin else a.distance
        self.speed = a.wz if self.spin else a.speed
        self.sign = math.copysign(1, a.spin) if self.spin else 1.0
        self.ramp, self.rest, self.hold = a.ramp, a.rest, a.hold
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.hist = deque()        # (t, x, y, yaw) of the last `rest` seconds
        self.pose = None
        self.start = None
        self.turned = 0.0          # unwrapped yaw since the start
        self.v = 0.0
        self.state = 'wait_rest'
        self.limit = self.target / self.speed * 1.5 + 2 * self.ramp + 3.0
        self.create_timer(1.0 / RATE, self.tick)

    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = yaw_of(p.orientation)
        if self.pose is not None and self.state in ('drive', 'brake', 'stopped'):
            self.turned += math.atan2(math.sin(yaw - self.pose[2]), math.cos(yaw - self.pose[2]))
        self.pose = (p.position.x, p.position.y, yaw)
        now = time.monotonic()
        self.hist.append((now, *self.pose))
        while self.hist and now - self.hist[0][0] > self.rest:
            self.hist.popleft()

    def at_rest(self):
        if len(self.hist) < 5 or self.hist[-1][0] - self.hist[0][0] < 0.9 * self.rest:
            return False
        xs, ys = [h[1] for h in self.hist], [h[2] for h in self.hist]
        y0 = self.hist[-1][3]
        dyaw = [abs(math.atan2(math.sin(h[3] - y0), math.cos(h[3] - y0))) for h in self.hist]
        return max(max(xs) - min(xs), max(ys) - min(ys)) <= STILL_XY and max(dyaw) <= STILL_YAW

    def progress(self):
        if self.spin:
            return abs(self.turned)
        return math.hypot(self.pose[0] - self.start[0], self.pose[1] - self.start[1])

    def tick(self):
        now = time.monotonic()
        if self.state == 'wait_rest':
            if self.pose is None:
                self.get_logger().info('waiting for /odom', throttle_duration_sec=2.0)
                return
            if not self.at_rest():
                self.get_logger().info(f'waiting for the robot to be still {self.rest:.1f} s '
                                       '(hands off the remote)', throttle_duration_sec=2.0)
                return
            self.start, self.turned, self.t0, self.state = self.pose, 0.0, now, 'drive'
            what = (f'{self.target / (2 * math.pi):.1f} turns to the {"left" if self.sign > 0 else "right"} '
                    f'at {self.speed:.2f} rad/s' if self.spin else f'{self.target:.1f} m at {self.speed:.2f} m/s')
            self.get_logger().info(f'robot still, go: {what}')
        d = self.progress()
        dv = self.speed / (self.ramp * RATE)
        if self.state == 'drive':
            # start braking so that the ramp ends at the target (v^2 / 2a, a = speed / ramp)
            brake = self.v * self.v * self.ramp / (2 * self.speed)
            if d + brake >= self.target or now - self.t0 > self.limit:
                if now - self.t0 > self.limit:
                    self.get_logger().warn('time limit reached, stopping')
                self.state = 'brake'
            else:
                self.v = min(self.speed, self.v + dv)
        if self.state == 'brake':
            self.v = max(0.0, self.v - dv)
            if self.v == 0.0:
                self.state, self.t_stop = 'stopped', now
                self.get_logger().info(f'stopped, holding still {self.hold:.0f} s (do not touch it)')
        cmd = Twist()
        if self.spin:
            cmd.angular.z = self.sign * self.v
        else:
            cmd.linear.x = self.v
        self.pub.publish(cmd)
        if self.state == 'stopped' and now - self.t_stop > self.hold:
            done = (f'{math.degrees(self.turned):.0f} deg' if self.spin else f'{d:.2f} m')
            self.get_logger().info(f'done: {done} by odometry in {self.t_stop - self.t0:.1f} s. '
                                   'Now Ctrl+C in record.sh and measure with the tape.')
            raise SystemExit


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('distance', nargs='?', type=float, default=10.0, help='m (default 10)')
    ap.add_argument('--speed', type=float, default=0.3, help='m/s (default 0.3)')
    ap.add_argument('--spin', type=float, help='turns in place instead of a line, + left / - right')
    ap.add_argument('--wz', type=float, default=0.6, help='rad/s for --spin (default 0.6)')
    ap.add_argument('--ramp', type=float, default=1.0, help='s from 0 to speed and back (default 1)')
    ap.add_argument('--rest', type=float, default=2.0, help='s still before starting (default 2)')
    ap.add_argument('--hold', type=float, default=3.0, help='s of zero after stopping (default 3)')
    a = ap.parse_args()
    rclpy.init()
    node = Drive(a)
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
