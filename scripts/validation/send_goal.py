#!/usr/bin/env python3
"""Sends an EasyNav goal relative to where the robot is now (repeatable nav tests).

  scripts/validation/send_goal.py 5            # 5 m straight ahead
  scripts/validation/send_goal.py 3 -2         # 3 m ahead, 2 m to the right
  scripts/validation/send_goal.py 0 0 180      # turn around in place
  scripts/validation/send_goal.py 5 --back     # 5 m ahead, then back to the start pose

Reads the current pose from /odom (map -> odom is ~identity in config/easynav_real.params.yaml),
publishes /goal_pose (frame map) and waits for FINISHED / FAILED on /easynav_control, printing
time and final error. Ctrl+C cancels the goal.
"""
import argparse
import math
import time

import rclpy
from easynav_interfaces.msg import NavigationControl
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

NAMES = {v: k for k, v in vars(NavigationControl).items() if k.isupper() and isinstance(v, int)}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GoalSender(Node):
    def __init__(self):
        super().__init__('validation_send_goal')
        self.pose = None
        self.result = None
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(NavigationControl, '/easynav_control', self.control_cb, 100)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.control_pub = self.create_publisher(NavigationControl, '/easynav_control', 100)

    def odom_cb(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def control_cb(self, m):
        if m.type in (NavigationControl.FINISHED, NavigationControl.FAILED,
                      NavigationControl.REJECT, NavigationControl.ERROR,
                      NavigationControl.CANCELLED):
            self.result = (NAMES.get(m.type, m.type), m.status_message)

    def wait_pose(self):
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose

    def go(self, x, y, yaw, timeout):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.result = None
        # the publisher needs a moment to be matched by system_main
        for _ in range(20):
            if self.goal_pub.get_subscription_count():
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.goal_pub.publish(g)
        print(f'goal map ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
        t0 = time.monotonic()
        try:
            while rclpy.ok() and self.result is None and time.monotonic() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            c = NavigationControl()
            c.type = NavigationControl.CANCEL
            c.user_id = 'validation_send_goal'
            self.control_pub.publish(c)
            print('cancelled')
            raise
        dt = time.monotonic() - t0
        px, py, pyaw = self.pose
        err = math.hypot(px - x, py - y)
        eyaw = math.degrees((pyaw - yaw + math.pi) % (2 * math.pi) - math.pi)
        res = self.result or ('TIMEOUT', '')
        print(f'  {res[0]} {res[1]}  time {dt:.1f} s  final error {err:.2f} m, yaw {eyaw:+.1f} deg')
        return res[0] == 'FINISHED'


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split('\n\n', 1)[1])
    p.add_argument('forward', type=float, help='m ahead of the robot')
    p.add_argument('left', type=float, nargs='?', default=0.0, help='m to the left (negative: right)')
    p.add_argument('turn', type=float, nargs='?', default=None,
                   help='final yaw relative to now, deg (default: facing along the path)')
    p.add_argument('--back', action='store_true', help='then return to the start pose')
    p.add_argument('--timeout', type=float, default=120.0)
    a = p.parse_args()

    rclpy.init()
    n = GoalSender()
    x0, y0, th0 = n.wait_pose()
    gx = x0 + a.forward * math.cos(th0) - a.left * math.sin(th0)
    gy = y0 + a.forward * math.sin(th0) + a.left * math.cos(th0)
    if a.turn is not None:
        gyaw = th0 + math.radians(a.turn)
    elif a.forward or a.left:
        gyaw = th0 + math.atan2(a.left, a.forward)
    else:
        gyaw = th0
    try:
        ok = n.go(gx, gy, gyaw, a.timeout)
        if ok and a.back:
            n.go(x0, y0, th0, a.timeout)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
