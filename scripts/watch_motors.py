#!/usr/bin/env python3
# Live table of the 20 LowState motors, marking the ones that move.
# Used to check the motor index of each joint: move one wheel/leg by hand
# and see which row lights up. Reads /lowstate directly (not the driver).
#   GO2_IFACE=eth0 source scripts/env.sh && python3 scripts/watch_motors.py
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowState

# Index -> joint assumed in config/go2w_driver.yaml (16-19 unused)
ASSUMED = [
    'FR_hip', 'FR_thigh', 'FR_calf',
    'FL_hip', 'FL_thigh', 'FL_calf',
    'RR_hip', 'RR_thigh', 'RR_calf',
    'RL_hip', 'RL_thigh', 'RL_calf',
    'FR_foot (wheel)', 'FL_foot (wheel)', 'RR_foot (wheel)', 'RL_foot (wheel)',
    '-', '-', '-', '-',
]
DQ_MOVING = 0.5      # rad/s
DELTA_MOVED = 0.3    # rad from start


class WatchMotors(Node):

    def __init__(self):
        super().__init__('watch_motors')
        self.msg = None
        self.prev = None
        self.moved = [0.0] * 20
        self.count = 0
        self.create_subscription(LowState, 'lowstate', self.callback, qos_profile_sensor_data)
        self.create_timer(0.2, self.draw)

    def callback(self, msg):
        q = [m.q for m in msg.motor_state]
        # Accumulate wrapped differences: wheel angles may jump between -pi and pi
        if self.prev is not None:
            for i, (a, b) in enumerate(zip(q, self.prev)):
                self.moved[i] += math.remainder(a - b, 2 * math.pi)
        self.prev = q
        self.msg = msg
        self.count += 1

    def draw(self):
        out = ['\033[H\033[J']
        if self.msg is None:
            out.append('waiting for /lowstate ... (GO2_IFACE set? robot on?)')
            print(''.join(out), flush=True)
            return
        out.append(f'/lowstate msgs: {self.count}   (Ctrl+C to exit)\n\n')
        out.append(f"{'idx':>3}  {'assumed joint':<16} {'q':>8} {'dq':>8} {'moved':>8}\n")
        for i, m in enumerate(self.msg.motor_state):
            moved = self.moved[i]
            mark = ''
            if abs(m.dq) > DQ_MOVING:
                mark = '  <<< MOVING ' + ('+' if m.dq > 0 else '-')
            elif abs(moved) > DELTA_MOVED:
                mark = '  <- moved'
            line = f'{i:>3}  {ASSUMED[i]:<16} {m.q:8.3f} {m.dq:8.3f} {moved:8.2f}{mark}'
            if mark:
                line = f'\033[1;33m{line}\033[0m'
            out.append(line + '\n')
        out.append(f'\nmoved = rad since start (wheels: {2 * math.pi:.2f} = one turn)\n')
        print(''.join(out), end='', flush=True)


def main():
    rclpy.init()
    node = WatchMotors()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
