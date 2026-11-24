#!/usr/bin/env python3
"""LiDAR range scale from wall bags (scripts/validation/wall_record.sh). PC container:

  python3 scripts/validation/wall_range.py bags/*_wall

Per bag: the wall plane in front of the robot (RANSAC on all clouds of the bag, the robot is
still) and its distance from the LiDAR origin, next to the tape distance in meta.yaml. With two or
more bags: cloud = k * tape + c; k - 1 is the range scale error (c is the unknown offset between
the optical centre and the floor point that was taped, it cancels out).
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# base <- utlidar_lidar, from go2w_description (radar_joint, utlidar_lidar_joint). Only used to
# pick the points in front of the robot; the distance is measured in the LiDAR frame.
R_BL = rot_y(2.8782) @ rot_z(-2.0682)
T_BL = np.array([0.28945, 0.0, -0.046825])


def clouds(bag, limit=300):
    """xyz of the first `limit` clouds (~20 s; the robot is still, more only costs memory)."""
    r = SequentialReader()
    r.open(StorageOptions(uri=str(bag), storage_id='sqlite3'), ConverterOptions('', ''))
    n = 0
    while r.has_next() and n < limit:
        topic, data, _ = r.read_next()
        if topic == '/utlidar/cloud':
            n += 1
            m = deserialize_message(data, PointCloud2)
            p = point_cloud2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
            yield np.stack([p['x'], p['y'], p['z']], axis=1).astype(float)


def wall_distance(bag, rng=np.random.default_rng(0)):
    pts = np.vstack(list(clouds(bag)))
    pb = pts @ R_BL.T + T_BL
    # in front of the head, a body-wide strip, above the floor
    sel = (pb[:, 0] > 0.6) & (np.abs(pb[:, 1]) < 0.8) & (pb[:, 2] > -0.2) & (pb[:, 2] < 1.5)
    p = pts[sel]
    if len(p) < 50:
        raise RuntimeError(f'only {len(p)} points in front of the robot')
    best, tol = None, 0.02
    for _ in range(400):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        if abs((R_BL @ n)[0]) < 0.9:      # wall: normal along the robot's x
            continue
        inl = np.abs((p - a) @ n) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None:
        raise RuntimeError('no wall plane found')
    w = p[best]
    centre = w.mean(axis=0)
    n = np.linalg.svd(w - centre)[2][-1]  # least squares plane on the inliers
    res = (w - centre) @ n
    return abs(centre @ n), len(w), res.std(), math.degrees(math.acos(min(1, abs((R_BL @ n)[0]))))


def main():
    rows = []
    for b in sys.argv[1:]:
        b = Path(b)
        tape = (yaml.safe_load((b / 'meta.yaml').read_text()).get('ground_truth') or {}).get('wall_tape_m')
        d, n, rms, ang = wall_distance(b)
        print(f'{b.name}: tape {tape} m, cloud {d:.4f} m, diff {d - tape:+.4f} m '
              f'({n} wall points, plane rms {rms * 1000:.1f} mm, wall {ang:.1f} deg off the robot axis)')
        rows.append((tape, d))
    if len(rows) >= 2:
        t, d = np.array(rows).T
        k, c = np.polyfit(t, d, 1)
        print(f'\ncloud = {k:.4f} * tape {c:+.4f} m  ->  range scale error {100 * (k - 1):+.2f} % '
              '(- = LiDAR short; FAST-LIO in the line test: about -3 %)')
        if abs(t.max() - t.min()) < 2:
            print('!! distances less than 2 m apart: the scale is not well determined')


if __name__ == '__main__':
    main()
