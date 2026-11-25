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
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
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


def power(bag):
    """Battery and temperatures from /lowstate (recorded since 240113f), '' if not in the bag."""
    try:
        from unitree_go.msg import LowState
    except ImportError:
        return ''
    r = SequentialReader()
    r.open(StorageOptions(uri=str(bag), storage_id='sqlite3'), ConverterOptions('', ''))
    r.set_filter(StorageFilter(topics=['/lowstate']))
    v, a, soc, bms, imu = [], [], [], [], []
    while r.has_next():
        m = deserialize_message(r.read_next()[1], LowState)
        v.append(m.power_v)
        a.append(m.power_a)
        soc.append(m.bms_state.soc)
        bms.append(max(m.bms_state.bq_ntc))
        imu.append(m.imu_state.temperature)
    if not v:
        return ''
    return (f'battery {np.mean(v):.2f} V {np.mean(a):.1f} A soc {np.median(soc):.0f} %, '
            f'bms {np.median(bms):.0f} C, imu {np.median(imu):.0f} C | ')


def fit_plane(p, axis, rng, tol=0.02):
    """RANSAC + least squares plane among points p (LiDAR frame) whose normal, in the robot frame,
    is along `axis` (0 = x: wall in front, 2 = z: floor). Distance from the LiDAR origin, inliers,
    rms, angle of the normal to the axis."""
    best = None
    for _ in range(400):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        if abs((R_BL @ n)[axis]) < 0.9:
            continue
        inl = np.abs((p - a) @ n) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None:
        return None
    w = p[best]
    centre = w.mean(axis=0)
    n = np.linalg.svd(w - centre, full_matrices=False)[2][-1]
    return abs(centre @ n), len(w), ((w - centre) @ n).std(), math.degrees(math.acos(min(1, abs((R_BL @ n)[axis]))))


def planes(bag):
    rng = np.random.default_rng(0)
    pts = np.vstack(list(clouds(bag)))
    pb = pts @ R_BL.T + T_BL
    # wall: in front of the head, a body-wide strip, above the floor
    wall = pts[(pb[:, 0] > 0.6) & (np.abs(pb[:, 1]) < 0.8) & (pb[:, 2] > -0.2) & (pb[:, 2] < 1.5)]
    if len(wall) < 50:
        raise RuntimeError(f'only {len(wall)} points in front of the robot')
    w = fit_plane(wall, 0, rng)
    if w is None:
        raise RuntimeError('no wall plane found')
    # floor: around the robot, below the body, not under the wall
    floor = pts[(pb[:, 2] < -0.35) & (np.hypot(pb[:, 0], pb[:, 1]) < 1.5)]
    f = fit_plane(floor, 2, rng) if len(floor) > 50 else None
    return w, f


def wall_distance(bag):
    return planes(bag)[0]


def main():
    rows = []
    for b in sys.argv[1:]:
        b = Path(b)
        meta = yaml.safe_load((b / 'meta.yaml').read_text())
        tape = (meta.get('ground_truth') or {}).get('wall_tape_m')
        (d, n, rms, ang), f = planes(b)
        fl = f[0] if f else float('nan')
        when = f"minute {meta['warmup_min']:>3}, " if 'warmup_min' in meta else ''
        up = f"NX up {float(meta['uptime_s']) / 60:.0f} min, " if 'uptime_s' in meta else ''
        print(f'{b.name}: {when}{up}{power(b)}tape {tape} m, wall {d:.4f} m (diff {d - tape:+.4f}), floor {fl:.4f} m '
              f'| {n} wall points, rms {rms * 1000:.1f} mm, wall {ang:.1f} deg off axis')
        rows.append((tape, d, fl))
    if len(rows) < 2:
        return
    t, d, fl = np.array(rows).T
    if np.ptp(t) < 0.05:
        # warm-up series at one distance: change of the wall and the floor since the first record.
        # Same change in mm -> offset; change in proportion to the range (wall/floor) -> scale.
        print(f'\nsince the first record: wall {1000 * (d[-1] - d[0]):+.1f} mm ({100 * (d[-1] / d[0] - 1):+.2f} %), '
              f'floor {1000 * (fl[-1] - fl[0]):+.1f} mm ({100 * (fl[-1] / fl[0] - 1):+.2f} %)')
        print('   same mm -> range offset; same % -> range scale')
        return
    k, c = np.polyfit(t, d, 1)
    print(f'\ncloud = {k:.4f} * tape {c:+.4f} m  ->  range scale error {100 * (k - 1):+.2f} % '
          '(- = LiDAR short; FAST-LIO in the line test: about -3 %)')
    if np.ptp(t) < 2:
        print('!! distances less than 2 m apart: the scale is not well determined')


if __name__ == '__main__':
    main()
