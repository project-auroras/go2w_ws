"""KISS-ICP (LiDAR only, no IMU) on the raw /utlidar/cloud of line bags vs FAST-LIO and the tape.
Independent check of the L2 scale error. Outside ROS, in a venv:
  python3 -m venv v && v/bin/pip install kiss-icp rosbags scipy pyyaml
  v/bin/python scripts/validation/offline/kiss_line.py --accumulate 3 bags/*_line
One /utlidar/cloud is only a slice of the L1 sweep: KISS-ICP diverges on single clouds,
--accumulate 3-5 joins them (with per-point times for the deskew).
Distances are start -> end of the LiDAR origin, rest windows like validate (1.5 s before the first
non-zero /cmd_vel, 1.0-2.5 s after the last)."""
import argparse, math
from pathlib import Path

import numpy as np
import yaml
from kiss_icp.config import load_config
from kiss_icp.kiss_icp import KissICP
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

DT = {7: np.float32, 4: np.uint16, 2: np.uint8, 6: np.uint32, 8: np.float64}


def cloud_xyz_t(m):
    dt = np.dtype({'names': [f.name for f in m.fields], 'formats': [DT[f.datatype] for f in m.fields],
                   'offsets': [f.offset for f in m.fields], 'itemsize': m.point_step})
    a = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height)
    xyz = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
    ok = np.isfinite(xyz).all(1)
    return xyz[ok], a['time'][ok].astype(np.float64)


def _r(ax, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) if ax == 'y' else np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


R_BL = _r('y', 2.8782) @ _r('z', -2.0682)   # base <- utlidar_lidar (go2w_description)
T_BL = np.array([0.28945, 0.0, -0.046825])


def tape_dist(gt):
    if gt.get('forward_m') and gt.get('lateral_m') is not None:
        return math.hypot(gt['forward_m'], gt['lateral_m'])
    xa, a, xb, b = (gt.get(k) for k in ('mark_a_m', 'dist_a_m', 'mark_b_m', 'dist_b_m'))
    x = (a * a - b * b + xb * xb - xa * xa) / (2 * (xb - xa))
    return math.hypot(x, math.sqrt(max(a * a - (x - xa) ** 2, 0)))


def rest_mean(t, p, t0, t1):
    m = (t >= t0) & (t <= t1)
    return p[m].mean(0) if m.sum() >= 3 else None


def run(bag, args):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    cfg = load_config(None)
    cfg.data.max_range, cfg.data.min_range, cfg.data.deskew = args.max_range, args.min_range, not args.no_deskew
    cfg.mapping.voxel_size = args.voxel
    icp = KissICP(cfg)
    kt, kp, ft, fp, cmd, buf = [], [], [], [], [], []
    with AnyReader([Path(bag)], default_typestore=ts) as r:
        conns = [c for c in r.connections if c.topic in ('/utlidar/cloud', '/Odometry', '/cmd_vel')]
        for c, t, raw in r.messages(connections=conns):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/cmd_vel':
                cmd.append((t * 1e-9, m.linear.x))
                continue
            st = t * 1e-9   # bag receive time, same clock as /cmd_vel (the LiDAR stamps may not be)
            if c.topic == '/Odometry':
                ft.append(st); fp.append([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
                continue
            xyz, tt = cloud_xyz_t(m)
            if args.floor_z is not None:
                keep = (xyz @ R_BL.T + T_BL)[:, 2] > args.floor_z
                xyz, tt = xyz[keep], tt[keep]
            hs = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            buf.append((xyz, hs + tt))      # absolute point times (LiDAR clock)
            if len(buf) < args.accumulate:
                continue
            xyz = np.vstack([b[0] for b in buf]); tt = np.concatenate([b[1] for b in buf]); buf = []
            span = tt.max() - tt.min()
            icp.register_frame(xyz, (tt - tt.min()) / span if span > 0 else np.zeros(len(tt)))
            kt.append(st); kp.append(icp.last_pose[:3, 3].copy())
    kt, kp, ft, fp = map(np.array, (kt, kp, ft, fp))
    mv = [t for t, v in cmd if abs(v) > 1e-3]
    t_go, t_stop = mv[0], mv[-1]
    res = {}
    for name, t, p in (('kiss', kt, kp), ('fastlio', ft, fp)):
        a, b = rest_mean(t, p, t_go - 1.5, t_go - 0.05), rest_mean(t, p, t_stop + 1.0, t_stop + 2.5)
        res[name] = float(np.linalg.norm(b - a)) if a is not None and b is not None else float('nan')
        res[name + '_z'] = float((b - a)[2]) if a is not None and b is not None else float('nan')
    gt = yaml.safe_load((Path(bag) / 'meta.yaml').read_text())['ground_truth']
    tape = tape_dist(gt)
    pct = lambda d: 100 * (d - tape) / tape
    print(f'{Path(bag).name}: tape {tape:.3f} | FAST-LIO {res["fastlio"]:.3f} ({pct(res["fastlio"]):+.2f} %) '
          f'| KISS {res["kiss"]:.3f} ({pct(res["kiss"]):+.2f} %) | dz LIO {res["fastlio_z"]:+.2f} KISS {res["kiss_z"]:+.2f} m '
          f'| {len(kt)} clouds, points t-span {span*1000:.0f} ms', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--voxel', type=float, default=0.25)
    ap.add_argument('--max-range', type=float, default=20.0)
    ap.add_argument('--min-range', type=float, default=0.4)
    ap.add_argument('--no-deskew', action='store_true')
    ap.add_argument('--accumulate', type=int, default=1, help='clouds per KISS frame')
    ap.add_argument('--floor-z', type=float, help='drop points below this z in base (floor ~ -0.54)')
    a = ap.parse_args()
    for b in a.bags:
        run(b, a)
