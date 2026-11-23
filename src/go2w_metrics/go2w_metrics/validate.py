"""Validation report of one test bag recorded with scripts/validation/record.sh.

  ros2 run go2w_metrics validate bags/2026-11-19_10-00-00_line          # -> <bag>/report/
  ros2 run go2w_metrics validate bags/*_static --test static             # without meta.yaml

Reads meta.yaml (test type + tape ground truth), computes the metrics of the test, checks them
against the acceptance criteria of docs/validation/README.md (PASS / FAIL / INFO) and writes
report.md, metrics.yaml and plots. Hardware comes from /vitals/status and /jetson/stats
(go2w_metrics, converted with bag_to_csv into <bag>/csv/).

Tests: static (drift at rest), line (straight line vs tape), spin (turns in place),
loop (closed loop back to a floor mark), nav (EasyNav goals), free (only topics + hardware).
Other tests live in their own module go2w_metrics/validate_<test>.py with TOPICS (extra topics
to read), CRITERIA and section(rep, d, o, meta); e.g. detect / follow (person follower).
"""

import argparse
import csv
import importlib
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

from go2w_metrics.bag_to_csv import convert

# Acceptance criteria (docs/validation/README.md). Keep both in sync.
CRITERIA = {
    'cloud_hz': ('>=', 14.0),
    'imu_hz': ('>=', 230.0),
    'odom_hz': ('>=', 14.0),
    'odom_gap_max_ms': ('<=', 200.0),
    'lio_latency_p95_ms': ('<=', 100.0),
    'static_drift_xy_m_per_5min': ('<=', 0.05),
    'static_drift_yaw_deg_per_5min': ('<=', 1.0),
    'line_scale_error_pct': ('<=', 2.0),
    'spin_yaw_error_deg_per_turn': ('<=', 1.0),
    'spin_translation_m': ('<=', 0.10),
    'loop_drift_upper_pct': ('<=', 0.5),
    'nav_success_rate': ('>=', 1.0),
    'nav_final_error_m_max': ('<=', 0.35),
    'nav_final_yaw_deg_max': ('<=', 17.0),
    'nav_min_clearance_m': ('>=', 0.25),
    'hw_lio_stack_cpu_mean_pct': ('<=', 150.0),
    'hw_nx_cpu_mean_pct': ('<=', 50.0),
    'hw_nx_temp_max_c': ('<=', 80.0),
}
# process names of the localization stack (ros2_vitals node names; go2w_driver runs in go2w_container)
LIO_STACK = ('laser_mapping', 'fastlio_mapping', 'lio_scan', 'go2w_driver', 'go2w_container')

BUILTIN = ('static', 'line', 'spin', 'loop', 'nav', 'free')
TOPICS = {'/utlidar/cloud', '/utlidar/imu', '/lidar_imu', '/Odometry', '/odom', '/scan',
          '/cmd_vel', '/goal_pose', '/easynav_control'}


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def read_bag(bag, topics=TOPICS):
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=storage), ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    d = defaultdict(list)   # topic -> [(t_bag, t_stamp, msg)]
    while r.has_next():
        topic, data, t = r.read_next()
        if topic not in topics or topic not in types:
            continue
        m = deserialize_message(data, get_message(types[topic]))
        h = getattr(m, 'header', None)
        d[topic].append((t * 1e-9, stamp(h) if h is not None else t * 1e-9, m))
    return d


def odom_array(msgs):
    """[(t_stamp, x, y, z, yaw)] of nav_msgs/Odometry."""
    return np.array([(s, m.pose.pose.position.x, m.pose.pose.position.y,
                      m.pose.pose.position.z, yaw_of(m.pose.pose.orientation))
                     for _, s, m in msgs]).reshape(-1, 5)


class Report:
    def __init__(self):
        self.metrics = {}
        self.lines = []
        self.results = []

    def add(self, key, value, unit='', note=''):
        value = None if value is None or (isinstance(value, float) and not math.isfinite(value)) \
            else round(float(value), 4)
        self.metrics[key] = value
        verdict = 'INFO'
        if key in CRITERIA and value is not None:
            op, lim = CRITERIA[key]
            verdict = 'PASS' if (value >= lim if op == '>=' else value <= lim) else 'FAIL'
            self.results.append(verdict)
        elif key in CRITERIA:
            self.results.append('MISSING')
        crit = f'{CRITERIA[key][0]} {CRITERIA[key][1]:g}' if key in CRITERIA else ''
        shown = '—' if value is None else f'{value:g}'
        self.lines.append(f'| {key} | {shown} {unit} | {crit} | {verdict} | {note} |')

    def section(self, title):
        self.lines += ['', f'### {title}', '', '| metric | value | criterion | result | note |',
                       '|---|---|---|---|---|']


def topics_section(rep, d):
    rep.section('Inputs and odometry')
    for topic, key in (('/utlidar/cloud', 'cloud_hz'), ('/utlidar/imu', 'imu_hz'),
                       ('/lidar_imu', None), ('/Odometry', None), ('/odom', 'odom_hz'),
                       ('/scan', None)):
        msgs = d.get(topic, [])
        if len(msgs) < 3:
            if key:
                rep.add(key, None, 'Hz', f'{topic} not in the bag')
            continue
        tb = np.array([x[0] for x in msgs])
        hz = (len(tb) - 1) / (tb[-1] - tb[0])
        gap = np.diff(tb).max() * 1e3
        rep.add(key or f'{topic.strip("/").replace("/", "_")}_hz', hz, 'Hz', f'gap max {gap:.0f} ms')
        if topic == '/odom':
            rep.add('odom_gap_max_ms', gap, 'ms')
    # processing latency on the NX: arrival of /Odometry - arrival of the cloud it comes from
    clouds, odo = d.get('/utlidar/cloud', []), d.get('/Odometry', [])
    if clouds and odo:
        cs = np.array([x[1] for x in clouds])
        ca = np.array([x[0] for x in clouds])
        lat = []
        for ta, ts, _ in odo:
            i = np.searchsorted(cs, ts, side='right') - 1
            if i >= 0 and ts - cs[i] < 0.15:
                lat.append((ta - ca[i]) * 1e3)
        if lat:
            rep.add('lio_latency_p50_ms', np.percentile(lat, 50), 'ms', 'cloud arrival -> /Odometry')
            rep.add('lio_latency_p95_ms', np.percentile(lat, 95), 'ms')


def static_section(rep, o, O):
    rep.section('Static drift')
    # drift = slope of a line fitted to x, y, yaw over time (the last sample alone measures the
    # jitter of the standing robot, ±1-2 deg, not drift); jitter = spread around that line
    t = o[:, 0] - o[0, 0]
    dur = t[-1]
    yaw = np.degrees(np.unwrap(o[:, 4] - o[0, 4]))
    fx, fy, fyaw = (np.polyfit(t, v, 1) for v in (o[:, 1], o[:, 2], yaw))
    dxy = np.linalg.norm(o[:, 1:3] - o[0, 1:3], axis=1)
    rep.add('static_duration_s', dur, 's')
    rep.add('static_drift_xy_max_m', dxy.max(), 'm', f'end {dxy[-1]:.3f} m (jitter + drift)')
    rep.add('static_drift_xy_m_per_5min', 300 * math.hypot(fx[0], fy[0]), 'm', 'slope of the fit')
    rep.add('static_drift_yaw_deg_per_5min', 300 * abs(fyaw[0]), 'deg', 'slope of the fit')
    res_xy = np.hypot(o[:, 1] - np.polyval(fx, t), o[:, 2] - np.polyval(fy, t))
    res_yaw = np.abs(yaw - np.polyval(fyaw, t))
    rep.add('static_jitter_xy_p95_m', np.percentile(res_xy, 95), 'm', 'around the fit')
    rep.add('static_jitter_yaw_p95_deg', np.percentile(res_yaw, 95), 'deg', f'max {res_yaw.max():.2f} deg')
    if len(O):
        rep.add('static_drift_z_lio_m', np.abs(O[:, 3] - O[0, 3]).max(), 'm',
                '/Odometry (L1 frame), go2w_driver drops z')


def line_section(rep, o, gt):
    rep.section('Straight line')
    d = np.linalg.norm(o[-1, 1:3] - o[0, 1:3])
    rep.add('line_distance_lio_m', d, 'm')
    u = (o[-1, 1:3] - o[0, 1:3]) / max(d, 1e-6)
    rel = o[:, 1:3] - o[0, 1:3]
    lateral = np.abs(rel[:, 0] * u[1] - rel[:, 1] * u[0])
    rep.add('line_lateral_max_m', lateral.max(), 'm', 'deviation from the start-end line')
    rep.add('line_heading_change_deg', math.degrees(wrap(o[-1, 4] - o[0, 4])), 'deg')
    tape = gt.get('distance_m')
    if tape:
        rep.add('line_distance_tape_m', tape, 'm')
        rep.add('line_scale_error_pct', 100 * abs(d - tape) / tape, '%')
    else:
        rep.add('line_scale_error_pct', None, '%', 'no tape distance in meta.yaml')
    # end point in the robot's start frame (x forward, y left): the wheels drift sideways in open
    # loop, that is the robot, not odometry. With the tape lateral offset it is checked in 2D.
    c, s = math.cos(o[0, 4]), math.sin(o[0, 4])
    fwd, left = rel[-1, 0] * c + rel[-1, 1] * s, -rel[-1, 0] * s + rel[-1, 1] * c
    rep.add('line_drift_lio_deg', math.degrees(math.atan2(left, fwd)), 'deg',
            f'robot went {left:+.2f} m sideways in {fwd:.2f} m (wheels, + left)')
    lat = gt.get('lateral_m')
    if tape and lat is not None and abs(lat) < tape:
        fwd_t = math.sqrt(tape ** 2 - lat ** 2)
        rep.add('line_drift_tape_deg', math.degrees(math.atan2(lat, fwd_t)), 'deg',
                'real drift, relative to the floor line')
        rep.add('line_end_error_2d_m', math.hypot(fwd - fwd_t, left - lat), 'm',
                'includes how well the robot was aligned with the line at the start')


def spin_section(rep, o, gt):
    rep.section('Turns in place')
    yaw = np.unwrap(o[:, 4])
    total = math.degrees(yaw[-1] - yaw[0])
    rep.add('spin_total_deg', total, 'deg')
    rep.add('spin_translation_m', np.linalg.norm(o[:, 1:3] - o[0, 1:3], axis=1).max(), 'm',
            'max distance of base_link from the start')
    turns = gt.get('turns')
    if turns:
        err = abs(abs(total) - 360 * turns)
        rep.add('spin_yaw_error_deg', err, 'deg', f'expected {360 * turns:.0f}')
        rep.add('spin_yaw_error_deg_per_turn', err / turns, 'deg')
    else:
        rep.add('spin_yaw_error_deg_per_turn', None, 'deg', 'no turn count in meta.yaml')


def loop_section(rep, o, gt):
    rep.section('Closed loop')
    path = np.linalg.norm(np.diff(o[:, 1:3], axis=0), axis=1).sum()
    lio = np.linalg.norm(o[-1, 1:3] - o[0, 1:3])
    rep.add('loop_path_m', path, 'm')
    rep.add('loop_return_lio_m', lio, 'm', 'distance to start according to LIO')
    rep.add('loop_yaw_end_deg', math.degrees(wrap(o[-1, 4] - o[0, 4])), 'deg')
    tape = gt.get('return_error_m')
    if tape is not None:
        # only the magnitude of the true offset is known: drift is between |lio - tape| and lio + tape
        rep.add('loop_return_tape_m', tape, 'm')
        rep.add('loop_drift_lower_m', abs(lio - tape), 'm')
        rep.add('loop_drift_upper_pct', 100 * (lio + tape) / max(path, 1e-6), '%',
                f'upper bound {lio + tape:.3f} m')
    else:
        rep.add('loop_drift_upper_pct', None, '%', 'no tape return distance in meta.yaml')


def nav_section(rep, d, o):
    rep.section('EasyNav goals')
    goals = d.get('/goal_pose', [])
    ends = [(tb, m) for tb, _, m in d.get('/easynav_control', []) if m.type in (4, 5, 1, 8, 7)]
    names = {4: 'FINISHED', 5: 'FAILED', 1: 'REJECT', 8: 'ERROR', 7: 'CANCELLED'}
    scans = d.get('/scan', [])
    cmds = d.get('/cmd_vel', [])
    ob = np.array([x[0] for x in d.get('/odom', [])])   # arrival times, aligned with o
    rows, ok_n, final_err, final_yaw, clear = [], 0, [], [], []
    for i, (tg, _, g) in enumerate(goals):
        t_next = goals[i + 1][0] if i + 1 < len(goals) else float('inf')
        end = next(((tb, m) for tb, m in ends if tg < tb < t_next), None)
        te = end[0] if end else min(t_next, ob[-1])
        res = names[end[1].type] if end else 'NO_RESULT'
        sel = (ob >= tg) & (ob <= te)
        if sel.sum() < 2:
            continue
        seg = o[sel]
        gx, gy, gyaw = g.pose.position.x, g.pose.position.y, yaw_of(g.pose.orientation)
        err = math.hypot(seg[-1, 1] - gx, seg[-1, 2] - gy)
        eyaw = abs(math.degrees(wrap(seg[-1, 4] - gyaw)))
        straight = math.hypot(gx - seg[0, 1], gy - seg[0, 2])
        path = np.linalg.norm(np.diff(seg[:, 1:3], axis=0), axis=1).sum()
        mins = [np.nanmin(np.where(np.isfinite(r := np.asarray(m.ranges)) & (r >= m.range_min), r, np.nan))
                for tb, _, m in scans if tg <= tb <= te and np.isfinite(m.ranges).any()]
        mn = float(np.nanmin(mins)) if mins else float('nan')
        wz = np.array([m.angular.z for tb, _, m in cmds if tg <= tb <= te])
        flips = int(np.sum(np.diff(np.sign(wz[np.abs(wz) > 0.05])) != 0)) if len(wz) else 0
        ok_n += res == 'FINISHED'
        final_err.append(err)
        final_yaw.append(eyaw)
        clear.append(mn)
        rows.append(f'| {i + 1} | {res} | {straight:.2f} | {path:.2f} | {path / max(straight, 1e-6):.2f} | '
                    f'{te - tg:.1f} | {err:.2f} | {eyaw:.1f} | {mn:.2f} | {flips} |')
    if not rows:
        rep.add('nav_success_rate', None, '', 'no /goal_pose in the bag')
        return
    rep.add('nav_goals', len(rows))
    rep.add('nav_success_rate', ok_n / len(rows), '', f'{ok_n}/{len(rows)} FINISHED')
    rep.add('nav_final_error_m_max', max(final_err), 'm', 'in odom (localizer frame), not tape')
    rep.add('nav_final_yaw_deg_max', max(final_yaw), 'deg')
    rep.add('nav_min_clearance_m', np.nanmin(clear), 'm', 'min /scan range while navigating')
    rep.lines += ['', '| goal | result | straight m | driven m | ratio | time s | final err m | '
                  'yaw err deg | min clearance m | w sign flips |', '|---|---|---|---|---|---|---|---|---|---|'] + rows


def hardware_section(rep, bag, out_dir):
    csv_dir = os.path.join(out_dir, 'csv')
    convert(bag, csv_dir, 0.0, '/external')
    rep.section('Hardware')

    def load(name):
        p = os.path.join(csv_dir, name)
        return list(csv.DictReader(open(p))) if os.path.isfile(p) else []

    jet, procs, system = load('jetson.csv'), load('processes.csv'), load('system.csv')
    by_host = defaultdict(list)
    for r in system:
        by_host[r['host']].append(r)
    for host, rs in sorted(by_host.items()):
        def col(k):
            return np.array([float(r[k]) for r in rs if r.get(k) not in (None, '')])
        h = re.sub(r'\W', '_', host).lower()
        c = col('cpu_percent')
        if len(c):
            rep.add(f'hw_{h}_cpu_mean_pct', c.mean(), '%', f'{host}: max {c.max():.0f} %, {rs[0].get("cpu_count")} cores (ros2_vitals)')
        if len(g := col('gpu0_percent')):
            rep.add(f'hw_{h}_gpu_mean_pct', g.mean(), '%', f'max {g.max():.0f} %, VRAM max {col("gpu0_mem_used_mb").max():.0f} MB')
    hw = {}
    if jet:
        def col(k):
            return np.array([float(r[k]) for r in jet if r.get(k) not in (None, '')])
        cpu = col('cpu_percent')
        rep.add('hw_nx_cpu_mean_pct', cpu.mean(), '%', f'max {cpu.max():.0f} % (tegrastats, all cores)')
        # the governor lowers the clock with low load: the same work then shows as more CPU %
        mhz = np.concatenate([col(k) for k in jet[0] if re.fullmatch(r'cpu\d+_mhz', k)] or [np.array([])])
        if len(mhz):
            rep.add('hw_nx_cpu_mhz_mean', mhz.mean(), 'MHz', 'compare CPU % only at similar clocks')
        ram = col('ram_used_mb')
        rep.add('hw_nx_ram_used_max_mb', ram.max(), 'MB', f'of {col("ram_total_mb").max():.0f} MB')
        if len(g := col('gpu_percent')):
            rep.add('hw_nx_gpu_mean_pct', g.mean(), '%')
        temps = {k: col(k).max() for k in jet[0] if k.startswith('temp_')
                 and len(col(k)) and col(k).max() > -40}
        if temps:
            hot = max(temps, key=temps.get)
            rep.add('hw_nx_temp_max_c', temps[hot], 'C', hot[5:-2])
        for k in jet[0]:
            if k.startswith('power_') and 'IN' in k:
                rep.add(f'hw_{k[:-3].lower()}_mean_w', col(k).mean() / 1000, 'W', f'max {col(k).max() / 1000:.1f} W')
        hw['jetson'] = jet
    else:
        rep.add('hw_nx_cpu_mean_pct', None, '%', '/jetson/stats not in the bag')
    if procs:
        nx = {r['host'] for r in procs}
        by = defaultdict(list)
        for r in procs:
            if r['launch'] in ('True', 'true') or not r['name']:
                continue
            by[(r['host'], r['name'])].append((float(r['cpu_percent']), float(r['ram_mb'])))
        table = sorted(((h, n, np.mean([c for c, _ in v]), np.percentile([c for c, _ in v], 95),
                         max(m for _, m in v)) for (h, n), v in by.items()), key=lambda x: -x[2])
        stack = sum(c for h, n, c, _, _ in table if any(s in n for s in LIO_STACK))
        rep.add('hw_lio_stack_cpu_mean_pct', stack, '%', '+'.join(LIO_STACK) + ' (100 = one core)')
        rep.lines += ['', f'Per process (ros2_vitals, hosts: {", ".join(sorted(nx))}; 100 % = one core):', '',
                      '| host | node | CPU mean % | CPU p95 % | RAM max MB |', '|---|---|---|---|---|']
        rep.lines += [f'| {h} | {n} | {c:.0f} | {p:.0f} | {m:.0f} |' for h, n, c, p, m in table[:15]]
    else:
        rep.lines.append('\n/vitals/status not in the bag: no per-process data.')
    return hw


def plots(out_dir, o, hw, test):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    files = []
    if len(o):
        t = o[:, 0] - o[0, 0]
        f, a = plt.subplots(1, 2, figsize=(12, 4.5))
        a[0].plot(o[:, 1], o[:, 2])
        a[0].plot(o[0, 1], o[0, 2], 'go', label='start')
        a[0].plot(o[-1, 1], o[-1, 2], 'rx', label='end')
        a[0].axis('equal'); a[0].grid(); a[0].legend(); a[0].set_title(f'/odom xy [m] ({test})')
        a[1].plot(t, np.degrees(np.unwrap(o[:, 4])))
        a[1].grid(); a[1].set_title('/odom yaw [deg] vs t [s]')
        plt.tight_layout(); plt.savefig(os.path.join(out_dir, 'trajectory.png'), dpi=80); plt.close()
        files.append('trajectory.png')
    jet = hw.get('jetson')
    if jet:
        tb = np.array([float(r['t_bag']) for r in jet]); tb -= tb[0]
        f, a = plt.subplots(1, 3, figsize=(16, 4))
        a[0].plot(tb, [float(r['cpu_percent']) for r in jet]); a[0].set_title('NX CPU % (all cores)')
        for k in jet[0]:
            if k.startswith('temp_'):
                v = [float(r[k]) for r in jet if r.get(k)]
                if v and max(v) > -40:
                    a[1].plot(tb[:len(v)], v, label=k[5:-2])
        a[1].legend(fontsize=7); a[1].set_title('NX temperatures C')
        for k in jet[0]:
            if k.startswith('power_') and k.endswith('_mw') and '_avg' not in k:
                a[2].plot(tb, [float(r[k]) / 1000 if r.get(k) else np.nan for r in jet], label=k[6:-3])
        a[2].legend(fontsize=7); a[2].set_title('NX power W')
        for x in a:
            x.grid(); x.set_xlabel('t [s]')
        plt.tight_layout(); plt.savefig(os.path.join(out_dir, 'hardware.png'), dpi=80); plt.close()
        files.append('hardware.png')
    return files


def run(bag, test=None):
    meta = {}
    if os.path.isfile(p := os.path.join(bag, 'meta.yaml')):
        meta = yaml.safe_load(open(p)) or {}
    test = test or meta.get('test', 'free')
    gt = meta.get('ground_truth') or {}
    out_dir = os.path.join(bag, 'report')
    os.makedirs(out_dir, exist_ok=True)
    print(f'{bag}: test {test}')
    mod = None
    if test not in BUILTIN:
        mod = importlib.import_module(f'go2w_metrics.validate_{test}')
        CRITERIA.update(getattr(mod, 'CRITERIA', {}))
    d = read_bag(bag, TOPICS | set(getattr(mod, 'TOPICS', ())))
    o, O = odom_array(d.get('/odom', [])), odom_array(d.get('/Odometry', []))
    rep = Report()
    topics_section(rep, d)
    if len(o) > 2:
        {'static': lambda: static_section(rep, o, O), 'line': lambda: line_section(rep, o, gt),
         'spin': lambda: spin_section(rep, o, gt), 'loop': lambda: loop_section(rep, o, gt),
         'nav': lambda: nav_section(rep, d, o)}.get(test, lambda: None)()
    if mod:
        mod.section(rep, d, o, meta)
    hw = hardware_section(rep, bag, out_dir)
    pics = plots(out_dir, o, hw, test)
    verdict = ('FAIL' if 'FAIL' in rep.results else 'INCOMPLETE' if 'MISSING' in rep.results
               else 'PASS' if rep.results else 'INFO')
    head = [f'## {os.path.basename(os.path.normpath(bag))} — {test}: **{verdict}**', '',
            f'- date: {meta.get("date", "?")}, duration {meta.get("duration_s", "?")} s, '
            f'host {meta.get("host", "?")}, git {meta.get("git", "?")}, '
            f'lio_scan {meta.get("lio_scan", "?")}, numpy {meta.get("numpy", "?")}',
            f'- ground truth: {gt or "none"}', f'- notes: {meta.get("notes", "")}']
    md = head + rep.lines + [''] + [f'![{p}]({p})' for p in pics]
    with open(os.path.join(out_dir, 'report.md'), 'w') as f:
        f.write('\n'.join(md) + '\n')
    with open(os.path.join(out_dir, 'metrics.yaml'), 'w') as f:
        yaml.safe_dump({'test': test, 'verdict': verdict, 'meta': meta, 'metrics': rep.metrics}, f,
                       sort_keys=False)
    print('\n'.join(md))
    return verdict


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split('\n\n', 1)[1])
    p.add_argument('bags', nargs='+')
    p.add_argument('--test', help='override meta.yaml: ' + ', '.join(BUILTIN) +
                   ' or a go2w_metrics/validate_<test>.py module (detect, follow...)')
    a = p.parse_args()
    bad = 0
    for bag in a.bags:
        if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
            print(f'{bag}: not a bag folder', file=sys.stderr)
            continue
        bad += run(bag, a.test) == 'FAIL'
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
