"""Test bag -> CSVs for statistics (one folder per bag).

  ros2 run go2w_metrics bag_to_csv /bags/my_test                      # -> /bags/my_test/csv/
  ros2 run go2w_metrics bag_to_csv /bags/my_test --clock-offset 1.334 # NX clock - laptop clock
  ros2 run go2w_metrics bag_to_csv /bags/*                            # several bags

Every row has t_bag: when the laptop received the message (laptop clock), the common time
axis of all the files. Files (only those whose topics are in the bag):
  system.csv     /vitals/status per host: CPU (total and per core), load, RAM, swap, CPU
                 temperature, NVIDIA GPU (laptop: load, VRAM, temperature, power)
  network.csv    /vitals/status per host and interface: tx/rx rates, drops, errors
  processes.csv  /vitals/status per host and ROS process (and child node of a launch):
                 CPU, RAM, threads, TCP rx/tx
  jetson.csv     /jetson/stats (tegrastats of the NX): GPU, EMC, per-core CPU, thermal
                 zones, power rails
  messages.csv   one row per message of the other topics: t_bag, t_stamp (header.stamp,
                 corrected with --clock-offset) and latency = t_bag - t_stamp
  summary.csv    per topic: count, rate, period jitter and latency (mean, p50, p95, max);
                 per host: mean and max of CPU, RAM, GPU, temperatures, power

latency: for topics stamped on the NX (cameras, /people, /odom, /scan...) it includes the
NX -> laptop clock offset. Measure it with the claps (scripts/bag_frames.py) and pass it
with --clock-offset: it is subtracted from the stamps of every topic not starting with
--laptop-prefix (default /external, the laptop camera).
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

VITALS = 'ros2_vitals_msgs/msg/SystemStatus'
DIAG = 'diagnostic_msgs/msg/DiagnosticArray'
# no point in a row per message (static, or huge and already in other files)
SKIP_MESSAGES = {'/tf_static', '/rosout', '/parameter_events'}


def sec(t):
    return t.sec + t.nanosec * 1e-9


def mb(b):
    return round(b / 1e6, 2)


class Writer:
    """CSV files opened on first use; the header is the keys of the first row (+ new ones
    of later rows are added at the end)."""

    def __init__(self, folder):
        self.folder = folder
        self.rows = defaultdict(list)

    def add(self, name, row):
        self.rows[name].append(row)

    def save(self):
        os.makedirs(self.folder, exist_ok=True)
        for name, rows in self.rows.items():
            fields = list(dict.fromkeys(k for r in rows for k in r))
            with open(os.path.join(self.folder, name), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                w.writerows(rows)
            print(f'  {name}: {len(rows)} rows')


def vitals_rows(out, t_bag, m):
    host = m.hostname
    base = {'t_bag': t_bag, 't_stamp': sec(m.stamp), 'host': host}
    row = dict(base, cpu_percent=m.cpu_percent, cpu_count=m.cpu_count,
               load_1min=m.load_avg_1min, ram_used_mb=mb(m.ram_used_bytes),
               ram_total_mb=mb(m.ram_total_bytes), ram_available_mb=mb(m.ram_available_bytes),
               swap_used_mb=mb(m.swap_used_bytes), cpu_temp_c=m.cpu_temperature_celsius)
    for i, c in enumerate(m.cpu_per_core):
        row[f'cpu{i}_percent'] = c
    for g in m.gpus:
        p = f'gpu{g.index}_'
        row.update({p + 'percent': g.utilization_percent,
                    p + 'mem_used_mb': mb(g.memory_used_bytes),
                    p + 'mem_total_mb': mb(g.memory_total_bytes),
                    p + 'temp_c': g.temperature_celsius, p + 'power_w': g.power_watts})
    out.add('system.csv', row)
    for n in m.network_interfaces:
        out.add('network.csv', dict(
            base, iface=n.name, up=n.is_up, tx_kbps=round(n.bytes_sent_per_sec * 8e-3, 1),
            rx_kbps=round(n.bytes_recv_per_sec * 8e-3, 1), drops_in=n.drops_in,
            drops_out=n.drops_out, errors_in=n.errors_in, errors_out=n.errors_out))
    for p in m.processes:
        name = p.launch_name if p.is_launch_process else p.node_name
        if not name and p.cmdline:
            name = os.path.basename(p.cmdline.split()[0])
        out.add('processes.csv', dict(
            base, pid=p.pid, parent_pid='', name=name, namespace=p.node_namespace,
            launch=p.is_launch_process,
            cpu_percent=p.cpu_percent, ram_mb=mb(p.ram_bytes), threads=p.num_threads,
            gpu_mem_mb=mb(p.gpu_memory_bytes), net_rx_kbps=round(p.net_rx_bytes_per_sec * 8e-3, 1),
            net_tx_kbps=round(p.net_tx_bytes_per_sec * 8e-3, 1), cmdline=p.cmdline[:200]))
        for c in p.child_nodes:
            out.add('processes.csv', dict(
                base, pid=c.pid, parent_pid=p.pid, name=c.node_name, namespace=c.node_namespace,
                launch=False, cpu_percent=c.cpu_percent, ram_mb=mb(c.ram_bytes), threads='',
                gpu_mem_mb=mb(c.gpu_memory_bytes),
                net_rx_kbps=round(c.net_rx_bytes_per_sec * 8e-3, 1),
                net_tx_kbps=round(c.net_tx_bytes_per_sec * 8e-3, 1), cmdline=c.cmdline[:200]))


def jetson_rows(out, t_bag, m):
    for s in m.status:
        row = {'t_bag': t_bag, 't_stamp': sec(m.header.stamp), 'host': s.hardware_id}
        for kv in s.values:
            try:
                row[kv.key] = float(kv.value)
            except ValueError:
                row[kv.key] = kv.value
        out.add('jetson.csv', row)


def stats(x):
    x = np.asarray([v for v in x if v is not None and math.isfinite(v)], dtype=float)
    if not len(x):
        return {}
    return {'mean': x.mean(), 'p50': np.percentile(x, 50), 'p95': np.percentile(x, 95),
            'max': x.max()}


def summary(out):
    rows = []
    # topics
    by_topic = defaultdict(list)
    for r in out.rows.get('messages.csv', []):
        by_topic[r['topic']].append(r)
    for topic, msgs in sorted(by_topic.items()):
        t = np.array([r['t_bag'] for r in msgs])
        row = {'kind': 'topic', 'name': topic, 'count': len(msgs),
               'duration_s': round(t[-1] - t[0], 3) if len(t) > 1 else 0}
        if len(t) > 1:
            dt = np.diff(t)
            row['rate_hz'] = round((len(t) - 1) / (t[-1] - t[0]), 3) if t[-1] > t[0] else ''
            row['period_ms_p95'] = round(np.percentile(dt, 95) * 1e3, 1)
            row['period_ms_max'] = round(dt.max() * 1e3, 1)
        for k, v in stats(r['latency_ms'] for r in msgs if r['latency_ms'] != '').items():
            row[f'latency_ms_{k}'] = round(v, 1)
        rows.append(row)
    # hosts
    for name, keys in (('system.csv', None), ('jetson.csv', None)):
        by_host = defaultdict(list)
        for r in out.rows.get(name, []):
            by_host[r['host']].append(r)
        for host, rs in sorted(by_host.items()):
            row = {'kind': name[:-4], 'name': host, 'count': len(rs)}
            cols = [k for k in rs[0] if k not in ('t_bag', 't_stamp', 'host')
                    and isinstance(rs[0][k], (int, float)) and not isinstance(rs[0][k], bool)
                    and not (k.startswith('cpu') and k[3:4].isdigit())]
            for k in cols:
                s = stats(r.get(k) for r in rs if isinstance(r.get(k), (int, float)))
                if s:
                    row[f'{k}_mean'] = round(s['mean'], 2)
                    row[f'{k}_max'] = round(s['max'], 2)
            rows.append(row)
    for r in rows:
        out.add('summary.csv', r)


def convert(bag, folder, clock_offset, laptop_prefix):
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=storage), ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    for topic, type_name in types.items():
        try:
            classes[topic] = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            print(f'  ! {topic}: type {type_name} not available (source the workspace)',
                  file=sys.stderr)
    out = Writer(folder)
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in classes:
            continue
        t_bag = t_ns * 1e-9
        type_name = types[topic]
        if type_name == VITALS:
            vitals_rows(out, t_bag, deserialize_message(data, classes[topic]))
            continue
        if type_name == DIAG and topic.startswith('/jetson'):
            jetson_rows(out, t_bag, deserialize_message(data, classes[topic]))
            continue
        if topic in SKIP_MESSAGES:
            continue
        msg = deserialize_message(data, classes[topic])
        row = {'topic': topic, 't_bag': t_bag, 't_stamp': '', 'latency_ms': ''}
        header = getattr(msg, 'header', None)
        if header is not None and (header.stamp.sec or header.stamp.nanosec):
            stamp = sec(header.stamp)
            if not topic.startswith(laptop_prefix):
                stamp -= clock_offset
            row['t_stamp'] = stamp
            row['latency_ms'] = round((t_bag - stamp) * 1e3, 2)
        out.add('messages.csv', row)
    summary(out)
    out.save()


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__.split('\n\n', 1)[1])
    p.add_argument('bags', nargs='+', help='bag folders')
    p.add_argument('--clock-offset', type=float, default=0.0,
                   help='s, NX clock - laptop clock (claps), subtracted from NX stamps')
    p.add_argument('--laptop-prefix', default='/external',
                   help='topics stamped with the laptop clock (no offset)')
    p.add_argument('--out', help='output folder (default <bag>/csv; only with one bag)')
    a = p.parse_args()
    for bag in a.bags:
        if not os.path.isfile(os.path.join(bag, 'metadata.yaml')):
            print(f'{bag}: not a bag folder (no metadata.yaml), skipped', file=sys.stderr)
            continue
        folder = a.out if a.out and len(a.bags) == 1 else os.path.join(bag, 'csv')
        print(f'{bag} -> {folder}')
        convert(bag, folder, a.clock_offset, a.laptop_prefix)


if __name__ == '__main__':
    main()
