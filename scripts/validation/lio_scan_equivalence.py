"""Equivalence test of two lio_scan.py versions: same /Odometry + /cloud_registered in, is /scan
bit-for-bit identical, and how long does on_cloud take?

  git show <rev>:src/go2w_bringup/scripts/lio_scan.py > /tmp/lio_scan_old.py
  python3 scripts/validation/lio_scan_equivalence.py <bag> /tmp/lio_scan_old.py \
      src/go2w_bringup/scripts/lio_scan.py [--profile]

<bag>: /cloud_registered, /Odometry and /tf_static, e.g. recorded while replaying a bag with raw
sensors through fastlio_mapping (config/fast_lio.yaml):
  ros2 bag record -o /tmp/lio_out /cloud_registered /Odometry /tf_static
  ros2 bag play bags/<bag> --topics /utlidar/cloud /utlidar/imu /lidar_imu /tf_static
"""
import importlib.util, sys, time, cProfile, pstats
import numpy as np, rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from rclpy.time import Time

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m

bag, old_py, new_py = sys.argv[1:4]
r = SequentialReader(); r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
msgs, buf = [], Buffer()
while r.has_next():
    topic, data, _ = r.read_next()
    m = deserialize_message(data, get_message(types[topic]))
    if topic == '/tf_static':
        for t in m.transforms: buf.set_transform_static(t, 'bag')
    else:
        msgs.append((topic, m))
rclpy.init()
results = {}
for label, path in (('old', old_py), ('new', new_py)):
    mod = load(path, 'lio_scan_' + label)
    node = mod.LioScan()
    t = buf.lookup_transform(node.base_frame, node.lidar_frame, Time())
    rr, tr = t.transform.rotation, t.transform.translation
    node.base_to_lidar = mod.matrix((rr.x, rr.y, rr.z, rr.w), (tr.x, tr.y, tr.z))
    out = []
    node.scan_pub.publish = lambda s, out=out: out.append(np.array(s.ranges, dtype=np.float32))
    times = []
    prof = cProfile.Profile() if '--profile' in sys.argv else None
    for topic, m in msgs:
        if topic == '/Odometry':
            node.on_odometry(m)
        elif topic == '/cloud_registered':
            t0 = time.perf_counter()
            if prof: prof.enable()
            node.on_cloud(m)
            if prof: prof.disable()
            times.append(time.perf_counter() - t0)
    times = np.array(times) * 1e3
    results[label] = out
    npts = np.mean([m.width * m.height for tp, m in msgs if tp == '/cloud_registered'])
    print(f'{label}: {len(out)} scans, {npts:.0f} pts/cloud, on_cloud mean {times.mean():.2f} ms '
          f'p95 {np.percentile(times, 95):.2f} ms  -> {times.mean() * 15.4 / 10:.1f} % of a core at 15.4 Hz')
    if prof: pstats.Stats(prof).sort_stats('tottime').print_stats(8)
    node.destroy_node()
a, b = results['old'], results['new']
assert len(a) == len(b), (len(a), len(b))
same = [np.array_equal(x, y) for x, y in zip(a, b)]
diff = np.concatenate([np.abs(np.where(np.isfinite(x) & np.isfinite(y), np.nan_to_num(x - y), 0)) for x, y in zip(a, b)])
inf_mismatch = sum(int((np.isfinite(x) != np.isfinite(y)).sum()) for x, y in zip(a, b))
print(f'identical scans: {sum(same)}/{len(same)}, max |diff| {diff.max():.2e} m, beams finite in one only: {inf_mismatch}')
