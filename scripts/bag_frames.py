#!/usr/bin/env python3
"""Export the frames of a time window of a rosbag as JPEG named by header.stamp.

Used to sync the NX and laptop clocks with a clap: find the frame where the hands touch
in each camera; offset = stamp_nx - stamp_laptop, subtract it from the NX stamps.

  python3 scripts/bag_frames.py BAG OUT_DIR --start 0 --duration 5 TOPIC [TOPIC ...]

--start is seconds from the start of the bag (bag time); the file names are the
header.stamp of each image (capture time, the clock of the machine with the camera).
Works with sensor_msgs/Image and sensor_msgs/CompressedImage topics.
"""

import argparse
import os

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from sensor_msgs.msg import CompressedImage, Image


def to_bgr(msg):
    if isinstance(msg, CompressedImage):
        return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return img if msg.encoding == 'bgr8' else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('out_dir')
    p.add_argument('topics', nargs='+')
    p.add_argument('--start', type=float, default=0.0, help='s from the start of the bag')
    p.add_argument('--duration', type=float, default=5.0, help='s to export')
    a = p.parse_args()

    r = SequentialReader()
    r.open(StorageOptions(uri=a.bag, storage_id='sqlite3'), ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    for t in a.topics:
        if types.get(t) not in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
            raise SystemExit(f'{t}: not an image topic in the bag ({types.get(t)})')
    r.set_filter(StorageFilter(topics=a.topics))

    t0 = None
    count = {t: 0 for t in a.topics}
    while r.has_next():
        topic, data, bag_ns = r.read_next()
        t0 = bag_ns if t0 is None else t0
        rel = (bag_ns - t0) * 1e-9
        if rel < a.start:
            continue
        if rel > a.start + a.duration:
            break
        cls = CompressedImage if types[topic].endswith('CompressedImage') else Image
        msg = deserialize_message(data, cls)
        stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        folder = os.path.join(a.out_dir, topic.strip('/').replace('/', '_'))
        os.makedirs(folder, exist_ok=True)
        img = to_bgr(msg)
        cv2.putText(img, stamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imwrite(os.path.join(folder, f'{stamp}.jpg'), img)
        count[topic] += 1

    for t, n in count.items():
        print(f'{t}: {n} frames')


if __name__ == '__main__':
    main()
