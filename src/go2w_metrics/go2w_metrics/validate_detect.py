"""validate --test detect: person detection with the robot still (docs/validation/people_follow.md, P1).

The person stands ~10 s on each tape mark (meta.yaml ground_truth.marks_m, e.g. [1, 2, 3, 4, 5, 6]).
Also the perception part shared with the follow test: rates, frames with a person and latency.

Latency without clock offset: /people keeps the stamp of its image, so
  processing = arrival of /people - arrival of the NX image with the same stamp (laptop clock:
  decompression + YOLO + person_filter). Camera + WiFi needs the NX - laptop clock offset
  (meta.yaml clock_offset_s, claps); without it, it is shown relative to the fastest frame.
"""

import math

import numpy as np

NX_IMAGE = '/camera/camera/color/image_raw/compressed'
TOPICS = {NX_IMAGE, '/people', '/yolo/cam0/detections_3d', '/person_follower/status',
          '/cmd_vel', '/external/cam/color/image_raw/compressed'}
CRITERIA = {
    'people_hz': ('>=', 10.0),
    'perception_latency_p95_ms': ('<=', 150.0),
    'detect_range_bias_max_m': ('<=', 0.25),
    'detect_frames_with_person_min_pct': ('>=', 90.0),
}


def ranges(m):
    """Horizontal distance of every person of a DetectionArray (0 = no valid depth)."""
    out = []
    for det in m.detections:
        p = det.bbox3d.center.position
        frame = det.bbox3d.frame_id or m.header.frame_id
        f, l = (p.z, -p.x) if 'optical' in frame else (p.x, p.y)
        if f > 0.1 and math.isfinite(f):
            out.append(math.hypot(f, l))
    return out


def perception_section(rep, d, meta):
    rep.section('Perception (camera NX -> WiFi -> YOLO on the laptop)')
    imgs, people = d.get(NX_IMAGE, []), d.get('/people', [])
    for topic, key, msgs in ((NX_IMAGE, 'camera_hz', imgs), ('/people', 'people_hz', people)):
        if len(msgs) > 2:
            tb = np.array([x[0] for x in msgs])
            rep.add(key, (len(tb) - 1) / (tb[-1] - tb[0]), 'Hz',
                    f'{topic}, gap max {np.diff(tb).max() * 1e3:.0f} ms')
        else:
            rep.add(key, None, 'Hz', f'{topic} not in the bag')
    if not people:
        return
    with_person = np.array([len(ranges(m)) > 0 for _, _, m in people])
    rep.add('frames_with_person_pct', 100 * with_person.mean(), '%', 'of /people messages')
    if imgs:
        arrival = {round(s, 4): tb for tb, s, _ in imgs}
        proc = [(tb - arrival[round(s, 4)]) * 1e3 for tb, s, _ in people if round(s, 4) in arrival]
        if proc:
            rep.add('perception_latency_p50_ms', np.percentile(proc, 50), 'ms',
                    'image arrival -> /people (decompress + YOLO + filter)')
            rep.add('perception_latency_p95_ms', np.percentile(proc, 95), 'ms')
        net = np.array([(tb - s) * 1e3 for tb, s, _ in imgs])
        off = meta.get('clock_offset_s')
        if off is not None:
            net = net - off * 1e3
            rep.add('camera_wifi_latency_p50_ms', np.percentile(net, 50), 'ms', 'stamp -> laptop')
            rep.add('camera_wifi_latency_p95_ms', np.percentile(net, 95), 'ms')
        else:
            rel = net - net.min()
            rep.add('camera_wifi_jitter_p95_ms', np.percentile(rel, 95), 'ms',
                    'over the fastest frame (no clock_offset_s in meta.yaml)')


def section(rep, d, o, meta):
    perception_section(rep, d, meta)
    rep.section('Detection at tape marks (robot still)')
    marks = sorted((meta.get('ground_truth') or {}).get('marks_m') or [])
    people = d.get('/people', [])
    if not marks or not people:
        rep.add('detect_range_bias_max_m', None, 'm', 'no ground_truth.marks_m in meta.yaml')
        return
    # every message -> nearest mark (half the spacing around it); a frame without the person
    # counts against the mark where the person was last seen
    half = min(0.5, min(np.diff(marks)) / 2) if len(marks) > 1 else 0.5
    per = {mk: {'r': [], 'frames': 0} for mk in marks}
    current = None
    for _, _, m in people:
        rs = ranges(m)
        near = [(abs(r - mk), r, mk) for r in rs for mk in marks if abs(r - mk) <= half]
        if near:
            _, r, mk = min(near)
            per[mk]['r'].append(r)
            current = mk
        if current is not None:
            per[current]['frames'] += 1
    rows, biases, rates = [], [], []
    for mk in marks:
        r = np.array(per[mk]['r'])
        if not len(r):
            rows.append(f'| {mk:g} | 0 | — | — | — | 0 |')
            continue
        bias = r.mean() - mk
        rate = 100 * len(r) / max(per[mk]['frames'], 1)
        biases.append(abs(bias))
        rates.append(rate)
        rows.append(f'| {mk:g} | {len(r)} | {r.mean():.2f} | {bias:+.2f} | {r.std():.3f} | {rate:.0f} |')
    rep.add('detect_marks_seen', sum(1 for mk in marks if per[mk]['r']), '', f'of {len(marks)}')
    rep.add('detect_range_bias_max_m', max(biases) if biases else None, 'm')
    rep.add('detect_frames_with_person_min_pct', min(rates) if rates else None, '%',
            'worst mark: frames with the person / frames while there')
    rep.lines += ['', '| mark m | detections | measured m | bias m | std m | frames with person % |',
                  '|---|---|---|---|---|---|'] + rows
