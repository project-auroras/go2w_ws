"""validate --test follow: person follower on the robot (docs/validation/people_follow.md, P2-P8).

Uses /person_follower/status (JSON every control cycle: state, distance, target_distance,
bearing, person track in odom) + /odom + /cmd_vel + /scan, and the perception part of
validate_detect. Person ground speed comes from the follower's track (odom frame).
"""

import json
import math

import numpy as np

from go2w_metrics.validate_detect import CRITERIA as DETECT_CRITERIA
from go2w_metrics.validate_detect import TOPICS as DETECT_TOPICS, perception_section

TOPICS = DETECT_TOPICS | {'/person_follower/status'}
CRITERIA = {
    **{k: DETECT_CRITERIA[k] for k in ('people_hz', 'perception_latency_p95_ms')},
    'follow_distance_error_mean_m': ('<=', 0.30),
    'follow_distance_error_p95_m': ('<=', 0.80),
    'follow_min_distance_m': ('>=', 0.90),
    'follow_reacquire_time_max_s': ('<=', 5.0),
    'follow_time_pct': ('>=', 85.0),
}
FOLLOW, LOST, SEARCH, IDLE = 'siguiendo', 'perdida', 'buscando', 'parado'


def section(rep, d, o, meta):
    perception_section(rep, d, meta)
    rep.section('Following')
    st = []
    for tb, _, m in d.get('/person_follower/status', []):
        try:
            st.append((tb, json.loads(m.data)))
        except ValueError:
            pass
    if len(st) < 5:
        rep.add('follow_time_pct', None, '%', '/person_follower/status not in the bag')
        return
    t = np.array([x[0] for x in st])
    state = [s.get('state') for _, s in st]
    # from the first time it follows someone (before that it waits for the person)
    first = next((i for i, s in enumerate(state) if s == FOLLOW), None)
    if first is None:
        rep.add('follow_time_pct', 0.0, '%', 'never followed anyone')
        return
    t, state, st = t[first:], state[first:], st[first:]
    dur = t[-1] - t[0]
    dt = np.diff(t, append=t[-1])
    follow = np.array([s == FOLLOW for s in state])
    rep.add('follow_duration_s', dur, 's', 'from the first detection followed')
    rep.add('follow_time_pct', 100 * dt[follow].sum() / max(dur, 1e-6), '%')

    # episodes without seeing her: FOLLOW -> (LOST/SEARCH)* -> FOLLOW (reacquired) or not
    episodes, start, searched = [], None, False
    for i in range(1, len(state)):
        if state[i - 1] == FOLLOW and state[i] != FOLLOW:
            start, searched = t[i], False
        if start is not None and state[i] == SEARCH:
            searched = True
        if start is not None and state[i] == FOLLOW:
            episodes.append((t[i] - start, searched, True))
            start = None
    if start is not None:
        episodes.append((t[-1] - start, searched, False))
    back = [e[0] for e in episodes if e[2]]
    rep.add('follow_loss_events', len(episodes), '',
            f'{sum(e[1] for e in episodes)} with search, {sum(not e[2] for e in episodes)} never reacquired')
    rep.add('follow_loss_per_min', 60 * len(episodes) / max(dur, 1e-6), '/min')
    rep.add('follow_reacquire_time_max_s', max(back) if back else (0.0 if not episodes else None), 's',
            f'median {np.median(back):.1f} s' if back else '')
    ids = [s.get('id') for _, s in st if s.get('id') is not None]
    rep.add('follow_id_changes', sum(a != b for a, b in zip(ids, ids[1:])), '',
            'tracker ID switches of the followed person (re-hooked or another person)')

    f = [s for (_, s), ok in zip(st, follow) if ok and s.get('distance') is not None]
    if f:
        dist = np.array([s['distance'] for s in f])
        err = dist - np.array([s['target_distance'] for s in f])
        bear = np.degrees(np.abs([s['bearing'] for s in f]))
        rep.add('follow_distance_mean_m', dist.mean(), 'm',
                f'target {np.mean([s["target_distance"] for s in f]):.2f} m on average')
        rep.add('follow_distance_error_mean_m', abs(err.mean()), 'm', f'signed {err.mean():+.2f} m (+ = behind)')
        rep.add('follow_distance_error_p95_m', np.percentile(np.abs(err), 95), 'm')
        rep.add('follow_min_distance_m', dist.min(), 'm', 'closest it got to the person')
        rep.add('follow_bearing_p95_deg', np.percentile(bear, 95), 'deg')
        notes = [s.get('note', '') for s in f]
        rep.add('follow_obstacle_limited_pct', 100 * np.mean(['obst' in n for n in notes]), '%',
                'cycles slowed down by an obstacle')

    # speeds: person (track, odom) vs robot (/odom twist)
    pv = np.array([(tb, math.hypot(s.get('vx', 0), s.get('vy', 0))) for tb, s in st if 'vx' in s])
    od = d.get('/odom', [])
    if len(pv) > 5 and len(od) > 5:
        ot = np.array([x[0] for x in od])
        ov = np.array([math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) for _, _, m in od])
        sel = (ot >= t[0]) & (ot <= t[-1])
        rep.add('robot_speed_mean_mps', ov[sel].mean(), 'm/s', f'max {ov[sel].max():.2f} m/s (/odom)')
        rep.add('person_speed_mean_mps', pv[:, 1].mean(), 'm/s', f'max {pv[:, 1].max():.2f} m/s (follower track)')
        # response time: lag that best aligns robot speed with person speed (10 Hz grid)
        grid = np.arange(t[0], t[-1], 0.1)
        if len(grid) > 50:
            a = np.interp(grid, pv[:, 0], pv[:, 1]) - pv[:, 1].mean()
            b = np.interp(grid, ot, ov) - ov[sel].mean()
            lags = range(0, 31)
            corr = [np.dot(a[:len(a) - k], b[k:]) for k in lags]
            if max(corr) > 0:
                rep.add('follow_response_lag_s', 0.1 * lags[int(np.argmax(corr))], 's',
                        'robot speed vs person speed, cross-correlation')
    cmd = d.get('/cmd_vel', [])
    if len(cmd) > 5:
        ct = np.array([x[0] for x in cmd])
        cv = np.array([m.linear.x for _, _, m in cmd])
        acc = np.abs(np.diff(cv) / np.maximum(np.diff(ct), 1e-3))
        rep.add('cmd_linear_max_mps', cv.max(), 'm/s', f'limit {meta.get("max_linear", "?")}')
        rep.add('cmd_accel_p95_mps2', np.percentile(acc, 95), 'm/s2')
    scans = [m for tb, _, m in d.get('/scan', []) if t[0] <= tb <= t[-1]]
    if scans:
        mins = [np.nanmin(np.where(np.isfinite(r := np.asarray(m.ranges)) & (r >= m.range_min), r, np.nan))
                for m in scans if np.isfinite(m.ranges).any()]
        if mins:
            rep.add('follow_min_scan_range_m', float(np.nanmin(mins)), 'm',
                    'includes the person (lio_scan sees her too)')
