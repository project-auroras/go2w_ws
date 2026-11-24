"""Point-to-plane information of the scene around the start pose (robot at rest, first ~3 s):
sum of n n^T over planar points (normals by local PCA), in the robot frame. Low information along x
(forward) = the scene barely constrains the motion of the line test. Same venv as kiss_line.py:
  v/bin/python scripts/validation/offline/scene_constraint.py bags/*_line"""
import sys, math, numpy as np
from pathlib import Path
from scipy.spatial import cKDTree
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
sys.path.insert(0, str(Path(__file__).parent)); from kiss_line import cloud_xyz_t, R_BL, T_BL
for bag in sys.argv[1:]:
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as r:
        c = [c for c in r.connections if c.topic == '/utlidar/cloud']
        cl = []
        for i, (cn, t, raw) in enumerate(r.messages(connections=c)):
            if i >= 40: break
            cl.append(cloud_xyz_t(r.deserialize(raw, cn.msgtype))[0])
    p = np.vstack(cl) @ R_BL.T + T_BL
    d = np.linalg.norm(p - T_BL, axis=1)
    p = p[(d > 0.5) & (d < 20)]
    p = p[np.unique(np.floor(p / 0.05), axis=0, return_index=True)[1]]   # 5 cm voxels
    tree = cKDTree(p); nb = tree.query(p, k=12)[1]
    q = p[nb] - p[nb].mean(1, keepdims=True)
    w, v = np.linalg.eigh(np.einsum('nki,nkj->nij', q, q))
    n = v[:, :, 0]; planar = w[:, 0] < 0.05 * w[:, 1]
    n = n[planar]; dist = np.linalg.norm(p[planar] - T_BL, axis=1)
    def info(m): H = (n[m].T @ n[m]); return np.diag(H) / max(m.sum(), 1)
    nofloor = np.abs(n[:, 2]) < 0.7
    ix, iy, iz = info(nofloor)
    walls = n[nofloor]
    ang = np.degrees(np.arctan2(walls[:, 1], walls[:, 0])) % 180
    h, _ = np.histogram(ang, bins=6, range=(0, 180))
    print(f'{Path(bag).name}: {len(p)} pts, {planar.sum()} planar, {nofloor.sum()} on walls/objects '
          f'| wall normals: x(forward) {ix:.2f}  y(side) {iy:.2f}  -> x/y {ix/iy:.2f} '
          f'| normal azimuth 0-180 deg in 30 deg bins: {[int(x) for x in h]} | median wall range {np.median(dist[nofloor]):.1f} m')
