#!/usr/bin/env python3
"""
전이 점검 2 — LaserScan 규약 (gzbench_runner --probe scan 의 probe_scan.json → 표).

    python3 rl/eval/gazebo/analyze_scan.py <probe_scan.json> [--out <json>]

본 것: 빔 수·각도(angle_min, increment, angle_max)·range_min/max·frame·inf/NaN 수를 계약
(constants.py, sensors.yaml)과 대조,
그리고 **순서·원점·방향**: 지면 진실 자세에서 스캔 평면(0.20 m)을 지나는 월드 도형으로 광선을 쏴 기대 거리를 만들고
측정 거리와 비교한다. 도형은 두 벌:
  - visual (gpu_lidar 가 실제로 재는 면 — 랙은 기둥·선반만;
    tests/integration/amr_itest/worldmap.footprints 를 읽기만 한다)
  - collision 상자 (Isaac 학습 환경의 RayCaster 가 쏘는 메시 = rl/isaac warehouse_geom 과 같은 사각형)
규약이 맞으면 visual 잔차가 LiDAR 잡음(σ 0.03) 수준이고, 빔 순서를 뒤집거나 180° 돌리면 커진다 (그 두 대조를 같이 낸다 —
점검이 실제로 규약을 가르는지 보이려고). collision 잔차의 양의 꼬리는 Isaac 이 막힌 상자로 보는 곳을 Gazebo LiDAR 가 뚫고
더 멀리 보는 빔(랙 기둥 사이)이다 — 규약이 아니라 **월드 표현의 차이**로 따로 센다.
"""

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'amr_rl_bench'))
sys.path.insert(0, os.path.join(REPO, 'rl', 'contract'))

from amr_rl_bench import world_geom  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402

SCAN_Z = 0.20


def visual_shapes(z=SCAN_Z):
    """스캔 평면을 지나는 visual 도형 → (사각형 obb [M, 5], 원 [K, 3]). amr_itest.worldmap 을 읽기만 한다."""
    sys.path.insert(0, os.path.join(REPO, 'tests', 'integration'))
    from amr_itest import worldmap
    shapes = worldmap.footprints(os.path.join(REPO, 'src', 'amr_simulation', 'worlds',
                                              'warehouse.sdf'),
                                 os.path.join(REPO, 'src', 'amr_simulation', 'models'), z,
                                 'visual')
    rects = [[s.cx, s.cy, s.sx / 2, s.sy / 2, s.yaw] for s in shapes
             if isinstance(s, worldmap.Rect)]
    circles = [[s.cx, s.cy, s.r] for s in shapes if isinstance(s, worldmap.Circle)]
    return (np.asarray(rects, dtype=float).reshape(-1, 5),
            np.asarray(circles, dtype=float).reshape(-1, 3))


def raycast_circles(circles, ox, oy, angles):
    """광선 ↔ 원 첫 교차 거리 [N] (없으면 inf)."""
    if circles.size == 0:
        return np.full(len(angles), np.inf)
    d = np.stack([np.cos(angles), np.sin(angles)], 1)
    oc = np.array([ox, oy])[None, :] - circles[:, :2]            # [K, 2]
    b = d @ oc.T                                                  # [N, K]
    c = np.sum(oc * oc, axis=1)[None, :] - circles[None, :, 2] ** 2
    disc = b * b - c
    t = -b - np.sqrt(np.where(disc >= 0, disc, np.nan))
    t = np.where((disc >= 0) & (t > 1e-6), t, np.inf)
    return np.min(t, axis=1)


def raycast(obb, ox, oy, angles, max_range=25.0):
    """2D 광선 (원점 ox, oy, 세계 각 angles [N]) ↔ 사각형 변들 첫 교차 거리 [N] (없으면 inf)."""
    segs = []
    for cx, cy, hx, hy, yaw in obb:
        c = world_geom.rect_corners(cx, cy, hx, hy, yaw)
        for i in range(4):
            segs.append((c[i], c[(i + 1) % 4]))
    a = np.array([s[0] for s in segs])
    b = np.array([s[1] for s in segs])
    d = np.stack([np.cos(angles), np.sin(angles)], 1)            # [N, 2]
    e = b - a                                                    # [M, 2]
    o = np.array([ox, oy])
    # o + t·d = a + u·e  →  t·d − u·e = a − o
    den = d[:, None, 0] * (-e[None, :, 1]) - d[:, None, 1] * (-e[None, :, 0])   # [N, M]
    ao = a - o
    t = (ao[None, :, 0] * (-e[None, :, 1]) - ao[None, :, 1] * (-e[None, :, 0])) / \
        np.where(np.abs(den) < 1e-12, np.nan, den)
    u = (d[:, None, 0] * ao[None, :, 1] - d[:, None, 1] * ao[None, :, 0]) / \
        np.where(np.abs(den) < 1e-12, np.nan, den)
    ok = (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
    t = np.where(ok, t, np.inf)
    r = np.min(t, axis=1)
    return np.where(r <= max_range, r, np.inf)


def residuals(scan, obb, order='as_is', circles=None):
    """한 스캔의 (측정 − 기대) 잔차 (측정·기대 모두 유한하고 10 m 안인 빔)."""
    x, y, yaw = scan['gt'][1], scan['gt'][2], scan['gt'][3]
    n = len(scan['ranges'])
    ang = scan['angle_min'] + np.arange(n) * scan['angle_increment']
    meas = np.array([np.inf if r is None else r for r in scan['ranges']], dtype=float)
    if order == 'reversed':
        meas = meas[::-1]
    elif order == 'rot180':
        meas = np.roll(meas, n // 2)
    ox, oy = x + C.LIDAR_X * math.cos(yaw), y + C.LIDAR_X * math.sin(yaw)
    exp = raycast(obb, ox, oy, yaw + ang)
    if circles is not None:
        exp = np.minimum(exp, raycast_circles(circles, ox, oy, yaw + ang))
    m = np.isfinite(meas) & np.isfinite(exp) & (exp <= 10.0) & (meas <= 10.0)
    return meas[m] - exp[m], int(np.isfinite(exp).sum()), int(np.isfinite(meas).sum())


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('probe')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    d = json.load(open(a.probe, encoding='utf-8'))
    sdf = os.path.join(REPO, 'src', 'amr_simulation', 'worlds', 'warehouse.sdf')
    obb = world_geom.load_obb(sdf, os.path.join(REPO, 'src', 'amr_simulation', 'models'),
                              SCAN_Z - 1e-3, SCAN_Z + 1e-3)
    scans = [s for s in d['scans'] if s.get('gt')]
    s0 = scans[0]
    conv = {
        'n_beams': len(s0['ranges']), 'contract_n': C.LIDAR_SAMPLES,
        'angle_min': s0['angle_min'], 'contract_angle_min': C.LIDAR_ANGLE_MIN,
        'angle_increment_deg': math.degrees(s0['angle_increment']),
        'contract_increment_deg': math.degrees(C.LIDAR_ANGLE_INC),
        'angle_max': s0['angle_max'], 'contract_angle_max': C.LIDAR_ANGLE_MAX,
        'beam_index_of_0deg': -s0['angle_min'] / s0['angle_increment'],
        'range_min': s0['range_min'], 'range_max': s0['range_max'],
        'frame_id': s0['frame_id'], 'scan_time': s0.get('scan_time'),
        'n_inf_per_scan': [s['n_inf'] for s in scans],
        'n_nan_per_scan': [s['n_nan'] for s in scans],
        'raw': {k: s0['raw'][k] for k in ('frame_id', 'n', 'angle_min', 'angle_increment')}
        if s0.get('raw') else None,
    }
    if s0.get('raw'):
        raw = np.array([np.inf if r is None else r for r in s0['raw']['ranges']])
        filt = np.array([np.inf if r is None else r for r in s0['ranges']])
        both = np.isfinite(raw) & np.isfinite(filt)
        conv['filtered_vs_raw'] = {'finite_raw': int(np.isfinite(raw).sum()),
                                   'finite_filtered': int(np.isfinite(filt).sum()),
                                   'max_abs_diff_common':
                                   float(np.max(np.abs(raw[both] - filt[both])))
                                   if both.any() else None}
    vis_rect, vis_circ = visual_shapes()
    res = {}
    for geom, (rects, circ) in (('visual', (vis_rect, vis_circ)), ('collision', (obb, None))):
        for order in ('as_is', 'reversed', 'rot180'):
            if geom == 'collision' and order != 'as_is':
                continue
            rs = []
            for s in scans:
                r, _ne, _nm = residuals(s, rects, order, circ)
                rs.append(r)
            r = np.concatenate(rs)
            res[f'{geom}/{order}'] = {
                'n_pairs': int(r.size), 'median_abs_m': float(np.median(np.abs(r))),
                'p90_abs_m': float(np.percentile(np.abs(r), 90)),
                'frac_within_0p05': float(np.mean(np.abs(r) <= 0.05)),
                'frac_within_0p10': float(np.mean(np.abs(r) <= 0.10)),
                'frac_farther_0p10': float(np.mean(r > 0.10)),
                'frac_closer_0p10': float(np.mean(r < -0.10)),
                'mean_m': float(np.mean(r)), 'std_m': float(np.std(r))}
    conv['n_visual_rects'] = int(vis_rect.shape[0])
    conv['n_visual_circles'] = int(vis_circ.shape[0])
    out = {'convention': conv, 'raycast_residuals': res, 'n_scans': len(scans),
           'pose_gt': s0['gt'][1:4], 'n_rect_scan_plane': int(obb.shape[0])}
    txt = json.dumps(out, ensure_ascii=False, indent=1)
    print(txt)
    if a.out:
        open(a.out, 'w', encoding='utf-8').write(txt)


if __name__ == '__main__':
    main()
