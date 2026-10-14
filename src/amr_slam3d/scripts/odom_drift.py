#!/usr/bin/env python3
"""
보충(사전 등록 외): 녹화 bag 의 휠-EKF odom 과 wheel_odom 이 정답에서 얼마나 벗어나나 (컨테이너, rosbag2_py).

첫 자세를 SE(2) 로 맞춘 뒤(odom 원점 = 출발 자세) 시간에 따른 xy·yaw 오차. 2026-10-14 E2 중간 분석에서 찾은 EKF 결함
(기동 자이로 바이어스 추정 실패 → yaw 0.01 rad/s 표류) 의 크기를 bag 마다 잰다.

    odom_drift.py --rec <b2-rec rec 루트> --out <디렉터리>
"""

import argparse
import json
import math
import os
import subprocess
import sys

import numpy as np

BAGS = ('s3_aisle_a1', 's1_seed1_a1', 's1_seed3_a2', 's1_seed2x_a1', 's6_open_a3',
        's6_queue_a1', 's6_wall_a1', 'map_long_a1')
TOPICS = ('/amr_01/odometry/filtered', '/amr_01/wheel_odom')


def drift(t, oxy, oyaw, gt):
    """주행 기록 (t, xy, yaw) 대 정답 Traj2D — 첫 자세 맞춤 뒤 xy·yaw 오차 요약과 yaw 표류율."""
    m = (t >= gt.t[0]) & (t <= gt.t[-1])
    t, oxy, oyaw = t[m], oxy[m], oyaw[m]
    gx = np.interp(t, gt.t, gt.xy[:, 0])
    gy = np.interp(t, gt.t, gt.xy[:, 1])
    gyaw = np.interp(t, gt.t, np.unwrap(gt.yaw))
    dth = gyaw[0] - oyaw[0]
    c, s = math.cos(dth), math.sin(dth)
    rel = oxy - oxy[0]
    exy = np.column_stack([c * rel[:, 0] - s * rel[:, 1] + gx[0],
                           s * rel[:, 0] + c * rel[:, 1] + gy[0]])
    err = np.hypot(exy[:, 0] - gx, exy[:, 1] - gy)
    yerr = (np.unwrap(oyaw) + dth) - gyaw
    rate = float(np.polyfit(t - t[0], yerr, 1)[0]) if len(t) > 2 else math.nan
    return {'n': int(len(t)), 'err_final_cm': 100 * float(err[-1]),
            'err_max_cm': 100 * float(err.max()), 't_err_max': float(t[int(np.argmax(err))]),
            'yaw_err_max_deg': float(np.degrees(np.abs(yerr).max())),
            'yaw_drift_rate_rad_s': rate,
            'path_m': float(np.sum(np.hypot(np.diff(gx), np.diff(gy))))}


def main(argv=None):
    """명령줄."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--rec', default='/home/CAPYI_AI/runs/b24/b2-rec/rec')
    ap.add_argument('--out', required=True)
    ns = ap.parse_args(argv)
    from amr_evaluation.slam3d import se2
    here = os.path.dirname(os.path.abspath(__file__))
    os.makedirs(ns.out, exist_ok=True)
    res = {}
    for b in BAGS:
        gt = se2.tum_to_se2(os.path.join(ns.rec, b, 'gt.tum'))
        res[b] = {}
        for topic in TOPICS:
            name = topic.split('/')[-1]
            tum = os.path.join(ns.out, f'{b}_{name}.tum')
            if not os.path.isfile(tum):
                subprocess.run([sys.executable, os.path.join(here, 'bag_tools.py'), 'tum',
                                os.path.join(ns.rec, b, 'bag'), '--topic', topic, '--out', tum],
                               check=True, capture_output=True)
            od = se2.tum_to_se2(tum)
            res[b][name] = drift(od.t, od.xy, od.yaw, gt)
            print(b, name, json.dumps(res[b][name]), flush=True)
    with open(os.path.join(ns.out, 'odom_drift.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
