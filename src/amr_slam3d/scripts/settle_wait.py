#!/usr/bin/env python3
"""
녹화 전 정착 대기 (b2-rerec, 재녹화 결과 전) — 로봇이 스폰 뒤 정지했는지 원시 IMU 로 확인하고 끝난다.

왜: 녹화 3/8 bag 에서 imu_filter_node 가 기동 직후(첫 판정 표본 20 개, sim 0.1 s) "motion detected during
startup bias estimation; falling back to parameter bias" 를 냈다 → 바이어스 0 → bag 의 EKF odom 이
0.01 rad/s 로 yaw 표류 (bonus2_results.md §0-2). bringup 은 스폰 확인 뒤 월드를 재개하자마자
위치추정 스택을 띄우므로, 스폰(z 0.02 m)에서 바닥에 닿아 자리 잡는 동안의 표본을 imu_filter 가 받을 수
있다 [추정 — 파라미터 바이어스로 떨어진 4 시도가 모두 RTF 0.25, RTF 0.5 시도 6 회는 0].
그래서 record_bag.sh 는 위치추정 스택을 뺀 채 월드·로봇을 띄우고, 이 노드가 0 으로 끝난 뒤에 localization.launch.py 를
(bringup 과 같은 인자로) 띄운다.

    settle_wait.py --robot amr_01 --out settle.json [--min-sim 3.0] [--window 2.0] [--timeout 900]

판정: 마지막 IMU 스탬프(sim) ≥ --min-sim 이고, 마지막 --window s(sim) 의 imu/data_raw 표본이 기대 수(--rate × window)의
80 % 이상이며 recgate.imu_still (자이로 σ ≤ 0.005 rad/s, 가속도 σ ≤ 0.1 m/s², |‖ā‖ − 9.8| ≤ 0.25 m/s²
— imu_filter 정지 판정의 절반 문턱) 을 만족하면 0. --timeout (벽시계 s) 안에 못 만나면 1. 결과·마지막 판정은 --out JSON 에.
use_sim_time 은 쓰지 않는다 — sim 시각은 IMU header 스탬프로 본다 (/clock 구독 없음).
"""

import argparse
import collections
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from amr_slam3d import recgate  # noqa: E402


def main(argv=None) -> int:
    """명령줄."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--robot', default='amr_01')
    ap.add_argument('--out', required=True)
    ap.add_argument('--min-sim', type=float, default=3.0, help='[s, sim] 이 시각 전에는 판정하지 않는다')
    ap.add_argument('--window', type=float, default=2.0, help='[s, sim] 판정 창')
    ap.add_argument('--rate', type=float, default=200.0, help='[Hz] imu/data_raw 명목 주기')
    ap.add_argument('--timeout', type=float, default=900.0, help='[s, 벽시계]')
    ns = ap.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = Node('b2rerec_settle_wait')
    buf = collections.deque()
    state = {'n_total': 0, 'first_sim': None}

    def on_imu(m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        av, la = m.angular_velocity, m.linear_acceleration
        buf.append((st, (av.x, av.y, av.z), (la.x, la.y, la.z)))
        state['n_total'] += 1
        if state['first_sim'] is None:
            state['first_sim'] = st
        while buf and buf[0][0] < st - ns.window:
            buf.popleft()

    node.create_subscription(Imu, f'/{ns.robot}/imu/data_raw', on_imu, qos_profile_sensor_data)
    t0 = time.monotonic()
    last = {}
    rc = 1
    checks = 0
    while time.monotonic() - t0 < ns.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not buf:
            continue
        sim_now = buf[-1][0]
        if sim_now < ns.min_sim:
            continue
        checks += 1
        need = int(0.8 * ns.window * ns.rate)
        win = list(buf)
        span = win[-1][0] - win[0][0]
        res = recgate.imu_still([w[1] for w in win], [w[2] for w in win])
        res.update({'sim_now': sim_now, 'window_span_s': span, 'need': need})
        if len(win) < need:
            res['ok'] = False
            res['why'] = f'창 표본 {len(win)} < {need}'
        last = res
        if res['ok']:
            rc = 0
            break
    out = {'ok': rc == 0, 'wall_s': round(time.monotonic() - t0, 2), 'checks': checks,
           'n_total': state['n_total'], 'first_imu_sim': state['first_sim'], 'last': last,
           'rule': {'min_sim_s': ns.min_sim, 'window_s': ns.window, 'rate_hz': ns.rate,
                    'gyro_std_max': recgate.SETTLE_GYRO_STD,
                    'accel_std_max': recgate.SETTLE_ACCEL_STD,
                    'norm_tol': recgate.SETTLE_NORM_TOL, 'g': recgate.SETTLE_G}}
    with open(ns.out, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=1, default=float)
    print(json.dumps(out, ensure_ascii=False, default=float))
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
