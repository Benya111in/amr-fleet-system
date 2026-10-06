#!/usr/bin/env python3
"""
전이 점검 5 — 구동 계단 응답 (gzbench_runner --probe step 의 probe_step.json → 플랜트 식별 표).

    python3 rl/eval/gazebo/analyze_step.py <probe_step.json> [--out <json>]

탐침: cmd_vel_nav 에 v 0.5 m/s 3 s → 0 → ω 1.0 rad/s 2 s → 0 (20 Hz).
하류 사슬 velocity_profiler(저크 제한 + PI) →
safety_node → cmd_vel → Gazebo DiffDrive. 두 가지를 본다.
  (1) 플랜트 = cmd_vel(safety 출력, 플랜트 입력) → 지면 진실 속도. Isaac 의 플랜트 모형(순수 지연 T_d + 1 차 지연 T_m +
      가속 한계, rl/isaac/amr_isaac/env.py)으로 격자 최소제곱 식별 →
      Isaac DR 범위(T_m 0.048–0.112 s, T_d 0.02–0.08 s)와 대조.
  (2) 사슬 전체 = cmd_vel_nav(제어기 출력) 계단 → 지면 진실: 첫 움직임 지연, 10–90 % 상승 시간, 넘침, 정상 오차, 최대 가속.
"""

import argparse
import json
import math

import numpy as np

# Gazebo DiffDrive max/min_linear_acceleration (amr_gazebo.xacro — R:59 와 emergency_deceleration)
A_LIN_UP, A_LIN_DOWN = 1.0, 3.0
A_ANG = 2.0                              # max_angular_acceleration (R:61)
DT = 0.001


def simulate(t_u, u, t_out, tm, td, a_up, a_down):
    """영차 유지 입력 u(t_u) → 지연 td → 1 차 지연 tm → 가속 한계 → t_out 에서의 출력 (Isaac 플랜트 모형)."""
    t0, t1 = t_out[0] - 0.5, t_out[-1]
    ts = np.arange(t0, t1 + DT, DT)
    idx = np.searchsorted(t_u, ts - td, side='right') - 1
    uu = np.where(idx >= 0, np.asarray(u)[np.clip(idx, 0, None)], 0.0)
    y = np.zeros_like(ts)
    alpha = 1.0 - math.exp(-DT / tm)
    for k in range(1, ts.size):
        dv = alpha * (uu[k] - y[k - 1])
        y[k] = y[k - 1] + max(-a_down * DT, min(a_up * DT, dv))
    return np.interp(t_out, ts, y)


def fit(t_u, u, t_y, y, a_up, a_down):
    """격자 최소제곱 (T_m, T_d) → (최적, RMSE, 최소 RMSE × 1.1 안의 범위)."""
    grid_m = np.arange(0.01, 0.30 + 1e-9, 0.005)
    grid_d = np.arange(0.0, 0.20 + 1e-9, 0.005)
    best, surf = None, []
    for tm in grid_m:
        for td in grid_d:
            e = simulate(t_u, u, t_y, tm, td, a_up, a_down) - y
            rmse = float(np.sqrt(np.mean(e * e)))
            surf.append((rmse, tm, td))
            if best is None or rmse < best[0]:
                best = (rmse, tm, td)
    near = [s for s in surf if s[0] <= 1.1 * best[0]]
    return {'T_m_s': round(best[1], 4), 'T_d_s': round(best[2], 4), 'rmse': round(best[0], 5),
            'T_m_range_within_10pct': [round(min(s[1] for s in near), 4),
                                       round(max(s[1] for s in near), 4)],
            'T_d_range_within_10pct': [round(min(s[2] for s in near), 4),
                                       round(max(s[2] for s in near), 4)]}


def chain(t_cmd, cmd, t_y, y, t_on, level):
    """사슬 전체: 계단 시각 t_on, 목표 level → 지연·상승·넘침·정상 오차."""
    m = t_y >= t_on
    ty, yy = t_y[m] - t_on, y[m]
    moving = np.abs(yy) > 0.02 * abs(level) + 0.005
    first = ty[np.argmax(moving)] if np.any(moving) else None
    r10 = ty[np.argmax(yy >= 0.1 * level)] if np.any(yy >= 0.1 * level) else None
    r90 = ty[np.argmax(yy >= 0.9 * level)] if np.any(yy >= 0.9 * level) else None
    win = (ty > ty[-1] - 0.5)
    return {'first_motion_s': None if first is None else round(float(first), 4),
            'rise_10_90_s': None if (r10 is None or r90 is None) else round(float(r90 - r10), 4),
            't90_s': None if r90 is None else round(float(r90), 4),
            'overshoot': round(float(yy.max() / level - 1.0), 4),
            'steady_err': round(float(np.mean(yy[win]) - level), 4)}


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('probe')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    d = json.load(open(a.probe, encoding='utf-8'))
    gt = np.array(d['gt'], dtype=float)
    co = np.array(d['cmd_out'], dtype=float)
    cn = np.array(d['cmd_nav'], dtype=float)
    ph = d['phases']                        # [[t, v, w], …] 0 / v 0.5 / 0 / ω 1.0 / 0
    t_v_on, t_w_on, t_end = ph[1][0], ph[3][0], ph[4][0] + 2.5
    out = {'phases': ph, 'n_gt': int(gt.shape[0]), 'n_cmd_out': int(co.shape[0])}
    # (1) 플랜트 식별: 선속도 (v 계단 ~ 그 뒤 정지까지), 각속도
    lin = (gt[:, 0] >= t_v_on - 0.2) & (gt[:, 0] <= t_w_on - 0.1)
    out['plant_linear'] = fit(co[:, 0], co[:, 1], gt[lin, 0], gt[lin, 4], A_LIN_UP, A_LIN_DOWN)
    ang = (gt[:, 0] >= t_w_on - 0.2) & (gt[:, 0] <= t_end)
    out['plant_angular'] = fit(co[:, 0], co[:, 2], gt[ang, 0], gt[ang, 5], A_ANG, A_ANG)
    # (2) 사슬 전체
    out['chain_linear'] = chain(cn[:, 0], cn[:, 2], gt[:, 0], gt[:, 4], t_v_on, 0.5)
    out['chain_angular'] = chain(cn[:, 0], cn[:, 3], gt[:, 0], gt[:, 5], t_w_on, 1.0)
    # 최대 가속 (지면 진실 속도의 50 ms 차분)
    for key, col, t0, t1 in (('linear', 4, t_v_on, t_w_on), ('angular', 5, t_w_on, t_end)):
        m = (gt[:, 0] >= t0) & (gt[:, 0] <= t1)
        tt, vv = gt[m, 0], gt[m, col]
        if tt.size > 10:
            k = max(1, int(round(0.05 / np.median(np.diff(tt)))))
            acc = (vv[k:] - vv[:-k]) / (tt[k:] - tt[:-k])
            out[f'max_accel_{key}'] = [round(float(acc.max()), 3), round(float(acc.min()), 3)]
    # safety 출력 대 프로파일러 출력 (개입이 없었는지)
    ci = np.array(d['cmd_in'], dtype=float)
    if ci.size and co.size:
        k = np.searchsorted(ci[:, 0], co[:, 0], side='right') - 1
        ok = k >= 0
        out['safety_max_abs_diff'] = [round(float(np.max(np.abs(co[ok, 1] - ci[k[ok], 1]))), 4),
                                      round(float(np.max(np.abs(co[ok, 2] - ci[k[ok], 2]))), 4)]
    out['isaac_dr'] = {'T_m_s': [0.048, 0.112], 'T_d_s': [0.02, 0.08], 'accel_scale': [0.84, 1.0],
                       'nominal': {'T_m_s': 0.08, 'T_d_s': 0.04}}
    for key in ('plant_linear', 'plant_angular'):
        f = out[key]
        f['in_isaac_dr'] = bool(0.048 <= f['T_m_s'] <= 0.112 and 0.02 <= f['T_d_s'] <= 0.08)
    txt = json.dumps(out, ensure_ascii=False, indent=1)
    print(txt)
    if a.out:
        open(a.out, 'w', encoding='utf-8').write(txt)


if __name__ == '__main__':
    main()
