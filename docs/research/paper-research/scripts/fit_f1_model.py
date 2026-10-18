#!/usr/bin/env python3
"""
F1 사슬 모형의 ROS 보정 (prereg_r3.md §1): model_delay d · model_tau τ 를 **튜닝-20 의 2 라운드 ROS 궤적**에
최소제곱으로 맞춘다. held-out C 궤적은 쓰지 않는다.

    python3 fit_f1_model.py --traces /home/CAPYI_AI/paper_research/runs/r2/tuning --out <dir>
    python3 fit_f1_model.py --check-chain /home/CAPYI_AI/paper_research/runs/r2/chain   # 이식 확인 (chain 궤적)

모형 (pr-core `src/amr_nav_research/src/chain_model.cpp` 의 선속도 부분을 그대로 옮김):
- 제어기 명령 v_cmd 를 한 제어 주기(0.05 s = 서보 5 스텝) 동안 유지한다.
- 프로파일러 기준: 전역 서보 스텝 번호가 짝수일 때 core::JerkLimitedFilter (a 1.0, j 2.0, dt 0.02) 한 스텝.
  (amr_navigation `jerk_limiter.cpp` 의 step 식 그대로: 종단 스냅, a* = sgn(c)·j·(−dt/2 + √(dt²/4 + 2|c|/j)), 저크·가속 클램프)
- 서보 (100 Hz): 순수 지연 round(d/0.01) 스텝 → 1차 α = 1 − exp(−0.01/τ) → 가속 제한 1.0 m/s² (스텝당 0.01).
- 시작: 정지(0) 에서 초기화 (실행마다 정지 출발).

맞춤:
- 자료: 튜닝-20 경로 실행 가운데 잡음 없음(seed 0) 실행 전부 (모든 격자 팔 — 하류 사슬은 팔과 무관). O 장면 실행은 뺀다.
- 표본: A2 궤적 20 Hz 격자. 격자 k 의 명령 v_cmd[k] 로 0.05 s 전진한 예상 v̂[k+1] 을 측정 v_meas[k+1] (odometry/filtered)
  과 비교한다. 수락(t = 0)부터 판정 통과(t_reach)까지.
- 목적: 잔차 제곱합 최소. d ∈ {0.00, 0.01, …, 0.30} (C++ 가 10 ms 단위로 반올림하므로 같은 눈금), τ ∈ {0.01, …, 0.40}
  (0.01 간격) 격자 전수 탐색. 짝·홀 실행으로 나눈 반쪽 맞춤도 낸다 (안정성).
- 출력: d*·τ*, 잔차 RMS (전체·출발 2 s·그 뒤), 명목값 (0.04, 0.08) 의 잔차, 시각별 평균 잔차.
"""
import argparse
import csv
import glob
import json
import math
import os

import numpy as np

SERVO_DT = 0.01
STEPS = 5          # 20 Hz 제어 주기당 서보 스텝
A_MAX, J_MAX = 1.0, 2.0
SERVO_A = 1.0
DP = 0.02          # 프로파일러 dt (서보 2 스텝)


def jerk_step(v, a, v_ref, dt=DP, a_max=A_MAX, j=J_MAX):
    """core::JerkLimitedFilter::step 의 벡터판 (v, a, v_ref 는 배열)."""
    snap = (np.abs(v_ref - v) <= j * dt * dt) & (np.abs(a) <= j * dt)
    c = v_ref - v - 0.5 * a * dt
    a_star = np.sign(np.where(c >= 0, 1.0, -1.0)) * j * (-0.5 * dt + np.sqrt(0.25 * dt * dt +
                                                                           2.0 * np.abs(c) / j))
    jerk = np.clip((a_star - a) / dt, -j, j)
    a1 = np.clip(a + jerk * dt, -a_max, a_max)
    a1 = np.clip(a1, a - j * dt, a + j * dt)
    v_new = v + 0.5 * (a + a1) * dt
    v_out = np.where(snap, v_ref, v_new)
    a_out = np.where(snap, 0.0, a1)
    return v_out, a_out


def profiler_outputs(cmds):
    """cmds: (R, N) 격자 명령 → (R, N·5) 서보 스텝마다의 프로파일러 출력 (갱신 사이 유지)."""
    R, N = cmds.shape
    v = np.zeros(R)
    a = np.zeros(R)
    out = np.zeros(R)
    res = np.zeros((R, N * STEPS))
    step = 0
    for k in range(N):
        for _ in range(STEPS):
            if step % 2 == 0:
                v, a = jerk_step(v, a, cmds[:, k])
                out = v
            res[:, step] = out
            step += 1
    return res


def servo(prof, d, tau):
    """prof (R, M) → 서보 출력 (R, M). 순수 지연 round(d/0.01) → 1차 → 가속 제한."""
    R, M = prof.shape
    n = int(round(d / SERVO_DT))
    alpha = 1.0 - math.exp(-SERVO_DT / tau) if tau > 0 else 1.0
    lim = SERVO_A * SERVO_DT
    x = np.zeros(R)
    out = np.zeros((R, M))
    for m in range(M):
        u = prof[:, m - n] if m - n >= 0 else np.zeros(R)
        x = x + np.clip(alpha * (u - x), -lim, lim)
        out[:, m] = x
    return out


def load_ros(root, max_t=60.0):
    """튜닝 원자료 + 궤적 → (cmds (R, N), meas (R, N), mask (R, N), t (N), 정보)."""
    runs = []
    for fn in sorted(glob.glob(os.path.join(root, 'raw', '*.jsonl'))):
        jid = os.path.basename(fn)[:-6]
        with open(fn, encoding='utf-8') as f:
            for line in f:
                r = json.loads(line)
                if r['family'] == 'obstacle' or int(r.get('seed', 0)) != 0:
                    continue
                tr = os.path.join(root, 'traces', jid,
                                  f"{r['arm']}__{r['scene']}__s{r['seed']}__r{r['rep']}.csv")
                if not os.path.exists(tr):
                    continue
                t_end = r['t_reach'] if r['success'] else r['t_end']
                rows = []
                with open(tr, encoding='utf-8') as tf:
                    for x in csv.DictReader(tf):
                        t = float(x['t'])
                        if t < -1e-9 or t > min(t_end, max_t) + 1e-9:
                            continue
                        rows.append((t, float(x['v_cmd']) if x['v_cmd'] else 0.0,
                                     float(x['v_meas']) if x['v_meas'] else float('nan')))
                if len(rows) > 10:
                    runs.append((r['arm'], r['scene'], rows))
    N = max(len(x[2]) for x in runs)
    R = len(runs)
    cmds = np.zeros((R, N))
    meas = np.full((R, N), np.nan)
    for i, (_, _, rows) in enumerate(runs):
        n = len(rows)
        cmds[i, :n] = [x[1] for x in rows]
        cmds[i, n:] = rows[-1][1]
        meas[i, :n] = [x[2] for x in rows]
    t = np.arange(N) * 0.05
    return cmds, meas, t, [(a, s) for a, s, _ in runs]


def residuals(prof, meas, d, tau):
    """예상 v̂[k+1] (명령 k 로 전진한 뒤) − 측정 v_meas[k+1]."""
    sv = servo(prof, d, tau)
    vhat = sv[:, STEPS - 1::STEPS]           # 격자 k 의 명령으로 5 스텝 전진한 뒤 = 격자 k+1 의 예상
    pred = vhat[:, :-1]
    obs = meas[:, 1:]
    return pred - obs


def fit(prof, meas, sel, d_grid, tau_grid):
    best = (float('inf'), None, None)
    table = {}
    for d in d_grid:
        for tau in tau_grid:
            res = residuals(prof[sel], meas[sel], d, tau)
            m = np.isfinite(res)
            sse = float(np.sum(res[m] ** 2))
            table[(round(d, 3), round(tau, 3))] = sse
            if sse < best[0]:
                best = (sse, d, tau)
    return best, table


def summary(prof, meas, t, d, tau):
    res = residuals(prof, meas, d, tau)
    m = np.isfinite(res)
    tt = np.broadcast_to(t[1:], res.shape)
    early = m & (tt <= 2.0)
    late = m & (tt > 2.0)
    bins = {}
    for lo in np.arange(0.0, 3.0, 0.2):
        mm = m & (tt > lo) & (tt <= lo + 0.2)
        if mm.any():
            bins[f'{lo:.1f}-{lo + 0.2:.1f}'] = float(np.mean(res[mm]))
    return {'d': d, 'tau': tau, 'n': int(m.sum()),
            'rms_all': float(np.sqrt(np.mean(res[m] ** 2))),
            'rms_first2s': float(np.sqrt(np.mean(res[early] ** 2))),
            'rms_after2s': float(np.sqrt(np.mean(res[late] ** 2))),
            'mean_resid_first2s': float(np.mean(res[early])),
            'mean_resid_by_time': bins}


def check_chain(root):
    """이식 확인: chain_sim_x 궤적(50 Hz, v = 서보, v_cmd = 명령) 에 같은 모형을 대면 명목값에서 잔차가 거의 0 이어야 한다."""
    errs = []
    for fn in sorted(glob.glob(os.path.join(root, 'traces', 'dwa_R__smooth__clean__st0', '*.csv')))[:10]:
        rows = list(csv.DictReader(open(fn, encoding='utf-8')))
        # chain 궤적은 0.02 s 간격, 제어 주기 0.05 s → t = 0.05 k 에 가까운 행을 뽑는다
        t = np.array([float(r['t']) for r in rows])
        vc = np.array([float(r['v_cmd']) for r in rows])
        v = np.array([float(r['v']) for r in rows])
        grid = np.arange(0.0, t[-1] - 0.1, 0.05)
        idx = np.searchsorted(t, grid + 0.011)   # 각 주기 시작 직후 명령
        cmds = vc[np.clip(idx, 0, len(vc) - 1)][None, :]
        meas = np.interp(grid, t, v)[None, :]
        prof = profiler_outputs(cmds)
        res = residuals(prof, meas, 0.04, 0.08)
        errs.append(float(np.sqrt(np.nanmean(res ** 2))))
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--traces', default='')
    ap.add_argument('--out', default='')
    ap.add_argument('--check-chain', default='')
    args = ap.parse_args()
    if args.check_chain:
        print(json.dumps({'chain_rms_nominal': check_chain(args.check_chain)}, indent=1))
        return 0
    cmds, meas, t, info = load_ros(args.traces)
    prof = profiler_outputs(cmds)
    d_grid = [round(0.01 * i, 2) for i in range(0, 31)]
    tau_grid = [round(0.01 * i, 2) for i in range(1, 41)]
    R = len(info)
    allr = np.arange(R)
    (sse, d, tau), table = fit(prof, meas, allr, d_grid, tau_grid)
    (_, d_e, tau_e), _ = fit(prof, meas, allr[0::2], d_grid, tau_grid)
    (_, d_o, tau_o), _ = fit(prof, meas, allr[1::2], d_grid, tau_grid)
    res = {'data': {'root': args.traces, 'runs': R, 'arms': sorted({a for a, _ in info}),
                    'note': '튜닝-20 경로, 잡음 없음(seed 0), O 제외'},
           'best': summary(prof, meas, t, d, tau), 'sse_best': sse,
           'nominal': summary(prof, meas, t, 0.04, 0.08),
           'split_half': {'even': [d_e, tau_e], 'odd': [d_o, tau_o]},
           'grid': {'d': [d_grid[0], d_grid[-1], 0.01], 'tau': [tau_grid[0], tau_grid[-1], 0.01]},
           'sse_near_best': {f'{k[0]},{k[1]}': v for k, v in sorted(table.items(), key=lambda kv: kv[1])[:10]}}
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        with open(os.path.join(args.out, 'f1_fit_r3.json'), 'w', encoding='utf-8') as f:
            json.dump(res, f, indent=1, ensure_ascii=False)
    print(json.dumps(res, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
