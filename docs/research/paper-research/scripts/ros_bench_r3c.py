#!/usr/bin/env python3
"""
R3-C E4 (prereg_r3c.md §3) 용 ros_bench_x.py 래퍼. ros_bench_x 를 고치지 않고 불러 두 가지만 더한다.

- dwa/stats 의 [1] n_samples (주기당 평가 후보 수)·[7] v (방출 선속도) 를 실행 창 [t_acc, t_res] 에서 모아
  실행마다 n_samples_med·n_samples_mean·v_abs_med·rollout_steps_med 를 기록에 더한다.
  rollout_steps = ⌈T_sim(v)/sim_dt⌉, T_sim(v) = clip(|v|/a + 0.5, 1.5, 2.5) s, sim_dt 0.1 s, a 1.0 (배포 DWA,
  DwaPlannerX::simTime·rollout). 후보는 방출 v 의 ±aΔt_c 창 안이라 그 v 의 스텝 수로 대표한다.
  DWA 계열이 아닌 팔(MPPI·DWB·RPP)은 dwa/stats 를 내지 않으므로 −1 이다.
- 그 밖(구독·지표·출력 형식)은 ros_bench_x 와 같다. 구독 콜백만 값을 더 담는다 — 제어 루프와 무관하다.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ros_bench_x as rb  # noqa: E402

A_MAX, SIM_DT, T_MIN, T_MAX = 1.0, 0.1, 1.5, 2.5
_orig_on_stats = rb.Bench.on_stats
_orig_run_one = rb.run_one


def on_stats(self, m):
    _orig_on_stats(self, m)
    if len(m.data) > 7:
        if not hasattr(self, 'stats_ext'):
            self.stats_ext = []
        self.stats_ext.append((self.now(), float(m.data[1]), float(m.data[7])))


def steps_of(v):
    t = min(max(abs(v) / A_MAX + 0.5, T_MIN), T_MAX)
    return max(1, int(math.ceil(t / SIM_DT - 1e-9)))


def run_one(ev, entry, arm, args, proc, idle_cpu_rate):
    ev.stats_ext = []
    r = _orig_run_one(ev, entry, arm, args, proc, idle_cpu_rate)
    ext = [s for s in getattr(ev, 'stats_ext', []) if r['t_acc_abs'] <= s[0] <= r['t_res_abs']]
    if ext:
        ns = np.array([s[1] for s in ext])
        vs = np.array([s[2] for s in ext])
        r.update({'n_stats': len(ext), 'n_samples_med': float(np.median(ns)),
                  'n_samples_mean': float(np.mean(ns)), 'v_abs_med': float(np.median(np.abs(vs))),
                  'rollout_steps_med': float(np.median([steps_of(v) for v in vs]))})
    else:
        r.update({'n_stats': 0, 'n_samples_med': -1.0, 'n_samples_mean': -1.0, 'v_abs_med': -1.0,
                  'rollout_steps_med': -1.0})
    return r


rb.Bench.on_stats = on_stats
rb.run_one = run_one

if __name__ == '__main__':
    sys.exit(rb.main())
