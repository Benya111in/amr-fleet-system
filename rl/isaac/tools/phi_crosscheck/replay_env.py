#!/usr/bin/env python3
"""
env 안 교차 시험 — record_phi_wiring.py 의 Isaac 기록을 결선 불변식으로 검사하고 C++ 원본으로 재생해 대조한다.

    rl/isaac/tools/phi_crosscheck/replay_env.sh <기록 디렉터리> [--out <json>]   (amr-ai/rl:0.3, ROS 없이)

입력은 기록의 **독립 출처**에서 명세대로 다시 만든다 (A3d 노트 2026-10-06_2129):
  - 에피소드 = env 하나의 리셋부터 다음 리셋 전까지 → 새 C++ SafetyGate (gate_runner seq 시나리오 하나)
  - 결정 k 의 스캔 = env.scan 기록 → 빔 (LIDAR_X, LIDAR_Y) + r(cos, sin) (무효 = NaN, contract scan_valid),
    시각 0.1·k, 같은 시각의 빈 점군 (점군 모델 'empty')
  - 서브스텝 j: now = 0.1·k + 0.02·j, 측정 = φ 호출 순간의 env.plant_v/w, 명령 = env._pairs 의 프로파일러 출력
불변식:
  I1 set_scan 직후 게이트 scan_stamp = 0.1·k (|Δ| ≤ 1e-9), 결정마다 set_scan 정확히 한 번
  I2 φ.step 의 now 인자 = 0.1·k + 0.02·j, 측정 인자 = env.plant_v/w, 명령 인자 = _pairs 의 v_ref·w_ref (정확)
  I3 리셋 직후 게이트 상태 = 새 게이트 (기록 쪽에서 텐서 비교)
  I4 관측의 근접 정지 비트 = 직전 결정 마지막 서브스텝의 prox (그 결정 뒤 리셋된 env 는 0)
대조: 20 채널 (crosscheck.py 와 같은 판정 — 이산 정확, 연속 |Δ| ≤ 1e-9·max(1, |a|, |b|)).
그림자(가지치기 없는 창 통계) 기록이 있으면 실제 주행에서 가지치기가 D·명령을 바꾼 비율·크기도 센다.
rc: 불일치·위반 0 이면 0, 아니면 1.
"""

import argparse
import json
import math
import os
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', '..')))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', '..', '..', 'contract')))

import crosscheck as X  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402

DT, SUB = 0.1, 0.02
N_SUB = 5


def spec_beams(r):
    """스캔 거리 [..., 720] → base_link 빔 끝점 [..., 720, 2] (명세: 원점 + r(cos, sin), 무효 = NaN)."""
    a = C.LIDAR_ANGLE_MIN + np.arange(C.LIDAR_SAMPLES) * C.LIDAR_ANGLE_INC
    ok = np.isfinite(r) & (r >= C.LIDAR_RANGE_MIN) & (r <= C.LIDAR_RANGE_MAX)
    rr = np.where(ok, r, 0.0)
    bx = np.where(ok, C.LIDAR_X + rr * np.cos(a), np.nan)
    by = np.where(ok, C.LIDAR_Y + rr * np.sin(a), np.nan)
    return np.stack([bx, by], -1)


def episodes(reset_mask, K):
    """에피소드 구간 [(env, 시작 결정, 끝 결정(포함))] — env 마다, 리셋 결정 k 는 결정 k 부터 새 에피소드."""
    out = []
    N = reset_mask.shape[1]
    for e in range(N):
        starts = sorted(int(k) for k in np.nonzero(reset_mask[:K, e])[0])
        if not starts or starts[0] != 0:
            raise RuntimeError(f'env {e}: 첫 리셋이 결정 0 이 아니다')
        bounds = starts + [K]
        for a, b in zip(bounds[:-1], bounds[1:]):
            if b > a:
                out.append((e, a, b - 1))
    return out


def invariants(R, K, N):
    """I1·I2·I4 (I3 는 기록 쪽 reset_fresh) → dict(이름: 위반 수), 예시."""
    bad, ex = {}, []
    exp_stamp = DT * np.arange(K + 1)
    calls = R['scan_calls'][:K + 1]
    bad['I1_scan_calls_not_one'] = int((calls != 1).sum())
    d = np.abs(R['scan_stamp_gate'][:K + 1] - exp_stamp[:, None])
    bad['I1_scan_stamp'] = int((~(d <= 1e-9)).sum())
    if bad['I1_scan_stamp']:
        k, e = np.argwhere(~(d <= 1e-9))[0]
        ex.append({'I1': {'k': int(k), 'env': int(e), 'gate': float(R['scan_stamp_gate'][k, e]),
                          'expected': float(exp_stamp[k])}})
    exp_now = DT * np.arange(K)[:, None] + SUB * np.arange(N_SUB)[None, :]
    bad['I2_now'] = int((np.abs(R['arg_now'][:K] - exp_now) > 1e-9).sum())
    pairs = R['pairs'][:K]                                   # [K, N, 5, 4]
    v_ref = np.transpose(pairs[..., 0], (0, 2, 1))           # [K, 5, N]
    w_ref = np.transpose(pairs[..., 1], (0, 2, 1))
    for nm, a, b in (('I2_meas_v', R['arg_v_meas'], R['plant_v']),
                     ('I2_meas_w', R['arg_w_meas'], R['plant_w']),
                     ('I2_cmd_v', R['arg_v_ref'], v_ref), ('I2_cmd_w', R['arg_w_ref'], w_ref)):
        bad[nm] = int((a[:K] != b).sum())
    out_v = np.transpose(pairs[..., 2], (0, 2, 1))
    bad['I2_pairs_out_v'] = int((out_v != R['out_v'][:K]).sum())
    bad['I3_reset_not_fresh'] = int((R['reset_mask'][:K + 1] & ~R['reset_fresh'][:K + 1]).sum())
    exp_prox = np.zeros((K + 1, N))
    exp_prox[1:] = R['out_prox'][:K, -1]
    exp_prox[R['reset_mask'][:K + 1]] = 0.0
    m = R['obs_prox'][:K + 1] != exp_prox
    bad['I4_obs_prox'] = int(m.sum())
    if m.any():
        k, e = np.argwhere(m)[0]
        ex.append({'I4': {'k': int(k), 'env': int(e), 'obs': float(R['obs_prox'][k, e]),
                          'expected': float(exp_prox[k, e])}})
    return bad, ex


def build_batch(R, eps, group_ns):
    """에피소드 묶음 → TickBatch (NS = 묶음 안 최대 결정 수, 짧은 에피소드는 플래그 0 틱으로 채운다)."""
    S = len(eps)
    T = group_ns * N_SUB
    tb = X.TickBatch(S, T, B=C.LIDAR_SAMPLES, NS=group_ns, inc=C.LIDAR_ANGLE_INC)
    for s, (e, k0, k1) in enumerate(eps):
        nk = k1 - k0 + 1
        tb.scans[s, :nk] = spec_beams(R['scan_r'][k0:k1 + 1, e])
        for i in range(group_ns):
            k = k0 + i
            for j in range(N_SUB):
                t = i * N_SUB + j
                now = DT * k + SUB * j
                tb.now[s, t] = now
                if i >= nk:
                    continue
                tb.meas[s, t] = (1.0, R['plant_v'][k, j, e], R['plant_w'][k, j, e], now)
                tb.cmd[s, t] = (1.0, R['pairs'][k, e, j, 0], R['pairs'][k, e, j, 1], now)
                if j == 0:
                    tb.scanf[s, t] = (1.0, float(i), DT * k)
                    tb.cloud_h[s, t] = (1.0, DT * k, DT * k, 0.0)
        tb.family[s] = f'env{e}_k{k0}-{k1}'
    return tb


def main():
    """검사·재생·대조."""
    ap = argparse.ArgumentParser()
    ap.add_argument('record_dir')
    ap.add_argument('--binary', required=True)
    ap.add_argument('--group', type=int, default=48, help='gate_runner 한 번에 넣을 에피소드 수')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    t0 = time.time()
    meta = json.load(open(os.path.join(a.record_dir, 'record.json')))
    R = dict(np.load(os.path.join(a.record_dir, 'wiring.npz')))
    K, N = meta['steps'], meta['num_envs']
    report = {'record': a.record_dir, 'meta': meta}
    inv, ex = invariants(R, K, N)
    report['invariants'] = inv
    report['invariant_examples'] = ex
    report['reset_state_bad_keys'] = meta.get('reset_state_bad_keys', [])
    eps = episodes(R['reset_mask'], K)
    lens = np.array([k1 - k0 + 1 for _e, k0, k1 in eps])
    order = np.argsort(lens)
    tally = X.Tally()
    cov = {'episodes': len(eps), 'ticks': 0, 'prox': 0, 'escaping': 0, 'retreat': 0,
           'command_stale': 0, 'cloud_stale': 0, 'zone': {str(z): 0 for z in range(4)},
           'causes': {str(c): 0 for c in (1, 2, 4, 8)}, 'episodes_ending_latched': 0,
           'episodes_ending_latched_then_reset': 0, 'v_changed': 0}
    tmp = tempfile.mkdtemp(dir=os.environ.get('XCHECK_TMP'))
    for g0 in range(0, len(eps), a.group):
        idx = order[g0:g0 + a.group]
        grp = [eps[i] for i in idx]
        ns = int(max(k1 - k0 + 1 for _e, k0, k1 in grp))
        tb = build_batch(R, grp, ns)
        inp, outp = os.path.join(tmp, 'seq.in'), os.path.join(tmp, 'seq.out')
        tb.pack(inp)
        X.run_cpp(a.binary, 'seq', inp, outp)
        cpp = np.fromfile(outp, dtype='<f8').reshape(tb.S, tb.T, len(X.OUT_NAMES))
        tor = np.full_like(cpp, np.nan)
        valid = np.zeros((tb.S, tb.T), bool)
        for s, (e, k0, k1) in enumerate(grp):
            nk = k1 - k0 + 1
            valid[s, :nk * N_SUB] = True
            for ci, name in enumerate(X.OUT_NAMES):
                if name == 'estop_active':
                    tor[s, :nk * N_SUB, ci] = 0.0
                else:
                    tor[s, :nk * N_SUB, ci] = R[f'out_{name}'][k0:k1 + 1, :, e].reshape(-1)
            last = cpp[s, nk * N_SUB - 1]
            cov['episodes_ending_latched'] += int(last[3] > 0)
            cov['episodes_ending_latched_then_reset'] += int(last[3] > 0 and k1 < K - 1)
        for ci, name in enumerate(X.OUT_NAMES):
            ca, tb_ = cpp[..., ci][valid], tor[..., ci][valid]
            ok = (ca == tb_) if name in X.DISCRETE else X.close(ca, tb_)
            tally.add('env', name, ok, ca, tb_)
        cv = cpp[valid]
        cov['ticks'] += int(valid.sum())
        cov['prox'] += int((cv[:, 3] > 0).sum())
        cov['escaping'] += int((cv[:, 5] > 0).sum())
        cov['retreat'] += int((cv[:, 6] > 0).sum())
        cov['cloud_stale'] += int((cv[:, 9] > 0).sum())
        cov['command_stale'] += int((cv[:, 10] > 0).sum())
        for z in range(4):
            cov['zone'][str(z)] += int((cv[:, 2] == z).sum())
        for c in (1, 2, 4, 8):
            cov['causes'][str(c)] += int(((cv[:, 4].astype(int) & c) != 0).sum())
        cov['v_changed'] += int((np.abs(cv[:, 0] - tb.cmd[..., 1][valid]) > 1e-12).sum())
    report['coverage'] = cov
    report['episode_len_decisions'] = {'min': int(lens.min()), 'median': float(np.median(lens)),
                                       'max': int(lens.max())}
    report['crosscheck'] = tally.summary()
    # 그림자: 가지치기 없는 창 통계 (같은 입력) — 실제 주행에서 가지치기가 바꾼 것
    if meta.get('shadow_unpruned'):
        ld, sd = R['out_lidar_distance'][:K], R['shadow_lidar_distance'][:K]
        fin = np.isfinite(ld) | np.isfinite(sd)
        with np.errstate(invalid='ignore'):
            diff = np.where(np.isfinite(ld) & np.isfinite(sd), ld - sd, np.nan)
        changed = fin & ~((ld == sd) | (np.isinf(ld) & np.isinf(sd)))
        dpos = diff[changed & np.isfinite(diff)]
        cmd_changed = (np.abs(R['out_v'][:K] - R['shadow_v'][:K]) > 1e-12) | \
            (np.abs(R['out_w'][:K] - R['shadow_w'][:K]) > 1e-12)
        near = fin & (np.minimum(np.where(np.isfinite(ld), ld, np.inf),
                                 np.where(np.isfinite(sd), sd, np.inf)) <= 1.05)
        report['pruning_in_driving'] = {
            'ticks': int(ld.size), 'ticks_lidar_finite': int(fin.sum()),
            'lidar_distance_changed': int(changed.sum()),
            'changed_frac_of_finite': float(changed.sum() / max(fin.sum(), 1)),
            'changed_frac_within_1p05m': float((changed & near).sum() / max(near.sum(), 1)),
            'ticks_within_1p05m': int(near.sum()),
            'pruned_minus_unpruned_m': {
                'n': int(dpos.size), 'min': float(dpos.min()) if dpos.size else None,
                'median': float(np.median(dpos)) if dpos.size else None,
                'p90': float(np.percentile(dpos, 90)) if dpos.size else None,
                'max': float(dpos.max()) if dpos.size else None},
            'one_side_inf': int((changed & ~np.isfinite(diff)).sum()),
            'command_changed': int(cmd_changed.sum()),
            'command_changed_frac': float(cmd_changed.mean()),
            'prox_differs': int((R['out_prox'][:K] != R['shadow_prox'][:K]).sum()),
            'note': '그림자는 실제 로봇을 움직이지 않는다 — 같은 순간 같은 입력에서의 차이다',
        }
    total_bad = report['crosscheck']['total_mismatches'] + sum(inv.values())
    report['total_violations'] = int(total_bad)
    report['wall_s'] = round(time.time() - t0, 1)
    print(json.dumps({k: v for k, v in report.items() if k not in ('meta',)}, indent=1,
                     ensure_ascii=False, default=float)[:20000])
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(report, f, indent=1, ensure_ascii=False, default=float)
    return 0 if total_bad == 0 and math.isfinite(total_bad) else 1


if __name__ == '__main__':
    sys.exit(main())
