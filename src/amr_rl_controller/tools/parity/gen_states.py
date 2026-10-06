#!/usr/bin/env python3
"""
출력 일치(보너스 1 A2) 입력 묶음 — golden decision 상태와 시드 고정 무작위 계약 상태·관측.

    RL_CONTRACT_DIR=<저장소>/rl/contract python3 gen_states.py --out DIR [--n 4096] [--seed 2026]

출력 (DIR):
  golden_states.jsonl · golden_obs_ref.npy   golden d01–d17 의 상태, numpy 기준 구현 관측 [17, 291]
  random_states.jsonl · random_obs_ref.npy   무작위 계약 상태 n 개와 기준 관측 [n, 291]
  normal_obs.npy                             표준정규 관측 [n, 291] (계약 범위 밖·클립 밖 포함)
  integrate_in.f64 · integrate_ref.npy       integrate_b 입력 [m, 5] (u0, u1, v_cmd, w_cmd, v_cap)
                                             과 기준 출력 [m, 4] (m = 100000)
  meta.json                                  시드·개수·생성 규칙 요약
무작위 상태는 경로(곡선·짧은 경로 포함)·스캔(무효 빔·풋프린트 안 점 포함)·속도(클립 밖 포함)·
트랙(0–12 개, 외삽 나이)·safety·이탈 상태를 섞는다. 경계 동률(트랙 선별 문턱 등)은 연속 난수라 사실상
나오지 않는다 — 나오면 parity_check 가 사례를 그대로 보고한다.
"""

import argparse
import glob
import json
import math
import os
import sys

import numpy as np

CONTRACT = os.environ.get('RL_CONTRACT_DIR', os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', 'rl', 'contract')))
sys.path.insert(0, CONTRACT)

from amr_rl_contract import constants as C  # noqa: E402
from amr_rl_contract import kinematics as K  # noqa: E402
from amr_rl_contract.observation import build_observation  # noqa: E402
from amr_rl_contract.state import DevState, SafetyState, State, Track  # noqa: E402


def _path(rng):
    """무작위 경로 (0.05 m 간격, 끝 = 목표). 길이 0.1–30 m, 곡률 변화, 드물게 점 2 개."""
    if rng.random() < 0.03:
        x0, y0 = rng.uniform(-20, 20, 2)
        a = rng.uniform(-math.pi, math.pi)
        return [(x0, y0), (x0 + 0.05 * math.cos(a), y0 + 0.05 * math.sin(a))]
    length = float(rng.choice([rng.uniform(0.1, 3.0), rng.uniform(3.0, 15.0),
                               rng.uniform(15.0, 30.0)]))
    n = max(2, int(round(length / 0.05)) + 1)
    x, y = rng.uniform(-20, 20, 2)
    a = rng.uniform(-math.pi, math.pi)
    kappa = 0.0
    pts = [(float(x), float(y))]
    for _ in range(n - 1):
        if rng.random() < 0.02:
            kappa = float(rng.choice([0.0, rng.uniform(-1.0, 1.0)]))
        a += kappa * 0.05
        x += 0.05 * math.cos(a)
        y += 0.05 * math.sin(a)
        pts.append((float(x), float(y)))
    return pts


def _pose_near(rng, path):
    """경로 위 한 점 근처의 자세 (횡 오차·방향 오차 무작위)."""
    i = int(rng.integers(0, len(path)))
    j = min(i + 1, len(path) - 1)
    k = max(j - 1, 0)
    a = math.atan2(path[j][1] - path[k][1], path[j][0] - path[k][0])
    off = rng.normal(0.0, 0.5) if rng.random() < 0.8 else rng.uniform(-3.0, 3.0)
    x = path[i][0] - math.sin(a) * off
    y = path[i][1] + math.cos(a) * off
    th = a + (rng.normal(0.0, 0.3) if rng.random() < 0.7 else rng.uniform(-math.pi, math.pi))
    return (float(x), float(y), float(math.remainder(th, 2 * math.pi)))


def _scan(rng):
    """720 빔. 방식 넷: 균일 난수(무효 섞음), 조각 상수 벽, 풋프린트 근접점, 거의 빈 공간."""
    mode = int(rng.integers(0, 4))
    n = C.LIDAR_SAMPLES
    if mode == 0:
        r = rng.uniform(0.0, 30.0, n)
    elif mode == 1:
        cuts = np.sort(rng.integers(0, n, int(rng.integers(2, 12))))
        r = np.empty(n)
        lo = 0
        for c in list(cuts) + [n]:
            r[lo:c] = rng.uniform(0.3, 12.0)
            lo = c
        r += rng.normal(0.0, 0.03, n)
    elif mode == 2:
        r = rng.uniform(1.0, 8.0, n)
        k = int(rng.integers(0, n))
        w = int(rng.integers(5, 120))
        idx = (np.arange(w) + k) % n
        r[idx] = rng.uniform(0.05, 0.7, w)
    else:
        r = np.full(n, np.inf)
        idx = rng.integers(0, n, int(rng.integers(0, 30)))
        r[idx] = rng.uniform(0.2, 20.0, len(idx))
    out = []
    for v in r:
        u = rng.random()
        if u < 0.05:
            out.append(None)          # null = NaN 빔
        elif u < 0.06:
            out.append(float(rng.choice([0.10, 25.0, 0.0999, 25.0001])))
        elif not math.isfinite(v):
            out.append(None)
        else:
            out.append(float(v))
    return out


def _tracks(rng, pose):
    out = []
    ids = rng.choice(1000, size=int(rng.integers(0, 13)), replace=False)
    for tid in ids:
        d = rng.uniform(0.3, 12.0)
        b = rng.uniform(-math.pi, math.pi)
        sp = float(rng.choice([rng.uniform(0.0, 0.5), rng.uniform(0.5, 1.6)]))
        h = rng.uniform(-math.pi, math.pi)
        out.append(Track(int(tid), float(pose[0] + d * math.cos(b)),
                         float(pose[1] + d * math.sin(b)), float(sp * math.cos(h)),
                         float(sp * math.sin(h)), bool(rng.random() < 0.6)))
    return out


def random_state(rng):
    """무작위 계약 상태 하나."""
    path = _path(rng)
    pose = _pose_near(rng, path)
    big = 10.0 if rng.random() < 0.03 else 1.0     # 드물게 클립 밖 (관측 [−3, 3] 자름 확인)
    s = State(pose=pose, path=path, scan_ranges=_scan(rng))
    s.v_meas = float(rng.uniform(-0.3, 2.2) * big)
    s.w_meas = float(rng.uniform(-2.0, 2.0) * big)
    s.a_v_meas = float(rng.uniform(-3.0, 3.0) * big)
    s.a_w_meas = float(rng.uniform(-6.0, 6.0) * big)
    s.v_cmd = float(rng.uniform(0.0, 2.0))
    s.w_cmd = float(rng.uniform(-1.5, 1.5))
    s.prev_actions = [(float(rng.uniform(-1.0, 1.0)), float(rng.uniform(-2.0, 2.0)))
                      for _ in range(C.N_PREV_ACTIONS)]
    s.v_cap = float(rng.choice([0.3, 0.5, 1.0, 2.0, 2.5, rng.uniform(0.0, 2.5)]))
    s.scan_age = float(rng.uniform(0.0, 0.5))
    s.track_age = float(rng.uniform(0.0, 0.8))
    s.tracks = _tracks(rng, pose)
    s.safety = SafetyState(int(rng.integers(0, 4)),
                           None if rng.random() < 0.5 else float(rng.uniform(0.0, 1.2)),
                           bool(rng.random() < 0.2),
                           None if rng.random() < 0.5 else float(rng.uniform(0.0, 6.0)))
    if rng.random() < 0.3:
        s.dev = DevState(True, float(rng.uniform(0.3, 2.5)), float(rng.uniform(0.0, 12.0)))
    return s


def main():
    """입력 묶음을 만든다."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--n', type=int, default=4096)
    ap.add_argument('--seed', type=int, default=2026)
    ap.add_argument('--n-integrate', type=int, default=100000)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rng = np.random.default_rng(a.seed)

    gold = sorted(glob.glob(os.path.join(CONTRACT, 'golden', 'd*.json')))
    with open(os.path.join(a.out, 'golden_states.jsonl'), 'w') as f:
        obs = []
        for p in gold:
            j = json.load(open(p))
            s = State.from_dict(j['state'])
            f.write(json.dumps(s.to_dict()) + '\n')
            obs.append(build_observation(s))
    np.save(os.path.join(a.out, 'golden_obs_ref.npy'), np.asarray(obs, dtype=np.float32))

    with open(os.path.join(a.out, 'random_states.jsonl'), 'w') as f:
        obs = []
        for _ in range(a.n):
            s = random_state(rng)
            f.write(json.dumps(s.to_dict()) + '\n')
            obs.append(build_observation(s))
    np.save(os.path.join(a.out, 'random_obs_ref.npy'), np.asarray(obs, dtype=np.float32))
    np.save(os.path.join(a.out, 'normal_obs.npy'),
            rng.standard_normal((a.n, len(obs[0]))).astype(np.float32))

    m = a.n_integrate
    caps = rng.choice([0.0, 0.3, 1.0, 2.5, -1.0], size=m)
    caps = np.where(caps < 0, rng.uniform(0.0, 2.5, m), caps)
    rows = np.stack([rng.uniform(-1.5, 1.5, m), rng.uniform(-1.5, 1.5, m),
                     rng.uniform(-0.2, 2.2, m), rng.uniform(-1.8, 1.8, m), caps], axis=1)
    rows.astype(np.float64).tofile(os.path.join(a.out, 'integrate_in.f64'))
    ref = np.array([K.integrate_b((r[0], r[1]), r[2], r[3], r[4]) for r in rows])
    np.save(os.path.join(a.out, 'integrate_ref.npy'), ref)
    meta = {'seed': a.seed, 'n_random': a.n, 'n_golden': len(gold), 'n_integrate': m,
            'contract_dir': CONTRACT, 'generator': os.path.abspath(__file__)}
    json.dump(meta, open(os.path.join(a.out, 'meta.json'), 'w'), indent=1)
    print(json.dumps(meta))


if __name__ == '__main__':
    main()
