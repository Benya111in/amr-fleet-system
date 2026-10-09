"""
구간 주행 시간 적합: 경합 없는 P5 구간(대기 0, 복구 없음)의 Gazebo 중앙값 [측정] 에 E1 로봇 1 대 주행 시간을 맞춘다.

    python3 -m layopt.sim.legfit --raw /home/CAPYI_AI/runs/b24/b4-p5/raw --out <dir>

- Gazebo 구간: phase 사건 MOVING → PERCEIVING (출발 → 적재 도크), MOVING → DOCKING (적재 → 하역 도크),
  RETURNING → IDLE (하역 도크 → 대기 슬롯). 그 사이에 게이트 대기 구간(calib.py 정의)이 겹치면(> 0.05 s) 빼고,
  RECOVERING 이 끼면 뺀다. 구간 종류(로봇, 출발 → 도착)마다 중앙값.
- E1: 같은 구간을 로봇 1 대만 주행 (다른 로봇은 대기 슬롯, 교통 관리 · actor 켬, 시드 0–2), 도착 = 정착 끝.
- 격자: omega_curve × nav_settle_s. 기준 = 구간 종류별 (E1 중앙값 − Gazebo 중앙값) 의 RMS. 결과 표 · 선택을 낸다.
교통 지표(makespan · 대기)는 보지 않는다 — 그것은 적합 뒤 P5 재현으로 검증한다.
"""

from __future__ import annotations

import argparse
import collections
import itertools
import json
import math
import multiprocessing as mp
import pathlib
import statistics
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from layopt.sim.calib import _boxes, gazebo_run  # noqa: E402
from layopt.sim.layout import load_current  # noqa: E402
from layopt.sim.robot import IDLE, RETURN  # noqa: E402
from layopt.sim.sim import E1Sim, SimConfig, World  # noqa: E402

OMEGAS = (1.5, 1.0, 0.8, 0.6, 0.5, 0.4)
SETTLES = (0.0, 1.0, 2.0, 3.0)
SEEDS = (0, 1, 2)


def gazebo_legs(raw: pathlib.Path) -> dict:
    boxes = _boxes()
    legs = collections.defaultdict(list)
    for d in sorted(raw.glob('p5_*')):
        if not (d / 'metrics.json').is_file():
            continue
        g = gazebo_run(d, boxes)
        if g is None:
            continue
        m = json.loads((d / 'metrics.json').read_text(encoding='utf-8'))
        plan = json.loads((d / 'plan.json').read_text(encoding='utf-8'))
        tinfo = {t['task_id']: (t['pickup_dock'], t['dropoff_dock']) for t in plan['tasks']}
        task_of = {t['robot']: t['task_id'] for t in m['tasks']}
        ph = collections.defaultdict(list)
        for line in (d / 'events.jsonl').read_text(encoding='utf-8').splitlines():
            if not line.strip():
                continue
            e = json.loads(line)
            if e.get('k') == 'phase':
                ph[e['r']].append((e['t'], e['phase']))
        stops = collections.defaultdict(list)
        for r, t0, dur, _z, _src in g['stops']:
            stops[r].append((t0, t0 + dur))
        for r, seq in ph.items():
            if r not in task_of:
                continue
            pick, drop = tinfo[task_of[r]]
            cur = None
            for i, (t, p) in enumerate(seq):
                prev = seq[i - 1][1] if i else None
                if p == 'MOVING' and prev in ('IDLE', 'UNDOCKING'):
                    cur = (t, 'spawn' if prev == 'IDLE' else pick)
                elif p == 'RETURNING' and prev == 'UNDOCKING':
                    cur = (t, drop)
                elif p in ('PERCEIVING', 'DOCKING', 'IDLE') and prev in ('MOVING', 'RETURNING') \
                        and cur is not None:
                    t0, src = cur
                    cur = None
                    dst = {'PERCEIVING': pick, 'DOCKING': drop, 'IDLE': 'spawn'}[p]
                    rec = any(q == 'RECOVERING' and t0 <= tt <= t for tt, q in seq)
                    wait = sum(max(0.0, min(b, t) - max(a, t0)) for a, b in stops[r])
                    if not rec and wait <= 0.05 and src != dst:
                        legs[f'{r}:{src}->{dst}'].append(round(t - t0, 2))
                elif p in ('RECOVERING', 'ERROR'):
                    pass
    return {k: {'n': len(v), 'median': statistics.median(v), 'all': v}
            for k, v in sorted(legs.items())}


_W = None


def _leg_time(args) -> float:
    key, omega, settle, seed = args
    global _W
    if _W is None:
        _W = World(load_current())
    lay = _W.layout
    rid, route = key.split(':')
    a, b = route.split('->')
    cfg = SimConfig(workload='p5', horizon_s=400, soc=100.0, omega_curve=omega,
                    nav_settle_s=settle)
    sim = E1Sim(_W, cfg, seed)
    sim.tasks = []
    r = sim.by_id[rid]
    start = lay.slots[rid] if a == 'spawn' else lay.docks[a]
    goal = lay.slots[rid] if b == 'spawn' else lay.docks[b]
    r.x, r.y, r.th = start
    r._nav(0.0, goal, RETURN)
    dt = cfg.dt
    for k in range(int(300 / dt)):
        t = k * dt
        sim.now = t
        if sim.tm is not None and k % 5 == 2:
            sim._traffic(t)
        sim._make_bodies(k)
        for rr in sim.robots:
            rr.step(t, dt, k)
        if r.stage == IDLE:
            return (k + 1) * dt
    return math.nan


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='구간 주행 시간 적합')
    ap.add_argument('--raw', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--workers', type=int, default=6)
    a = ap.parse_args(argv)
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    g = gazebo_legs(pathlib.Path(a.raw))
    keys = sorted(g)
    jobs = [(k, om, st, s) for k in keys for om in OMEGAS for st in SETTLES for s in SEEDS]
    with mp.get_context('fork').Pool(min(a.workers, 12)) as pool:
        times = pool.map(_leg_time, jobs, chunksize=4)
    res = collections.defaultdict(list)
    for (k, om, st, s), t in zip(jobs, times):
        res[(k, om, st)].append(t)
    grid = []
    for om, st in itertools.product(OMEGAS, SETTLES):
        errs = [statistics.median(res[(k, om, st)]) - g[k]['median'] for k in keys]
        grid.append({'omega_curve': om, 'nav_settle_s': st,
                     'rms_s': round(math.sqrt(sum(e * e for e in errs) / len(errs)), 3),
                     'bias_s': round(sum(errs) / len(errs), 3),
                     'err_s': {k: round(e, 2) for k, e in zip(keys, errs)}})
    grid.sort(key=lambda r: r['rms_s'])
    doc = {'label': '[계산] (목표: [측정] Gazebo 구간 중앙값)', 'gazebo_legs': g, 'grid': grid,
           'best': {k: grid[0][k] for k in ('omega_curve', 'nav_settle_s', 'rms_s', 'bias_s')}}
    (out / 'legfit.json').write_text(json.dumps(doc, ensure_ascii=False, indent=1),
                                     encoding='utf-8')
    for r in grid[:8]:
        print(r['omega_curve'], r['nav_settle_s'], 'rms', r['rms_s'], 'bias', r['bias_s'])
    print('구간 수', len(keys), {k: g[k]['n'] for k in keys})
    return 0


if __name__ == '__main__':
    sys.exit(main())
