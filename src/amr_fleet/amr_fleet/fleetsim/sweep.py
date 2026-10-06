"""
셀 × 시드 스윕 CLI — 결과를 JSONL 로 쓴다 (한 줄 = 실행 1 건).

  python3 -m amr_fleet.fleetsim.sweep --out DIR --arms A1_nearest_central,A6_cbaa_vote
      --losses 0,0.05,0.2,0.4 --faults F0,F1,F5,F6 --seeds 1-30 [--reps 0] [--workers 6]

DIR/runs.jsonl 은 (arm, loss, fault, seed, rep) 순으로 정렬해 쓴다 (병렬 완료 순서와 무관). DIR/meta.json 에는
명령 · 설정 · 시작·끝 시각 · loadavg · 실행 수 · 실패 목록을 남긴다. 실패한 실행은 errors.jsonl 에 남기고 멈추지 않는다.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import json
import multiprocessing as mp
import os
import sys
import time
import traceback
from typing import Dict, List, Tuple

from amr_fleet.fleetsim.config import ARMS, RunConfig
from amr_fleet.fleetsim.sim import run
from amr_fleet.fleetsim.world import default_world


def parse_range(text: str) -> List[int]:
    """'1-5,8' → [1, 2, 3, 4, 5, 8]."""
    out: List[int] = []
    for part in text.split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            a, b = part.split('-', 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def _job(args: Tuple[str, float, str, int, int, dict]) -> dict:
    arm, loss, fault, seed, rep, extra = args
    try:
        cfg = RunConfig(ARMS[arm], loss=loss, fault=fault, seed=seed, rep=rep, **extra)
        return {'ok': True, 'res': run(cfg, default_world())}
    except Exception:  # noqa: B902 — 실행 하나의 실패로 스윕을 멈추지 않는다
        return {'ok': False, 'job': [arm, loss, fault, seed, rep], 'err': traceback.format_exc()}


def _load() -> List[float]:
    try:
        return [round(x, 2) for x in os.getloadavg()]
    except OSError:
        return []


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--arms', required=True)
    ap.add_argument('--losses', default='0')
    ap.add_argument('--faults', default='F0')
    ap.add_argument('--seeds', default='1-30')
    ap.add_argument('--reps', default='0')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--horizon', type=float, default=None)
    ap.add_argument('--rate', type=float, default=None, help='작업 도착률 [1/s]')
    ap.add_argument('--t-fault', type=float, default=None)
    ap.add_argument('--record', action='store_true')
    a = ap.parse_args(argv)
    arms = [x for x in a.arms.split(',') if x]
    for x in arms:
        if x not in ARMS:
            ap.error(f'모르는 비교군 {x!r}: {sorted(ARMS)}')
    losses = [float(x) for x in a.losses.split(',') if x != '']
    faults = [x for x in a.faults.split(',') if x]
    seeds = parse_range(a.seeds)
    reps = parse_range(a.reps)
    extra: Dict[str, object] = {}
    if a.horizon is not None:
        extra['horizon_s'] = a.horizon
    if a.rate is not None:
        extra['arrival_rate'] = a.rate
    if a.t_fault is not None:
        extra['t_fault'] = a.t_fault
    if a.record:
        extra['record'] = True
    jobs = [(arm, loss, fault, s, r, extra) for arm in arms for loss in losses for fault in faults
            for s in seeds for r in reps]
    os.makedirs(a.out, exist_ok=True)
    base = RunConfig(ARMS[arms[0]], **extra)
    meta = {
        'argv': sys.argv if argv is None else ['sweep'] + list(argv),
        'started': dt.datetime.now().isoformat(timespec='seconds'),
        'load_start': _load(),
        'n_jobs': len(jobs), 'arms': arms, 'losses': losses, 'faults': faults,
        'seeds': [min(seeds), max(seeds), len(seeds)], 'reps': reps,
        'config': {k: v for k, v in dataclasses.asdict(base).items() if k != 'arm'},
        'pythonhashseed': os.environ.get('PYTHONHASHSEED'),
        'workers': a.workers,
    }
    t0 = time.time()
    default_world()                        # 부모에서 한 번 만들어 fork 로 나눠 쓴다
    results, errors = [], []
    ctx = mp.get_context('fork')
    with ctx.Pool(a.workers) as pool:
        for k, out in enumerate(pool.imap_unordered(_job, jobs, chunksize=4), 1):
            (results if out['ok'] else errors).append(out.get('res') or out)
            if k % 200 == 0 or k == len(jobs):
                print(f'{k}/{len(jobs)} {time.time() - t0:.0f}s load={_load()}', flush=True)
    results.sort(key=lambda r: (r['arm'], r['loss'], r['fault'], r['seed'], r['rep']))
    with open(os.path.join(a.out, 'runs.jsonl'), 'w', encoding='utf-8') as f:
        for r in results:
            f.write(json.dumps(r, ensure_ascii=False, sort_keys=True) + '\n')
    if errors:
        with open(os.path.join(a.out, 'errors.jsonl'), 'w', encoding='utf-8') as f:
            for e in errors:
                f.write(json.dumps(e, ensure_ascii=False) + '\n')
    meta.update({'ended': dt.datetime.now().isoformat(timespec='seconds'),
                 'load_end': _load(), 'wall_s': round(time.time() - t0, 1),
                 'n_ok': len(results), 'n_err': len(errors),
                 'run_wall_s_mean': round(sum(r['wall_s'] for r in results) / max(1, len(results)),
                                          3)})
    with open(os.path.join(a.out, 'meta.json'), 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    print(json.dumps({k: meta[k] for k in ('n_ok', 'n_err', 'wall_s', 'load_start', 'load_end')}))
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
