"""
E1 스윕: 묶음 × 시드 × 부하 → results.jsonl (contract §4) + extras.jsonl (진단).

    python3 -m layopt.sim.sweep --bundles '/home/CAPYI_AI/runs/layopt/bundles/*' --seeds 0:9 \
        --loads 1.0 --out /home/CAPYI_AI/runs/layopt/e1/<UTC>_<tag> --workers 12

- --seeds A:B 는 양 끝 포함 (0:9 = 10 개). --bundles 는 glob(쉼표로 여럿) 또는 'current'(지금 저장소 파일).
- 이어 하기: results.jsonl 에 이미 있는 (spec_id, seed, load) 는 건너뛴다. 모든 출력은 덧붙이기만 하고 행마다 flush.
  실패한 실행은 errors.jsonl 에 남기고(다음 이어 하기에서 다시 시도) 계속한다.
- manifest.jsonl: 호출마다 start · end 기록 (커밋, 인자, 시각, loadavg, 건수, 실행당 벽시계).
- registry.json: 끝까지 돈 뒤 한 번 (notebook register 입력). 이미 있으면 덮어쓰지 않는다.
- 작업은 묶음끼리 모아(--chunk 개씩) 한 일꾼이 같은 배치를 이어 돌게 한다 (지도 · 경로 기억 재사용).
- valid.json 이 거짓인 묶음은 건너뛰고 manifest 에 적는다 (--include-invalid 로 포함).
커밋 · 미커밋 여부는 컨테이너 안에서 git 을 못 보므로 환경변수 LAYOPT_COMMIT · LAYOPT_DIRTY 로 받는다
(layopt/sim/run_sweep.sh 가 넣는다).
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import multiprocessing as mp
import os
import pathlib
import sys
import time
import traceback
from typing import Dict, List, Optional, Tuple

ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from layopt.sim.layout import load_bundle, load_current  # noqa: E402
from layopt.sim.sim import BASE_RATE, SimConfig, World, run  # noqa: E402

_LOCK = None
_OUT: Optional[pathlib.Path] = None
_WORLDS: Dict[str, World] = {}
_WORLD_KEEP = 2


def _now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec='seconds')


def _load() -> str:
    try:
        return ' '.join(f'{v:.2f}' for v in os.getloadavg())
    except OSError:
        return 'n/a'


def parse_seeds(text: str) -> List[int]:
    """'A:B' (양 끝 포함) · 'A,B,C' · 'A'."""
    out: List[int] = []
    for part in text.split(','):
        part = part.strip()
        if ':' in part:
            a, b = part.split(':')
            out.extend(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def find_bundles(spec: str) -> List[str]:
    """쉼표로 나눈 glob 들 ('current' = 지금 저장소 파일) → 묶음 경로 목록 (중복 제거, 순서 유지)."""
    out: List[str] = []
    for pat in spec.split(','):
        pat = pat.strip()
        if pat == 'current':
            cands = ['current']
        else:
            cands = [p for p in sorted(glob.glob(pat))
                     if os.path.isfile(os.path.join(p, 'layout.json'))]
        out.extend(c for c in cands if c not in out)
    return out


def bundle_info(path: str) -> Tuple[str, bool, List[str]]:
    """(spec_id, valid, reasons) — 가볍게 (layout.json · valid.json 만)."""
    if path == 'current':
        return 'current', True, []
    lj = json.loads(pathlib.Path(path, 'layout.json').read_text(encoding='utf-8'))
    valid, reasons = True, []
    vp = pathlib.Path(path, 'valid.json')
    if vp.is_file():
        v = json.loads(vp.read_text(encoding='utf-8'))
        valid, reasons = bool(v.get('valid', True)), list(v.get('reasons', ()))
    return str(lj.get('spec_id', os.path.basename(path))), valid, reasons


def _world(path: str) -> World:
    w = _WORLDS.get(path)
    if w is None:
        while len(_WORLDS) >= _WORLD_KEEP:
            _WORLDS.pop(next(iter(_WORLDS)))
        lay = load_current() if path == 'current' else load_bundle(path)
        w = _WORLDS[path] = World(lay)
    return w


def _init(lock, out: str) -> None:
    global _LOCK, _OUT
    _LOCK = lock
    _OUT = pathlib.Path(out)


def _append(name: str, rec: dict) -> None:
    line = json.dumps(rec, ensure_ascii=False, separators=(',', ':')) + '\n'
    with _LOCK:
        with open(_OUT / name, 'a', encoding='utf-8') as f:
            f.write(line)
            f.flush()


def _job(job) -> Tuple[int, int, float]:
    """(묶음, [(seed, load)], 설정 dict) → (성공, 실패, 벽시계 합)."""
    path, runs, cfgd = job
    ok = bad = 0
    wall = 0.0
    try:
        w = _world(path)
    except Exception:                                   # noqa: BLE001 — 묶음 자체 실패
        tb = traceback.format_exc()
        for seed, load in runs:
            _append('errors.jsonl', {'bundle': path, 'seed': seed, 'load': load,
                                     'stage': 'world', 'error': tb[-2000:], 't': _now()})
        return 0, len(runs), 0.0
    for seed, load in runs:
        try:
            cfg = SimConfig(**dict(cfgd, load=load))
            res = run(w, cfg, seed)
            wall += res['extras']['wall_s']
            res['extras']['bundle'] = path
            _append('extras.jsonl', res['extras'])
            _append('results.jsonl', res['row'])
            ok += 1
        except Exception:                               # noqa: BLE001 — 실행 하나 실패는 기록하고 계속
            _append('errors.jsonl', {'bundle': path, 'spec_id': w.layout.spec_id, 'seed': seed,
                                     'load': load, 'stage': 'run',
                                     'error': traceback.format_exc()[-2000:], 't': _now()})
            bad += 1
    return ok, bad, wall


def done_keys(out: pathlib.Path) -> set:
    keys = set()
    p = out / 'results.jsonl'
    if p.is_file():
        for line in p.read_text(encoding='utf-8').splitlines():
            if not line.strip():
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:                # 중간에 끊긴 마지막 줄
                continue
            keys.add((r['spec_id'], int(r['seed']), float(r['load'])))
    return keys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--bundles', required=True)
    ap.add_argument('--seeds', required=True)
    ap.add_argument('--loads', default='1.0')
    ap.add_argument('--out', required=True)
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--chunk', type=int, default=10)
    ap.add_argument('--workload', default='poisson', choices=('poisson', 'p5'))
    ap.add_argument('--horizon', type=float, default=3600.0)
    ap.add_argument('--base-rate', type=float, default=BASE_RATE)
    ap.add_argument('--soc', type=float, default=None)
    ap.add_argument('--end-when-done', action='store_true')
    ap.add_argument('--no-actors', action='store_true')
    ap.add_argument('--no-charging', action='store_true')
    ap.add_argument('--include-invalid', action='store_true')
    ap.add_argument('--stage', default='E1')
    ap.add_argument('--label', default='[계산]')
    ap.add_argument('--note', default='')
    ap.add_argument('--limit', type=int, default=0, help='시험용: 앞의 N 실행만')
    a = ap.parse_args(argv)
    if a.workers > 12:
        ap.error('--workers 는 12 이하 (공유 서버 규칙)')
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    seeds = parse_seeds(a.seeds)
    loads = [float(x) for x in a.loads.split(',')]
    cfgd = dict(horizon_s=a.horizon, base_rate=a.base_rate, workload=a.workload,
                actors=not a.no_actors, charging=not a.no_charging, soc=a.soc,
                end_when_done=a.end_when_done)
    bundles = find_bundles(a.bundles)
    done = done_keys(out)
    skipped_invalid = []
    jobs = []
    total = 0
    for b in bundles:
        sid, valid, reasons = bundle_info(b)
        if not valid and not a.include_invalid:
            skipped_invalid.append({'spec_id': sid, 'reasons': reasons})
            continue
        todo = [(s, ld) for ld in loads for s in seeds if (sid, s, ld) not in done]
        total += len(seeds) * len(loads)
        for i in range(0, len(todo), a.chunk):
            jobs.append((b, todo[i:i + a.chunk], cfgd))
    if a.limit:
        flat = [(b, [r], c) for b, rs, c in jobs for r in rs][:a.limit]
        jobs = flat
    n_todo = sum(len(j[1]) for j in jobs)
    t0 = time.time()
    start = {'event': 'start', 't': _now(), 'argv': sys.argv, 'args': vars(a),
             'commit': os.environ.get('LAYOPT_COMMIT', 'unknown'),
             'dirty': os.environ.get('LAYOPT_DIRTY', 'unknown'), 'load': _load(),
             'bundles': len(bundles), 'skipped_invalid': skipped_invalid, 'planned_rows': total,
             'already_done': len(done), 'todo_rows': n_todo, 'workers': a.workers,
             'cfg': cfgd}
    with open(out / 'manifest.jsonl', 'a', encoding='utf-8') as f:
        f.write(json.dumps(start, ensure_ascii=False) + '\n')
    print(f'E1 스윕: 묶음 {len(bundles)} (무효 건너뜀 {len(skipped_invalid)}), '
          f'할 일 {n_todo} / 계획 {total}, 이미 {len(done)}', flush=True)
    ok = bad = 0
    wall = 0.0
    lock = mp.Lock()
    if jobs:
        ctx = mp.get_context('fork')
        with ctx.Pool(min(a.workers, len(jobs)), initializer=_init,
                      initargs=(lock, str(out))) as pool:
            for k, (o, b_, w_) in enumerate(pool.imap_unordered(_job, jobs), 1):
                ok, bad, wall = ok + o, bad + b_, wall + w_
                if k % 10 == 0 or k == len(jobs):
                    el = time.time() - t0
                    print(f'  {k}/{len(jobs)} 묶음 작업, 행 {ok} 실패 {bad}, 경과 {el:.0f} s, '
                          f'실행당 {wall / max(ok, 1):.1f} s, load {_load()}', flush=True)
    end = {'event': 'end', 't': _now(), 'load': _load(), 'rows_ok': ok, 'rows_failed': bad,
           'elapsed_s': round(time.time() - t0, 1),
           'wall_per_run_s': round(wall / ok, 2) if ok else None,
           'rows_total_in_file': len(done_keys(out))}
    with open(out / 'manifest.jsonl', 'a', encoding='utf-8') as f:
        f.write(json.dumps(end, ensure_ascii=False) + '\n')
    complete = end['rows_total_in_file'] >= total and bad == 0
    reg = out / 'registry.json'
    if complete and not reg.exists():
        first = json.loads((out / 'manifest.jsonl').read_text(encoding='utf-8').splitlines()[0])
        rec = {'run_id': out.name, 'stage': a.stage, 'agent': 'layopt-sim', 'kind': 'eval',
               'sim': 'offline', 'commit': first['commit'], 'dirty': first['dirty'],
               'config': json.dumps({'bundles': a.bundles, 'seeds': a.seeds, 'loads': a.loads,
                                     **cfgd}, ensure_ascii=False),
               'seed': a.seeds, 'started': first['t'], 'ended': end['t'], 'raw_dir': str(out),
               'result': {'rows': end['rows_total_in_file'], 'bundles': len(bundles),
                          'skipped_invalid': len(skipped_invalid),
                          'wall_per_run_s': end['wall_per_run_s'], 'n': total},
               'label': a.label, 'load': f"{first['load']} → {end['load']}",
               'note': a.note}
        reg.write_text(json.dumps(rec, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'끝: 행 {ok} 실패 {bad}, 파일 행 {end["rows_total_in_file"]}/{total}, '
          f'완료={complete}', flush=True)
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
