"""
E1 스윕 요약: results.jsonl → 배치 × 부하별 시드 평균과 설계 변수 수준별 주효과.

    python3 -m layopt.sim.summarize --run <e1 run_dir> --out <json>

배치 × 부하마다: 시드 수, 처리량 T, 작업 p50 · p95, 게이트 대기, 교착 · UNRESOLVED, 대기 슬롯 k ≥ 1 체류 W,
대기 슬롯 체류 합, 대기열 체류 합, 도크 · 충전기 체류 합. 수준별 주효과 = 그 수준을 가진 배치들의 평균 (배치 평균의 평균).
기준 대비 쌍차(공통 난수, 시드별 차의 평균)도 낸다.
"""

from __future__ import annotations

import argparse
import collections
import json
import pathlib
import statistics
import sys

BASELINE = 'v10-v20-v30-v40-v50'
METRICS = ('throughput_per_h', 'task_p50_s', 'task_p95_s', 'gate_wait_s', 'deadlocks',
           'unresolved', 'distance_m')


def dwell_sums(r: dict) -> dict:
    out = collections.Counter()
    for d in r['dwell']:
        kind = d['id'].split(':', 1)[0]
        for k, v in d['k_hist_s'].items():
            out[f'{kind}_s'] += v
            if int(k) >= 1:
                out[f'{kind}_k1_s'] += v
    return out


def row_values(r: dict) -> dict:
    v = {m: r[m] for m in METRICS}
    ds = dwell_sums(r)
    v['W'] = ds.get('wait_k1_s', 0.0)
    for key in ('wait_s', 'queue_s', 'queue_k1_s', 'dock_s', 'dock_k1_s', 'charger_s',
                'charger_k1_s'):
        v[key] = ds.get(key, 0.0)
    return v


def summarize(results: pathlib.Path) -> dict:
    cells = collections.defaultdict(dict)                 # (spec, load) → seed → values
    for line in results.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        cells[(r['spec_id'], float(r['load']))][int(r['seed'])] = row_values(r)
    per = {}
    for (sid, load), seeds in cells.items():
        keys = next(iter(seeds.values())).keys()
        agg = {}
        for k in keys:
            xs = [s[k] for s in seeds.values() if s[k] is not None]
            agg[k] = round(statistics.fmean(xs), 3) if xs else None
        agg['n_seeds'] = len(seeds)
        base = cells.get((BASELINE, load))
        if base is not None and sid != BASELINE:
            common = sorted(set(seeds) & set(base))
            agg['paired_vs_baseline'] = {
                k: round(statistics.fmean(seeds[s][k] - base[s][k] for s in common), 3)
                for k in ('throughput_per_h', 'task_p50_s', 'W', 'gate_wait_s')
                if all(seeds[s][k] is not None and base[s][k] is not None for s in common)
            } if common else None
        per[f'{sid}|{load:g}'] = agg
    main_eff = collections.defaultdict(lambda: collections.defaultdict(list))
    for key, agg in per.items():
        sid, load = key.split('|')
        for i, lv in enumerate(sid.split('-')):
            for k in ('throughput_per_h', 'task_p50_s', 'task_p95_s', 'W', 'gate_wait_s',
                      'deadlocks', 'unresolved', 'queue_s'):
                if agg.get(k) is not None:
                    main_eff[f'load={load}|V{i + 1}'][f'{lv}:{k}'].append(agg[k])
    eff = {g: {k: round(statistics.fmean(v), 2) for k, v in sorted(d.items())}
           for g, d in sorted(main_eff.items())}
    return {'n_cells': len(per), 'cells': per, 'main_effects': eff}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='E1 스윕 요약')
    ap.add_argument('--run', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    out = pathlib.Path(a.out)
    if out.exists():
        print(f'이미 있다 (덮어쓰지 않는다): {out}', file=sys.stderr)
        return 3
    doc = summarize(pathlib.Path(a.run) / 'results.jsonl')
    doc['run'] = str(a.run)
    out.write_text(json.dumps(doc, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'{doc["n_cells"]} 칸 → {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
