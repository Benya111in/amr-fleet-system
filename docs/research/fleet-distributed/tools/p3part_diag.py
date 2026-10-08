#!/usr/bin/env python3
"""
회귀 진단 분석 (b4-wire, 사전 등록 notebook *_b4-wire_p3-partition-regression-diagnosis.md).

사용: p3part_diag.py <팔 디렉터리>... — 각 디렉터리의 run_<k>/ (tools/p3part_diag.sh 산출)를 읽어 실행마다 한 줄,
팔마다 요약 한 줄을 낸다. 판정에 쓰는 값은 시험 자신의 summary.json('④c') 이고, 기전 확인용으로 사건 로그에서
stale_lock_cleared (누가 · 어느 작업 · 보유자 · 시각)와 amr_05 의 task_granted 를 함께 센다.
"""

import json
import pathlib
import sys

PART_START, PART_END = 118.0, 160.0


def events(raw):
    out = []
    for f in sorted(raw.glob('events_*.jsonl')):
        for line in f.read_text(encoding='utf-8').splitlines():
            if line.strip():
                out.append(json.loads(line))
    return out


def one(run):
    rcf = run / 'rc'
    rc = int(rcf.read_text().strip()) if rcf.exists() else None
    raw = run / 'raw'
    sp = raw / 'summary.json'
    s = json.loads(sp.read_text(encoding='utf-8')) if sp.exists() else {}
    c = s.get('④c', {})
    evs = events(raw) if raw.exists() else []
    gc = [(e.get('robot'), e.get('task_id'), e.get('holder'), round(e['t'], 1))
          for e in evs if e.get('event') == 'stale_lock_cleared']
    gc_in = [g for g in gc if PART_START <= g[3] < PART_END]
    grants = sorted((round(e['t'], 1), e.get('robot'), e.get('task_id'))
                    for e in evs if e.get('event') == 'task_granted')
    meta = (run / 'meta').read_text().split() if (run / 'meta').exists() else []
    return {
        'run': run.name, 'rc': rc,
        'majority_pickups_in_partition': len(c.get('majority_pickups_in_partition', []) or []),
        'minority_pickups_in_partition': len(c.get('minority_pickups_in_partition', []) or []),
        'duplicates': len(c.get('duplicates', {}) or {}),
        'minority_commits_in_partition': len(c.get('minority_commits_in_partition', []) or []),
        'failed': [f[2] for f in c.get('failed', [])],
        'starts': c.get('starts'),
        'grants': grants,
        'stale_lock_cleared': len(gc), 'stale_lock_cleared_in_partition': len(gc_in),
        'stale_lock_cleared_list': gc,
        'strays': [meta[i + 1] for i, w in enumerate(meta) if w.startswith('strays_')],
    }


def main(argv):
    for arm in argv:
        arm = pathlib.Path(arm)
        rows = [one(r) for r in sorted(arm.glob('run_*'), key=lambda p: int(p.name.split('_')[1]))]
        for r in rows:
            print(arm.name, json.dumps(r, ensure_ascii=False))
        ran = [r for r in rows if r['rc'] is not None]
        vac = sum(1 for r in ran if r['majority_pickups_in_partition'] == 0)
        unsafe = sum(1 for r in ran if r['minority_pickups_in_partition'] or r['duplicates'])
        nfail = sum(1 for r in ran if r['rc'] != 0)
        print(f'SUMMARY {arm.name}: n={len(ran)} pytest_fail={nfail} '
              f'majority_pickups_in_partition==0: {vac}/{len(ran)} unsafe={unsafe} '
              f'runs_with_stale_lock_cleared={sum(1 for r in ran if r["stale_lock_cleared"])}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
