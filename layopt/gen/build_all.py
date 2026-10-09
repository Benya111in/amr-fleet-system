#!/usr/bin/env python3
"""
540 개 배치 묶음 전부 만들기 (컨테이너 안, 다중 프로세스). 이미 있는 묶음(봉인됨)은 건너뛴다 — 끊겨도 다시 부르면 이어진다.

    layopt/gen/run_build.sh all --workers 6        # 호스트 (cpu_slot.sh 안에서)

결과 요약: runs/layopt/tmp/gen/batches/<run_id>/{summary.json, results.jsonl} — 묶음마다 valid.json 이 원본이다.
"""

import argparse
import collections
import datetime as dt
import json
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from layopt.gen import levels as L  # noqa: E402
from layopt.gen.common import BUNDLES  # noqa: E402

BATCHES = Path('/home/CAPYI_AI/runs/layopt/tmp/gen/batches')


def _one(args):
    sid, commit, dirty = args
    from layopt.gen import build_bundle as B
    out = BUNDLES / sid
    t0 = time.time()
    if out.exists():
        try:
            v = json.loads((out / 'valid.json').read_text(encoding='utf-8'))
            return {'spec_id': sid, 'status': 'exists', 'valid': v['valid'],
                    'reasons': v['reasons'], 's': 0.0}
        except (OSError, ValueError, KeyError) as exc:
            return {'spec_id': sid, 'status': f'exists_unreadable {exc}', 'valid': None,
                    'reasons': [], 's': 0.0}
    try:
        v = B.build(L.parse_spec_id(sid), out, commit, dirty, seal=True)
        return {'spec_id': sid, 'status': 'built', 'valid': v['valid'], 'reasons': v['reasons'],
                's': round(time.time() - t0, 2)}
    except Exception as exc:  # noqa: B902 — 한 배치 실패가 전체를 멈추지 않게
        return {'spec_id': sid, 'status': f'error {type(exc).__name__}: {exc}', 'valid': None,
                'reasons': [], 's': round(time.time() - t0, 2)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--commit', required=True)
    ap.add_argument('--dirty', action='store_true')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--only', default='', help='쉼표로 spec_id 목록 (기본 540 전부)')
    a = ap.parse_args(argv)
    run_id = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_bundles'
    bdir = BATCHES / run_id
    bdir.mkdir(parents=True, exist_ok=False)
    ids = [s for s in a.only.split(',') if s] or L.all_spec_ids()
    started = dt.datetime.now().astimezone().isoformat(timespec='seconds')
    load0 = os.getloadavg()
    t0 = time.time()
    res = []
    with mp.Pool(a.workers) as pool, (bdir / 'results.jsonl').open('w', encoding='utf-8') as f:
        for r in pool.imap_unordered(_one, [(s, a.commit, a.dirty) for s in ids]):
            res.append(r)
            f.write(json.dumps(r, ensure_ascii=False) + '\n')
            f.flush()
    hist = collections.Counter()
    for r in res:
        for kind in sorted({x.split(':', 1)[0] for x in r['reasons']}):
            hist[kind] += 1
    detail = collections.Counter(x for r in res for x in r['reasons'])
    summary = {'run_id': run_id, 'commit': a.commit, 'dirty': a.dirty, 'started': started,
               'ended': dt.datetime.now().astimezone().isoformat(timespec='seconds'),
               'wall_s': round(time.time() - t0, 1), 'workers': a.workers,
               'load_start': [round(v, 2) for v in load0],
               'load_end': [round(v, 2) for v in os.getloadavg()],
               'n': len(res), 'n_valid': sum(1 for r in res if r['valid'] is True),
               'n_invalid': sum(1 for r in res if r['valid'] is False),
               'n_error': sum(1 for r in res if r['valid'] is None),
               'status': dict(collections.Counter(r['status'].split(' ')[0] for r in res)),
               'invalid_kinds': dict(hist), 'invalid_reasons_top': detail.most_common(40),
               'errors': [r for r in res if r['valid'] is None][:20]}
    (bdir / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1) + '\n',
                                       encoding='utf-8')
    print(json.dumps({k: summary[k] for k in ('run_id', 'n', 'n_valid', 'n_invalid', 'n_error',
                                              'invalid_kinds', 'wall_s')}, ensure_ascii=False))
    return 0 if summary['n_error'] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
