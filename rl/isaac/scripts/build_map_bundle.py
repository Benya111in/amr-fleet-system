#!/usr/bin/env python3
"""
지도 변형 묶음 만들기 (CPU, amr-ai/rl:0.3) — suite JSON → bundle npz + 요약 JSON.

    docker run --rm -v $WT:/ws:ro -v $R:$R -w /ws amr-ai/rl:0.3 \
        python3 rl/isaac/scripts/build_map_bundle.py \
        --suite rl/isaac/eval_sets/b1_map_suite_v1.json --assets $R/assets \
        --out $R/suite/map_bundle_v1.npz \
        --workers 8

평가(eval_mapgen.py)는 이 bundle 을 sha256 으로 받는다. 같은 suite·같은 코드면 bundle 배열은 workers 와 무관하게 같다.
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np  # noqa: E402

from amr_isaac import eval_sets as ES  # noqa: E402
from amr_isaac import map_bundle as MB  # noqa: E402
from amr_isaac import mapgen as MG  # noqa: E402


def main():
    """만들기."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--suite', required=True)
    ap.add_argument('--eval_set', default=os.path.join(os.path.dirname(HERE), 'eval_sets',
                                                       'b1_eval_sets_v1.json'))
    ap.add_argument('--assets', required=True, help='경로 풀·기하 캐시 디렉터리 (paths_*_s1.npz)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--workers', type=int, default=1)
    a = ap.parse_args()
    suite = MB.load_suite(a.suite)
    doc = ES.load_sets(a.eval_set)
    pool_file = os.path.join(a.assets, f'paths_{doc["geometry_sha"]}_s{doc["pool_seed"]}.npz')
    pool = dict(np.load(pool_file))
    nominal = MG.nominal_layout()
    t0 = time.time()
    arr, summ = MB.build_bundle(suite, doc, pool, nominal=nominal, workers=a.workers)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    sha = MB.save_bundle(a.out, arr)
    summ.update({'bundle': os.path.abspath(a.out), 'bundle_sha256': sha,
                 'suite_file': a.suite, 'suite_sha': str(arr['suite_sha']),
                 'pool_file': pool_file, 'eval_set': a.eval_set,
                 'build_wall_s': round(time.time() - t0, 1), 'workers': a.workers})
    with open(a.out.replace('.npz', '.summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summ, f, indent=1, ensure_ascii=False)
    brief = {c: {s: (v[s]['valid'], v[s]['valid_variant_main'], v[s]['changed_share_of_valid'])
                 for s in arr['set_names'].tolist()} for c, v in summ['conditions'].items()}
    print('MAP_BUNDLE ' + json.dumps({'sha256': sha, 'valid/main/changed': brief},
                                     ensure_ascii=False), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
