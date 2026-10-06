"""
B 단계 평가 집합 파일을 만든다 (Isaac 불필요 — amr-ai/rl:0.3 에서).

    docker run --rm -v $WT:/ws -v $R:$R -w /ws amr-ai/rl:0.3 \
        python3 rl/isaac/scripts/build_eval_sets.py --assets $R/assets \
        --out rl/isaac/eval_sets/b1_eval_sets_v1.json --clearance_out $R/eval_sets

held-out 경로 풀(seed 1)의 경로마다 끝 접선 자세의 풋프린트 앞 끝 직진 여유를 재고(amr_isaac.eval_sets),
3–15 m·3–30 m 마다 주 집합(≥ 0.40 m, n 2048)과 좁은 목표 스트레스 집합(< 0.40 m, n 512)을 고정 seed 로 뽑는다.
같은 풀·seed 로 다시 만들면 같은 파일이 나와야 한다 (--check 로 확인).
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'contract'))

import numpy as np  # noqa: E402

from amr_isaac import eval_sets as ES  # noqa: E402
from amr_isaac.assets import load_geometry, load_pool  # noqa: E402


def main():
    """집합 생성 또는 확인."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--assets', required=True, help='자산 캐시 디렉터리 (paths_<sha>_s1.npz 가 있는 곳)')
    ap.add_argument('--pool_seed', type=int, default=1)
    ap.add_argument('--seed', type=int, default=20261006, help='표본 seed')
    ap.add_argument('--n_main', type=int, default=2048)
    ap.add_argument('--n_stress', type=int, default=512)
    ap.add_argument('--n_perturb', type=int, default=1024)
    ap.add_argument('--threshold', type=float, default=ES.THRESHOLD)
    ap.add_argument('--out', default='rl/isaac/eval_sets/b1_eval_sets_v1.json')
    ap.add_argument('--clearance_out', default='', help='경로별 여유 npz 를 남길 디렉터리 (원자료)')
    ap.add_argument('--check', action='store_true', help='--out 파일을 다시 만들어 같은지 확인만 한다')
    a = ap.parse_args()
    t0 = time.time()
    geo = load_geometry(a.assets)
    pool = load_pool(seed=a.pool_seed, out_dir=a.assets)
    clear = ES.front_clearance(pool['pts'], pool['offs'], geo['obb'])
    sets, stats = ES.build_eval_sets(pool['length'], clear, n_main=a.n_main, n_stress=a.n_stress,
                                     seed=a.seed, threshold=a.threshold, n_perturb=a.n_perturb)
    for name, ids in sets.items():
        _, ng = ES.goal_groups(pool['pts'], pool['offs'], ids)
        stats[name]['distinct_goals'] = ng
    meta = {'pool_seed': a.pool_seed, 'pool_paths': int(pool['length'].shape[0]),
            'pool_digest': ES.pool_digest(pool['pts'], pool['offs'], pool['length']),
            'geometry_sha': geo['sha'], 'sample_seed': a.seed, 'threshold_m': a.threshold,
            'rule': 'front straight clearance of footprint at path-end tangent pose '
                    '(tangent = point 0.25 m back -> last point); '
                    'main >= threshold, stress < threshold',
            'stress_sampling': 'balanced replication over all narrow candidate paths',
            'main_sampling': 'without replacement when candidates >= n',
            'clearance_function': 'amr_isaac.eval_sets.straight_approach_batch '
                                  '(= eval_metrics.straight_approach)'}
    if a.check:
        old = ES.load_sets(a.out)
        same = all(old['sets'][k] == [int(x) for x in v] for k, v in sets.items()) and \
            old['pool_digest'] == meta['pool_digest'] and set(old['sets']) == set(sets)
        print(json.dumps({'check_same': same, 'pool_digest': meta['pool_digest']}))
        return 0 if same else 1
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    ES.save_sets(a.out, sets, stats, meta)
    if a.clearance_out:
        os.makedirs(a.clearance_out, exist_ok=True)
        np.savez_compressed(os.path.join(a.clearance_out, f'front_clearance_s{a.pool_seed}.npz'),
                            clearance=clear, length=pool['length'],
                            end_pose=ES.pool_end_poses(pool['pts'], pool['offs']))
    rng_counts = {}
    for lo, hi in ((3.0, 15.0), (3.0, 30.0)):
        inr = (pool['length'] >= lo) & (pool['length'] <= hi)
        rng_counts[f'{lo:g}_{hi:g}'] = {
            'paths': int(inr.sum()), 'lt_0p30': int((inr & (clear < 0.30)).sum()),
            'lt_0p40': int((inr & (clear < 0.40)).sum()),
            'inf': int((inr & np.isinf(clear)).sum())}
    print(json.dumps({'out': a.out, 'meta': meta, 'stats': stats, 'range_counts': rng_counts,
                      'seconds': round(time.time() - t0, 1)}, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
