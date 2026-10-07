#!/usr/bin/env python3
"""
지도 변형 평가 집계 (b1-mapgen, 사전 등록 분석 — 결과 전에 커밋) — amr-ai/rl:0.3, CPU.

    B=/home/CAPYI_AI/runs/bonus1
    docker run --rm --network none -v $SRC:/ws:ro -v $B:$B:ro \
        -v $R/analysis:$R/analysis -w /ws amr-ai/rl:0.3 \
        python3 rl/isaac/scripts/analyze_mapgen.py \
        --out $R/analysis/mapgen_analysis.json --md $R/analysis/mapgen_analysis.md

입력: DR 켬 = <root>/mg_on_s<S>/ (정책 b1-ppo10 B1_ppo10_s<S> 최종),
DR 끔 = <root>/mg_off_s<S>/ (b1-dr B4_droff_s<S> 최종),
기존 명목 평가 (비트 대조) = b1-ppo10-eval B1e_ppo10_s<S>_final_sets ·
b1-dr B4_droff_s<S>/sets 의 episodes.npz.
정의·판정 규칙은 amr_isaac/mapgen_stats.py 머리말과 노트 항목. --partial 은 없는 시드를 빼고 낸다 (중간 점검용 —
판정은 두 팔 5 시드 전부로만).
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC = os.path.dirname(HERE)
sys.path.insert(0, ISAAC)
sys.path.insert(0, os.path.join(os.path.dirname(ISAAC), 'eval'))

import numpy as np  # noqa: E402

from amr_isaac import map_bundle as MB  # noqa: E402
from amr_isaac import mapgen_stats as MS  # noqa: E402

R = '/home/CAPYI_AI/runs/bonus1'


def main():
    """집계."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=f'{R}/b1-mapgen/eval')
    ap.add_argument('--on_fmt', default='mg_on_s{s}')
    ap.add_argument('--off_fmt', default='mg_off_s{s}')
    ap.add_argument('--seeds', default='0,1,2,3,4')
    ap.add_argument('--ref_on_fmt',
                    default=f'{R}/b1-ppo10-eval/eval/B1e_ppo10_s{{s}}_final_sets/episodes.npz')
    ap.add_argument('--ref_off_fmt', default=f'{R}/b1-dr/train/B4_droff_s{{s}}/sets/episodes.npz')
    ap.add_argument('--bundle', default=f'{R}/b1-mapgen/suite/map_bundle_v1.npz')
    ap.add_argument('--bundle_sha', default='7f7bf288d909')
    ap.add_argument('--out', required=True)
    ap.add_argument('--md', default='')
    ap.add_argument('--partial', action='store_true')
    a = ap.parse_args()
    seeds = [int(x) for x in a.seeds.split(',')]
    bundle = MB.load_bundle(a.bundle, expect_sha=a.bundle_sha or None)
    runs = {}
    for arm, f in (('on', a.on_fmt), ('off', a.off_fmt)):
        runs[arm] = []
        for s in seeds:
            d = os.path.join(a.root, f.format(s=s))
            if not os.path.exists(os.path.join(d, 'eval.json')):
                if a.partial:
                    continue
                print(f'없음: {d}', file=sys.stderr)
                return 1
            ev, ep = MS.load_run(d)
            runs[arm].append((s, ev, ep))
    refs = {}
    for arm, f in (('on', a.ref_on_fmt), ('off', a.ref_off_fmt)):
        refs[arm] = {}
        for s in seeds:
            p = f.format(s=s)
            if os.path.exists(p):
                refs[arm][s] = dict(np.load(p))
    res = MS.analyze(runs['on'], runs['off'], refs['on'], refs['off'], bundle=bundle)
    res['inputs'] = {arm: [os.path.join(a.root, (a.on_fmt if arm == 'on' else a.off_fmt)
                                        .format(s=s)) for s, _e, _p in runs[arm]]
                     for arm in ('on', 'off')}
    res['partial'] = a.partial
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    md = MS.markdown(res)
    if a.md:
        with open(a.md, 'w', encoding='utf-8') as f:
            f.write(md)
    print(md)
    return 0


if __name__ == '__main__':
    sys.exit(main())
