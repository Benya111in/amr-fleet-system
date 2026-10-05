#!/usr/bin/env python3
"""
결과 CSV → 사전 등록 판정표·JSON·그림.

사용:
  python3 rl/eval/scripts/evaluate.py <결과.csv> [<결과.csv> ...] --out-dir <디렉터리>
      [--primary-policy C_hybrid/s3/best] [--reps 2000] [--no-figures]
  python3 rl/eval/scripts/evaluate.py --synthetic --out-dir /tmp/x   # 합성 데이터로 점검

판정 기준은 amr_rl_eval.report.PREREG (= PROTOCOL.md §6) 이고 명령행으로 바꿀 수 없다.
"""

import argparse
import dataclasses
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from amr_rl_eval import figures, report, schema, synth  # noqa: E402


def _jsonable(x):
    if dataclasses.is_dataclass(x):
        return {k: _jsonable(v) for k, v in dataclasses.asdict(x).items() if k != 'history'}
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return None
    if hasattr(x, 'item'):
        return _jsonable(x.item())
    return x


def main(argv=None):
    """명령행 진입점."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('csv', nargs='*')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--primary-policy', default=None)
    ap.add_argument('--reps', type=int, default=2000)
    ap.add_argument('--synthetic', action='store_true')
    ap.add_argument('--no-figures', action='store_true')
    a = ap.parse_args(argv)
    if a.synthetic:
        rows = synth.make_dataset(seed=0)
        os.makedirs(a.out_dir, exist_ok=True)
        schema.write_csv(os.path.join(a.out_dir, 'synthetic_episodes.csv'), rows)
    elif a.csv:
        rows = report.load(a.csv)
    else:
        ap.error('결과 CSV 또는 --synthetic 이 필요하다')
    res = report.evaluate(rows, primary_policy=a.primary_policy, reps=a.reps)
    os.makedirs(a.out_dir, exist_ok=True)
    md = report.render_markdown(res)
    if a.synthetic:
        md = '> **합성 데이터 — 측정값이 아니다** (amr_rl_eval.synth).\n\n' + md
    with open(os.path.join(a.out_dir, 'report.md'), 'w', encoding='utf-8') as f:
        f.write(md)
    with open(os.path.join(a.out_dir, 'report.json'), 'w', encoding='utf-8') as f:
        json.dump(_jsonable(res), f, ensure_ascii=False, indent=1)
    made = [] if a.no_figures else figures.render_all(res, rows, a.out_dir)
    for p in ['report.md', 'report.json'] + [os.path.basename(m) for m in made]:
        print(os.path.join(a.out_dir, p))
    return 0


if __name__ == '__main__':
    sys.exit(main())
