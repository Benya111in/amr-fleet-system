#!/usr/bin/env python3
"""
하네스 원자료에서 지표를 다시 계산한다 (Gazebo 를 다시 돌리지 않고 지표 정의 고침을 적용).

    python3 rl/eval/gazebo/recompute.py <세션 디렉터리>… [--suffix _recomputed]

세션의 `episodes/*.json.gz` (실행기가 남긴 원자료 전부: meta, place, episode 시계열) 를 지금의 `amr_rl_bench.metrics`
로 다시 계산해 `gzbench<suffix>.csv`·`episodes_v1<suffix>.csv` 를 쓴다. 원래 CSV 는 그대로 둔다 (무엇이 바뀌었는지 비교할 수
있게). 바뀐 열을 세어 출력한다.
"""

import argparse
import csv
import glob
import gzip
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'src', 'amr_rl_bench'))

from amr_rl_bench import metrics as M  # noqa: E402
from amr_rl_bench import records as R  # noqa: E402


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('sessions', nargs='+')
    ap.add_argument('--suffix', default='_recomputed')
    ap.add_argument('--train-seed', type=int, default=None)
    a = ap.parse_args()
    for sess in a.sessions:
        old = {r['exec_index']: r for r in csv.DictReader(
            open(os.path.join(sess, 'gzbench.csv'), encoding='utf-8'))}
        out_g = os.path.join(sess, f'gzbench{a.suffix}.csv')
        out_e = os.path.join(sess, f'episodes_v1{a.suffix}.csv')
        for f in (out_g, out_e):
            if os.path.exists(f):
                os.remove(f)
        seen, changed = {}, {}
        for f in sorted(glob.glob(os.path.join(sess, 'episodes', '*.json.gz'))):
            raw = json.load(gzip.open(f, 'rt', encoding='utf-8'))
            meta = raw['meta']
            if 'episode' in raw and meta.get('setup_ok'):
                mt = M.episode_metrics(raw['episode'])
            else:
                mt = {'outcome_contract': 'error', 'outcome_system': 'error'}
            row = R.gzbench_row(meta, mt)
            sd = row['scenario_seed']
            ep_index = seen.get(sd, 0)
            seen[sd] = ep_index + 1
            R.append_csv(out_g, R.COLUMNS, row)
            v1 = R.episode_v1_row(row, ep_index, a.train_seed)
            R.append_csv(out_e, list(v1.keys()), v1)
            o = old.get(str(row['exec_index']), {})
            for k in R.COLUMNS:
                if str(R.fmt(row.get(k))) != o.get(k, ''):
                    changed[k] = changed.get(k, 0) + 1
        print(json.dumps({'session': sess, 'episodes': sum(seen.values()),
                          'changed_columns': changed}, ensure_ascii=False))


if __name__ == '__main__':
    main()
