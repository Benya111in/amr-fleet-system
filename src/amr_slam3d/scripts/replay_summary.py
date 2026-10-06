#!/usr/bin/env python3
"""
재생 묶음 요약 — <루트>/<행>/r<번호>/ 를 모두 읽어 처리 스캔 누락 표와 대칭 비교표 manifest 를 만든다.

    replay_summary.py --root /runs/replay --gt gt.tum --out-dir /runs/summary [--rows a b ...]

출력: drops.json (실행별 누락 표 한 줄), drops.md (표), manifest.json (amr_evaluation.slam3d.symmetric 입력).
"""

import argparse
import json
import os
import re
import sys

from amr_slam3d import replaystat

COLS = ('input', 'processed', 'missing', 'missing_stamps', 'null_odom', 'imu_wait_drops',
        'tail_exempt', 'tail_missing', 'fatal', 'tf_valid', 'traj_n', 'launch_exit',
        'db_optimized_poses', 'load_play_start', 'load_play_end')


def collect(root, only=None):
    out = []
    for row in sorted(os.listdir(root)):
        d = os.path.join(root, row)
        if not os.path.isdir(d) or (only and row not in only):
            continue
        for rk in sorted(os.listdir(d)):
            m = re.fullmatch(r'r(\d+)', rk)
            if m and os.path.isfile(os.path.join(d, rk, 'probe.json')):
                out.append((row, int(m.group(1)), os.path.join(d, rk)))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--root', required=True)
    ap.add_argument('--gt', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--rows', nargs='*', default=None)
    ns = ap.parse_args(argv)
    os.makedirs(ns.out_dir, exist_ok=True)
    runs = collect(ns.root, ns.rows)
    drops = []
    for row, k, d in runs:
        s = replaystat.summarize(d, row)
        s['replay'] = k
        drops.append(s)
    with open(os.path.join(ns.out_dir, 'drops.json'), 'w', encoding='utf-8') as f:
        json.dump(drops, f, ensure_ascii=False, indent=1)
    lines = ['| 행 | 재생 | ' + ' | '.join(COLS) + ' |', '|' + '---|' * (len(COLS) + 2)]
    for s in drops:
        lines.append(f"| {s['row']} | r{s['replay']} | " +
                     ' | '.join('미측정' if s.get(c) is None else str(s.get(c)) for c in COLS) + ' |')
    with open(os.path.join(ns.out_dir, 'drops.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    man = {'gt': ns.gt, 'runs': [replaystat.manifest_entry(d, row, k) for row, k, d in runs]}
    with open(os.path.join(ns.out_dir, 'manifest.json'), 'w', encoding='utf-8') as f:
        json.dump(man, f, ensure_ascii=False, indent=1)
    print(f'{len(runs)} 실행 → {ns.out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
