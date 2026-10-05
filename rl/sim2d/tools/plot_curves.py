#!/usr/bin/env python3
"""
학습 곡선 그림 — run 디렉터리들의 eval.jsonl(결정론적 평가)과 train.jsonl(학습 롤아웃) 성공·접촉률.

    python3 rl/sim2d/tools/plot_curves.py --runs /runs/B_bc2 /runs/C_scratch1 --out curves.png
"""

import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def load(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [json.loads(x) for x in f if x.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    for run in a.runs:
        name = os.path.basename(run.rstrip('/'))
        ev = sorted(load(os.path.join(run, 'eval.jsonl')), key=lambda r: r['env_steps'])
        tr = load(os.path.join(run, 'train.jsonl'))
        if ev:
            x = [r['env_steps'] / 1e6 for r in ev]
            lab = f'{name} eval (deterministic)'
            axes[0].plot(x, [r.get('success', 0) for r in ev], 'o-', label=lab)
            axes[1].plot(x, [r.get('collision', 0) for r in ev], 'o-', label=lab)
        if tr:
            x = [r['env_steps'] / 1e6 for r in tr if 'success' in r]
            axes[0].plot(x, [r['success'] for r in tr if 'success' in r], alpha=0.4,
                         label=f'{name} train rollout')
            axes[1].plot(x, [r['collision'] for r in tr if 'success' in r], alpha=0.4,
                         label=f'{name} train rollout')
            axes[2].plot(x, [r['return'] for r in tr if 'success' in r], label=name)
    axes[0].set_title('success rate')
    axes[1].set_title('collision rate')
    axes[2].set_title('train episode return (raw reward)')
    for ax in axes:
        ax.set_xlabel('env steps [M]')
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print('saved', a.out)


if __name__ == '__main__':
    main()
