#!/usr/bin/env python3
"""
학습 곡선 그림 (curve.jsonl → curves.png) — amr-ai/rl:0.3 (matplotlib) 에서.

    python3 rl/isaac/scripts/plot_curves.py <run 디렉터리> [<run 디렉터리> …] --out <png>
"""

import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def load(run):
    """curve.jsonl 의 행들."""
    with open(os.path.join(run, 'curve.jsonl')) as f:
        return [json.loads(line) for line in f if line.strip()]


def main():
    """그린다."""
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    fig, axs = plt.subplots(2, 3, figsize=(15, 8))
    panels = [('success_rate', '성공률 (끝난 에피소드)'), ('collision_rate', '충돌률'),
              ('time_limit_rate', '시간 상한 비율'), ('stall_rate', '정체 비율'),
              ('mean_return', '평균 에피소드 반환 (무할인)'),
              ('chunk_env_steps_per_s', '처리량 env-steps/s (학습 포함)')]
    for run in a.runs:
        rows = load(run)
        x = [r['env_steps'] / 1e6 for r in rows]
        for ax, (k, title) in zip(axs.flat, panels):
            ax.plot(x, [r[k] for r in rows], label=os.path.basename(run.rstrip('/')))
            ax.set_title(title, fontsize=10)
            ax.set_xlabel('env steps [M]')
            ax.grid(alpha=0.3)
    axs.flat[0].legend(fontsize=8)
    fig.suptitle('Isaac Lab AMR nav (arm B) — rsl_rl PPO', fontsize=11)
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print(a.out)


if __name__ == '__main__':
    main()
