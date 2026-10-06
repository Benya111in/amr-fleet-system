#!/usr/bin/env python3
"""
학습 곡선 그림 — run 디렉터리들의 평가(결정론적 정책)와 train.jsonl(학습 롤아웃) 성공·접촉률.

    python3 rl/sim2d/tools/plot_curves.py --runs /runs/B_bc2 /runs/C_scratch1 --out curves.png

평가는 reeval.jsonl(reeval_runs.py, env 마다 첫 에피소드)이 있으면 그것을, 없으면 eval.jsonl 을 쓴다.
eval.jsonl 의 행에 'estimator' 가 없으면 a8afe77 전의 편향 추정량(먼저 끝난 에피소드 k 개)이라 그림에서
'biased' 로 표시한다. 학습 롤아웃 값은 롤아웃 동안 끝난 에피소드의 비율이라 짧게 끝나는 에피소드 쪽으로
치우친다 — 평가 지표가 아니다.
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
    for i, run in enumerate(a.runs):
        col = f'C{i % 10}'
        name = os.path.basename(run.rstrip('/'))
        re_rows = load(os.path.join(run, 'reeval.jsonl'))
        if re_rows:
            seed0 = re_rows[0]['eval_seed']
            ev = [r for r in re_rows if r['ckpt'] != 'heuristic' and r['eval_seed'] == seed0]
            heur = [r for r in re_rows if r['ckpt'] == 'heuristic' and r['eval_seed'] == seed0]
            lab = f"{name} eval (first ep./env, n={ev[0]['episodes'] if ev else 0})"
        else:
            ev = load(os.path.join(run, 'eval.jsonl'))
            heur = []
            biased = bool(ev) and 'estimator' not in ev[0]
            lab = f"{name} eval{' (biased: first-k-finished)' if biased else ''}"
        ev = sorted(ev, key=lambda r: r['env_steps'])
        tr = load(os.path.join(run, 'train.jsonl'))
        if ev:
            x = [r['env_steps'] / 1e6 for r in ev]
            se = [2 * r.get('success_se', 0.0) for r in ev]
            axes[0].errorbar(x, [r.get('success', 0) for r in ev], yerr=se, fmt='o-',
                             capsize=2, color=col, label=lab)
            axes[1].plot(x, [r.get('collision', 0) for r in ev], 'o-', color=col, label=lab)
        for h in heur:
            axes[0].axhline(h['success'], ls=':', lw=1, color=col,
                            label=f"{name} heuristic (n={h['episodes']})")
        if tr:
            x = [r['env_steps'] / 1e6 for r in tr if 'success' in r]
            lab_tr = f'{name} train rollout (finished-episode rate)'
            axes[0].plot(x, [r['success'] for r in tr if 'success' in r], alpha=0.35,
                         color=col, lw=1, label=lab_tr)
            axes[1].plot(x, [r['collision'] for r in tr if 'success' in r], alpha=0.35,
                         color=col, lw=1, label=lab_tr)
            axes[2].plot(x, [r['return'] for r in tr if 'success' in r], color=col, label=name)
    axes[0].set_title('success rate (eval error bar = ±2 SE)')
    axes[1].set_title('collision rate')
    axes[2].set_title('train episode return (raw reward)')
    for ax in axes:
        ax.set_xlabel('env steps [M]')
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=6)
    axes[2].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print('saved', a.out)


if __name__ == '__main__':
    main()
