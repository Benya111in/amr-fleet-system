#!/usr/bin/env python3
"""
결과 그림: 탐침 직선의 속도 응답 — M3 켬/끔 × 사슬/이상 플랜트 (20 Hz) 과 절제·비교 구간 그림.

    python3 figures.py --runs /runs --out <docs/research/dwa-ablation/fig>
"""
import argparse
import csv
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def read_trace(fn):
    with open(fn, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    return [float(r['t']) for r in rows], [float(r['v']) for r in rows], \
        [float(r['v_cmd']) for r in rows]


def fig_probe(runs, out):
    fig, axs = plt.subplots(1, 2, figsize=(9, 3.2), sharey=True)
    for ax, plant in zip(axs, ('chain', 'ideal')):
        d = os.path.join(runs, 'traces', f'probe_{plant}_20')
        for arm, style, label in (('dwa_full', '-', 'M3 켬 (창 중심 = 직전 명령)'),
                                  ('dwa_no_m3', '--', 'M3 끔 (창 중심 = 측정값)')):
            fn = os.path.join(d, f'{arm}__P_straight30__s1.csv')
            if not os.path.exists(fn):
                continue
            t, v, vc = read_trace(fn)
            k = [i for i, x in enumerate(t) if x <= 6.0]
            ax.plot([t[i] for i in k], [v[i] for i in k], style, label=label)
        ax.plot([0, 1.0], [0, 1.0], ':', color='gray', label='한계 가속 1.0 m/s²')
        ax.set_title('하류 사슬 있음' if plant == 'chain' else '이상 플랜트 (사슬 없음)')
        ax.set_xlabel('시간 [s]')
        ax.grid(alpha=0.3)
    axs[0].set_ylabel('실제 속도 [m/s]')
    axs[0].legend(fontsize=7, loc='lower right')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'm3_probe.png'), dpi=130)


def fig_forest(runs, out):
    """표 1·2 의 짝 차 구간 (시간 비 %, |정지 오차| 차 cm) — chain 절제와 ROS 비교."""
    import math
    import analyze as A
    raw = os.path.join(runs, 'raw')
    with open(os.path.join(raw, 'tuning_selection.json'), encoding='utf-8') as f:
        chosen = json.load(f)
    groups = [('절제 (chain_sim, 하나 끔 − 전체)', A.load(os.path.join(raw, 'chain_clean.jsonl')),
               ['dwa_no_m1', 'dwa_no_m1c', 'dwa_no_m1e', 'dwa_no_m2', 'dwa_m2_trap', 'dwa_no_m3',
                'dwa_no_m4', 'dwa_no_m5', 'dwa_textbook']),
              ('비교 (ROS, 팔 − 우리 DWA)',
               A.substitute_tuned(A.load(os.path.join(raw, 'ros_heldout.jsonl')), chosen),
               ['dwa_textbook', 'DWB_D00', 'DWB_tuned', 'MPPI_P00', 'MPPI_tuned', 'RPP_R00',
                'RPP_tuned', 'pp'])]
    fig, axs = plt.subplots(1, 2, figsize=(10, 5.2))
    for ax, (title, recs, arms) in zip(axs, groups):
        ys, labels = [], []
        for i, arm in enumerate(arms):
            d = A.paired(recs, arm, 'dwa_full', 't_reach', A.logratio)
            m, lo, hi = A.boot(d.values())
            if math.isnan(m):
                continue
            k, n, _ = A.success_rate(recs, arm)
            y = len(arms) - i
            ax.errorbar(100 * (math.exp(m) - 1), y,
                        xerr=[[100 * (math.exp(m) - math.exp(lo))],
                              [100 * (math.exp(hi) - math.exp(m))]], fmt='o', capsize=3)
            ys.append(y)
            labels.append(f'{arm} (성공 {k}/{n})')
        ax.axvline(0, color='gray', lw=0.8)
        ax.set_yticks(ys)
        ax.set_yticklabels(labels, fontsize=8)
        ax.set_xlabel('주행 시간 차 [%] (둘 다 성공한 경로, 95 % 부트스트랩)')
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3, axis='x')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'forest_time.png'), dpi=130)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'NanumGothic', 'DejaVu Sans']
    fig_probe(args.runs, args.out)
    fig_forest(args.runs, args.out)
    print(json.dumps(sorted(os.listdir(args.out))))


if __name__ == '__main__':
    main()
