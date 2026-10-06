#!/usr/bin/env python3
"""
G2 학습 가능성 요약 표와 학습 곡선.

run 디렉터리들의 eval.jsonl(편향 없는 첫 에피소드 추정량)·train.jsonl·wall.json 을 모아 마크다운 표와 그림을
만든다.

    python3 rl/sim2d/tools/g2_summary.py --runs /runs/g2/prog50_s0 ... --out-dir /runs/g2/summary

표: 마지막 평가(사전에 정한 판정 기준 = 마지막 반복 정책)의 성공률 ± SE, 충돌·정체·시간 상한, 진행 비율, 제자리 회전
비율, 1 m 밖 스텝 비율, 최대 이탈 평균, 목표 0.5 m 안 도달 비율, 개입·CTE 비용, 평가 최고 성공률, 성공률이 처음 30 %
이상인 env-step, 벽시계. 그림: 평가 성공·정체·진행 비율과 학습 롤아웃 성공(끝난 에피소드 비율, 짧은 에피소드 쪽 편향).
"""

import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def load(path):
    """줄마다 JSON 인 파일 → 행 목록 (없으면 빈 목록)."""
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return [json.loads(x) for x in f if x.strip()]


def summarize(run):
    """학습 run 디렉터리 → 요약 dict."""
    ev = sorted(load(os.path.join(run, 'eval.jsonl')), key=lambda r: r['env_steps'])
    cfg = json.load(open(os.path.join(run, 'config.json')))
    wall = {}
    if os.path.exists(os.path.join(run, 'wall.json')):
        wall = json.load(open(os.path.join(run, 'wall.json')))
    a = cfg['args']
    out = {'run': os.path.basename(run.rstrip('/')), 'arm': a['arm'], 'reward': a.get('reward'),
           'seed': a['seed'], 'obstacles': a['obstacles'], 'git_commit': a.get('git_commit'),
           'evals': len(ev), 'wall_s': wall.get('wall_s'),
           'gpu_max_reserved_gb': wall.get('gpu_max_reserved_gb')}
    if not ev:
        return out
    last = ev[-1]
    keys = ('success', 'success_se', 'collision', 'stalled', 'timeout', 'progress', 'spin_frac',
            'offpath_1m_frac', 'max_e', 'near_goal_0p5', 'goal_min_median', 'cost_intervention',
            'cost_cte_track', 'cost_deviation_1m', 'time_s', 'episodes', 'unfinished', 'env_steps')
    out.update({'final_' + k: last.get(k) for k in keys})
    out['max_success'] = max(r.get('success', 0.0) for r in ev)
    hit = [r['env_steps'] for r in ev if r.get('success', 0.0) >= 0.30]
    out['steps_to_30pct'] = hit[0] if hit else None
    return out


def fmt(x, nd=3):
    """숫자 → 문자열 (None = —)."""
    if x is None:
        return '—'
    if isinstance(x, float):
        return f'{x:.{nd}f}'
    return str(x)


def table(rows):
    """요약 → 마크다운 표."""
    head = ('| run | 보상 | 시드 | 장애물 | 성공 ± SE | 충돌 | 정체 | 시간상한 | 진행 | 회전 | 1 m 밖 | '
            '최대 이탈 [m] | 목표 0.5 m 도달 | 개입 | 최고 성공 | 30 % 도달 [M] | 벽시계 [s] |')
    lines = [head, '|' + '---|' * 4 + '---:|' * 13]
    for r in rows:
        s30 = r.get('steps_to_30pct')
        lines.append(
            f"| {r['run']} | {r['reward']} | {r['seed']} | {r['obstacles']} | "
            f"{fmt(r.get('final_success'))} ± {fmt(r.get('final_success_se'))} | "
            f"{fmt(r.get('final_collision'))} | {fmt(r.get('final_stalled'))} | "
            f"{fmt(r.get('final_timeout'))} | {fmt(r.get('final_progress'))} | "
            f"{fmt(r.get('final_spin_frac'))} | {fmt(r.get('final_offpath_1m_frac'))} | "
            f"{fmt(r.get('final_max_e'), 2)} | {fmt(r.get('final_near_goal_0p5'))} | "
            f"{fmt(r.get('final_cost_intervention'), 2)} | {fmt(r.get('max_success'))} | "
            f"{fmt(s30 / 1e6 if s30 is not None else None, 1)} | {fmt(r.get('wall_s'), 0)} |")
    return '\n'.join(lines) + '\n'


def plot(runs, path):
    """학습 곡선 4 칸."""
    fig, ax = plt.subplots(1, 4, figsize=(20, 4.5))
    for i, run in enumerate(runs):
        name = os.path.basename(run.rstrip('/'))
        col, ls = f'C{i % 10}', '-' if i < 10 else '--'
        ev = sorted(load(os.path.join(run, 'eval.jsonl')), key=lambda r: r['env_steps'])
        tr = [r for r in load(os.path.join(run, 'train.jsonl')) if 'success' in r]
        if ev:
            x = [r['env_steps'] / 1e6 for r in ev]
            ax[0].errorbar(x, [r.get('success', 0) for r in ev],
                           yerr=[2 * r.get('success_se', 0) for r in ev], fmt='o' + ls,
                           color=col, capsize=2, label=name)
            ax[1].plot(x, [r.get('stalled', 0) for r in ev], 'o' + ls, color=col, label=name)
            ax[2].plot(x, [r.get('progress', 0) for r in ev], 'o' + ls, color=col, label=name)
        if tr:
            ax[3].plot([r['env_steps'] / 1e6 for r in tr], [r['success'] for r in tr], ls,
                       color=col, lw=1, label=name)
    ax[0].axhline(0.30, color='k', lw=0.8, ls=':')
    ax[0].axhline(0.05, color='k', lw=0.8, ls=':')
    titles = ('eval success (n=2048, first ep./env, ±2 SE)', 'eval stall rate',
              'eval progress 1 − d_end/d_0', 'train rollout success (finished episodes)')
    for a, t in zip(ax, titles):
        a.set_title(t, fontsize=9)
        a.set_xlabel('env steps [M]')
        a.grid(alpha=0.3)
    ax[0].legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--out-dir', required=True)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    rows = [summarize(r) for r in a.runs]
    with open(os.path.join(a.out_dir, 'g2_summary.json'), 'w') as f:
        json.dump(rows, f, indent=1)
    md = table(rows)
    with open(os.path.join(a.out_dir, 'g2_summary.md'), 'w') as f:
        f.write(md)
    plot(a.runs, os.path.join(a.out_dir, 'g2_curves.png'))
    print(md)


if __name__ == '__main__':
    main()
