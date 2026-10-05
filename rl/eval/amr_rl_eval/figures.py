"""
판정표 그림 (PNG, matplotlib Agg) — 보고서·발표용 정적 그림 네 장.

  seed_iqm.png   시뮬레이터별 비교군 성공률 IQM + 층화 부트스트랩 95 % 구간 (점·가로 막대)
  sprt.png       주 비교군 접촉 순차 감시 Λ_n 궤적과 NI 경계 ln(1/α)
  time_ecdf.png  Gazebo 성공 에피소드 주행 시간 누적분포 (비교군별)
  srcc.png       시뮬레이터 쌍의 정책별 성공률 산점도 (SRCC)

색은 비교군(entity)에 고정한다 — 순서가 바뀌거나 비교군이 빠져도 색이 옮겨 가지 않는다.
팔레트는 dataviz 기준 범주 팔레트 1~5 번(밝은 면)이다. 글자는 잉크색, 색은 표식에만 쓴다.
"""

import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from . import schema  # noqa: E402

ARM_COLORS = dict(zip(schema.ARMS, ('#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4')))
INK, INK2, GRID = '#1f1f1e', '#5e5d59', '#e4e2da'


def _style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=9)


def _font():
    # 한글 글꼴이 있으면 쓰고, 없으면 기본 글꼴 (영문 라벨만 쓰므로 깨지지 않는다)
    plt.rcParams['font.size'] = 10
    plt.rcParams['axes.titlesize'] = 11


def seed_iqm(res, path):
    """시뮬레이터별 IQM 점·구간."""
    _font()
    blocks = [(sim, rows) for sim, rows in res['seed_aggregate'].items() if rows]
    if not blocks:
        return None
    fig, axes = plt.subplots(1, len(blocks), figsize=(4.2 * len(blocks), 2.8), squeeze=False)
    for ax, (sim, rows) in zip(axes[0], blocks):
        _style(ax)
        rows = sorted(rows, key=lambda r: schema.ARMS.index(r['arm']))
        for i, r in enumerate(rows):
            col = ARM_COLORS[r['arm']]
            ax.plot([r['lo'], r['hi']], [i, i], color=col, linewidth=2, solid_capstyle='round')
            ax.plot([r['iqm']], [i], 'o', color=col, markersize=7, markeredgecolor='white',
                    markeredgewidth=1.5)
        ax.set_yticks(range(len(rows)))
        ax.set_yticklabels([f'{r["arm"]} (runs {r["runs"]})' for r in rows], color=INK)
        ax.invert_yaxis()
        ax.set_xlabel('success IQM (95% strat. bootstrap)', color=INK2)
        ax.set_title(sim, color=INK, loc='left')
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    return path


def sprt(res, path, prereg=None):
    """주 비교군 Λ_n 궤적."""
    _font()
    c = res['confirmatory']
    st = c.get('sprt')
    if st is None or not st.history:
        return None
    d = st.design
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    _style(ax)
    n = np.arange(1, len(st.history) + 1)
    ax.plot(n, st.history, color=ARM_COLORS.get(c['arm'], INK), linewidth=2,
            label=f'{c["arm"]} log-LR')
    ax.axhline(d.upper, color=INK2, linewidth=1, linestyle='--')
    ax.text(1, d.upper, f' NI boundary ln(1/α) = {d.upper:.2f}', color=INK2, va='bottom',
            fontsize=8)
    ax.axvline(d.n_zero, color=GRID, linewidth=1)
    ax.text(d.n_zero, ax.get_ylim()[0], f' zero-contact stop n={d.n_zero}', color=INK2,
            va='bottom', fontsize=8, rotation=90)
    ax.set_xlim(0, max(d.n_max, len(n)) + 5)
    ax.set_xlabel('Gazebo episodes (execution order)', color=INK2)
    ax.set_ylabel('Λ_n', color=INK2)
    ax.set_title(f'Collision NI monitor (p0={d.p0:.0%}, p1={d.p1:.1%}, α={d.alpha}) — '
                 f'{st.decision} at n={st.n}', color=INK, loc='left')
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    return path


def time_ecdf(rows, path, simulator='gazebo'):
    """성공 에피소드 주행 시간 ECDF."""
    _font()
    sub = [r for r in rows if r['simulator'] == simulator and r['safety_node'] == 'on'
           and schema.success(r) and r['time_s'] is not None]
    arms = [a for a in schema.ARMS if any(r['arm'] == a for r in sub)]
    if not arms:
        return None
    fig, ax = plt.subplots(figsize=(6.0, 3.0))
    _style(ax)
    for a in arms:
        t = np.sort([r['time_s'] for r in sub if r['arm'] == a])
        ax.step(t, np.arange(1, t.size + 1) / t.size, where='post', color=ARM_COLORS[a],
                linewidth=2, label=f'{a} (n={t.size})')
    ax.set_xlabel(f'time to goal on success [s] ({simulator})', color=INK2)
    ax.set_ylabel('fraction of episodes', color=INK2)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK)
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    return path


def srcc_scatter(res, path):
    """첫 번째 SRCC 쌍의 정책별 점."""
    _font()
    if not res['srcc']:
        return None
    s = res['srcc'][0]
    fig, ax = plt.subplots(figsize=(4.0, 3.6))
    _style(ax)
    pts = []
    for p, xa, xb in zip(s['policies'], s['scores_a'], s['scores_b']):
        arm = p.split('/')[0] if '/' in p else 'A_dwa'
        pts.append((arm if arm in ARM_COLORS else 'A_dwa', xa, xb))
    pts.sort(key=lambda t: schema.ARMS.index(t[0]))      # 범례를 비교군 고정 순서로
    seen = set()
    for arm, xa, xb in pts:
        ax.plot([xa], [xb], 'o', color=ARM_COLORS[arm], markersize=7, markeredgecolor='white',
                markeredgewidth=1.5, label=None if arm in seen else arm)
        seen.add(arm)
    lo = min(s['scores_a'] + s['scores_b'])
    hi = max(s['scores_a'] + s['scores_b'])
    pad = 0.02 * (hi - lo if hi > lo else 1)
    ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad], color=GRID, linewidth=1)
    ax.set_xlabel(f'{s["sim_a"]} success rate', color=INK2)
    ax.set_ylabel(f'{s["sim_b"]} success rate', color=INK2)
    rho = s['rho']
    ax.set_title(f'SRCC ρ={rho:.2f} (N={s["n_policies"]}, BW 95% '
                 f'[{s["bw_lo"]:.2f}, {s["bw_hi"]:.2f}])' if not math.isnan(rho) else 'SRCC',
                 color=INK, loc='left', fontsize=9)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK)
    fig.tight_layout()
    fig.savefig(path, dpi=150, facecolor='white')
    plt.close(fig)
    return path


def render_all(res, rows, out_dir):
    """네 그림을 out_dir 에 쓰고 만든 경로 목록을 돌려준다."""
    os.makedirs(out_dir, exist_ok=True)
    made = [seed_iqm(res, os.path.join(out_dir, 'seed_iqm.png')),
            sprt(res, os.path.join(out_dir, 'sprt.png')),
            time_ecdf(rows, os.path.join(out_dir, 'time_ecdf.png')),
            srcc_scatter(res, os.path.join(out_dir, 'srcc.png'))]
    return [m for m in made if m]
