#!/usr/bin/env python3
"""
단계 C 집계 JSON(analyze_c.py 출력) → 결과 문서용 마크다운 표 (손으로 숫자를 옮기지 않으려고).

    report_c.py <analysis.json> [--c4 <c4 analysis.json>] > tables.md

표: 주 결론, 팔·집합별 성공 층위(유효·전체), 짝 차이(구간), 최종 거리·시간·CTE·개입·접촉·직전 정지·yaw, 제외 분류,
부하·실효 주기, Isaac 방향 비교. --interim 표시가 있는 JSON 은 머리에 '중간 집계 — 보고서에 쓰지 않음' 을 찍는다.
"""

import argparse
import json
import sys

ARM_NAME = {'dwa': 'DWA', 'rl': 'RL', 'rlho': 'RL+인계'}
SET_NAME = {'main': '주', 'stress': '스트레스'}
PAIR_NAME = {'rl-dwa': 'RL − DWA', 'rlho-dwa': 'RL+인계 − DWA', 'rlho-rl': 'RL+인계 − RL'}


def f(x, nd=3):
    """수 → 문자열 (None → —)."""
    if x is None:
        return '—'
    if isinstance(x, float):
        return f'{x:.{nd}f}'
    return str(x)


def pct(x):
    """비율 → % 문자열."""
    return '—' if x is None else f'{100 * x:.1f} %'


def ci(e, nd=3, scale=1.0):
    """{'est','lo','hi'} → 'est [lo, hi]'."""
    if not e or e.get('est') is None:
        return '—'
    return (f"{e['est'] * scale:+.{nd}f} [{e['lo'] * scale:+.{nd}f}, "
            f"{e['hi'] * scale:+.{nd}f}] (n {e['n']})")


def rate(e):
    """{'x','n','rate','wilson'} → 'x/n (r %, Wilson)'."""
    if not e or not e.get('n'):
        return '—'
    lo, hi = e['wilson']
    r = e.get('rate', e['x'] / e['n'])
    return f"{e['x']}/{e['n']} ({100 * r:.1f} %, {100 * lo:.1f}–{100 * hi:.1f})"


def q(v, nd=3):
    """백분위 목록 → '5/25/50/75/95'."""
    if not v or v[0] is None:
        return '—'
    return ' / '.join(f'{x:.{nd}f}' for x in v)


def row(*cells):
    """표 한 줄."""
    return '| ' + ' | '.join(str(c) for c in cells) + ' |'


def header(*cells):
    """표 머리와 구분선."""
    return [row(*cells), row(*(['---'] * len(cells)))]


def render(r):
    """집계 dict → 마크다운."""
    out = []
    if r.get('interim'):
        out.append('> **중간 집계 — 보고서에 쓰지 않음** (모든 묶음이 끝나기 전)\n')
    out.append(f"집계 {r['generated']}, 원자료 `{r['c2']}`, 팔별 에피소드 수 "
               + ', '.join(f"{ARM_NAME[k]} {v}" for k, v in r['counts'].items()) + '\n')
    p = r.get('primary')
    if p:
        out.append('### 주 결론 (확증 — 주 집합 strict_gt, RL − DWA)\n')
        out += header('짝', 'n', 'RL 성공', 'DWA 성공', 'RL 만', 'DWA 만',
                      '짝 차이 [부트스트랩 95 %]', 'McNemar 정확 양측 p')
        for name, e in (('유효', p['valid']), ('전체', p['all'])):
            out.append(row(name, e['n'], e['a'], e['b'], e['a_only'], e['b_only'],
                           ci(e['diff_boot']), f"{e['mcnemar_p']:.4f}"))
        out.append(f"\n판정(유효): **{p['verdict_valid']}**, 유효·전체 일치(강건): {p['robust']}, "
                   f"자료 완결: {p['complete']}\n")
    out.append('### 성공 층위 (팔·집합별, 유효 / 전체, Wilson 95 %)\n')
    out += header('팔', '집합', 'n', '제외 E1/E2/E3/E4', 'strict_gt 유효', 'strict_gt 전체',
                  'strict_est 유효', 'nav2compat 0.25 유효', 'Nav2 성공 유효')
    for arm, sets in r['summary'].items():
        for st, s in sets.items():
            if not s['n']:
                continue
            ex = '/'.join(str(s['excl'][k]) for k in ('E1', 'E2', 'E3', 'E4'))
            out.append(row(ARM_NAME[arm], SET_NAME[st], s['n'], ex,
                           rate(s['strict_gt_valid']), rate(s['strict_gt_all']),
                           rate(s['strict_est_valid']), rate(s['nav2compat_gt_valid']),
                           rate(s['system_success_valid'])))
    out.append('\n### 짝 차이 (기술 — 구간만)\n')
    out += header('비교', '집합', 'strict_gt 유효 (시나리오 부트스트랩)',
                  '같은 것 (목표 군집 부트스트랩)', 'strict_est 유효', 'nav2compat 유효',
                  'Nav2 성공 유효')
    for name, sets in r['paired'].items():
        for st, e in sets.items():
            g = e['strict_gt_valid']
            out.append(row(PAIR_NAME[name], SET_NAME[st], ci(g['diff_boot']),
                           f"{ci(g.get('diff_cluster_boot'))} (군집 {g.get('n_clusters', '—')})",
                           ci(e['strict_est_valid']['diff_boot']),
                           ci(e['nav2compat_gt_valid']['diff_boot']),
                           ci(e['system_success_valid']['diff_boot'])))
    out.append('')
    out += header('비교', '집합', '최종 거리 짝 차 중앙 [m]', '주행 시간 짝 차 평균 [s] (둘 다 성공)',
                  '경로 길이 짝 차 평균 [m]', 'CTE 평균 짝 차 [m]', '개입 건수 짝 차',
                  'STOP 존 시간 짝 차 [s]')
    for name, sets in r['paired'].items():
        for st, e in sets.items():
            out.append(row(PAIR_NAME[name], SET_NAME[st], ci(e['final_dist_gt_diff_median']),
                           ci(e['nav_time_s_both_success_mean'], 2),
                           ci(e['path_length_m_both_success_mean'], 2),
                           ci(e['cte_active_mean_m_mean']), ci(e['interventions_mean'], 1),
                           ci(e['zone_stop_s_mean'], 2)))
    out.append('\n### 분포·안전 (팔·집합별, 하네스 오류 E1 제외)\n')
    out += header('팔', '집합', '최종 거리 정답 5/25/50/75/95 % [m]', '> 0.10 / > 0.25 m',
                  '주행 시간 5/25/50/75/95 % [s]', 'CTE 평균 중앙 [m]', '개입 건 평균 · 시간 합 [s]',
                  'STOP 존 합 [s]', '접촉 에피소드', '계약 collision', '직전 정지 ≥ 2 s',
                  '최종 yaw 성공 50/95/100 % [rad]')
    for arm, sets in r['summary'].items():
        for st, s in sets.items():
            if not s['n']:
                continue
            out.append(row(
                ARM_NAME[arm], SET_NAME[st], q(s['final_dist_gt_q']),
                f"{s['final_dist_gt_gt010']} / {s['final_dist_gt_gt025']}",
                q(s['nav_time_s_q'], 1), f(s['cte_active_mean_m_q'][2]),
                f"{f(s['interventions']['mean'], 2)} · {f(s['intervention_time_s']['sum'], 1)}",
                f(s['zone_stop_s']['sum'], 1), rate(s['contact_eps']), s['collision_outcome'],
                s['near_goal_stop_ge2s'], q(s['final_yaw_succ_q'])))
    out.append('\n### 결과 분류 (계약 | Nav2 시스템)\n')
    for arm, sets in r['summary'].items():
        for st, s in sets.items():
            if s['n']:
                items = ', '.join(f'{k} {v}' for k, v in sorted(s['outcome_cross'].items()))
                out.append(f"- {ARM_NAME[arm]} {SET_NAME[st]}: {items}")
    t = r.get('timing', {})
    if t:
        out.append('\n### 부하·실효 주기 (에피소드 5/25/50/75/95 %, 공유 Isaac 부하 하 값)\n')
        out += header('팔', 'n', 'RTF', '제어 간격 sim [s]', '제어 간격 / (0.05·RTF)',
                      'sim 초당 결정', '결정 주기 RL [s]', '벽시계 [s]', 'GPU 사용률 [%]',
                      '동시 Isaac 수', 'loadavg 1 분')
        for arm, e in t.items():
            dp = None
            if arm != 'dwa':
                dp = r['summary'][arm]['main'].get('decision_period_mean_s_q')
            out.append(row(ARM_NAME[arm], e['n'], q(e['rtf_q']), q(e['ctrl_dt_sim_q'], 4),
                           q(e['ctrl_dt_over_005rtf_q']), q(e['decisions_per_sim_s_q'], 1),
                           q(dp, 4) if dp else '—', q(e['wall_episode_q'], 1),
                           q(e['gpu_util_q'], 0), q(e['n_isaac_q'], 1), q(e['load1_q'], 1)))
        d = t.get('dwa', {}).get('rtf_tercile')
        if d:
            parts = []
            for k, v in d.items():
                lo, hi = v['rtf_range']
                parts.append(f"{k} (RTF {f(lo, 3)}–{f(hi, 3)}) "
                             f"strict_gt {v['strict_gt']}/{v['n']}, "
                             f"성공 주행 시간 중앙 {f(v['nav_time_median_success'], 1)} s")
            out.append('\nDWA 를 에피소드 RTF 삼분위로 (기술): ' + '; '.join(parts))
        for arm in ('rl', 'rlho'):
            if arm in t and 'decision_period_out_of_band' in t[arm]:
                e = t[arm]
                out.append(f"\n{ARM_NAME[arm]} 결정 주기 [0.099, 0.101] 밖 에피소드: "
                           f"{e['decision_period_out_of_band']}/{e['n']} "
                           f"(값 없음 {e.get('decision_period_missing', '—')})")
    isa = r.get('isaac')
    if isa:
        out.append('\n### Isaac (같은 정책의 집합 평가) 와 나란히 — 엄격 성공\n')
        out += header('집합', 'Isaac', 'Gazebo RL strict_gt (유효)')
        gz = r['summary'].get('rl', {})
        for k in ('main_3_15', 'main_3_30', 'stress_3_15', 'stress_3_30'):
            st = 'main' if k.startswith('main') else 'stress'
            v = isa[k]
            out.append(row(k, f"{v['strict']}/{v['n']} ({100 * v['rate']:.1f} %)",
                           f"{SET_NAME[st]}: {rate(gz.get(st, {}).get('strict_gt_valid'))}"))
    return '\n'.join(out) + '\n'


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('analysis')
    a = ap.parse_args()
    with open(a.analysis, encoding='utf-8') as fh:
        r = json.load(fh)
    sys.stdout.write(render(r))
    return 0


if __name__ == '__main__':
    sys.exit(main())
