"""
보너스 4 P5b 분석 (docs/research/fleet-distributed/tools/p5b_analyze.py) · P5 도구 더하기 — 합성 자료로 판정.

P5b 는 구역 진입 몸체 판정 뒤 Gazebo 재확인이다. 안전 재확인 판정(불변식 · P5 대비 비율 · 결정점 정지 침범), 갈래 고르기
(`.done` 만, 모드 거르기), 막힘 거리 0 사건, F5L 문구, 드라이버 더하기(agent · run_id 머리 · 메모리 조건 — 기본값이 P5
동작)를 본다.
"""

import json
import math
from pathlib import Path
import sys

import pytest

HERE = Path(__file__).resolve()
REPO = HERE.parents[3]
TOOLS = REPO / 'docs' / 'research' / 'fleet-distributed' / 'tools'
for p in (HERE.parents[1] / 'fleet_p5', TOOLS):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
import p5_plan as P  # noqa: E402
import p5b_analyze as B  # noqa: E402

from test_fleet_p5 import track, write_run  # noqa: E402


# ---------------------------------------------------------------------- 통계
def test_conditional_binomial_lower_tail_exact_values():
    assert B.cond_binom_lower(0, 980, 8, 980) == pytest.approx(0.5 ** 8)
    assert B.cond_binom_lower(1, 980, 8, 980) == pytest.approx(10 / 512)
    assert B.cond_binom_lower(2, 980, 8, 980) == pytest.approx(56 / 1024)        # 기각 못 함
    assert B.cond_binom_lower(0, 0, 8, 980) == 1.0 and B.cond_binom_lower(0, 10, 0, 10) == 1.0
    # 방향: 새 쪽이 많으면 p 는 1 가까이 (적다고 하지 않는다)
    assert B.cond_binom_lower(8, 980, 0, 980) == pytest.approx(1.0)
    # 노출이 다르면 율로 본다: 새 0 / 490 대 P5 8 / 980 → 8 건이 모두 P5 쪽일 확률 (2/3)^8 = 0.039
    assert B.cond_binom_lower(0, 490, 8, 980) == pytest.approx((2 / 3) ** 8)


def test_fisher_lower_tail_exact_values():
    # 새 0/149 대 P5 2/149: P(첫 행 0) = C(149,2)/C(298,2)
    assert B.fisher_lower(0, 149, 2, 149) == pytest.approx(math.comb(149, 2) / math.comb(298, 2))
    assert B.fisher_lower(0, 149, 2, 149) > 0.05                                  # 이 기준으로는 가를 수 없다
    assert B.fisher_lower(0, 374, 21, 374) < 0.001
    assert B.fisher_lower(2, 149, 2, 149) == pytest.approx(1.0) or \
        B.fisher_lower(2, 149, 2, 149) > 0.5
    assert B.fisher_lower(0, 0, 0, 0) == 1.0


def test_rule_of_three():
    assert B.rule_of_three(1000) == pytest.approx(0.003) and B.rule_of_three(0) is None


def test_verdict_never_says_same_and_flags_any_violation():
    base = {'disc': 8, 'entries': 980, 'gate_stops': 149, 'gate_stops_neg': 2,
            'held_overlaps': 0, 'dup_pickups': 0, 'low_epoch_accepted': 0}
    v = B.safety_verdict(dict(base, disc=3), base)
    assert v['invariant'].startswith('미달') and v['rate'].startswith('가르지 못함')
    v = B.safety_verdict(dict(base, disc=0, gate_stops_neg=0), base)
    assert v['invariant'].startswith('충족') and v['rate'].startswith('P5 보다 적다')
    assert '3.061e-03' in v['invariant'] or '0.0031' in v['invariant']
    assert v['stop'].startswith('가르지 못함')
    v = B.safety_verdict(dict(base, disc=0, dup_pickups=1), base)
    assert v['invariant'].startswith('미달') and 'dup_pickups 1' in v['invariant']
    for v in (B.safety_verdict(dict(base, disc=k), base) for k in range(0, 12)):
        assert '같다' not in json.dumps(v, ensure_ascii=False)


# ---------------------------------------------------------------------- 갈래 고르기 · 사건
def _arm(root, tag, mode, exp='C3', pair=1, label='dist_alive', **m):
    d = root / tag
    d.mkdir(parents=True, exist_ok=True)
    (d / 'metrics.json').write_text(json.dumps(dict({'mode': mode, 'exp': exp, 'pair': pair,
                                                     'label': label, 'valid': True}, **m)))
    return d


def test_select_arms_uses_done_files_only_and_filters_mode(tmp_path):
    (tmp_path / 'pairs').mkdir()
    for k in (1, 2, 10):
        _arm(tmp_path, f'p5_C2_p{k}_central', 'central', 'C2', k, 'central')
        _arm(tmp_path, f'p5_C2_p{k}_distributed', 'distributed', 'C2', k, 'distributed')
        (tmp_path / 'pairs' / f'C2_p{k}.done').write_text(json.dumps(
            {'tags': {'0': f'p5_C2_p{k}_central', '1': f'p5_C2_p{k}_distributed'}}))
    (tmp_path / 'pairs' / 'C2_p3.done.pre_fix_x').write_text(json.dumps(
        {'tags': {'0': 'p5_C2_p3_central', '1': 'p5_C2_p3_distributed'}}))
    got = B.select_arms(str(tmp_path), B.parse_sets('C2:distributed'))
    assert got == [('C2', 'p5_C2_p1_distributed'), ('C2', 'p5_C2_p2_distributed'),
                   ('C2', 'p5_C2_p10_distributed')]                # 짝 번호 수 순, 고치기 전 짝 없음
    assert len(B.select_arms(str(tmp_path), B.parse_sets('C2'))) == 6
    assert B.select_arms(str(tmp_path), B.parse_sets('C3')) == []


def test_zero_distance_blocks_counts_only_blocking_at_distance_zero():
    def gate(blocking, first):
        return {'k': 'agent_event', 'r': 'a',
                'ev': {'event': 'gate', 'blocking': blocking, 'first_entry_m': first}}
    ev = [gate(['zone:z'], 0.0), gate(['zone:z'], 1.4), gate([], 0.0), gate(['zone:?pose'], None)]
    assert B.zero_distance_blocks(ev) == 1


def test_arm_row_classifies_an_unrequested_graze(tmp_path):
    """허가 없는 로봇이 요청 없이 스친 원판 사건 → 기제 '요청 없음' (P5 리뷰 재확인 도구와 같은 분류)."""
    import p5_analyze as A
    gt = track('amr_01', 100, 120, 27.5, 17.0, 27.5, 17.0, v=0.0) + \
        track('amr_02', 100, 120, 26.6, 15.0, 26.6, 19.0)
    held = [{'k': 'agent_state', 'r': 'amr_01', 't': 99.0, 'held': ['zone:dock_a_approach']},
            {'k': 'agent_state', 'r': 'amr_02', 't': 99.0, 'held': []}]
    d = write_run(tmp_path, gt, held, t_base=100.0, t_end=120.0)
    m = A.run_metrics(str(d))
    (d / 'metrics.json').write_text(json.dumps(m))
    assert len(m['occupancy_disc']) == 1
    row = B.arm_row(str(tmp_path), 'C1', d.name)
    assert row['disc'] == 1 and [x['mech'] for x in row['mech']] == ['요청 없음']
    assert row['mech'][0]['robot'] == 'amr_02'


def test_f5l_not_worse_needs_fewer_or_equal_dock_events_and_no_completion_regression():
    per = {'central_new': {'fixed_disc': 2}, 'central_old': {'fixed_disc': 3}}
    flat = {'sign_p': 1.0, 'pos': 0, 'neg': 0}
    assert B.f5l_not_worse(per, flat) == (True, False)
    assert B.f5l_not_worse({'central_new': {'fixed_disc': 3}, 'central_old': {'fixed_disc': 3}},
                           flat) == (True, False)                     # 같으면 '나쁘지 않다' (≤)
    assert B.f5l_not_worse({'central_new': {'fixed_disc': 4}, 'central_old': {'fixed_disc': 3}},
                           flat) == (False, False)
    old_better = {'sign_p': 0.0078, 'pos': 8, 'neg': 0}               # b(옛) − a(새) > 0 8/8
    assert B.f5l_not_worse(per, old_better) == (False, True)
    new_better = {'sign_p': 0.0078, 'pos': 0, 'neg': 8}
    assert B.f5l_not_worse(per, new_better) == (True, False)
    assert B.f5l_not_worse({'central_new': {'fixed_disc': 0}}, flat)[0] is False   # 옛 자료 없음


def test_dock_counts_split_fixed_and_own_boxes():
    m = {'occupancy_dock_fixed_disc': [1, 2], 'occupancy_dock_fixed_center': [1],
         'occupancy_disc': [{'zone': 'dock_a_approach'}, {'zone': 'x_ab_4'}],
         'dock_n': 10, 'dock_within_2cm_1deg': 9}
    c = B.dock_counts(m)
    assert (c['fixed_disc'], c['fixed_center'], c['own_disc'], c['dock_n'], c['dock_within']) == \
        (2, 1, 1, 10, 9)


# ---------------------------------------------------------------------- 계획 · 드라이버 더하기
def test_sc2_is_the_c2_distributed_arm_replicated():
    c2d = P.EXPERIMENTS['C2']['arms'][1]
    e = P.EXPERIMENTS['SC2']
    assert e['pairs'] == 4 and [a['label'] for a in e['arms']] == ['dist_s2a', 'dist_s2b']
    for a in e['arms']:
        assert {k: v for k, v in a.items() if k != 'label'} == \
            {k: v for k, v in c2d.items() if k != 'label'}
    for k in ('tasks', 'horizon_s', 'min_end_s', 'lead_s'):
        assert e[k] == P.EXPERIMENTS['C2'][k], k
    assert 'SC2' not in P.ORDER                                       # P5 기본 순서는 그대로


def test_driver_additions_default_to_p5_behaviour():
    import p5_driver as D
    assert D.AGENT == 'b4-p5' and D.RUN_PREFIX == 'P5'
    assert D.mem_ok(0) is True and D.mem_avail_gb() > 0
    assert D.mem_ok(1e9) is False or D.mem_avail_gb() == float('inf')
    try:
        with pytest.raises(ValueError):                               # 잘못된 순서 낱말은 시작 전에 거부
            D.main(['--ws-new', 'x', '--ws-old', 'y', '--order', 'NOPE', '--agent', 'b4-p5b',
                    '--run-prefix', 'P5b'])
        assert D.AGENT == 'b4-p5b' and D.RUN_PREFIX == 'P5b'
    finally:
        D.AGENT, D.RUN_PREFIX = 'b4-p5', 'P5'
