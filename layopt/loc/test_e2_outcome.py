"""
E2 판정·로그 해석 시험 — 로그 문자열은 P5 launch.log 실측 줄을 그대로 쓴다.

    cd /ros2_ws && python3 -m pytest -q layopt/loc/test_e2_outcome.py
"""

import math

from layopt.loc import e2_outcome as eo
import pytest

# /home/CAPYI_AI/runs/b24/b4-p5/raw/p5_AA_p1_dist_ar9/launch.log (amr_01) 에서 옮김
SEARCH = ('global seed search: 5 hypotheses in 0.34 s: (18.00, -16.02, 92 deg, 0.000 m, inlier '
          '0.923), (-17.90, 15.98, -88 deg, 0.010 m, inlier 0.888), (-26.00, -8.02, 2 deg, '
          '0.042 m, inlier 0.727), (26.00, 7.98, -178 deg, 0.052 m, inlier 0.720), '
          '(-5.90, 15.98, -88 deg, 0.082 m, inlier 0.692)')
SEED2 = 'seed #2: initialpose (-17.90, 15.98, -87 deg), score 0.010 m, inlier 0.888'
LOST = '[RECOVERING] LOST: scan-map inlier 0.47 < 0.50 for 6 scans'
GATE = ('[RECOVERING] alias margin blocks convergence: margin 0.035 < 0.050 '
        '(rejections 1, trust uses 9)')
GATE_NEG = ('[RECOVERING] alias margin blocks convergence: margin -0.035 < 0.050 '
            '(rejections 113, trust uses 9)')
BYPASS = ('[RECOVERING] alias margin bypassed by external trust: margin 0.042 < 0.050 '
          '(rejections 0, trust uses 1)')
REINIT = '[RECOVERING] global re-init #3'
RECOVERED = '[TRACKING] recovered in 16.9 s'
RESEED = '가설 #3 은 1 순위보다 인라이어가 많이 낮다 (별칭 의심) → 1 순위를 다시 심는다'
REG_LOST = '[RECOVERING] LOST: scan registration rejected 20/20 (1.00 > 0.80)'


@pytest.mark.parametrize('err,want', [(0.0, 'correct'), (0.499, 'correct'), (0.5, 'ambiguous'),
                                      (1.0, 'ambiguous'), (1.001, 'misconverge'),
                                      (48.0, 'misconverge'), (None, 'unknown'),
                                      (math.inf, 'unknown')])
def test_classify_boundaries(err, want):
    assert eo.classify(err) == want


def test_parse_search():
    ev = eo.parse_monitor_line(SEARCH)
    assert ev['ev'] == 'search' and ev['n'] == 5 and len(ev['hyps']) == 5
    assert ev['hyps'][1][:2] == [-17.9, 15.98]
    assert ev['hyps'][0][4] == pytest.approx(0.923)
    assert ev['hyps'][0][2] == pytest.approx(math.radians(92))


def test_parse_seed_lost_gate_reinit_recovered():
    s = eo.parse_monitor_line(SEED2)
    assert s == {'ev': 'seed', 'attempt': 2, 'x': -17.9, 'y': 15.98,
                 'yaw': pytest.approx(math.radians(-87)), 'score': 0.01, 'inlier': 0.888}
    assert eo.parse_monitor_line(LOST) == {'ev': 'lost',
                                           'reason': 'scan-map inlier 0.47 < 0.50 for 6 scans'}
    assert eo.parse_monitor_line(REG_LOST)['reason'].startswith('scan registration rejected')
    assert eo.parse_monitor_line(GATE) == {'ev': 'gate', 'bypassed': False, 'margin': 0.035}
    assert eo.parse_monitor_line(GATE_NEG)['margin'] == -0.035
    assert eo.parse_monitor_line(BYPASS)['bypassed'] is True
    assert eo.parse_monitor_line(REINIT) == {'ev': 'reinit', 'n': 3}
    assert eo.parse_monitor_line(RECOVERED) == {'ev': 'recovered', 'after_s': 16.9}
    assert eo.parse_monitor_line(RESEED) == {'ev': 'reseed_top1', 'attempt': 3}
    assert eo.parse_monitor_line('spin 360 deg requested') is None


def test_summarize_recovery_window():
    lines = [(99.0, REINIT), (100.0, LOST), (100.1, SEARCH),
             (100.2, 'seed #1: initialpose (18.00, -16.02, 92 deg), score 0.000 m, inlier 0.923'),
             (127.0, SEED2), (127.1, GATE), (127.2, '[RECOVERING] global re-init #2'),
             (150.0, GATE_NEG), (150.1, BYPASS), (160.0, RECOVERED)]
    events = [(t, eo.parse_monitor_line(m)) for t, m in lines]
    s = eo.summarize_recovery(events, 100.0, 160.0)
    assert s['n_reinit'] == 2           # 99.0 의 #3 은 창 밖
    assert s['n_seed'] == 2
    assert [x['attempt'] for x in s['seeds']] == [1, 2]
    assert s['gate_margins'] == [0.035, -0.035]
    assert s['gate_bypassed'] == 1
    assert s['lost_reason'].startswith('scan-map inlier')
    assert len(s['searches']) == 1 and s['searches'][0]['t'] == pytest.approx(0.1)
    assert eo.seed_is_true(s['seeds'][0], (18.0, -16.0, 1.57))
    assert not eo.seed_is_true(s['seeds'][1], (18.0, -16.0, 1.57))


def test_track_lookup():
    tr = eo.Track()
    for i in range(10):
        tr.add(i * 0.1, i, 0.0, 0.0)
    tr.add(0.05, 99, 0, 0)               # 시각이 뒤로 가면 무시
    assert len(tr) == 10
    assert tr.at_or_after(0.25)[1][0] == 3
    assert tr.nearest(0.26)[1][0] == 3
    assert tr.nearest(0.24)[1][0] == 2
    assert tr.at_or_after(5.0) is None   # 1 s 안에 없음
    assert tr.at_or_after(1.5)[1][0] == 9  # 뒤에 없으면 1 s 안 직전 표본


def test_pose_error_wraps_yaw():
    d, a = eo.pose_error((1.0, 0.0, math.pi - 0.01), (0.0, 0.0, -math.pi + 0.01))
    assert d == pytest.approx(1.0)
    assert a == pytest.approx(0.02)
    assert eo.pose_error(None, (0, 0, 0)) == (None, None)
