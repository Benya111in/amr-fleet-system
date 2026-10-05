"""
연구 계산 재현 — 새 구현이 보고서의 자체 계산과 같은 수를 내는지 고정한다.

출처: /home/CAPYI_AI/research/artifacts/distributed_fleet/ (저장소 밖, 읽기 전용)
- mrta_small.py      B 절: CBAA 동기 라운드(지연 U(0,100) ms·전역 종료 오라클·K=2), 이벤트 구동, 이탈
- verify_neg/verify_mrta2.py: 국소 확정 K·"이번 라운드 전원 직접 수신" 변형, 사례 1,000

재현 설정: 병합 규칙 'max'(연구 계산과 같은 최대 합의), SharedStreamChannel(같은 난수 소비 순서),
announce_commits=False(연구 계산에는 잠금 방송이 없다). 기대값은 그 스크립트의 출력
(mrta_small_out.txt, verify_mrta2_out.txt) 의 백분율을 사례 수로 되돌린 것이다.
연구 스크립트가 있으면 사례별 완전 일치도 확인한다 (없으면 skip — CI 에는 없다).
"""

import os
import pathlib
import random
import statistics
import sys

import pytest

from amr_fleet.negotiation import scenario
from amr_fleet.negotiation.agent import AgentConfig
from amr_fleet.negotiation.channel import SharedStreamChannel
from amr_fleet.negotiation.cost import EuclideanCost
from amr_fleet.negotiation.sim import assignment_indices, HOLDER, run_case, SimConfig

REF_DIR = pathlib.Path(os.environ.get(
    'NEGO_REFERENCE_DIR', '/home/CAPYI_AI/research/artifacts/distributed_fleet'))
COST = EuclideanCost(1.0)


def local_commit_run(nt, k, p, K, heard_all):
    """verify_mrta2.run 과 같은 조건 (사례 시드 55·nt + k, 메시지 시드 13·k + 1)."""
    case = scenario.make_case(nt, random.Random(55 * nt + k), 'scatter')
    cost = COST.matrix(case.robots, case.tasks)
    cfg = AgentConfig(rule='max', stable_rounds=K, require_all_heard=heard_all,
                      heartbeat_timeout=None, announce_commits=False)
    ch = SharedStreamChannel(random.Random(k * 13 + 1), p_loss=p)
    return run_case(cost, case.robot_ids, case.task_ids, cfg, ch, SimConfig(max_rounds=80))


def oracle_run(nt, case_seed, msg_seed, p, hb=None, kill=None, fixed=None, event=False):
    """mrta_small.cbaa_protocol 과 같은 조건 (지연 추첨, K=2 전역 종료 또는 고정 라운드)."""
    case = scenario.make_case(nt, random.Random(case_seed), 'scatter')
    cost = COST.matrix(case.robots, case.tasks)
    cfg = AgentConfig(rule='max', stable_rounds=2, require_all_heard=False, heartbeat_timeout=hb,
                      local_commit=False, announce_commits=False)
    ch = SharedStreamChannel(random.Random(msg_seed), p_loss=p, draw_delay=True)
    sim = SimConfig(max_rounds=fixed or 200, termination='fixed' if fixed else 'global',
                    event_driven=event)
    res = run_case(cost, case.robot_ids, case.task_ids, cfg, ch, sim,
                   dropouts={HOLDER: kill} if kill else None)
    return case, cost, res


# verify_mrta2_out.txt (사례 1,000, 손실 40 %): (작업 수, K, 전원 수신) → (중복 실행 수, 미완 실행 수, 평균 라운드)
LOCAL_COMMIT_040 = {
    (3, 2, False): (35, 0, 5.4), (3, 3, False): (4, 0, 6.4), (3, 5, False): (0, 0, 8.4),
    (3, 2, True): (0, 0, 17.9), (3, 3, True): (0, 0, 18.9),
    (5, 2, False): (26, 26, 6.7), (5, 3, False): (1, 1, 8.0), (5, 5, False): (0, 0, 10.1),
    (5, 2, True): (0, 0, 22.8), (5, 3, True): (0, 0, 23.9),
}


@pytest.mark.parametrize('key', sorted(LOCAL_COMMIT_040))
def test_local_commit_duplicates_match_research_1000_cases(key):
    nt, K, heard_all = key
    want_dup, want_inc, want_rounds = LOCAL_COMMIT_040[key]
    runs = [local_commit_run(nt, k, 0.4, K, heard_all) for k in range(1000)]
    assert sum(r.dup > 0 for r in runs) == want_dup
    assert sum(r.unassigned > 0 for r in runs) == want_inc
    assert round(statistics.mean(r.rounds for r in runs), 1) == want_rounds


# mrta_small_out.txt B 절 (사례 300, 과제 3·5·8): 손실 → 평균 (라운드, 방송, 벽시계 ms)
ORACLE = {
    (3, 0.0): (3.9, 19.4, 370), (3, 0.4): (5.4, 26.8, 804),
    (5, 0.0): (5.0, 24.9, 473), (5, 0.4): (6.9, 34.5, 1035),
    (8, 0.0): (4.5, 22.4, 426), (8, 0.4): (6.4, 31.8, 954),
}


@pytest.mark.parametrize('key', sorted(ORACLE))
def test_oracle_cbaa_equals_central_greedy_300_of_300(key):
    nt, p = key
    same = 0
    rounds, bcast, wall = [], [], []
    for k in range(300):
        case, cost, res = oracle_run(nt, 55 * nt + k, k * 13 + 1, p)
        same += assignment_indices(res, case.robot_ids, case.task_ids) == \
            scenario.central_greedy(cost)
        rounds.append(res.rounds)
        bcast.append(res.broadcasts)
        wall.append(res.wall_ms)
    assert same == 300
    want_r, want_b, want_w = ORACLE[key]
    assert round(statistics.mean(rounds), 1) == want_r
    assert round(statistics.mean(bcast), 1) == want_b
    assert round(statistics.mean(wall)) == want_w


def test_event_driven_negative_control_matches_research():
    """이벤트 구동 방송, 손실 40 %: 중복 실행 4.3 %(13/300)·4.7 %(14/300), 주기 재방송은 0."""
    for nt, want in ((3, 13), (5, 14)):
        ev = sum(oracle_run(nt, 55 * nt + k, k * 13 + 1, 0.4, event=True)[2].dup > 0
                 for k in range(300))
        per = sum(oracle_run(nt, 55 * nt + k, k * 13 + 1, 0.4)[2].dup > 0 for k in range(300))
        assert (ev, per) == (want, 0)


def test_holder_dropout_stuck_without_heartbeat_and_cleared_with_h3():
    """4 라운드째 보유 로봇 이탈, 30 라운드: 하트비트 없음 300/300 고착, H=3 이면 0/300."""
    for hb, want in ((None, 300), (3, 0)):
        stuck = sum(oracle_run(3, 99 + k, k, 0.0, hb=hb, kill=4, fixed=30)[2].stuck > 0
                    for k in range(300))
        assert stuck == want


@pytest.mark.skipif(not (REF_DIR / 'verify_neg' / 'verify_mrta2.py').is_file(),
                    reason='연구 스크립트 없음 (NEGO_REFERENCE_DIR)')
def test_case_by_case_identical_to_research_scripts():
    np = pytest.importorskip('numpy')
    assert np
    sys.path.insert(0, str(REF_DIR))
    try:
        import mrta_small as ref
    finally:
        sys.path.pop(0)
    src = (REF_DIR / 'verify_neg' / 'verify_mrta2.py').read_text()
    ns = {'__file__': str(REF_DIR / 'verify_neg' / 'verify_mrta2.py')}
    exec(compile(src.split('\nfor nt in (3, 5):')[0], 'verify_mrta2', 'exec'), ns)
    ref_run = ns['run']
    for nt in (3, 5):
        for K, ha in ((2, None), (3, 1)):
            for k in range(200):
                tasks, robots = ref.make_case(nt, random.Random(55 * nt + k), 'scatter')
                dd, uu, rr = ref_run(ref.cost_matrix(tasks, robots), random.Random(k * 13 + 1),
                                     0.4, K, ha, False)
                res = local_commit_run(nt, k, 0.4, K, ha is not None)
                assert ((res.dup > 0), (res.unassigned > 0), res.rounds) == (dd, uu, rr)
    for k in range(100):
        tasks, robots = ref.make_case(5, random.Random(55 * 5 + k), 'scatter')
        r = ref.cbaa_protocol(ref.cost_matrix(tasks, robots), random.Random(k * 13 + 1),
                              p_loss=0.2)
        case, cost, res = oracle_run(5, 55 * 5 + k, k * 13 + 1, 0.2)
        assert (r['rounds'], r['broadcasts'], r['dup']) == (res.rounds, res.broadcasts, res.dup)
        assert r['wall_ms'] == pytest.approx(res.wall_ms)
        assert r['assign'] == assignment_indices(res, case.robot_ids, case.task_ids)
