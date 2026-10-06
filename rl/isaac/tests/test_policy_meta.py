"""ONNX 메타데이터가 학습 사실(라그랑주 승수·예산·BC)을 그대로 적는지 (torch 만, Isaac 불필요)."""

import json

from amr_isaac import policy_export as PE
from amr_rl_contract import constants as C

SPEC = '0,0.05,0.05,0.05,0.005/0.5,0.01,0.01,0.01,0.001/5.0'   # ppo_b_bc_fz 의 인자 그대로


def _rows(lams):
    return [{'iteration': 10 * (i + 1), 'lagrange': lam} for i, lam in enumerate(lams)]


def _summary(final, spec=SPEC):
    return {'lagrange': spec, 'lagrange_final': final, 'bc_steps': 200, 'seed': 0,
            'env_steps': 39321600}


# ppo_b_bc_fz curve.jsonl 의 λ 모양: 반복 50 에 거의 0, 60 부터 끝까지 0
FZ = _rows([[0, 0.04, 0.04, 0.04, 0.004], [0, 0.03, 0.03, 0.03, 0.003],
            [0, 0.02, 0.02, 0.02, 0.002], [0, 0.01, 0.01, 0.01, 0.001],
            [0, 0.00033, 0, 0, 0.00012]] + [[0, 0, 0, 0, 0]] * 25)


def test_facts_inactive_lagrange_like_ppo_b_bc_fz():
    """λ 가 반복 60 기록부터 끝까지 0 이면 그렇게 적고, d₁ 5.0 은 계약 훑기 밖이다."""
    f = PE.lagrange_facts(_summary([0.0] * 5), FZ)
    assert f['configured'] and not f['active_at_end']
    assert f['all_zero_from_iteration'] == 60
    assert f['d1'] == 5.0 and not f['d1_in_contract_sweep']


def test_facts_zero_streak_resets():
    """중간에 0 이 됐다가 다시 커지면 '부터 0' 은 마지막 연속 구간에서 센다 (끝이 0 이 아니면 None)."""
    rows = _rows([[0, 0.01, 0, 0, 0], [0] * 5, [0, 0.02, 0, 0, 0], [0] * 5, [0] * 5])
    assert PE.lagrange_facts(_summary([0.0] * 5), rows)['all_zero_from_iteration'] == 40
    rows = _rows([[0] * 5, [0, 0.02, 0, 0, 0]])
    f = PE.lagrange_facts(_summary([0, 0.02, 0, 0, 0]), rows)
    assert f['all_zero_from_iteration'] is None and f['active_at_end']


def test_facts_d1_in_contract_sweep():
    """계약 §7.2 의 훑기 값 {0.02, 0.05, 0.1, 0.2} 만 범위 안."""
    for d1, ok in ((0.02, True), (0.05, True), (0.1, True), (0.2, True), (5.0, False),
                   (0.3, False)):
        spec = SPEC.rsplit('/', 1)[0] + f'/{d1}'
        assert PE.lagrange_facts(_summary([0.0] * 5, spec), FZ)['d1_in_contract_sweep'] == ok


def test_algo_never_claims_inactive_lagrangian():
    """꺼진 라그랑주: 자동 algo 는 '제약 없는 PPO' 를 밝히고, 예전 문자열은 검사에서 걸린다."""
    f = PE.lagrange_facts(_summary([0.0] * 5), FZ)
    algo = PE.describe_algo('final', _summary([0.0] * 5), f)
    assert 'effectively unconstrained PPO' in algo and 'from iteration 60' in algo
    assert 'outside contract 7.2 sweep' in algo
    assert PE.algo_claim_ok(algo, f)
    assert not PE.algo_claim_ok('BC warm start + PPO (rsl_rl 3.1.2), Lagrangian penalty', f)
    assert PE.algo_claim_ok('BC warm start + PPO (rsl_rl 3.1.2)', f)


def test_algo_active_lagrangian():
    """끝 λ 가 0 이 아니면 PPO-Lagrangian 이라 적고 검사도 통과한다."""
    s = _summary([0, 0.3, 0, 0, 0], SPEC.rsplit('/', 1)[0] + '/0.05')
    f = PE.lagrange_facts(s, _rows([[0, 0.3, 0, 0, 0]]))
    algo = PE.describe_algo('final', s, f)
    assert 'PPO-Lagrangian' in algo and 'in contract 7.2 sweep' in algo
    assert PE.algo_claim_ok('PPO, Lagrangian penalty', f)


def test_metadata_final_and_bc():
    """최종 정책은 라그랑주 사실을 메타데이터에, BC 정책은 RL 단계 0 과 시연 표본 수를."""
    s = _summary([0.0] * 5)
    f = PE.lagrange_facts(s, FZ)
    m = PE.onnx_metadata('final', s, f, 'abc', C.CONTRACT_VERSION)
    for k in ('contract_version', 'arm', 'obs_dim', 'dt', 'normalization', 'algo'):
        assert isinstance(m[k], str) and m[k]
    assert json.loads(m['lagrange_final']) == [0.0] * 5
    assert m['lagrange_all_zero_from_iteration'] == '60'
    assert m['intervention_budget_d1'] == '5'
    assert m['intervention_budget_in_contract_sweep'] == 'false'
    assert m['train_steps'] == '39321600'
    b = PE.onnx_metadata('bc', s, f, 'abc', C.CONTRACT_VERSION, bc_info={'bc_samples': 409600})
    assert b['train_steps'] == '0' and b['bc_samples'] == '409600'
    assert 'no RL fine-tuning' in b['algo']
    assert not any(k.startswith('lagrange') for k in b)
    assert all(isinstance(v, str) for v in list(m.values()) + list(b.values()))
