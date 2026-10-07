"""
DR 축 목록·교란 묶음 시험 (amr_isaac.perturb, Isaac 불필요).

- perturb.NOMINAL·TRAIN_DR_RANGES = env_cfg.py DomainRandCfg 원문
  (env_cfg 는 isaaclab 을 import 하므로 AST 로 읽는다)
- 커밋된 교란 묶음 파일이 규칙을 지킨다: 'unseen' 은 한 축 이상 학습 범위 밖, 'edge' 는 모두 안, 지연은 격자·버퍼 안
- validate_suite 가 규칙 위반을 잡는다 (가드마다 한 사례)
- env.py 의 _apply_perturbation 이 모든 축을 다룬다 (결선), 지연 양자화가 env 와 같다
"""

import ast
import copy
import json
import os

import numpy as np
import pytest

from amr_isaac import perturb as PT

HERE = os.path.dirname(os.path.abspath(__file__))
ISAAC = os.path.dirname(HERE)
SUITE = os.path.join(ISAAC, 'eval_sets', 'b1_perturb_suite_v1.json')
ENV_CFG = os.path.join(ISAAC, 'amr_isaac', 'env_cfg.py')
ENV_PY = os.path.join(ISAAC, 'amr_isaac', 'env.py')


def _dr_defaults():
    """env_cfg.py 의 DomainRandCfg 클래스 본문 기본값 (리터럴만)."""
    tree = ast.parse(open(ENV_CFG, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == 'DomainRandCfg':
            out = {}
            for st in node.body:
                if isinstance(st, ast.AnnAssign) and st.value is not None:
                    try:
                        out[st.target.id] = ast.literal_eval(st.value)
                    except ValueError:
                        pass
            return out
    raise AssertionError('DomainRandCfg 없음')


def test_nominal_and_ranges_match_env_cfg():
    """명목값·학습 범위가 env_cfg 원문과 같다 (설정이 바뀌면 깨진다)."""
    d = _dr_defaults()
    assert PT.NOMINAL['plant_time_constant_s'] == d['nominal_plant_time_constant_s']
    assert PT.NOMINAL['plant_delay_s'] == d['nominal_plant_delay_s']
    assert PT.NOMINAL['lidar_sigma_m'] == d['nominal_lidar_sigma_m']
    assert PT.NOMINAL['accel_limit_scale'] == d['nominal_accel_limit_scale']
    assert PT.NOMINAL['beam_dropout_p'] == d['nominal_beam_dropout_p']
    assert PT.TRAIN_DR_RANGES['plant_time_constant_s'] == d['plant_time_constant_s']
    assert PT.TRAIN_DR_RANGES['plant_delay_s'] == d['plant_delay_s']
    assert PT.TRAIN_DR_RANGES['accel_limit_scale'] == d['accel_limit_scale']
    assert PT.TRAIN_DR_RANGES['lidar_sigma_m'] == d['lidar_sigma_m']
    assert PT.TRAIN_DR_RANGES['beam_dropout_p'] == d['beam_dropout_p']
    assert d['enable'] is True


def test_committed_suite_valid_and_fixed():
    """커밋된 묶음: 검증 통과, 15 조건 (nominal 1 · unseen 13 · edge 1), 파일의 train_dr_ranges = 코드."""
    doc = PT.load_suite(SUITE)
    groups = [c['group'] for c in doc['conditions']]
    assert groups.count('nominal') == 1 and groups.count('unseen') == 13 and \
        groups.count('edge') == 1
    assert {k: tuple(v) for k, v in doc['train_dr_ranges'].items()} == PT.TRAIN_DR_RANGES
    assert doc['per_condition_n'] >= 1024
    assert 'NOT COVERED' in doc['contract_11_coverage']['1_map_diversity']
    for c in doc['conditions']:
        if c['group'] == 'unseen':
            assert any(PT.outside_train_range(k, v) for k, v in c['params'].items()), c['name']


@pytest.mark.parametrize('mut, needle', [
    (lambda d: d.update(format='x'), 'format'),
    (lambda d: d['conditions'].append(dict(d['conditions'][1])), '중복'),
    (lambda d: d['conditions'][1].update(group='weird'), 'group'),
    (lambda d: d['conditions'][1]['params'].update(foo=1.0), '모르는 축'),
    (lambda d: d['conditions'][1].update(params={'plant_delay_s': 0.06}), 'unseen 인데'),
    (lambda d: d['conditions'][-1]['params'].update(lidar_sigma_m=0.2), 'edge 인데'),
    (lambda d: d['conditions'][0].update(params={'drive_gain': 1.1}), 'nominal 은'),
    (lambda d: d['conditions'][1].update(params={'plant_delay_s': 0.13}), '0.02 s 격자'),
    (lambda d: d['conditions'][1].update(params={'plant_delay_s': 0.30}), '0.02 s 격자'),
    (lambda d: d['conditions'][3].update(params={'obs_scan_delay_s': 0.15}), '0.1 s 격자'),
    (lambda d: d['conditions'][3].update(params={'obs_scan_delay_s': 0.5}), '0.1 s 격자'),
    (lambda d: d['conditions'][9].update(params={'drive_gain': -1.0}), '≤ 0'),
    (lambda d: d['conditions'][7].update(params={'beam_dropout_p': 1.5}), '범위'),
])
def test_validate_suite_catches(mut, needle):
    """가드마다 위반 사례 하나 — 문제 목록에 그 가드의 문구가 나온다."""
    doc = json.load(open(SUITE, encoding='utf-8'))
    bad = copy.deepcopy(doc)
    mut(bad)
    errs = PT.validate_suite(bad)
    assert any(needle in e for e in errs), errs


def test_override_arrays_and_quantization():
    """조건 번호 → 축별 값 (지정 안 한 축 = 명목값), 지연 양자화 = env (반올림·clamp)."""
    doc = PT.load_suite(SUITE)
    conds = doc['conditions']
    idx = np.array([0, 1, 13, 14, 0])
    arr = PT.override_arrays(conds, idx)
    assert set(arr) == set(PT.KEYS)
    assert arr['plant_delay_s'].tolist() == [0.04, 0.12, 0.12, 0.08, 0.04]
    assert arr['drive_gain'].tolist() == [1.0, 1.0, 0.9, 1.0, 1.0]
    assert PT.delay_steps(arr['plant_delay_s'], 10).tolist() == [2, 6, 6, 4, 2]
    assert PT.delay_steps([0.0, 0.5], 10).tolist() == [1, 10]
    assert PT.obs_delay_steps([0.0, 0.1, 0.2, 0.9], 3).tolist() == [0, 1, 2, 3]


def test_env_applies_every_axis():
    """env.py _apply_perturbation 이 perturb.KEYS 의 모든 축 이름을 다룬다 (결선 누락 방지)."""
    tree = ast.parse(open(ENV_PY, encoding='utf-8').read())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and
              n.name == '_apply_perturbation')
    names = {n.value for n in ast.walk(fn) if isinstance(n, ast.Constant) and
             isinstance(n.value, str)}
    assert set(PT.KEYS) <= names, set(PT.KEYS) - names
