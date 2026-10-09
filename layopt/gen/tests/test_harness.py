"""
P5 하네스의 layopt 확장 회귀 (tests/integration/fleet_p5).

기존 실험 계획 · 드라이버 기본값은 그대로, LO 계획 형식, 묶음 확인 (표준 라이브러리만).
"""

import importlib.util
import json
import subprocess
import sys

import pytest

from layopt.gen.common import REPO

P5 = REPO / 'tests' / 'integration' / 'fleet_p5'
sys.path.insert(0, str(P5))
import p5_driver  # noqa: E402
import p5_plan  # noqa: E402


def _plan_at_commit(rev):
    text = subprocess.run(['git', '-C', str(REPO), 'show', f'{rev}:tests/integration/fleet_p5/'
                           'p5_plan.py'], capture_output=True, text=True)
    if text.returncode != 0:
        pytest.skip('git 기록을 읽을 수 없다 (작업 트리 밖)')
    spec = importlib.util.spec_from_loader('p5_plan_old', loader=None)
    mod = importlib.util.module_from_spec(spec)
    exec(compile(text.stdout, 'p5_plan_old', 'exec'), mod.__dict__)
    return mod


def test_existing_experiments_unchanged():
    old = _plan_at_commit('1268431')            # layopt 확장 직전
    assert old.ORDER == p5_plan.ORDER
    for exp in old.EXPERIMENTS:
        for arm in range(len(old.EXPERIMENTS[exp]['arms'])):
            a = old.run_plan(exp, 1, arm, 60.0)
            b = p5_plan.run_plan(exp, 1, arm, 60.0)
            assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True), exp
    assert old.schedule() == p5_plan.schedule()


def test_lo_plan():
    plan = p5_plan.run_plan('LO', 1, 0, 60.0)
    assert plan['horizon_s'] == 900.0 and plan['end_when_done'] is False
    offs = [t['offset_s'] for t in plan['tasks']]
    assert offs == [0.0] * 5 + [300.0] * 5
    assert len({t['task_id'] for t in plan['tasks']}) == 10
    first = [(t['pickup_dock'], t['dropoff_dock']) for t in plan['tasks'][:5]]
    second = [(t['pickup_dock'], t['dropoff_dock']) for t in plan['tasks'][5:]]
    assert first == second == p5_plan.PLAN_PAIRS[:5]
    assert p5_plan.run_plan('LOd', 1, 0, 60.0)['mode'] == 'distributed'
    assert 'LO' not in p5_plan.ORDER and 'LOsmoke' not in p5_plan.ORDER


def test_driver_defaults_unchanged(tmp_path):
    ap_args = ['--raw', str(tmp_path), '--ws-new', 'x', '--ws-old', 'y']
    captured = {}

    class Fake(p5_driver.Driver):
        def run(self):
            captured['a'] = self.a
            captured['vols'] = self.bundle_docker()
            return 0

    orig = p5_driver.Driver
    p5_driver.Driver = Fake
    try:
        assert p5_driver.main(ap_args) == 0
    finally:
        p5_driver.Driver = orig
    a = captured['a']
    assert a.agent_id == 'b4-p5' and a.partition == '' and a.bundle == '' and a.run_one == ''
    assert a.notebook == 'docs/research/fleet-distributed/notebook' and a.run_id_prefix == 'P5_'
    assert a.stack_stagger == '' and a.record_agent == ''
    assert captured['vols'] == ([], [])


def test_check_bundle(tmp_path):
    b = tmp_path / 'v10-v20-v30-v40-v50'
    (b / 'config').mkdir(parents=True)
    for f in p5_driver.BUNDLE_FILES:
        (b / f).write_text('x')
    (b / 'spec.json').write_text(json.dumps({'spec_id': 'v10-v20-v30-v40-v50'}))
    (b / 'valid.json').write_text(json.dumps({'valid': False, 'reasons': ['lane:x']}))
    with pytest.raises(ValueError):
        p5_driver.check_bundle(str(b))
    (b / 'valid.json').write_text(json.dumps({'valid': True, 'reasons': []}))
    assert p5_driver.check_bundle(str(b))['spec_id'] == 'v10-v20-v30-v40-v50'
    (b / 'SEALED').unlink()
    with pytest.raises(ValueError):
        p5_driver.check_bundle(str(b))
