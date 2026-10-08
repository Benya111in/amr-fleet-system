"""
분산 플릿 인자 (보너스 4 P5): ITEST_FLEET_MODE → multi_robot.launch.py 인자.

- 환경 변수가 없으면 인자를 하나도 더하지 않는다 (기존 시나리오 동작 그대로).
- distributed 면 실행마다 새 에이전트 저장소 · 관리자 epoch 파일 (옛 펜스가 새 관리자를 거부하지 않게).
- 하위 런치가 선언하는 상속 인자(fleet_store_dir · epoch_file)는 그 런치 파일에 실제로 선언돼 있다.
"""

from amr_itest import catalog, stack as stack_mod
from amr_itest import requirements as req
from amr_itest.scenario import Context, GAZEBO
import pytest
from test_launch_contract import declared_arguments


def test_no_env_adds_no_argument(tmp_path):
    assert stack_mod.fleet_launch_args({}, tmp_path, 's') == {}
    assert stack_mod.fleet_launch_args({'ITEST_FLEET_MODE': '  '}, tmp_path, 's') == {}


def test_central_passes_only_the_mode(tmp_path):
    assert stack_mod.fleet_launch_args({'ITEST_FLEET_MODE': 'Central'}, tmp_path, 's') == \
        {'fleet_mode': 'central'}


def test_distributed_gets_a_fresh_store_and_epoch_file_per_stamp(tmp_path):
    a = stack_mod.fleet_launch_args({'ITEST_FLEET_MODE': 'distributed'}, tmp_path, 'r1')
    b = stack_mod.fleet_launch_args({'ITEST_FLEET_MODE': 'distributed'}, tmp_path, 'r2')
    assert a['fleet_mode'] == 'distributed'
    assert a['fleet_store_dir'] == str(tmp_path / 'fleet_r1' / 'store')
    assert a['epoch_file'] == str(tmp_path / 'fleet_r1' / 'epoch.txt')
    assert a['fleet_store_dir'] != b['fleet_store_dir'] and a['epoch_file'] != b['epoch_file']


def test_explicit_store_and_epoch_paths_win(tmp_path):
    env = {'ITEST_FLEET_MODE': 'distributed', 'ITEST_FLEET_STORE_DIR': '/x/store',
           'ITEST_FLEET_EPOCH_FILE': '/x/epoch.txt'}
    got = stack_mod.fleet_launch_args(env, tmp_path, 's')
    assert got == {'fleet_mode': 'distributed', 'fleet_store_dir': '/x/store',
                   'epoch_file': '/x/epoch.txt'}


def test_unknown_mode_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='ITEST_FLEET_MODE'):
        stack_mod.fleet_launch_args({'ITEST_FLEET_MODE': 'peer'}, tmp_path, 's')


def test_inherited_fleet_arguments_are_declared_by_their_sublaunch():
    """상속 인자는 표의 하위 런치가 선언한다 (선언되지 않으면 조용히 무시된다)."""
    for name, (pkg, launch_file) in stack_mod.FLEET_INHERITED_ARGS.items():
        assert name in declared_arguments(pkg, launch_file), (name, pkg, launch_file)
    assert 'fleet_mode' in declared_arguments('amr_bringup', 'multi_robot.launch.py')
    assert set(stack_mod.FLEET_MODES) == {'central', 'distributed'}


def _multi(monkeypatch, tmp_path, env):
    f = tmp_path / 'x.launch.py'
    f.write_text('')
    monkeypatch.setattr(req, 'launch_file', lambda pkg, name: f)
    for k in ('ITEST_FLEET_MODE', 'ITEST_FLEET_STORE_DIR', 'ITEST_FLEET_EPOCH_FILE'):
        monkeypatch.delenv(k, raising=False)
    for k, v in env.items():
        monkeypatch.setenv(k, v)
    st = stack_mod.Stack(Context(catalog.get(12)), GAZEBO, 'system')
    st.multi_robot(5)
    return st


def test_multi_robot_without_env_is_unchanged(itest_env, monkeypatch, tmp_path):
    st = _multi(monkeypatch, tmp_path, {})
    assert 'fleet_mode' not in st.system_args and 'fleet_mode' not in st.components
    assert set(st.system_args) <= set(stack_mod.MULTI_ARGS)


def test_multi_robot_distributed_args(itest_env, monkeypatch, tmp_path):
    st = _multi(monkeypatch, tmp_path, {'ITEST_FLEET_MODE': 'distributed'})
    assert st.system_args['fleet_mode'] == 'distributed'
    assert st.components['fleet_mode'] == 'distributed'
    assert st.system_args['fleet_store_dir'].startswith(str(st.settings.log_root))
    assert st.system_args['epoch_file'].endswith('epoch.txt')
    assert set(st.system_args) <= set(stack_mod.MULTI_ARGS) | set(stack_mod.FLEET_INHERITED_ARGS)
