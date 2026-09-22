"""
multi_robot.launch.py / system.launch.py 조립 결과 (가짜 ament prefix 로 격리).

런치를 실제로 띄우지 않고 OpaqueFunction 이 만든 액션에서 include 대상·인자를 확인한다 (include_path 를
기록용으로 바꿔 끼운다 — Humble 의 LaunchDescriptionSource 는 불러오기 전에는 경로를 문자열로 주지 않는다).
가짜 prefix: 시뮬레이션·설명·fleet·dashboard·evaluation 은 런치 파일이 있고, amr_localization 은 런치 파일이
있고, amr_navigation 은 패키지만 있고(런치 없음), amr_perception·amr_behavior 는 색인에 없다.
"""

from collections import namedtuple
import importlib.util
from pathlib import Path

from amr_bringup import fleet_spawn, launch_utils as lu
from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.utilities import perform_substitutions
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG / 'config' / 'fleet_spawn.yaml'
FAKE_LAUNCH = {
    'amr_simulation': ['warehouse.launch.py'],
    'amr_description': ['description.launch.py', 'spawn.launch.py'],
    'amr_fleet': ['fleet_manager.launch.py'],
    'amr_dashboard': ['dashboard.launch.py'],
    'amr_evaluation': ['evaluation.launch.py'],
    'amr_localization': ['localization.launch.py'],
    'amr_navigation': [],
}
Included = namedtuple('Included', 'pkg file args')
FLEET_YAML = {'/fleet/fleet_manager_node': {'ros__parameters': {'comm_latency_ms': [0.0, 100.0]}},
              '/**/fleet_adapter_node': {'ros__parameters': {'comm_latency_ms': [0.0, 100.0]}}}


@pytest.fixture
def prefix(tmp_path, monkeypatch):
    root = tmp_path / 'prefix'
    index = root / 'share' / 'ament_index' / 'resource_index' / 'packages'
    index.mkdir(parents=True)
    for pkg, files in FAKE_LAUNCH.items():
        (index / pkg).write_text('')
        launch = root / 'share' / pkg / 'launch'
        launch.mkdir(parents=True)
        for name in files:
            (launch / name).write_text('# fake\n')
    (root / 'share' / 'amr_fleet' / 'config').mkdir()
    (root / 'share' / 'amr_fleet' / 'config' / 'fleet.yaml').write_text(yaml.safe_dump(FLEET_YAML))
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(root))
    monkeypatch.setattr(lu, 'include_path', lambda path, args: Included(
        Path(path).parts[-3], Path(path).name, {k: str(v) for k, v in args.items()}))
    return root


def _load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), PKG / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def setup(launch_name, **args):
    """런치 인자를 채우고 OpaqueFunction 을 실행해 (context, actions) 를 돌려준다."""
    ld = _load(launch_name).generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations.update({'robots_file': str(CONFIG)})
    ctx.launch_configurations.update({k: str(v) for k, v in args.items()})
    opaque = None
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
        elif isinstance(entity, OpaqueFunction):
            opaque = entity
    return ctx, opaque.visit(ctx)


def includes(_ctx, actions):
    """[(패키지, 런치 파일, {인자})] — 기록된 include 만."""
    return [a for a in actions if isinstance(a, Included)]


def by_file(incs, name):
    return [args for _, f, args in incs if f == name]


def logs(ctx, actions):
    return [perform_substitutions(ctx, a.msg) for a in actions if isinstance(a, LogInfo)]


def test_multi_robot_default_five(prefix):
    ctx, actions = setup('multi_robot.launch.py')
    incs = includes(ctx, actions)
    world = by_file(incs, 'warehouse.launch.py')
    assert world == [{'headless': 'true', 'world': 'warehouse.sdf', 'use_sim_time': 'true',
                      'spawn_robot': 'false'}]
    desc = by_file(incs, 'description.launch.py')
    assert [(d['robot_name'], d['prefix']) for d in desc] == [
        (f'amr_0{i}', f'amr_0{i}/') for i in range(1, 6)]
    assert by_file(incs, 'spawn.launch.py') == []            # 스폰은 월드 대기 뒤
    fleet = by_file(incs, 'fleet_manager.launch.py')
    assert fleet == [{'robot_ids': 'amr_01,amr_02,amr_03,amr_04,amr_05', 'use_sim_time': 'true'}]
    assert by_file(incs, 'dashboard.launch.py') == [{'use_sim_time': 'true'}]
    evals = by_file(incs, 'evaluation.launch.py')
    assert [(e['namespace'], e['with_cpu']) for e in evals] == [
        ('amr_01', 'true')] + [(f'amr_0{i}', 'false') for i in range(2, 6)]
    assert len({e['run_name'] for e in evals}) == 1 and evals[0]['run_name'].startswith('multi_')
    hold = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(hold) == 1
    cmd = [perform_substitutions(ctx, part) for part in hold[0].cmd]
    assert cmd[-4:] == ['hold', '--world', 'warehouse', '--pause']
    assert sum(isinstance(a, RegisterEventHandler) for a in actions) == 2


def test_multi_robot_flags_off(prefix):
    ctx, actions = setup('multi_robot.launch.py', num_robots=2, with_fleet='false',
                         with_dashboard='false', with_evaluation='false', headless='false',
                         use_sim_time='false', world='warehouse_small.sdf')
    incs = includes(ctx, actions)
    assert {f for _, f, _ in incs} == {'warehouse.launch.py', 'description.launch.py'}
    assert by_file(incs, 'warehouse.launch.py')[0]['headless'] == 'false'
    assert all(d['use_sim_time'] == 'false' for d in by_file(incs, 'description.launch.py'))
    hold = next(a for a in actions if isinstance(a, ExecuteProcess))
    assert 'warehouse_small' in [perform_substitutions(ctx, p) for p in hold.cmd]


@pytest.mark.parametrize('num', [0, 6, 'x'])
def test_multi_robot_bad_num_robots(prefix, num):
    with pytest.raises((RuntimeError, ValueError)):
        setup('multi_robot.launch.py', num_robots=num)


def test_required_package_missing(prefix):
    (prefix / 'share' / 'ament_index' / 'resource_index' / 'packages' / 'amr_dashboard').unlink()
    with pytest.raises(RuntimeError, match='amr_dashboard'):
        setup('multi_robot.launch.py')


def _opts(**flags):
    base = {name: True for name in lu.GLOBALS}
    base.update({flag: True for flag, _, _, _ in lu.STACKS})
    base.update(flags)
    return lu.Options(flags=base, eval_run_name='t')


def test_after_hold_spawns_all_then_release(prefix):
    spawn = fleet_spawn.load_fleet_spawn(str(CONFIG))
    ctx = LaunchContext()
    release = ExecuteProcess(cmd=['true'])
    out = lu.after_hold(0, spawn.robots, spawn, _opts(), release)
    assert out[-1] is release
    spawns = by_file(includes(ctx, out), 'spawn.launch.py')
    assert [(s['robot_name'], s['x'], s['y'], s['yaw']) for s in spawns] == [
        (r.name, repr(r.x), repr(r.y), repr(r.yaw)) for r in spawn.robots]
    assert all(s['world'] == 'warehouse' and s['z'] == '0.02' for s in spawns)
    assert includes(ctx, lu.after_hold(4, spawn.robots, spawn, _opts(), release))  # 멈춤 실패도 스폰
    for code in (3, -2, 1):
        skipped = lu.after_hold(code, spawn.robots, spawn, _opts(), release)
        assert len(skipped) == 1 and isinstance(skipped[0], LogInfo)


def test_after_release():
    called = []
    for code in (0, 2, 4):
        assert lu.after_release(code, lambda: called.append(code) or ['x']) == ['x']
    assert called == [0, 2, 4]
    assert isinstance(lu.after_release(-2, lambda: ['x'])[0], LogInfo)


def test_gate_commands_follow_pause_option():
    robot = {'name': 'amr_01', 'x': 0, 'y': 0, 'yaw': 0}
    spawn = fleet_spawn.parse_fleet_spawn({'robots': [robot], 'pause_during_spawn': False,
                                           'spawn_timeout_s': 30})
    hold, release = lu.gate_commands(spawn.robots, spawn, _opts())
    assert hold == ['hold', '--world', 'warehouse']
    assert release == ['release', '--world', 'warehouse', '--models', 'amr_01',
                       '--timeout', '30.0']


def test_stacks_included_only_when_installed(prefix):
    spawn = fleet_spawn.load_fleet_spawn(str(CONFIG))
    ctx = LaunchContext()
    out = lu.stack_actions(spawn.robots, True, _opts())
    loc = by_file(includes(ctx, out), 'localization.launch.py')
    assert [a['robot_name'] for a in loc] == fleet_spawn.robot_ids(spawn.robots)
    assert [a['start_map_server'] for a in loc] == ['true'] + ['false'] * 4
    first = loc[0]
    assert first['frame_prefix'] == first['prefix'] == 'amr_01/' and first['namespace'] == 'amr_01'
    assert (first['initial_x'], first['initial_y']) == ('18.0', '-16.0')
    assert first['waiting_pose'] == '18.0,-16.0,1.5708'
    text = '\n'.join(logs(ctx, out))
    assert 'navigation skipped: package not built yet (amr_navigation has no launch/' in text
    assert 'perception skipped: package not built yet (amr_perception not in ament index' in text
    assert 'behavior skipped' in text
    none = _opts(with_localization=False, with_navigation=False, with_perception=False,
                 with_behavior=False)
    off = lu.stack_actions(spawn.robots, True, none)
    assert off == []


def test_system_single_robot_defaults(prefix):
    ctx, actions = setup('system.launch.py')
    incs = includes(ctx, actions)
    assert by_file(incs, 'description.launch.py') == [
        {'robot_name': 'amr_01', 'prefix': '', 'use_sim_time': 'true'}]
    assert by_file(incs, 'warehouse.launch.py')[0]['spawn_robot'] == 'false'
    assert by_file(incs, 'fleet_manager.launch.py')[0]['robot_ids'] == 'amr_01'
    evals = by_file(incs, 'evaluation.launch.py')
    assert len(evals) == 1 and evals[0]['with_cpu'] == 'true'
    assert evals[0]['run_name'].startswith('system_')
    assert any('(18, -16, 1.5708)' in m for m in logs(ctx, actions))    # 스폰 목록의 amr_01 자세


def test_system_pose_override_and_unlisted_robot(prefix):
    ctx, actions = setup('system.launch.py', x='1.5', y='-2', yaw='0.3')
    assert any('(1.5, -2, 0.3)' in m for m in logs(ctx, actions))
    ctx, actions = setup('system.launch.py', robot_name='amr_09')
    assert any('amr_09 (0, 0, 0)' in m for m in logs(ctx, actions))
    ctx, actions = setup('system.launch.py', prefix='amr_01/')
    assert by_file(includes(ctx, actions), 'description.launch.py')[0]['prefix'] == 'amr_01/'
    with pytest.raises(RuntimeError, match='prefix'):
        setup('system.launch.py', prefix='robot/')


def test_per_robot_latency_reaches_fleet_params(prefix, tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    data['robots'][1]['comm_latency_ms'] = [20, 80]
    data['comm_latency_ms'] = 60
    robots_file = tmp_path / 'spawn.yaml'
    robots_file.write_text(yaml.safe_dump(data))
    ctx, actions = setup('multi_robot.launch.py', robots_file=robots_file, num_robots=3)
    fleet = by_file(includes(ctx, actions), 'fleet_manager.launch.py')[0]
    params = yaml.safe_load(Path(fleet['params_file']).read_text())
    Path(fleet['params_file']).unlink()
    per_robot = params['/amr_02/fleet_adapter_node']['ros__parameters']
    assert per_robot['comm_latency_ms'] == [20.0, 80.0]
    assert params['/**/fleet_adapter_node']['ros__parameters']['comm_latency_ms'] == [0.0, 60.0]
    assert params['/fleet/fleet_manager_node']['ros__parameters']['comm_latency_ms'] == [0.0, 60.0]
    assert 'comm_latency_ms' not in fleet                     # 인자로 주면 로봇별 값을 덮는다


def test_include_path_is_scoped(tmp_path):
    """실제 include_path: 범위 격리 GroupAction 안에 인자를 문자열로 넘기는 include 하나."""
    group = lu.include_path(str(tmp_path / 'x.launch.py'), {'robot_name': 'amr_02', 'prefix': ''})
    assert isinstance(group, GroupAction)
    subs = group.get_sub_entities()
    inc = [s for s in subs if isinstance(s, IncludeLaunchDescription)]
    assert len(inc) == 1
    assert list(inc[0].launch_arguments) == [('robot_name', 'amr_02'), ('prefix', '')]
    names = [type(s).__name__ for s in subs]
    assert names[0] == 'PushLaunchConfigurations' and names[-1] == 'PopLaunchConfigurations'


def test_truthy():
    assert lu.truthy(' True ') and not lu.truthy('0')
    with pytest.raises(ValueError):
        lu.truthy('maybe')
