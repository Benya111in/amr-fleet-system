"""
multi_robot.launch.py fleet_mode (b4-wire) — 분산 모드를 켜는 인자 하나 (amr_bringup 이 설치돼 있을 때만).

launch_utils 는 하위 런치에 fleet_mode 를 따로 넘기지 않는다 — fleet_manager.launch.py 와 behavior.launch.py 가
상위 런치의 launch 인자를 상속받는다 (Humble 의 include 는 부모 범위의 설정을 그대로 본다). 그래서 multi_robot 은
(1) fleet_mode 를 선언하고 (2) 분산 모드면 정족수 멤버(스폰한 로봇)를 fleet_members 로 루트 범위에 둔다. 여기서는
그 두 가지와, 기본 central 이 예전과 같은 액션을 내는지(멤버 설정 없음)를 본다. 상속 자체는 Gazebo 기동에서
확인한다 (P5 요청 — 각 로봇 executor/assign_task · fleet_agent 노드가 있는지).
"""

import importlib.util
from pathlib import Path

import pytest

lu = pytest.importorskip('amr_bringup.launch_utils')
LAUNCH = Path(__file__).resolve().parents[2] / 'amr_bringup' / 'launch' / 'multi_robot.launch.py'
CONFIG = Path(__file__).resolve().parents[2] / 'amr_bringup' / 'config' / 'fleet_spawn.yaml'
pytestmark = pytest.mark.skipif(not LAUNCH.is_file(), reason='amr_bringup 소스가 없다')

from launch import LaunchContext  # noqa: E402
from launch.actions import (  # noqa: E402
    DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration,
)


@pytest.fixture
def prefix(tmp_path, monkeypatch):
    """가짜 ament prefix — 하위 런치 파일이 있는 것처럼 (amr_bringup 시험과 같은 방법)."""
    root = tmp_path / 'prefix'
    index = root / 'share' / 'ament_index' / 'resource_index' / 'packages'
    index.mkdir(parents=True)
    files = {'amr_simulation': ['warehouse.launch.py'],
             'amr_description': ['description.launch.py', 'spawn.launch.py'],
             'amr_fleet': ['fleet_manager.launch.py'], 'amr_dashboard': ['dashboard.launch.py'],
             'amr_evaluation': ['evaluation.launch.py']}
    for pkg, names in files.items():
        (index / pkg).write_text('')
        d = root / 'share' / pkg / 'launch'
        d.mkdir(parents=True)
        for n in names:
            (d / n).write_text('# fake\n')
    (root / 'share' / 'amr_fleet' / 'config').mkdir()
    (root / 'share' / 'amr_fleet' / 'config' / 'fleet.yaml').write_text('{}\n')
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(root))
    monkeypatch.setattr(lu, 'include_path', lambda path, args: (path, dict(args)))
    return root


def _setup(**args):
    spec = importlib.util.spec_from_file_location('multi_robot_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations.update({'robots_file': str(CONFIG)})
    ctx.launch_configurations.update({k: str(v) for k, v in args.items()})
    opaque = None
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            e.visit(ctx)
        elif isinstance(e, OpaqueFunction):
            opaque = e
    return ctx, opaque.visit(ctx)


def _members(ctx, actions):
    out = []
    for a in actions:
        if isinstance(a, SetLaunchConfiguration):
            a.visit(ctx)
            out.append(ctx.launch_configurations.get('fleet_members'))
    return out


def test_central_default_sets_no_fleet_members(prefix):
    ctx, actions = _setup()
    assert ctx.launch_configurations['fleet_mode'] == 'central'
    assert _members(ctx, actions) == []


def test_distributed_sets_members_to_the_spawned_robots(prefix):
    ctx, actions = _setup(fleet_mode='distributed', num_robots='4')
    assert isinstance(actions[0], SetLaunchConfiguration)      # 하위 런치 · 이벤트 처리기보다 먼저
    assert _members(ctx, actions) == ['amr_01,amr_02,amr_03,amr_04']
    assert ctx.launch_configurations['fleet_mode'] == 'distributed'   # 하위 런치가 상속받는 값


@pytest.mark.parametrize('args', [{'fleet_mode': 'mesh'},
                                  {'fleet_mode': 'distributed', 'num_robots': '2'}])
def test_bad_fleet_mode_or_too_few_robots_is_rejected(prefix, args):
    with pytest.raises(RuntimeError):
        _setup(**args)
