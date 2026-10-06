"""fleet_agents.launch.py 조립 시험 (런치를 띄우지 않고 OpaqueFunction 이 만든 프로세스 명령을 본다)."""

import importlib.util
from pathlib import Path

import pytest

try:
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
    HAVE_LAUNCH = True
except ImportError:
    HAVE_LAUNCH = False

pytestmark = pytest.mark.skipif(not HAVE_LAUNCH, reason='launch 가 없는 환경')

PKG = Path(__file__).resolve().parents[1]


def _procs(**args):
    spec = importlib.util.spec_from_file_location('fleet_agents_launch',
                                                  PKG / 'launch' / 'fleet_agents.launch.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.get_package_share_directory = lambda _: str(PKG)
    ld = mod.generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations.update({k: str(v) for k, v in args.items()})
    opaque = None
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            if e.name not in ctx.launch_configurations:
                e.visit(ctx)
        elif isinstance(e, OpaqueFunction):
            opaque = e
    return [a for a in opaque.visit(ctx) if isinstance(a, ExecuteProcess)]


def _cmd(p):
    return [''.join(s.perform(LaunchContext()) for s in part) for part in p.cmd]


def test_one_agent_per_robot_with_namespace_and_params():
    ps = _procs(robot_ids='amr_01,amr_02,amr_03')
    assert len(ps) == 3
    cmds = [_cmd(p) for p in ps]
    for r, c in zip(('amr_01', 'amr_02', 'amr_03'), cmds):
        assert c[1:3] == ['-m', 'amr_fleet.agent.node']
        assert f'__ns:=/{r}' in c
        assert 'members:=amr_01,amr_02,amr_03' in c
        assert any(x.endswith('fleet_agent.yaml') for x in c)
        assert any(x.startswith('zones_file:=') and x.endswith('traffic_zones.yaml') for x in c)


def test_flags_are_validated_and_passed():
    c = _cmd(_procs(robot_ids='amr_01', sim_execution='true', seed='7', members='amr_01,x,y')[0])
    assert 'sim_execution:=true' in c and 'seed:=7' in c and 'members:=amr_01,x,y' in c
    for bad in ({'sim_execution': 'yes'}, {'seed': '-1'}, {'robot_ids': ' '}):
        with pytest.raises(ValueError):
            _procs(**bad)


def test_config_file_parses_and_matches_node_parameters():
    import yaml
    from amr_fleet.agent.node import PARAMS
    cfg = yaml.safe_load((PKG / 'config' / 'fleet_agent.yaml').read_text(encoding='utf-8'))
    params = cfg['/**/fleet_agent']['ros__parameters']
    unknown = set(params) - set(PARAMS)
    assert not unknown, unknown
    assert params['modes.resync_s'] == 30.0 and params['detector.suspect_s'] == 1.5
