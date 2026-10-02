"""
하네스 ↔ 워크스페이스 계약 (launch 인자 선언, 시나리오 요구사항의 존재).

넘기는 launch 인자가 실제로 선언돼 있고, 시나리오가 요구하는 런치·실행 파일·설정이 머지된 트리에 있는가
(없으면 인자는 조용히 무시되고, 시나리오는 영원히 skip 된다).
"""

import ast
import importlib.util
from pathlib import Path
import re
import sys
import unittest

from amr_itest import catalog, config, stack as stack_mod
from amr_itest import requirements as req
from amr_itest.scenario import Context, GAZEBO, KINEMATIC, gazebo_requirements
import pytest

REPO = Path(__file__).resolve().parents[3]
SRC = REPO / 'src'
HARNESS = Path(__file__).resolve().parents[1]


def _launch_path(pkg: str, name: str) -> Path:
    """설치된 런치 파일, 없으면 소스 트리 (빌드 전 워크스페이스에서도 계약을 본다)."""
    installed = req.launch_file(pkg, name)
    return installed if installed is not None else SRC / pkg / 'launch' / name


def declared_arguments(pkg: str, name: str) -> set:
    """런치 파일의 generate_launch_description() 이 선언하는 DeclareLaunchArgument 이름."""
    from launch.actions import DeclareLaunchArgument
    if req.share_dir(pkg) is None and str(SRC / pkg) not in sys.path:
        sys.path.insert(0, str(SRC / pkg))
    path = _launch_path(pkg, name)
    spec = importlib.util.spec_from_file_location(f'_contract_{pkg}_{path.stem}', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    return {e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)}


def test_system_launch_declares_every_harness_argument():
    """Stack.system() 의 인자는 system.launch.py 가 모두 선언한다 (use_* 는 선언되지 않아 무시됐었다)."""
    declared = declared_arguments('amr_bringup', 'system.launch.py')
    assert set(stack_mod.SYSTEM_ARGS) <= declared, set(stack_mod.SYSTEM_ARGS) - declared
    assert not {a for a in declared if a.startswith('use_') and a != 'use_sim_time'}


def test_multi_robot_launch_declares_every_harness_argument():
    declared = declared_arguments('amr_bringup', 'multi_robot.launch.py')
    assert set(stack_mod.MULTI_ARGS) <= declared, set(stack_mod.MULTI_ARGS) - declared


def test_stack_switches_match_bringup():
    """하네스의 with_* ↔ 하위 런치 표가 bringup launch_utils 의 STACKS/GLOBALS 와 같다."""
    if req.share_dir('amr_bringup') is None:
        sys.path.insert(0, str(SRC / 'amr_bringup'))
    from amr_bringup import launch_utils
    stacks = {flag: (pkg, launch) for flag, pkg, launch, _ in launch_utils.STACKS}
    assert stacks == stack_mod.SYSTEM_SUBLAUNCH
    assert tuple(launch_utils.GLOBALS) == stack_mod.SYSTEM_GLOBALS


def _source_has_executable(pkg: str, exe: str) -> bool:
    root = SRC / pkg
    if (root / 'scripts' / exe).is_file():
        return True
    cmake = root / 'CMakeLists.txt'
    text = cmake.read_text(encoding='utf-8') if cmake.is_file() else ''
    word = re.escape(exe)
    if re.search(rf'add_executable\s*\(\s*{word}\b', text) or \
            re.search(rf'install\s*\(\s*TARGETS[^)]*\b{word}\b', text):
        return True
    setup = root / 'setup.py'
    return setup.is_file() and re.search(rf'[\'"]\s*{word}\s*=', setup.read_text()) is not None


def _check(kind: str, pkg: str, name: str) -> str:
    """요구사항 하나가 트리에 있는가 → 없으면 사유, 있으면 ''."""
    own = (SRC / pkg).is_dir()
    if kind == 'package':
        ok = own or req.has_package(pkg)
    elif kind == 'launch':
        ok = (SRC / pkg / 'launch' / name).is_file() if own else req.launch_file(pkg, name)
    elif kind == 'config':
        ok = (SRC / pkg / 'config' / name).is_file() if own else req.config_file(pkg, name)
    elif kind == 'executable':
        ok = _source_has_executable(pkg, name) if own else req.has_executable(pkg, name)
        if own and ok and req.has_package(pkg):      # 빌드된 워크스페이스면 설치본도 있어야 한다
            ok = req.has_executable(pkg, name)
    else:
        raise ValueError(kind)
    return '' if ok else f'{kind} {pkg}/{name}'


def _scenario_requirements(path: Path):
    """시나리오 파일의 req.<kind>('pkg', 'name') 와 stack.include('pkg', 'launch') 상수 호출."""
    out = []
    for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        args = [a.value for a in node.args if isinstance(a, ast.Constant)]
        owner = node.func.value.id if isinstance(node.func.value, ast.Name) else ''
        if owner == 'req' and node.func.attr in ('launch', 'executable', 'config') and \
                len(args) >= 2:
            out.append((node.func.attr, args[0], args[1]))
        elif owner == 'req' and node.func.attr == 'package' and args:
            out.append(('package', args[0], ''))
        elif node.func.attr == 'include' and len(args) >= 2:
            out.append(('launch', args[0], args[1]))
    return out


def test_every_scenario_requirement_exists_in_the_merged_tree():
    """
    시나리오·스택이 요구하는 것이 머지된 트리에 모두 있다.

    예전 12 는 amr_fleet/launch/multi_robot.launch.py(없음 — amr_bringup 에 있다)를 요구해 영원히 skip 했고,
    skip 은 통과로 셌다.
    """
    missing = []
    for s in catalog.SCENARIOS:
        path = HARNESS / f'test_{s.id}.py'
        found = _scenario_requirements(path)
        missing += [f'{s.id}: {m}' for m in (_check(*r) for r in found) if m]
    stack_reqs = ([('launch', *v) for v in stack_mod.SYSTEM_SUBLAUNCH.values()]
                  + [('launch', *stack_mod.SYSTEM_WORLD),
                     ('launch', 'amr_bringup', 'system.launch.py'),
                     ('launch', 'amr_bringup', 'multi_robot.launch.py')]
                  + [(r.kind, r.package, r.name) for r in gazebo_requirements()
                     if r.kind == 'launch']
                  + [('executable', pkg, exe) for pkg, exe, _ in stack_mod.REAL_NODES.values()])
    missing += [f'stack: {m}' for m in (_check(*r) for r in stack_reqs) if m]
    assert not missing, missing
    assert ('launch', 'amr_fleet', 'multi_robot.launch.py') not in [
        r for s in catalog.SCENARIOS for r in _scenario_requirements(HARNESS / f'test_{s.id}.py')]


def test_system_arguments_and_components(itest_env, monkeypatch, tmp_path):
    f = tmp_path / 'x.launch.py'
    f.write_text('')
    monkeypatch.setattr(req, 'launch_file', lambda pkg, name: f)
    ctx = Context(catalog.get(13))
    st = stack_mod.Stack(ctx, GAZEBO, 'system')
    st.system(use_behavior=True, pose=(-24.5, 15.0, 3.14159))
    args = st.system_args
    assert set(args) <= set(stack_mod.SYSTEM_ARGS)
    assert (args['x'], args['y']) == ('-24.5', '15.0') and args['yaw'] == '3.14159'
    assert args['map_yaml'] == str(config.repo_root() / 'maps' / 'warehouse.yaml')
    assert [args[k] for k in ('with_fleet', 'with_dashboard', 'with_evaluation')] == \
        ['false'] * 3
    assert args['with_behavior'] == 'true' and st.components['behavior'] == 'real'
    assert st.components['fleet'] == 'off' and st.spawn_pose == (-24.5, 15.0, 3.14159)
    assert st.drive_topic == 'cmd_vel_nav'
    with pytest.raises(ValueError, match='use_navigation'):
        stack_mod.Stack(Context(catalog.get(7)), GAZEBO, 'system').system(
            extra_args={'use_navigation': 'false'})
    loc = stack_mod.Stack(Context(catalog.get(4)), GAZEBO, 'system')
    loc.system(use_navigation=False, use_perception=False)
    assert loc.drive_topic == 'cmd_vel'           # safety_node 가 없으면 cmd_vel 은 비어 있다
    with pytest.raises(unittest.SkipTest):
        stack_mod.Stack(Context(catalog.get(4)), KINEMATIC, 'system').system()
    multi = stack_mod.Stack(Context(catalog.get(12)), GAZEBO, 'system')
    multi.multi_robot(5)
    assert set(multi.system_args) <= set(stack_mod.MULTI_ARGS)
    assert multi.system_args['num_robots'] == '5' and multi.components['fleet'] == 'real'
    assert multi.system_args['with_evaluation'] == 'false'


def test_drive_topic_for():
    assert stack_mod.drive_topic_for(True, True, True) == 'cmd_vel_nav'
    assert stack_mod.drive_topic_for(False, False, True) == 'cmd_vel_smoothed'
    assert stack_mod.drive_topic_for(False, False, False) == 'cmd_vel'


def test_standins_never_rejects_standin_only_components(itest_env, monkeypatch):
    """--standins never 는 AMCL·시뮬레이터·실행기 대역으로 합격하지 못한다 (예전 04 는 대역으로 통과)."""
    monkeypatch.setenv('ITEST_STANDINS', 'never')
    ctx = Context(catalog.get(4))
    with pytest.raises(unittest.SkipTest, match='simulator'):
        stack_mod.Stack(ctx, KINEMATIC).simulator()
    st = stack_mod.Stack(Context(catalog.get(4)), GAZEBO)
    monkeypatch.setattr(req, 'has_executable', lambda p, e: True)
    with pytest.raises(unittest.SkipTest, match='amcl'):
        st.localization(amcl=True)
    with pytest.raises(unittest.SkipTest, match='task_executor'):
        stack_mod.Stack(Context(catalog.get(13)), GAZEBO).task_executor()
    monkeypatch.setenv('ITEST_STANDINS', 'auto')
    auto = stack_mod.Stack(Context(catalog.get(4)), KINEMATIC)
    auto.standin_only('amcl', 'why')
    assert auto.components['amcl'] == stack_mod.STANDIN
