"""
navigation.launch.py 가 만드는 노드 구성.

통합 시험(5대 전체 기동)에서 lifecycle bond 4 s 가 부하에 끊겨 "Aborting bringup" 이 났다 →
localization.launch.py 와 같은 30 s. 그 밖에 cmd_vel 사슬 리맵(controller/behavior → cmd_vel_nav),
TTC BT 자동 선택, costmap_scan_filter_node 기동을 LaunchContext 로 _setup 을 실행해 확인한다.
"""
import importlib.util
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
REPO_CONFIG = PKG.parents[1] / 'config'


@pytest.fixture(scope='module')
def mod():
    pytest.importorskip('nav2_common')
    spec = importlib.util.spec_from_file_location(
        'nav_launch', PKG / 'launch' / 'navigation.launch.py')
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _nodes(mod, **overrides):
    # _setup 을 실행해 {노드 이름: (파라미터 dict, 리맵 목록, 파라미터 파일 수)} (Node 생성 인자를 평가)
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    ctx = LaunchContext()
    for k, v in overrides.items():
        ctx.launch_configurations[k] = v
    for e in mod.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            e.execute(ctx)

    def txt(x):
        return x if isinstance(x, str) else perform_substitutions(ctx, x)
    out = {}
    for a in mod._setup(ctx):
        if not isinstance(a, Node):
            continue
        params = {}
        n_files = 0
        for p in evaluate_parameters(ctx, a._Node__parameters or []):
            if isinstance(p, dict):
                params.update(p)
            else:
                n_files += 1
        remaps = [(txt(x), txt(y)) for x, y in (a._Node__remappings or [])]
        out[txt(a._Node__node_name)] = (params, remaps, n_files)
    return out


def test_bond_timeout_matches_localization(mod):
    assert mod.BOND_TIMEOUT_S == 30.0
    nodes = _nodes(mod, config_dir=str(REPO_CONFIG))
    p = nodes['lifecycle_manager_navigation'][0]
    assert p['bond_timeout'] == mod.BOND_TIMEOUT_S
    assert list(p['node_names']) == mod.LIFECYCLE_NODES
    loc = (PKG.parents[0] / 'amr_localization' / 'launch' / 'localization.launch.py')
    if loc.is_file():
        assert 'BOND_TIMEOUT_S = 30.0' in loc.read_text(encoding='utf-8')


def test_nodes_and_cmd_vel_chain(mod):
    nodes = _nodes(mod, config_dir=str(REPO_CONFIG), use_ttc_bt='false')
    assert set(nodes) == {'planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
                          'lifecycle_manager_navigation', 'velocity_profiler_node',
                          'costmap_scan_filter_node'}
    for name in ('controller_server', 'behavior_server'):
        assert ('cmd_vel', 'cmd_vel_nav') in nodes[name][1]
    bt = nodes['bt_navigator'][0]
    assert bt['default_nav_to_pose_bt_xml'].endswith('navigate_to_pose_no_ttc.xml')
    assert mod.TTC_BT_LIB not in bt['plugin_lib_names']


def test_ttc_bt_selection(mod):
    bt = _nodes(mod, config_dir=str(REPO_CONFIG), use_ttc_bt='true')['bt_navigator'][0]
    assert bt['default_nav_to_pose_bt_xml'].endswith('navigate_to_pose.xml')
    assert bt['default_nav_through_poses_bt_xml'].endswith('navigate_through_poses.xml')
    assert mod.TTC_BT_LIB in bt['plugin_lib_names']
    auto = _nodes(mod, config_dir=str(REPO_CONFIG), use_ttc_bt='auto')['bt_navigator'][0]
    want = 'navigate_to_pose.xml' if mod.ttc_plugin_available() else 'navigate_to_pose_no_ttc.xml'
    assert auto['default_nav_to_pose_bt_xml'].endswith(want)


def test_missing_robot_params_falls_back(mod, tmp_path):
    with_cfg = _nodes(mod, config_dir=str(REPO_CONFIG))['controller_server'][2]
    # robot_params.yaml 이 없으면 controller_server 는 템플릿 파라미터 파일만 받는다
    assert _nodes(mod, config_dir=str(tmp_path))['controller_server'][2] == with_cfg - 1
    assert mod._default_config_dir().endswith('config')
