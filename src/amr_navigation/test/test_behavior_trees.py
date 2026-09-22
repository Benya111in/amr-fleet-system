"""
BT 4 종의 재계획 규칙과 노드 라이브러리.

1 Hz 무조건 재계획은 지역 회피로 비낀 위치에서 목표로 곧장 가는 새 경로를 만들어 원래 경로로
돌아오지 않았다 (Gazebo 1.0 m/s 횡단 actor, docs/algorithms/costmap.md §6.3). 모든 BT 가 1 Hz 로
경로 유효성·목표 변경을 먼저 보고 막혔을 때만 계획하는지, RateController 가 주기를 잃는 반응형
부모(ReactiveFallback/ReactiveSequence) 아래에 있지 않은지, 그리고 쓰는 Nav2 BT 노드의 라이브러리를
navigation.launch.py 가 넘기는지 본다. 실제 BehaviorTree.CPP 로 tick 해서 재계획 횟수를 세는 시험은
test_bt_replan.cpp.
"""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PKG = Path(__file__).resolve().parents[1]
BT_DIR = PKG / 'behavior_trees'
BTS = sorted(BT_DIR.glob('*.xml'))

# BT 노드 → 라이브러리 (BehaviorTree.CPP v3 내장은 None)
NODE_LIBS = {
    'RecoveryNode': 'nav2_recovery_node_bt_node',
    'PipelineSequence': 'nav2_pipeline_sequence_bt_node',
    'RoundRobin': 'nav2_round_robin_node_bt_node',
    'RateController': 'nav2_rate_controller_bt_node',
    'PlannerSelector': 'nav2_planner_selector_bt_node',
    'ControllerSelector': 'nav2_controller_selector_bt_node',
    'ComputePathToPose': 'nav2_compute_path_to_pose_action_bt_node',
    'ComputePathThroughPoses': 'nav2_compute_path_through_poses_action_bt_node',
    'FollowPath': 'nav2_follow_path_action_bt_node',
    'ClearEntireCostmap': 'nav2_clear_costmap_service_bt_node',
    'GoalUpdated': 'nav2_goal_updated_condition_bt_node',
    'GlobalUpdatedGoal': 'nav2_globally_updated_goal_condition_bt_node',
    'IsPathValid': 'nav2_is_path_valid_condition_bt_node', 'Spin': 'nav2_spin_action_bt_node',
    'Wait': 'nav2_wait_action_bt_node', 'BackUp': 'nav2_back_up_action_bt_node',
    'RemovePassedGoals': 'nav2_remove_passed_goals_action_bt_node',
    'IsTTCBelowThreshold': 'amr_is_ttc_below_threshold_condition_bt_node',
    'ReactiveFallback': None, 'ReactiveSequence': None, 'Fallback': None, 'Sequence': None,
    'Inverter': None, 'ForceSuccess': None, 'BehaviorTree': None, 'root': None,
}


@pytest.fixture(scope='module')
def launch_mod():
    pytest.importorskip('nav2_common')
    spec = importlib.util.spec_from_file_location(
        'nav_launch', PKG / 'launch' / 'navigation.launch.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize('bt', BTS, ids=[b.name for b in BTS])
def test_replans_only_when_path_invalid(bt):
    root = ET.parse(bt).getroot()
    rates = root.findall('.//RateController')
    assert len(rates) == 1, bt.name
    fb = rates[0].find('Fallback')
    assert fb is not None, f'{bt.name}: 1 Hz 가지가 무조건 재계획'
    guard, plan = list(fb)
    assert guard.tag == 'ReactiveSequence'
    assert [c.tag for c in guard] == ['Inverter', 'IsPathValid']
    assert guard[0][0].tag == 'GlobalUpdatedGoal'
    assert plan.tag == 'RecoveryNode'
    assert (plan.find('.//ComputePathToPose') is not None
            or plan.find('.//ComputePathThroughPoses') is not None)


def _parents(root):
    return {c: p for p in root.iter() for c in p}


@pytest.mark.parametrize('bt', BTS, ids=[b.name for b in BTS])
def test_rate_controller_keeps_its_period(bt):
    # 반응형 부모는 자식 SUCCESS 때 형제를 halt → RateController 가 IDLE 로 돌아가 다음 tick 에
    # first_time 으로 다시 실행된다 (tick 마다 재계획, /plan 22 Hz). PipelineSequence 는 앞 자식을
    # halt 하지 않으므로 1 Hz 가지와 TTC 가지는 그 형제여야 한다.
    root = ET.parse(bt).getroot()
    parent = _parents(root)
    rate = root.find('.//RateController')
    assert parent[rate].tag == 'PipelineSequence', bt.name
    node = rate
    while node in parent:
        node = parent[node]
        assert not node.tag.startswith('Reactive'), f'{bt.name}: {node.tag} 아래 RateController'
    ttc = root.find('.//IsTTCBelowThreshold')
    if ttc is not None:
        seq = parent[ttc]
        assert seq.tag == 'Sequence' and parent[seq].tag == 'ForceSuccess'
        assert parent[parent[seq]] is parent[rate]      # 같은 PipelineSequence 의 형제
        kids = list(parent[rate])
        assert kids.index(parent[seq]) < kids.index(rate)
        if 'through_poses' in bt.name:
            # 1 Hz 가지가 계획하지 않으면 {goals} 에 지난 경유점이 남는다 → TTC 가지도 먼저 지운다
            assert [c.tag for c in seq][1] == 'RemovePassedGoals'


@pytest.mark.parametrize('bt', BTS, ids=[b.name for b in BTS])
def test_bt_nodes_have_plugin_libraries(bt, launch_mod):
    libs = set(launch_mod.NAV2_BT_LIBS) | {launch_mod.TTC_BT_LIB}
    for el in ET.parse(bt).getroot().iter():
        assert el.tag in NODE_LIBS, f'{bt.name}: 모르는 BT 노드 {el.tag}'
        lib = NODE_LIBS[el.tag]
        assert lib is None or lib in libs, \
            f'{bt.name}: {el.tag} 의 {lib} 를 launch 가 넘기지 않는다'
    if '_no_ttc' in bt.name:
        assert ET.parse(bt).getroot().find('.//IsTTCBelowThreshold') is None
