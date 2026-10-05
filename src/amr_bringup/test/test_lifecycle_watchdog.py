"""lifecycle_watchdog: 전이 응답 유실로 멈춘 노드를 직접 올린다 (모듈 설명)."""

from types import SimpleNamespace

from amr_bringup import launch_utils as lu
from amr_bringup.lifecycle_watchdog import LifecycleWatchdog, next_transition
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import pytest

rclpy = pytest.importorskip('rclpy')


def test_next_transition():
    assert next_transition(State.PRIMARY_STATE_UNCONFIGURED) == Transition.TRANSITION_CONFIGURE
    assert next_transition(State.PRIMARY_STATE_INACTIVE) == Transition.TRANSITION_ACTIVATE
    assert next_transition(State.PRIMARY_STATE_ACTIVE) is None
    assert next_transition(State.TRANSITION_STATE_CONFIGURING) is None   # 전이 중이면 기다린다
    assert next_transition(State.PRIMARY_STATE_FINALIZED) is None


class _FakeCalls:
    """call() 대역: 노드 이름마다 상태를 들고 있다가 change_state 로 한 단계 올린다."""

    def __init__(self, states, fail=()):
        self.states = dict(states)
        self.fail = set(fail)
        self.changes = []

    def __call__(self, srv_type, service, request):
        name = service.rsplit('/', 1)[0]
        if srv_type is GetState:
            state = self.states.get(name)
            if state is None:
                return None                     # 아직 안 뜬 노드
            labels = {State.PRIMARY_STATE_UNCONFIGURED: 'unconfigured',
                      State.PRIMARY_STATE_INACTIVE: 'inactive',
                      State.PRIMARY_STATE_ACTIVE: 'active'}
            return SimpleNamespace(current_state=SimpleNamespace(id=state, label=labels[state]))
        assert srv_type is ChangeState
        self.changes.append((name, request.transition.id))
        if name in self.fail:
            return SimpleNamespace(success=False)
        self.states[name] = (State.PRIMARY_STATE_INACTIVE
                             if request.transition.id == Transition.TRANSITION_CONFIGURE
                             else State.PRIMARY_STATE_ACTIVE)
        return SimpleNamespace(success=True)


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def watchdog():
    node = LifecycleWatchdog(parameter_overrides=[
        rclpy.parameter.Parameter('nodes', value=['/amr_01/amcl', '/amr_02/amcl', '/late/amcl']),
        rclpy.parameter.Parameter('grace', value=0.0),
        rclpy.parameter.Parameter('period', value=0.0)])
    yield node
    node.destroy_node()


def test_stuck_nodes_are_driven_to_active(watchdog, monkeypatch):
    calls = _FakeCalls({'/amr_01/amcl': State.PRIMARY_STATE_INACTIVE,      # 응답 유실로 멈춤
                        '/amr_02/amcl': State.PRIMARY_STATE_ACTIVE})       # 정상
    monkeypatch.setattr(watchdog, 'call', calls)
    watchdog.check_once()
    assert calls.changes == [('/amr_01/amcl', Transition.TRANSITION_ACTIVATE)]
    assert watchdog.pending == {'/amr_01/amcl', '/late/amcl'}              # active 는 빠진다
    watchdog.check_once()                                                  # 이제 active
    assert watchdog.pending == {'/late/amcl'}                              # 아직 안 뜬 노드만 남는다
    assert calls.changes == [('/amr_01/amcl', Transition.TRANSITION_ACTIVATE)]


def test_unconfigured_is_configured_then_activated_and_failures_retry(watchdog, monkeypatch):
    calls = _FakeCalls({'/amr_01/amcl': State.PRIMARY_STATE_UNCONFIGURED}, fail={'/amr_01/amcl'})
    monkeypatch.setattr(watchdog, 'call', calls)
    watchdog.check_once()
    assert calls.changes == [('/amr_01/amcl', Transition.TRANSITION_CONFIGURE)]
    calls.fail.clear()
    watchdog.check_once()                                                  # 실패는 다시 시도한다
    watchdog.check_once()                                                  # inactive → activate
    assert calls.changes[-1] == ('/amr_01/amcl', Transition.TRANSITION_ACTIVATE)
    watchdog.check_once()                                                  # active 확인 후 목록에서 빠진다
    assert watchdog.pending == {'/amr_02/amcl', '/late/amcl'}


def test_run_stops_when_all_active(watchdog, monkeypatch):
    monkeypatch.setattr(watchdog, 'call', _FakeCalls(
        {'/amr_01/amcl': State.PRIMARY_STATE_ACTIVE, '/amr_02/amcl': State.PRIMARY_STATE_ACTIVE,
         '/late/amcl': State.PRIMARY_STATE_ACTIVE}))
    ok = iter([True, False, False, False])     # 한 번만 검사하고 spin 루프는 돌지 않는다
    monkeypatch.setattr(rclpy, 'ok', lambda *a, **k: next(ok, False))
    watchdog.run(sleep=lambda _s: None)
    assert watchdog.pending == set()


def test_watched_nodes_follow_enabled_stacks():
    robots = [SimpleNamespace(name='amr_01'), SimpleNamespace(name='amr_02')]
    opts = SimpleNamespace(flags={'with_localization': True, 'with_navigation': True,
                                  'with_perception': True, 'with_behavior': True},
                           lifecycle_watchdog_grace=90.0)
    nodes = lu.watched_lifecycle_nodes(robots, opts)
    assert nodes[0] == '/map_server'
    assert '/amr_01/amcl' in nodes and '/amr_02/bt_navigator' in nodes
    assert len(nodes) == 1 + 2 * (1 + 4)          # map_server + 로봇마다 amcl + nav 4종
    assert set(nodes) == {'/map_server'} | {f'/{r}/{n}' for r in ('amr_01', 'amr_02')
                                            for n in ('amcl', 'planner_server',
                                                      'controller_server', 'behavior_server',
                                                      'bt_navigator')}
    opts.flags['with_navigation'] = False
    assert lu.watched_lifecycle_nodes(robots, opts) == ['/map_server', '/amr_01/amcl',
                                                        '/amr_02/amcl']
    opts.flags['with_localization'] = False
    assert lu.watched_lifecycle_nodes(robots, opts) == []
    assert lu.watchdog_action(robots, opts) == []                          # 볼 노드가 없으면 안 띄운다
