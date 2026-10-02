"""cases: 실행 중/종료 단계 크래시 구분, 러너 상한 예산, lifecycle 대기, map↔월드 정합 판정."""

from types import SimpleNamespace as NS
import unittest

from amr_itest import cases, catalog, config, worldmap
from amr_itest import requirements as req
from amr_itest.scenario import Context
import numpy as np
import pytest


def test_exit_helpers():
    events = [NS(process_name='a-1'), NS(process_name='b-2', returncode=-11),
              NS(process_name='gazebo-3', returncode=-9), NS(process_name='c-4', returncode=0)]
    assert cases.exit_events(events) == {'b-2': -11, 'gazebo-3': -9, 'c-4': 0}
    assert cases.bad_exits({'b-2': -11, 'gazebo-3': -9, 'c-4': 0, 'd': None}) == ['b-2=-11']


def test_crash_during_run_vs_at_shutdown(itest_env):
    """
    측정 중 죽은 프로세스는 실패, 측정이 끝난 뒤(종료 단계) 비정상 코드는 경고.

    예전에는 측정을 모두 통과한 04 가 robot_state_publisher 의 SIGINT 중 -11 로 실패했다.
    """
    ctx = Context(catalog.get(4)).begin()

    class Case(cases.ProbeCase):
        CTX = ctx

    case = Case('test_zz_processes_alive')
    Case.setUpClass()
    try:
        case.ctx = ctx
        case.test_zz_processes_alive([NS(process_name='rsp-1'),                  # 살아 있음
                                      NS(process_name='world_gate-2', returncode=0)])
        with pytest.raises(AssertionError):
            case.test_zz_processes_alive([NS(process_name='amcl-3', returncode=-6)])
        with pytest.raises(unittest.SkipTest):
            case.test_zz_processes_alive()
    finally:
        Case.tearDownClass()
    ctx.record.measure(cases.RUN_EXITS_KEY, {'world_gate-2': 0})
    after = cases.AfterShutdown('test_exit_codes')
    after.CTX = ctx
    after.test_exit_codes([NS(process_name='rsp-1', returncode=-11),       # 종료 단계 → 경고
                           NS(process_name='world_gate-2', returncode=0)])
    assert ctx.record.data['measurements']['shutdown_exit_warnings'] == ['rsp-1=-11']
    assert ctx.record.data['checks'][-1]['passed']
    ctx.record.measure(cases.RUN_EXITS_KEY, {'amcl-3': -6})
    with pytest.raises(AssertionError):                                   # 실행 중 크래시 → 실패
        after.test_exit_codes([NS(process_name='amcl-3', returncode=-6)])


def test_time_budget(itest_env, monkeypatch):
    """러너 상한(ITEST_SCENARIO_TIMEOUT) 안에 다음 시행을 끝낼 수 없으면 시작하지 않는다."""
    monkeypatch.setenv('ITEST_SCENARIO_TIMEOUT', '400')
    ctx = Context(catalog.get(8)).begin()

    class Case(cases.ProbeCase):
        CTX = ctx
    Case.setUpClass()
    try:
        case = Case('test_zz_processes_alive')
        case.ctx, case.settings = ctx, ctx.settings
        assert case.time_left() == pytest.approx(400 - cases.SHUTDOWN_MARGIN_S, abs=5.0)
        assert case.budget_for(100.0, 'trial 0')
        assert not case.budget_for(10000.0, 'trial 1')
        assert 'trial 1' in ctx.record.data['notes'][-1]
    finally:
        Case.tearDownClass()
    monkeypatch.delenv('ITEST_SCENARIO_TIMEOUT')
    assert Context(catalog.get(8)).settings.scenario_timeout == float('inf')


def test_wait_lifecycle_active(itest_env):
    """nav2 lifecycle_manager is_active (Trigger) 가 true 일 때만 통과, 아니면 판정 실패로 남긴다."""
    from std_srvs.srv import Trigger
    from amr_itest.probe import GraphProbe
    ctx = Context(catalog.get(6)).begin()
    server = GraphProbe('unit_lcm', namespace=ctx.settings.namespace)
    state = {'active': False}
    server.node.create_service(Trigger, 'lifecycle_manager_navigation/is_active',
                               lambda req_, res: setattr(res, 'success', state['active']) or res)

    class Case(cases.ProbeCase):
        CTX = ctx

        def test_x(self):
            with self.assertRaises(AssertionError):
                self.wait_lifecycle_active(['lifecycle_manager_navigation'], 1.5)
            state['active'] = True
            self.wait_lifecycle_active(['lifecycle_manager_navigation'], 10.0)

    try:
        result = unittest.TextTestRunner(verbosity=0).run(
            unittest.defaultTestLoader.loadTestsFromName('test_x', Case))
        assert result.wasSuccessful(), result.failures + result.errors
    finally:
        server.close()
    checks = [c for c in ctx.record.data['checks'] if c['name'].startswith('lifecycle')]
    assert [c['passed'] for c in checks] == [False, True]


def test_check_map_registration_on_world_raster(itest_env):
    """월드 visual 단면을 그대로 지도로 주면 항등 정합 통과, 0.2 m 옮기면 실패."""
    if req.share_dir('amr_simulation') is None:
        pytest.skip('amr_simulation 미설치 (빌드된 워크스페이스에서 돈다)')
    share = req.share_dir('amr_simulation')
    shapes = worldmap.footprints(share / 'worlds' / 'warehouse.sdf', share / 'models',
                                 config.scan_plane_height())
    res, origin, w, h = 0.05, (-31.0, -21.0), 1240, 840
    edges = np.vstack([s.perimeter(res) for s in shapes])

    def grid(dx):
        data = np.zeros((h, w), dtype=int)
        ix = np.floor((edges[:, 0] + dx - origin[0]) / res).astype(int)
        iy = np.floor((edges[:, 1] - origin[1]) / res).astype(int)
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        data[iy[ok], ix[ok]] = 100
        return NS(info=NS(resolution=res, width=w, height=h,
                          origin=NS(position=NS(x=origin[0], y=origin[1]))),
                  data=data.reshape(-1).tolist())

    ctx = Context(catalog.get(5)).begin()

    class Case(cases.ProbeCase):
        CTX = ctx

        def test_x(self):
            reg = self.check_map_registration(grid=grid(0.0))
            assert reg.identity_ok()
            with self.assertRaises(AssertionError):
                self.check_map_registration(grid=grid(0.2))

    result = unittest.TextTestRunner(verbosity=0).run(
        unittest.defaultTestLoader.loadTestsFromName('test_x', Case))
    assert result.wasSuccessful(), result.failures + result.errors
    checks = [c for c in ctx.record.data['checks'] if c['name'].startswith('map frame')]
    assert [c['passed'] for c in checks] == [True, False]


def test_wait_lifecycle_active_takes_the_wedge_shortcut_during_the_wait(itest_env):
    """
    멈춘 관리자의 노드가 모두 active 면 **상한을 다 태우지 않고** 통과한다.

    회귀 근거: 예전에는 폴백을 타임아웃이 지난 뒤에만 봤다. 시나리오 12 는 상한이
    600 × timeout_scale 3 = 1800 s 인데, lifecycle_watchdog 은 기동 +95 s 에 이미
    "감시 대상이 모두 active" 를 선언한다 (logs/X12) — 29 분을 아는 답에 썼다.
    """
    import time
    from std_srvs.srv import Trigger
    from amr_itest.probe import GraphProbe
    ctx = Context(catalog.get(6)).begin()
    server = GraphProbe('unit_lcm_wedge', namespace=ctx.settings.namespace)
    server.node.create_service(Trigger, 'lifecycle_manager_navigation/is_active',
                               lambda req_, res: setattr(res, 'success', False) or res)

    class Case(cases.ProbeCase):
        CTX = ctx

        # 관리자는 영원히 inactive 지만 그 노드는 전부 active 인 상황을 세운다
        def managed_nodes(self, manager):
            return ('/n1', '/n2')

        def lifecycle_state(self, node):
            return cases.LIFECYCLE_ACTIVE

        def test_x(self):
            t0 = time.monotonic()
            self.wait_lifecycle_active(['lifecycle_manager_navigation'], 60.0,
                                       wedge_grace=0.5, wedge_period=0.2)
            # 상한(60 × scale)을 태우지 않고 유예 직후에 빠져나와야 한다
            assert time.monotonic() - t0 < 10.0, '폴백이 대기 중에 안 걸렸다'

    try:
        result = unittest.TextTestRunner(verbosity=0).run(
            unittest.defaultTestLoader.loadTestsFromName('test_x', Case))
        assert result.wasSuccessful(), result.failures + result.errors
    finally:
        server.close()
    checks = [c for c in ctx.record.data['checks'] if c['name'].startswith('lifecycle')]
    assert [c['passed'] for c in checks] == [True]
    assert 'lifecycle_manager_wedged' in ctx.record.data['measurements']


def test_wait_lifecycle_active_does_not_shortcut_before_grace_or_with_a_down_node(itest_env):
    """유예 전에는 지름길을 타지 않고, 노드가 하나라도 active 가 아니면 통과하지 않는다."""
    from std_srvs.srv import Trigger
    from amr_itest.probe import GraphProbe
    ctx = Context(catalog.get(6)).begin()
    server = GraphProbe('unit_lcm_wedge2', namespace=ctx.settings.namespace)
    server.node.create_service(Trigger, 'lifecycle_manager_navigation/is_active',
                               lambda req_, res: setattr(res, 'success', False) or res)
    probed = {'n': 0}

    class Case(cases.ProbeCase):
        CTX = ctx

        def managed_nodes(self, manager):
            probed['n'] += 1
            return ('/n1', '/n2')

        def lifecycle_state(self, node):
            return cases.LIFECYCLE_ACTIVE if node == '/n1' else 1   # /n2 는 unconfigured

        def test_x(self):
            # (a) 유예가 상한보다 길면 폴백을 아예 보지 않는다
            with self.assertRaises(AssertionError):
                self.wait_lifecycle_active(['lifecycle_manager_navigation'], 1.0,
                                           wedge_grace=600.0, wedge_period=0.2)
            assert probed['n'] == 0, '유예 전에 노드 상태를 조회했다'
            # (b) 노드 하나가 active 가 아니면 폴백이 통과시키지 않는다
            with self.assertRaises(AssertionError):
                self.wait_lifecycle_active(['lifecycle_manager_navigation'], 2.0,
                                           wedge_grace=0.2, wedge_period=0.2)
            assert probed['n'] > 0, '유예 뒤에도 폴백을 안 봤다'

    try:
        result = unittest.TextTestRunner(verbosity=0).run(
            unittest.defaultTestLoader.loadTestsFromName('test_x', Case))
        assert result.wasSuccessful(), result.failures + result.errors
    finally:
        server.close()
    assert 'lifecycle_manager_wedged' not in ctx.record.data['measurements']
