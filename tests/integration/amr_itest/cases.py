"""
시나리오 테스트 케이스 베이스 (launch_testing 의 unittest 클래스가 상속).

  ProbeCase      pre-shutdown: 클래스 단위 GraphProbe (로봇 네임스페이스, 백엔드에 맞는 시계),
                 판정 헬퍼 check() — result.json 에 기록하고 unittest assert 로 실패시킨다.
                 ready_gate() — launch_testing_ros.WaitForTopics 로 스택 기동 확인.
                 wait_lifecycle_active() — Nav2·AMCL·map_server 수명 주기 활성 대기.
                 check_map_registration() — /map ↔ 월드 SE(2) 항등 정합 (map = 월드 가정 확인).
                 test_zz_processes_alive — 실행 중(종료 전) 크래시 판정 (종료 단계 크래시와 구분).
  AfterShutdown  post-shutdown: 실행 중 크래시는 실패, 종료 단계에서만 난 비정상 코드는 경고로 기록 +
                 result.json 마무리.

    class TestEmergencyStop(cases.ProbeCase):
        CTX = CTX
        def test_10_button(self): ...

    @launch_testing.post_shutdown_test()
    class TestAfterShutdown(cases.AfterShutdown):
        CTX = CTX
"""

import math
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple
import unittest

from amr_itest import config
from amr_itest.probe import GraphProbe
from amr_itest.scenario import Context
from launch_testing_ros import WaitForTopics

# SIGINT/SIGTERM 으로 정상 종료한 코드 (launch 가 종료 시 SIGINT → SIGTERM 순으로 보낸다)
CLEAN_EXIT_CODES = (0, -2, -15, 130, 143)
# 종료 코드를 판정하지 않는 외부 프로세스 (Gazebo 서버는 렌더링 정리 중 비정상 코드로 끝나는 일이 있다)
IGNORED_PROCESSES = ('gazebo', 'ign', 'ruby')
RUN_EXITS_KEY = 'exited_during_run'
LIFECYCLE_ACTIVE = 3        # lifecycle_msgs/State PRIMARY_STATE_ACTIVE
SHUTDOWN_MARGIN_S = 120.0   # 러너 상한 − 이만큼: launch 종료(SIGINT→SIGTERM→SIGKILL) + post-shutdown 판정


def host_under_load() -> bool:
    """외부 부하가 CPU 수의 절반을 넘는가 (results.host_load)."""
    from amr_itest.results import host_load
    return bool(host_load()['provisional_under_load'])


def latency_gate(summary: dict, limit_ms: float) -> tuple:
    """
    지연 판정: (통계 이름, 값, 통과) — 항상 최댓값.

    부하 때문에 기준을 느슨하게 하지 않는다 (p95 로 바꾸면 20회 중 1회의 2 s 정지 실패도 통과했다).
    부하 중 실패는 시나리오가 한 번 다시 재고(retry_under_load), 두 번 모두 기록한다.
    """
    value = summary['max']
    ok = summary['count'] > 0 and math.isfinite(value) and value <= limit_ms
    return 'max', value, bool(ok)


def retry_under_load(measure, limit_ms: float, under_load=host_under_load):
    """
    measure() → latency_summary 를 한 번 재고 최댓값으로 판정한다.

    실패했고 그때 호스트가 부하 아래였으면 한 번 더 재서 그 결과로 판정한다 (둘 다 반환).
    반환: (최종 요약, 통과, 시도 목록 [(요약, 부하였나)]).
    """
    attempts = []
    summary = measure()
    loaded = under_load()
    attempts.append((summary, loaded))
    ok = latency_gate(summary, limit_ms)[2]
    if not ok and loaded:
        summary = measure()
        attempts.append((summary, under_load()))
        ok = latency_gate(summary, limit_ms)[2]
    return summary, ok, attempts


def wait_for_topics(topics: Sequence[Tuple[str, type]],
                    timeout: float) -> Tuple[bool, List[str]]:
    """
    launch_testing_ros.WaitForTopics 로 토픽마다 메시지 1 개 이상 수신을 기다린다.

    프로브와 독립된 rclpy 컨텍스트·노드(QoS reliable, depth 10)라, 스택 기동 판정을 ROS 표준
    launch 테스트 도구로 한 번 더 확인한다. topics 는 절대 이름. 반환: (전부 수신, 못 받은 토픽).
    """
    waiter = WaitForTopics(list(topics), timeout=timeout)
    try:
        ok = bool(waiter.wait())
        missing = sorted(waiter.topics_not_received())
    finally:
        waiter.shutdown()
    return ok, missing


def imu_bias_time(default: float = 60.0) -> float:
    """amr_localization imu_filter.yaml 의 bias_estimation_time [s] (패키지가 없으면 default)."""
    from amr_itest import requirements as req
    path = req.config_file('amr_localization', 'imu_filter.yaml')
    if path is None:
        return default
    try:
        params = config.load_ros_params(path, '/**/imu_filter_node')
    except (KeyError, OSError, ValueError):
        return default
    return float(params.get('bias_estimation_time', default))


def fmt(value: Any, digits: int = 4) -> Any:
    """result.json·메시지용 반올림 (유한 float 만)."""
    if isinstance(value, float) and math.isfinite(value):
        return round(value, digits)
    return value


def exit_events(proc_info) -> Dict[str, int]:
    """launch_testing proc_info 의 종료 이벤트 {프로세스 이름: 종료 코드} (아직 살아 있으면 빠진다)."""
    out = {}
    for info in proc_info:
        code = getattr(info, 'returncode', None)
        if code is None:
            continue
        out[getattr(info, 'process_name', str(info))] = code
    return out


def bad_exits(codes: Dict[str, Optional[int]],
              ignored: Iterable[str] = IGNORED_PROCESSES) -> List[str]:
    """정상 종료 코드가 아닌 프로세스 ('이름=코드'), ignored 접두어는 뺀다."""
    ignored = tuple(ignored)
    return [f'{name}={code}' for name, code in sorted(codes.items())
            if code is not None and code not in CLEAN_EXIT_CODES
            and not name.startswith(ignored)]


class ProbeCase(unittest.TestCase):
    """시나리오 pre-shutdown 테스트 베이스."""

    CTX: Optional[Context] = None
    probe: GraphProbe

    @classmethod
    def setUpClass(cls) -> None:
        if cls.CTX is None:
            raise RuntimeError(f'{cls.__name__}.CTX 가 설정되지 않았다')
        cls.ctx = cls.CTX
        cls.settings = cls.CTX.settings
        cls.probe = GraphProbe(f'itest_{cls.CTX.scenario.slug}',
                               use_sim_time=cls.CTX.use_sim_time,
                               namespace=cls.CTX.settings.namespace)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.probe.close()

    # --- 판정 ---
    def check(self, name: str, value: Any, threshold: Any, passed: bool, unit: str = '',
              detail: str = '') -> None:
        """판정을 result.json 에 기록하고, 실패면 테스트를 실패시킨다."""
        self.ctx.record.check(name, fmt(value), fmt(threshold), passed, unit)
        if not passed:
            self.fail(f'{name}: {fmt(value)} {unit} (기준 {fmt(threshold)} {unit})'
                      + (f' — {detail}' if detail else ''))

    def measure(self, key: str, value: Any) -> None:
        self.ctx.record.measure(key, value)

    def time_left(self, margin: float = SHUTDOWN_MARGIN_S) -> float:
        """러너 상한(ITEST_SCENARIO_TIMEOUT)까지 남은 wall 시간 [s] − 종료·post-shutdown 여유."""
        return self.settings.scenario_timeout - self.ctx.record.elapsed() - margin

    def budget_for(self, seconds: float, what: str) -> bool:
        """
        다음 시행(최악 seconds [s, wall])을 러너 상한 안에 끝낼 수 있는가. 모자라면 기록하고 False.

        반복 시나리오는 이것이 False 면 시행을 멈추고 지금까지의 측정으로 판정한다 (시행 수 판정이 실패로
        드러난다) — 러너에 잘려 CSV·판정 없이 끝나지 않게.
        """
        left = self.time_left()
        if left >= seconds:
            return True
        self.ctx.record.note(f'{what}: 러너 상한까지 {left:.0f} s < 필요 {seconds:.0f} s → 중단')
        return False

    def timeout(self, seconds: float) -> float:
        return self.ctx.timeout(seconds)

    def require_topic(self, rec, n: int, seconds: float, what: str = '') -> None:
        """토픽 rec 에서 n 개 수신까지 기다리고, 못 받으면 실패 (그래프 상태를 메시지에)."""
        ok = self.probe.wait_for_messages(rec, n, self.timeout(seconds))
        if not ok:
            pubs = self.probe.publisher_count(rec.topic)
            self.fail(f'{what or rec.topic}: {seconds * self.settings.timeout_scale:.0f} s 안에 '
                      f'{n} 개를 못 받음 (받은 수 {rec.count}, 발행자 {pubs}, '
                      f'노드 {sorted(self.probe.node_names())})')

    def wait_startup_still(self, still_s: float = 10.0) -> float:
        """
        기동 직후(system 프로필) 로봇을 움직이기 전에 imu_filter_node 바이어스 추정용 정지 표본을 모은다.

        imu_filter.yaml bias_estimation_time(60 s, sim) 안에 움직이면 그때까지 모은 표본으로, 표본이 없으면(기동
        직후) 파라미터 바이어스 0 으로 대신한다 — 통합 실측: 03 이 기동 0.7 s 에 주행을 시작해 "falling back to
        parameter bias" → 자이로 바이어스(≈ 0.01 rad/s)가 보정되지 않아 EKF 헤딩이 흘러 SLAM 맵이 뭉개졌다
        (재현율 0.65, 오점유 0.23). still_s(sim) 동안 정지면 표본 ≈ 1000 개(100 Hz)로 표준오차 ≈ 1e-5 rad/s.
        반환: 기다린 뒤 sim 시각 [s].
        """
        target = min(imu_bias_time(), still_s)
        self.probe.wait_until(lambda: self.probe.now() >= target, self.timeout(target * 10.0), 0.2)
        self.measure('startup_still_until_s', fmt(self.probe.now(), 2))
        return self.probe.now()

    def warm_up_localization(self, drive_topic: str, gt, turn: float = 2.0 * math.pi,
                             rate: float = 0.5) -> bool:
        """
        제자리 한 바퀴 (순간 이동 시험 전): 위치 추정이 "추적 중" 상태가 되게 한다.

        AMCL 은 움직여야 갱신·발행하고(update_min_a), kidnap_monitor_node 는 마지막 amcl_pose ∘ odom 으로 스캔
        일치도를 재므로, 기동 후 한 번도 움직이지 않은 로봇을 순간 이동하면 비교 기준이 없어 감지하지 못한다
        (05 실측: 정지 상태 첫 납치 미감지, 추정 오차 13.4 m 유지). 실제 운용 중 납치를 흉내 낸다.
        """
        from amr_itest import actions
        duration = turn / rate
        ok = actions.drive_for(self.probe, drive_topic, 0.0, rate, duration,
                               self.timeout(duration * 10.0))
        actions.drive_for(self.probe, drive_topic, 0.0, 0.0, 1.0, self.timeout(30.0))
        rested = actions.wait_rest(self.probe, gt, hold=1.0, timeout=self.timeout(30.0))
        self.measure('localization_warm_up', {'turn_deg': round(math.degrees(turn)),
                                              'drove': ok, 'rested': rested})
        return ok and rested

    def ready_gate(self, topics: Sequence[Tuple[str, type]], seconds: float) -> None:
        """
        기동 게이트: launch_testing_ros.WaitForTopics 로 topics 가 모두 흐를 때까지 기다린다.

        상대 이름은 로봇 네임스페이스 기준으로 푼다. WaitForTopics 는 reliable 로 구독하므로
        reliable 로 발행되는 토픽(ground_truth/odom, /tf, 브리지 센서 토픽 등)만 넣는다.
        결과는 result.json 판정 'ready (launch_testing_ros WaitForTopics)' 로 남는다.
        """
        names = [(self.probe.node.resolve_topic_name(t), typ) for t, typ in topics]
        ok, missing = wait_for_topics(names, self.timeout(seconds))
        self.check('ready (launch_testing_ros WaitForTopics)', missing or 'all received', [],
                   ok, '', f'{seconds * self.settings.timeout_scale:.0f} s 안에 못 받은 토픽 '
                   f'{missing}, 노드 {sorted(self.probe.node_names())}')

    def managed_nodes(self, manager: str) -> Tuple[str, ...]:
        """관리자 이름 → 그 관리자가 맡는 lifecycle 노드 (이름은 amr_bringup 이 단일 출처)."""
        from amr_bringup.launch_utils import LIFECYCLE_NODES
        label = manager.rstrip('/').rsplit('lifecycle_manager_', 1)[-1]
        if label == 'map':
            return ('/map_server',)                      # map 은 로봇 이름공간 밖이다
        return LIFECYCLE_NODES.get(label, ())

    def lifecycle_state(self, node: str) -> int:
        """<node>/get_state 의 상태 id (응답 없으면 0 = UNKNOWN)."""
        from lifecycle_msgs.srv import GetState
        res = self.probe.call(GetState, f'{node}/get_state', GetState.Request(), 2.0)
        return int(res.current_state.id) if res is not None else 0

    def wait_lifecycle_active(self, managers: Sequence[str], seconds: float,
                              name: str = 'lifecycle managers active') -> None:
        """
        nav2_lifecycle_manager 들의 <이름>/is_active 가 true 가 될 때까지 기다린다 (판정 기록).

        Nav2 가 활성화 도중일 때 요청을 보내면 거절되거나(06: 50/50 거절), 종료가 활성화 전이 도중에 오면
        controller_server 가 abort(-6)로 끝났다 — 요청 전에 스택이 활성인지 확인한다.

        관리자가 응답하지 않아도 그 관리자가 맡는 노드가 모두 active 면 통과로 본다: nav2_lifecycle_manager
        는 change_state 를 무한 대기로 부르므로(nav2_util::LifecycleServiceClient, 시간 제한 인자 없음)
        전이 응답이 한 번 유실되면 영원히 멈춘다 — 그때 amr_bringup 의 lifecycle_watchdog 이 노드를 직접
        올려 스택은 정상 동작한다. 시나리오가 재려는 것은 스택 동작이므로 측정은 진행하고, 멈춘 관리자는
        measure('lifecycle_manager_wedged') 로 따로 남긴다 (bond 감시가 없는 상태라 결함은 결함이다).
        """
        from std_srvs.srv import Trigger
        state = {}

        def active() -> bool:
            for m in managers:
                if state.get(m):
                    continue
                res = self.probe.call(Trigger, f'{m}/is_active', Trigger.Request(), 2.0)
                state[m] = bool(res is not None and res.success)
            return all(state.get(m) for m in managers)

        ok = self.probe.wait_until(active, self.timeout(seconds), 1.0)
        inactive = [m for m in managers if not state.get(m)]
        note = f'{seconds * self.settings.timeout_scale:.0f} s 안에 비활성: {inactive}'
        if not ok:
            wedged = {}
            for m in inactive:
                nodes = self.managed_nodes(m)
                wedged[m] = {n: self.lifecycle_state(n) for n in nodes}
            all_active = bool(wedged) and all(
                nodes and all(s == LIFECYCLE_ACTIVE for s in nodes.values())
                for nodes in wedged.values())
            if all_active:
                self.measure('lifecycle_manager_wedged', wedged)
                ok = True
                note = (f'관리자 {inactive} 가 응답하지 않지만 그 노드는 모두 active '
                        f'(lifecycle_watchdog 이 직접 전이) — 관리자 결함은 measure 에 기록')
        self.check(name, inactive or 'all active', [], ok, '', note)

    def check_map_registration(self, seconds: float = 120.0, topic: str = '/map', grid=None):
        """
        지도(map 프레임) ↔ 월드 지면 진실의 항등 정합을 확인한다 (catalog.REGISTRATION).

        시나리오는 map 좌표로 목표를 주고 월드 좌표(GT·SDF)로 판정한다 — 그 둘이 같다는 가정
        (maps/warehouse.yaml 은 월드에 정합된 지도)을 조용히 믿지 않고 여기서 잰다. 월드 쪽은 SDF visual 의
        LiDAR 평면 단면 (gpu_lidar 가 재는 면). 결과는 map_registration 측정 + 판정.
        """
        from amr_itest import requirements as req
        from amr_itest import worldmap
        import numpy as np
        if grid is None:
            from nav_msgs.msg import OccupancyGrid
            rec = self.probe.subscribe(topic, OccupancyGrid, 'latched', keep_messages=2)
            self.require_topic(rec, 1, seconds, f'{topic} (map_server)')
            grid = rec.last()
        info = grid.info
        data = np.asarray(grid.data, dtype=int).reshape(info.height, info.width)
        pts = worldmap.occupied_points(data, info.resolution,
                                       (info.origin.position.x, info.origin.position.y))
        share = req.share_dir('amr_simulation')
        shapes = worldmap.footprints(share / 'worlds' / self.settings.world, share / 'models',
                                     config.scan_plane_height(), 'visual')
        reg = worldmap.register(pts, shapes)
        self.measure('map_registration', reg.as_dict())
        self.check('map frame == world frame (SE(2) registration)',
                   {'t_m': fmt(reg.translation), 'yaw_deg': fmt(math.degrees(reg.dyaw)),
                    'median_m': fmt(reg.median_identity)},
                   {'t_m': 0.05, 'yaw_deg': 0.1, 'median_m': 0.05}, reg.identity_ok())
        return reg

    def test_zz_processes_alive(self, proc_info=None) -> None:
        """
        실행 중(종료 전) 비정상 종료한 프로세스가 없는가.

        launch 종료 단계의 크래시(robot_state_publisher SIGINT 중 -11, lifecycle 전이 중 -6 등)는 측정이
        끝난 뒤라 AfterShutdown 이 경고로만 남긴다 — 여기서는 측정 도중에 죽은 것만 실패로 본다.
        proc_info 는 launch_testing 이 이름으로 넣어 준다 (launch_testing 밖에서 돌리면 None → 건너뜀).
        """
        if proc_info is None:
            self.skipTest('launch_testing proc_info 없음 (launch_testing 밖에서 실행)')
        died = exit_events(proc_info)
        self.ctx.record.measure(RUN_EXITS_KEY, died)
        bad = bad_exits(died)
        self.check('no process crash during the run', bad or 'none', [], not bad, '',
                   f'측정 중 비정상 종료: {bad}')


class AfterShutdown(unittest.TestCase):
    """post-shutdown 공통: 종료 코드 판정 + result.json 마무리."""

    CTX: Optional[Context] = None
    ignored: Iterable[str] = IGNORED_PROCESSES

    def test_exit_codes(self, proc_info) -> None:
        """
        실행 중 크래시는 실패, 종료 단계에서만 난 비정상 종료는 경고 (측정이 모두 끝난 뒤).

        ProbeCase.test_zz_processes_alive 가 종료 직전 상태를 기록했으면 그 뒤에 끝난 프로세스는 종료
        단계로 본다. 기록이 없으면(pre-shutdown 이 돌지 않음) 모든 비정상 종료를 실패로 본다.
        """
        codes = {getattr(i, 'process_name', str(i)): getattr(i, 'returncode', None)
                 for i in proc_info}
        record = self.CTX.record
        record.measure('exit_codes', codes)
        run_exits = record.data['measurements'].get(RUN_EXITS_KEY)
        if run_exits is None:
            bad = bad_exits(codes, self.ignored)
            record.check('process exit codes', bad or 'all clean', list(CLEAN_EXIT_CODES),
                         not bad)
            self.assertFalse(bad, f'비정상 종료: {bad}')
            return
        during = {n: c for n, c in codes.items() if n in run_exits}
        at_shutdown = {n: c for n, c in codes.items() if n not in run_exits}
        bad = bad_exits(during, self.ignored)
        warn = bad_exits(at_shutdown, self.ignored)
        if warn:
            record.measure('shutdown_exit_warnings', warn)
            record.note(f'종료 단계 비정상 종료 (측정 뒤, 판정 제외): {warn}')
        record.check('process exit codes (during run)', bad or 'all clean',
                     list(CLEAN_EXIT_CODES), not bad)
        self.assertFalse(bad, f'실행 중 비정상 종료: {bad}')

    def test_zz_finalize(self) -> None:
        """종료 시각·부하·소요 시간 기록 (항상 마지막)."""
        self.CTX.finish()
