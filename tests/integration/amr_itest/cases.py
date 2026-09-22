"""
시나리오 테스트 케이스 베이스 (launch_testing 의 unittest 클래스가 상속).

  ProbeCase      pre-shutdown: 클래스 단위 GraphProbe (로봇 네임스페이스, 백엔드에 맞는 시계),
                 판정 헬퍼 check() — result.json 에 기록하고 unittest assert 로 실패시킨다.
                 ready_gate() — launch_testing_ros.WaitForTopics 로 스택 기동 확인.
  AfterShutdown  post-shutdown: 프로세스 종료 코드 판정(크래시 = 실패) + result.json 마무리.

    class TestEmergencyStop(cases.ProbeCase):
        CTX = CTX
        def test_10_button(self): ...

    @launch_testing.post_shutdown_test()
    class TestAfterShutdown(cases.AfterShutdown):
        CTX = CTX
"""

import math
from typing import Any, Iterable, List, Optional, Sequence, Tuple
import unittest

from amr_itest.probe import GraphProbe
from amr_itest.scenario import Context
from launch_testing_ros import WaitForTopics

# SIGINT/SIGTERM 으로 정상 종료한 코드 (launch 가 종료 시 SIGINT → SIGTERM 순으로 보낸다)
CLEAN_EXIT_CODES = (0, -2, -15, 130, 143)
# 종료 코드를 판정하지 않는 외부 프로세스 (Gazebo 서버는 렌더링 정리 중 비정상 코드로 끝나는 일이 있다)
IGNORED_PROCESSES = ('gazebo', 'ign', 'ruby')


def latency_gate(summary: dict, limit_ms: float) -> tuple:
    """
    지연 판정 통계 선택: (통계 이름, 값, 통과).

    호스트가 외부 부하 아래(loadavg > CPU 수의 절반, results.host_load)면 스케줄러 지연이 섞인
    최댓값 대신 p95 로 판정하고 이름에 표시한다 — 최댓값은 result.json 에 그대로 남는다.
    부하가 없으면 최댓값으로 판정한다 (명세 "즉시 정지" 를 가장 엄격하게).
    """
    from amr_itest.results import host_load
    stat = 'p95 (provisional_under_load)' if host_load()['provisional_under_load'] else 'max'
    value = summary['p95'] if stat.startswith('p95') else summary['max']
    ok = summary['count'] > 0 and math.isfinite(value) and value <= limit_ms
    return stat, value, bool(ok)


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


def fmt(value: Any, digits: int = 4) -> Any:
    """result.json·메시지용 반올림 (유한 float 만)."""
    if isinstance(value, float) and math.isfinite(value):
        return round(value, digits)
    return value


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


class AfterShutdown(unittest.TestCase):
    """post-shutdown 공통: 종료 코드 판정 + result.json 마무리."""

    CTX: Optional[Context] = None
    ignored: Iterable[str] = IGNORED_PROCESSES

    def test_exit_codes(self, proc_info) -> None:
        """테스트 대상 프로세스가 크래시 없이 끝났는가 (정상 종료 코드만 허용)."""
        codes = {}
        bad = []
        for info in proc_info:
            name = getattr(info, 'process_name', str(info))
            code = getattr(info, 'returncode', None)
            codes[name] = code
            if any(name.startswith(p) for p in self.ignored):
                continue
            if code is not None and code not in CLEAN_EXIT_CODES:
                bad.append(f'{name}={code}')
        self.CTX.record.measure('exit_codes', codes)
        self.CTX.record.check('process exit codes', bad or 'all clean', list(CLEAN_EXIT_CODES),
                              not bad)
        self.assertFalse(bad, f'비정상 종료: {bad}')

    def test_zz_finalize(self) -> None:
        """종료 시각·부하·소요 시간 기록 (항상 마지막)."""
        self.CTX.finish()
