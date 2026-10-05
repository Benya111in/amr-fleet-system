"""
시나리오 04: EKF 위치 추정 정확도 (명세 4.3 정지 3 / 직선 5 / 회전 8 cm, 4.10 RMSE 절차).

구성 두 가지 (catalog: system 이 기본)
  system     명세 판정. system.launch.py with_localization (Gazebo + 실제 wheel_odometry · imu_filter ·
             scan_filter · AMCL(maps/warehouse.yaml) · scan_matcher · 이중 EKF · kidnap_monitor),
             스폰 = 월드 원점(명시, AMCL 초기 자세와 같다). navigation·perception 은 끄고 cmd_vel 을 직접 낸다.
             판정 전에 lifecycle(map_server·AMCL) 활성과 map ↔ 월드 항등 정합(check_map_registration)을
             확인한다 — 추정(map 프레임)과 GT(월드)를 정렬 변환 없이 비교하는 근거.
  component  EKF 배관 확인 (명세 판정 아님). 운동학 대역/Gazebo + AMCL 대역(GT + 축별 N(0, 1.5 cm), 2 Hz,
             0.1 s 지연 — stack.localization) + 실제 이중 EKF. 결과는 대역 σ 가 정한다(≈ √2·σ) —
             판정 이름에 'EKF plumbing (AMCL stand-in)' 을 붙이고, ITEST_STANDINS=never 면 건너뛴다.
시험 주행: 정지 → 직선(0.5 m/s) → 정지 → 원호(0.3 m/s, 0.5 rad/s) → 제자리 회전(0.8 rad/s) → 정지.

측정 (두 경로를 모두 판정)
  1) amr_evaluation pose_error_logger → pose_error.csv → analyze (명세 표준 절차, 설치돼 있을 때)
  2) 하네스: 프로브가 모은 odometry/filtered_map 과 ground_truth/odom 을 metrics.pair_pose_errors 로
     독립 계산 → harness_pose_error.csv
GT 를 추정 스탬프에 선형 보간하고 GT 속도로 정지/직선/회전을 나눈다 (구간별 ≥ 100 샘플).
"""

import math

from amr_itest import actions, cases, catalog, evaluation, metrics
from amr_itest.scenario import Context, SYSTEM
from amr_itest.stack import Stack
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest

CTX = Context(catalog.get(4))

# (이름, v [m/s], ω [rad/s], 지속 [s]) — 구간마다 50 Hz × 지속 ≫ 100 샘플
PROGRAM = (
    ('stop', 0.0, 0.0, 8.0),
    ('straight', 0.5, 0.0, 8.0),
    ('stop', 0.0, 0.0, 4.0),
    ('arc', 0.3, 0.5, 8.0),
    ('spin', 0.0, 0.8, 4.0),
    ('stop', 0.0, 0.0, 4.0),
)
STARTUP_WALL_S = 240.0
SETTLE_S = 3.0            # [s] EKF 첫 출력 후 수렴 대기
EST_TOPIC = 'odometry/filtered_map'
GT_TOPIC = 'ground_truth/odom'
CONVERGE_TOL_M = 0.10     # [m] 주행 전 추정 수렴 (AMCL 초기 자세 = 스폰)
CONVERGE_MAX_S = 60.0
LIFECYCLE = ('/lifecycle_manager_map', 'lifecycle_manager_localization')


def label(profile: str) -> str:
    """판정 이름 머리말: component 는 AMCL 대역이라 EKF 배관 확인이지 명세 판정이 아니다."""
    return 'RMSE' if profile == SYSTEM else 'EKF plumbing (AMCL stand-in) RMSE'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    stack = Stack(CTX, *CTX.select())
    if stack.profile == SYSTEM:
        stack.system(use_localization=True, use_navigation=False, use_perception=False,
                     pose=(0.0, 0.0, 0.0))
    else:
        stack.simulator()
        stack.description()
        stack.localization(amcl=True, ekf=True)
    stack.eval_logger('pose_error_logger', {'gt_topic': GT_TOPIC, 'est_topic': EST_TOPIC})
    return stack.launch_description(), {'stack': stack}


class TestEkfAccuracy(cases.ProbeCase):
    """정지/직선/회전 구간별 RMSE."""

    CTX = CTX

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        cls.gt = cls.probe.subscribe(GT_TOPIC, Odometry, keep_messages=20000)
        cls.est = cls.probe.subscribe(EST_TOPIC, Odometry, keep_messages=20000)

    def test_10_drive_and_measure(self, stack) -> None:
        """시험 주행 후 하네스 자체 계산으로 구간별 RMSE 판정."""
        self.ready_gate([(GT_TOPIC, Odometry), (EST_TOPIC, Odometry)], STARTUP_WALL_S)
        self.require_topic(self.gt, 10, STARTUP_WALL_S)
        self.require_topic(self.est, 10, STARTUP_WALL_S, f'{EST_TOPIC} (EKF map 노드)')
        if stack.profile == SYSTEM:
            self.wait_lifecycle_active(LIFECYCLE, STARTUP_WALL_S)
            self.check_map_registration()
            self.wait_startup_still()
            self.measure('converged_before_drive', self._converge())
        self.assertTrue(self.probe.sleep_ros(SETTLE_S, self.timeout(120.0)))
        t_start = self.probe.now()
        for name, v, w, duration in PROGRAM:
            ok = actions.drive_for(self.probe, stack.drive_topic, v, w, duration,
                                   self.timeout(duration * 60))
            self.assertTrue(ok, f'{name} 구간 주행 시간 초과 (sim time 정지?)')
        actions.drive_for(self.probe, stack.drive_topic, 0.0, 0.0, 1.0, self.timeout(60.0))
        t_end = self.probe.now()

        gt = [metrics.sample_from_odom(m) for _, m in self.gt.messages()]
        est = [s for s in (metrics.sample_from_odom(m) for _, m in self.est.messages())
               if t_start <= s.t <= t_end]
        rows = metrics.pair_pose_errors(gt, est)
        self.ctx.record.write_csv('harness_pose_error.csv', metrics.POSE_COLUMNS,
                                  [r.as_list() for r in rows])
        summary = metrics.pose_summary(rows)
        self.measure('harness_pose_error_cm', {k: s.as_dict(scale=100.0, digits=2)
                                               for k, s in summary.items()})
        self.measure('drive_window_s', round(t_end - t_start, 2))
        failed = []
        for seg, rmse, thr, ok, n in metrics.pose_verdicts(summary):
            self.ctx.record.check(f'harness {label(stack.profile)} {seg} (n={n})',
                                  cases.fmt(rmse), thr, ok, 'm')
            if not ok:
                failed.append(f'{seg}: RMSE {rmse:.4f} m (n={n}, 기준 {thr})')
        self.assertFalse(failed, '; '.join(failed))

    def _converge(self) -> bool:
        """추정이 GT 에 CONVERGE_TOL_M 안으로 들어올 때까지 (못 들어와도 주행은 한다 — RMSE 가 판정)."""
        def err() -> float:
            g, e = self.gt.last(), self.est.last()
            if g is None or e is None:
                return math.inf
            a, b = metrics.sample_from_odom(g), metrics.sample_from_odom(e)
            return math.hypot(a.x - b.x, a.y - b.y)
        ok = self.probe.wait_until(lambda: err() <= CONVERGE_TOL_M,
                                   self.timeout(CONVERGE_MAX_S), 0.1)
        self.measure('pre_drive_error_m', cases.fmt(err()))
        return ok


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX

    def test_evaluation_logger(self) -> None:
        """amr_evaluation pose_error_logger CSV → analyze 판정 (설치돼 있을 때)."""
        if CTX.record.data['components'].get('evaluation') != 'real':
            self.skipTest('amr_evaluation 미설치 — 하네스 자체 계산(test_10)으로만 판정')
        result = evaluation.analyze(CTX.log_dir)
        rows = evaluation.gated_rows(result, '위치 추정 오차')
        CTX.record.measure('evaluation_pose_rows', rows)
        CTX.record.measure('evaluation_warnings', result['warnings'])
        failed = []
        prefix = '' if CTX.profile == SYSTEM else 'EKF plumbing (AMCL stand-in) '
        for r in rows:
            CTX.record.check(f"amr_evaluation {prefix}{r['stat']} {r['segment']} (n={r['count']})",
                             cases.fmt(r['value']), r['threshold'], bool(r['passed']), 'm')
            if not r['passed']:
                failed.append(f"{r['segment']} {r['value']}")
        self.assertTrue(rows, f"pose_error.csv 판정 행 없음: {result['warnings']}")
        self.assertFalse(failed, f'amr_evaluation 판정 실패: {failed}')
