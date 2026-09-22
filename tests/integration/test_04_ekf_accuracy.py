"""
시나리오 04: EKF 위치 추정 정확도 (명세 4.3 정지 3 / 직선 5 / 회전 8 cm, 4.10 RMSE 절차).

스택: 시뮬레이터(운동학 대역 또는 Gazebo) + URDF TF + wheel_odometry(실제 또는 대역) +
amcl 대역(GT + 축별 N(0, 1.5 cm)·yaw N(0, 0.01 rad) 을 2 Hz, 0.1 s 지연으로 발행 — stack.localization)
+ robot_localization 이중 EKF(config/ekf.yaml). 실제 amr_localization 이 설치되면 wheel_odometry 는
실제 노드로 바뀐다 (AMCL 은 지도·스캔이 필요해 system 프로필에서 실제 노드).
시험 주행: 정지 → 직선(0.5 m/s) → 정지 → 원호(0.3 m/s, 0.5 rad/s) → 제자리 회전(0.8 rad/s) → 정지.

측정 (두 경로를 모두 판정)
  1) amr_evaluation pose_error_logger → pose_error.csv → analyze (명세 표준 절차, 설치돼 있을 때)
  2) 하네스: 프로브가 모은 odometry/filtered_map 과 ground_truth/odom 을 metrics.pair_pose_errors 로
     독립 계산 → harness_pose_error.csv
GT 를 추정 스탬프에 선형 보간하고 GT 속도로 정지/직선/회전을 나눈다 (구간별 ≥ 100 샘플).
map 프레임 = GT world 프레임 (amcl 대역이 GT 기준으로 발행하므로 정렬 변환 없이 비교한다).
"""

from amr_itest import actions, cases, catalog, evaluation, metrics
from amr_itest.scenario import Context
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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    stack = Stack(CTX, CTX.select_backend(), CTX.select_profile())
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
            self.ctx.record.check(f'harness RMSE {seg} (n={n})', cases.fmt(rmse), thr, ok, 'm')
            if not ok:
                failed.append(f'{seg}: RMSE {rmse:.4f} m (n={n}, 기준 {thr})')
        self.assertFalse(failed, '; '.join(failed))


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
        for r in rows:
            CTX.record.check(f"amr_evaluation {r['stat']} {r['segment']} (n={r['count']})",
                             cases.fmt(r['value']), r['threshold'], bool(r['passed']), 'm')
            if not r['passed']:
                failed.append(f"{r['segment']} {r['value']}")
        self.assertTrue(rows, f"pose_error.csv 판정 행 없음: {result['warnings']}")
        self.assertFalse(failed, f'amr_evaluation 판정 실패: {failed}')
