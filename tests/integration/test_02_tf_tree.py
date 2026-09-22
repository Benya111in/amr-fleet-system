"""
시나리오 02: TF 트리 무결성 (명세 4.1 "센서 위치와 TF 트리가 정확히 일치", 9장 view_frames).

스택: 시뮬레이터(Gazebo 또는 운동학 대역) + amr_description(robot_state_publisher, 실제 URDF) +
wheel_odometry·amcl(실제 또는 대역) + robot_localization 이중 EKF(config/ekf.yaml).
/tf, /tf_static 원시 메시지로 간선 그래프를 만들어 계약 간선(tf_tree.expected_edges:
map→odom→base_footprint→base_link→{lidar, camera(→광학 2), imu, 바퀴 2})과 비교하고,
tf2 Buffer 로 map → 센서 프레임 조회가 실제로 되는지 확인한다.

산출: tf_edges.csv (간선별 판정), frames.dot (Graphviz, 누락 간선은 빨간 점선).
"""

import math

from amr_itest import cases, catalog, config, tf_tree
from amr_itest.scenario import Context
from amr_itest.stack import Stack
import launch_testing
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
from rclpy.duration import Duration
from rclpy.time import Time

CTX = Context(catalog.get(2))

CAPTURE_S = 5.0           # [s] ROS 시각 수집 창 (동적 간선 주기 측정)
STARTUP_WALL_S = 240.0    # [s] 모든 기대 간선이 나타날 때까지 wall 상한
EKF_RATE_HZ = 50.0        # config/ekf.yaml frequency


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    CTX.begin()
    stack = Stack(CTX, *CTX.select())
    stack.simulator()
    stack.description()
    stack.localization(amcl=True, ekf=True)
    return stack.launch_description(), {'stack': stack}


class TestTfTree(cases.ProbeCase):
    """계약 간선 존재·유일 부모·값·주기 + tf2 조회."""

    CTX = CTX

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        cls.graph = cls.probe.capture_tf()
        cls.buffer = cls.probe.tf_buffer()
        cls.expected = tf_tree.expected_edges(config.sensors(), config.robot_params(),
                                              config.ekf_params(), cls.CTX.settings.frame_prefix)

    def _missing(self):
        snap = self.probe.tf_snapshot()
        return [e for e in self.expected if snap.edge(e.parent, e.child) is None]

    def test_05_ready(self) -> None:
        """기동 게이트 (launch_testing_ros WaitForTopics): GT·odom EKF 출력 흐름."""
        self.ready_gate([('ground_truth/odom', Odometry), ('odometry/filtered', Odometry)],
                        STARTUP_WALL_S)

    def test_10_edges(self) -> None:
        """기대 간선이 모두 있고, 부모가 유일하며, 정적 값이 config 와 같다."""
        self.probe.wait_until(lambda: not self._missing(), self.timeout(STARTUP_WALL_S), 0.5)
        self.assertTrue(self.probe.sleep_ros(CAPTURE_S, self.timeout(CAPTURE_S * 60)),
                        'ROS 시각이 진행하지 않음')
        snap = self.probe.tf_snapshot()
        min_rate = {e.child: 0.9 * EKF_RATE_HZ for e in self.expected[:2]}
        checks = tf_tree.check_edges(snap, self.expected, min_rate=min_rate)
        self.ctx.record.write_csv('tf_edges.csv', tf_tree.EDGE_CSV_COLUMNS,
                                  [c.as_row() for c in checks])
        missing = [(c.expected.parent, c.expected.child) for c in checks if not c.present]
        self.ctx.record.write_text('frames.dot', snap.to_dot(highlight=missing))
        self.measure('edges', {f'{c.expected.parent}->{c.expected.child}': {
            'present': c.present, 'rate_hz': round(c.rate_hz, 2),
            'translation_error_m': cases.fmt(c.translation_error, 5),
            'rotation_error_deg': cases.fmt(math.degrees(c.rotation_error), 3)
            if c.rotation_error is not None else None,
            'problems': c.problems} for c in checks})
        self.measure('observed_frames', sorted(snap.frames()))
        self.check('missing edges', missing, [], not missing)
        problems = [f'{c.expected.parent}->{c.expected.child}: {"; ".join(c.problems)}'
                    for c in checks if c.problems]
        self.check('edge problems', problems, [], not problems)

    def test_20_structure(self) -> None:
        """이중 부모 없음, 순환 없음, 루트는 map 하나, 프레임 접두어 규약 일치."""
        snap = self.probe.tf_snapshot()
        dup = snap.duplicate_parents()
        self.check('duplicate parents', dup, {}, not dup)
        self.check('cycle', snap.has_cycle(), False, not snap.has_cycle())
        roots = sorted(snap.roots())
        self.check('roots', roots, ['map'], roots == ['map'])
        prefix = tf_tree.detect_prefix(snap.frames())
        self.check('frame prefix', prefix, self.settings.frame_prefix,
                   prefix == self.settings.frame_prefix)

    def test_30_tf2_lookup(self) -> None:
        """tf2 Buffer 로 map → 모든 센서 프레임 조회 (체인이 실제로 이어져 있다)."""
        failed = []
        for e in self.expected[2:]:
            ok = self.probe.wait_until(
                lambda e=e: self.buffer.can_transform('map', e.child, Time(),
                                                      Duration(seconds=0.0)),
                self.timeout(10.0), 0.1)
            if not ok:
                failed.append(e.child)
        self.check('tf2 map -> sensor lookups', failed or 'all ok', 'all ok', not failed)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(cases.AfterShutdown):
    CTX = CTX
