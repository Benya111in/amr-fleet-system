import math

from amr_evaluation import metrics
from amr_evaluation import segments
from amr_evaluation.metrics import PoseSample
import numpy as np
import pytest


# --- 각도 / 쿼터니언 ---

def test_wrap_angle():
    assert metrics.wrap_angle(2 * math.pi + 0.1) == pytest.approx(0.1)
    assert metrics.wrap_angle(-2 * math.pi - 0.1) == pytest.approx(-0.1)
    assert metrics.wrap_angle(0.0) == 0.0


def test_yaw_from_quaternion():
    yaw = 1.0
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert metrics.yaw_from_quaternion(*q) == pytest.approx(yaw)
    assert metrics.yaw_from_quaternion(0, 0, 0, 1) == 0.0


def test_stamp_to_sec():
    assert metrics.stamp_to_sec(12, 500_000_000) == pytest.approx(12.5)


# --- 보간 ---

def test_interpolate_pose_midpoint_and_yaw_wrap():
    a = PoseSample(0.0, 0.0, 0.0, yaw=3.0, v=1.0, w=0.0)
    b = PoseSample(1.0, 1.0, 2.0, yaw=-3.0, v=0.0, w=1.0)
    m = metrics.interpolate_pose(a, b, 0.5)
    assert (m.x, m.y) == (0.5, 1.0)
    assert abs(m.yaw) == pytest.approx(math.pi)   # 최단 호로 ±pi 를 지난다
    assert m.v == pytest.approx(0.5)
    assert m.w == pytest.approx(0.5)
    assert metrics.interpolate_pose(a, b, 0.0) == a
    assert metrics.interpolate_pose(a, b, 1.0).x == 1.0


def test_interpolate_pose_rejects_extrapolation():
    a = PoseSample(0.0, 0.0, 0.0)
    b = PoseSample(1.0, 1.0, 0.0)
    with pytest.raises(ValueError):
        metrics.interpolate_pose(a, b, 1.5)
    with pytest.raises(ValueError):
        metrics.interpolate_pose(a, b, -0.1)
    same = PoseSample(0.0, 5.0, 5.0)
    assert metrics.interpolate_pose(a, same, 0.0) == a


def test_pose_buffer_range_gap_and_reset():
    buf = metrics.PoseBuffer(max_age=10.0, max_gap=0.2)
    assert buf.interpolate(0.0) is None
    for i in range(11):
        buf.add(PoseSample(i * 0.1, i * 0.1, 0.0))
    assert buf.oldest_time == 0.0 and buf.latest_time == pytest.approx(1.0)
    assert buf.interpolate(0.55).x == pytest.approx(0.55)
    assert buf.interpolate(0.5) is buf.interpolate(0.5)   # 정확히 일치하는 샘플은 그대로
    assert buf.interpolate(-0.1) is None
    assert buf.interpolate(1.1) is None
    # 간격 0.4 s 인 구간은 보간하지 않는다 (외삽/가짜 오차 방지)
    buf.add(PoseSample(1.4, 5.0, 0.0))
    assert buf.interpolate(1.2) is None
    assert buf.interpolate(0.95) is not None
    # 시각 역행 → 버퍼 리셋
    buf.add(PoseSample(0.2, 0.0, 0.0))
    assert len(buf) == 1


def test_pose_buffer_max_age():
    buf = metrics.PoseBuffer(max_age=1.0)
    for i in range(21):
        buf.add(PoseSample(i * 0.1, 0.0, 0.0))
    assert buf.oldest_time >= 1.0 - 1e-9
    assert buf.latest_time == pytest.approx(2.0)


def test_position_and_yaw_error():
    gt = PoseSample(0.0, 0.0, 0.0, yaw=3.0)
    est = PoseSample(0.0, 3.0, 4.0, yaw=-3.0)
    assert metrics.position_error(gt, est) == 5.0
    assert metrics.yaw_error(gt.yaw, est.yaw) == pytest.approx(2 * math.pi - 6.0)


# --- GT/추정 짝짓기 (합성 궤적) ---

def _gt_straight(rate: float = 50.0, duration: float = 2.0, speed: float = 1.0):
    n = int(duration * rate)
    return [PoseSample(i / rate, speed * i / rate, 0.0, 0.0, v=speed, w=0.0) for i in range(n)]


def test_pairer_straight_trajectory_exact_interpolation():
    pairer = metrics.PoseErrorPairer()
    for s in _gt_straight():
        pairer.add_ground_truth(s)
    # 추정은 GT 샘플 사이 시각에, 1 cm 앞서 있다 → 직선이라 보간이 정확해 오차 = 1 cm
    for i in range(10, 90):
        t = i / 50.0 + 0.007
        pairer.add_estimate(PoseSample(t, t + 0.01, 0.0, 0.0))
    rows = pairer.flush()
    assert len(rows) == 80
    assert all(r.error == pytest.approx(0.01, abs=1e-9) for r in rows)
    assert {r.segment for r in rows} == {segments.STRAIGHT}
    assert rows[0].as_list()[:6] == [rows[0].timestamp, rows[0].gt_x, rows[0].gt_y,
                                     rows[0].est_x, rows[0].est_y, rows[0].error]
    assert pairer.dropped == 0


def test_pairer_circle_no_fake_error():
    r, w, rate = 2.0, 0.25, 50.0
    pairer = metrics.PoseErrorPairer()

    def pose(t):
        th = w * t
        return PoseSample(t, r * math.cos(th), r * math.sin(th), th + math.pi / 2, r * w, w)

    for i in range(200):
        pairer.add_ground_truth(pose(i / rate))
    for i in range(20, 180):
        pairer.add_estimate(pose(i / rate + 0.01))      # GT 와 동일한 원 위의 점
    rows = pairer.flush()
    assert len(rows) == 160
    # 선형 보간의 현(chord) 오차 r(1 − cos(Δθ/2)) ≈ 6e-6 m — 1e-4 이하여야 "가짜 오차" 가 없다
    assert max(r_.error for r_ in rows) < 1e-4
    assert max(abs(r_.yaw_error) for r_ in rows) < 1e-3
    assert {r_.segment for r_ in rows} == {segments.TURN}


def test_pairer_stationary_segment():
    pairer = metrics.PoseErrorPairer()
    for i in range(50):
        pairer.add_ground_truth(PoseSample(i * 0.02, 1.0, 1.0, 0.5, 0.0, 0.0))
    pairer.add_estimate(PoseSample(0.5, 1.02, 1.0, 0.5))
    rows = pairer.flush()
    assert rows[0].segment == segments.STOP
    assert rows[0].error == pytest.approx(0.02)


def test_pairer_waits_for_gt_and_drops_stale():
    pairer = metrics.PoseErrorPairer(max_wait=1.0)
    for s in _gt_straight(duration=1.0):
        pairer.add_ground_truth(s)
    pairer.add_estimate(PoseSample(1.5, 1.5, 0.0))      # GT 아직 도달 전 → 대기
    pairer.add_estimate(PoseSample(-1.0, 0.0, 0.0))     # GT 범위 이전 → 폐기
    pairer.add_estimate(PoseSample(3.0, 3.0, 0.0))      # max_wait 넘게 앞섬 → 폐기
    assert pairer.flush() == []
    assert pairer.pending == 1
    assert pairer.dropped == 2
    for s in _gt_straight(duration=2.0)[50:]:
        pairer.add_ground_truth(s)
    rows = pairer.flush()
    assert len(rows) == 1 and rows[0].timestamp == 1.5
    assert pairer.pending == 0


def test_pairer_without_gt_returns_nothing():
    pairer = metrics.PoseErrorPairer()
    pairer.add_estimate(PoseSample(0.0, 0.0, 0.0))
    assert pairer.flush() == []
    assert pairer.pending == 1


# --- CTE ---

SEG = np.array([[0.0, 0.0], [10.0, 0.0]])


@pytest.mark.parametrize('p, cte, planned, clamped', [
    ((5.0, 1.0), 1.0, (5.0, 0.0), False),        # 좌측 +
    ((5.0, -1.0), -1.0, (5.0, 0.0), False),      # 우측 −
    ((5.0, 0.0), 0.0, (5.0, 0.0), False),        # 경로 위
    ((10.0, 0.3), 0.3, (10.0, 0.0), False),      # 끝점 바로 옆 (t = 1) 은 수직 거리
    ((12.0, 0.0), 2.0, (10.0, 0.0), True),       # 끝점 너머 (종방향 거리 → clamped)
    ((12.0, 1.0), math.hypot(2.0, 1.0), (10.0, 0.0), True),
    ((-3.0, -1.0), -math.hypot(3.0, 1.0), (0.0, 0.0), True),   # 시작점 앞, 우측
])
def test_cross_track_error_single_segment(p, cte, planned, clamped):
    res = metrics.cross_track_error(SEG, *p)
    assert res.cte == pytest.approx(cte)
    assert (res.planned_x, res.planned_y) == pytest.approx(planned)
    assert res.segment_index == 0
    assert res.clamped is clamped


def test_cross_track_error_polyline_corner_and_sign():
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    res = metrics.cross_track_error(pts, 1.5, 0.5)     # 두 번째 선분(위쪽 진행) 의 우측
    assert res.cte == pytest.approx(-0.5)
    assert res.segment_index == 1 and not res.clamped
    assert (res.planned_x, res.planned_y) == pytest.approx((1.0, 0.5))
    res = metrics.cross_track_error(pts, 0.5, 0.2)
    assert res.cte == pytest.approx(0.2) and res.segment_index == 0
    # 모서리 바깥 (중간 정점에서 잘림) 은 경로 끝이 아니므로 clamped 아님
    res = metrics.cross_track_error(pts, 1.3, -0.3)
    assert not res.clamped
    assert metrics.cross_track_error(pts, 1.0, 1.5).clamped       # 끝점 너머


def test_cross_track_error_degenerate_paths():
    assert metrics.cross_track_error(np.zeros((0, 2)), 1.0, 1.0) is None
    single = metrics.cross_track_error(np.array([[0.0, 0.0]]), 3.0, 4.0)
    assert single.cte == 5.0 and (single.planned_x, single.planned_y) == (0.0, 0.0)
    assert single.clamped
    # 길이 0 선분은 이웃 선분의 방향으로 부호를 정한다
    pts = np.array([[0.0, 0.0], [0.0, 0.0], [1.0, 0.0]])
    res = metrics.cross_track_error(pts, -1.0, 0.1)
    assert res.cte == pytest.approx(math.hypot(1.0, 0.1))
    res = metrics.cross_track_error(pts, 0.5, -0.2)
    assert res.cte == pytest.approx(-0.2)


# --- 응답 시간 ---

def test_response_matcher_basic_latency():
    m = metrics.ResponseTimeMatcher(linear_threshold=0.05, angular_threshold=0.1)
    assert m.add_motion(0.0, 0.0, 0.0) == []
    assert m.add_command(1.0, 'task_a', 'dispatch')
    assert m.take_ready() == []
    assert m.add_motion(1.05, 0.01, 0.0) == []
    rows = m.add_motion(1.12, 0.1, 0.0)
    assert len(rows) == 1
    assert rows[0].cmd_id == 'task_a' and rows[0].cmd_source == 'dispatch'
    assert rows[0].latency_ms == pytest.approx(120.0)
    assert rows[0].as_list()[:5] == [1.0, 1.12, rows[0].latency_ms, 'task_a', 'dispatch']
    assert math.isnan(rows[0].as_list()[5]) and math.isnan(rows[0].as_list()[6])
    assert m.pending == 0 and m.last_motion_time == pytest.approx(1.12)


def test_response_matcher_require_rest_and_angular_motion():
    m = metrics.ResponseTimeMatcher()
    m.add_motion(0.0, 0.0, 0.5)                 # 제자리 회전도 움직임
    assert not m.add_command(0.1, 'while_moving')
    assert m.skipped_moving == 1
    m.add_motion(0.2, 0.0, 0.0)
    assert m.add_command(0.3, 'from_rest')
    assert m.add_motion(0.25, 1.0, 0.0) == []   # 명령 이전 시각의 샘플은 응답이 아니다
    assert len(m.add_motion(0.4, 0.0, 0.2)) == 1

    loose = metrics.ResponseTimeMatcher(require_rest=False)
    loose.add_motion(0.0, 1.0, 0.0)
    assert loose.add_command(0.1)
    assert loose.add_motion(0.1, 1.0, 0.0)[0].latency_ms == 0.0


def test_response_matcher_late_command_uses_history():
    """작업 이벤트가 로봇 출발보다 늦게 도착해도 명령 시각 기준으로 소급해 잰다."""
    m = metrics.ResponseTimeMatcher()
    for i in range(10):                         # 0.0 ~ 0.18 s 정지
        m.add_motion(i * 0.02, 0.0, 0.0)
    for i in range(10, 20):                     # 0.20 s 부터 주행
        m.add_motion(i * 0.02, 0.3, 0.0)
    assert m.add_command(0.05, 'late_event')    # 명령 시각 0.05 s (그때는 정지) → 첫 움직임 0.20 s
    rows = m.take_ready()
    assert len(rows) == 1 and rows[0].latency_ms == pytest.approx(150.0)
    assert m.pending == 0
    # 명령 시각에 이미 움직이던 경우는 제외
    assert not m.add_command(0.25, 'moving')
    assert m.skipped_moving == 1
    # 보관 이력보다 오래된 명령은 판정 불가 → 무응답으로 센다
    short = metrics.ResponseTimeMatcher(history_sec=0.1)
    for i in range(20):
        short.add_motion(i * 0.02, 0.0, 0.0)
    assert not short.add_command(0.01)
    assert short.timed_out == 1


def test_response_matcher_timeout():
    m = metrics.ResponseTimeMatcher(timeout=2.0)
    m.add_command(5.0, 'never')
    assert m.add_motion(6.0, 0.0, 0.0) == []
    assert m.add_motion(7.5, 0.0, 0.0) == []
    assert m.timed_out == 1 and m.pending == 0


def test_response_matcher_clock_domain_mismatch():
    # fleet 는 use_sim_time:=false (벽시계 에포크), GT 는 시뮬 시간 → 짝짓지 않고 센다
    m = metrics.ResponseTimeMatcher(timeout=10.0)
    m.add_motion(100.0, 0.0, 0.0)
    assert not m.add_command(1.79e9, 'wall_clock_task')
    assert m.clock_mismatch == 1 and m.pending == 0
    # 움직임보다 먼저 온 명령도 첫 움직임 샘플에서 걸러진다 (영원히 대기하지 않는다)
    m2 = metrics.ResponseTimeMatcher(timeout=10.0)
    m2.add_command(1.79e9, 'early')
    rows = []
    for i in range(100):
        rows += m2.add_motion(100.0 + i * 0.02, 0.5, 0.0)
    assert rows == [] and m2.pending == 0 and m2.clock_mismatch == 1
    # 같은 영역이어도 timeout 넘게 "미래" 인 명령은 불일치로 버린다
    m3 = metrics.ResponseTimeMatcher(timeout=2.0)
    m3.add_command(50.0, 'future')
    m3.add_motion(10.0, 0.0, 0.0)
    assert m3.clock_mismatch == 1 and m3.pending == 0
    # max_skew 를 넘게 떨어진 같은 영역 스탬프
    m4 = metrics.ResponseTimeMatcher(max_skew=100.0)
    m4.add_motion(10.0, 0.0, 0.0)
    assert not m4.add_command(500.0)
    assert m4.clock_mismatch == 1


def test_response_matcher_time_reversal():
    m = metrics.ResponseTimeMatcher()
    m.add_motion(50.0, 0.0, 0.0)
    m.add_command(50.5, 'before_reset')
    assert m.add_motion(49.8, 0.0, 0.0) == []       # 작은 역행(≤ 1 s)은 무시
    assert m.resets == 0 and m.pending == 1
    m.add_motion(1.0, 0.0, 0.0)                      # 시뮬 리셋
    assert m.resets == 1 and m.pending == 0 and m.last_motion_time == 1.0
    assert m.add_command(1.5, 'after_reset')
    assert len(m.add_motion(1.7, 0.2, 0.0)) == 1


# --- CPU ---

def test_parse_proc_stat_and_cpu_percent():
    text = ('cpu  100 0 100 800 0 0 0 0 0 0\n'
            'cpu0 50 0 50 400 0 0 0 0 0 0\n'
            'intr 12345\n'
            'ctxt 1\n')
    stat = metrics.parse_proc_stat(text)
    assert set(stat) == {'cpu', 'cpu0'}
    assert stat['cpu'][:4] == [100, 0, 100, 800]
    cur = {'cpu': [200, 0, 200, 1600, 0, 0, 0, 0, 0, 0]}
    assert metrics.cpu_percent(stat['cpu'], cur['cpu']) == pytest.approx(20.0)
    assert metrics.cpu_percent(stat['cpu'], stat['cpu']) == 0.0
    assert metrics.cpu_percent([0, 0, 0, 0], [10, 0, 0, 0]) == 100.0


# --- 요약 / 정렬 ---

def test_summarize():
    s = metrics.summarize([3.0, -4.0])
    assert (s.count, s.mean, s.max) == (2, 3.5, 4.0)
    assert s.rmse == pytest.approx(math.sqrt(12.5))
    assert s.p95 == pytest.approx(3.95)
    empty = metrics.summarize([])
    assert empty.count == 0 and math.isnan(empty.rmse)
    assert metrics.summarize([float('nan'), 1.0]).count == 1


def test_se2_align_recovers_transform():
    rng = np.random.default_rng(0)
    src = rng.uniform(-5, 5, size=(50, 2))
    yaw, tx, ty = 0.3, 1.0, -2.0
    dst = metrics.apply_se2(src, yaw, tx, ty)
    est = metrics.se2_align(src, dst)
    assert est == pytest.approx((yaw, tx, ty), abs=1e-9)
    assert np.allclose(metrics.apply_se2(src, *est), dst)
    with pytest.raises(ValueError):
        metrics.se2_align(src[:1], dst[:1])
