"""납치 감지·복구 상태기 단위 테스트 (rclpy 비의존)."""

import math

from amr_localization.kidnap_detector import (
    ActionType, compose, interpolate_pose, jump_discrepancy, KidnapDetector, KidnapParams,
    relative_pose, StampedPose, State, StopUpdateScheduler, wrap_angle)
import pytest


def kinds(actions):
    return [a.kind for a in actions]


def feed_odom(det, t0, t1, pose=(0.0, 0.0, 0.0), rate=50.0):
    n = int(round((t1 - t0) * rate))
    for i in range(n + 1):
        det.on_odom(t0 + i / rate, pose)


def make_lost(det, t=10.0):
    """정지 상태에서 스캔-맵 불일치로 LOST 까지 몰아간다. LOST 선언 동작 반환."""
    feed_odom(det, 0.0, t)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 0.01, 0.001)
    actions = []
    step = 0.5
    k = 0
    while det.state != State.RECOVERING:
        actions = det.on_match(t + k * step, 0.1, 150) + det.tick(t + k * step)
        k += 1
        assert k < 100
    return actions, t + (k - 1) * step


def test_geometry_helpers():
    assert wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    a = (1.0, 2.0, math.pi / 2)
    b = (1.0, 3.0, math.pi / 2)
    assert relative_pose(a, b) == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)
    c = compose(a, (1.0, 0.0, 0.0))
    assert c == pytest.approx((1.0, 3.0, math.pi / 2), abs=1e-12)
    assert compose(a, relative_pose(a, b)) == pytest.approx(b, abs=1e-12)


def test_interpolate_pose():
    s = [StampedPose(0.0, (0.0, 0.0, math.pi - 0.1)),
         StampedPose(1.0, (2.0, 0.0, -math.pi + 0.1))]
    assert interpolate_pose([], 0.5) is None
    assert interpolate_pose(s, -1.0) == s[0].pose
    assert interpolate_pose(s, 2.0) == s[1].pose
    x, y, yaw = interpolate_pose(s, 0.5)
    assert x == pytest.approx(1.0)
    assert abs(wrap_angle(yaw - math.pi)) < 1e-9  # ±π 경계를 짧은 쪽으로 보간
    many = [StampedPose(float(i), (float(i), 0.0, 0.0)) for i in range(10)]
    assert interpolate_pose(many, 6.25)[0] == pytest.approx(6.25)


def test_jump_discrepancy_ignores_frame_offset():
    # map→odom 오프셋이 커도 증분이 같으면 불일치 0
    amcl_prev, amcl_now = (10.0, 5.0, 1.0), compose((10.0, 5.0, 1.0), (0.5, 0.1, 0.2))
    odom_prev, odom_now = (0.0, 0.0, 0.0), (0.5, 0.1, 0.2)
    dp, dth = jump_discrepancy(amcl_prev, amcl_now, odom_prev, odom_now)
    assert dp == pytest.approx(0.0, abs=1e-12)
    assert dth == pytest.approx(0.0, abs=1e-12)
    dp, _ = jump_discrepancy(amcl_prev, (20.0, 5.0, 1.0), odom_prev, odom_now)
    assert dp > 9.0


def test_normal_tracking_has_no_alarm():
    det = KidnapDetector()
    for i in range(200):
        t = i * 0.1
        pose = (0.05 * i, 0.0, 0.0)
        det.on_odom(t, pose)
        if i % 5 == 0:
            assert det.on_amcl(t, (pose[0] + 0.01, 0.0, 0.0), 0.002, 0.0005) == []
            assert det.on_match(t, 0.9, 170) == []
        assert det.tick(t) == []
    assert det.state == State.TRACKING
    assert not det.lost


def test_map_pose_prediction_uses_odom_increment():
    det = KidnapDetector()
    assert det.map_pose_at(0.0) is None
    feed_odom(det, 0.0, 1.0, (1.0, 0.0, 0.0))
    det.on_amcl(1.0, (5.0, 5.0, math.pi / 2), 0.01, 0.001)
    det.on_odom(1.5, (2.0, 0.0, 0.0))   # odom 에서 +x 로 1 m → map 에서는 +y 로 1 m
    x, y, yaw = det.map_pose_at(1.5)
    assert (x, y) == pytest.approx((5.0, 6.0), abs=1e-9)
    assert yaw == pytest.approx(math.pi / 2)


def test_stationary_kidnap_detected_by_scan_mismatch():
    det = KidnapDetector(KidnapParams(match_window=3, suspect_time=1.0))
    actions, t_lost = make_lost(det)
    assert det.lost
    assert kinds(actions) == [ActionType.PUBLISH_LOST, ActionType.REINITIALIZE, ActionType.SPIN]
    assert actions[0].value is True
    assert actions[1].value == 1            # 첫 시도
    assert actions[2].value == pytest.approx(2 * math.pi)
    assert det.detect_time == pytest.approx(t_lost)
    assert any('scan-map' in e.reason for e in det.events)


def test_single_low_scans_do_not_trigger():
    det = KidnapDetector(KidnapParams(match_window=5))
    for k in range(4):
        det.on_match(float(k), 0.1, 150)
    det.on_match(4.0, 0.9, 150)            # 지나가는 가림: 창을 채우기 전에 회복
    for k in range(5, 20):
        det.tick(float(k))
    assert det.state == State.TRACKING


def test_too_few_beams_is_ignored():
    det = KidnapDetector(KidnapParams(match_window=1))
    assert det.on_match(0.0, 0.0, 5) == []
    assert det.state == State.TRACKING


def test_covariance_alarm_and_recovery():
    p = KidnapParams(suspect_time=0.5, converge_count=2, converge_cov=0.1, converge_match=0.7)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 30.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 2.0, 0.1)          # 공분산 급증
    assert det.state == State.SUSPECT
    actions = det.tick(1.6)
    assert kinds(actions)[:2] == [ActionType.PUBLISH_LOST, ActionType.REINITIALIZE]
    assert det.state == State.RECOVERING
    # 수렴: AMCL 공분산 작고 일치도 높음이 2 회 연속
    assert det.on_amcl(5.0, (3.0, 4.0, 0.5), 0.02, 0.01) == []
    assert det.on_match(5.1, 0.9, 150) == []
    actions = det.on_match(5.6, 0.92, 150)
    assert kinds(actions) == [ActionType.CANCEL_SPIN, ActionType.SET_EKF_POSE,
                              ActionType.PUBLISH_LOST]
    assert actions[1].value[0] == (3.0, 4.0, 0.5)
    assert actions[2].value is False
    assert det.state == State.TRACKING and not det.lost
    assert det.recovery_times == [pytest.approx(5.6 - 1.6)]


def test_alias_margin_blocks_convergence_until_resolved():
    # 별칭(대칭 배치) 가설이 현재 추정만큼 잘 맞으면(ρ_alt ≈ ρ) 수렴을 선언하지 않고 lost 를 유지한다
    p = KidnapParams(suspect_time=0.5, converge_count=2, converge_margin=0.05)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 30.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 2.0, 0.1)
    det.tick(1.6)
    assert det.state == State.RECOVERING
    det.on_amcl(5.0, (3.0, 4.0, 0.5), 0.02, 0.01)
    for k in range(10):
        assert det.on_match(5.1 + 0.5 * k, 0.93, 150, alias_margin=0.02) == []
    assert det.state == State.RECOVERING and det.lost
    assert det.alias_rejections >= 10
    # 한 스캔만 여백을 넘으면 연속 조건(converge_count 2)을 못 채운다
    assert det.on_match(10.2, 0.95, 150, alias_margin=0.10) == []
    assert det.on_match(10.7, 0.93, 150, alias_margin=-0.01) == []
    # 회전·이동으로 별칭이 떨어져 나가면(여백 ≥ 0.05 가 2 회 연속) 수렴
    det.on_match(11.2, 0.95, 150, alias_margin=0.15)
    actions = det.on_match(11.7, 0.94, 150, alias_margin=0.32)
    assert ActionType.SET_EKF_POSE in kinds(actions)
    assert det.state == State.TRACKING and not det.lost
    # 별칭 정보가 없으면(None) 이전 규칙 (ρ 하한만)
    assert KidnapParams().converge_margin == pytest.approx(0.05)


def test_amcl_update_before_lost_does_not_count_for_convergence():
    p = KidnapParams(match_window=1, suspect_time=0.0, converge_count=1)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 10.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 0.01, 0.001)
    det.on_match(2.0, 0.1, 150)
    det.tick(2.0)
    assert det.state == State.RECOVERING
    assert det.on_match(3.0, 0.95, 150) == []     # AMCL 이 LOST 이후 아직 갱신 안 됨
    assert det.state == State.RECOVERING


def test_jump_alone_returns_to_tracking():
    det = KidnapDetector(KidnapParams(suspect_time=1.0))
    feed_odom(det, 0.0, 10.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 0.01, 0.001)
    det.on_amcl(2.0, (3.0, 0.0, 0.0), 0.01, 0.001)       # odom 은 정지인데 AMCL 이 3 m 점프
    assert det.state == State.SUSPECT
    assert 'jump' in det.events[-1].reason
    det.on_match(2.5, 0.95, 150)                          # 스캔은 잘 맞음 → 보정이었다
    assert det.tick(3.1) == []
    assert det.state == State.TRACKING


def test_spin_retries_then_next_attempts_then_failed():
    p = KidnapParams(match_window=1, suspect_time=0.0, spin_retry=2, reinit_retry=2,
                     converge_count=1)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 10.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 0.01, 0.001)
    det.on_match(2.0, 0.1, 150)
    det.tick(2.0)
    assert det.on_spin_done(10.0, True)[0].kind == ActionType.SPIN     # 같은 시도 두 번째 회전
    actions = det.on_spin_done(20.0, True)                             # 시도 2
    assert kinds(actions) == [ActionType.REINITIALIZE, ActionType.SPIN]
    assert actions[0].value == 2
    det.on_spin_done(30.0, False)                                      # 시도 2 의 재회전
    actions = det.on_spin_done(40.0, True)
    assert det.state == State.FAILED and det.lost
    assert actions == []
    # FAILED 에서도 수렴하면 복귀
    det.on_amcl(41.0, (1.0, 1.0, 0.0), 0.01, 0.001)
    actions = det.on_match(41.5, 0.9, 150)
    assert ActionType.PUBLISH_LOST in kinds(actions)
    assert det.state == State.TRACKING


def test_recovery_timeout_fails_and_cancels_spin():
    p = KidnapParams(match_window=1, suspect_time=0.0, recovery_timeout=5.0)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 10.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 0.01, 0.001)
    det.on_match(2.0, 0.1, 150)
    det.tick(2.0)
    actions = det.tick(8.0)
    assert kinds(actions) == [ActionType.CANCEL_SPIN]
    assert det.state == State.FAILED
    assert det.on_spin_done(9.0, False) == []
    assert det.state == State.FAILED


def test_spin_done_outside_recovery_is_ignored():
    det = KidnapDetector()
    assert det.on_spin_done(1.0, True) == []


def test_stop_update_scheduler():
    s = StopUpdateScheduler(count=3, period=1.0, settle_time=0.5)
    assert not s.update(0.0, 0.0, 0.0)          # 처음부터 정지: 움직인 적 없음 → 요청 없음
    assert not s.update(1.0, 0.0, 0.0)
    calls = []
    t = 2.0
    for k in range(100):                        # 1 s 주행
        assert not s.update(t, 0.5, 0.0)
        t += 0.01
    for k in range(600):                        # 6 s 정지
        if s.update(t, 0.0, 0.001):
            calls.append(round(t - 3.0, 2))
        t += 0.01
    assert len(calls) == 3
    assert calls[0] == pytest.approx(0.5, abs=0.02)
    assert calls[1] - calls[0] == pytest.approx(1.0, abs=0.02)
    s.update(t, 0.0, 0.3)                       # 제자리 회전도 움직임
    assert not StopUpdateScheduler(count=0).update(0.0, 0.0, 0.0)


def test_cooldown_blocks_immediate_realarm():
    p = KidnapParams(match_window=1, suspect_time=0.0, converge_count=1, cooldown=3.0)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 20.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 0.01, 0.001)
    det.on_match(2.0, 0.1, 150)
    det.tick(2.0)
    det.on_amcl(3.0, (5.0, 0.0, 0.0), 0.01, 0.001)
    det.on_match(3.5, 0.9, 150)
    assert det.state == State.TRACKING
    det.on_match(4.0, 0.1, 150)        # 유예 안: 무시
    assert det.state == State.TRACKING
    det.on_match(7.0, 0.1, 150)        # 유예 후: 다시 감지
    assert det.state == State.SUSPECT


def test_marker_fix_declares_lost_and_seeds_that_pose():
    """마커로 역산한 자세가 추정과 멀면 LOST + 그 자세 시드 (전역 재초기화는 하지 않는다)."""
    det = KidnapDetector()
    feed_odom(det, 0.0, 2.0)
    det.on_amcl(1.0, (-27.78, 13.0, math.pi), 0.01, 0.001)   # 믿고 있는 자리: dock_2 앞
    truth = (-27.78, 17.0, math.pi)                          # 실제로는 dock_1 앞 (4 m 옆)
    actions = det.on_marker_fix(2.0, truth)
    assert kinds(actions) == [ActionType.PUBLISH_LOST, ActionType.SEED_POSE, ActionType.SPIN]
    assert actions[0].value is True
    assert actions[1].value == truth
    assert det.lost and det.state == State.RECOVERING
    assert 'marker fix' in det.events[-2].reason


def test_marker_fix_consistent_with_estimate_is_ignored():
    """추정과 가까운 마커 보정은 정상 관측이다 — 상태를 건드리지 않는다."""
    det = KidnapDetector()
    feed_odom(det, 0.0, 2.0)
    det.on_amcl(1.0, (-27.78, 13.0, math.pi), 0.01, 0.001)
    assert det.on_marker_fix(2.0, (-27.85, 13.06, math.pi)) == []
    assert det.state == State.TRACKING and not det.lost


def test_marker_fix_needs_an_estimate_and_respects_cooldown():
    det = KidnapDetector(KidnapParams(cooldown=5.0))
    assert det.on_marker_fix(1.0, (0.0, 0.0, 0.0)) == []      # AMCL 자세를 아직 못 받았다
    feed_odom(det, 0.0, 2.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 0.01, 0.001)
    det._cooldown_until = 9.0                                 # 복구 직후 유예 구간
    assert det.on_marker_fix(2.0, (10.0, 0.0, 0.0)) == []
    assert kinds(det.on_marker_fix(9.0, (10.0, 0.0, 0.0))) == [
        ActionType.PUBLISH_LOST, ActionType.SEED_POSE, ActionType.SPIN]
