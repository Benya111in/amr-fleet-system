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
    assert det.on_marker_fix(2.0, truth) == []               # 한 번만으로는 움직이지 않는다
    actions = det.on_marker_fix(2.0 + KidnapParams().marker_fix_persist, truth)
    # 회전은 남는다: AMCL 은 움직여야 갱신하므로 정지 상태로는 수렴 판정이 오지 않는다
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
    assert det.on_marker_fix(9.0, (10.0, 0.0, 0.0)) == []     # 유예가 끝난 첫 관측 (지속 시작)
    assert kinds(det.on_marker_fix(9.7, (10.0, 0.0, 0.0))) == [
        ActionType.PUBLISH_LOST, ActionType.SEED_POSE, ActionType.SPIN]


def test_external_pose_reset_suppresses_marker_kidnap():
    """바깥에서 자세를 잡아 주면 그 수렴 구간은 납치가 아니다 (잡아 준 자세를 도로 덮지 않는다)."""
    det = KidnapDetector()
    feed_odom(det, 0.0, 2.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 0.01, 0.001)
    det.on_external_pose_reset(2.0)                           # initialpose 도착
    assert det.on_marker_fix(2.1, (30.0, 0.0, 0.0)) == []     # 수렴 구간 — 판정하지 않는다
    assert det.on_marker_fix(3.0, (30.0, 0.0, 0.0)) == []
    assert det.state == State.TRACKING and not det.lost
    # 유예(cooldown 3 s)가 지나고도 불일치가 이어지면 그때는 납치다
    assert det.on_marker_fix(5.1, (30.0, 0.0, 0.0)) == []     # 지속 시작
    assert kinds(det.on_marker_fix(5.8, (30.0, 0.0, 0.0))) == [
        ActionType.PUBLISH_LOST, ActionType.SEED_POSE, ActionType.SPIN]


def test_marker_fix_transient_mismatch_is_not_a_kidnap():
    """불일치가 잠깐 보였다 사라지면 납치가 아니다 (로봇을 옮기고 초기 자세를 알려 주는 전이)."""
    det = KidnapDetector()
    feed_odom(det, 0.0, 2.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 0.01, 0.001)
    assert det.on_marker_fix(2.0, (30.0, 0.0, 0.0)) == []     # 큰 불일치 — 지속 시작
    assert det.on_marker_fix(2.2, (0.1, 0.0, 0.0)) == []      # 다음 관측은 추정과 일치 → 지속 해제
    assert det.on_marker_fix(3.0, (30.0, 0.0, 0.0)) == []     # 다시 시작 — 아직 지속 시간 미달
    assert det.state == State.TRACKING and not det.lost


def test_scan_registration_rejections_declare_lost():
    """
    옆 통로로 6 m 옮겨졌을 때 LiDAR 쪽에서 유일하게 걸리는 신호.

    통합 11 실측 18회: LOST 235 건 중 cov 0 · jump 0 · scan-map inlier 1 건뿐이고 나머지 234 건이
    전부 ArUco 마커 보정이었다. 랙 행 간격이 정확히 6 m 라 옆 통로 스캔이 지도에 그대로 맞고,
    match_ratio 는 "지도 벽을 뚫은 빔" 만 세는 비대칭 지표라 옆 통로 이동을 보지 못한다.
    한편 스캔 정합기는 같은 구간에서 reg1 기준 0 ok / 300 rejected 를 600 s 연속 냈고
    (복구 직후 288/300), 정상 주행에서는 6 m 벽 앞 정지를 포함해 283~300/300 을 수락했다.
    """
    det = KidnapDetector(KidnapParams(suspect_time=1.0, reg_window=20, reg_min_samples=10))
    for k in range(30):                     # 정상 주행: 거의 전부 수락
        det.on_scan_match(k * 0.1, k % 25 != 0)
    assert not det.lost
    assert det.state == State.TRACKING
    t = 3.0
    for k in range(20):                     # 납치: 정합 전부 거부
        det.on_scan_match(t + k * 0.1, False)
    assert not det.lost                     # suspect_time 을 아직 못 채웠다
    actions = det.tick(t + 4.0)   # SUSPECT 진입(≈ t+1.6) 뒤 suspect_time 경과
    assert det.lost
    assert kinds(actions)[0] == ActionType.PUBLISH_LOST
    assert any('scan registration rejected' in e.reason for e in det.events)


def test_scan_registration_normal_run_does_not_trigger():
    """정상 주행의 산발적 거부(283~300/300)로는 경보가 켜지지 않는다."""
    det = KidnapDetector(KidnapParams(reg_window=20, reg_min_samples=10))
    for k in range(200):                    # 300 중 17 거부 = 5.7 %
        det.on_scan_match(k * 0.1, k % 18 != 0)
        det.tick(k * 0.1)
    assert not det.lost
    assert det.state == State.TRACKING


def test_marker_fix_is_ignored_when_scan_registration_agrees():
    """
    오검출 마커가 만드는 가짜 LOST 를 스캔 정합기가 막는다.

    통합 12 실측(reg1): 정지한 amr_05 가 36 m 밖의 마커 17 을 "2.15 m 앞" 으로 읽어 30.32 m
    어긋난 자세를 발행했고, 같은 시각 스캔 정합기는 86 ok / 1 rejected (inlier 0.93) 로 멀쩡했다.
    그 LOST 가 스스로 풀리지 않아 548 s 동안 작업을 205 회 거절했다 (s12 는 1873 회).
    진짜 납치는 반대로 정합기가 거부한다 (통합 11 reg1: 0 ok / 300 rejected, 600 s 연속).
    """
    det = KidnapDetector(KidnapParams(marker_fix_persist=0.2, reg_min_samples=10))
    det.on_amcl(0.0, (26.0, -16.0, 0.0), 0.01, 0.01)
    for k in range(20):                      # 정합기: 거의 전부 수락 = "여기 맞다"
        det.on_scan_match(k * 0.1, True)
    det.on_marker_fix(3.0, (9.76, 9.60, 1.17))      # 30 m 를 주장하는 오검출
    det.on_marker_fix(3.5, (9.76, 9.60, 1.17))      # 지속 게이트도 채운다
    assert not det.lost
    assert any('무시' in e.reason for e in det.events)

    # 같은 마커 보정이라도 정합기가 거부 중이면(진짜 납치) 그대로 LOST 를 낸다
    det2 = KidnapDetector(KidnapParams(marker_fix_persist=0.2, reg_min_samples=10))
    det2.on_amcl(0.0, (26.0, -16.0, 0.0), 0.01, 0.01)
    for k in range(20):
        det2.on_scan_match(k * 0.1, False)
    det2.on_marker_fix(3.0, (9.76, 9.60, 1.17))
    det2.on_marker_fix(3.5, (9.76, 9.60, 1.17))
    assert det2.lost


def test_external_pose_reset_lets_recovery_converge_in_an_alias_spot():
    """
    주기 별칭 자리에서 바깥이 자세를 잡아 주면 여백 게이트가 복구를 막지 않는다.

    회귀 근거: 지도에 ρ 차가 converge_margin 미만인 자리가 실제로 있다 —
    docs/research/state-estimation/checks/alias_margin.py 가 test_12 피해 로봇 자리
    (0.8, 5, 0) 에서 0.034 를 낸다. 그 자리에서는 참 자세를 알려 줘도 영원히 lost 였다
    (test_20_forced_deadlock: "배치 뒤 위치 추정이 안정되지 않음").
    """
    p = KidnapParams(suspect_time=0.5, converge_count=2, converge_margin=0.05)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 30.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 2.0, 0.1)
    det.tick(1.6)
    assert det.state == State.RECOVERING and det.lost

    # 여백 0.034 (실측값) — 바깥이 잡아 주기 전에는 몇 번을 줘도 수렴하지 않는다
    det.on_amcl(5.0, (0.8, 5.0, 0.0), 0.02, 0.01)
    for k in range(6):
        assert det.on_match(5.1 + 0.5 * k, 0.93, 150, alias_margin=0.034) == []
    assert det.lost and det.alias_rejections >= 6
    assert det.external_trust_uses == 0

    # 바깥(운영자/상위 시스템)이 initialpose 로 자세를 잡아 준다 → 같은 여백에서 수렴한다.
    # on_amcl 도 _evaluate 를 부르므로 수렴 카운트는 on_amcl 1 + on_match 1 로 채워진다.
    det.on_external_pose_reset(8.5)
    actions = det.on_amcl(9.0, (0.8, 5.0, 0.0), 0.02, 0.01) \
        + det.on_match(9.1, 0.93, 150, alias_margin=0.034)
    assert ActionType.SET_EKF_POSE in kinds(actions)
    assert det.state == State.TRACKING and not det.lost
    assert det.external_trust_uses >= 2
    # 수렴했으면 유예는 닫힌다 — 다음 복구는 다시 여백을 요구한다
    # (복구 직후 cooldown 3 s 를 지나서 경보를 올린다)
    det.on_amcl(16.0, (0.0, 0.0, 0.0), 2.0, 0.1)
    det.tick(16.6)
    assert det.state == State.RECOVERING
    rejections = det.alias_rejections
    det.on_amcl(17.0, (0.8, 5.0, 0.0), 0.02, 0.01)
    for k in range(4):
        det.on_match(17.1 + 0.5 * k, 0.93, 150, alias_margin=0.034)
    assert det.lost and det.alias_rejections > rejections


def test_external_trust_does_not_waive_the_other_convergence_conditions():
    """유예는 **별칭 여백**만 건너뛴다 — ρ 하한과 공분산 조건은 그대로 요구한다."""
    p = KidnapParams(suspect_time=0.5, converge_count=2, converge_margin=0.05)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 30.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 2.0, 0.1)
    det.tick(1.6)
    det.on_external_pose_reset(2.0)

    # ρ 가 converge_match(0.7) 미만이면 유예가 있어도 수렴하지 않는다
    det.on_amcl(5.0, (0.8, 5.0, 0.0), 0.02, 0.01)
    for k in range(4):
        assert det.on_match(5.1 + 0.5 * k, 0.60, 150, alias_margin=0.034) == []
    assert det.lost
    # 공분산이 크면(여기서는 trace 2.0) 역시 수렴하지 않는다
    det.on_amcl(7.5, (0.8, 5.0, 0.0), 2.0, 0.01)
    for k in range(4):
        assert det.on_match(7.6 + 0.5 * k, 0.93, 150, alias_margin=0.034) == []
    assert det.lost


def test_external_trust_expires_and_closes_on_a_new_kidnap():
    """유예는 창이 지나면 닫히고, 새 LOST 선언에서도 닫힌다 (잡아 준 자세가 더는 유효하지 않다)."""
    p = KidnapParams(suspect_time=0.5, converge_count=2, converge_margin=0.05,
                     external_trust_window=5.0)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 60.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 2.0, 0.1)
    det.tick(1.6)
    det.on_external_pose_reset(2.0)          # 창 = 2.0 ~ 7.0 s

    # 창 밖(t ≥ 7.0)에서는 여백 게이트가 그대로 산다
    det.on_amcl(20.0, (0.8, 5.0, 0.0), 0.02, 0.01)
    before = det.alias_rejections
    for k in range(4):
        assert det.on_match(20.1 + 0.5 * k, 0.93, 150, alias_margin=0.034) == []
    assert det.lost and det.alias_rejections > before
    assert det.external_trust_uses == 0

    # 새 LOST 선언이 유예를 닫는다
    det.on_external_pose_reset(25.0)
    det._declare_lost(25.5, '새 납치')
    det.on_amcl(26.0, (0.8, 5.0, 0.0), 0.02, 0.01)
    rejections = det.alias_rejections
    for k in range(4):
        det.on_match(26.1 + 0.5 * k, 0.93, 150, alias_margin=0.034)
    assert det.lost and det.alias_rejections > rejections


def test_alias_gate_still_blocks_self_recovery_without_external_help():
    """회귀 방어: 바깥 도움이 없으면 여백 게이트는 예전 그대로 막는다 (이 수정의 안전 속성)."""
    p = KidnapParams(suspect_time=0.5, converge_count=2, converge_margin=0.05)
    det = KidnapDetector(p)
    feed_odom(det, 0.0, 30.0)
    det.on_amcl(1.0, (0.0, 0.0, 0.0), 2.0, 0.1)
    det.tick(1.6)
    det.on_amcl(5.0, (3.0, 4.0, 0.5), 0.02, 0.01)
    for k in range(10):
        assert det.on_match(5.1 + 0.5 * k, 0.93, 150, alias_margin=0.02) == []
    assert det.state == State.RECOVERING and det.lost
    assert det.alias_rejections >= 10 and det.external_trust_uses == 0
