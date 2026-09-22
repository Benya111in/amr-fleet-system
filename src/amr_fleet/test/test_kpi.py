"""kpi: 처리량·평균 작업 시간·가동률·교착 카운트 산술 (명세 4.9 모니터링)."""

import pytest

from amr_fleet.kpi import (
    STATUS_CHARGING, STATUS_DOCKING, STATUS_ESTOP, STATUS_IDLE, STATUS_LOADING, STATUS_MOVING,
    KpiTracker,
)


def test_throughput_sliding_window():
    k = KpiTracker(throughput_window_s=100.0)
    assert k.throughput(0.0) == 0.0
    k.start(0.0)
    for t in (10.0, 20.0, 30.0):
        k.record_completion(t, duration_s=5.0)
    # 가동 50 s < 창 100 s → 3 / 50 s = 216 tasks/h
    assert k.throughput(50.0) == pytest.approx(3 / 50 * 3600)
    # 가동 200 s → 창 100 s 안(100, 200] 에는 완료 없음
    assert k.throughput(200.0) == 0.0
    k.record_completion(150.0, 2.0)
    assert k.throughput(200.0) == pytest.approx(1 / 100 * 3600)
    assert k.completed == 4


def test_throughput_min_window_guards_division():
    k = KpiTracker(throughput_window_s=600.0, min_window_s=1.0)
    k.record_completion(0.0, 1.0)
    assert k.throughput(0.0) == pytest.approx(3600.0)


def test_avg_task_duration_and_failures():
    k = KpiTracker()
    assert k.avg_task_duration() == 0.0
    k.record_completion(1.0, 4.0)
    k.record_completion(2.0, 6.0)
    k.record_completion(3.0, -1.0)          # 음수 방어 → 0
    assert k.avg_task_duration() == pytest.approx(10.0 / 3)
    k.record_failure(4.0)
    assert k.failed == 1


def test_robot_utilization_time_weighted():
    k = KpiTracker(utilization_mode='moving')
    k.update_robot('a', STATUS_IDLE, 0.0)
    k.update_robot('a', STATUS_MOVING, 10.0)     # 0~10 idle
    k.update_robot('a', STATUS_IDLE, 30.0)       # 10~30 busy
    assert k.robot_busy_time('a', 40.0) == pytest.approx(20.0)
    assert k.robot_utilization(40.0) == pytest.approx(0.5)
    # 현재 busy 상태는 now 까지 연장해서 센다
    k.update_robot('a', STATUS_DOCKING, 40.0)
    assert k.robot_utilization(50.0) == pytest.approx(30.0 / 50.0)
    # 두 번째 로봇: 20 s 관측, 전부 LOADING
    k.update_robot('b', STATUS_LOADING, 30.0)
    assert k.robot_utilization(50.0) == pytest.approx((30.0 + 20.0) / (50.0 + 20.0))
    assert k.robot_busy_time('zzz', 50.0) == 0.0


def test_charging_and_error_are_not_busy():
    k = KpiTracker(utilization_mode='moving')
    k.update_robot('a', STATUS_CHARGING, 0.0)
    assert k.robot_utilization(10.0) == 0.0
    k.update_robot('a', STATUS_MOVING, 10.0)
    k.update_robot('a', STATUS_MOVING, 5.0)      # 과거 시각 갱신은 무시(시간 역행 방어)
    assert k.robot_utilization(20.0) == pytest.approx(0.5)


def test_deadlock_count_and_snapshot():
    k = KpiTracker(throughput_window_s=60.0)
    k.record_deadlock()
    k.record_deadlock(2)
    k.record_deadlock(-5)
    k.update_robot('a', STATUS_MOVING, 0.0, assigned=True)   # 집계 시작 = 0 s
    k.record_completion(10.0, 3.0)
    snap = k.snapshot(20.0, {'pending': 2, 'in_progress': 1, 'completed': 1, 'failed': 0})
    assert snap == {
        'tasks_pending': 2, 'tasks_in_progress': 1, 'tasks_completed': 1, 'tasks_failed': 0,
        'throughput': pytest.approx(1 / 20 * 3600), 'avg_task_duration': 3.0,
        'robot_utilization': 1.0, 'deadlock_count': 3,
    }
    # counts 없이도 자체 카운터로 채운다
    assert k.snapshot(20.0)['tasks_completed'] == 1


def test_invalid_window_and_mode():
    with pytest.raises(ValueError):
        KpiTracker(throughput_window_s=0.0)
    with pytest.raises(ValueError):
        KpiTracker(utilization_mode='busy')


def test_utilization_assigned_mode_counts_task_time_not_motion():
    # 기본 모드: 진행 중 작업이 있는 시간 (작업 로그 duration 합과 같은 구간)
    k = KpiTracker()
    assert k.utilization_mode == 'assigned'
    k.update_robot('a', STATUS_IDLE, 0.0)
    k.update_robot('a', STATUS_IDLE, 10.0, assigned=True)       # 작업 시작 10 s
    k.update_robot('a', STATUS_ESTOP, 14.0, assigned=True)      # 멈춰도 작업 중이면 가동
    k.update_robot('a', STATUS_MOVING, 16.0, assigned=True)
    k.update_robot('a', STATUS_IDLE, 25.0, assigned=False)      # 작업 종료 25 s
    k.update_robot('a', STATUS_MOVING, 30.0, assigned=False)    # 작업 없이 이동은 가동 아님
    assert k.robot_busy_time('a', 40.0) == pytest.approx(15.0)
    assert k.robot_utilization(40.0) == pytest.approx(15.0 / 40.0)
    # 같은 입력을 moving 모드로: MOVING 구간만 (16~25, 30~40)
    m = KpiTracker(utilization_mode='moving')
    for status, t, assigned in ((STATUS_IDLE, 0.0, False), (STATUS_IDLE, 10.0, True),
                                (STATUS_ESTOP, 14.0, True), (STATUS_MOVING, 16.0, True),
                                (STATUS_IDLE, 25.0, False), (STATUS_MOVING, 30.0, False)):
        m.update_robot('a', status, t, assigned=assigned)
    assert m.robot_busy_time('a', 40.0) == pytest.approx(19.0)
    assert m.is_busy(STATUS_LOADING, False) and not k.is_busy(STATUS_LOADING, False)
