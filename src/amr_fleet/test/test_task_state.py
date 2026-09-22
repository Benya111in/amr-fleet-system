"""task_state: 상태 머신 전이·이벤트·거리 적분·작업 로그 (명세 4.8)."""

import csv
import datetime as dt

import pytest

from amr_fleet.task_schema import (
    STATUS_COMPLETED, STATUS_FAILED, STATUS_IN_PROGRESS, STATUS_PENDING, Pose2D, TaskSpec,
)
from amr_fleet.task_state import (
    InvalidTransition, TaskEvent, TaskLogWriter, TaskRecord, TaskStateMachine,
)


def spec(task_id='t1', robot_id='', deadline=None):
    return TaskSpec(task_id=task_id, pickup=Pose2D(0, 0), dropoff=Pose2D(5, 0), item_type='small',
                    item_mass=2.0, robot_id=robot_id, deadline=deadline)


def test_happy_path_emits_events_and_records_times():
    events = []
    sm = TaskStateMachine(on_event=events.append)
    rec = sm.add(spec(), now=10.0)
    assert rec.status == STATUS_PENDING and rec.created_time == 10.0
    assert sm.start('t1', 'amr_01', now=12.0).new_status == STATUS_IN_PROGRESS
    assert rec.robot_id == 'amr_01' and rec.spec.robot_id == 'amr_01'
    assert sm.active_task_of('amr_01') is rec
    ev = sm.complete('t1', now=20.0)
    assert (ev.old_status, ev.new_status) == (STATUS_IN_PROGRESS, STATUS_COMPLETED)
    assert ev.name == 'COMPLETED'
    assert rec.duration_s == 8.0 and rec.result == 'completed' and rec.is_terminal()
    assert [e.new_status for e in events] == [STATUS_PENDING, STATUS_IN_PROGRESS, STATUS_COMPLETED]
    assert events == sm.events
    assert sm.active_task_of('amr_01') is None
    assert sm.counts() == {'pending': 0, 'in_progress': 0, 'completed': 1, 'failed': 0}


def test_fail_from_pending_and_in_progress():
    sm = TaskStateMachine()
    sm.add(spec('a'), 0.0)
    sm.add(spec('b'), 0.0)
    ev = sm.fail('a', 1.0, reason='cancelled')
    assert ev.new_status == STATUS_FAILED and sm.get('a').result == 'failed:cancelled'
    assert sm.get('a').duration_s is None
    sm.start('b', 'amr_02', 1.0)
    sm.fail('b', 3.0)
    assert sm.get('b').result == 'failed' and sm.get('b').duration_s == 2.0
    assert sm.counts()['failed'] == 2
    assert sm.with_status(STATUS_FAILED) == [sm.get('a'), sm.get('b')]


def test_invalid_transitions_rejected():
    sm = TaskStateMachine()
    sm.add(spec(), 0.0)
    with pytest.raises(InvalidTransition):
        sm.complete('t1', 1.0)                       # PENDING → COMPLETED 불가
    sm.start('t1', 'amr_01', 1.0)
    with pytest.raises(InvalidTransition):
        sm.start('t1', 'amr_02', 2.0)                # 다른 로봇으로 재시작 불가
    sm.complete('t1', 3.0)
    for op in (lambda: sm.start('t1', 'amr_01', 4.0), lambda: sm.fail('t1', 4.0),
               lambda: sm.complete('t1', 4.0)):
        with pytest.raises(InvalidTransition):
            op()
    with pytest.raises(KeyError):
        sm.start('nope', 'amr_01', 0.0)
    with pytest.raises(ValueError):
        sm.add(spec(), 0.0)


def test_start_is_idempotent_for_same_robot():
    sm = TaskStateMachine()
    sm.add(spec(), 0.0)
    assert sm.start('t1', 'amr_01', 1.0) is not None
    assert sm.start('t1', 'amr_01', 2.0) is None
    assert sm.get('t1').attempts == 1 and sm.get('t1').start_time == 1.0
    assert len(sm.events) == 2


def test_distance_integration_ignores_idle_robot_and_jumps():
    sm = TaskStateMachine(max_pose_step_m=5.0)
    sm.add(spec(), 0.0)
    assert sm.update_pose('amr_01', 0, 0) == 0.0          # 진행 중 작업 없음
    sm.start('t1', 'amr_01', 0.0)
    assert sm.update_pose('amr_01', 0, 0) == 0.0          # 첫 자세: 기준점만 잡음
    assert sm.update_pose('amr_01', 3, 4) == 5.0
    assert sm.update_pose('amr_01', 3, 4) == 0.0
    assert sm.update_pose('amr_01', 103, 4) == 0.0         # 100 m 점프 → 무시
    assert sm.update_pose('amr_01', 104, 4) == 1.0
    assert sm.get('t1').distance_m == pytest.approx(6.0)


def test_retry_within_limit_restores_pin_and_resets():
    sm = TaskStateMachine(max_retries=1)
    sm.add(spec('p', robot_id='amr_05'), 0.0)
    sm.start('p', 'amr_05', 1.0, command_time=0.9)
    assert sm.get('p').command_time == 0.9
    sm.update_pose('amr_05', 0, 0)
    sm.update_pose('amr_05', 1, 0)
    sm.fail('p', 2.0, 'nav_failed')
    assert sm.can_retry('p')
    ev = sm.retry('p', 3.0)
    assert ev.new_status == STATUS_PENDING and ev.reason == 'retry'
    rec = sm.get('p')
    assert rec.robot_id == 'amr_05' and rec.spec.robot_id == 'amr_05'
    assert rec.start_time is None and rec.distance_m == 0.0 and rec.result == ''
    assert rec.command_time is None
    sm.start('p', 'amr_05', 4.0)
    sm.fail('p', 5.0)
    assert rec.attempts == 2 and rec.failures == 2
    assert not sm.can_retry('p') and sm.retry('p', 6.0) is None


def test_fail_from_pending_counts_as_failure():
    # 기본 max_retries=0 이면 시작 전 실패도 재시도하지 않는다 (예전: attempts=0 이라 무한 재시도)
    sm = TaskStateMachine()
    sm.add(spec('a'), 0.0)
    sm.fail('a', 1.0, 'robot_reported')
    assert sm.get('a').failures == 1 and sm.get('a').attempts == 0
    assert not sm.can_retry('a') and sm.retry('a', 2.0) is None
    # max_retries=1: 시작 전 실패 → 한 번만 재시도, 두 번째 실패는 끝
    sm1 = TaskStateMachine(max_retries=1)
    sm1.add(spec('b'), 0.0)
    sm1.fail('b', 1.0)
    assert sm1.retry('b', 1.5).new_status == STATUS_PENDING
    sm1.fail('b', 2.0)
    assert sm1.get('b').failures == 2 and not sm1.can_retry('b')
    assert sm1.retry('b', 3.0) is None and sm1.get('b').status == STATUS_FAILED
    assert not sm1.can_retry('nope')


def test_retry_not_allowed_by_default_or_for_other_states():
    sm = TaskStateMachine()
    sm.add(spec(), 0.0)
    assert sm.retry('t1', 1.0) is None
    sm.start('t1', 'amr_01', 1.0)
    sm.fail('t1', 2.0)
    assert sm.retry('t1', 3.0) is None


def test_check_deadlines_flags_once():
    sm = TaskStateMachine()
    sm.add(spec('late', deadline=10.0), 0.0)
    sm.add(spec('ok', deadline=100.0), 0.0)
    sm.add(spec('none'), 0.0)
    assert sm.check_deadlines(5.0) == []
    missed = sm.check_deadlines(11.0)
    assert [r.task_id for r in missed] == ['late'] and missed[0].deadline_missed
    assert sm.check_deadlines(12.0) == []
    sm.start('ok', 'amr_01', 12.0)
    sm.complete('ok', 13.0)
    assert sm.check_deadlines(200.0) == []          # 종료된 작업은 검사하지 않는다
    assert len(sm) == 3 and 'late' in sm and sm.records()[0].task_id == 'late'


def test_log_writer_format_and_rollover(tmp_path):
    t0 = dt.datetime(2026, 9, 22, 9, 0, 0).timestamp()
    wall = [t0]
    log = TaskLogWriter(tmp_path / 'logs', time_format='iso', wall_clock=lambda: wall[0])
    sm = TaskStateMachine(log_writer=log)
    sm.add(spec('done'), t0)
    sm.start('done', 'amr_01', t0 + 1.0)
    sm.update_pose('amr_01', 0, 0)
    sm.update_pose('amr_01', 0, 2.5)
    sm.complete('done', t0 + 4.5)
    sm.add(spec('never_started'), t0)
    sm.fail('never_started', t0 + 5.0, 'deadline')
    path = log.path_for(t0)
    assert path.name == 'tasks_20260922.csv' and log.rows_written == 2
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.reader(f))
    assert rows[0] == ['task_id', 'robot_id', 'start_time', 'end_time', 'distance_m',
                       'duration_s', 'result']
    assert rows[1] == ['done', 'amr_01', '2026-09-22T09:00:01.000', '2026-09-22T09:00:04.500',
                       '2.500', '3.500', 'completed']
    assert rows[2] == ['never_started', '', '', '2026-09-22T09:00:05.000', '0.000', '',
                       'failed:deadline']
    # 다음 날(벽시계) 기록 → 새 파일 + 헤더
    sm.add(spec('next_day'), t0)
    sm.start('next_day', 'amr_02', t0 + 10.0)
    wall[0] = t0 + 86400.0
    sm.complete('next_day', t0 + 86400.0)
    assert (tmp_path / 'logs' / 'tasks_20260923.csv').read_text(encoding='utf-8').startswith(
        'task_id,robot_id')


def test_log_writer_epoch_format_and_validation(tmp_path):
    wall = dt.datetime(2026, 9, 22, 12, 0, 0).timestamp()
    log = TaskLogWriter(tmp_path, time_format='epoch', wall_clock=lambda: wall)
    rec = TaskRecord(spec=spec(), robot_id='amr_01', start_time=1.0, end_time=2.25,
                     distance_m=1.23456, result='completed', status=STATUS_COMPLETED)
    assert log.row_for(rec) == ['t1', 'amr_01', '1.000', '2.250', '1.235', '1.250', 'completed']
    log.write(rec)
    log.write(rec)
    # use_sim_time(노드 시계 1~2 s)이어도 파일 날짜는 벽시계 → tasks_19700101.csv 가 생기지 않는다
    assert log.path_for().name == 'tasks_20260922.csv' == log.path_for(wall).name
    assert log.path_for().read_text(encoding='utf-8').count('\n') == 3
    assert not list(tmp_path.glob('tasks_1970*.csv'))
    with pytest.raises(ValueError):
        log.write(TaskRecord(spec=spec()))
    with pytest.raises(ValueError):
        TaskLogWriter(tmp_path, time_format='unix')
    assert log.format_time(None) == ''


def test_log_write_failure_does_not_undo_transition(tmp_path):
    blocker = tmp_path / 'not_a_dir'
    blocker.write_text('x', encoding='utf-8')
    errors = []
    sm = TaskStateMachine(log_writer=TaskLogWriter(blocker / 'logs'),
                          on_log_error=lambda rec, exc: errors.append((rec.task_id, exc)))
    sm.add(spec(), 0.0)
    sm.start('t1', 'amr_01', 1.0)
    sm.complete('t1', 2.0)
    assert sm.get('t1').status == STATUS_COMPLETED and errors and errors[0][0] == 't1'
    strict = TaskStateMachine(log_writer=TaskLogWriter(blocker / 'logs'))
    strict.add(spec('t2'), 0.0)
    with pytest.raises(OSError):
        strict.fail('t2', 1.0)


def test_task_event_dataclass():
    ev = TaskEvent('t', 'r', STATUS_PENDING, STATUS_IN_PROGRESS, 1.0)
    assert ev.name == 'IN_PROGRESS' and ev.reason == ''
