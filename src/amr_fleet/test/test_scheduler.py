"""scheduler: 우선순위 밴드 + EDF + 기아 방지 (명세 4.8 스케줄링)."""

import pytest

from amr_fleet.scheduler import SchedulerConfig, TaskScheduler, is_deadline_missed
from amr_fleet.task_schema import Pose2D, TaskSpec


def task(task_id, priority=0, deadline=None, created=0.0):
    return TaskSpec(task_id=task_id, pickup=Pose2D(), dropoff=Pose2D(1, 1), item_type='small',
                    item_mass=2.0, priority=priority, deadline=deadline, created=created)


def no_boost():
    return SchedulerConfig(age_boost_rate=0.0)


def ids(specs):
    return [t.task_id for t in specs]


def test_priority_bands_come_first():
    s = TaskScheduler(no_boost())
    s.push(task('low', priority=10, deadline=5.0))
    s.push(task('high', priority=200, deadline=500.0))
    s.push(task('mid', priority=100))
    assert ids(s.ordered(0.0)) == ['high', 'mid', 'low']


def test_edf_within_same_band():
    s = TaskScheduler(no_boost())
    s.push(task('late', priority=40, deadline=300.0))
    s.push(task('soon', priority=41, deadline=30.0))
    s.push(task('none', priority=42))              # 마감 없음 → 밴드 안에서 마지막
    assert ids(s.ordered(0.0)) == ['soon', 'late', 'none']


def test_same_band_no_deadline_fifo_tiebreak():
    s = TaskScheduler(no_boost())
    s.push(task('second', created=2.0))
    s.push(task('first', created=1.0))
    assert ids(s.ordered(10.0)) == ['first', 'second']


def test_fifo_weight_can_override_edf():
    cfg = SchedulerConfig(age_boost_rate=0.0, deadline_weight=0.0, fifo_weight=1.0)
    s = TaskScheduler(cfg)
    s.push(task('old_late', created=0.0, deadline=1000.0))
    s.push(task('new_soon', created=5.0, deadline=10.0))
    assert ids(s.ordered(6.0)) == ['old_late', 'new_soon']


def test_starvation_guard_promotes_old_low_priority_task():
    cfg = SchedulerConfig(band_width=32, age_boost_rate=1.0, age_boost_max=64.0)
    s = TaskScheduler(cfg)
    s.push(task('old_low', priority=0, created=0.0))
    s.push(task('fresh_high', priority=40, created=100.0))
    # t=100: old_low 유효 우선순위 = 0 + min(64, 100) = 64 → 밴드 2 > fresh_high 밴드 1
    assert ids(s.ordered(100.0)) == ['old_low', 'fresh_high']
    # 갓 들어왔을 때는 fresh_high 가 먼저
    assert ids(s.ordered(0.0)) == ['fresh_high', 'old_low']
    assert s.effective_priority(s.get('old_low'), 1000.0) == 64.0   # 상한


def test_effective_priority_and_band_arithmetic():
    s = TaskScheduler(SchedulerConfig(band_width=32, age_boost_rate=0.5, age_boost_max=64.0))
    t = task('t', priority=30, created=10.0)
    assert s.effective_priority(t, 10.0) == 30.0
    assert s.effective_priority(t, 20.0) == 35.0
    assert s.band(t, 10.0) == 0 and s.band(t, 20.0) == 1
    assert s.effective_priority(t, 5.0) == 30.0          # 미래에 생성된 작업도 음수 나이 없음


def test_peek_pop_remove_and_container_protocol():
    s = TaskScheduler(no_boost())
    assert s.peek(0.0) is None and s.pop(0.0) is None
    s.push(task('a', priority=1))
    s.push(task('b', priority=100))
    assert len(s) == 2 and 'a' in s and 'zzz' not in s
    assert {t.task_id for t in s} == {'a', 'b'}
    assert s.peek(0.0).task_id == 'b'
    assert s.pop(0.0).task_id == 'b'
    assert s.remove('a').task_id == 'a'
    assert s.remove('a') is None
    assert len(s) == 0
    s.push(task('c'))
    s.clear()
    assert len(s) == 0


def test_duplicate_push_rejected():
    s = TaskScheduler()
    s.push(task('dup'))
    with pytest.raises(ValueError):
        s.push(task('dup'))


def test_overdue_detection():
    s = TaskScheduler(no_boost())
    s.push(task('ok', deadline=100.0))
    s.push(task('late2', deadline=20.0))
    s.push(task('late1', deadline=10.0))
    s.push(task('never'))
    assert ids(s.overdue(50.0)) == ['late1', 'late2']
    assert s.overdue(5.0) == []
    assert is_deadline_missed(task('x', deadline=1.0), 1.0) is False
    assert is_deadline_missed(task('x', deadline=1.0), 1.01) is True


def test_config_validation():
    with pytest.raises(ValueError):
        SchedulerConfig(band_width=0)
    with pytest.raises(ValueError):
        SchedulerConfig(age_boost_rate=-1.0)


def test_sort_key_is_deterministic():
    s = TaskScheduler(no_boost())
    a, b = task('a', priority=5, deadline=50.0), task('b', priority=5, deadline=50.0)
    assert s.sort_key(a, 0.0) < s.sort_key(b, 0.0)
