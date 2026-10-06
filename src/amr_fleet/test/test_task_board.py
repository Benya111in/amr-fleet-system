"""task_board: 게시판 멱등 규칙 · 전역 task_id · epoch 결정 (보너스 4 P0, 순수 모듈)."""

import math
import re

import pytest

from amr_fleet.task_board import (
    ACCEPT, CONFLICT, DUPLICATE, STALE, UPDATE, CommandStamp, TaskContent, TaskLedger,
    make_task_id, resolve_epoch,
)
from amr_fleet.task_schema import TASK_ID_RE, Pose2D, TaskSpec


def _spec(task_id='src.000001', x=1.0, yaw=0.0, priority=10, robot='', version=0, deadline=None):
    return TaskSpec(task_id=task_id, pickup=Pose2D(x, 2.0, yaw), dropoff=Pose2D(5.0, 6.0),
                    item_type='small', item_mass=2.0, priority=priority, deadline=deadline,
                    robot_id=robot, version=version)


def test_ledger_rules_follow_vda5050_idempotency():
    led = TaskLedger()
    a = _spec(version=1)
    assert led.check(a, 1) == ACCEPT and 'src.000001' not in led
    led.record(a, 1)
    assert 'src.000001' in led and len(led) == 1
    assert led.check(_spec(version=1), 1) == DUPLICATE                  # 재전송
    assert led.check(_spec(deadline=99.0), 1) == DUPLICATE              # 마감 · 접수 시각은 내용 밖
    assert led.check(_spec(x=1.5), 1) == CONFLICT                       # 같은 판, 다른 내용
    assert led.check(_spec(priority=11), 1) == CONFLICT
    assert led.check(_spec(robot='amr_02'), 1) == CONFLICT
    assert led.check(_spec(), 0) == STALE                               # 옛 판
    assert led.check(_spec(), 2) == UPDATE                              # 새 판 (P0: 갱신 미지원)
    assert led.check(_spec(task_id='src.000002'), 0) == ACCEPT


def test_content_tolerates_quaternion_round_trip_and_yaw_wrap():
    a = TaskContent.of(_spec(yaw=math.pi))
    b = TaskContent.of(_spec(yaw=-math.pi + 1e-9))                     # 같은 방향
    assert a.same_as(b)
    assert TaskContent.of(_spec(x=1.0)).same_as(TaskContent.of(_spec(x=1.0 + 5e-7)))
    assert not TaskContent.of(_spec(x=1.0)).same_as(TaskContent.of(_spec(x=1.0 + 1e-3)))
    other_frame = _spec()
    other_frame.dropoff = Pose2D(5.0, 6.0, 0.0, 'odom')
    assert not TaskContent.of(_spec()).same_as(TaskContent.of(other_frame))


def test_make_task_id_is_global_and_valid():
    assert make_task_id('dash', 7) == 'dash.000007'
    ids = {make_task_id('itest') for _ in range(200)}
    assert len(ids) == 200
    assert all(TASK_ID_RE.match(t) and re.fullmatch(r'itest\.[0-9a-f]{16}', t) for t in ids)
    for bad in ('', 'a.b', 'x' * 70):
        with pytest.raises(ValueError):
            make_task_id(bad, 1)


def test_resolve_epoch_param_file_and_default(tmp_path):
    assert resolve_epoch() == 1
    assert resolve_epoch(5, str(tmp_path / 'ignored')) == 5             # 파라미터가 이긴다
    assert not (tmp_path / 'ignored').exists()
    f = tmp_path / 'sub' / 'epoch'
    assert [resolve_epoch(0, str(f)) for _ in range(3)] == [1, 2, 3]    # 재시작마다 +1, 영속
    assert f.read_text().strip() == '3'
    assert not list((tmp_path / 'sub').glob('.epoch_*'))                # 임시 파일 남기지 않음
    f.write_text('')
    assert resolve_epoch(0, str(f)) == 1
    with pytest.raises(ValueError):
        resolve_epoch(-1)
    f.write_text('-4')
    with pytest.raises(ValueError):
        resolve_epoch(0, str(f))


def test_command_stamp_sequence():
    c = CommandStamp(3, '/fleet/fleet_manager_node')
    assert [c.next() for _ in range(3)] == [
        (3, '/fleet/fleet_manager_node', 1), (3, '/fleet/fleet_manager_node', 2),
        (3, '/fleet/fleet_manager_node', 3)]
