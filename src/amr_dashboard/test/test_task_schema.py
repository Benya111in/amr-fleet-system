"""task_schema: JSON Task Description 검증·정규화 (/fleet/task_request 와이어 형식), E-stop 입력 검증."""

import datetime
import json
import math
import os

import pytest

from amr_dashboard import task_schema as ts

NOW = 1_800_000_000.0
WORLD = {'origin_x': 0.0, 'origin_y': 0.0, 'width': 60.0, 'height': 40.0}
WIRE_KEYS = {'task_id', 'priority', 'deadline', 'pickup', 'dropoff', 'item_type', 'robot_id'}


def full_payload(**overrides):
    payload = {
        'priority': 200, 'deadline_sec': 120,
        'pickup_pose': {'x': 5.0, 'y': 5.0, 'yaw': 0.0},
        'dropoff_pose': {'x': 50.0, 'y': 30.0, 'yaw': 4.0},
        'item_type': 'large',
    }
    payload.update(overrides)
    return payload


def test_valid_task_normalised_to_wire_format():
    task, errors = ts.validate_task(full_payload(), now=NOW, world=WORLD)
    assert errors == []
    assert set(task) == WIRE_KEYS - {'robot_id'}   # 자동 할당이면 robot_id 키 없음
    assert task['task_id'].startswith('T-') and ts.TASK_ID_RE.match(task['task_id'])
    assert task['priority'] == 200
    assert task['deadline'] == 120.0                 # 지금부터 초 그대로 (플릿 매니저 시계 기준)
    assert task['pickup'] == {'x': 5.0, 'y': 5.0, 'yaw': 0.0, 'frame_id': 'map'}
    assert task['dropoff']['yaw'] == pytest.approx(4.0 - 2 * math.pi)  # [-pi, pi] 정규화
    assert task['item_type'] == 'large'
    json.dumps(task, allow_nan=False)


def test_canonical_names_and_explicit_fields():
    payload = {
        'task_id': 'T-abc.1:2', 'robot_id': 'amr_03', 'priority': 0, 'deadline': 3600,
        'pickup': {'x': 1, 'y': 2, 'frame_id': 'map'}, 'dropoff': {'x': 3, 'y': 4},
        'item_type': 'small',
    }
    task, errors = ts.validate_task(payload, now=NOW)
    assert errors == []
    assert task == {
        'task_id': 'T-abc.1:2', 'priority': 0, 'deadline': 3600.0,
        'pickup': {'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'frame_id': 'map'},
        'dropoff': {'x': 3.0, 'y': 4.0, 'yaw': 0.0, 'frame_id': 'map'},
        'item_type': 'small', 'robot_id': 'amr_03',
    }


def test_defaults_priority_zero_and_no_deadline():
    task, errors = ts.validate_task(
        {'pickup': {'x': 1, 'y': 1}, 'dropoff': {'x': 2, 'y': 2}, 'item_type': 'medium'}, now=NOW)
    assert errors == []
    assert task['priority'] == ts.DEFAULT_PRIORITY == 0
    assert 'deadline' not in task and 'robot_id' not in task
    task, errors = ts.validate_task(full_payload(deadline_sec=None, deadline=''), now=NOW)
    assert errors == [] and 'deadline' not in task


def test_deadline_iso_normalised_with_timezone():
    payload = full_payload(deadline_sec=None, deadline='2027-01-01T00:00:00Z')
    task, errors = ts.validate_task(payload)
    assert errors == []
    assert task['deadline'] == '2027-01-01T00:00:00+00:00'
    # 시간대 없는 ISO 문자열은 이 컴퓨터의 로컬 시간으로 보고 오프셋을 붙인다
    payload = full_payload(deadline_sec=None, deadline='2027-01-01T09:00:00')
    task, errors = ts.validate_task(payload)
    assert errors == []
    parsed = datetime.datetime.fromisoformat(task['deadline'])
    assert parsed.tzinfo is not None
    assert parsed.timestamp() == pytest.approx(
        datetime.datetime(2027, 1, 1, 9, 0, 0).astimezone().timestamp())


@pytest.mark.parametrize('payload, fragment', [
    (None, 'JSON 객체'),
    ([], 'JSON 객체'),
    (full_payload(item_mass=10.0), '모르는 필드: item_mass'),
    (full_payload(status=0), '모르는 필드: status'),
    (full_payload(task_id='bad id!'), 'task_id'),
    (full_payload(task_id='x' * 65), 'task_id'),
    (full_payload(robot_id='AMR-01'), 'robot_id'),
    (full_payload(robot_id=3), 'robot_id'),
    (full_payload(priority=256), 'priority'),
    (full_payload(priority=-1), 'priority'),
    (full_payload(priority=True), 'priority'),
    (full_payload(priority='high'), 'priority'),
    (full_payload(deadline_sec=-5), 'deadline'),
    (full_payload(deadline_sec=0), '0 보다 크고'),
    (full_payload(deadline_sec=None, deadline=0.0), '0 보다 크고'),
    (full_payload(deadline_sec=1e9), 'deadline'),
    (full_payload(deadline_sec='soon'), 'deadline_sec 는 숫자'),
    (full_payload(deadline_sec=float('inf')), 'deadline'),
    (full_payload(deadline_sec=None, deadline='not-a-date'), 'ISO 8601'),
    (full_payload(deadline_sec=None, deadline=-1), 'deadline'),
    (full_payload(deadline_sec=None, deadline=[1]), 'deadline'),
    (full_payload(deadline=60), 'deadline_sec / deadline 중 하나만'),
    (full_payload(pickup={'x': 1, 'y': 1}), 'pickup / pickup_pose 중 하나만'),
    (full_payload(pickup_pose=None), 'pickup 가 필요'),
    (full_payload(pickup_pose='here'), 'pickup 은'),
    (full_payload(pickup_pose={'x': float('nan'), 'y': 1}), 'pickup.x'),
    (full_payload(pickup_pose={'x': 1, 'y': '2'}), 'pickup.y'),
    (full_payload(pickup_pose={'x': 1, 'y': 2, 'yaw': 'n'}), 'pickup.yaw'),
    (full_payload(pickup_pose={'x': 1, 'y': 2, 'frame_id': ''}), 'frame_id'),
    (full_payload(pickup_pose={'x': 1, 'y': 2, 'z': 0.0}), 'pickup.z 는 모르는 필드'),
    (full_payload(dropoff_pose={'x': 1, 'y': 2}, pickup_pose={'x': 70, 'y': 2}), '지도 범위 밖'),
    (full_payload(item_type='huge'), 'item_type'),
    (full_payload(item_type=None), 'item_type'),
])
def test_invalid_payloads(payload, fragment):
    task, errors = ts.validate_task(payload, now=NOW, world=WORLD)
    assert task is None
    assert any(fragment in e for e in errors), errors


def test_multiple_errors_reported_together():
    task, errors = ts.validate_task({'priority': 999, 'item_type': 'x'}, now=NOW)
    assert task is None
    assert len(errors) >= 4  # priority, pickup, dropoff, item_type


def test_world_bounds_optional():
    payload = full_payload(pickup_pose={'x': -100, 'y': -100})
    task, errors = ts.validate_task(payload, now=NOW)
    assert errors == [] and task['pickup']['x'] == -100.0


def test_make_task_id_format():
    tid = ts.make_task_id(NOW)
    assert ts.TASK_ID_RE.match(tid)
    assert tid.startswith('T-') and len(tid) == len('T-YYYYmmdd-HHMMSS-abcd')
    assert ts.make_task_id() != ts.make_task_id()


def test_parse_pose_direct():
    pose, errors = ts.parse_pose({'x': 1, 'y': 2, 'yaw': math.pi}, 'p')
    assert errors == [] and pose['yaw'] == pytest.approx(math.pi)
    pose, errors = ts.parse_pose(42, 'p')
    assert pose is None and errors


def _fleet_task_schema_path():
    """플릿 매니저의 task_schema.json: $AMR_FLEET_TASK_SCHEMA → 소스 트리 → 설치 share 순."""
    candidates = [os.environ.get('AMR_FLEET_TASK_SCHEMA', '')]
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, '..', '..', 'amr_fleet', 'config', 'task_schema.json'))
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('amr_fleet'), 'config', 'task_schema.json'))
    except Exception:  # noqa: B902 — ament 가 없거나 amr_fleet 미설치
        pass
    return next((p for p in candidates if p and os.path.isfile(p)), None)


def test_wire_format_accepted_by_fleet_manager_schema():
    """대시보드가 내는 JSON 이 fleet_manager_node 의 스키마(additionalProperties: false)를 통과한다."""
    jsonschema = pytest.importorskip('jsonschema')
    path = _fleet_task_schema_path()
    if path is None:
        pytest.skip('amr_fleet config/task_schema.json 없음 (통합 전 단독 빌드)')
    with open(path, encoding='utf-8') as f:
        validator = jsonschema.Draft202012Validator(json.load(f))
    samples = [
        full_payload(),
        full_payload(robot_id='amr_02', task_id='order_1001'),
        full_payload(deadline_sec=None, deadline='2027-01-01T09:00:00'),
        {'pickup': {'x': 1, 'y': 1}, 'dropoff': {'x': 2, 'y': 2}, 'item_type': 'small'},
    ]
    for payload in samples:
        task, errors = ts.validate_task(payload, now=NOW, world=WORLD)
        assert errors == []
        wire = json.loads(json.dumps(task))
        problems = [e.message for e in validator.iter_errors(wire)]
        assert problems == [], (wire, problems)


def test_validate_estop():
    known = ['amr_01', 'amr_02']
    assert ts.validate_estop({'robot_id': 'all', 'active': True}, known) == ('all', True, [])
    assert ts.validate_estop({'active': False}, known) == ('all', False, [])
    assert ts.validate_estop({'robot_id': '', 'active': True}, known)[0] == 'all'
    assert ts.validate_estop({'robot_id': 'amr_02', 'active': True}, known) == ('amr_02', True, [])
    _, _, errors = ts.validate_estop({'robot_id': 'amr_09', 'active': True}, known)
    assert errors and 'amr_09' in errors[0]
    _, _, errors = ts.validate_estop({'robot_id': 'amr_01', 'active': 'yes'}, known)
    assert errors and 'active' in errors[0]
    _, _, errors = ts.validate_estop({'robot_id': 5, 'active': True}, known)
    assert errors and 'robot_id' in errors[0]
    _, _, errors = ts.validate_estop('nope', known)
    assert errors == ['JSON 객체가 필요하다']
    _, _, errors = ts.validate_estop({'robot_id': 'amr_01', 'active': True}, [])
    assert errors and '허용: all, -' in errors[0]
