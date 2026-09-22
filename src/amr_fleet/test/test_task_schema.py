"""task_schema: JSON Task Description 검증/변환 (명세 4.8)."""

import dataclasses
import datetime as dt
import json
import pathlib

import jsonschema
import pytest

from amr_fleet.task_schema import (
    _make_validator, DEFAULT_ITEM_MASSES, MAX_DEADLINE_HORIZON_S, MAX_PAYLOAD_BYTES,
    STATUS_PENDING, Pose2D, TaskSchema, TaskSpec, TaskValidationError, default_schema_path,
    generate_task_id, load_item_masses, loads_strict, parse_deadline, safe_text, spec_to_doc,
    task_to_dict,
)

PKG_DIR = pathlib.Path(__file__).resolve().parents[1]
SCHEMA_PATH = PKG_DIR / 'config' / 'task_schema.json'
EXAMPLES_DIR = PKG_DIR / 'config' / 'examples'
WS_ROBOT_PARAMS = PKG_DIR.parents[1] / 'config' / 'robot_params.yaml'
NOW = 1_700_000_000.0


@pytest.fixture
def schema():
    """워크스페이스 config/robot_params.yaml 의 질량 표를 쓰는 검증기."""
    return TaskSchema(SCHEMA_PATH, item_masses=load_item_masses(WS_ROBOT_PARAMS),
                      id_factory=lambda: 'task_fixed')


def minimal_doc(**extra):
    doc = {'pickup': {'x': 1.0, 'y': 2.0}, 'dropoff': {'x': 3.0, 'y': 4.0}, 'item_type': 'small'}
    doc.update(extra)
    return doc


def test_default_schema_path_finds_the_package_schema():
    # symlink-install 이면 소스 config/, 일반 install 이면 share/amr_fleet/config/ — 위치가 아니라 내용을 본다
    p = default_schema_path()
    assert p.is_file() and p.name == 'task_schema.json'
    assert json.loads(p.read_text(encoding='utf-8')) == json.loads(
        SCHEMA_PATH.read_text(encoding='utf-8'))


def test_schema_is_draft07_and_validator_is_version_agnostic(schema):
    # rosdep 의 apt python3-jsonschema 3.2.0 에는 Draft202012Validator 가 없다 → draft-07 로 적는다.
    # CI(3.2.0)와 이미지(pip 4.x) 양쪽에서 이 파일 전체가 통과해야 한다.
    assert schema.schema['$schema'].startswith('http://json-schema.org/draft-07/')
    assert isinstance(schema._validator, jsonschema.Draft7Validator)
    no_decl = {k: v for k, v in schema.schema.items() if k != '$schema'}
    assert isinstance(_make_validator(no_decl), jsonschema.Draft7Validator)   # 선언 없으면 draft-07
    # $ref(#/definitions/pose2d) 가 두 버전에서 모두 풀린다
    with pytest.raises(TaskValidationError, match='pickup'):
        schema.validate(minimal_doc(pickup={'x': 1}))


def test_minimal_document_fills_defaults(schema):
    spec = schema.from_dict(minimal_doc(), NOW)
    assert spec.task_id == 'task_fixed'
    assert spec.priority == 0
    assert spec.deadline is None and not spec.has_deadline()
    assert spec.time_to_deadline(NOW) == float('inf')
    assert spec.pickup == Pose2D(1.0, 2.0, 0.0, 'map')
    assert spec.dropoff.frame_id == 'map'
    assert spec.item_mass == pytest.approx(2.0)
    assert spec.robot_id == ''
    assert spec.created == NOW
    assert spec.status == STATUS_PENDING


def test_full_document(schema):
    doc = minimal_doc(task_id='order_1', priority=255, deadline=90, robot_id='amr_02',
                      item_type='large')
    doc['pickup'].update(yaw=1.5, frame_id='odom')
    spec = schema.from_dict(doc, NOW)
    assert spec.task_id == 'order_1'
    assert spec.priority == 255
    assert spec.deadline == pytest.approx(NOW + 90)
    assert spec.time_to_deadline(NOW + 30) == pytest.approx(60)
    assert spec.pickup.yaw == 1.5 and spec.pickup.frame_id == 'odom'
    assert spec.item_mass == pytest.approx(25.0)
    assert spec.robot_id == 'amr_02'


def test_from_json_text(schema):
    spec = schema.from_json(json.dumps(minimal_doc(item_type='medium')), NOW)
    assert spec.item_mass == pytest.approx(10.0)


@pytest.mark.parametrize('bad, where', [
    ({'dropoff': {'x': 0, 'y': 0}, 'item_type': 'small'}, 'pickup'),
    (minimal_doc(pickup={'x': 1e5, 'y': 0}), 'pickup'),
    (minimal_doc(dropoff={'x': 0, 'y': -2e4}), 'dropoff'),
    (minimal_doc(robot_id='amr 5'), 'robot_id'),
    (minimal_doc(robot_id='<b>'), 'robot_id'),
    (minimal_doc(deadline=3e9), 'deadline'),
    (minimal_doc(deadline='x' * 65), 'deadline'),
    (minimal_doc(priority=256), 'priority'),
    (minimal_doc(priority=-1), 'priority'),
    (minimal_doc(priority=1.5), 'priority'),
    (minimal_doc(item_type='huge'), 'item_type'),
    (minimal_doc(unknown_field=1), 'unknown_field'),
    (minimal_doc(task_id=''), 'task_id'),
    (minimal_doc(task_id='bad id with spaces'), 'task_id'),
    (minimal_doc(deadline=-5), 'deadline'),
    (minimal_doc(deadline=True), 'deadline'),
    (minimal_doc(pickup={'x': 'one', 'y': 2}), 'pickup'),
    (minimal_doc(pickup={'x': 1}), 'pickup'),
    (minimal_doc(pickup={'x': 1, 'y': 2, 'z': 3}), 'z'),
])
def test_schema_violations(schema, bad, where):
    with pytest.raises(TaskValidationError) as excinfo:
        schema.from_dict(bad, NOW)
    assert where in str(excinfo.value)


def test_non_object_and_bad_json(schema):
    with pytest.raises(TaskValidationError):
        schema.from_dict(['not', 'an', 'object'], NOW)
    with pytest.raises(TaskValidationError, match='구문'):
        schema.from_json('{"pickup": ', NOW)
    with pytest.raises(TaskValidationError):
        schema.from_json('[1, 2]', NOW)


def _doc_text(x='1.0', deadline=None):
    extra = '' if deadline is None else f', "deadline": {deadline}'
    return ('{"pickup": {"x": %s, "y": 0}, "dropoff": {"x": 1, "y": 1}, "item_type": "small"%s}'
            % (x, extra))


@pytest.mark.parametrize('text, match', [
    (_doc_text('NaN'), 'NaN'),
    (_doc_text('Infinity'), 'Infinity'),
    (_doc_text('-Infinity'), 'Infinity'),
    (_doc_text('1e400'), '유한'),
    (_doc_text('1' + '0' * 400), 'pickup'),              # 큰 정수: 스키마 범위에서 걸린다
    (_doc_text('1' + '0' * 5000), '구문'),               # 정수 자릿수 한도 (ValueError)
    (_doc_text(deadline='1e999'), '유한'),
    (_doc_text(deadline='3000000000'), 'deadline'),
    (_doc_text(deadline='"2100-01-01T00:00:00Z"'), '30일'),
    (_doc_text(deadline='"1900-01-01T00:00:00Z"'), '30일'),
    ('[' * 8000 + ']' * 8000, '중첩'),
    ('{"pickup": ' + '[' * 900 + ']' * 900 + '}', ''),  # 깊은 중첩 값도 예외 대신 검증 오류
    ('"just a string"', '객체'),
    ('null', '객체'),
    ('', '구문'),
])
def test_malformed_or_hostile_json_is_rejected_not_raised(schema, text, match):
    with pytest.raises(TaskValidationError, match=match):
        schema.from_json(text, NOW)


def test_loads_strict_input_types_and_size():
    assert loads_strict(b'{"a": 1.5}') == {'a': 1.5}
    with pytest.raises(TaskValidationError, match='UTF-8'):
        loads_strict(b'\xff\xfe')
    with pytest.raises(TaskValidationError, match='문자열'):
        loads_strict(12)
    with pytest.raises(TaskValidationError, match='바이트'):
        loads_strict('{"a": "' + 'x' * MAX_PAYLOAD_BYTES + '"}')


def test_error_text_is_sanitised(schema):
    with pytest.raises(TaskValidationError) as excinfo:
        schema.from_dict(minimal_doc(task_id='<img src=x onerror=alert(1)>'), NOW)
    text = str(excinfo.value)
    assert 'task_id' in text and '<' not in text and '>' not in text
    assert safe_text('a<b>&`\x00c') == 'a?b????c'
    assert safe_text('x' * 500, limit=10) == 'xxxxxxx...'


def test_deadline_iso_with_timezone():
    # 노드 시계 = 벽시계(now 를 벽시계로 줌)이면 절대 시각 그대로
    target = dt.datetime(2026, 9, 22, 1, 30, tzinfo=dt.timezone.utc).timestamp()
    wall = target - 3600.0
    assert parse_deadline('2026-09-22T10:30:00+09:00', wall) == pytest.approx(target)
    assert parse_deadline('2026-09-22T01:30:00Z', wall) == pytest.approx(target)


def test_deadline_iso_naive_is_local_time():
    naive = dt.datetime(2026, 9, 22, 10, 30)
    wall = naive.timestamp() - 600.0
    assert parse_deadline(naive.isoformat(), wall) == pytest.approx(naive.timestamp())


def test_deadline_relative_and_none():
    assert parse_deadline(0, NOW) == NOW
    assert parse_deadline(12.5, NOW) == NOW + 12.5
    assert parse_deadline(None, NOW) is None
    assert parse_deadline(MAX_DEADLINE_HORIZON_S, NOW) == NOW + MAX_DEADLINE_HORIZON_S


@pytest.mark.parametrize('bad', ['yesterday', '2026-13-45T00:00:00', -1, True, float('nan'),
                                 float('inf'), MAX_DEADLINE_HORIZON_S + 1, 10 ** 400,
                                 '2100-01-01T00:00:00Z', '0001-01-01T00:00:00', 'x' * 70])
def test_deadline_invalid(bad):
    with pytest.raises(TaskValidationError):
        parse_deadline(bad, NOW)


def test_iso_deadline_is_moved_into_node_clock():
    # use_sim_time: 노드 시계 120 s, 벽시계 NOW. ISO 마감 = 벽시계 NOW + 90 s → 노드 시계 210 s
    wall_deadline = dt.datetime.fromtimestamp(NOW + 90.0, tz=dt.timezone.utc).isoformat()
    assert parse_deadline(wall_deadline, 120.0, wall_now=NOW) == pytest.approx(210.0)
    # 이미 지난 ISO 마감(지평선 안)은 받아들이고 곧바로 초과로 판정된다
    past = dt.datetime.fromtimestamp(NOW - 30.0, tz=dt.timezone.utc).isoformat()
    assert parse_deadline(past, 120.0, wall_now=NOW) == pytest.approx(90.0)
    # 상대 초는 벽시계와 무관하게 노드 시계 기준
    assert parse_deadline(15, 120.0, wall_now=NOW) == 135.0


def test_from_json_iso_deadline_under_sim_time(schema):
    iso = dt.datetime.fromtimestamp(NOW + 45.0, tz=dt.timezone.utc).isoformat()
    spec = schema.from_json(json.dumps(minimal_doc(deadline=iso)), 120.0, wall_now=NOW)
    assert spec.deadline == pytest.approx(165.0) and spec.created == 120.0


def test_validate_spec_applies_json_rules_to_service_requests(schema):
    good = schema.from_dict(minimal_doc(task_id='svc_1', robot_id='amr_01'), NOW)
    schema.validate_spec(good, NOW)
    assert spec_to_doc(good)['robot_id'] == 'amr_01'
    assert 'robot_id' not in spec_to_doc(dataclasses.replace(good, robot_id=''))
    bad_cases = [
        (dict(task_id='<img src=x onerror=alert(1)>'), 'task_id'),
        (dict(robot_id='amr 99'), 'robot_id'),
        (dict(item_type='huge'), 'item_type'),
        (dict(item_mass=float('nan')), 'item_mass'),
        (dict(item_mass=0.0), 'item_mass'),
        (dict(pickup=Pose2D(float('nan'), 0.0)), 'pickup'),
        (dict(dropoff=Pose2D(0.0, float('inf'))), 'dropoff'),
        (dict(pickup=Pose2D(0.0, 0.0, float('nan'))), 'pickup'),
        (dict(dropoff=Pose2D(2e4, 0.0)), 'dropoff'),
        (dict(pickup=Pose2D(0.0, 0.0, 0.0, '')), 'frame_id'),
        (dict(priority=300), 'priority'),
        (dict(deadline=float('inf')), 'deadline'),
        (dict(deadline=NOW + MAX_DEADLINE_HORIZON_S + 10.0), 'deadline'),
    ]
    for change, where in bad_cases:
        with pytest.raises(TaskValidationError, match=where):
            schema.validate_spec(dataclasses.replace(good, **change), NOW)


def test_item_masses_from_workspace_yaml():
    masses = load_item_masses(WS_ROBOT_PARAMS)
    assert masses == {'small': 2.0, 'medium': 10.0, 'large': 25.0}


def test_item_masses_custom_yaml(tmp_path):
    p = tmp_path / 'rp.yaml'
    p.write_text('/**:\n  ros__parameters:\n    payload:\n      small: {mass: 1.0}\n'
                 '      medium: {mass: 5.0}\n      large: {mass: 9.0}\n', encoding='utf-8')
    assert load_item_masses(p) == {'small': 1.0, 'medium': 5.0, 'large': 9.0}


def test_item_masses_fallback(tmp_path):
    assert load_item_masses(tmp_path / 'missing.yaml') == DEFAULT_ITEM_MASSES
    broken = tmp_path / 'broken.yaml'
    broken.write_text('/**:\n  ros__parameters:\n    payload: 3\n', encoding='utf-8')
    assert load_item_masses(broken) == DEFAULT_ITEM_MASSES


def test_generated_ids_are_unique_and_valid(schema):
    ids = {generate_task_id() for _ in range(50)}
    assert len(ids) == 50
    for task_id in ids:
        schema.validate(minimal_doc(task_id=task_id))


def test_example_files_validate(schema):
    files = sorted(EXAMPLES_DIR.glob('*.json'))
    assert len(files) >= 3
    # 예시의 ISO 마감(2026-09-22T18:30+09:00)이 지평선 안에 들도록 그날 아침을 벽시계로 쓴다
    wall = dt.datetime(2026, 9, 22, 9, 0, tzinfo=dt.timezone(dt.timedelta(hours=9))).timestamp()
    specs = [schema.from_json(f.read_text(encoding='utf-8'), NOW, wall_now=wall) for f in files]
    assert {s.item_type for s in specs} == {'small', 'medium', 'large'}
    assert any(s.robot_id for s in specs)
    assert any(s.deadline is not None for s in specs)


def test_task_to_dict_round_trips_through_schema(schema):
    spec = schema.from_dict(minimal_doc(task_id='t1', priority=7, deadline=10), NOW)
    d = task_to_dict(spec)
    assert d['status'] == 'PENDING' and d['deadline'] == pytest.approx(NOW + 10)
    doc = {k: d[k] for k in ('task_id', 'priority', 'pickup', 'dropoff', 'item_type')}
    again = schema.from_dict(doc, NOW)
    assert isinstance(again, TaskSpec) and again.pickup == spec.pickup
