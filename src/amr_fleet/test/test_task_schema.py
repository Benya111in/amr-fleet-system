"""task_schema: JSON Task Description 검증/변환 (명세 4.8)."""

import datetime as dt
import json
import pathlib

import pytest

from amr_fleet.task_schema import (
    DEFAULT_ITEM_MASSES, STATUS_PENDING, Pose2D, TaskSchema, TaskSpec, TaskValidationError,
    default_schema_path, generate_task_id, load_item_masses, parse_deadline, task_to_dict,
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


def test_default_schema_path_points_to_config():
    assert default_schema_path() == SCHEMA_PATH


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


def test_deadline_iso_with_timezone():
    assert parse_deadline('2026-09-22T10:30:00+09:00', NOW) == pytest.approx(
        dt.datetime(2026, 9, 22, 1, 30, tzinfo=dt.timezone.utc).timestamp())
    assert parse_deadline('2026-09-22T01:30:00Z', NOW) == pytest.approx(
        dt.datetime(2026, 9, 22, 1, 30, tzinfo=dt.timezone.utc).timestamp())


def test_deadline_iso_naive_is_local_time():
    naive = dt.datetime(2026, 9, 22, 10, 30)
    assert parse_deadline(naive.isoformat(), NOW) == pytest.approx(naive.timestamp())


def test_deadline_relative_and_none():
    assert parse_deadline(0, NOW) == NOW
    assert parse_deadline(12.5, NOW) == NOW + 12.5
    assert parse_deadline(None, NOW) is None


@pytest.mark.parametrize('bad', ['yesterday', '2026-13-45T00:00:00', -1, True])
def test_deadline_invalid(bad):
    with pytest.raises(TaskValidationError):
        parse_deadline(bad, NOW)


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
    specs = [schema.from_json(f.read_text(encoding='utf-8'), NOW) for f in files]
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
