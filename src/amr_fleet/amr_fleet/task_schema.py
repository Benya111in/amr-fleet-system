"""
JSON Task Description 파싱·검증 (명세 4.8 작업 관리).

config/task_schema.json 을 jsonschema 로 검증한 뒤 TaskSpec 으로 바꾼다.
ROS 에 의존하지 않는 순수 모듈 — amr_msgs/Task 변환은 task_msg.py 가 맡는다.

검증은 두 겹이다. 어느 쪽을 어겨도 TaskValidationError 하나로 알린다 (노드는 알림만 내고 계속 돈다).
1. 스키마(draft-07): 필드·타입·범위·패턴. 검증기는 스키마의 $schema 로 고르고 없으면 Draft7Validator
   — apt python3-jsonschema 3.2.0(rosdep, CI) 과 pip 4.x(이미지) 에서 똑같이 동작한다.
2. 의미 규칙(check_spec): 유한한 좌표·질량, 마감 지평선(±30일). /fleet/assign_task 서비스로 들어온
   작업도 validate_spec 으로 같은 규칙을 거친다.
JSON 구문은 엄격하게 읽는다: NaN/Infinity, 1e400 같은 비유한 수, 깊은 중첩, 과대 페이로드는 거절.

deadline 은 두 형식을 받고, 접수 시 노드 시계(now, use_sim_time 이면 /clock) 기준 초로 바꾼다.
- 숫자: 접수 시각(now)으로부터의 초. deadline = now + 값.
- ISO-8601 문자열: 벽시계 절대 시각(시간대가 없으면 로컬). 남은 벽시계 초를 노드 시계에 더한다:
  deadline = now + (iso_epoch - wall_now). 시뮬레이션 RTF 가 1 이 아니면 접수 이후의 흐름은 노드 시계를
  따른다 (마감 판정·정렬이 모두 한 시계에서 이루어지게 하려는 선택).
마감이 없으면 None 이다.
"""

from __future__ import annotations

import dataclasses
import datetime as _dt
import json
import math
import os
import pathlib
import re
from typing import Any, Callable, Dict, Optional
import uuid

import jsonschema
import yaml

# amr_msgs/msg/Task 의 STATUS_* 와 같은 값 (ROS 없이도 쓰도록 여기 복제)
STATUS_PENDING = 0
STATUS_IN_PROGRESS = 1
STATUS_COMPLETED = 2
STATUS_FAILED = 3
STATUS_NAMES = {
    STATUS_PENDING: 'PENDING',
    STATUS_IN_PROGRESS: 'IN_PROGRESS',
    STATUS_COMPLETED: 'COMPLETED',
    STATUS_FAILED: 'FAILED',
}

ITEM_TYPES = ('small', 'medium', 'large')
# config/robot_params.yaml payload 표를 읽지 못할 때의 기본값 (명세 4.8 물품 표)
DEFAULT_ITEM_MASSES: Dict[str, float] = {'small': 2.0, 'medium': 10.0, 'large': 25.0}
DEFAULT_FRAME_ID = 'map'

MAX_DEADLINE_HORIZON_S = 30 * 86400.0   # 마감 지평선: 접수 시각 ±30일 (스키마 deadline 상한과 같다)
MAX_COORD_M = 1.0e4                     # |x|, |y| 상한 [m] (스키마와 같다)
MAX_PAYLOAD_BYTES = 16 * 1024           # JSON 작업 1건 상한 (정상 문서는 1 KB 미만)
TASK_ID_RE = re.compile(r'^[A-Za-z0-9_.:-]{1,64}$')
ROBOT_ID_RE = re.compile(r'^[A-Za-z0-9_]{0,32}$')
_UNSAFE_TEXT_RE = re.compile(r'[<>&`\x00-\x1f\x7f]')


class TaskValidationError(ValueError):
    """JSON 작업 정의가 스키마 또는 의미 규칙을 어겼을 때."""


@dataclasses.dataclass
class Pose2D:
    """평면 자세 [m, m, rad] + 기준 프레임."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    frame_id: str = DEFAULT_FRAME_ID


@dataclasses.dataclass
class TaskSpec:
    """검증이 끝난 작업 1건. amr_msgs/msg/Task 와 필드가 대응한다."""

    task_id: str
    pickup: Pose2D
    dropoff: Pose2D
    item_type: str
    item_mass: float
    priority: int = 0
    deadline: Optional[float] = None       # 노드 시계 [s], None = 마감 없음
    robot_id: str = ''                     # 비어 있으면 자동 할당
    created: float = 0.0                   # 접수 시각, 노드 시계 [s]
    status: int = STATUS_PENDING

    def has_deadline(self) -> bool:
        """마감이 지정되어 있는지."""
        return self.deadline is not None

    def time_to_deadline(self, now: float) -> float:
        """마감까지 남은 초. 마감이 없으면 +inf."""
        if self.deadline is None:
            return float('inf')
        return self.deadline - now


def safe_text(text: object, limit: int = 200) -> str:
    """
    외부 입력이 섞인 문자열을 알림·응답에 싣기 전에 다듬는다.

    마크업·제어 문자(< > & ` 와 C0 제어)는 '?' 로 바꾸고 limit 자로 자른다. 대시보드는 따로
    이스케이프해야 하지만(r-dashboard), fleet 쪽에서도 원문을 그대로 흘리지 않는다.
    """
    s = _UNSAFE_TEXT_RE.sub('?', str(text))
    return s if len(s) <= limit else s[:limit - 3] + '...'


def default_schema_path() -> pathlib.Path:
    """config/task_schema.json 위치. 소스 트리(symlink-install 포함) → ament share 순으로 찾는다."""
    here = pathlib.Path(__file__).resolve()
    candidate = here.parents[1] / 'config' / 'task_schema.json'
    if candidate.is_file():
        return candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        share = pathlib.Path(get_package_share_directory('amr_fleet'))
        return share / 'config' / 'task_schema.json'
    except Exception:  # noqa: B902 — ament 가 없는 순수 파이썬 환경
        return candidate


def default_robot_params_path() -> pathlib.Path:
    """config/robot_params.yaml 위치 ($ROS_WS/config, 없으면 소스 트리 기준)."""
    ws = os.environ.get('ROS_WS')
    if ws:
        return pathlib.Path(ws) / 'config' / 'robot_params.yaml'
    return pathlib.Path(__file__).resolve().parents[3] / 'config' / 'robot_params.yaml'


def load_item_masses(path: Optional[os.PathLike] = None) -> Dict[str, float]:
    """
    robot_params.yaml 의 payload.<type>.mass 표를 읽는다.

    파일이 없거나 형식이 다르면 명세 표의 기본값(2/10/25 kg)을 돌려준다.
    """
    path = pathlib.Path(path) if path is not None else default_robot_params_path()
    masses = dict(DEFAULT_ITEM_MASSES)
    try:
        with open(path, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
        payload = doc['/**']['ros__parameters']['payload']
        for item_type in ITEM_TYPES:
            masses[item_type] = float(payload[item_type]['mass'])
    except (OSError, KeyError, TypeError, ValueError):
        return dict(DEFAULT_ITEM_MASSES)
    return masses


def generate_task_id() -> str:
    """task_<8 hex> 형식의 식별자."""
    return 'task_' + uuid.uuid4().hex[:8]


def _reject_constant(name: str) -> None:
    raise TaskValidationError(f'JSON 에 {name} 은 쓸 수 없다 (유한한 수만)')


def _finite_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):
        raise TaskValidationError(f'JSON 수 {safe_text(text, 40)} 가 유한하지 않다')
    return value


def loads_strict(text: Any) -> Any:
    """
    JSON 문자열을 엄격하게 읽는다. 어떤 입력이든 실패는 TaskValidationError 로 통일한다.

    NaN/Infinity/-Infinity 와 넘치는 실수(1e400 → inf), MAX_PAYLOAD_BYTES 초과, 깊은 중첩
    (RecursionError), 과대 정수(ValueError) 를 거절한다.
    """
    if isinstance(text, (bytes, bytearray)):
        try:
            text = bytes(text).decode('utf-8')
        except UnicodeDecodeError as exc:
            raise TaskValidationError(f'JSON UTF-8 오류: {exc.reason}') from exc
    if not isinstance(text, str):
        raise TaskValidationError('JSON 문자열이 아니다')
    if len(text.encode('utf-8', 'surrogatepass')) > MAX_PAYLOAD_BYTES:
        raise TaskValidationError(f'JSON 이 {MAX_PAYLOAD_BYTES} 바이트를 넘는다')
    try:
        return json.loads(text, parse_constant=_reject_constant, parse_float=_finite_float)
    except TaskValidationError:
        raise
    except RecursionError as exc:
        raise TaskValidationError('JSON 중첩이 너무 깊다') from exc
    except (ValueError, TypeError) as exc:   # JSONDecodeError ⊂ ValueError, 정수 자릿수 한도 포함
        raise TaskValidationError(f'JSON 구문 오류: {safe_text(exc, 120)}') from exc


def parse_deadline(value: Any, now: float, wall_now: Optional[float] = None,
                   horizon_s: float = MAX_DEADLINE_HORIZON_S) -> Optional[float]:
    """
    마감(deadline) 값을 노드 시계 [s] 로 바꾼다.

    숫자 → now + 초 (0 ~ horizon_s). 문자열 → ISO-8601 벽시계 절대 시각('Z' 접미 허용) 을
    now + (iso_epoch - wall_now) 로 옮긴다 (|iso_epoch - wall_now| <= horizon_s). None → None.
    wall_now 를 주지 않으면 노드 시계가 곧 벽시계라고 본다 (wall_now = now).
    """
    if value is None:
        return None
    if isinstance(value, bool):
        raise TaskValidationError('deadline 은 숫자 또는 ISO-8601 문자열이어야 한다')
    if isinstance(value, (int, float)):
        try:
            seconds = float(value)
        except OverflowError as exc:
            raise TaskValidationError('deadline(초)이 너무 크다') from exc
        if not math.isfinite(seconds) or seconds < 0.0:
            raise TaskValidationError('deadline(초)은 0 이상의 유한한 수여야 한다')
        if seconds > horizon_s:
            raise TaskValidationError(f'deadline(초)이 지평선 {horizon_s:.0f} s 를 넘는다')
        return float(now) + seconds
    text = str(value).strip()
    if len(text) > 64:
        raise TaskValidationError('deadline 문자열이 너무 길다')
    if text.endswith('Z') or text.endswith('z'):
        text = text[:-1] + '+00:00'
    try:
        # 시간대가 없으면 로컬 시간으로 해석한다 (timestamp() 의 기본 동작)
        epoch = _dt.datetime.fromisoformat(text).timestamp()
    except (ValueError, OverflowError, OSError) as exc:
        raise TaskValidationError(f'deadline ISO-8601 형식 오류: {safe_text(value, 64)}') from exc
    wall = float(now) if wall_now is None else float(wall_now)
    remaining = epoch - wall
    if abs(remaining) > horizon_s:
        raise TaskValidationError(
            f'deadline {safe_text(value, 64)} 이 접수 시각 ±{horizon_s / 86400.0:.0f}일 밖이다')
    return float(now) + remaining


def spec_to_doc(spec: TaskSpec) -> Dict[str, Any]:
    """
    `TaskSpec` → 스키마 검증용 JSON 문서 (deadline 제외, 빈 robot_id 제외).

    서비스 경로(/fleet/assign_task)가 JSON 경로와 같은 스키마 규칙을 거치게 할 때 쓴다.
    """
    doc: Dict[str, Any] = {
        'task_id': spec.task_id,
        'priority': spec.priority,
        'pickup': pose_to_dict(spec.pickup),
        'dropoff': pose_to_dict(spec.dropoff),
        'item_type': spec.item_type,
    }
    if spec.robot_id:
        doc['robot_id'] = spec.robot_id
    return doc


def _make_validator(schema: Dict[str, Any]):
    """검증기를 고른다: $schema 에 맞춘 것, 선언이 없으면 draft-07 (jsonschema 3.2 ~ 4.x 공통)."""
    cls = jsonschema.validators.validator_for(schema, default=jsonschema.Draft7Validator)
    cls.check_schema(schema)
    return cls(schema)


class TaskSchema:
    """task_schema.json 기반 검증기 + TaskSpec 변환기."""

    def __init__(self, schema_path: Optional[os.PathLike] = None,
                 item_masses: Optional[Dict[str, float]] = None,
                 id_factory: Callable[[], str] = generate_task_id,
                 horizon_s: float = MAX_DEADLINE_HORIZON_S):
        path = pathlib.Path(schema_path) if schema_path is not None else default_schema_path()
        with open(path, 'r', encoding='utf-8') as f:
            self.schema = json.load(f)
        self.schema_path = path
        self._validator = _make_validator(self.schema)
        self.item_masses = dict(item_masses) if item_masses else dict(DEFAULT_ITEM_MASSES)
        self._id_factory = id_factory
        self.horizon_s = float(horizon_s)

    # --- 검증 ---
    def validate(self, doc: Any) -> None:
        """스키마 위반이면 TaskValidationError(첫 오류 메시지, 다듬은 문자열)."""
        try:
            errors = sorted(self._validator.iter_errors(doc), key=lambda e: list(map(str, e.path)))
        except RecursionError as exc:
            raise TaskValidationError('작업 정의 중첩이 너무 깊다') from exc
        if errors:
            err = errors[0]
            where = '/'.join(str(p) for p in err.path) or '(root)'
            raise TaskValidationError(safe_text(f'{where}: {err.message}'))

    def check_spec(self, spec: TaskSpec, now: float) -> None:
        """
        의미 규칙: 식별자 형식, 유한한 자세·질량, 좌표 범위, 마감 지평선.

        스키마가 막지 못하는 NaN/inf(서비스 경로의 float 필드)도 여기서 걸러진다.
        """
        if not TASK_ID_RE.match(spec.task_id or ''):
            raise TaskValidationError(f'task_id 형식 오류: {safe_text(spec.task_id, 64)}')
        if not ROBOT_ID_RE.match(spec.robot_id or ''):
            raise TaskValidationError(f'robot_id 형식 오류: {safe_text(spec.robot_id, 32)}')
        if spec.item_type not in ITEM_TYPES:
            raise TaskValidationError(f'item_type 은 {ITEM_TYPES} 중 하나')
        for name, pose in (('pickup', spec.pickup), ('dropoff', spec.dropoff)):
            values = (pose.x, pose.y, pose.yaw)
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                raise TaskValidationError(f'{name}: 좌표가 유한한 수가 아니다')
            if abs(pose.x) > MAX_COORD_M or abs(pose.y) > MAX_COORD_M:
                raise TaskValidationError(f'{name}: 좌표가 ±{MAX_COORD_M:g} m 밖이다')
        if not (math.isfinite(spec.item_mass) and spec.item_mass > 0.0):
            raise TaskValidationError('item_mass 는 양의 유한한 수여야 한다')
        if spec.deadline is not None:
            if not math.isfinite(spec.deadline):
                raise TaskValidationError('deadline 이 유한하지 않다')
            if abs(spec.deadline - now) > self.horizon_s:
                raise TaskValidationError(
                    f'deadline 이 접수 시각 ±{self.horizon_s / 86400.0:.0f}일 밖이다')

    def validate_spec(self, spec: TaskSpec, now: float) -> None:
        """이미 만들어진 `TaskSpec`(서비스 경로)을 JSON 경로와 같은 스키마 + 의미 규칙으로 검사."""
        self.check_spec(spec, now)
        self.validate(spec_to_doc(spec))

    # --- 변환 ---
    def from_dict(self, doc: Any, now: float, wall_now: Optional[float] = None) -> TaskSpec:
        """
        검증 후 TaskSpec 으로. task_id 가 없으면 생성, item_mass 는 payload 표에서 채운다.

        now 는 노드 시계, wall_now 는 ISO 마감을 옮길 벽시계 (parse_deadline 참고).
        """
        if not isinstance(doc, dict):
            raise TaskValidationError('작업 정의는 JSON 객체여야 한다')
        self.validate(doc)
        item_type = doc['item_type']
        spec = TaskSpec(
            task_id=doc.get('task_id') or self._id_factory(),
            pickup=_pose_from_dict(doc['pickup']),
            dropoff=_pose_from_dict(doc['dropoff']),
            item_type=item_type,
            item_mass=float(self.item_masses.get(item_type, DEFAULT_ITEM_MASSES[item_type])),
            priority=int(doc.get('priority', 0)),
            deadline=parse_deadline(doc.get('deadline'), now, wall_now, self.horizon_s),
            robot_id=str(doc.get('robot_id') or ''),
            created=float(now),
            status=STATUS_PENDING,
        )
        self.check_spec(spec, now)
        return spec

    def from_json(self, text: Any, now: float, wall_now: Optional[float] = None) -> TaskSpec:
        """JSON 문자열 → TaskSpec. 구문 오류도 TaskValidationError 로 통일한다."""
        return self.from_dict(loads_strict(text), now, wall_now)


def _pose_from_dict(d: Dict[str, Any]) -> Pose2D:
    return Pose2D(x=float(d['x']), y=float(d['y']), yaw=float(d.get('yaw', 0.0)),
                  frame_id=str(d.get('frame_id') or DEFAULT_FRAME_ID))


def pose_to_dict(pose: Pose2D) -> Dict[str, Any]:
    """Pose2D → JSON 친화 dict."""
    return {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw, 'frame_id': pose.frame_id}


def task_to_dict(spec: TaskSpec) -> Dict[str, Any]:
    """
    `TaskSpec` → JSON 친화 dict (대시보드/로그용).

    deadline 은 노드 시계 초(float) 그대로 두고, status 는 이름 문자열로 적는다.
    """
    return {
        'task_id': spec.task_id,
        'robot_id': spec.robot_id,
        'priority': spec.priority,
        'deadline': spec.deadline,
        'pickup': pose_to_dict(spec.pickup),
        'dropoff': pose_to_dict(spec.dropoff),
        'item_type': spec.item_type,
        'item_mass': spec.item_mass,
        'created': spec.created,
        'status': STATUS_NAMES.get(spec.status, str(spec.status)),
    }
