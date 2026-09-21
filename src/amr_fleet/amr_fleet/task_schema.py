"""
JSON Task Description 파싱·검증 (명세 4.8 작업 관리).

config/task_schema.json 을 jsonschema 로 검증한 뒤 TaskSpec 으로 바꾼다.
ROS 에 의존하지 않는 순수 모듈 — amr_msgs/Task 변환은 task_msg.py 가 맡는다.

deadline 은 두 형식을 받는다.
- ISO-8601 문자열: 절대 시각. 시간대가 없으면 로컬 시간으로 본다.
- 숫자: 파싱 시각(now)으로부터의 초.
내부적으로는 epoch 초(float) 로 통일하고, 마감이 없으면 None 이다.
"""

from __future__ import annotations

import dataclasses
import datetime as _dt
import json
import os
import pathlib
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
    deadline: Optional[float] = None       # epoch [s], None = 마감 없음
    robot_id: str = ''                     # 비어 있으면 자동 할당
    created: float = 0.0                   # epoch [s]
    status: int = STATUS_PENDING

    def has_deadline(self) -> bool:
        """마감이 지정되어 있는지."""
        return self.deadline is not None

    def time_to_deadline(self, now: float) -> float:
        """마감까지 남은 초. 마감이 없으면 +inf."""
        if self.deadline is None:
            return float('inf')
        return self.deadline - now


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


def parse_deadline(value: Any, now: float) -> Optional[float]:
    """
    마감(deadline) 값을 epoch 초로 바꾼다.

    숫자 → now + 초, 문자열 → ISO-8601 절대 시각('Z' 접미 허용). None → None.
    """
    if value is None:
        return None
    if isinstance(value, bool):
        raise TaskValidationError('deadline 은 숫자 또는 ISO-8601 문자열이어야 한다')
    if isinstance(value, (int, float)):
        if value < 0:
            raise TaskValidationError('deadline(초)은 0 이상이어야 한다')
        return float(now) + float(value)
    text = str(value).strip()
    if text.endswith('Z') or text.endswith('z'):
        text = text[:-1] + '+00:00'
    try:
        dt = _dt.datetime.fromisoformat(text)
    except ValueError as exc:
        raise TaskValidationError(f'deadline ISO-8601 형식 오류: {value!r}') from exc
    # 시간대가 없으면 로컬 시간으로 해석한다 (timestamp() 의 기본 동작)
    return dt.timestamp()


class TaskSchema:
    """task_schema.json 기반 검증기 + TaskSpec 변환기."""

    def __init__(self, schema_path: Optional[os.PathLike] = None,
                 item_masses: Optional[Dict[str, float]] = None,
                 id_factory: Callable[[], str] = generate_task_id):
        path = pathlib.Path(schema_path) if schema_path is not None else default_schema_path()
        with open(path, 'r', encoding='utf-8') as f:
            self.schema = json.load(f)
        self.schema_path = path
        self._validator = jsonschema.Draft202012Validator(self.schema)
        self.item_masses = dict(item_masses) if item_masses else dict(DEFAULT_ITEM_MASSES)
        self._id_factory = id_factory

    # --- 검증 ---
    def validate(self, doc: Dict[str, Any]) -> None:
        """스키마 위반이면 TaskValidationError(첫 오류 메시지)."""
        errors = sorted(self._validator.iter_errors(doc), key=lambda e: list(e.path))
        if errors:
            err = errors[0]
            where = '/'.join(str(p) for p in err.path) or '(root)'
            raise TaskValidationError(f'{where}: {err.message}')

    # --- 변환 ---
    def from_dict(self, doc: Dict[str, Any], now: float) -> TaskSpec:
        """검증 후 TaskSpec 으로. task_id 가 없으면 생성, item_mass 는 payload 표에서 채운다."""
        if not isinstance(doc, dict):
            raise TaskValidationError('작업 정의는 JSON 객체여야 한다')
        self.validate(doc)
        item_type = doc['item_type']
        return TaskSpec(
            task_id=doc.get('task_id') or self._id_factory(),
            pickup=_pose_from_dict(doc['pickup']),
            dropoff=_pose_from_dict(doc['dropoff']),
            item_type=item_type,
            item_mass=float(self.item_masses.get(item_type, DEFAULT_ITEM_MASSES[item_type])),
            priority=int(doc.get('priority', 0)),
            deadline=parse_deadline(doc.get('deadline'), now),
            robot_id=str(doc.get('robot_id') or ''),
            created=float(now),
            status=STATUS_PENDING,
        )

    def from_json(self, text: str, now: float) -> TaskSpec:
        """JSON 문자열 → TaskSpec. 구문 오류도 TaskValidationError 로 통일한다."""
        try:
            doc = json.loads(text)
        except (json.JSONDecodeError, TypeError) as exc:
            raise TaskValidationError(f'JSON 구문 오류: {exc}') from exc
        return self.from_dict(doc, now)


def _pose_from_dict(d: Dict[str, Any]) -> Pose2D:
    return Pose2D(x=float(d['x']), y=float(d['y']), yaw=float(d.get('yaw', 0.0)),
                  frame_id=str(d.get('frame_id') or DEFAULT_FRAME_ID))


def pose_to_dict(pose: Pose2D) -> Dict[str, Any]:
    """Pose2D → JSON 친화 dict."""
    return {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw, 'frame_id': pose.frame_id}


def task_to_dict(spec: TaskSpec) -> Dict[str, Any]:
    """
    `TaskSpec` → JSON 친화 dict (대시보드/로그용).

    deadline 은 epoch 초(float) 그대로 두고, status 는 이름 문자열로 적는다.
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
