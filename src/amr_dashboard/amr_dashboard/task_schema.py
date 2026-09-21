"""
JSON Task Description 검증·정규화 (순수 파이썬). /fleet/task_request 와이어 형식.

fleet_manager_node 가 받는 형식(amr_fleet config/task_schema.json)과 같다. 그 스키마는
additionalProperties: false 이므로 아래 키만 싣는다:

    {
      "task_id": "T-20260922-101500-3f2a",   # 생략 시 자동 생성
      "priority": 128,                       # 0(낮음) ~ 255(높음), 생략 시 0
      "deadline": 600.0,                     # 지금부터 초(숫자) 또는 ISO 8601 절대 시각(문자열)
                                             #   생략 시 키 없음 = 마감 없음
      "pickup":  {"x": 5.0,  "y": 3.0,  "yaw": 0.0,  "frame_id": "map"},
      "dropoff": {"x": 50.0, "y": 30.0, "yaw": 1.57, "frame_id": "map"},
      "item_type": "medium",                 # small | medium | large (질량은 플릿 매니저가 물품 표로)
      "robot_id": "amr_03"                   # 고정 할당할 때만 (빈 문자열이면 키 없음)
    }

입력 별칭: pickup_pose/dropoff_pose (amr_msgs/Task 필드 이름), deadline_sec (= 지금부터 초).
마감을 상대 초로 보내는 이유: 플릿 매니저는 노드 시계(시뮬레이션 시간)로 마감을 계산하므로
벽시계 절대 시각보다 시계 차이에 강하다.
검증은 최소한으로: 타입·범위·유한값·(선택) 지도 범위·모르는 키. 스케줄링 검증은 플릿 매니저 몫.
"""

import datetime
import math
import re
import time
import uuid

ITEM_TYPES = ('small', 'medium', 'large')

TASK_ID_RE = re.compile(r'^[A-Za-z0-9_.:-]{1,64}$')
ROBOT_ID_RE = re.compile(r'^[a-z][a-z0-9_]{0,31}$')

DEFAULT_PRIORITY = 0          # 플릿 매니저 스키마의 생략 시 기본값과 같게
MAX_DEADLINE_SEC = 7 * 24 * 3600.0
DEFAULT_FRAME_ID = 'map'

# 출력 키 ← 입력 키 (앞의 것이 정식 이름)
_POSE_KEYS = {
    'pickup': ('pickup', 'pickup_pose'),
    'dropoff': ('dropoff', 'dropoff_pose'),
}
_ALLOWED_KEYS = frozenset({
    'task_id', 'priority', 'deadline', 'deadline_sec', 'item_type', 'robot_id',
    'pickup', 'pickup_pose', 'dropoff', 'dropoff_pose',
})
_POSE_FIELDS = frozenset({'x', 'y', 'yaw', 'frame_id'})


def make_task_id(now=None) -> str:
    """자동 task_id: T-YYYYmmdd-HHMMSS-<4 hex>."""
    ts = datetime.datetime.fromtimestamp(time.time() if now is None else now)
    return f'T-{ts:%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:4]}'


def _finite_number(value):
    """값이 bool 이 아닌 유한 실수면 float, 아니면 None."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    f = float(value)
    return f if math.isfinite(f) else None


def _pick(payload: dict, names):
    """별칭 중 하나로 들어온 값 → (값 또는 None, 오류 또는 None). 둘 이상이면 오류."""
    present = [n for n in names if payload.get(n) is not None]
    if len(present) > 1:
        return None, f'{" / ".join(present)} 중 하나만 지정해야 한다'
    return (payload[present[0]] if present else None), None


def parse_deadline(payload: dict):
    """
    마감 입력 → (와이어 값, 오류 또는 None). 와이어 값이 None 이면 마감 없음(키 생략).

    숫자(deadline_sec 또는 deadline) = 지금부터 초 → float, 문자열 deadline = ISO 8601 절대 시각 →
    시간대를 붙여 정규화한 문자열 (시간대가 없으면 이 컴퓨터의 로컬 시간으로 본다).
    """
    raw, err = _pick(payload, ('deadline_sec', 'deadline'))
    if err:
        return None, err
    if raw is None or raw == '':
        return None, None
    if isinstance(raw, str):
        if payload.get('deadline_sec') is not None:
            return None, 'deadline_sec 는 숫자(지금부터 초)여야 한다'
        try:
            dt = datetime.datetime.fromisoformat(raw.strip().replace('Z', '+00:00'))
        except ValueError:
            return None, 'deadline 문자열은 ISO 8601 시각이어야 한다'
        if dt.tzinfo is None:
            dt = dt.astimezone()
        return dt.isoformat(), None
    sec = _finite_number(raw)
    if sec is None or sec < 0 or sec > MAX_DEADLINE_SEC:
        return None, f'deadline(지금부터 초)은 0~{int(MAX_DEADLINE_SEC)} 사이의 숫자여야 한다'
    return sec, None


def parse_pose(value, name: str, world=None):
    """{x, y, yaw?, frame_id?} → (정규화 dict, 오류 목록). world 가 있으면 x/y 범위도 검사."""
    if not isinstance(value, dict):
        return None, [f'{name} 은 {{x, y, yaw}} 객체여야 한다']
    errors = [f'{name}.{k} 는 모르는 필드다 (x, y, yaw, frame_id)'
              for k in sorted(set(value) - _POSE_FIELDS)]
    pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'frame_id': DEFAULT_FRAME_ID}
    for axis in ('x', 'y'):
        num = _finite_number(value.get(axis))
        if num is None:
            errors.append(f'{name}.{axis} 는 유한한 숫자여야 한다')
        else:
            pose[axis] = num
    num = _finite_number(value.get('yaw', 0.0))
    if num is None:
        errors.append(f'{name}.yaw 는 유한한 숫자(rad)여야 한다')
    else:
        pose['yaw'] = math.atan2(math.sin(num), math.cos(num))  # [-pi, pi] 로 정규화
    frame_id = value.get('frame_id', DEFAULT_FRAME_ID)
    if not isinstance(frame_id, str) or not frame_id:
        errors.append(f'{name}.frame_id 는 비어 있지 않은 문자열이어야 한다')
    else:
        pose['frame_id'] = frame_id
    if not errors and world:
        x0, y0 = float(world['origin_x']), float(world['origin_y'])
        x1, y1 = x0 + float(world['width']), y0 + float(world['height'])
        if not (x0 <= pose['x'] <= x1 and y0 <= pose['y'] <= y1):
            errors.append(f'{name} 이 지도 범위 밖이다 (x {x0:g}~{x1:g}, y {y0:g}~{y1:g})')
    return (None if errors else pose), errors


def validate_task(payload, now=None, world=None):
    """
    폼/REST 입력 → (와이어 형식 Task JSON dict, 오류 목록). 오류가 있으면 dict 는 None.

    now 는 task_id 자동 생성에만 쓴다 (마감은 상대 초 그대로 보낸다).
    """
    now = time.time() if now is None else float(now)
    if not isinstance(payload, dict):
        return None, ['JSON 객체가 필요하다']
    errors = [f'모르는 필드: {k}' for k in sorted(set(payload) - _ALLOWED_KEYS, key=str)]
    task = {}

    task_id = payload.get('task_id') or make_task_id(now)
    if not isinstance(task_id, str) or not TASK_ID_RE.match(task_id):
        errors.append('task_id 는 1~64자의 영숫자/_ . : - 만 허용한다')
    task['task_id'] = task_id

    priority = payload.get('priority', DEFAULT_PRIORITY)
    if isinstance(priority, bool) or not isinstance(priority, int) or not 0 <= priority <= 255:
        errors.append('priority 는 0~255 정수여야 한다')
    task['priority'] = priority

    deadline, err = parse_deadline(payload)
    if err:
        errors.append(err)
    elif deadline is not None:
        task['deadline'] = deadline

    for key, aliases in _POSE_KEYS.items():
        raw, err = _pick(payload, aliases)
        if err:
            errors.append(err)
            continue
        if raw is None:
            errors.append(f'{key} 가 필요하다')
            continue
        pose, pose_errors = parse_pose(raw, key, world)
        errors.extend(pose_errors)
        if pose is not None:
            task[key] = pose

    item_type = payload.get('item_type')
    if item_type not in ITEM_TYPES:
        errors.append(f'item_type 은 {"/".join(ITEM_TYPES)} 중 하나여야 한다')
    task['item_type'] = item_type

    robot_id = payload.get('robot_id') or ''
    if not isinstance(robot_id, str) or (robot_id and not ROBOT_ID_RE.match(robot_id)):
        errors.append('robot_id 는 소문자로 시작하는 [a-z0-9_] 1~32자이거나 빈 문자열이어야 한다')
    elif robot_id:
        task['robot_id'] = robot_id

    return (None if errors else task), errors


def validate_estop(payload, known_robots):
    """{robot_id: 'all'|id, active: bool} → (target, active, 오류 목록)."""
    if not isinstance(payload, dict):
        return None, None, ['JSON 객체가 필요하다']
    errors = []
    target = payload.get('robot_id', 'all')
    if target is None or target == '':
        target = 'all'
    if not isinstance(target, str):
        errors.append('robot_id 는 문자열이어야 한다')
    elif target != 'all' and target not in known_robots:
        errors.append(f'알 수 없는 robot_id: {target} (허용: all, {", ".join(known_robots) or "-"})')
    active = payload.get('active')
    if not isinstance(active, bool):
        errors.append('active 는 true/false 여야 한다')
    return target, active, errors
