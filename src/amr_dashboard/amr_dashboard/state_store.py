"""
대시보드 상태 저장소 (순수 파이썬, rclpy 미의존).

ROS 메시지를 JSON 직렬화 가능한 dict 로 바꾸고, 최신 스냅샷을 스레드 안전하게 보관하며,
SSE 구독자 큐에 이벤트를 배포한다. 메시지 변환은 속성 접근만 사용하므로 실제 amr_msgs 메시지와
테스트용 대역(duck typing) 모두 받는다.

rclpy 실행기 스레드가 갱신하고 Flask 요청 스레드가 조회한다. 배포(broadcast)는 절대 막히지 않는다:
느린 구독자의 큐가 가득 차면 가장 오래된 항목을 버린다.
"""

import collections
import math
import queue
import threading
import time

from amr_dashboard import map_encoder

# amr_msgs/RobotState.status 상수
ROBOT_STATUS_NAMES = {
    0: 'IDLE',
    1: 'MOVING',
    2: 'DOCKING',
    3: 'LOADING',
    4: 'CHARGING',
    5: 'ERROR',
    6: 'ESTOP',
}

# amr_msgs/Task.status 상수
TASK_STATUS_NAMES = {
    0: 'PENDING',
    1: 'IN_PROGRESS',
    2: 'COMPLETED',
    3: 'FAILED',
}

# diagnostic_msgs/DiagnosticStatus.level 상수 (byte)
ALERT_LEVEL_NAMES = {
    0: 'OK',
    1: 'WARN',
    2: 'ERROR',
    3: 'STALE',
}

# 지도가 없을 때 캔버스가 그릴 기본 월드 (명세 4.1: 물류센터 60 m x 40 m)
DEFAULT_WORLD = {'origin_x': 0.0, 'origin_y': 0.0, 'width': 60.0, 'height': 40.0}


def finite(value, default=None):
    """JSON 에 실을 수 있도록 NaN/inf 를 default 로 바꾼 float 를 돌려준다."""
    try:
        f = float(value)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


def stamp_to_sec(stamp) -> float:
    """builtin_interfaces/Time → 초 (float)."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_yaw(q) -> float:
    """geometry_msgs/Quaternion → yaw [rad] (Z 축 회전)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def level_to_int(level) -> int:
    """DiagnosticStatus.level 은 길이 1 의 bytes 로 오므로 int 로 정규화한다."""
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else 0
    return int(level)


def pose_to_dict(pose_stamped) -> dict:
    """geometry_msgs/PoseStamped → {frame_id, x, y, yaw}."""
    pose = pose_stamped.pose
    return {
        'frame_id': pose_stamped.header.frame_id,
        'x': finite(pose.position.x, 0.0),
        'y': finite(pose.position.y, 0.0),
        'yaw': finite(quat_to_yaw(pose.orientation), 0.0),
    }


def robot_state_to_dict(msg) -> dict:
    """amr_msgs/RobotState → dict (status 이름 포함)."""
    status = int(msg.status)
    return {
        'robot_id': msg.robot_id,
        'stamp': stamp_to_sec(msg.header.stamp),
        'pose': pose_to_dict(msg.pose),
        'battery_level': finite(msg.battery_level, 0.0),
        'current_task_id': msg.current_task_id,
        'status': status,
        'status_name': ROBOT_STATUS_NAMES.get(status, 'UNKNOWN'),
    }


def fleet_status_to_dict(msg, received_at=None) -> dict:
    """amr_msgs/FleetStatus → {stamp, received_at, robots[], kpi{}}."""
    return {
        'stamp': stamp_to_sec(msg.header.stamp),
        'received_at': time.time() if received_at is None else received_at,
        'robots': [robot_state_to_dict(r) for r in msg.robots],
        'kpi': {
            'tasks_pending': int(msg.tasks_pending),
            'tasks_in_progress': int(msg.tasks_in_progress),
            'tasks_completed': int(msg.tasks_completed),
            'tasks_failed': int(msg.tasks_failed),
            'throughput': finite(msg.throughput, 0.0),
            'avg_task_duration': finite(msg.avg_task_duration, 0.0),
            'robot_utilization': finite(msg.robot_utilization, 0.0),
            'deadlock_count': int(msg.deadlock_count),
        },
    }


def diagnostic_array_to_dicts(msg, received_at=None) -> list:
    """diagnostic_msgs/DiagnosticArray → 알림 dict 목록 (status 하나당 하나)."""
    stamp = stamp_to_sec(msg.header.stamp)
    now = time.time() if received_at is None else received_at
    alerts = []
    for st in msg.status:
        level = level_to_int(st.level)
        alerts.append({
            'stamp': stamp,
            'received_at': now,
            'level': level,
            'level_name': ALERT_LEVEL_NAMES.get(level, 'UNKNOWN'),
            'name': st.name,
            'message': st.message,
            'hardware_id': st.hardware_id,
            'values': {kv.key: kv.value for kv in st.values},
        })
    return alerts


def task_to_dict(msg, received_at=None) -> dict:
    """amr_msgs/Task → dict (status 이름 포함)."""
    status = int(msg.status)
    return {
        'stamp': stamp_to_sec(msg.header.stamp),
        'received_at': time.time() if received_at is None else received_at,
        'task_id': msg.task_id,
        'robot_id': msg.robot_id,
        'priority': int(msg.priority),
        'deadline': stamp_to_sec(msg.deadline),
        'pickup_pose': pose_to_dict(msg.pickup_pose),
        'dropoff_pose': pose_to_dict(msg.dropoff_pose),
        'item_type': msg.item_type,
        'item_mass': finite(msg.item_mass, 0.0),
        'status': status,
        'status_name': TASK_STATUS_NAMES.get(status, 'UNKNOWN'),
    }


def occupancy_grid_meta(msg) -> dict:
    """nav_msgs/OccupancyGrid → 메타데이터 dict (data 제외)."""
    info = msg.info
    return {
        'stamp': stamp_to_sec(msg.header.stamp),
        'frame_id': msg.header.frame_id,
        'width': int(info.width),
        'height': int(info.height),
        'resolution': finite(info.resolution, 0.0),
        'origin': {
            'x': finite(info.origin.position.x, 0.0),
            'y': finite(info.origin.position.y, 0.0),
            'yaw': finite(quat_to_yaw(info.origin.orientation), 0.0),
        },
    }


class StateStore:
    """
    최신 플릿 상태 + 알림/작업 이벤트 이력 + 지도 + SSE 구독자 큐.

    모든 공개 메서드는 스레드 안전하다 (rclpy 실행기 스레드에서 갱신, Flask 스레드에서 조회).
    """

    def __init__(self, max_alerts=100, max_task_events=50, max_tasks=200,
                 queue_size=256, world=None, robot_ids=()):
        self._lock = threading.RLock()
        self._status = None
        self._alerts = collections.deque(maxlen=max_alerts)
        self._task_events = collections.deque(maxlen=max_task_events)
        self._tasks = collections.OrderedDict()
        self._max_tasks = max_tasks
        self._map_meta = None
        self._map_data = None
        self._map_cache = {}
        self._estops = {'all': False}
        self._last_task_request = None
        self._world = dict(DEFAULT_WORLD if world is None else world)
        self._configured_robots = list(robot_ids)
        self._seen_robots = []
        self._queue_size = queue_size
        self._subscribers = []
        self._seq = 0
        self._dropped = 0
        self._started_at = time.time()

    # ----- 메시지 갱신 (rclpy 콜백에서 호출) -----

    def update_status(self, msg) -> dict:
        """amr_msgs/FleetStatus 를 저장하고 'status' 이벤트를 배포한다."""
        data = fleet_status_to_dict(msg)
        with self._lock:
            self._status = data
            for robot in data['robots']:
                rid = robot['robot_id']
                if rid and rid not in self._seen_robots:
                    self._seen_robots.append(rid)
        self.broadcast('status', data)
        return data

    def add_alerts(self, msg) -> list:
        """diagnostic_msgs/DiagnosticArray 를 알림 이력에 넣고 'alerts' 이벤트를 배포한다."""
        alerts = diagnostic_array_to_dicts(msg)
        if not alerts:
            return alerts
        with self._lock:
            self._alerts.extend(alerts)
        self.broadcast('alerts', alerts)
        return alerts

    def add_task_event(self, msg) -> dict:
        """Task 이벤트를 타임라인에 넣고 'task_event' 이벤트를 배포한다."""
        event = task_to_dict(msg)
        with self._lock:
            self._task_events.append(event)
            self._tasks[event['task_id']] = event
            self._tasks.move_to_end(event['task_id'])
            while len(self._tasks) > self._max_tasks:
                self._tasks.popitem(last=False)
        self.broadcast('task_event', event)
        return event

    def update_map(self, msg) -> dict:
        """nav_msgs/OccupancyGrid 를 저장하고 'map_updated' 이벤트(메타데이터만)를 배포한다."""
        meta = occupancy_grid_meta(msg)
        data = map_encoder.grid_bytes(msg.data)
        with self._lock:
            self._map_meta = meta
            self._map_data = data
            self._map_cache = {}
            if meta['width'] > 0 and meta['height'] > 0 and meta['resolution'] > 0:
                self._world = {
                    'origin_x': meta['origin']['x'],
                    'origin_y': meta['origin']['y'],
                    'width': meta['width'] * meta['resolution'],
                    'height': meta['height'] * meta['resolution'],
                }
        self.broadcast('map_updated', meta)
        return meta

    # ----- 조작 (HTTP 핸들러에서 호출) -----

    def set_estop(self, target: str, active: bool) -> dict:
        """E-stop 상태를 기록하고 'estop' 이벤트를 배포한다. target 은 robot_id 또는 'all'."""
        with self._lock:
            self._estops[target] = bool(active)
            if target == 'all' and not active:
                # 전체 해제는 개별 래치도 함께 푼다 (버튼 표시 일관성)
                for key in list(self._estops):
                    self._estops[key] = False
            estops = dict(self._estops)
        data = {'target': target, 'active': bool(active), 'estops': estops, 'ts': time.time()}
        self.broadcast('estop', data)
        return data

    def record_task_request(self, task: dict) -> dict:
        """웹 폼에서 투입한 작업 요청을 기록하고 'task_request' 이벤트를 배포한다."""
        with self._lock:
            self._last_task_request = task
        self.broadcast('task_request', task)
        return task

    # ----- 조회 -----

    def snapshot(self) -> dict:
        """전체 상태의 JSON 직렬화 가능한 복사본 (지도 셀 데이터는 제외)."""
        with self._lock:
            return {
                'server_time': time.time(),
                'uptime_sec': time.time() - self._started_at,
                'world': dict(self._world),
                'robot_ids': self.known_robot_ids(),
                'status': self._status,
                'alerts': list(self._alerts),
                'task_events': list(self._task_events),
                'tasks': dict(self._tasks),
                'map': self._map_meta,
                'estops': dict(self._estops),
                'last_task_request': self._last_task_request,
                'sse_clients': len(self._subscribers),
                'events_dropped': self._dropped,
            }

    def known_robot_ids(self) -> list:
        """E-stop 대상으로 허용하는 robot_id: 설정값 ∪ FleetStatus 에서 본 것 (순서 유지)."""
        with self._lock:
            ids = list(self._configured_robots)
            for rid in self._seen_robots:
                if rid not in ids:
                    ids.append(rid)
            return ids

    def map_meta(self):
        """지도 메타데이터 dict, 지도가 없으면 None."""
        with self._lock:
            return self._map_meta

    def map_png(self):
        """지도를 8-bit 그레이스케일 PNG bytes 로 (캐시), 지도가 없으면 None."""
        with self._lock:
            if self._map_meta is None:
                return None
            if 'png' not in self._map_cache:
                self._map_cache['png'] = map_encoder.grid_to_png(
                    self._map_meta['width'], self._map_meta['height'], self._map_data)
            return self._map_cache['png']

    def map_json(self):
        """지도를 {meta..., encoding: 'rle', data: [[value, count], ...]} 로 (캐시)."""
        with self._lock:
            if self._map_meta is None:
                return None
            if 'json' not in self._map_cache:
                doc = dict(self._map_meta)
                doc['encoding'] = 'rle'
                doc['data'] = map_encoder.grid_to_rle(self._map_data)
                self._map_cache['json'] = doc
            return self._map_cache['json']

    @property
    def world(self) -> dict:
        """캔버스 기준 월드 범위 [m]."""
        with self._lock:
            return dict(self._world)

    # ----- SSE 구독 -----

    def subscribe(self) -> queue.Queue:
        """새 구독자 큐를 만들어 등록한다. 큐 항목은 (event, data, seq)."""
        q = queue.Queue(maxsize=self._queue_size)
        with self._lock:
            self._subscribers.append(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        """구독자 큐를 제거한다 (이미 없으면 무시)."""
        with self._lock:
            try:
                self._subscribers.remove(q)
            except ValueError:
                pass

    def subscriber_count(self) -> int:
        """현재 SSE 구독자 수."""
        with self._lock:
            return len(self._subscribers)

    def broadcast(self, event: str, data) -> int:
        """
        모든 구독자 큐에 (event, data, seq) 를 넣는다.

        큐가 가득 찬 느린 구독자는 가장 오래된 항목을 버리고 넣는다 (생산자는 절대 막히지 않는다).
        돌려주는 값은 전달한 구독자 수.
        """
        with self._lock:
            self._seq += 1
            seq = self._seq
            subscribers = list(self._subscribers)
        delivered = 0
        for q in subscribers:
            item = (event, data, seq)
            try:
                q.put_nowait(item)
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
                with self._lock:
                    self._dropped += 1
                try:
                    q.put_nowait(item)
                except queue.Full:
                    continue
            delivered += 1
        return delivered
