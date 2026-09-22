"""
traffic_manager_node — 중앙 교통 관리 · 교착 탐지/해소 (components.md §3.6 / §5.6, sequences.md §3).

네임스페이스 /fleet 에서 뜬다 (토픽은 모두 절대 이름).
- Sub  /map (nav_msgs/OccupancyGrid, latched)                     구역 자동 유도 · 포켓 탐색 격자
- Sub  /amr_XX/plan (nav_msgs/Path), /amr_XX/odometry/filtered_map (nav_msgs/Odometry),
       /amr_XX/robot_state (amr_msgs/RobotState)                  로봇마다
- Sub  /fleet/task_events (amr_msgs/Task)                         작업 우선순위 · 마감 (task_id 로 찾는다)
- Pub  /amr_XX/traffic/hold (std_msgs/Bool, latched)              우선순위 양보 · 구역 대기 · 교차 양보
- Pub  /amr_XX/traffic/yield_pose (geometry_msgs/PoseStamped)     대기 포켓 (hold=true 와 함께)
- Pub  /amr_XX/keepout_mask (nav_msgs/OccupancyGrid, latched)     전략 2 — 분쟁 구간 lethal(100), 해소 후 0
- Pub  /amr_XX/costmap_filter_info (nav2_msgs/CostmapFilterInfo, latched)
       type=0 (keepout), filter_mask_topic=/amr_XX/keepout_mask (절대 이름 — Nav2 costmap 노드의
       네임스페이스에서 상대 이름이 풀리면 /amr_XX/global_costmap/keepout_mask 가 된다), base 0, multiplier 1
- Pub  /fleet/traffic_events (DiagnosticArray)
       traffic/DEADLOCK · RESOLVED · ESCALATED · UNRESOLVED · STALL
       (fleet_manager_node 는 traffic/DEADLOCK 만 deadlock_count 에 센다)
- Pub  /fleet/alerts (DiagnosticArray)          fleet/DEADLOCK (탐지 ERROR · 해소 OK · 해소 실패 ERROR)

update_rate_hz(2 Hz) 마다 관측을 모아 TrafficManager.update() 한 번 → 바뀐 명령만 발행한다.
관측 시각은 수신 시각(노드 시계)이고 obs_timeout_s 보다 오래된 로봇은 이번 주기에서 뺀다.
로직은 순수 모듈(traffic_manager · traffic_prediction · traffic_zones · deadlock · traffic_resolution)이
갖고 이 노드는 ROS 입출력과 타이머만 맡는다. 모든 콜백은 예외를 삼키고 fleet/INTERNAL_ERROR 로 알린다.
"""

from __future__ import annotations

import array
import collections
import functools
import math
import pathlib
import time
import traceback
from typing import Deque, Dict, List, Optional, Tuple

from amr_fleet.alerts import ALERT_INTERNAL_ERROR, alert_values
from amr_fleet.task_schema import ROBOT_ID_RE, safe_text
from amr_fleet.traffic_geometry import GridSpec
from amr_fleet.traffic_manager import (
    LEVEL_ERROR, LEVEL_OK, LEVEL_WARN, ST_MOVING, RobotCommand, RobotObservation, TrafficConfig,
    TrafficEvent, TrafficManager, occupancy_from_values,
)
from amr_fleet.traffic_resolution import KeepoutMask, empty_mask
from amr_msgs.msg import RobotState, Task
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

# components.md §5 의 latched (transient_local 구독은 volatile 발행자와 짝이 맺어지지 않는다)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
EVENTS_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
# 오도메트리는 best-effort 구독 (reliable · best-effort 발행자 모두와 맞는다)
ODOM_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
KEEPOUT_FILTER_TYPE = 0          # nav2_costmap_2d KEEPOUT_FILTER
TASK_TABLE_MAX = 2000            # task_id → (우선순위, 마감) 표 상한 (오래된 것부터 버린다)
_LEVELS = {LEVEL_OK: DiagnosticStatus.OK, LEVEL_WARN: DiagnosticStatus.WARN,
           LEVEL_ERROR: DiagnosticStatus.ERROR}


def _guarded(fn):
    """콜백 예외를 삼키고 fleet/INTERNAL_ERROR 로 알린다 (노드는 계속 돈다)."""
    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        try:
            return fn(self, *args, **kwargs)
        except Exception as exc:  # noqa: B902 — 어떤 입력도 노드를 죽이지 않는다
            self._internal_error(fn.__name__, exc)
            return None
    return wrapper


def parse_ids(value) -> List[str]:
    """robot_ids 파라미터: 쉼표 구분 문자열 또는 문자열 배열 (형식이 틀린 id 는 뺀다)."""
    items = value.split(',') if isinstance(value, str) else list(value or [])
    out: List[str] = []
    for item in items:
        rid = str(item).strip()
        if rid and ROBOT_ID_RE.match(rid) and rid not in out:
            out.append(rid)
    return out


def default_zones_path() -> str:
    """config/traffic_zones.yaml 위치. 소스 트리(symlink-install 포함) → ament share 순."""
    here = pathlib.Path(__file__).resolve()
    candidate = here.parents[1] / 'config' / 'traffic_zones.yaml'
    if candidate.is_file():
        return str(candidate)
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(pathlib.Path(get_package_share_directory('amr_fleet'))
                   / 'config' / 'traffic_zones.yaml')
    except Exception:  # noqa: B902 — ament 가 없는 순수 파이썬 환경
        return str(candidate)


def yaw_from_quaternion(z: float, w: float, x: float = 0.0, y: float = 0.0) -> float:
    """쿼터니언 → yaw [rad]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def path_to_array(msg: Path) -> Optional[np.ndarray]:
    """nav_msgs/Path → (N, 2) 배열 (점 2개 미만이면 None)."""
    if len(msg.poses) < 2:
        return None
    return np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses], dtype=float)


def mask_to_msg(mask: KeepoutMask, frame_id: str, stamp) -> OccupancyGrid:
    """마스크(KeepoutMask) → nav_msgs/OccupancyGrid (행 = y, 0 / 100)."""
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.info.map_load_time = stamp
    msg.info.resolution = float(mask.spec.resolution)
    msg.info.width = int(mask.spec.width)
    msg.info.height = int(mask.spec.height)
    msg.info.origin.position.x = float(mask.spec.origin_x)
    msg.info.origin.position.y = float(mask.spec.origin_y)
    msg.info.origin.orientation.w = 1.0
    msg.data = array.array('b', np.ascontiguousarray(mask.data, dtype=np.int8).tobytes())
    return msg


class _Robot:
    """로봇 1대의 최신 입력과 발행 상태."""

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.odom: Optional[Tuple[float, float, float, float, float]] = None  # x, y, yaw, v, t
        self.plan: Optional[np.ndarray] = None
        self.status: Optional[int] = None
        self.task_id = ''
        self.pub_hold = None
        self.pub_yield = None
        self.pub_mask = None
        self.pub_info = None
        self.hold: Optional[bool] = None            # 마지막으로 발행한 값
        self.yield_pose: Optional[Tuple[float, float, float]] = None
        self.keepout: Optional[KeepoutMask] = None


class TrafficManagerNode(Node):
    """plan · 자세 · 상태 → 충돌 예측 · 구역 토큰 · 교착 탐지/해소 → hold · yield_pose · keepout."""

    def __init__(self, **kwargs):
        kwargs.setdefault('namespace', '/fleet')
        super().__init__('traffic_manager_node', **kwargs)
        self.internal_errors = 0
        self._declare_params()
        p = self._param
        flat = {k: p(k) for k in TrafficConfig().to_flat()}
        if not flat['zones_file'] and flat['zones.source'] in ('yaml', 'both'):
            flat['zones_file'] = default_zones_path()
        self.cfg = TrafficConfig.from_flat(flat)
        self.tm = TrafficManager(self.cfg)
        self.frame_id = str(p('frame_id'))
        self.event_counts: Dict[str, int] = collections.Counter()
        self.tick_ms: Deque[float] = collections.deque(maxlen=2000)
        self._tasks: 'collections.OrderedDict[str, Tuple[int, Optional[float]]]' = \
            collections.OrderedDict()
        self._map_warned = False

        self._pub_events = self.create_publisher(DiagnosticArray, '/fleet/traffic_events',
                                                 EVENTS_QOS)
        self._pub_alerts = self.create_publisher(DiagnosticArray, '/fleet/alerts', EVENTS_QOS)
        self.create_subscription(OccupancyGrid, str(p('map_topic')), self._on_map, LATCHED_QOS)
        self.create_subscription(Task, '/fleet/task_events', self._on_task_event, EVENTS_QOS)
        self._robots: Dict[str, _Robot] = {}
        for rid in parse_ids(p('robot_ids')):
            self._add_robot(rid)
        if not self._robots:
            self.get_logger().error('robot_ids 가 비었다 — 관리할 로봇이 없다')
        rate = max(float(p('update_rate_hz')), 0.1)
        self.create_timer(1.0 / rate, self._on_tick)
        self.get_logger().info(
            f'traffic_manager 시작: mode={self.cfg.mode}, robots={sorted(self._robots)}, '
            f'rate={rate:g} Hz, zones={self.tm.describe_zones()}, pockets={len(self.tm.pockets)} '
            f'({self.cfg.zones_file or "파일 없음"})')

    # ------------------------------------------------------------------ 파라미터
    def _declare_params(self) -> None:
        defaults = {
            'robot_ids': '',                 # "amr_01,amr_02" 또는 문자열 배열
            'update_rate_hz': 2.0,           # 평가 주기 (components.md §3.6: 2 Hz)
            'map_topic': '/map',
            'frame_id': 'map',               # plan · 자세 · 마스크 프레임
        }
        defaults.update(TrafficConfig().to_flat())
        for name, value in defaults.items():
            descriptor = ParameterDescriptor(dynamic_typing=name == 'robot_ids')
            self.declare_parameter(name, value, descriptor)

    def _param(self, name: str):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ 유틸
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _internal_error(self, where: str, exc: BaseException) -> None:
        self.internal_errors += 1
        try:
            self.get_logger().error(f'{where} 예외 (무시하고 계속): {exc!r}\n{traceback.format_exc()}')
            self._publish_alert(DiagnosticStatus.ERROR, ALERT_INTERNAL_ERROR,
                                f'traffic_manager {where}: {type(exc).__name__}', '',
                                {'callback': where, 'error': safe_text(exc, 120)})
        except Exception:  # noqa: B902 — 알림 실패가 다시 콜백을 죽이지 않게
            pass

    def _publish_alert(self, level: bytes, name: str, message: str, hardware_id: str,
                       values: Dict[str, str]) -> None:
        st = DiagnosticStatus(level=level, name=name, message=safe_text(message, 300),
                              hardware_id=hardware_id)
        st.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self._pub_alerts.publish(arr)

    # ------------------------------------------------------------------ 로봇 등록 · 입력
    def _add_robot(self, rid: str) -> None:
        r = self._robots[rid] = _Robot(rid)
        base = f'/{rid}'
        self.create_subscription(Path, f'{base}/plan',
                                 functools.partial(self._on_plan, rid), 5)
        self.create_subscription(Odometry, f'{base}/odometry/filtered_map',
                                 functools.partial(self._on_odom, rid), ODOM_QOS)
        self.create_subscription(RobotState, f'{base}/robot_state',
                                 functools.partial(self._on_state, rid), 10)
        r.pub_hold = self.create_publisher(Bool, f'{base}/traffic/hold', LATCHED_QOS)
        r.pub_yield = self.create_publisher(PoseStamped, f'{base}/traffic/yield_pose', 10)
        r.pub_mask = self.create_publisher(OccupancyGrid, f'{base}/keepout_mask', LATCHED_QOS)
        r.pub_info = self.create_publisher(CostmapFilterInfo, f'{base}/costmap_filter_info',
                                           LATCHED_QOS)
        info = CostmapFilterInfo()
        info.header.frame_id = self.frame_id
        info.header.stamp = self.get_clock().now().to_msg()
        info.type = KEEPOUT_FILTER_TYPE
        info.filter_mask_topic = f'{base}/keepout_mask'
        info.base = 0.0
        info.multiplier = 1.0
        r.pub_info.publish(info)
        self._publish_hold(r, False)          # 늦게 뜬 실행기도 확실한 초기값(false)을 받는다

    @_guarded
    def _on_plan(self, rid: str, msg: Path) -> None:
        frame = msg.header.frame_id
        if frame and frame.lstrip('/') != self.frame_id:
            self.get_logger().warn(f'{rid}/plan 프레임 {frame!r} ≠ {self.frame_id} — 무시',
                                   throttle_duration_sec=10.0)
            return
        self._robots[rid].plan = path_to_array(msg)

    @_guarded
    def _on_odom(self, rid: str, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pos = msg.pose.pose.position
        v = msg.twist.twist.linear
        vals = (pos.x, pos.y, yaw_from_quaternion(q.z, q.w, q.x, q.y), math.hypot(v.x, v.y))
        if not all(math.isfinite(x) for x in vals):
            return
        self._robots[rid].odom = vals + (self._now(),)

    @_guarded
    def _on_state(self, rid: str, msg: RobotState) -> None:
        r = self._robots[rid]
        r.status = int(msg.status)
        r.task_id = str(msg.current_task_id)

    @_guarded
    def _on_task_event(self, msg: Task) -> None:
        deadline = msg.deadline.sec + msg.deadline.nanosec * 1e-9
        self._tasks[msg.task_id] = (int(msg.priority), deadline if deadline > 0.0 else None)
        self._tasks.move_to_end(msg.task_id)
        while len(self._tasks) > TASK_TABLE_MAX:
            self._tasks.popitem(last=False)

    @_guarded
    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        q = info.origin.orientation
        if abs(yaw_from_quaternion(q.z, q.w, q.x, q.y)) > 1e-3 and not self._map_warned:
            self._map_warned = True
            self.get_logger().warn('/map 원점이 회전돼 있다 — 회전은 무시한다')
        values = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        spec = GridSpec(float(info.resolution), float(info.origin.position.x),
                        float(info.origin.position.y), int(info.width), int(info.height))
        t0 = time.monotonic()
        self.tm.set_map(occupancy_from_values(values, self.cfg.zones.occupied_threshold), spec)
        self.get_logger().info(
            f'/map {info.width}x{info.height} @ {info.resolution:g} m 반영 '
            f'({(time.monotonic() - t0) * 1e3:.0f} ms): 구역 {self.tm.describe_zones()}')
        for r in self._robots.values():       # 지도 전체 크기 빈 마스크 (KeepoutFilter 가 마스크를 기다린다)
            if r.keepout is None:
                r.pub_mask.publish(mask_to_msg(empty_mask(self.tm.mask_spec), self.frame_id,
                                               self.get_clock().now().to_msg()))

    # ------------------------------------------------------------------ 주기
    def observations(self) -> Dict[str, RobotObservation]:
        """최신 입력 → TrafficManager 관측 (오도메트리가 없는 로봇은 뺀다)."""
        out: Dict[str, RobotObservation] = {}
        for rid, r in self._robots.items():
            if r.odom is None:
                continue
            x, y, yaw, speed, stamp = r.odom
            status = ST_MOVING if r.status is None else r.status   # 상태 미수신: 주행 중으로 본다
            prio, deadline = self._tasks.get(r.task_id, (-1, None)) if r.task_id else (-1, None)
            out[rid] = RobotObservation(rid, x, y, yaw, speed, r.plan, status, r.task_id,
                                        prio, deadline, stamp)
        return out

    @_guarded
    def _on_tick(self) -> None:
        t0 = time.perf_counter()
        now = self._now()
        res = self.tm.update(now, self.observations())
        for rid, cmd in res.commands.items():
            self._apply(self._robots[rid], cmd)
        if res.events:
            self._publish_events(res.events)
        self.tick_ms.append((time.perf_counter() - t0) * 1e3)

    def _apply(self, r: _Robot, cmd: RobotCommand) -> None:
        stamp = self.get_clock().now().to_msg()
        if cmd.keepout is not r.keepout:
            if cmd.keepout is not None:
                r.pub_mask.publish(mask_to_msg(cmd.keepout, self.frame_id, stamp))
                self.get_logger().info(f'{r.robot_id} keepout {cmd.keepout.lethal_cells} 셀 '
                                       f'(사건 {cmd.incident_id})')
            else:
                spec = self.tm.mask_spec or r.keepout.spec
                r.pub_mask.publish(mask_to_msg(empty_mask(spec), self.frame_id, stamp))
                self.get_logger().info(f'{r.robot_id} keepout 해제')
            r.keepout = cmd.keepout
        if cmd.yield_pose is not None and cmd.yield_pose != r.yield_pose:
            x, y, yaw = cmd.yield_pose
            msg = PoseStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = stamp
            msg.pose.position.x, msg.pose.position.y = float(x), float(y)
            msg.pose.orientation.z = math.sin(0.5 * yaw)
            msg.pose.orientation.w = math.cos(0.5 * yaw)
            r.pub_yield.publish(msg)          # hold=true 보다 먼저 — 실행기가 hold 를 보고 포켓을 찾는다
        r.yield_pose = cmd.yield_pose
        if cmd.hold != r.hold:
            self._publish_hold(r, cmd.hold, cmd.reason)

    def _publish_hold(self, r: _Robot, hold: bool, reason: str = '') -> None:
        r.pub_hold.publish(Bool(data=bool(hold)))
        if r.hold is not None:
            self.get_logger().info(f'{r.robot_id} hold={hold} {reason}')
        r.hold = bool(hold)

    def _publish_events(self, events: List[TrafficEvent]) -> None:
        stamp = self.get_clock().now().to_msg()
        arr = DiagnosticArray()
        arr.header.stamp = stamp
        for ev in events:
            self.event_counts[ev.name] += 1
            level = _LEVELS.get(ev.level, DiagnosticStatus.WARN)
            st = DiagnosticStatus(level=level, name=ev.name, message=safe_text(ev.message, 300),
                                  hardware_id=ev.hardware_id)
            st.values = [KeyValue(key=k, value=safe_text(v)) for k, v in ev.values.items()]
            arr.status.append(st)
            text = f'{ev.name} [{ev.hardware_id}] {ev.message} {ev.values}'
            if ev.level == LEVEL_ERROR:
                self.get_logger().error(text)
            elif ev.level == LEVEL_OK:
                self.get_logger().info(text)
            else:
                self.get_logger().warn(text)
            if ev.alert_name:
                victim = self._robots.get(ev.hardware_id)
                extra = {k: v for k, v in ev.values.items() if k != 'robots'}
                extra['event'] = ev.name
                self._publish_alert(level, ev.alert_name, ev.message, ev.hardware_id,
                                    alert_values(victim.task_id if victim else '',
                                                 ','.join(ev.robots), **extra))
        self._pub_events.publish(arr)


def main(args=None):
    """ros2 run amr_fleet traffic_manager_node."""
    rclpy.init(args=args)
    node = TrafficManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
