"""
TF 트리 무결성 판정 (rclpy 비의존 순수 모듈, 시나리오 02).

/tf, /tf_static 에서 관측한 변환을 (부모, 자식) 간선 그래프로 모으고, 계약의 기대 간선
(docs/architecture/components.md §4.2, README.md TF 트리)과 비교한다.

  - 간선 존재 / 누락 (누락 간선 목록을 그대로 보고)
  - 이중 부모 (한 자식 프레임을 두 부모가 발행 → TF 요동, 명세 4.9 "TF 프레임 충돌 없음")
  - 순환, 루트가 map 하나인지
  - 정적 변환 값 = config/sensors.yaml extrinsic, robot_params.yaml 치수 (명세 4.1
    "실제 센서 위치와 TF 트리가 정확히 일치")
  - 동적 간선 발행 주기 (EKF 50 Hz 등)
"""

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]     # (x, y, z, w)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    """ZYX 오일러 → 쿼터니언 (x, y, z, w). URDF rpy 와 같은 규약."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_angle(a: Quat, b: Quat) -> float:
    """두 회전 사이의 각도 [rad] (부호 무관, q 와 −q 동일)."""
    na = math.sqrt(sum(v * v for v in a)) or 1.0
    nb = math.sqrt(sum(v * v for v in b)) or 1.0
    dot = abs(sum(x * y for x, y in zip(a, b))) / (na * nb)
    return 2.0 * math.acos(min(1.0, dot))


@dataclass
class ObservedEdge:
    """관측한 간선 하나 (부모 → 자식)."""

    parent: str
    child: str
    static: bool = False
    count: int = 0
    first_stamp: float = math.inf
    last_stamp: float = -math.inf
    translation: Vec3 = (0.0, 0.0, 0.0)
    rotation: Quat = (0.0, 0.0, 0.0, 1.0)
    stamps: List[float] = field(default_factory=list)

    def rate(self) -> float:
        """동적 간선의 발행 주기 [Hz] (스탬프 기준, 정적 간선이나 표본 부족이면 0)."""
        if self.static or len(self.stamps) < 2:
            return 0.0
        uniq = sorted(set(self.stamps))
        if len(uniq) < 2 or uniq[-1] <= uniq[0]:
            return 0.0
        return (len(uniq) - 1) / (uniq[-1] - uniq[0])


class TfGraph:
    """관측 간선 그래프. 자식마다 본 부모를 모두 기억해 이중 부모를 잡는다."""

    MAX_STAMPS = 2000

    def __init__(self) -> None:
        self._edges: Dict[Tuple[str, str], ObservedEdge] = {}

    @staticmethod
    def _norm(frame: str) -> str:
        return frame.lstrip('/')

    def add(self, parent: str, child: str, stamp: float, static: bool,
            translation: Vec3 = (0.0, 0.0, 0.0),
            rotation: Quat = (0.0, 0.0, 0.0, 1.0)) -> None:
        """변환 하나를 기록한다."""
        key = (self._norm(parent), self._norm(child))
        edge = self._edges.get(key)
        if edge is None:
            edge = ObservedEdge(key[0], key[1], static=static)
            self._edges[key] = edge
        edge.static = edge.static or static
        edge.count += 1
        edge.first_stamp = min(edge.first_stamp, stamp)
        edge.last_stamp = max(edge.last_stamp, stamp)
        edge.translation = tuple(float(v) for v in translation)
        edge.rotation = tuple(float(v) for v in rotation)
        if not static and len(edge.stamps) < self.MAX_STAMPS:
            edge.stamps.append(stamp)

    @property
    def edges(self) -> List[ObservedEdge]:
        return list(self._edges.values())

    def edge(self, parent: str, child: str) -> Optional[ObservedEdge]:
        return self._edges.get((self._norm(parent), self._norm(child)))

    def frames(self) -> Set[str]:
        out: Set[str] = set()
        for p, c in self._edges:
            out.add(p)
            out.add(c)
        return out

    def parents_of(self, child: str) -> List[str]:
        c = self._norm(child)
        return sorted(p for (p, cc) in self._edges if cc == c)

    def duplicate_parents(self) -> Dict[str, List[str]]:
        """부모가 둘 이상인 자식 프레임 → 부모 목록."""
        out: Dict[str, List[str]] = {}
        for frame in self.frames():
            parents = self.parents_of(frame)
            if len(parents) > 1:
                out[frame] = parents
        return out

    def roots(self) -> Set[str]:
        """부모가 없는 프레임."""
        children = {c for (_, c) in self._edges}
        return {f for f in self.frames() if f not in children}

    def chain(self, frame: str, max_depth: int = 64) -> List[str]:
        """프레임 frame 에서 루트까지 (첫 부모를 따라감). 순환이면 ValueError."""
        out = [self._norm(frame)]
        seen = {out[0]}
        while len(out) <= max_depth:
            parents = self.parents_of(out[-1])
            if not parents:
                return out
            nxt = parents[0]
            if nxt in seen:
                raise ValueError(f'TF 순환: {" -> ".join(out + [nxt])}')
            out.append(nxt)
            seen.add(nxt)
        raise ValueError(f'TF 깊이 {max_depth} 초과: {frame}')

    def has_cycle(self) -> bool:
        try:
            for f in self.frames():
                self.chain(f)
        except ValueError:
            return True
        return False

    def to_dot(self, highlight: Iterable[Tuple[str, str]] = ()) -> str:
        """Graphviz DOT (view_frames 대용 시각화 산출물). highlight 간선은 빨간 점선(누락)."""
        lines = ['digraph tf {', '  rankdir=TB;', '  node [shape=box, fontsize=10];']
        for e in sorted(self._edges.values(), key=lambda e: (e.parent, e.child)):
            label = 'static' if e.static else f'{e.rate():.1f} Hz'
            lines.append(f'  "{e.parent}" -> "{e.child}" [label="{label}", fontsize=8];')
        for p, c in highlight:
            lines.append(f'  "{p}" -> "{c}" [color=red, style=dashed, label="missing"];')
        lines.append('}')
        return '\n'.join(lines) + '\n'


@dataclass(frozen=True)
class ExpectedEdge:
    """계약상 있어야 하는 간선과 기대 변환 (값을 모르면 None)."""

    parent: str
    child: str
    publisher: str                       # 계약상 발행자 (보고용)
    static: bool
    translation: Optional[Vec3] = None
    rotation: Optional[Quat] = None      # 동적 조인트(바퀴)는 None


@dataclass
class EdgeCheck:
    """기대 간선 하나의 판정 결과."""

    expected: ExpectedEdge
    present: bool
    parents_seen: List[str]
    translation_error: Optional[float] = None    # [m]
    rotation_error: Optional[float] = None       # [rad]
    rate_hz: float = 0.0
    problems: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.present and not self.problems

    def as_row(self) -> list:
        e = self.expected
        return [e.parent, e.child, e.publisher, 'static' if e.static else 'dynamic',
                int(self.present), '' if self.translation_error is None
                else f'{self.translation_error:.5f}',
                '' if self.rotation_error is None else f'{math.degrees(self.rotation_error):.3f}',
                f'{self.rate_hz:.2f}', int(self.ok), '; '.join(self.problems)]


EDGE_CSV_COLUMNS = ['parent', 'child', 'publisher', 'kind', 'present', 'translation_error_m',
                    'rotation_error_deg', 'rate_hz', 'ok', 'problems']


def _xyz(ext: Dict, what: str = 'extrinsic') -> Vec3:
    """외부 파라미터(extrinsic)의 x, y, z (필수 — 없으면 KeyError: 옛 기본값으로 판정하지 않는다)."""
    try:
        return (float(ext['x']), float(ext['y']), float(ext['z']))
    except KeyError as exc:
        raise KeyError(f'{what}.{exc.args[0]} 가 설정에 없다') from None


def _rpy(ext: Dict) -> Quat:
    return quat_from_rpy(float(ext.get('roll', 0.0)), float(ext.get('pitch', 0.0)),
                         float(ext.get('yaw', 0.0)))


def expected_edges(sensors: Dict, robot: Dict, ekf: Dict, prefix: str = '') -> List[ExpectedEdge]:
    """
    설정(config)에서 기대 간선 목록을 만든다.

    sensors = sensors.yaml, robot = robot_params.yaml, ekf = ekf.yaml(map 노드) 의
    ros__parameters. prefix 는 'map' 을 뺀 모든 프레임에 붙는다 (multi_robot.md §2).
    """
    def f(name: str) -> str:
        return name if name == 'map' or not prefix else prefix + name

    rb = robot.get('robot', {})
    try:
        height = float(rb['base_link_height'])
        sep = float(rb['wheel_separation'])
        radius = float(rb['wheel_radius'])
    except KeyError as exc:
        raise KeyError(f'robot_params.yaml robot.{exc.args[0]} 가 없다') from None
    lidar = sensors.get('lidar', {})
    cam = sensors.get('camera_link', {})
    imu = sensors.get('imu', {})
    rgb = sensors.get('rgb_camera', {})
    depth = sensors.get('depth_camera', {})
    optical = cam.get('optical_rpy', [-math.pi / 2, 0.0, -math.pi / 2])
    optical_q = quat_from_rpy(*[float(v) for v in optical])
    map_f = f(ekf.get('map_frame', 'map'))
    odom_f = f(ekf.get('odom_frame', 'odom'))
    base_fp = f(ekf.get('base_link_frame', 'base_footprint'))
    base = f('base_link')
    cam_f = f(cam.get('frame_id', 'camera_link'))
    rsp = 'robot_state_publisher'
    ident = (0.0, 0.0, 0.0, 1.0)
    return [
        ExpectedEdge(map_f, odom_f, 'ekf_filter_node_map', False),
        ExpectedEdge(odom_f, base_fp, 'ekf_filter_node_odom', False),
        ExpectedEdge(base_fp, base, rsp, True, (0.0, 0.0, height), ident),
        ExpectedEdge(base, f(lidar.get('frame_id', 'lidar_link')), rsp, True,
                     _xyz(lidar.get('extrinsic', {}), 'lidar.extrinsic'),
                     _rpy(lidar.get('extrinsic', {}))),
        ExpectedEdge(base, cam_f, rsp, True,
                     _xyz(cam.get('extrinsic', {}), 'camera_link.extrinsic'),
                     _rpy(cam.get('extrinsic', {}))),
        ExpectedEdge(cam_f, f(rgb.get('frame_id', 'camera_optical_frame')), rsp, True,
                     (0.0, 0.0, 0.0), optical_q),
        ExpectedEdge(cam_f, f(depth.get('frame_id', 'camera_depth_optical_frame')), rsp, True,
                     (0.0, 0.0, 0.0), optical_q),
        ExpectedEdge(base, f(imu.get('frame_id', 'imu_link')), rsp, True,
                     _xyz(imu.get('extrinsic', {}), 'imu.extrinsic'),
                     _rpy(imu.get('extrinsic', {}))),
        ExpectedEdge(base, f('left_wheel_link'), rsp + ' (joint_states)', False,
                     (0.0, sep / 2.0, radius - height), None),
        ExpectedEdge(base, f('right_wheel_link'), rsp + ' (joint_states)', False,
                     (0.0, -sep / 2.0, radius - height), None),
    ]


def check_edges(graph: TfGraph, expected: Sequence[ExpectedEdge], trans_tol: float = 1e-3,
                rot_tol: float = math.radians(0.1), min_rate: Dict[str, float] = None
                ) -> List[EdgeCheck]:
    """
    기대 간선마다 존재·부모 유일성·값·주기를 판정한다.

    min_rate: {child: 최소 Hz} — 동적 간선 주기 하한 (EKF 50 Hz 등).
    """
    min_rate = min_rate or {}
    out: List[EdgeCheck] = []
    for exp in expected:
        parents = graph.parents_of(exp.child)
        edge = graph.edge(exp.parent, exp.child)
        chk = EdgeCheck(exp, present=edge is not None, parents_seen=parents)
        if edge is None:
            chk.problems.append('missing' if not parents
                                else f'wrong parent {parents} (expected {exp.parent})')
            out.append(chk)
            continue
        if len(parents) > 1:
            chk.problems.append(f'multiple parents {parents}')
        if exp.translation is not None:
            chk.translation_error = float(np.linalg.norm(
                np.subtract(edge.translation, exp.translation)))
            if chk.translation_error > trans_tol:
                chk.problems.append(
                    f'translation {tuple(round(v, 4) for v in edge.translation)} != '
                    f'{tuple(round(v, 4) for v in exp.translation)}')
        if exp.rotation is not None:
            chk.rotation_error = quat_angle(edge.rotation, exp.rotation)
            if chk.rotation_error > rot_tol:
                chk.problems.append(f'rotation off by {math.degrees(chk.rotation_error):.2f} deg')
        chk.rate_hz = edge.rate()
        need = min_rate.get(exp.child)
        if need is not None and not edge.static and chk.rate_hz < need:
            chk.problems.append(f'rate {chk.rate_hz:.1f} Hz < {need:.1f} Hz')
        out.append(chk)
    return out


def detect_prefix(frames: Iterable[str], base: str = 'base_link') -> Optional[str]:
    """
    관측 프레임에서 로봇 접두어를 찾는다.

    'base_link' 가 있으면 '', 'amr_01/base_link' 하나만 있으면 'amr_01/'. 없거나 여럿이면 None.
    """
    names = {f.lstrip('/') for f in frames}
    if base in names:
        return ''
    found = sorted(n[:-len(base)] for n in names if n.endswith('/' + base))
    return found[0] if len(found) == 1 else None
