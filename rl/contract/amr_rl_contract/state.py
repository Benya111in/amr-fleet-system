"""
계약 상태 스키마 — 관측·마스크·보상·비용 함수의 입력 (golden JSON 의 "state" 블록과 1:1).

JSON 에는 inf/NaN 을 쓰지 않는다: 무반사 빔·상한 없음·TTC 없음은 null.
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

from . import constants as C


@dataclass
class Track:
    """추적 객체 하나 (TrackedObstacle.msg 의 부분집합, 지도 좌표, 메시지 스탬프 시각 기준)."""

    id: int
    x: float
    y: float
    vx: float
    vy: float
    is_dynamic: bool


@dataclass
class SafetyState:
    """safety node 상태 (직전 주기). speed_limit·min_ttc 의 None = +inf."""

    zone: int = C.ZONE_CLEAR
    speed_limit: Optional[float] = None
    proximity_stop: bool = False
    min_ttc: Optional[float] = None


@dataclass
class DevState:
    """이탈 상태기계 (reward machine). in_dev 가 거짓이면 peak·t_since_peak 는 0."""

    in_dev: bool = False
    peak: float = 0.0
    t_since_peak: float = 0.0


@dataclass
class State:
    """결정 시각의 계약 상태."""

    pose: Tuple[float, float, float]                 # 지도 좌표 base_footprint (x, y, θ)
    path: List[Tuple[float, float]]                  # A* 경로 (지도 좌표), 끝 = 목표
    scan_ranges: List[Optional[float]]               # 720 빔, None = 무반사
    v_meas: float = 0.0                              # 실행(측정) 속도
    w_meas: float = 0.0
    a_v_meas: float = 0.0                            # 실행 가속도 (프로파일러 상태)
    a_w_meas: float = 0.0
    v_cmd: float = 0.0                               # 직전 정책 명령 (적분 상태)
    w_cmd: float = 0.0
    prev_actions: List[Tuple[float, float]] = field(
        default_factory=lambda: [(0.0, 0.0)] * C.N_PREV_ACTIONS)   # 최근 순, 물리 단위
    v_cap: float = C.V_MAX_OP
    scan_age: float = 0.0
    tracks: List[Track] = field(default_factory=list)
    track_age: float = 0.0
    safety: SafetyState = field(default_factory=SafetyState)
    dev: DevState = field(default_factory=DevState)

    # ------------------------------------------------------------ JSON
    @staticmethod
    def from_dict(d):
        """JSON dict → State."""
        tracks = [Track(int(t['id']), float(t['x']), float(t['y']), float(t['vx']),
                        float(t['vy']), bool(t['is_dynamic'])) for t in d.get('tracks', [])]
        sf = d.get('safety', {})
        dv = d.get('dev', {})
        return State(
            pose=tuple(float(v) for v in d['pose']),
            path=[tuple(float(v) for v in p) for p in d['path']],
            scan_ranges=list(d['scan_ranges']),
            v_meas=float(d.get('v_meas', 0.0)), w_meas=float(d.get('w_meas', 0.0)),
            a_v_meas=float(d.get('a_v_meas', 0.0)), a_w_meas=float(d.get('a_w_meas', 0.0)),
            v_cmd=float(d.get('v_cmd', 0.0)), w_cmd=float(d.get('w_cmd', 0.0)),
            prev_actions=[tuple(float(v) for v in a) for a in d.get(
                'prev_actions', [(0.0, 0.0)] * C.N_PREV_ACTIONS)],
            v_cap=float(d.get('v_cap', C.V_MAX_OP)),
            scan_age=float(d.get('scan_age', 0.0)),
            tracks=tracks, track_age=float(d.get('track_age', 0.0)),
            safety=SafetyState(int(sf.get('zone', 0)), sf.get('speed_limit'),
                               bool(sf.get('proximity_stop', False)), sf.get('min_ttc')),
            dev=DevState(bool(dv.get('in_dev', False)), float(dv.get('peak', 0.0)),
                         float(dv.get('t_since_peak', 0.0))),
        )

    def to_dict(self):
        """State → JSON dict (inf 를 None 으로)."""
        def fin(v):
            return None if v is None or not math.isfinite(v) else float(v)
        return {
            'pose': [float(v) for v in self.pose],
            'path': [[float(v) for v in p] for p in self.path],
            'scan_ranges': [fin(v) if v is not None else None for v in self.scan_ranges],
            'v_meas': self.v_meas, 'w_meas': self.w_meas,
            'a_v_meas': self.a_v_meas, 'a_w_meas': self.a_w_meas,
            'v_cmd': self.v_cmd, 'w_cmd': self.w_cmd,
            'prev_actions': [[float(a[0]), float(a[1])] for a in self.prev_actions],
            'v_cap': self.v_cap, 'scan_age': self.scan_age,
            'tracks': [{'id': t.id, 'x': t.x, 'y': t.y, 'vx': t.vx, 'vy': t.vy,
                        'is_dynamic': t.is_dynamic} for t in self.tracks],
            'track_age': self.track_age,
            'safety': {'zone': self.safety.zone, 'speed_limit': fin(self.safety.speed_limit),
                       'proximity_stop': self.safety.proximity_stop,
                       'min_ttc': fin(self.safety.min_ttc)},
            'dev': {'in_dev': self.dev.in_dev, 'peak': self.dev.peak,
                    't_since_peak': self.dev.t_since_peak},
        }


def empty_scan():
    """무반사 720 빔."""
    return [None] * C.LIDAR_SAMPLES
