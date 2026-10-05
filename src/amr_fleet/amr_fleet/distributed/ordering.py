"""
sim 시각 · 요청 ID · 대기 순서 키 (프로토콜 명세 3·7 절).

시각 비교는 항상 (sec, nanosec) 전체로 한다. Open-RMF rmf_ros2 #551 은 claim 시각의 nanosec 만 비교해
무작위 쌍의 약 절반을 다르게 정렬했다 — 여기서는 Stamp 를 정규화된 (sec, nanosec) 튜플로만 비교하고
그 성질을 단위 시험(test_dist_ordering.py)으로 고정한다.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Any, Dict, Optional, Tuple, Union

NSEC = 1_000_000_000


@dataclasses.dataclass(frozen=True, order=True)
class Stamp:
    """sim 시각 (ROS builtin_interfaces/Time 과 같은 (sec, nanosec)). 0 ≤ nanosec < 1e9 로 정규화."""

    sec: int
    nanosec: int = 0

    def __post_init__(self):
        sec, nsec = int(self.sec), int(self.nanosec)
        carry, nsec = divmod(nsec, NSEC)
        object.__setattr__(self, 'sec', sec + carry)
        object.__setattr__(self, 'nanosec', nsec)

    @classmethod
    def from_seconds(cls, t: float) -> 'Stamp':
        """초(float) → Stamp. nanosec 로 반올림."""
        if not math.isfinite(t):
            raise ValueError(f'유한한 시각이 아님: {t!r}')
        total = int(round(t * NSEC))
        return cls(*divmod(total, NSEC))

    @classmethod
    def coerce(cls, t: Union['Stamp', float, int]) -> 'Stamp':
        """Stamp · 초(float/int) 를 Stamp 로."""
        if isinstance(t, Stamp):
            return t
        return cls.from_seconds(float(t))

    def to_seconds(self) -> float:
        """Stamp → 초(float). 표시·로그용 (비교에는 쓰지 않는다)."""
        return self.sec + self.nanosec / NSEC

    def total_ns(self) -> int:
        """정수 나노초."""
        return self.sec * NSEC + self.nanosec

    def add(self, seconds: float) -> 'Stamp':
        """주어진 초(seconds) 뒤의 시각."""
        return Stamp(0, self.total_ns() + int(round(seconds * NSEC)))

    def to_dict(self) -> Dict[str, int]:
        """JSON 용."""
        return {'sec': self.sec, 'nanosec': self.nanosec}

    @classmethod
    def from_dict(cls, d: Optional[Dict[str, Any]]) -> Optional['Stamp']:
        """to_dict 의 역 (None 은 그대로)."""
        return None if d is None else cls(int(d['sec']), int(d['nanosec']))


def claim_key(stamp: Stamp, robot_id: str) -> Tuple[int, int, str]:
    """
    claim(요청 시각, 로봇) 순서 키: (sec, nanosec, robot_id).

    같은 시각이면 robot_id 로 깬다. Open-RMF #551 의 버그(nanosec 만 비교)를 되풀이하지 않도록
    비교 대상 전체를 한 튜플로 만든다.
    """
    return (stamp.sec, stamp.nanosec, robot_id)


@dataclasses.dataclass(frozen=True, order=True)
class ReqId:
    """요청 ID = (로봇, 재시작 번호, 일련번호). 재시도마다 새 ID."""

    robot: str
    incarnation: int
    seq: int

    def to_list(self) -> list:
        """JSON 용."""
        return [self.robot, self.incarnation, self.seq]

    @classmethod
    def from_list(cls, v: Any) -> 'ReqId':
        """to_list 의 역."""
        return cls(str(v[0]), int(v[1]), int(v[2]))

    def __str__(self) -> str:
        return f'{self.robot}#{self.incarnation}.{self.seq}'


@dataclasses.dataclass(frozen=True)
class PriorityKey:
    """
    투표자 대기열 순서 키 (요청 메시지에 실려 모든 투표자가 같은 순서를 쓴다).

    순서 (protocol.md 7 절, traffic_zones.py 중앙 규칙을 옮김):
    기아 먼저(기아끼리는 since 오래된 순) → 작업 우선순위 높은 순 → 마감 이른 순 → since 오래된 순
    → robot_id → incarnation → seq.
    since 는 같은 논리 요청의 첫 시도 Lamport 시각 (재시도해도 유지 = 나이).
    """

    priority: int
    deadline: Optional[Stamp]
    since: int
    starved: bool = False

    def sort_key(self, req: ReqId) -> tuple:
        """정렬 키 (작을수록 먼저). 마감 없음 = 가장 늦음."""
        dl = (1, 0, 0) if self.deadline is None else (0, self.deadline.sec, self.deadline.nanosec)
        tail = (req.robot, req.incarnation, req.seq)
        if self.starved:
            return (0, self.since, -self.priority) + dl + tail
        return (1, -self.priority) + dl + (self.since,) + tail

    def to_dict(self) -> Dict[str, Any]:
        """JSON 용."""
        return {'priority': self.priority,
                'deadline': None if self.deadline is None else self.deadline.to_dict(),
                'since': self.since, 'starved': self.starved}

    @classmethod
    def from_dict(cls, d: Optional[Dict[str, Any]]) -> Optional['PriorityKey']:
        """to_dict 의 역 (None 은 그대로)."""
        if d is None:
            return None
        return cls(int(d['priority']), Stamp.from_dict(d.get('deadline')), int(d['since']),
                   bool(d.get('starved', False)))
