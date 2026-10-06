"""
실행 설정 · 비교군 · 외생 입력(작업 열 · 고장 일정 · 시드 파생).

쌍(pair) = 같은 seed. 같은 seed 의 실행은 비교군과 무관하게 **같은 외생 입력**을 받는다: 작업 도착 시각·내용,
고장 시각·대상, 링크·메시지 단위 통신 난수(CommEmulator 시드), 실행 잡음. rep 는 A/A 산포용 미시 반복 번호다 —
rep 가 다르면 통신 난수와 실행 잡음만 바뀌고 작업 열·고장은 같다 (Gazebo 의 실행 간 비결정성에 해당).
"""

from __future__ import annotations

import dataclasses
import hashlib
import math
import random
from typing import Dict, List, Optional, Tuple

from amr_fleet.task_schema import Pose2D, TaskSpec

CENTRAL_ALLOC = ('nearest', 'hungarian', 'ssi')
CBAA_ALLOC = ('cbaa_strict', 'cbaa_opt', 'cbaa_event')
ALLOC = CENTRAL_ALLOC + CBAA_ALLOC
TRAFFIC = ('central', 'vote', 'none', 'hold')
# central: 서버가 2 Hz 로 보내는 보유 구역 집합에만 들어간다 (허가 목록, 서버가 죽으면 fail-closed — P4 1 차 모형)
# hold   : 현행 traffic/hold (std_msgs/Bool, latched) 의미 — 토큰을 못 받은 로봇만 hold=true, 바뀔 때만 신뢰 경로로
#          보내고 로봇은 마지막 값을 따른다 (첫 값 전 = 대기). 서버가 죽으면 hold=false 로봇은 관리 없이 움직인다
#          (fail-open, hold_failsafe.md D4 · §4 위험 1). 리뷰 B1 반영 비교군 A1h · A2h · A5h
FAULTS = ('F0', 'F1', 'F5', 'F6')
FAULT_DOC = {
    'F0': '무장애',
    'F1': '서버(fleet_manager + traffic_manager) 충돌, 재기동 없음',
    'F5': '부분 분할 {amr_01, amr_02} 대 {amr_03, amr_04, amr_05, 서버, 작업 게시판}',
    'F6': '로봇 1 대 충돌(이탈) — 그 자리에 멈추고 송수신 중단, 재기동 없음',
}
ITEMS = {'small': 5.0, 'medium': 10.0, 'large': 15.0}   # robot_params.yaml payload load_time
PRIORITIES = (50, 100, 150, 200)


@dataclasses.dataclass(frozen=True)
class Arm:
    """비교군: 할당 × 교통 × 폴백."""

    name: str
    alloc: str
    traffic: str
    fallback: bool = False       # 중앙 할당 + 서버 과반 DEAD 시 CBAA(엄격) 협상으로 전환

    def __post_init__(self):
        if self.alloc not in ALLOC:
            raise ValueError(f'alloc 은 {ALLOC} 중 하나: {self.alloc!r}')
        if self.traffic not in TRAFFIC:
            raise ValueError(f'traffic 은 {TRAFFIC} 중 하나: {self.traffic!r}')
        if self.fallback and self.alloc not in CENTRAL_ALLOC:
            raise ValueError('fallback 은 중앙 할당 비교군에만')

    @property
    def central_alloc(self) -> bool:
        return self.alloc in CENTRAL_ALLOC

    @property
    def central_traffic(self) -> bool:
        """중앙 서버(traffic_manager)가 구역 토큰을 관리하는가 (허가 목록 또는 latched hold)."""
        return self.traffic in ('central', 'hold')

    @property
    def needs_task_lock(self) -> bool:
        """픽업 전에 작업 ID 투표 잠금이 필요한가 (엄격 협상 · 폴백 설계)."""
        return self.alloc in ('cbaa_strict', 'cbaa_event') or self.fallback


ARMS: Dict[str, Arm] = {a.name: a for a in (
    Arm('A1_nearest_central', 'nearest', 'central'),          # 현행 기본
    Arm('A2_hungarian_central', 'hungarian', 'central'),
    Arm('A3_ssi_central', 'ssi', 'central'),
    Arm('A4_hungarian_vote', 'hungarian', 'vote'),             # 2×2: 할당 중앙 × 교통 분산
    Arm('A5_cbaa_central', 'cbaa_strict', 'central'),          # 2×2: 할당 분산 × 교통 중앙
    Arm('A6_cbaa_vote', 'cbaa_strict', 'vote'),                # D1 완전 분산
    Arm('A7_cbaa_opt_vote', 'cbaa_opt', 'vote'),               # D2 낙관 (잠금 없음)
    Arm('A8_cbaa_event_vote', 'cbaa_event', 'vote'),           # D4 이벤트 구동 방송 (부정 대조군)
    Arm('A9_hybrid_vote', 'hungarian', 'vote', fallback=True),  # 제안 설계: 중앙 최적화 + 투표 안전 + 폴백
    Arm('X0_hungarian_none', 'hungarian', 'none'),             # 진단: 교통 무시(유령) 기준선
    # 리뷰 B1: 현행 latched hold 의미 (서버 사망 뒤 hold=false 로봇은 fail-open) — A1 · A5 의 충실도 대조
    Arm('A1h_nearest_hold', 'nearest', 'hold'),                # 현행 기본, hold 의미
    Arm('A2h_hungarian_hold', 'hungarian', 'hold'),            # 2×2 CC 칸, hold 의미
    Arm('A5h_cbaa_hold', 'cbaa_strict', 'hold'),               # 2×2 DC 칸, hold 의미
)}


@dataclasses.dataclass(frozen=True)
class RunConfig:
    """실행 1 건."""

    arm: Arm
    loss: float = 0.0
    fault: str = 'F0'
    seed: int = 1
    rep: int = 0
    horizon_s: float = 1200.0
    arrival_rate: float = 1.0 / 30.0       # 작업 도착률 [1/s] (포아송)
    t_fault: float = 300.0
    partition_s: float = 180.0             # F5 분할 지속
    latency_ms: Tuple[float, float] = (0.0, 100.0)
    dt: float = 0.1
    dock_overhead_s: float = 10.0          # 도킹 접근·이탈 고정 시간 (load_time 에 더한다)
    speed_noise: Tuple[float, float] = (0.8, 1.0)   # 구간별 최고 속도 배율
    dwell_noise: Tuple[float, float] = (0.9, 1.1)
    cbaa_period_s: float = 0.2             # T_b
    wait_max_s: float = 120.0              # traffic.yaml deadlock.wait_max_s — 넘으면 미해결 대기
    record: bool = False                   # 사건 기록 (결정성 시험 · 디버깅)

    def __post_init__(self):
        if not 0.0 <= self.loss < 1.0:
            raise ValueError('loss 는 [0, 1)')
        if self.fault not in FAULTS:
            raise ValueError(f'fault 는 {FAULTS} 중 하나')
        if self.dt <= 0 or self.horizon_s <= 0:
            raise ValueError('dt · horizon_s 는 양수')

    def key(self) -> str:
        """셀 키 (seed·rep 제외)."""
        return f'{self.arm.name}|loss={self.loss:g}|{self.fault}'


def derive(*parts) -> int:
    """(부분들) → 63 비트 정수 시드 (blake2b — 프로세스·파이썬 버전과 무관)."""
    text = '\x1f'.join(str(p) for p in parts).encode('utf-8')
    return int.from_bytes(hashlib.blake2b(text, digest_size=8).digest(), 'little') >> 1


def uniform(lo: float, hi: float, *parts) -> float:
    """(부분들) 에서 파생한 U(lo, hi)."""
    u = derive(*parts) / float(1 << 63)
    return lo + (hi - lo) * u


@dataclasses.dataclass(frozen=True)
class TaskArrival:
    """외생 작업 1 건."""

    t: float
    spec: TaskSpec
    pickup: str
    dropoff: str


def task_stream(cfg: RunConfig, docks: Tuple[str, ...],
                places: Dict[str, Tuple[float, float]]) -> List[TaskArrival]:
    """포아송 작업 열 — seed 로 정해지고 비교군·rep 와 무관하다."""
    rng = random.Random(derive(cfg.seed, 'tasks'))
    out: List[TaskArrival] = []
    t = 0.0
    k = 0
    while True:
        t += rng.expovariate(cfg.arrival_rate)
        if t >= cfg.horizon_s:
            break
        pick = rng.choice(docks)
        drop = rng.choice([d for d in docks if d != pick])
        item = rng.choice(sorted(ITEMS))
        prio = rng.choice(PRIORITIES)
        k += 1
        tid = f'task_{k:03d}'
        px, py = places[pick]
        dx, dy = places[drop]
        spec = TaskSpec(task_id=tid, pickup=Pose2D(px, py), dropoff=Pose2D(dx, dy),
                        item_type=item, item_mass=2.0, priority=prio, created=t)
        out.append(TaskArrival(t, spec, pick, drop))
    return out


def fault_victim(cfg: RunConfig, robot_ids: Tuple[str, ...]) -> Optional[str]:
    """F6 의 이탈 로봇 (seed 로 정해진다)."""
    if cfg.fault != 'F6':
        return None
    return robot_ids[derive(cfg.seed, 'victim') % len(robot_ids)]


def comm_seed(cfg: RunConfig) -> int:
    """통신 모의 루트 시드 = (seed, rep) 파생값 (0 이 아니게)."""
    return derive(cfg.seed, cfg.rep, 'comm') or 1


def dwell_s(cfg: RunConfig, task: TaskSpec, robot: str, stage: str) -> float:
    """도크 체류 시간 = (도킹 고정 + load_time) × 잡음 (seed, rep, 로봇, 작업, 단계 해시)."""
    base = cfg.dock_overhead_s + ITEMS.get(task.item_type, 5.0)
    return base * uniform(*cfg.dwell_noise, cfg.seed, cfg.rep, 'dwell', robot, task.task_id, stage)


def leg_speed(cfg: RunConfig, robot: str, task_id: str, stage: str, vmax: float) -> float:
    """구간 최고 속도 = vmax × 잡음."""
    return vmax * uniform(*cfg.speed_noise, cfg.seed, cfg.rep, 'speed', robot, task_id, stage)


def is_finite(x: float) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(x)


__all__ = ['ALLOC', 'ARMS', 'Arm', 'CBAA_ALLOC', 'CENTRAL_ALLOC', 'FAULTS', 'FAULT_DOC', 'ITEMS',
           'RunConfig', 'TRAFFIC', 'TaskArrival', 'comm_seed', 'derive', 'dwell_s',
           'fault_victim', 'leg_speed', 'task_stream', 'uniform']
