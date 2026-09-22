"""
작업 상태 머신 + 작업 로그 (명세 4.8 "작업 상태 관리·이벤트 발행·작업 로그").

    PENDING ──start──▶ IN_PROGRESS ──complete──▶ COMPLETED
       │                   │
       └──fail──▶ FAILED ◀─fail┘        (FAILED ──retry──▶ PENDING, 실패 횟수 ≤ max_retries 일 때만)

- 전이마다 TaskEvent 를 콜백으로 알린다 (노드가 /fleet/task_events 로 발행).
- 이동 거리는 로봇 자세를 적분한다 (update_pose): 진행 중 작업을 가진 로봇의 연속 자세 간 거리 합.
- 종료(COMPLETED/FAILED) 시 TaskLogWriter 가 logs/tasks_YYYYmmdd.csv 에 한 줄 쓴다:
      task_id, robot_id, start_time, end_time, distance_m, duration_s, result
  시각 칸은 노드 시계(use_sim_time 이면 sim 초), 파일 날짜는 기록 시점의 벽시계 날짜다.
- max_retries = 허용하는 재시도 횟수. 실패(fail) 횟수로 센다 (start 없이 PENDING 에서 실패해도 센다).
"""

from __future__ import annotations

import csv
import dataclasses
import datetime as _dt
import math
import os
import pathlib
import time
from typing import Callable, Dict, List, Optional, Tuple

from amr_fleet.task_schema import (
    STATUS_COMPLETED, STATUS_FAILED, STATUS_IN_PROGRESS, STATUS_NAMES, STATUS_PENDING, TaskSpec,
)

TERMINAL = frozenset({STATUS_COMPLETED, STATUS_FAILED})
TRANSITIONS: Dict[int, frozenset] = {
    STATUS_PENDING: frozenset({STATUS_IN_PROGRESS, STATUS_FAILED}),
    STATUS_IN_PROGRESS: frozenset({STATUS_COMPLETED, STATUS_FAILED}),
    STATUS_COMPLETED: frozenset(),
    STATUS_FAILED: frozenset({STATUS_PENDING}),   # retry 전용
}


class InvalidTransition(ValueError):
    """허용되지 않는 상태 전이."""


@dataclasses.dataclass
class TaskEvent:
    """상태 전이 1건."""

    task_id: str
    robot_id: str
    old_status: int
    new_status: int
    time: float
    reason: str = ''

    @property
    def name(self) -> str:
        """새 상태 이름 (PENDING/IN_PROGRESS/COMPLETED/FAILED)."""
        return STATUS_NAMES[self.new_status]


@dataclasses.dataclass
class TaskRecord:
    """작업 1건의 이력."""

    spec: TaskSpec
    status: int = STATUS_PENDING
    robot_id: str = ''
    created_time: float = 0.0
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    distance_m: float = 0.0
    result: str = ''
    attempts: int = 0                 # start 횟수
    failures: int = 0                 # fail 횟수 (재시도 한도 판정)
    deadline_missed: bool = False
    last_pose: Optional[Tuple[float, float]] = None
    pinned_robot_id: str = ''         # 요청에 지정된 로봇 (retry 시 복원)
    command_time: Optional[float] = None   # 수락된 assign_task 요청의 stamp (명령 시각)

    @property
    def task_id(self) -> str:
        """작업 식별자."""
        return self.spec.task_id

    @property
    def duration_s(self) -> Optional[float]:
        """시작~종료 [s]. 시작하지 않았거나 끝나지 않았으면 None."""
        if self.start_time is None or self.end_time is None:
            return None
        return self.end_time - self.start_time

    def is_terminal(self) -> bool:
        """COMPLETED 또는 FAILED 인지."""
        return self.status in TERMINAL


class TaskLogWriter:
    """
    logs/<prefix>_YYYYmmdd.csv 에 종료된 작업을 한 줄씩 추가한다 (날짜별 파일).

    파일 날짜는 기록 시점의 벽시계(wall_clock)로 정한다 — use_sim_time 이면 노드 시계가 수백 초
    부근이라 19700101 파일이 생기므로. 시각 칸은 노드 시계 값 그대로다.
    """

    HEADER = ['task_id', 'robot_id', 'start_time', 'end_time', 'distance_m', 'duration_s',
              'result']

    def __init__(self, log_dir: os.PathLike, prefix: str = 'tasks', time_format: str = 'iso',
                 wall_clock: Callable[[], float] = time.time):
        if time_format not in ('iso', 'epoch'):
            raise ValueError("time_format 은 'iso' 또는 'epoch'")
        self.log_dir = pathlib.Path(log_dir)
        self.prefix = prefix
        self.time_format = time_format
        self.wall_clock = wall_clock
        self.rows_written = 0

    def path_for(self, wall_t: Optional[float] = None) -> pathlib.Path:
        """벽시계 시각 wall_t(기본 지금)가 속한 날짜의 파일 경로."""
        t = self.wall_clock() if wall_t is None else wall_t
        day = _dt.datetime.fromtimestamp(t).strftime('%Y%m%d')
        return self.log_dir / f'{self.prefix}_{day}.csv'

    def format_time(self, t: Optional[float]) -> str:
        """None → 빈 칸, iso → 2026-09-22T10:00:00.123, epoch → 소수 3자리."""
        if t is None:
            return ''
        if self.time_format == 'epoch':
            return f'{t:.3f}'
        return _dt.datetime.fromtimestamp(t).isoformat(timespec='milliseconds')

    def row_for(self, rec: TaskRecord) -> List[str]:
        """`TaskRecord` → CSV 한 줄 (HEADER 순서)."""
        dur = rec.duration_s
        return [
            rec.task_id, rec.robot_id,
            self.format_time(rec.start_time), self.format_time(rec.end_time),
            f'{rec.distance_m:.3f}', '' if dur is None else f'{dur:.3f}', rec.result,
        ]

    def write(self, rec: TaskRecord) -> pathlib.Path:
        """종료 레코드를 추가한다. 파일이 새로 생기면 헤더를 먼저 쓴다."""
        if rec.end_time is None:
            raise ValueError('종료되지 않은 작업은 기록하지 않는다')
        path = self.path_for()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        new_file = not path.exists() or path.stat().st_size == 0
        with open(path, 'a', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(self.HEADER)
            w.writerow(self.row_for(rec))
        self.rows_written += 1
        return path


class TaskStateMachine:
    """모든 작업의 상태·이력을 보관하고 전이를 검증한다."""

    def __init__(self, on_event: Optional[Callable[[TaskEvent], None]] = None,
                 log_writer: Optional[TaskLogWriter] = None, max_retries: int = 0,
                 max_pose_step_m: float = 5.0,
                 on_log_error: Optional[Callable[[TaskRecord, OSError], None]] = None):
        self._records: Dict[str, TaskRecord] = {}
        self._on_event = on_event
        self._log = log_writer
        # 로그 쓰기 실패(OSError)는 전이를 되돌리지 않는다: 콜백이 있으면 알리고, 없으면 다시 던진다
        self._on_log_error = on_log_error
        self.max_retries = max(0, int(max_retries))
        # 위치 추정 점프(재초기화 등)로 인한 거짓 거리 누적을 막는 한 스텝 상한
        self.max_pose_step_m = float(max_pose_step_m)
        self.events: List[TaskEvent] = []

    # --- 조회 ---
    def __len__(self) -> int:
        return len(self._records)

    def __contains__(self, task_id: str) -> bool:
        return task_id in self._records

    def get(self, task_id: str) -> Optional[TaskRecord]:
        """레코드 조회 (없으면 None)."""
        return self._records.get(task_id)

    def records(self) -> List[TaskRecord]:
        """생성 순서의 전체 레코드."""
        return list(self._records.values())

    def with_status(self, status: int) -> List[TaskRecord]:
        """해당 상태의 레코드."""
        return [r for r in self._records.values() if r.status == status]

    def active_task_of(self, robot_id: str) -> Optional[TaskRecord]:
        """로봇이 진행 중인 작업 (없으면 None)."""
        for r in self._records.values():
            if r.status == STATUS_IN_PROGRESS and r.robot_id == robot_id:
                return r
        return None

    def counts(self) -> Dict[str, int]:
        """`FleetStatus` 의 tasks_* 카운터."""
        c = {'pending': 0, 'in_progress': 0, 'completed': 0, 'failed': 0}
        for r in self._records.values():
            c[STATUS_NAMES[r.status].lower()] += 1
        return c

    # --- 전이 ---
    def add(self, spec: TaskSpec, now: float) -> TaskRecord:
        """새 작업 등록 (PENDING). 중복 task_id 면 ValueError."""
        if spec.task_id in self._records:
            raise ValueError(f'중복 task_id: {spec.task_id}')
        spec.status = STATUS_PENDING
        rec = TaskRecord(spec=spec, robot_id=spec.robot_id, created_time=now,
                         pinned_robot_id=spec.robot_id)
        self._records[spec.task_id] = rec
        self._emit(rec, STATUS_PENDING, STATUS_PENDING, now, 'created')
        return rec

    def start(self, task_id: str, robot_id: str, now: float,
              command_time: Optional[float] = None) -> Optional[TaskEvent]:
        """
        PENDING → IN_PROGRESS. command_time = 수락된 assign_task 요청의 stamp (없으면 None).

        이미 같은 로봇으로 진행 중이면 None (멱등: 서비스 응답과 task_status 가 둘 다 알릴 수 있다).
        """
        rec = self._require(task_id)
        if rec.status == STATUS_IN_PROGRESS and rec.robot_id == robot_id:
            return None
        self._check(rec, STATUS_IN_PROGRESS)
        rec.robot_id = robot_id
        rec.spec.robot_id = robot_id
        rec.start_time = now
        rec.command_time = command_time
        rec.attempts += 1
        rec.last_pose = None
        return self._transition(rec, STATUS_IN_PROGRESS, now, 'assigned')

    def complete(self, task_id: str, now: float, result: str = 'completed') -> TaskEvent:
        """IN_PROGRESS → COMPLETED. 로그 기록."""
        rec = self._require(task_id)
        self._check(rec, STATUS_COMPLETED)
        rec.end_time = now
        rec.result = result
        ev = self._transition(rec, STATUS_COMPLETED, now, result)
        self._write_log(rec)
        return ev

    def fail(self, task_id: str, now: float, reason: str = 'failed') -> TaskEvent:
        """PENDING | IN_PROGRESS → FAILED. 로그 기록 (result = 'failed:<reason>')."""
        rec = self._require(task_id)
        self._check(rec, STATUS_FAILED)
        rec.end_time = now
        rec.failures += 1
        rec.result = f'failed:{reason}' if reason and reason != 'failed' else 'failed'
        ev = self._transition(rec, STATUS_FAILED, now, reason)
        self._write_log(rec)
        return ev

    def retry(self, task_id: str, now: float) -> Optional[TaskEvent]:
        """FAILED → PENDING (실패 횟수 <= max_retries 일 때만). 불가하면 None."""
        if not self.can_retry(task_id):
            return None
        rec = self._records[task_id]
        rec.robot_id = rec.spec.robot_id = rec.pinned_robot_id
        rec.start_time = rec.end_time = rec.command_time = None
        rec.distance_m = 0.0
        rec.result = ''
        rec.last_pose = None
        return self._transition(rec, STATUS_PENDING, now, 'retry')

    def can_retry(self, task_id: str) -> bool:
        """
        `retry` 가 가능한 상태인지: FAILED 이고 실패 횟수가 max_retries 이하.

        max_retries=0 이면 재시도하지 않는다. 1 이면 첫 실패 뒤 한 번만 다시 큐에 넣는다.
        """
        rec = self._records.get(task_id)
        return (rec is not None and rec.status == STATUS_FAILED
                and 0 < rec.failures <= self.max_retries)

    # --- 거리 적분 / 마감 ---
    def update_pose(self, robot_id: str, x: float, y: float) -> float:
        """로봇 자세 갱신. 진행 중 작업이 있으면 이동 거리에 스텝을 더하고 그 값을 돌려준다."""
        rec = self.active_task_of(robot_id)
        if rec is None:
            return 0.0
        step = 0.0
        if rec.last_pose is not None:
            step = math.hypot(x - rec.last_pose[0], y - rec.last_pose[1])
            if step > self.max_pose_step_m:
                step = 0.0
            rec.distance_m += step
        rec.last_pose = (x, y)
        return step

    def check_deadlines(self, now: float) -> List[TaskRecord]:
        """종료되지 않은 작업 중 마감을 새로 넘긴 것을 표시하고 돌려준다 (한 번만)."""
        newly: List[TaskRecord] = []
        for rec in self._records.values():
            if rec.is_terminal() or rec.deadline_missed or rec.spec.deadline is None:
                continue
            if now > rec.spec.deadline:
                rec.deadline_missed = True
                newly.append(rec)
        return newly

    # --- 내부 ---
    def _require(self, task_id: str) -> TaskRecord:
        try:
            return self._records[task_id]
        except KeyError:
            raise KeyError(f'모르는 task_id: {task_id}') from None

    @staticmethod
    def _check(rec: TaskRecord, new_status: int) -> None:
        if new_status not in TRANSITIONS[rec.status]:
            raise InvalidTransition(
                f'{rec.task_id}: {STATUS_NAMES[rec.status]} → {STATUS_NAMES[new_status]} 불가')

    def _transition(self, rec: TaskRecord, new_status: int, now: float, reason: str) -> TaskEvent:
        old = rec.status
        rec.status = new_status
        rec.spec.status = new_status
        return self._emit(rec, old, new_status, now, reason)

    def _emit(self, rec: TaskRecord, old: int, new: int, now: float, reason: str) -> TaskEvent:
        ev = TaskEvent(rec.task_id, rec.robot_id, old, new, now, reason)
        self.events.append(ev)
        if self._on_event is not None:
            self._on_event(ev)
        return ev

    def _write_log(self, rec: TaskRecord) -> None:
        if self._log is None:
            return
        try:
            self._log.write(rec)
        except OSError as exc:
            if self._on_log_error is None:
                raise
            self._on_log_error(rec, exc)
