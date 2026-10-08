"""
작업 게시판(/fleet/task_board) 규칙 · 전역 task_id · 권위자 epoch (보너스 4 P0, rclpy 비의존).

근거: 보고서 「분산 플릿 관리 협상 할당과 폴백」 135–178행 — 작업 JSON 이 fleet_manager 로만 들어오면 서버가
죽은 동안 들어온 작업이 협상 전에 사라진다. 그래서 작업 소스가 /fleet/task_board (RELIABLE + TRANSIENT_LOCAL,
깊이 ≥ 미처리 상한) 에도 내고, 전역 고유 task_id 로 재전송해도 한 번만 처리한다. 설계: p0.md §4.

게시물 = amr_msgs/Task 한 건 (task_id 필수, version = 같은 작업 내용의 판).
수신 규칙 (VDA 5050 orderId/orderUpdateId 의 멱등 규칙을 빌렸다) — TaskLedger.check:
- 처음 보는 task_id                       → ACCEPT  (접수)
- 같은 task_id · 같은 version · 같은 내용  → DUPLICATE (조용히 무시: 재전송 · 두 경로로 같은 작업)
- 같은 task_id · 같은 version · 다른 내용  → CONFLICT (거절 + 알림: 소스가 id 를 재사용했다)
- 같은 task_id · 낮은 version             → STALE   (거절: 늦게 온 옛 판)
- 같은 task_id · 높은 version             → UPDATE  (P0 에서는 접수한 작업을 바꾸지 않는다 → 거절 + 알림)
"내용" 은 픽업 · 하역 자세, 물품 유형, 우선순위, 고정 로봇이다. 마감과 접수 시각은 넣지 않는다 — JSON 경로는
상대 마감을 도착 시각에 절대 시각으로 바꾸므로 같은 작업도 경로 · 도착 시각마다 값이 다르다.

epoch: 권위자(리더)가 내는 명령마다 (epoch, leader_id, seq) 를 싣는다 (Raft term · Chubby lock generation 과 같은
펜싱). resolve_epoch 는 파라미터 > 파일(시작마다 +1 해 영속) > 1 순으로 정한다. 받는 쪽 펜싱(낮은 epoch 거부)은
분산 에이전트(P3)의 몫이다 — 0 은 "펜싱 없음" 이라 옛 구성과 섞여도 거절하지 않는다.
"""

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import tempfile
from typing import Dict, Optional, Tuple
import uuid

from amr_fleet.task_schema import ROBOT_ID_RE, TASK_ID_RE, Pose2D, TaskSpec

BOARD_TOPIC = '/fleet/task_board'
BOARD_DEPTH = 256                 # TRANSIENT_LOCAL 이력 깊이 (미처리 작업 상한 이상)

ACCEPT = 'accept'
DUPLICATE = 'duplicate'
CONFLICT = 'conflict'
STALE = 'stale'
UPDATE = 'update'

_POSE_TOL = 1e-6                  # 자세 비교 허용오차 (JSON ↔ 메시지 쿼터니언 왕복 오차를 덮는다)


def make_task_id(source: str, seq: Optional[int] = None) -> str:
    """
    작업 소스가 게시 전에 붙이는 전역 고유 task_id.

    seq 를 주면 '<source>.<seq 6 자리>' (소스가 순번을 영속할 때), 아니면 '<source>.<uuid4 16 hex>' (64 비트).
    task_id 형식([A-Za-z0-9_.:-]{1,64})을 지킨다.
    """
    if not ROBOT_ID_RE.match(source or '') or not source:
        raise ValueError(f'source 는 [A-Za-z0-9_]+ 이어야 한다: {source!r}')
    suffix = f'{int(seq):06d}' if seq is not None else uuid.uuid4().hex[:16]
    task_id = f'{source}.{suffix}'
    if not TASK_ID_RE.match(task_id):
        raise ValueError(f'task_id 가 형식을 벗어난다: {task_id!r}')
    return task_id


@dataclasses.dataclass(frozen=True)
class TaskContent:
    """멱등 비교에 쓰는 작업 내용 (마감 · 접수 시각 제외)."""

    pickup: Tuple[float, float, float, str]
    dropoff: Tuple[float, float, float, str]
    item_type: str
    priority: int
    robot_id: str

    @staticmethod
    def of(spec: TaskSpec) -> 'TaskContent':
        """작업 명세(TaskSpec) → 내용."""
        def pose(p: Pose2D) -> Tuple[float, float, float, str]:
            return (float(p.x), float(p.y), math.remainder(float(p.yaw), math.tau), p.frame_id)
        return TaskContent(pose(spec.pickup), pose(spec.dropoff), spec.item_type,
                           int(spec.priority), spec.robot_id or '')

    def same_as(self, other: 'TaskContent') -> bool:
        """허용오차 안에서 같은 내용인가."""
        def close(a, b) -> bool:
            return (a[3] == b[3] and all(abs(x - y) <= _POSE_TOL for x, y in zip(a[:2], b[:2]))
                    and abs(math.remainder(a[2] - b[2], math.tau)) <= _POSE_TOL)
        return (close(self.pickup, other.pickup) and close(self.dropoff, other.dropoff)
                and (self.item_type, self.priority, self.robot_id)
                == (other.item_type, other.priority, other.robot_id))


class TaskLedger:
    """접수한 작업의 (version, 내용) 장부 — 두 경로(task_request · task_board)가 함께 쓴다."""

    def __init__(self):
        self._entries: Dict[str, Tuple[int, TaskContent]] = {}

    def __contains__(self, task_id: str) -> bool:
        return task_id in self._entries

    def __len__(self) -> int:
        return len(self._entries)

    def check(self, spec: TaskSpec, version: int = 0) -> str:
        """이 게시물을 어떻게 다룰지 (모듈 설명의 규칙). 장부는 바꾸지 않는다."""
        entry = self._entries.get(spec.task_id)
        if entry is None:
            return ACCEPT
        have_version, have = entry
        if version < have_version:
            return STALE
        if version > have_version:
            return UPDATE
        return DUPLICATE if have.same_as(TaskContent.of(spec)) else CONFLICT

    def record(self, spec: TaskSpec, version: int = 0) -> None:
        """접수한 작업을 적는다."""
        self._entries[spec.task_id] = (int(version), TaskContent.of(spec))


def resolve_epoch(param: int = 0, path: str = '') -> int:
    """
    이 권위자의 epoch.

    param > 0 이면 그 값. 아니면 path 가 있으면 파일의 마지막 epoch + 1 을 원자적으로 써서(fsync) 쓴다 — 재시작한
    관리자가 이전 인스턴스보다 높은 epoch 로 명령한다. 둘 다 없으면 1.
    """
    param = int(param)
    if param < 0:
        raise ValueError(f'epoch 는 0 이상이어야 한다: {param}')
    if param > 0:
        return param
    if not path:
        return 1
    p = pathlib.Path(path)
    last = 0
    if p.exists():
        text = p.read_text(encoding='utf-8').strip()
        last = int(text) if text else 0
        if last < 0:
            raise ValueError(f'epoch 파일 값이 음수다: {path}')
    epoch = last + 1
    write_epoch_file(path, epoch)
    return epoch


def write_epoch_file(path: str, epoch: int) -> None:
    """권위자 epoch 파일을 원자적으로 쓴다 (임시 파일 → fsync → rename). 관찰자 복귀로 올렸을 때도."""
    epoch = int(epoch)
    if epoch < 1:
        raise ValueError(f'epoch 는 1 이상이어야 한다: {epoch}')
    p = pathlib.Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.epoch_', dir=str(p.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(f'{epoch}\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, p)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


class CommandStamp:
    """권위자 명령 표식 (epoch, leader_id, seq). seq 는 1 부터 명령마다 하나씩."""

    def __init__(self, epoch: int, leader_id: str):
        self.epoch = int(epoch)
        self.leader_id = str(leader_id)
        self.seq = 0

    def next(self) -> Tuple[int, str, int]:
        """다음 명령의 (epoch, leader_id, seq)."""
        self.seq += 1
        return self.epoch, self.leader_id, self.seq
