"""
모드 상태기 (보고서 「분산 플릿 관리 협상 할당과 폴백」 145–158행, p3.md §3) — rclpy 비의존.

모드 다섯: NORMAL · HOLD_NEW · DEGRADED · ISOLATED · RESYNC. 진실의 원천은 로봇이다 — 각 로봇이 자기 관측
(서버 하트비트 · 비컨의 과반 의심 · 연결 과반)으로 모드를 정한다. 모드는 **할당 방식(효율)** 만 바꾼다.
구역 · 작업 ID 의 안전은 정족수 투표가 지고 모드와 무관하다 (오탐해도 안전은 그대로).

기본 모드(base)는 서버 관측으로만 정하고, ISOLATED 는 그 위에 덮는 표시다 (연결이 돌아오면 base 로 복귀):

| base     | 조건                                                    | 다음      |
|----------|---------------------------------------------------------|-----------|
| NORMAL   | 과반이 서버 DEAD 에 동의                                   | DEGRADED  |
| NORMAL   | 내 감시가 SUSPECT/DEAD (또는 부팅 뒤 dead_s 동안 무소식) | HOLD_NEW  |
| HOLD_NEW | 과반 DEAD 동의                                            | DEGRADED  |
| HOLD_NEW | 내 감시가 ALIVE                                           | NORMAL    |
| DEGRADED | 내 감시가 ALIVE (펜스를 통과한 하트비트)                   | RESYNC    |
| RESYNC   | 내 감시가 ALIVE 아님                                      | DEGRADED  |
| RESYNC   | ALIVE 가 resync_s 동안 이어지고 서버가 ACTIVE · 과반 DEAD 아님 | NORMAL |

effective = ISOLATED (연결 과반을 isolate_s 동안 못 봄, 부팅 뒤 boot_grace_s 는 봐준다) 아니면 base.
RESYNC 의 히스테리시스는 서버 깜빡임에 따른 할당 방식 왕복(flapping)을 막는다 — 보고서 sim 30 s.
"""

from __future__ import annotations

import dataclasses
from typing import List, Optional, Tuple

from amr_fleet.distributed.failure_detector import ALIVE, DEAD, SUSPECT, UNKNOWN

NORMAL = 'NORMAL'
HOLD_NEW = 'HOLD_NEW'
DEGRADED = 'DEGRADED'
ISOLATED = 'ISOLATED'
RESYNC = 'RESYNC'
MODES = (NORMAL, HOLD_NEW, DEGRADED, ISOLATED, RESYNC)
# amr_msgs/AgentState.MODE_* 와 같은 값
MODE_CODES = {NORMAL: 0, HOLD_NEW: 1, DEGRADED: 2, ISOLATED: 3, RESYNC: 4}
SERVER_CODES = {UNKNOWN: 0, ALIVE: 1, SUSPECT: 2, DEAD: 3}


@dataclasses.dataclass(frozen=True)
class ModeConfig:
    """모드 파라미터 (sim 초)."""

    resync_s: float = 30.0          # RESYNC → NORMAL 히스테리시스 (보고서 sim 30 s)
    isolate_s: float = 2.0          # 연결 과반을 이만큼 못 보면 ISOLATED
    boot_grace_s: float = 3.0       # 부팅 직후 비컨이 모이기 전에는 ISOLATED 를 내지 않는다
    unknown_dead_s: float = 5.0     # 하트비트를 한 번도 못 받은 채 이만큼 지나면 서버 DEAD 로 본다

    def __post_init__(self):
        for name in ('resync_s', 'isolate_s', 'boot_grace_s', 'unknown_dead_s'):
            if not getattr(self, name) >= 0.0:
                raise ValueError(f'{name} 는 0 이상이어야 한다')


class ModeMachine:
    """로봇 1 대의 모드. update() 를 주기마다 부른다. 전이는 transitions 에 (시각, 이전, 다음, 이유)로 남는다."""

    def __init__(self, cfg: ModeConfig, now: float):
        self.cfg = cfg
        self.boot = float(now)
        self.base = NORMAL
        self.isolated = False
        self.reason = 'boot'
        self._alive_since: Optional[float] = None
        self._disconnected_since: Optional[float] = None
        self.transitions: List[Tuple[float, str, str, str]] = []
        self._last = NORMAL

    @property
    def mode(self) -> str:
        """지금 모드 (ISOLATED 가 base 를 덮는다)."""
        return ISOLATED if self.isolated else self.base

    def server_dead(self, status: str, now: float) -> bool:
        """비컨에 실을 '서버 DEAD' 판단: 감시가 DEAD, 또는 한 번도 못 받은 채 unknown_dead_s 경과."""
        if status == DEAD:
            return True
        return status == UNKNOWN and now - self.boot >= self.cfg.unknown_dead_s

    def update(self, now: float, status: str, majority_dead: bool, connected: bool,
               server_active: bool = True) -> str:
        """
        관측으로 모드를 갱신한다.

        status        : 내 하트비트 감시 (UNKNOWN · ALIVE · SUSPECT · DEAD) — 펜스를 통과한 하트비트만 센다
        majority_dead : 신선한 비컨 기준 정적 과반이 서버 DEAD (MajoritySuspicion.degraded)
        connected     : 신선한 비컨(자기 포함)이 정적 과반 이상 (MajoritySuspicion.connected_majority)
        server_active : 마지막 하트비트의 역할이 ACTIVE (관찰 중인 서버로는 NORMAL 로 돌아가지 않는다)
        """
        now = float(now)
        if status == ALIVE:
            if self._alive_since is None:
                self._alive_since = now
        else:
            self._alive_since = None
        quiet = status in (SUSPECT, DEAD) or self.server_dead(status, now)
        b = self.base
        if b == NORMAL:
            if majority_dead:
                self._set_base(DEGRADED, now, '과반 서버 DEAD')
            elif quiet:
                self._set_base(HOLD_NEW, now, f'서버 {status}')
        elif b == HOLD_NEW:
            if majority_dead:
                self._set_base(DEGRADED, now, '과반 서버 DEAD')
            elif status == ALIVE:
                self._set_base(NORMAL, now, '서버 ALIVE')
        elif b == DEGRADED:
            if status == ALIVE:
                self._set_base(RESYNC, now, '서버 하트비트 복귀')
        elif b == RESYNC:
            if status != ALIVE:
                self._set_base(DEGRADED, now, f'복귀 중 서버 {status}')
            elif (server_active and not majority_dead and self._alive_since is not None
                  and now - self._alive_since >= self.cfg.resync_s):
                self._set_base(NORMAL, now, f'서버 {self.cfg.resync_s:g} s 안정')
        # ISOLATED 덮개
        if connected:
            self._disconnected_since = None
            self.isolated = False
        else:
            if self._disconnected_since is None:
                self._disconnected_since = now
            if (now - self.boot >= self.cfg.boot_grace_s
                    and now - self._disconnected_since >= self.cfg.isolate_s):
                self.isolated = True
        if self.mode != self._last:
            why = '연결 과반 상실' if self.mode == ISOLATED else self.reason
            if self._last == ISOLATED and self.mode != ISOLATED:
                why = '연결 과반 회복'
            self.transitions.append((now, self._last, self.mode, why))
            self._last = self.mode
        return self.mode

    def _set_base(self, mode: str, now: float, reason: str) -> None:
        self.base = mode
        self.reason = reason
        if mode == RESYNC:
            self._alive_since = now if self._alive_since is None else self._alive_since

    # ------------------------------------------------------------------ 모드가 허락하는 것
    def accepts_central_tasks(self) -> bool:
        """중앙 assign_task 를 받는가 — NORMAL 에서만 (HOLD_NEW · RESYNC 는 새 작업 보류)."""
        return self.mode == NORMAL

    def negotiates(self) -> bool:
        """CBAA 협상으로 새 작업을 잡는가 — DEGRADED 에서만."""
        return self.mode == DEGRADED

    def allows_new_entry(self) -> bool:
        """새 구역 진입 요청을 받는가 — ISOLATED 만 아니면 (어차피 과반 없이는 허가가 안 난다)."""
        return self.mode != ISOLATED
