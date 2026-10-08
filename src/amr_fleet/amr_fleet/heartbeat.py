"""
관리자 하트비트 (보너스 4, p3.md §7 요청 1 — b4-wire) — rclpy 비의존.

fleet_manager_node 가 같은 프로세스 · 같은 실행기에서 /fleet/heartbeat 를 sim 주기(기본 2 Hz)로 낸다. 하트비트가
서버의 생존을 뜻하려면 서버 프로세스에서 나와야 한다 (P3 의 manager_shim 은 시험용 대역이었다). 관리자의 콜백이
막히면 하트비트도 늦는다 — 보고서의 "애플리케이션 하트비트" 의미 그대로다.

관찰자 복귀 (보고서 145–158행, p3.md §3): observe_s > 0 이면 기동 직후 ROLE_OBSERVER 로 시작해 /fleet_p2p/beacon 의
fence_epoch(로봇이 본 최고 권위자 epoch) 최댓값을 모은다. observe_s 가 지나면 epoch = max(기본 epoch, 최댓값 + 1)
로 ACTIVE 가 된다 — 그 전에는 명령(할당)하지 않는다. 기본 epoch 은 resolve_epoch(파라미터 > epoch_file + 1 > 1).
- 관찰 중에도 후보 epoch(최댓값 + 1)를 싣는다. 로봇은 관찰자 하트비트로 펜스를 올리지 않는다 (되먹임 방지 —
  에이전트 core.on_heartbeat 의 _fence_peek). 그래서 관찰 중 후보가 커져도 로봇 펜스는 그대로다.
- seq 는 관찰 → ACTIVE 에서 이어 간다. 같은 epoch 안에서 seq 가 되돌면 로봇 감시가 그 하트비트를 버린다
  (P3 smoke1 의 둘째 결함).
- ACTIVE 뒤에 비컨이 내 epoch 보다 높은 fence 를 보이면 superseded() — 다른(또는 옛 저장소의) 권위자를 로봇이
  봤다는 뜻이고 내 명령은 펜스에 거부된다. 스스로 epoch 를 올리지는 않는다 (관리자 둘이 서로 올리는 경쟁을
  만들지 않으려고) — 노드가 경보를 낸다.
- 시각은 호출자가 주는 노드 시계(sim) 초. 0 이하(/clock 전)에는 아무것도 내지 않고 관찰도 시작하지 않는다.
"""

from __future__ import annotations

import dataclasses
from typing import Optional

ROLE_ACTIVE = 0          # amr_msgs/FleetHeartbeat.ROLE_*
ROLE_OBSERVER = 1


@dataclasses.dataclass(frozen=True)
class Beat:
    """내보낼 하트비트 한 번."""

    epoch: int
    seq: int
    role: int
    activated: bool = False     # 이번 박동에서 관찰 → ACTIVE 로 바뀌었다


class ManagerHeartbeat:
    """관리자 하트비트 상태기."""

    def __init__(self, base_epoch: int, observe_s: float = 0.0):
        base_epoch = int(base_epoch)
        observe_s = float(observe_s)
        if base_epoch < 1:
            raise ValueError(f'기본 epoch 는 1 이상: {base_epoch}')
        if not observe_s >= 0.0:
            raise ValueError(f'observe_s 는 0 이상: {observe_s}')
        self.base_epoch = base_epoch
        self.observe_s = observe_s
        self.role = ROLE_OBSERVER if observe_s > 0.0 else ROLE_ACTIVE
        self.epoch = base_epoch
        self.max_seen = 0
        self.seq = 0
        self.started: Optional[float] = None
        self.activated_at: Optional[float] = None

    @property
    def active(self) -> bool:
        """명령해도 되는가 (관찰이 끝났다)."""
        return self.role == ROLE_ACTIVE

    def on_beacon(self, fence_epoch: int) -> None:
        """로봇 비컨의 fence_epoch (로봇이 본 최고 권위자 epoch)."""
        self.max_seen = max(self.max_seen, int(fence_epoch))

    def superseded(self) -> bool:
        """ACTIVE 인데 로봇이 내 epoch 보다 높은 권위자를 봤다 (내 명령은 펜스에 거부된다)."""
        return self.active and self.max_seen > self.epoch

    def tick(self, now: float) -> Optional[Beat]:
        """주기마다 부른다. 내보낼 하트비트 (시각 0 이하면 None)."""
        now = float(now)
        if now <= 0.0:
            return None
        if self.started is None:
            self.started = now
        activated = False
        if self.role == ROLE_OBSERVER:
            self.epoch = max(self.base_epoch, self.max_seen + 1)
            if now - self.started >= self.observe_s:
                self.role = ROLE_ACTIVE
                self.activated_at = now
                activated = True
        self.seq += 1
        return Beat(self.epoch, self.seq, self.role, activated)
