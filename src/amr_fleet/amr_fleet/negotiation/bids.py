"""
입찰과 방송 상태.

입찰은 **비용**(작을수록 좋다)으로 나타낸다. 원 논문(CBAA)은 점수 c = CMAX − 비용을 최대화하지만
같은 순서다. 비교는 (비용 오름차순, robot_id 사전순) 이다 — 동점은 id 가 작은 로봇이 이긴다.

특수값
- NO_BID     : (inf, None) — 아무도 이 작업을 쥐지 않았다
- LOCKED_COST: -inf — 작업 잠금을 얻어 실행을 시작한 로봇의 표시. 어떤 입찰로도 이길 수 없다(비선점)
"""

from __future__ import annotations

import dataclasses
import math
from typing import Dict, Mapping, Optional, Tuple

LOCKED_COST = -math.inf


@dataclasses.dataclass(frozen=True)
class Bid:
    """작업 1건에 대해 지금 알고 있는 승자 입찰."""

    cost: float = math.inf
    winner: Optional[str] = None

    @property
    def locked(self) -> bool:
        """작업 잠금을 얻어 실행 중인 표시인지."""
        return self.winner is not None and self.cost == LOCKED_COST

    @property
    def empty(self) -> bool:
        """아무도 쥐지 않은 상태인지."""
        return self.winner is None


NO_BID = Bid()


def better(a: Bid, b: Bid) -> bool:
    """
    입찰 a 가 입찰 b 보다 엄밀히 나은지 판정한다.

    비용이 작으면 낫다. 비용이 같으면 승자가 있는 쪽, 둘 다 있으면 robot_id 가 작은 쪽이 낫다.
    """
    if a.winner is None:
        return False
    if b.winner is None:
        return True
    if a.cost != b.cost:
        return a.cost < b.cost
    return a.winner < b.winner


@dataclasses.dataclass(frozen=True)
class StateMessage:
    """
    로봇이 주기마다 방송하는 **전체 상태** (바뀌지 않아도 다시 보낸다 — 멱등, 유실 자동 복구).

    bids      : 작업 ID → 이 로봇이 아는 승자 입찰
    info_time : 로봇 ID → 그 로봇의 정보를 마지막으로 (직접·간접) 받은 시각 (CBBA 의 s_k)
    healthy   : 위치추정 건강 플래그 (False 면 입찰하지 않고, 받는 쪽은 이 로봇의 낙찰을 무효화한다)
    assigned  : 이 로봇이 지금 쥔 작업 (없으면 None)
    executing : 작업 잠금을 얻어 실행을 시작했는지
    """

    sender: str
    stamp: float
    bids: Mapping[str, Bid]
    info_time: Mapping[str, float]
    healthy: bool = True
    assigned: Optional[str] = None
    executing: bool = False

    def locks(self) -> Dict[str, str]:
        """이 상태에 실린 잠금 목록 {작업 ID: 보유 로봇}."""
        return {t: b.winner for t, b in self.bids.items() if b.locked and b.winner}


def table_key(bids: Mapping[str, Bid]) -> Tuple[Tuple[str, float, Optional[str]], ...]:
    """입찰 표의 비교용 정규형 (안정 판정에 쓴다)."""
    return tuple((t, b.cost, b.winner) for t, b in sorted(bids.items()))


__all__ = ['Bid', 'LOCKED_COST', 'NO_BID', 'StateMessage', 'better', 'table_key']
