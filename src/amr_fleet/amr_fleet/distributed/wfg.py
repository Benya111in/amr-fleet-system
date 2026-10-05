"""
복제 대기 그래프 (프로토콜 명세 9 절) — 서버 없이 각 로봇이 같은 교착 판정을 내린다.

각 로봇이 자기 out-edge(token · crossing · block, 대기 시작 시각)를 비컨(sim 2 Hz)에 싣는다. 모든 로봇이
같은 비컨 집합으로 같은 그래프를 만들고 같은 결정적 Tarjan(amr_fleet.deadlock 재사용 — 정렬된 순회)을
돌린다. 그래서 입력이 같으면 모든 복제가 같은 후보 · 같은 희생자를 고른다. 입력이 잠깐 어긋나면(지연)
판정도 잠깐 어긋날 수 있다 — ProgressConfirmer 의 confirm_s 창이 일시적 사이클을 거른다.

- token · crossing: AND 대기 (그 로봇이 비켜야만 풀린다) → 크기 ≥ 2 SCC 면 교착 후보
- block: Nav2 가 우회할 수도 있는 OR 대기 → block 간선이 낀 SCC 는 knot(SCC 밖으로 나가는 간선 없음)일 때만
- 희생자: 우선순위 낮은 순 → 처음 요청 시각이 늦은 순(어린 쪽) → robot_id 큰 순. 고정 timestamp 라 같은 쌍에서
  방향이 뒤집히지 않는다 (wait-die, Rosenkrantz 1978) — 희생자가 재요청해도 처음 요청 시각을 유지해야 한다.

"교착 없음" 은 주장하지 않는다. 사이클 없는 정체(라이브락)는 deadlock.StallDetector 가 맡는다.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, FrozenSet, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

from amr_fleet.deadlock import EDGE_BLOCK, strongly_connected_components, WaitForGraph
from amr_fleet.distributed.ordering import Stamp

Time = Union[Stamp, float, int]


@dataclasses.dataclass(frozen=True)
class OutEdge:
    """대기 간선 1 개: 이 로봇이 holder 때문에 서 있다."""

    holder: str
    reason: str


@dataclasses.dataclass(frozen=True)
class WaitInfo:
    """비컨에 싣는 대기 정보 (로봇 1 대)."""

    robot: str
    incarnation: int
    seq: int
    edges: Tuple[OutEdge, ...] = ()
    priority: int = 0
    request_stamp: Optional[Stamp] = None      # 이 대기의 처음 요청 시각 (재요청해도 유지)
    remaining_m: Optional[float] = None        # 남은 경로 길이 (진행 확인용)


class ReplicatedWfg:
    """비컨으로 모은 대기 정보 → 결정적 교착 후보."""

    def __init__(self, ttl_s: float = 2.0):
        self.ttl_s = float(ttl_s)
        self._info: Dict[str, Tuple[Stamp, WaitInfo]] = {}

    def update(self, info: WaitInfo, now: Time) -> bool:
        """비컨 1 개. 같은 로봇의 옛 비컨((incarnation, seq) 역행)은 무시."""
        now = Stamp.coerce(now)
        old = self._info.get(info.robot)
        if old is not None and (info.incarnation, info.seq) <= (old[1].incarnation, old[1].seq):
            return False
        self._info[info.robot] = (now, info)
        return True

    def fresh(self, now: Time) -> Dict[str, WaitInfo]:
        """신선한(ttl 안) 대기 정보."""
        now = Stamp.coerce(now)
        out = {}
        for r, (rx, info) in self._info.items():
            if rx <= now and (now.total_ns() - rx.total_ns()) / 1e9 <= self.ttl_s:
                out[r] = info
        return out

    def graph(self, now: Time) -> WaitForGraph:
        """신선한 비컨으로 만든 대기 그래프 (amr_fleet.deadlock.WaitForGraph)."""
        g = WaitForGraph()
        for r, info in sorted(self.fresh(now).items()):
            g.add_node(r)
            for e in sorted(info.edges, key=lambda e: (e.holder, e.reason)):
                g.add(r, e.holder, e.reason)
        return g

    def candidates(self, now: Time) -> List[List[str]]:
        """교착 후보 SCC (정렬). block 간선이 낀 SCC 는 knot 일 때만."""
        g = self.graph(now)
        return deadlock_candidates(g)


def deadlock_candidates(g: WaitForGraph) -> List[List[str]]:
    """크기 ≥ 2 SCC 중 AND 대기만으로 된 것, 또는 knot 인 것."""
    adj = g.adjacency()
    out = []
    for comp in strongly_connected_components(adj):
        if len(comp) < 2:
            continue
        members = set(comp)
        inner = [(s, d) for s in comp for d in adj.get(s, ()) if d in members]
        has_block = any(g.reason(s, d) == EDGE_BLOCK for s, d in inner)
        if has_block:
            leaves = any(d not in members for s in comp for d in adj.get(s, ()))
            if leaves:
                continue          # OR 대기: SCC 밖으로 나갈 길이 있다 — knot 이 아니다
        out.append(sorted(comp))
    return out


def victim_key(info: WaitInfo) -> Tuple[int, int, str]:
    """
    희생자 정렬 키 (클수록 먼저 희생): (-우선순위, 처음 요청 시각 ns, robot_id).

    요청 시각을 모르면 가장 늦은 것으로 본다 (가장 어린 쪽이 먼저 비킨다).
    """
    ts = info.request_stamp.total_ns() if info.request_stamp is not None else 1 << 62
    return (-int(info.priority), ts, info.robot)


def select_victim(members: Sequence[str], infos: Mapping[str, WaitInfo]) -> str:
    """SCC 안에서 비킬 로봇 1 대. 정보가 없는 구성원은 우선순위 0 · 시각 미상으로 본다."""
    def key(r: str):
        return victim_key(infos.get(r, WaitInfo(r, 0, 0)))
    return max(sorted(members), key=key)


class ProgressConfirmer:
    """후보 SCC 구성원 전원의 남은 경로 길이가 confirm_s 동안 eps_m 이상 줄지 않으면 확정 (한 번)."""

    def __init__(self, confirm_s: float = 1.0, eps_m: float = 0.05):
        self.confirm_s = float(confirm_s)
        self.eps_m = float(eps_m)
        self._since: Dict[FrozenSet[str], Stamp] = {}
        self._base: Dict[FrozenSet[str], Dict[str, float]] = {}
        self._reported: set = set()

    def update(self, now: Time, candidates: Iterable[Sequence[str]],
               remaining: Mapping[str, Optional[float]]) -> List[FrozenSet[str]]:
        """이번 주기의 후보와 남은 길이 → 새로 확정된 집합."""
        now = Stamp.coerce(now)
        cur = {frozenset(c) for c in candidates}
        for sig in list(self._since):
            if sig not in cur:
                del self._since[sig]
                del self._base[sig]
                self._reported.discard(sig)
        out = []
        for sig in sorted(cur, key=sorted):
            rem = {r: remaining.get(r) for r in sig}
            base = self._base.get(sig)
            moved = base is not None and any(
                rem[r] is not None and base.get(r) is not None and base[r] - rem[r] >= self.eps_m
                for r in sig)
            if base is None or moved:
                self._since[sig] = now
                self._base[sig] = dict(rem)
                self._reported.discard(sig)
                continue
            if sig in self._reported:
                continue
            if (now.total_ns() - self._since[sig].total_ns()) / 1e9 >= self.confirm_s:
                self._reported.add(sig)
                out.append(sig)
        return out
