"""
교착 탐지 (명세 4.9 "교착(Deadlock) 탐지 알고리즘"; sequences.md §3).

wait-for 그래프 W: 간선 i → j = "i 가 j 때문에 서 있다". 간선 이유 3가지
- token    : i 가 구역 토큰을 기다리는데 j 가 보유(또는 기아 예약) 중
- crossing : i 가 교차 충돌 예측으로 j 에게 양보 중
- block    : i 가 block_time 이상 정지해 있고, 정지한 j 의 몸체가 i 의 남은 경로 앞쪽
             (lookahead) 통로 폭 안에 있다 (지역 계획기로 못 비껴가는 물리적 막힘)
교착 = W 의 사이클 (Coffman 순환 대기). Tarjan SCC 로 O(V + E) 에 찾고, 크기 ≥ 2 인 SCC 가
confirm_s 이상 유지되면 확정한다 (통신 지연·재계획 순간의 일시적 사이클을 거른다).

사이클이 없는 정체(라이브락: 서로 비키다 제자리, 유휴 로봇이 목적지를 막음)는 StallDetector 가
"남은 경로 길이가 stall_time_s 동안 stall_progress_m 이상 줄지 않음" 으로 잡는다.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Dict, FrozenSet, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

EDGE_TOKEN = 'token'
EDGE_CROSSING = 'crossing'
EDGE_BLOCK = 'block'
EDGE_REASONS = (EDGE_TOKEN, EDGE_CROSSING, EDGE_BLOCK)

# 교착 유형 (docs/algorithms/deadlock.md 1장)
DL_HEAD_ON = 'HEAD_ON'        # T1 좁은 통로 정면 대치 (2대, 진행 방향 반대)
DL_CIRCULAR = 'CIRCULAR'      # T2 교차로 등 3대 이상 순환 대기
DL_BLOCKED = 'BLOCKED'        # T3 유휴·주차 로봇이 경로/구역을 막음 (사이클 없음)
DL_MUTUAL = 'MUTUAL'          # 그 밖의 2대 상호 대기 (교차 등)
DL_LIVELOCK = 'LIVELOCK'      # T5 움직이지만 진전 없음 (정체 탐지)


class WaitForGraph:
    """wait-for 그래프 (간선마다 이유 1개, 같은 쌍은 먼저 넣은 이유 유지)."""

    def __init__(self):
        self._succ: Dict[str, Dict[str, str]] = {}

    def add_node(self, node: str) -> None:
        """간선 없는 노드도 그래프에 둔다."""
        self._succ.setdefault(node, {})

    def add(self, src: str, dst: str, reason: str) -> None:
        """간선 src → dst 추가 (src 가 dst 를 기다린다). 자기 간선은 무시."""
        if reason not in EDGE_REASONS:
            raise ValueError(f'간선 이유는 {EDGE_REASONS} 중 하나: {reason!r}')
        self.add_node(dst)
        if src != dst:
            self._succ.setdefault(src, {}).setdefault(dst, reason)

    def nodes(self) -> List[str]:
        """노드 (정렬)."""
        return sorted(self._succ)

    def successors(self, node: str) -> List[str]:
        """해당 노드가 기다리는 로봇 (정렬)."""
        return sorted(self._succ.get(node, {}))

    def reason(self, src: str, dst: str) -> Optional[str]:
        """간선 이유 (없으면 None)."""
        return self._succ.get(src, {}).get(dst)

    def edges(self) -> List[Tuple[str, str, str]]:
        """(src, dst, reason) 목록 (정렬)."""
        return sorted((s, d, r) for s, ds in self._succ.items() for d, r in ds.items())

    def predecessors_closure(self, targets: Iterable[str], reason: Optional[str] = None
                             ) -> Set[str]:
        """주어진 노드(targets)에 (reason 간선만 따라) 이르는 노드. targets 자신은 간선으로 다시 닿을 때만."""
        pred: Dict[str, List[str]] = {}
        for s, ds in self._succ.items():
            for d, r in ds.items():
                if reason is None or r == reason:
                    pred.setdefault(d, []).append(s)
        out: Set[str] = set()
        stack = list(targets)
        while stack:
            for p in pred.get(stack.pop(), ()):
                if p not in out:
                    out.add(p)
                    stack.append(p)
        return out

    def adjacency(self) -> Dict[str, List[str]]:
        """노드 → 후속 목록."""
        return {n: self.successors(n) for n in self.nodes()}

    def __len__(self) -> int:
        return len(self._succ)


def strongly_connected_components(adj: Mapping[str, Iterable[str]]) -> List[List[str]]:
    """
    Tarjan SCC (반복 구현, 재귀 깊이 제한 없음). 각 성분은 정렬, 목록은 첫 원소 순.

    O(V + E). 크기 1 인 성분도 돌려준다 (자기 간선은 WaitForGraph 가 넣지 않는다).
    """
    succ = {n: sorted(set(vs)) for n, vs in adj.items()}
    for vs in list(succ.values()):
        for v in vs:
            succ.setdefault(v, [])
    index: Dict[str, int] = {}
    low: Dict[str, int] = {}
    on_stack: Set[str] = set()
    stack: List[str] = []
    out: List[List[str]] = []
    counter = 0
    for root in sorted(succ):
        if root in index:
            continue
        work = [(root, 0)]
        while work:
            node, i = work.pop()
            if i == 0:
                index[node] = low[node] = counter
                counter += 1
                stack.append(node)
                on_stack.add(node)
            recurse = False
            children = succ[node]
            while i < len(children):
                child = children[i]
                i += 1
                if child not in index:
                    work.append((node, i))
                    work.append((child, 0))
                    recurse = True
                    break
                if child in on_stack:
                    low[node] = min(low[node], index[child])
            if recurse:
                continue
            if low[node] == index[node]:
                comp = []
                while True:
                    w = stack.pop()
                    on_stack.discard(w)
                    comp.append(w)
                    if w == node:
                        break
                out.append(sorted(comp))
            if work:
                parent = work[-1][0]
                low[parent] = min(low[parent], low[node])
    out.sort(key=lambda c: c[0])
    return out


def find_cycle(adj: Mapping[str, Iterable[str]], component: Sequence[str]) -> List[str]:
    """
    SCC 안의 단순 사이클 1개 (가장 작은 id 에서 시작해 DFS). 크기 1 이면 [].

    방문 표시를 되돌리지 않는 DFS 라 O(V + E): SCC 에서는 start 로 들어오는 간선을 가진 노드를
    방문할 때 그 노드가 스택 위에 있으므로, 그때의 스택이 곧 start 로 돌아오는 단순 사이클이다.
    """
    members = set(component)
    if len(members) < 2:
        return []
    start = min(members)
    path = [start]
    seen = {start}
    iters = [iter(sorted(v for v in adj.get(start, ()) if v in members))]
    while iters:
        nxt = next(iters[-1], None)
        if nxt is None:
            iters.pop()
            path.pop()
            continue
        if nxt == start:
            return list(path)
        if nxt not in seen:
            path.append(nxt)
            seen.add(nxt)
            iters.append(iter(sorted(v for v in adj.get(nxt, ()) if v in members)))
    return []   # SCC 가 맞다면 도달하지 않는다


def deadlock_components(graph: WaitForGraph) -> List[List[str]]:
    """크기 ≥ 2 인 SCC (교착 후보)."""
    return [c for c in strongly_connected_components(graph.adjacency()) if len(c) >= 2]


class CycleConfirmer:
    """같은 로봇 집합의 사이클이 confirm_s 이상 이어지면 한 번 확정한다."""

    def __init__(self, confirm_s: float = 2.0):
        self.confirm_s = float(confirm_s)
        self._first_seen: Dict[FrozenSet[str], float] = {}
        self._reported: Set[FrozenSet[str]] = set()

    def update(self, now: float, components: Iterable[Iterable[str]]) -> List[FrozenSet[str]]:
        """이번 주기의 후보 → 새로 확정된 집합 (사라진 후보는 기록을 지운다)."""
        current = {frozenset(c) for c in components}
        for sig in list(self._first_seen):
            if sig not in current:
                del self._first_seen[sig]
                self._reported.discard(sig)
        confirmed = []
        for sig in sorted(current, key=sorted):
            first = self._first_seen.setdefault(sig, now)
            if sig not in self._reported and now - first >= self.confirm_s:
                self._reported.add(sig)
                confirmed.append(sig)
        return confirmed

    def pending(self, now: float) -> Dict[FrozenSet[str], float]:
        """확정 전 후보와 지속 시간."""
        return {s: now - t for s, t in self._first_seen.items() if s not in self._reported}

    def reported(self) -> Set[FrozenSet[str]]:
        """확정(보고)했고 아직 이어지는 사이클."""
        return set(self._reported)


@dataclasses.dataclass
class _Progress:
    goal: Tuple[float, float]
    anchor: float          # 마지막 진전 시점의 남은 길이
    anchor_time: float
    best: float            # 지금까지 최소 남은 길이
    stalled: bool = False


class StallDetector:
    """
    진전 감시: 남은 경로 길이가 stall_time_s 동안 progress_m 이상 줄지 않으면 정체.

    남은 길이가 지금까지 최솟값보다 detour_reset_m 넘게 늘면(큰 우회 경로로 재계획) 새 경로로 보고
    기준을 다시 잡는다. 목표가 goal_tol_m 넘게 바뀌어도 다시 잡는다.
    """

    def __init__(self, stall_time_s: float = 15.0, progress_m: float = 0.5,
                 detour_reset_m: float = 2.0, goal_tol_m: float = 0.5):
        self.stall_time_s = float(stall_time_s)
        self.progress_m = float(progress_m)
        self.detour_reset_m = float(detour_reset_m)
        self.goal_tol_m = float(goal_tol_m)
        self._p: Dict[str, _Progress] = {}

    def reset(self, robot_id: str) -> None:
        """기록 삭제 (해소 직후 등)."""
        self._p.pop(robot_id, None)

    def update(self, robot_id: str, now: float, remaining_m: Optional[float],
               goal: Optional[Tuple[float, float]], watch: bool = True) -> bool:
        """이번 관측을 반영하고 정체 여부를 돌려준다. watch=False(대기 명령 중 등)면 기준 재설정."""
        if remaining_m is None or goal is None or not watch:
            self._p.pop(robot_id, None)
            return False
        p = self._p.get(robot_id)
        if p is None or math.hypot(goal[0] - p.goal[0], goal[1] - p.goal[1]) > self.goal_tol_m:
            self._p[robot_id] = _Progress(goal, remaining_m, now, remaining_m)
            return False
        if remaining_m > p.best + self.detour_reset_m:        # 큰 우회 = 새 경로
            p.anchor, p.anchor_time, p.best = remaining_m, now, remaining_m
        elif remaining_m < p.anchor - self.progress_m:        # 진전
            p.anchor, p.anchor_time = remaining_m, now
        p.best = min(p.best, remaining_m)
        p.stalled = now - p.anchor_time >= self.stall_time_s
        return p.stalled

    def stalled_for(self, robot_id: str, now: float) -> float:
        """마지막 진전 이후 경과 [s] (감시 안 하면 0)."""
        p = self._p.get(robot_id)
        return 0.0 if p is None else now - p.anchor_time

    def anchor_time(self, robot_id: str) -> Optional[float]:
        """마지막 진전 시각 (감시 안 하면 None). 바뀌면 진전이 있었다는 뜻 — 정체 에피소드 경계."""
        p = self._p.get(robot_id)
        return None if p is None else p.anchor_time


def classify_deadlock(members: Sequence[str], headings: Mapping[str, float],
                      head_on_deg: float = 135.0) -> str:
    """사이클 유형: 3대 이상 CIRCULAR, 2대면 진행 방향이 반대(≥ head_on_deg)면 HEAD_ON, 아니면 MUTUAL."""
    if len(members) >= 3:
        return DL_CIRCULAR
    a, b = members[0], members[1]
    if a in headings and b in headings:
        d = abs((headings[a] - headings[b] + math.pi) % (2.0 * math.pi) - math.pi)
        if math.degrees(d) >= head_on_deg:
            return DL_HEAD_ON
    return DL_MUTUAL
