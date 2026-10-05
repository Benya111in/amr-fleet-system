"""
TF 트리 유효성 검사 (rclpy 비의존) — 재생 중 관측한 (부모, 자식) 간선으로 판정한다.

유효 = (1) 모든 자식 프레임의 부모가 하나 (2) 순환 없음 (3) 지정한 프레임이 지정한 뿌리까지 이어짐.
SLAM 행을 잘못 엮으면(예: LIO-SAM 행에 bag 의 EKF /tf 를 같이 틀면 base_footprint 의 부모가 둘) 여기서 걸린다.
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Set, Tuple


def parents_of(edges: Iterable[Tuple[str, str]]) -> Dict[str, Set[str]]:
    """간선 (부모, 자식) → 자식별 부모 집합. 프레임 이름의 앞 '/' 는 뗀다 (tf2 규약)."""
    out: Dict[str, Set[str]] = {}
    for parent, child in edges:
        out.setdefault(child.lstrip('/'), set()).add(parent.lstrip('/'))
    return out


def chain(parents: Dict[str, Set[str]], frame: str, limit: int = 64) -> Tuple[List[str], str]:
    """
    프레임에서 뿌리까지 부모를 따라간다 → (경로, 상태).

    상태: 'ok' (부모 없는 뿌리에 닿음), 'multi_parent', 'cycle'. 부모가 여럿이면 그 자리에서 멈춘다.
    """
    path = [frame.lstrip('/')]
    seen = {path[0]}
    while len(path) <= limit:
        ps = parents.get(path[-1], set())
        if not ps:
            return path, 'ok'
        if len(ps) > 1:
            return path, 'multi_parent'
        nxt = next(iter(ps))
        if nxt in seen:
            return path + [nxt], 'cycle'
        path.append(nxt)
        seen.add(nxt)
    return path, 'cycle'


def check(edges: Iterable[Tuple[str, str]], frames: Iterable[str],
          root: Optional[str] = None) -> Dict[str, object]:
    """
    TF 트리 판정.

    frames 마다 뿌리까지의 경로를 내고, root 를 주면 그 뿌리에 닿는지 본다. valid = 부모 둘 이상인 자식 없음 ∧
    모든 frames 의 경로 상태 'ok' ∧ (root 지정 시) 뿌리 일치.
    """
    parents = parents_of(edges)
    multi = {c: sorted(p) for c, p in parents.items() if len(p) > 1}
    chains = {}
    ok = not multi
    for f in frames:
        path, status = chain(parents, f)
        reached = path[-1] if status == 'ok' else None
        good = status == 'ok' and (root is None or reached == root.lstrip('/'))
        chains[f] = {'path': path, 'status': status, 'root': reached, 'ok': good}
        ok = ok and good
    roots = sorted({p for ps in parents.values() for p in ps if p not in parents})
    return {'valid': ok, 'multi_parent': multi, 'chains': chains, 'roots': roots,
            'n_frames': len(set(parents) | {p for ps in parents.values() for p in ps})}
