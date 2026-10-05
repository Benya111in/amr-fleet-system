"""
수신자 병합 규칙 — 로봇 i 가 로봇 k 의 방송 상태를 받았을 때 작업 j 의 입찰을 어떻게 고치는가.

- 'max'  : CBAA 원 알고리즘(Choi·Brunet·How 2009, Alg. 2)의 최대 합의. 더 나은 입찰이면 받아들인다.
           정적·완전 통신에서는 충분하지만 **입찰 철회(건강 저하)나 무효화가 퍼지지 않는다** —
           다른 로봇이 옛 입찰을 계속 되돌려 보낸다. 연구 계산(mrta_small.py)이 쓴 규칙이라 재현에 쓴다.
- 'cbba' : CBBA 표 1 의 update / reset / leave 규칙(출처별 최신 시각 s 비교). L_t = 1 이면 번들 해제
           부분이 사라져 작업별 규칙만 남는다. 철회·무효화가 시각 비교로 퍼진다 (보고서 54행 채택안).

표기: i = 받는 로봇(me), k = 보낸 로봇(sender), z = 승자, s_i·s_k = 각자의 출처별 최신 시각.
"""

from __future__ import annotations

import math
from typing import Callable, Dict, Mapping

from amr_fleet.negotiation.bids import Bid, better, NO_BID

MergeFn = Callable[[str, str, Bid, Bid, Mapping[str, float], Mapping[str, float]], Bid]


def merge_max(me: str, sender: str, bi: Bid, bk: Bid,
              s_i: Mapping[str, float], s_k: Mapping[str, float]) -> Bid:
    """
    CBAA 최대 합의: 받은 입찰이 더 나으면 받아들이고, 아니면 그대로 둔다.

    단, 나를 승자로 적은 입찰은 받지 않는다 — 내 입찰은 내가 가장 잘 안다. 무효화·철회가 없으면
    (연구 계산의 조건) 그런 입찰이 내 표보다 나을 수 없으므로 원 알고리즘과 결과가 같다. 철회 뒤
    다른 로봇이 되돌려 보낸 내 옛 입찰을 내가 다시 받아들이는 것만 막는다.
    """
    if bk.winner == me:
        return bi
    return bk if better(bk, bi) else bi


def merge_cbba(me: str, sender: str, bi: Bid, bk: Bid,
               s_i: Mapping[str, float], s_k: Mapping[str, float]) -> Bid:
    """
    CBBA 표 1 (Choi·Brunet·How 2009) 의 작업별 수신 규칙, L_t = 1.

    반환값이 bi 면 leave, bk 면 update, NO_BID 면 reset 이다.
    """
    zi, zk = bi.winner, bk.winner

    def newer(m: str) -> bool:
        return s_k.get(m, -math.inf) > s_i.get(m, -math.inf)

    k_better = better(bk, bi)
    if zk == sender:                       # 보낸 로봇이 자기가 이긴다고 본다
        if zi == me:
            return bk if k_better else bi
        if zi == sender or zi is None:
            return bk
        return bk if (newer(zi) or k_better) else bi
    if zk == me:                           # 보낸 로봇이 내가 이긴다고 본다
        if zi == me or zi is None:
            return bi
        if zi == sender:
            return NO_BID
        return NO_BID if newer(zi) else bi
    if zk is None:                         # 보낸 로봇은 승자가 없다고 본다
        if zi == me or zi is None:
            return bi
        if zi == sender:
            return bk
        return bk if newer(zi) else bi
    m = zk                                 # 보낸 로봇은 제3의 로봇 m 이 이긴다고 본다
    if zi == me:
        return bk if (newer(m) and k_better) else bi
    if zi == sender:
        return bk if newer(m) else NO_BID
    if zi == m or zi is None:
        return bk if newer(m) else bi
    n = zi                                 # 나는 또 다른 로봇 n 이 이긴다고 본다
    if newer(m) and newer(n):
        return bk
    if newer(m) and k_better:
        return bk
    if newer(n) and s_i.get(m, -math.inf) > s_k.get(m, -math.inf):
        return NO_BID
    return bi


RULES: Dict[str, MergeFn] = {'max': merge_max, 'cbba': merge_cbba}


def get_rule(name: str) -> MergeFn:
    """이름으로 병합 규칙을 고른다. 모르는 이름이면 KeyError."""
    try:
        return RULES[name]
    except KeyError:
        raise KeyError(f'알 수 없는 병합 규칙 {name!r}. 사용 가능: {sorted(RULES)}') from None


__all__ = ['MergeFn', 'RULES', 'get_rule', 'merge_cbba', 'merge_max']
