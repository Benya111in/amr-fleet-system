"""negotiation.bids · consensus: 입찰 비교와 CBBA 표 1 수신 규칙 17 칸."""

import math

import pytest

from amr_fleet.negotiation.bids import Bid, better, LOCKED_COST, NO_BID, StateMessage, table_key
from amr_fleet.negotiation.consensus import get_rule, merge_cbba, merge_max

I, K, M, N = 'r_i', 'r_k', 'r_m', 'r_n'     # 받는 로봇, 보낸 로봇, 제3자 m, 제4자 n
OLD, NEW = 1.0, 2.0


def s(**times):
    """출처별 최신 시각 dict (기본 0)."""
    base = {I: 0.0, K: 0.0, M: 0.0, N: 0.0}
    base.update({{'i': I, 'k': K, 'm': M, 'n': N}[key]: v for key, v in times.items()})
    return base


# ---------- 입찰 비교 ----------

def test_better_orders_by_cost_then_id():
    assert better(Bid(1.0, 'b'), Bid(2.0, 'a'))
    assert not better(Bid(2.0, 'a'), Bid(1.0, 'b'))
    assert better(Bid(1.0, 'a'), Bid(1.0, 'b'))          # 동점 → id 사전순
    assert not better(Bid(1.0, 'b'), Bid(1.0, 'a'))
    assert not better(Bid(1.0, 'a'), Bid(1.0, 'a'))      # 엄밀
    assert better(Bid(5.0, 'a'), NO_BID)
    assert not better(NO_BID, Bid(5.0, 'a'))
    assert not better(NO_BID, NO_BID)
    assert better(Bid(LOCKED_COST, 'z'), Bid(0.0, 'a'))  # 잠금은 어떤 입찰보다 낫다


def test_bid_flags_and_message_locks():
    assert NO_BID.empty and not NO_BID.locked
    assert Bid(LOCKED_COST, 'a').locked
    assert not Bid(math.inf, 'a').locked
    msg = StateMessage('a', 1.0, {'t1': Bid(LOCKED_COST, 'a'), 't2': Bid(3.0, 'b')}, {'a': 1.0})
    assert msg.locks() == {'t1': 'a'}
    assert table_key({'b': NO_BID, 'a': Bid(1.0, 'x')}) == (('a', 1.0, 'x'), ('b', math.inf, None))


def test_get_rule():
    assert get_rule('max') is merge_max
    assert get_rule('cbba') is merge_cbba
    with pytest.raises(KeyError):
        get_rule('nope')


def test_merge_max_is_max_consensus():
    assert merge_max(I, K, Bid(5.0, I), Bid(3.0, K), {}, {}) == Bid(3.0, K)
    assert merge_max(I, K, Bid(3.0, I), Bid(5.0, K), {}, {}) == Bid(3.0, I)
    # 철회(NO_BID)는 최대 합의로 퍼지지 않는다 — 'cbba' 를 쓰는 이유
    assert merge_max(I, K, Bid(3.0, K), NO_BID, {}, {}) == Bid(3.0, K)


def test_merge_max_never_adopts_own_stale_bid():
    # 철회한 내 옛 입찰이 되돌아와도 받지 않는다
    assert merge_max(I, K, NO_BID, Bid(1.0, I), {}, {}) == NO_BID
    assert merge_max(I, K, Bid(5.0, M), Bid(1.0, I), {}, {}) == Bid(5.0, M)


# ---------- CBBA 표 1 (Choi·Brunet·How 2009), L_t = 1 ----------
# (번호, 보낸 쪽 승자, 받는 쪽 승자, 보낸 입찰, 받는 입찰, s_k, s_i, 기대: 'update'|'reset'|'leave')
LOW, HIGH = 1.0, 9.0                          # 비용: LOW 가 더 나은 입찰
CASES = [
    # 보낸 로봇이 자기가 이긴다고 본다
    (1, K, I, LOW, HIGH, s(), s(), 'update'),
    (1, K, I, HIGH, LOW, s(), s(), 'leave'),
    (2, K, K, HIGH, LOW, s(), s(), 'update'),
    (3, K, M, HIGH, LOW, s(m=NEW), s(m=OLD), 'update'),      # s_km 이 더 새롭다
    (3, K, M, LOW, HIGH, s(m=OLD), s(m=NEW), 'update'),      # 입찰이 더 낫다
    (3, K, M, HIGH, LOW, s(m=OLD), s(m=NEW), 'leave'),
    (4, K, None, HIGH, math.inf, s(), s(), 'update'),
    # 보낸 로봇이 내가 이긴다고 본다
    (5, I, I, LOW, LOW, s(), s(), 'leave'),
    (6, I, K, LOW, LOW, s(), s(), 'reset'),
    (7, I, M, LOW, LOW, s(m=NEW), s(m=OLD), 'reset'),
    (7, I, M, LOW, LOW, s(m=OLD), s(m=NEW), 'leave'),
    (8, I, None, LOW, math.inf, s(), s(), 'leave'),
    # 보낸 로봇은 제3자 m 이 이긴다고 본다
    (9, M, I, LOW, HIGH, s(m=NEW), s(m=OLD), 'update'),
    (9, M, I, LOW, HIGH, s(m=OLD), s(m=NEW), 'leave'),
    (9, M, I, HIGH, LOW, s(m=NEW), s(m=OLD), 'leave'),
    (10, M, K, LOW, LOW, s(m=NEW), s(m=OLD), 'update'),
    (10, M, K, LOW, LOW, s(m=OLD), s(m=NEW), 'reset'),
    (11, M, M, LOW, HIGH, s(m=NEW), s(m=OLD), 'update'),
    (11, M, M, LOW, HIGH, s(m=OLD), s(m=NEW), 'leave'),
    (12, M, N, HIGH, LOW, s(m=NEW, n=NEW), s(m=OLD, n=OLD), 'update'),
    (12, M, N, LOW, HIGH, s(m=NEW, n=OLD), s(m=OLD, n=NEW), 'update'),
    (12, M, N, HIGH, LOW, s(m=OLD, n=NEW), s(m=NEW, n=OLD), 'reset'),
    (12, M, N, HIGH, LOW, s(m=NEW, n=OLD), s(m=OLD, n=NEW), 'leave'),
    (13, M, None, LOW, math.inf, s(m=NEW), s(m=OLD), 'update'),
    (13, M, None, LOW, math.inf, s(m=OLD), s(m=NEW), 'leave'),
    # 보낸 로봇은 승자가 없다고 본다
    (14, None, I, math.inf, LOW, s(), s(), 'leave'),
    (15, None, K, math.inf, LOW, s(), s(), 'update'),
    (16, None, M, math.inf, LOW, s(m=NEW), s(m=OLD), 'update'),
    (16, None, M, math.inf, LOW, s(m=OLD), s(m=NEW), 'leave'),
    (17, None, None, math.inf, math.inf, s(), s(), 'leave'),
    # 시각이 같으면 "더 새롭다"가 아니다 (엄밀 비교)
    (3, K, M, HIGH, LOW, s(m=OLD), s(m=OLD), 'leave'),
    (7, I, M, LOW, LOW, s(m=OLD), s(m=OLD), 'leave'),
    (10, M, K, LOW, LOW, s(m=OLD), s(m=OLD), 'reset'),
    (16, None, M, math.inf, LOW, s(m=OLD), s(m=OLD), 'leave'),
]


@pytest.mark.parametrize('entry,zk,zi,yk,yi,s_k,s_i,expect', CASES)
def test_merge_cbba_table(entry, zk, zi, yk, yi, s_k, s_i, expect):
    bk, bi = Bid(yk, zk), Bid(yi, zi)
    out = merge_cbba(I, K, bi, bk, s_i, s_k)
    want = {'update': bk, 'reset': NO_BID, 'leave': bi}[expect]
    assert out == want, f'표 1 의 {entry} 번 칸'


def test_merge_cbba_tie_breaks_by_robot_id():
    # 1 번 칸, 같은 비용: id 가 작은 쪽이 이긴다 ('r_i' < 'r_k' → 받는 쪽 유지)
    assert merge_cbba(I, K, Bid(2.0, I), Bid(2.0, K), {}, {}) == Bid(2.0, I)
    assert merge_cbba(K, I, Bid(2.0, K), Bid(2.0, I), {}, {}) == Bid(2.0, I)


def test_all_seventeen_entries_covered():
    assert {c[0] for c in CASES} == set(range(1, 18))
