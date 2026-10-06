"""순수 자료형 ↔ amr_msgs 왕복 시험 (agent/codec.py). 메시지 모듈이 없으면 건너뛴다."""

import math

import pytest

from amr_fleet.agent.core import Beacon, StateView
from amr_fleet.distributed import messages as m
from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.ordering import PriorityKey, ReqId, Stamp
from amr_fleet.distributed.wfg import OutEdge
from amr_fleet.negotiation.bids import Bid, LOCKED_COST, NO_BID, StateMessage

# 모듈 수준 importorskip 은 launch_testing pytest 플러그인 아래에서 디렉터리 전체 수집을 멈춘다 (f1 교훈) —
# 가져오기 실패를 표시만 하고 시험마다 건너뛴다
try:
    from amr_fleet.agent import codec
except ImportError:      # amr_msgs 가 빌드되지 않은 환경
    codec = None

pytestmark = pytest.mark.skipif(codec is None, reason='amr_msgs 가 빌드되지 않은 환경')


@pytest.mark.parametrize('kind', m.KINDS)
def test_vote_round_trip_every_kind(kind):
    v = VoteMsg(kind, 'amr_01', 'amr_02', 7, ReqId('amr_01', (1 << 62) + 5, 42),
                frozenset({'zone:b', 'zone:a'}),
                PriorityKey(3, Stamp(12, 999_999_999), 17, starved=True),
                Stamp(2_000_000, 1), 99, m.R_BUSY)
    ch, back = codec.msg_to_vote(codec.vote_to_msg(1, v))
    assert ch == 1 and back == v


def test_vote_without_key_and_deadline():
    v = VoteMsg(m.RELEASE, 'a', 'b', 0, ReqId('a', 1, 1))
    assert codec.msg_to_vote(codec.vote_to_msg(0, v)) == (0, v)
    key_no_dl = VoteMsg(m.REQUEST, 'a', 'b', 0, ReqId('a', 1, 2), frozenset({'z'}),
                        PriorityKey(0, None, 1), Stamp(5, 0), 3)
    assert codec.msg_to_vote(codec.vote_to_msg(0, key_no_dl))[1] == key_no_dl


def test_batch_filters_by_destination():
    vs = [(0, VoteMsg(m.VOTE, 'a', d, 0, ReqId(d, 1, 1))) for d in ('a', 'b', 'b', 'c')]
    b = codec.votes_to_batch('a', vs, 3.25)
    assert b.src == 'a' and codec.time_to_seconds(b.stamp) == pytest.approx(3.25)
    assert [v.dst for _, v in codec.batch_for(b, 'b')] == ['b', 'b']
    assert codec.batch_for(b, 'z') == []


def test_unknown_vote_kind_is_rejected():
    f = codec.vote_to_msg(0, VoteMsg(m.VOTE, 'a', 'b', 0, ReqId('a', 1, 1)))
    f.kind = 200
    with pytest.raises(ValueError):
        codec.msg_to_vote(f)


def test_neg_state_round_trip_keeps_locked_and_empty_bids():
    s = StateMessage('amr_03', 41.0, {'t1': Bid(LOCKED_COST, 'amr_02'), 't2': NO_BID,
                                      't3': Bid(12.5, 'amr_03')},
                     {'amr_01': 39.0, 'amr_03': 41.0}, healthy=False, assigned='t3',
                     executing=True)
    back = codec.msg_to_neg(codec.neg_to_msg(s))
    assert back == s
    assert back.bids['t1'].locked and back.bids['t2'] == NO_BID


def test_beacon_round_trip_with_nan_and_optional_fields():
    b = Beacon('amr_04', 3, 17, 123.5, 2, 9, '/fleet/fleet_manager_node', 'DEGRADED', 'DEAD',
               True, 1.0, -2.0, 0.5, 77.0, False, 't9', True, ('task:t9', 'zone:x'),
               (OutEdge('amr_02', 'token'),), 4, 100.25, 3.5)
    back = codec.msg_to_beacon(codec.beacon_to_msg(b))
    assert back == b
    nan = Beacon('amr_01', 1, 1, 1.0)
    back = codec.msg_to_beacon(codec.beacon_to_msg(nan))
    assert math.isnan(back.x) and back.request_stamp is None and back.remaining_m is None


def test_state_view_to_msg():
    sv = StateView('amr_02', 'RESYNC', '서버 하트비트 복귀', 'ALIVE', 1, 4, 'L', ('zone:x',),
                   ('zone:y',), 'VOTING', ('task:t3',), 't1', 'COMMIT', ('amr_01', 'amr_02'),
                   'amr_02', 5)
    msg = codec.state_to_msg(sv, 10.5)
    assert msg.mode == msg.MODE_RESYNC and msg.server_status == msg.SERVER_ALIVE
    assert list(msg.held) == ['zone:x'] and msg.low_epoch_rejected == 5
    assert codec.time_to_seconds(msg.stamp) == pytest.approx(10.5)


def test_board_task_from_task_msg():
    from amr_msgs.msg import Task
    t = Task()
    t.task_id, t.priority, t.robot_id = 'src.000001', 7, ''
    t.pickup_pose.pose.position.x, t.pickup_pose.pose.position.y = 3.0, -4.0
    bt = codec.task_to_board(t)
    assert (bt.task_id, bt.priority, bt.pickup_x, bt.pickup_y, bt.robot_id) == (
        'src.000001', 7, 3.0, -4.0, '')
