"""
순수 자료형 ↔ amr_msgs 변환 (rclpy 비의존 — 메시지 모듈만 쓴다, 시험에서 바로 왕복 검사).

- VoteMsg(distributed.messages) ↔ amr_msgs/FleetVote (+ 채널)
- StateMessage(negotiation.bids)  ↔ amr_msgs/NegState
- Beacon(agent.core)              ↔ amr_msgs/AgentBeacon
- StateView(agent.core)           → amr_msgs/AgentState
- amr_msgs/Task(작업 게시판)      → BoardTask
- 시각: sim 초(float) · Stamp ↔ builtin_interfaces/Time. Stamp 는 (sec, nanosec) 를 그대로 옮긴다 (정밀도 손실 없음)
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from amr_fleet.agent.core import Beacon, BoardTask, mode_code, server_code, StateView
from amr_fleet.agent.modes import MODE_CODES, SERVER_CODES
from amr_fleet.distributed import messages as m
from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.ordering import PriorityKey, ReqId, Stamp
from amr_fleet.distributed.wfg import OutEdge
from amr_fleet.negotiation.bids import Bid, StateMessage
from amr_msgs.msg import (
    AgentBeacon, AgentState, FleetVote, FleetVoteBatch, NegBid, NegState, Task, WaitEdge,
)
from builtin_interfaces.msg import Time

KIND_TO_CODE = {m.REQUEST: FleetVote.KIND_REQUEST, m.VOTE: FleetVote.KIND_VOTE,
                m.NACK: FleetVote.KIND_NACK, m.ENTER: FleetVote.KIND_ENTER,
                m.ENTER_ACK: FleetVote.KIND_ENTER_ACK, m.ABANDON: FleetVote.KIND_ABANDON,
                m.RELEASE: FleetVote.KIND_RELEASE, m.RELEASE_ACK: FleetVote.KIND_RELEASE_ACK}
CODE_TO_KIND = {v: k for k, v in KIND_TO_CODE.items()}
CODE_TO_MODE = {v: k for k, v in MODE_CODES.items()}
CODE_TO_SERVER = {v: k for k, v in SERVER_CODES.items()}


# ---------------------------------------------------------------------- 시각
def stamp_to_time(s: Stamp) -> Time:
    """Stamp → Time (sec 는 int32 범위여야 한다)."""
    return Time(sec=int(s.sec), nanosec=int(s.nanosec))


def time_to_stamp(t: Time) -> Stamp:
    """Time → Stamp."""
    return Stamp(int(t.sec), int(t.nanosec))


def seconds_to_time(t: float) -> Time:
    """시각(sim 초) → Time."""
    return stamp_to_time(Stamp.from_seconds(float(t)))


def time_to_seconds(t: Time) -> float:
    """Time → sim 초."""
    return int(t.sec) + int(t.nanosec) * 1e-9


# ---------------------------------------------------------------------- 투표
def vote_to_msg(channel: int, v: VoteMsg) -> FleetVote:
    """투표 메시지 VoteMsg → FleetVote."""
    out = FleetVote()
    out.channel = int(channel)
    out.kind = KIND_TO_CODE[v.kind]
    out.src, out.dst, out.epoch = v.src, v.dst, int(v.epoch)
    out.req_robot, out.req_incarnation, out.req_seq = (v.req.robot, int(v.req.incarnation),
                                                       int(v.req.seq))
    out.resources = sorted(v.resources)
    if v.key is not None:
        out.has_key = True
        out.key_priority = int(v.key.priority)
        if v.key.deadline is not None:
            out.key_has_deadline = True
            out.key_deadline = stamp_to_time(v.key.deadline)
        out.key_since = int(v.key.since)
        out.key_starved = bool(v.key.starved)
    if v.deadline is not None:
        out.has_deadline = True
        out.deadline = stamp_to_time(v.deadline)
    out.lamport = int(v.lamport)
    out.reason = v.reason
    return out


def msg_to_vote(f: FleetVote) -> Tuple[int, VoteMsg]:
    """투표 메시지 FleetVote → (채널, VoteMsg). 모르는 종류면 ValueError."""
    if f.kind not in CODE_TO_KIND:
        raise ValueError(f'모르는 투표 종류 {f.kind}')
    key: Optional[PriorityKey] = None
    if f.has_key:
        key = PriorityKey(int(f.key_priority),
                          time_to_stamp(f.key_deadline) if f.key_has_deadline else None,
                          int(f.key_since), bool(f.key_starved))
    deadline = time_to_stamp(f.deadline) if f.has_deadline else None
    v = VoteMsg(CODE_TO_KIND[f.kind], f.src, f.dst, int(f.epoch),
                ReqId(f.req_robot, int(f.req_incarnation), int(f.req_seq)),
                frozenset(f.resources), key, deadline, int(f.lamport), f.reason)
    return int(f.channel), v


def votes_to_batch(src: str, votes: Sequence[Tuple[int, VoteMsg]], now: float) -> FleetVoteBatch:
    """(채널, VoteMsg) 목록 → FleetVoteBatch."""
    b = FleetVoteBatch()
    b.src = src
    b.stamp = seconds_to_time(now)
    b.votes = [vote_to_msg(ch, v) for ch, v in votes]
    return b


def batch_for(batch: FleetVoteBatch, dst: str) -> List[Tuple[int, VoteMsg]]:
    """묶음에서 dst 앞 메시지만 (채널, VoteMsg) 로."""
    return [msg_to_vote(f) for f in batch.votes if f.dst == dst]


# ---------------------------------------------------------------------- 협상
def neg_to_msg(s: StateMessage) -> NegState:
    """협상 상태 StateMessage → NegState."""
    out = NegState()
    out.sender = s.sender
    out.stamp = float(s.stamp)
    out.bids = [NegBid(task_id=t, cost=float(b.cost), winner=b.winner or '')
                for t, b in sorted(s.bids.items())]
    items = sorted(s.info_time.items())
    out.info_robots = [r for r, _ in items]
    out.info_times = [float(t) for _, t in items]
    out.healthy = bool(s.healthy)
    out.assigned = s.assigned or ''
    out.executing = bool(s.executing)
    return out


def msg_to_neg(n: NegState) -> StateMessage:
    """협상 상태 NegState → StateMessage."""
    bids = {b.task_id: Bid(float(b.cost), b.winner or None) for b in n.bids}
    info = {r: float(t) for r, t in zip(n.info_robots, n.info_times)}
    return StateMessage(n.sender, float(n.stamp), bids, info, bool(n.healthy),
                        n.assigned or None, bool(n.executing))


# ---------------------------------------------------------------------- 비컨
def _nan(v: Optional[float]) -> float:
    return math.nan if v is None else float(v)


def beacon_to_msg(b: Beacon) -> AgentBeacon:
    """비컨 Beacon → AgentBeacon."""
    out = AgentBeacon()
    out.stamp = seconds_to_time(b.stamp)
    out.robot_id = b.robot_id
    out.incarnation, out.seq = int(b.incarnation), int(b.seq)
    out.vote_epoch, out.fence_epoch, out.fence_leader = (int(b.vote_epoch), int(b.fence_epoch),
                                                         b.fence_leader)
    out.mode = mode_code(b.mode)
    out.server_status = server_code(b.server_status)
    out.server_dead = bool(b.server_dead)
    out.x, out.y, out.yaw = float(b.x), float(b.y), float(b.yaw)
    out.battery = float(b.battery)
    out.localization_healthy = bool(b.localization_healthy)
    out.current_task_id = b.current_task_id
    out.picked_up = bool(b.picked_up)
    out.held = list(b.held)
    out.wait_edges = [WaitEdge(holder=e.holder, reason=e.reason) for e in b.wait_edges]
    out.priority = int(b.priority)
    out.request_stamp = seconds_to_time(b.request_stamp) if b.request_stamp is not None else Time()
    out.remaining_m = _nan(b.remaining_m)
    return out


def msg_to_beacon(a: AgentBeacon) -> Beacon:
    """비컨 AgentBeacon → Beacon (모르는 모드 · 상태 코드면 ValueError)."""
    rs = a.request_stamp
    req = None if (rs.sec == 0 and rs.nanosec == 0) else time_to_seconds(rs)
    rem = None if math.isnan(a.remaining_m) else float(a.remaining_m)
    return Beacon(a.robot_id, int(a.incarnation), int(a.seq), time_to_seconds(a.stamp),
                  int(a.vote_epoch), int(a.fence_epoch), a.fence_leader, CODE_TO_MODE[a.mode],
                  CODE_TO_SERVER[a.server_status], bool(a.server_dead), float(a.x), float(a.y),
                  float(a.yaw), float(a.battery), bool(a.localization_healthy),
                  a.current_task_id, bool(a.picked_up), tuple(a.held),
                  tuple(OutEdge(e.holder, e.reason) for e in a.wait_edges), int(a.priority),
                  req, rem)


# ---------------------------------------------------------------------- 상태 · 게시판
def state_to_msg(s: StateView, now: float) -> AgentState:
    """에이전트 상태 StateView → AgentState."""
    out = AgentState()
    out.stamp = seconds_to_time(now)
    out.robot_id = s.robot_id
    out.mode = mode_code(s.mode)
    out.mode_reason = s.mode_reason
    out.server_status = server_code(s.server_status)
    out.vote_epoch, out.fence_epoch, out.fence_leader = (int(s.vote_epoch), int(s.fence_epoch),
                                                         s.fence_leader)
    out.held = list(s.held)
    out.requested = list(s.requested)
    out.zone_phase = s.zone_phase
    out.denied = list(s.denied)
    out.current_task_id = s.current_task_id
    out.task_phase = s.task_phase
    out.deadlock_members = list(s.deadlock_members)
    out.deadlock_victim = s.deadlock_victim
    out.low_epoch_rejected = int(s.low_epoch_rejected)
    return out


def task_to_board(t: Task) -> BoardTask:
    """작업 게시판 Task → BoardTask (픽업 자세 x, y 만)."""
    p = t.pickup_pose.pose.position
    return BoardTask(t.task_id, int(t.priority), float(p.x), float(p.y), t.robot_id or '')
