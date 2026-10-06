"""
투표 메시지 (프로토콜 명세 4 절).

모든 메시지는 같은 필드를 갖는 1:1 메시지다 (방송은 송신 측이 투표자마다 하나씩 만든다 — 자기 자신 포함).
P3 의 ROS 메시지(`/fleet_p2p/vote`)는 이 필드를 그대로 옮긴다. to_dict/from_dict 는 로그 재생용.
"""

from __future__ import annotations

import dataclasses
from typing import Any, Dict, FrozenSet, Optional

from amr_fleet.distributed.ordering import PriorityKey, ReqId, Stamp

REQUEST = 'REQUEST'
VOTE = 'VOTE'
NACK = 'NACK'
ENTER = 'ENTER'
ENTER_ACK = 'ENTER_ACK'
ABANDON = 'ABANDON'
RELEASE = 'RELEASE'
RELEASE_ACK = 'RELEASE_ACK'
KINDS = (REQUEST, VOTE, NACK, ENTER, ENTER_ACK, ABANDON, RELEASE, RELEASE_ACK)

# 요청자 → 투표자 / 투표자 → 요청자
TO_VOTER = frozenset({REQUEST, ENTER, ABANDON, RELEASE})
TO_REQUESTER = frozenset({VOTE, NACK, ENTER_ACK, RELEASE_ACK})

# NACK 사유
R_BUSY = 'busy'                  # 확정(점유) 표가 막음 — 대기열에 넣었음, 풀리면 VOTE 가 온다
R_CONTENDED = 'contended'        # 다른 요청의 잠정 표가 막음 (표 갈림 가능) — 대기열에 넣었음
R_EXPIRED = 'expired'            # 요청 기한이 투표자 시각으로 이미 지남
R_FINISHED = 'finished'          # 묘비: 이미 RELEASE/ABANDON 된 요청
R_STALE_EPOCH = 'stale_epoch'    # 송신자 epoch 가 투표자의 max_epoch_seen 보다 낮음 (수용 안 함)
R_NOT_TENTATIVE = 'not_tentative'  # ENTER 인데 잠정 표가 그 요청에 묶여 있지 않음
R_AMNESIC = 'amnesic'            # 영속 상태를 잃은 투표자 — 영구 기권
REASONS = (R_BUSY, R_CONTENDED, R_EXPIRED, R_FINISHED, R_STALE_EPOCH, R_NOT_TENTATIVE, R_AMNESIC)


@dataclasses.dataclass(frozen=True)
class VoteMsg:
    """투표 프로토콜 메시지 1 개."""

    kind: str
    src: str
    dst: str
    epoch: int
    req: ReqId
    resources: FrozenSet[str] = frozenset()
    key: Optional[PriorityKey] = None
    deadline: Optional[Stamp] = None
    lamport: int = 0
    reason: str = ''

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f'메시지 종류는 {KINDS} 중 하나: {self.kind!r}')
        if not isinstance(self.resources, frozenset):
            object.__setattr__(self, 'resources', frozenset(self.resources))

    def to_dict(self) -> Dict[str, Any]:
        """JSON 용 (자원은 정렬 목록)."""
        return {
            'kind': self.kind, 'src': self.src, 'dst': self.dst, 'epoch': self.epoch,
            'req': self.req.to_list(), 'resources': sorted(self.resources),
            'key': None if self.key is None else self.key.to_dict(),
            'deadline': None if self.deadline is None else self.deadline.to_dict(),
            'lamport': self.lamport, 'reason': self.reason,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'VoteMsg':
        """to_dict 의 역."""
        return cls(str(d['kind']), str(d['src']), str(d['dst']), int(d['epoch']),
                   ReqId.from_list(d['req']), frozenset(d.get('resources', ())),
                   PriorityKey.from_dict(d.get('key')), Stamp.from_dict(d.get('deadline')),
                   int(d.get('lamport', 0)), str(d.get('reason', '')))

    def short(self) -> str:
        """한 줄 표시 (반례 출력용)."""
        res = ','.join(sorted(self.resources))
        extra = f' {self.reason}' if self.reason else ''
        return f'{self.kind}({self.req} {{{res}}} e{self.epoch}{extra}) {self.src}->{self.dst}'
