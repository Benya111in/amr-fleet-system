# 투표 프로토콜 메시지 1 개 (distributed/messages.py VoteMsg 를 그대로 옮긴 것, protocol.md §4)
# FleetVoteBatch 에 묶여 /fleet_p2p/vote 로 간다.

uint8 KIND_REQUEST=0
uint8 KIND_VOTE=1
uint8 KIND_NACK=2
uint8 KIND_ENTER=3
uint8 KIND_ENTER_ACK=4
uint8 KIND_ABANDON=5
uint8 KIND_RELEASE=6
uint8 KIND_RELEASE_ACK=7

uint8 CHANNEL_ZONE=0                # 구역 · 포켓 사슬 (zone:* · pocket:*)
uint8 CHANNEL_TASK=1                # 작업 ID 잠금 (task:*) — 프로토콜 인스턴스가 따로다

uint8 channel
uint8 kind
string src
string dst
uint32 epoch                        # 송신자의 max_epoch_seen (투표 프로토콜 epoch)
string req_robot                    # ReqId = (robot, incarnation, seq)
uint64 req_incarnation
uint64 req_seq
string[] resources                  # 정렬된 자원 ID
bool has_key                        # REQUEST 에만: 대기열 순서 키
int32 key_priority
bool key_has_deadline
builtin_interfaces/Time key_deadline
uint64 key_since                    # 같은 논리 요청의 첫 시도 Lamport 시각
bool key_starved
bool has_deadline                   # REQUEST 에만: 잠정 표 기한 (요청자 sim 시각)
builtin_interfaces/Time deadline
uint64 lamport
string reason                       # NACK 사유 (busy · contended · expired · finished · stale_epoch · ...)
