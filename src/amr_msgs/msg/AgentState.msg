# 에이전트 상태 (로봇 네임스페이스 fleet_agent/state) — BT 노드 · 대시보드 · 시험이 읽는다.
# QoS RELIABLE · TRANSIENT_LOCAL · KEEP_LAST 1 (늦게 뜬 BT 실행기도 마지막 허가 상태를 받는다).
# 진입 허가의 근거는 held 뿐이다 — held 에 없으면 진입하지 않는다 (허가 목록, fail-closed).

uint8 MODE_NORMAL=0                 # 서버 하트비트 정상: 중앙 할당, 구역은 정족수 투표
uint8 MODE_HOLD_NEW=1               # 서버 SUSPECT: 새 작업 보류, 진행 중 작업 계속
uint8 MODE_DEGRADED=2               # 과반이 서버 DEAD 동의: CBAA 협상 + 작업 ID 잠금
uint8 MODE_ISOLATED=3               # 자신이 과반과 연결되지 않음: 새 작업 · 새 진입 없음
uint8 MODE_RESYNC=4                 # 서버 복귀 후 히스테리시스: 새 작업 보류, 안정되면 NORMAL

uint8 SERVER_UNKNOWN=0
uint8 SERVER_ALIVE=1
uint8 SERVER_SUSPECT=2
uint8 SERVER_DEAD=3

builtin_interfaces/Time stamp
string robot_id
uint8 mode
string mode_reason
uint8 server_status
uint32 vote_epoch
uint32 fence_epoch
string fence_leader
string[] held                       # 진입 허가된 자원 (과반 ENTER_ACK 를 받은 것만)
string[] requested                  # 진행 중인 구역 요청 자원
string zone_phase                   # IDLE · VOTING · VOTED · ENTERING · HELD · BACKOFF
string[] denied                     # 거부된 작업 잠금 (task:*) — 다른 로봇이 확정했다
string current_task_id
string task_phase                   # '' · TRAVEL · COMMIT · PICKUP · DELIVER · DONE · FAILED
string[] deadlock_members           # 복제 WFG 가 확정한 교착 후보 (진단)
string deadlock_victim
uint32 low_epoch_rejected           # 낮은 (epoch, leader_id) 로 거부한 명령 · 하트비트 수
