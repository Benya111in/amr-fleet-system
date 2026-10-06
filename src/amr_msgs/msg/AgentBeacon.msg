# 로봇 에이전트 비컨 (보너스 4 P3, docs/research/fleet-distributed/p3.md §2)
# 토픽 /fleet_p2p/beacon — BEST_EFFORT · VOLATILE · KEEP_LAST 1, 로봇마다 sim 2 Hz. 최신 값만 의미가 있다.
# 쓰임: 과반 의심(서버 DEAD 동의), 연결 과반(ISOLATED 판정), 복제 WFG, 관찰자 복귀(서버가 max epoch 를 읽는다).
# 구역 · 작업 잠금의 안전에는 쓰지 않는다 (held 는 진단 · 협상 후보 정리용).

builtin_interfaces/Time stamp       # 송신 sim 시각
string robot_id
uint64 incarnation                  # 재시작 번호 (투표 코어와 같은 값)
uint64 seq                          # 같은 incarnation 안 비컨 순번 — (incarnation, seq) 역행 비컨은 버린다
uint32 vote_epoch                   # 투표 프로토콜 max_epoch_seen
uint32 fence_epoch                  # 본 적 있는 최고 권위자 (epoch, leader_id)
string fence_leader
uint8 mode                          # AgentState.MODE_*
uint8 server_status                 # AgentState.SERVER_*
bool server_dead                    # 이 로봇의 판단: 서버 DEAD (과반 의심에 들어간다)
float64 x                           # map 자세 (모르면 NaN)
float64 y
float64 yaw
float32 battery                     # [%]
bool localization_healthy
string current_task_id              # 실행 중 작업 ('' = 없음)
bool picked_up                      # 픽업을 시작했나 (작업 ID 확정 뒤)
string[] held                       # 보유 자원 (zone:* · task:*)
WaitEdge[] wait_edges               # 복제 WFG out-edge
int32 priority                      # 대기 우선순위 (WFG 희생자 선정)
builtin_interfaces/Time request_stamp  # 지금 대기의 처음 요청 시각 (0 = 없음)
float32 remaining_m                 # 남은 경로 길이 (진전 확인, NaN = 모름)
