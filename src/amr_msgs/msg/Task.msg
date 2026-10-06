# JSON Task Description 에 대응하는 작업 정의 (명세 8장)
std_msgs/Header header

string task_id
string robot_id                     # 미할당 시 빈 문자열
uint8 priority                      # 0(낮음) ~ 255(높음)
builtin_interfaces/Time deadline

geometry_msgs/PoseStamped pickup_pose
geometry_msgs/PoseStamped dropoff_pose

string item_type                    # "small", "medium", "large"
float32 item_mass                   # [kg] — 적재 시 동역학 반영

uint8 status

uint8 STATUS_PENDING=0
uint8 STATUS_IN_PROGRESS=1
uint8 STATUS_COMPLETED=2
uint8 STATUS_FAILED=3

# --- 분산 플릿 (보너스 4 P0, docs/research/fleet-distributed/p0.md §4) — 기본값(0, '')이면 예전과 같다 ---
uint32 version                      # 같은 task_id 작업 내용의 판. /fleet/task_board 게시물의 멱등 규칙
                                    # (같은 판·같은 내용 = 재전송 무시, 낮은 판 = 거절; VDA 5050 orderUpdateId)
uint32 epoch                        # 이 메시지를 낸 권위자(fleet_manager 등)의 epoch. 0 = 펜싱 없음
string leader_id                    # 그 권위자 id. 같은 epoch 는 (epoch, leader_id) 사전식으로 가른다
uint64 seq                          # 그 권위자가 낸 명령 순번 (1 부터, 권위자 안에서 단조 증가. 0 = 없음)
