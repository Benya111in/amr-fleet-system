# 서버 생존 신호 (보너스 4 P3, docs/research/fleet-distributed/p3.md §2)
# 토픽 /fleet/heartbeat — BEST_EFFORT · VOLATILE · KEEP_LAST 1, sim 2 Hz.
# 늦은 하트비트를 재전송할 이유가 없다 (다음 것이 곧 온다). 로봇은 sim-time 으로 무수신 시간을 잰다.
# 안전에는 쓰지 않는다 — 할당 모드(효율) 전환에만 쓴다. 낮은 (epoch, leader_id) 의 하트비트는 무시한다.

uint8 ROLE_ACTIVE=0                 # 권위자로 명령 중
uint8 ROLE_OBSERVER=1               # 재기동 직후 관찰 중 (로봇 상태를 모으는 중, 명령하지 않는다)

builtin_interfaces/Time stamp       # 서버 sim 시각
string leader_id                    # 권위자 id (amr_msgs/Task.leader_id 와 같은 값)
uint32 epoch                        # 권위자 epoch (amr_msgs/Task.epoch 와 같은 값)
uint64 seq                          # 하트비트 순번 (같은 epoch 안에서 단조 증가)
uint8 role
