# CBAA 협상 전체 상태 (negotiation/bids.py StateMessage, 보너스 4 P3)
# 토픽 /fleet_p2p/neg — RELIABLE · VOLATILE · KEEP_LAST 10, DEGRADED 에서 라운드(sim T_b)마다, 바뀌지 않아도 다시.
# 전체 상태 재방송이라 유실은 다음 라운드가 덮는다. 시각 단위는 라운드 번호 (sim 시각 / T_b).

string sender
float64 stamp                       # 라운드 번호
NegBid[] bids
string[] info_robots                # CBBA s_k: 로봇 → 마지막 소식 라운드
float64[] info_times
bool healthy
string assigned                     # '' = 없음
bool executing
