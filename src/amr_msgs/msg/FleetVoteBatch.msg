# 한 번의 처리에서 나온 투표 메시지 묶음 (보너스 4 P3, docs/research/fleet-distributed/p3.md §2)
# 토픽 /fleet_p2p/vote — RELIABLE · VOLATILE · KEEP_LAST 256. 모든 에이전트가 받고 dst 가 자기인 것만 처리한다.
# 손실 · 재정렬 · 중복은 프로토콜이 견딘다 (재전송 · 묘비 · 멱등) — RELIABLE 은 손실을 줄이는 최적화일 뿐이다.
# 시험의 손실 · 지연은 수신 측 CommEmulator(링크 src→나)가 묶음 단위로 건다 (한 묶음 = 한 패킷).

string src
builtin_interfaces/Time stamp       # 송신 sim 시각 (로그용)
FleetVote[] votes
