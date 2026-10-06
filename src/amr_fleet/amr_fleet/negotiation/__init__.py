"""
서버 없는 협상 할당 코어 (명세 5장 보너스 4 "로봇 간 직접 통신을 통한 협상 기반 작업 할당").

동기 라운드 CBAA(Choi·Brunet·How 2009, 로봇당 1건 = CBBA L_t=1)를 rclpy 없이 순수 파이썬으로 둔다.
설계 근거: 보고서 「분산 플릿 관리 협상 할당과 폴백」 19–73행(채택안)·218–276행(P2).

구성
- bids        : 입찰(Bid)·방송 상태(StateMessage)·비교 규칙 (비용이 작을수록 좋고, 동점은 robot_id 사전순)
- consensus   : 수신자 병합 규칙 — 'max'(CBAA Alg.2 최대 합의) · 'cbba'(CBBA 표 1, update/reset/leave)
- agent       : 로봇 1대의 협상 상태기계 (입찰 → 방송 → 병합 → 무효화·패배 판정 → 국소 확정 → 작업 잠금)
- task_lock   : 작업 ID 정족수 잠금 인터페이스(추상) + 시험용 가짜 구현 (실제 투표 결합은 다음 물결)
- cost        : 기존 할당(allocation.py)과 같은 비용 = 로봇→픽업 이동 시간, 격자 A* 경로 비용 선택지
- grid        : 점유 격자(PGM 지도) 적재·축소·팽창 + 8-이웃 A*
- channel     : 링크 모델 — 지연 U(lo, hi) ms, Bernoulli/버스트 손실, 링크별 파생 난수
- scenario    : 저장소 도크 4곳·스폰 5곳 좌표와 사례 생성기 (연구 계산 mrta_small.py 와 같은 난수 소비)
- sim         : 오프라인 동기 라운드 시뮬레이터 (이탈·건강 저하 주입, 중복·미할당·고착·라운드·비용 지표)
- sweep       : K·H·손실률·작업 수 스윕 → CSV·마크다운 표 (python3 -m amr_fleet.negotiation.sweep)
"""
