"""
T1 운동학 플릿-인-더-루프 모의 (보너스 4 P4) — rclpy 비의존 순수 Python.

5 대 차동구동 로봇을 고정 dt 로 적분하고, 저장소의 실제 지도·구역·도크 위에서 중앙 모드(현행 할당·교통 규칙)와
분산 모드(구역 투표 + CBAA + 작업 ID 잠금)를 **순수 코어를 고치지 않고 그대로** 붙여 돌린다.

- geometry : 차체(0.60 × 0.40 m)·구역 상자 겹침, 경로(호 길이·자세)
- world    : 지도 → 격자 경로(Dijkstra + 시선 당기기), 구역·도크·스폰, 경로 위 구역 방문·정지선
- net      : 사건 큐 + CommEmulator 링크(모든 링크 같은 모델), 신뢰 경로 재전송
- central  : 중앙 서버 모형 (fleet_manager 할당 의미 + traffic_manager 토큰)
- robot    : 로봇 1 대 (운동학, 작업 상태기계, 구역 게이트, 투표·협상 참여, 장애 감지)
- oracle   : 실제 차체 기준 동시 점유 · 중복 실행 · 대기 · 교착 순환
- sim      : 한 실행 (외생 입력 → 결과 dict)
- sweep    : 셀 × 시드 스윕 CLI (JSONL)
- analysis : 쌍차 효과크기·CI·k/n, rule of three

설계 근거와 결과: docs/research/fleet-distributed/p4.md
"""
