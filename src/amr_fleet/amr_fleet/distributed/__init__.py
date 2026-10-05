"""
분산 플릿 관리 코어 (보너스 4) — rclpy 비의존 순수 Python.

- ordering         : sim 시각 Stamp · 요청 ID · 대기 순서 키 (비교는 항상 (sec, nanosec) 전체)
- messages         : 투표 메시지 VoteMsg (8 종)
- store            : 투표 상태 영속 저장 (파일 + fsync, 메모리)
- vote_core        : 정적 과반 배타 투표 + 2단계 진입 (투표자 · 요청자 · 로봇 1 대 에이전트)
- failure_detector : sim-time 하트비트 감지기 (SUSPECT/DEAD, 과반 의심, 자기 지연 점검), epoch 펜스
- wfg              : 복제 대기 그래프 + 결정적 Tarjan + timestamp 희생자

프로토콜 명세: docs/research/fleet-distributed/protocol.md
"""
