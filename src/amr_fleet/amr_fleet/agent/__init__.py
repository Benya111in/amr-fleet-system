"""
로봇마다 하나인 분산 플릿 에이전트 (보너스 4 P3) — 순수 로직은 rclpy 비의존, ROS 노드는 얇게.

- modes          : 모드 상태기 NORMAL · HOLD_NEW · DEGRADED · ISOLATED · RESYNC (보고서 145–158행)
- task_lock_vote : 작업 ID 잠금을 투표 코어(task 채널)로 — hold_enter(잠정 표를 쥔 채 이동) + commit(픽업 시작)
- sim_exec       : 실행기 없이 협상 작업의 이동 · 확정 · 픽업 · 하역을 흉내 내는 가짜 실행기 (시험 · 빠른 계층용)
- core           : 위 모두 + 투표 코어 두 채널(zone · task) + 하트비트 감시 · 과반 의심 · 에포크 펜스 · CBAA ·
                   복제 WFG 를 묶은 로봇 1 대의 에이전트. 입력(메시지 · 시각) → 보낼 것(Outbox)
- codec          : 순수 자료형 ↔ amr_msgs 변환 (rclpy 비의존, 메시지 모듈만 쓴다)
- node           : ROS 노드 (python3 -m amr_fleet.agent.node)
- manager_shim   : 시험용 서버 프로세스 — 원본 fleet_manager_node 와 하트비트 발행자를 한 프로세스에 묶는다
- fake_clock     : 시뮬레이터 없는 시험용 /clock

설계: docs/research/fleet-distributed/p3.md
"""
