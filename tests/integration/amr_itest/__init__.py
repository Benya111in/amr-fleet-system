"""
amr_itest: 통합 테스트 하네스 (명세 4.10 "통합 테스트 시나리오 ≥ 10 자동화").

구성
  config        저장소/설정 파일 경로, 환경 변수 노브 (ITEST_*)
  scenario      시나리오 메타데이터(지표·기준·로그 포맷) + 요구사항 검사 + 건너뛰기
  results       logs/itest/<시나리오>/result.json · CSV 기록
  requirements  패키지/실행 파일/런치 파일 존재 확인 (ament_index)
  stack         시나리오별 launch 구성: 실제 노드가 있으면 실제, 없으면 대역(stand-in)
  probe         rclpy 그래프 헬퍼 (토픽 대기·주기 측정·TF·서비스 호출)
  rates, tf_tree, kinematics, cdr, junit   rclpy 비의존 순수 로직 (unit/ 에서 pytest)
  standins      계약(docs/architecture/components.md §5)을 따르는 최소 대역 노드
  report        JUnit + result.json 집계 → logs/itest/summary.{json,md}
"""

__version__ = '0.1.0'
