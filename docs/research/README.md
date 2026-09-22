# 알고리즘 설계 브리프 (연구 조사 + 독자 제안)

명세 4.2~4.9 의 핵심 알고리즘마다, 구현 전에 쓴 설계 문서다. 각 브리프는 같은 구조를 따른다.

1. 명세 요구사항 → 설계 답 → 측정 방법 매핑
2. 기준(베이스라인) 알고리즘의 수학적 유도 — 명세가 "직접 구현"을 요구하는 부분
3. 2023-09 ~ 2026-09 문헌 조사 (fetch 로 확인한 인용은 VERIFIED, 고전은 RECALLED 로 구분)
4. 우리 제안 — 가장 가까운 선행 연구 대비 위치를 정직하게 밝힌다 (새 이론이라 주장하지 않는 경우 그렇게 적었다)
5. 평가 계획(명세 지표에 매핑)과 구현 계획(패키지·노드·인터페이스·테스트)

## 작성·검증 과정

- 영역별 작성 → **적대적 리뷰**(수식 오류, 이미 발표된 아이디어 여부, 명세 누락, 인용 실재 여부 점검; 1차 점수 10점 만점에 5~6점)
  → 저자 수정 → **감사**(리뷰 항목 전수 대조, 수치 재유도, 프로젝트 설정·인터페이스 계약과 정합).
- 리뷰 원문과 저자 요약은 각 영역의 `review/critique.json`, `review/survey_summary.json` 에 있다.
- 브리프 속 수치는 `checks/` 의 스크립트로 재유도한 것이다 (`python3 checks/<스크립트>.py`, 출력은 같은 이름의 `.out`/`.txt`/`.log`).
- 저장소에 넣지 않은 것: 저작권이 있는 논문 PDF·본문 추출본·웹 페이지, 확인용으로 받은 Nav2/robot_localization/Gazebo
  원본 소스(`checks/src/` 로 인용된 것 — ROS2 Humble 태그의 GitHub 소스에서 같은 파일을 볼 수 있다), 빌드 산출물.

| 영역 | 브리프 | 명세 | 리뷰 지적 반영 | 제안 (선행 연구 대비 위치) |
| --- | --- | --- | --- | --- |
| 기구학·오도메트리 | [kinematics-odometry](kinematics-odometry/kinematics-odometry.md) | 4.2 | 35/35 | SACO 슬립 적응 오도메트리 공분산 (변형), GRC 자이로 기준 휠 파라미터 RLS 보정 (변형) |
| 상태 추정 | [state-estimation](state-estimation/state-estimation.md) | 4.3 | 51/51 | IAG-EKF 혁신 적응 게이팅 (변형), SMC-CUSUM 납치 탐지기 (새 조합), 능동 복구 FSM |
| 전역 경로 | [global-planning](global-planning/global-planning.md) | 4.4 | 42/42 | TAR-A* 정지거리 속도맵 시간비용 탐색 (변형), 비용 변화 트리거 국소 재계획, LOS 단축 + 여유 평활화 |
| 지역 경로·회피 | [local-planning](local-planning/local-planning.md) | 4.4, 4.7 | 55/55 | PVT-DWA v2 — 5 s 결정론적 GVO-TTC + 제동 꼬리 확률 제약 (공학적 통합, 새 이론 아님; 프로토타입 288회 결과 포함) |
| 경로 추종·속도 제어 | [path-tracking-control](path-tracking-control/path-tracking-control.md) | 4.5 | 49/49 | CC-PP 현 보정 Pure Pursuit (변형), JRG 저크 제한 거버너 (새 조합), 적재 질량 스케줄 2-DOF PI |
| 인지·추적 | [perception-tracking](perception-tracking/perception-tracking.md) | 4.6, 4.7 | 54/54 | JTC-IMM 카메라 클래스 사후확률 기반 IMM (공학적 적용), 2D DATMO 동적성 로그오즈 |
| 작업 수행·도킹 | [task-execution-docking](task-execution-docking/task-execution-docking.md) | 4.8 | 42/42 | CGD 공분산 게이트 도킹, 위험 보정 최소 여유(LST) 디스패치, BT.CPP 작업 실행기 |
| Fleet·교통·교착 | [fleet-traffic-deadlock](fleet-traffic-deadlock/fleet-traffic-deadlock.md) | 4.8, 4.9 | 48/48 | CTR 통로 토큰 예약 교통 관리, HRA 헝가리안 + 후회 교환 할당, 실행 단계 교착 탐지·해소 |

각 브리프 말미의 "리뷰 반영 이력" 에 리뷰 항목별 처리 내용(수정, 또는 근거를 든 반론)이 있다.
구현 결과와 튜닝 과정은 `docs/algorithms/` 에, 설계 대비 달라진 점은 해당 구현 문서에 적는다.
