# 핵심 알고리즘 구현 및 튜닝 근거

> 명세 4.10 요구 산출물. 구현 원리와 파라미터 튜닝 과정, 측정 결과를 문서화한다.
> 동료 평가에서 "공분산 설정 근거", "비용 함수 구현"을 직접 확인한다.
> 구현 전에 쓴 설계 브리프(문헌 조사, 독자 제안, 적대적 리뷰 반영 이력)는 [`docs/research/`](../research/) 에 있다.

| 문서 | 명세 | 내용 | 구현 패키지 |
| --- | --- | --- | --- |
| [kinematics.md](kinematics.md) | 4.2 | 차동 구동 순기구학(정확 호 적분), 4096틱 엔코더·슬립 모델, 오도메트리 공분산 전파, 드리프트 실험, IMU/LiDAR 전처리 | `amr_localization` |
| [ekf.md](ekf.md) | 4.3 | 이중 EKF(odom/map) 예측·업데이트, 센서별 공분산 근거, 위치 추정 정확도 측정 | `amr_localization` + `config/ekf.yaml` |
| [slam.md](slam.md) | 4.3 | slam_toolbox 파라미터 의미·튜닝, 맵 품질 지표, AMCL 튜닝, 납치 탐지·복구 | `amr_localization` |
| [astar.md](astar.md) | 4.4 | A* 직접 구현·평활화, NavFn/Smac 비교(경로 길이, 계획 시간, 여유 거리), 50쌍 성공률 | `amr_navigation` |
| [dwa.md](dwa.md) | 4.4, 4.7 | DWA 속도 샘플링·궤적 시뮬레이션·비용 함수, Velocity Obstacle, TEB/DWB 비교 | `amr_navigation` |
| [costmap.md](costmap.md) | 4.4 | 레이어 구성, inflation 반경·cost scaling 튜닝, 0.60 m 좁은 통로 | `amr_navigation` |
| [path_tracking.md](path_tracking.md) | 4.5 | Pure Pursuit 속도 적응 look-ahead, S-curve·저크 제한, PID 게인 튜닝, CTE | `amr_navigation` |
| [perception.md](perception.md) | 4.6 | YOLOv8 연동·클래스·파인튜닝, Pinhole 2D→3D, 마커, ArUco | `amr_perception` |
| [tracking.md](tracking.md) | 4.7 | LiDAR 동적 장애물 분류, 칼만 필터 추적, TTC, 안전 게이트(구역·E-stop·센서 고장) | `amr_perception` |
| [behavior_tree.md](behavior_tree.md) | 4.8 | BT 설계(노드 34종, 서브트리, 복구 3종), Groot, 새 노드 추가 절차 | `amr_behavior` |
| [docking.md](docking.md) | 4.8 | ArUco 기반 정밀 도킹 제어, 2 cm / 1° 판정, 재시도 | `amr_behavior` |
| [deadlock.md](deadlock.md) | 4.9 | 교통 관리(충돌 예측·구역 토큰), 교착 탐지(wait-for 그래프), 해소 전략 2가지 | `amr_fleet` |

성능 측정 절차와 결과 표는 [`docs/reports/`](../reports/) 에 있다.
