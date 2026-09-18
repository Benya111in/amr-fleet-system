# 핵심 알고리즘 구현 및 튜닝 근거

> 명세 10장 요구 산출물. 구현 원리와 파라미터 튜닝 과정을 문서화한다.
> 동료 평가에서 "공분산 설정 근거", "비용 함수 구현"을 직접 확인한다.

## 작성 예정 항목

- [ ] `kinematics.md` — 차동 구동 순기구학 수식, 오도메트리 누적 오차 분석
- [ ] `ekf.md` — 예측/업데이트 단계, 센서별 공분산 설정 근거
- [ ] `slam.md` — slam_toolbox 주요 파라미터 의미와 튜닝 근거, Loop Closure
- [ ] `astar.md` — A* 구현, NavFn/Smac 과의 비교 (경로 길이/계획 시간/여유 거리)
- [ ] `dwa.md` — 속도 샘플링, 궤적 시뮬레이션, 비용 함수, TEB 와 비교
- [ ] `costmap.md` — 레이어 구성, inflation_radius / cost_scaling_factor 튜닝
- [ ] `path_tracking.md` — Pure Pursuit look-ahead 적응 조정, PID 게인 튜닝
- [ ] `perception.md` — YOLOv8 연동, Pinhole 모델 2D→3D 변환
- [ ] `tracking.md` — 칼만 필터 상태/관측 모델, TTC 계산
- [ ] `deadlock.md` — 교착 유형 분류, 탐지 알고리즘, 해소 전략 2가지 이상
