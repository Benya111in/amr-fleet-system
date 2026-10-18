# 3 라운드 튜닝 격자 (prereg_r3.md §2·§3) — 3 라운드 튜닝 결과를 보기 전에 커밋

## 0. 원칙
- **누적 예산을 같게 한다.** 우리 DWA 계열은 3 라운드 동안 X 12 + Y 12 + Z 12 = 36 설정을 받았다. Nav2 세 팔에도 1 라운드 격자 12 개(2 라운드 튜닝 자료를 그대로 쓴다)에 새 설정 24 개를 더해 36 으로 맞춘다.
- **각 팔이 공정하게 강해지도록 만든다.**
  - 손잡이는 Nav2 Humble 문서(nav2_bringup 예시·플러그인 README)에 있는 이름만 쓴다.
  - 값은 1·2 라운드 원자료에서 드러난 그 팔의 약점을 고치는 방향으로 고른다.
  - 2 라운드 선택 설정(DWB D10, MPPI P05, RPP R00)을 출발점으로 삼는다. 거기에 한두 손잡이를 바꾸고, 셋은 조합한다.
- **이름이 틀리면 Nav2 는 조용히 무시한다.**
  - 그래서 1 라운드 블록에 이미 있는 키와, 문서에 있는 critic 이름만 쓴다. 이미지 `amr-ai/rl:0.3` 의 `dwb_critics/default_critics.xml` 과 `nav2_mppi_controller/critics/*.hpp` 로 이름을 확인했다.
  - 이름을 잘못 써서 기본값과 같게 된 설정이 없는지는 튜닝 결과의 비트 동일 여부로 다시 확인한다 (결과 표에 적는다).
- 값의 출처는 `scripts/params_r3.py` (DWB_NEW·MPPI_NEW·RPP_NEW·build_z) 이고, 이 표와 같다.

## 1. 1·2 라운드에서 본 각 팔의 약점 (근거)

| 팔 | 약점 | 근거 (원자료) |
| --- | --- | --- |
| RPP | 경로 위 정적 원판에 **부딪힌다** | O 충돌 60/60 (1R·2R 모두, 최대 침투 50 cm). 충돌 검사(1 s 앞, carrot 까지)가 프로파일러 감속 거리보다 짧다 [추론] |
| RPP | 목표 앞 4.5 cm 에 선다 | 정지 오차 중앙 −4.5 cm (1R·2R) |
| DWB | 원판에 **부딪힌다** | O 충돌 D02 60/60, D00 58/60. BaseObstacle 은 로봇 중심 셀 비용만 보고 scale 0.02 로 작다 [문서: ObstacleFootprint 는 풋프린트 전체를 본다] |
| DWB | 추종이 거칠다 | 곡선 CTE 평균 2.1–2.6 cm (DWA 1.0, RPP 0.3), 저크 p99 63–66 |
| MPPI | **느리다** | T/T_ref 1.54 (DWA 0.99, RPP 0.98). 0.9 m/s 에 거의 닿지 않는다 |
| MPPI | O 우회는 Nav2 가운데 가장 낫다 | O 34/60 (1R) · 48/60 (2R) — 이 장점은 지킨다 |

## 2. DWB 새 24 (DWB_D12–D35; 기준 D10 = DWB_E + sim_time 1.0, 일부 D02 = sim_time 1.0)

| id | 바꾼 것 (기준 위) | 이유 |
| --- | --- | --- |
| D12 | critics 의 BaseObstacle → **ObstacleFootprint** (scale 0.02) | 풋프린트 전체로 충돌을 본다 → O 충돌 |
| D13 | BaseObstacle.scale 0.2 | 장애물 비용 가중을 10 배 → 원판 앞 회피 유도 |
| D14 | BaseObstacle.scale 0.5 | 위의 더 강한 판 |
| D15 | ObstacleFootprint, scale 0.1 | 풋프린트 + 가중 5 배 |
| D16 | (D02 위) ObstacleFootprint 0.02 | 1R 선택 D02 쪽에서도 같은 처방 |
| D17 | ObstacleFootprint 0.02 + sim_time 1.5 | 더 먼 앞을 보며 피한다 |
| D18 | BaseObstacle 0.2 + sim_time 1.5 | 같은 조합, 중심 비용판 |
| D19 | PathDist 64 + PathAlign 16 | 추종 강화 (경로 거리 ↑, 정렬 ↓로 곡선 안쪽 가로지름 줄임) |
| D20 | PathAlign·GoalAlign forward_point_distance 0.325 | 문서 예시 값 — 앞쪽 정렬점을 멀리 해 곡선 추종 매끈 |
| D21 | GoalAlign.scale 12 | 목표 정렬 과잉 억제 (목표 부근 흔들림) |
| D22 | vx_samples 30, vtheta_samples 60 | 표본 해상도 ↑ → 추종 오차·저크 ↓ |
| D23 | linear_granularity 0.025, angular_granularity 0.0125 | 궤적 평가 해상도 ↑ |
| D24 | sim_time 1.2 | 1R 의 1.0 / 1.5 사이 |
| D25 | sim_time 0.8 | 짧은 지평 → 빠른 반응·추종 |
| D26 | trans_stopped_velocity 0.05 | 정지 판단 문턱 ↓ → 목표 부근 감속 덜함 |
| D27 | RotateToGoal.slowing_factor 2.0 | 목표 접근 감속 완화 |
| D28 | PathDist 64 + GoalDist 32 | 추종 + 목표 당김 균형 |
| D29 | ObstacleFootprint 0.02 + PathDist 64 | 충돌 + 추종 동시 |
| D30 | (D02 위) BaseObstacle 0.2 | D02 쪽 장애물 가중 |
| D31 | ObstacleFootprint 0.02 + Twirling (scale 2) | 제자리 회전 억제 (흔들림·저크) |
| D32 | ObstacleFootprint 0.05 + GoalDist 32 | 충돌·목표 당김 |
| D33 | BaseObstacle 0.2 + PathDist 64 | 장애물 가중 + 추종 |
| D34 | ObstacleFootprint 0.02 + vx 30·vθ 60 | 충돌 + 표본 해상도 |
| D35 | BaseObstacle 0.5 + sim_time 1.5 + PathDist 64 | 강한 장애물 회피 + 추종 |

## 3. MPPI 새 24 (MPPI_P12–P35; 기준 P05 = vx_std 0.3 · wz_std 0.6)

| id | 바꾼 것 (P05 위) | 이유 |
| --- | --- | --- |
| P12 | PathFollowCritic weight 10, offset_from_furthest 8 | 경로 앞쪽으로 끄는 힘 ↑ → 속도 |
| P13 | PathFollowCritic offset_from_furthest 10 | 더 먼 목표점 → 속도 |
| P14 | vx_std 0.4 | 빠른 속도 쪽 표본 탐색 ↑ |
| P15 | vx_std 0.5, wz_std 0.8 | 위의 더 넓은 판 |
| P16 | time_steps 80 (지평 4.0 s) | 긴 지평 → 감속 덜함 |
| P17 | model_dt 0.07, time_steps 50 (3.5 s) | 더 긴 지평, 적은 계산 |
| P18 | temperature 0.15 | 더 탐욕적인 가중 평균 |
| P19 | temperature 0.5 | 반대 방향 (부드러움) — 대조 |
| P20 | gamma 0.05 | 제어 변화 벌점 ↑ (문서: 부드러움·속도 안정) |
| P21 | iteration_count 2 | 반복 최적화 ↑ |
| P22 | batch_size 3000 | 표본 수 ↑ |
| P23 | PathFollowCritic weight 15 | 앞쪽 당김 강하게 |
| P24 | PreferForwardCritic weight 10 | 전진 선호 ↑ |
| P25 | GoalCritic threshold 1.0, GoalAngleCritic threshold 0.25 | 목표 접근 비평 늦게 켬 → 감속 구간 짧게 |
| P26 | PathAngleCritic weight 4 | 경로 방향 정렬 ↑ (추종) |
| P27 | CostCritic weight 2.0 | 장애물 비용 가중 ↓ → 덜 조심 (속도) |
| P28 | vx_std 0.4 + PathFollow 10·offset 8 | 속도 조합 |
| P29 | PathAlignCritic offset 30, max_path_occupancy_ratio 0.07 | 경로 정렬 기준점 멀리 |
| P30 | PathAlign 20 + PathFollow 10 + vx_std 0.4 | 1R P10 쪽 + 속도 |
| P31 | vx_std 0.4 + temperature 0.15 | 속도 조합 |
| P32 | wz_std 0.8 | 회전 탐색 ↑ |
| P33 | vx_std 0.4 + PathFollow 15·offset 10 + time_steps 70 | 가장 공격적인 속도 조합 |
| P34 | GoalCritic weight 10 | 목표로 당김 ↑ |
| P35 | vx_std 0.5 + PathFollow 10·offset 8 + gamma 0.05 | 넓은 탐색 + 부드러움 |

## 4. RPP 새 24 (RPP_R12–R35; 기준 R00 = 기본, 일부 R03 = 속도 비례 lookahead 0.4–1.2 m · 0.8 s)

RPP 는 회피 기능이 없다. 경로 위 장애물에서 얻을 수 있는 최선은 **부딪히지 않고 멈추는 것**이다 (O 성공은 원리상 0).
J'' 의 O 항은 실패에 똑같이 10 을 주므로 충돌 처방은 선택 점수를 바꾸지 않는다. 그래도 안전 기술 지표(접촉)를 위해 넣는다.

| id | 바꾼 것 | 이유 |
| --- | --- | --- |
| R12 | max_allowed_time_to_collision_up_to_carrot 2.0 | 충돌 검사 지평 ↑ → O 충돌 |
| R13 | 같은 것 3.0 | 더 길게 |
| R14 | use_cost_regulated_linear_velocity_scaling true, cost_scaling_dist 0.6 | 장애물 가까이 감속 → 충돌 대신 정지 |
| R15 | 같은 것, cost_scaling_dist 1.0 | 더 일찍 감속 |
| R16 | 충돌 지평 3.0 + 비용 감속 0.6 | 둘 다 |
| R17 | lookahead_dist 0.8 | 고정 lookahead ↑ (추종 매끈) |
| R18 | 속도 비례 lookahead, time 1.0, 0.5–1.5 m | 문서 권장 속도 비례 |
| R19 | (R03 위) lookahead_time 0.6 | 짧은 시간 비례 |
| R20 | use_interpolation true | 문서: lookahead 점 보간 → 추종 |
| R21 | approach_velocity_scaling_dist 0.6 | 목표 감속 구간 짧게 (시간) |
| R22 | 접근 0.8 + min_approach_linear_velocity 0.1 | 감속 구간 짧고 최저 속도 ↑ → 정지 오차·시간 |
| R23 | min_approach_linear_velocity 0.1 | 목표 앞 기어감 줄임 |
| R24 | approach_velocity_scaling_dist 0.4 | 더 짧게 |
| R25 | regulated_linear_scaling_min_radius 0.7 | 곡률 감속 덜함 (속도) |
| R26 | regulated_linear_scaling_min_speed 0.4 | 곡선 최저 속도 ↑ |
| R27 | regulated_linear_scaling_min_radius 1.2 | 곡률 감속 더함 (추종) — 대조 |
| R28 | rotate_to_heading_min_angle 0.5 | 제자리 회전 문턱 |
| R29 | max_angular_accel 3.0 | 회전 응답 ↑ |
| R30 | 충돌 지평 2.0 + 보간 | 안전 + 추종 |
| R31 | (R03 위) 충돌 지평 2.0 | 1R 선택 쪽 안전 |
| R32 | 접근 0.6 + 최저 0.1 | 시간·정지 조합 |
| R33 | (R03 위) 비용 감속 0.6 | 1R 선택 쪽 안전 |
| R34 | 충돌 2.0 + 비용 감속 0.6 + 접근 0.8 | 안전·시간 조합 |
| R35 | 충돌 3.0 + lookahead 0.8 + 보간 | 안전·추종 조합 |

## 5. DWA-R3 격자 Z (prereg_r3 §3 그대로; 모두 Y11 위)

Y11 = R1 K 6 + F1 θ 0.10 + F2 D 1.0 + F3 S 1.0. G1 = `research.accel_hold` (pr-core CORE R3 READY 이름).
F1 ROS 보정값 d*·τ* 는 `analysis/f1_fit_r3.json` (`scripts/fit_f1_model.py`; 튜닝-20 의 2 라운드 ROS 궤적만) 이다.

| id | 설정 |
| --- | --- |
| Z00 | Y11 그대로 |
| Z01 | F1 끔 (reset_model false) |
| Z02 | Z01 + G1 |
| Z03 | F1 보정값 (model_delay d*, model_tau τ*), θ 0.10 |
| Z04 | Z03, θ 0.15 |
| Z05 | Z03 + G1 |
| Z06 | Z02 + velocity_weight (γ) 0.6 |
| Z07 | Z02 + γ 0.8 |
| Z08 | Z02 + blocked_detour_shift (F3 S) 0.8 |
| Z09 | Z05 + γ 0.6 |
| Z10 | Z02 + reset_persist_cycles (R1 K) 10 |
| Z11 | Z02 + window_reset_v 0.4 |

## 6. 선택
- 규칙: 2 라운드와 같은 J''·S'' (`scripts/analyze_r2.py` 의 식).
  - 자료는 튜닝-20 × {잡음 없음, seed 1–3} + 튜닝 O 6 이다.
- 후보:
  - DWB·MPPI·RPP: 각 36 개다. 1 라운드 격자 12 는 2 라운드 튜닝 원자료(`runs/r2/tuning`)를 쓰고, 새 24 는 3 라운드 원자료(`runs/r3/tuning`)를 쓴다.
  - DWA-R3: Z00–Z11 (3 라운드) 와 Y11 (2 라운드 원자료) 13 개다.
  - 교과서 팔은 T06 고정이다.
- 선택 결과를 커밋한 뒤에만 held-out D 를 돈다.
