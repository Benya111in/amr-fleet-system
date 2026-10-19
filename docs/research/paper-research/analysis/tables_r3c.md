# R3-C 표 (analyze_r3c.py 생성 — 기술 통계)

선택: CC-DWA = dwa_R3 = Z09, DWB DWB_D31, MPPI MPPI_P30, RPP RPP_R27.

## 표 1 재구성 — R3 원자료 (held-out D), Δ 는 제어기 − CC-DWA(dwa_R3) 짝 차

| 제어기 | 성공 C1 (매끈/통로/A*) | T/T_ref 중앙 | Δ시간 C1 % [95 %] | 곡선 CTE 중앙 [cm] | |정지 오차| 중앙 [cm] | C3 Δ시간 % [95 %] | C2 Δ시간 % | O 성공 /60 (접촉) | O 최소 이격 최솟값 [cm] | CPU [ms/주기] (R3) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| dwa_R3 | 60 (32/8/20) | 0.979 | – | 0.82 | 0.5 | – | – | 60 (0) | 41.8 | 1.24 |
| dwa_textbook_tuned | 38 (16/7/15) | 1.030 | +6.6 [+4.8, +8.6] (n 38) | 4.97 | 79.3 | +11.8 [+8.9, +16.0] (n 11) | +8.6 [+6.5, +11.7] (n 53) | 4 (0) | 26.2 | 1.36 |
| DWB_D31 | 60 (32/8/20) | 0.985 | +0.5 [-0.2, +1.1] (n 60) | 2.12 | 4.8 | +0.3 [-0.2, +0.9] (n 12) | +0.7 [+0.2, +1.2] (n 60) | 0 (0) | 1.3 | 19.21 |
| MPPI_P30 | 60 (32/8/20) | 1.522 | +57.2 [+53.2, +61.5] (n 60) | 1.02 | 2.4 | +58.1 [+49.1, +68.1] (n 12) | +57.3 [+53.5, +61.3] (n 60) | 58 (0) | 3.5 | 5.97 |
| RPP_R27 | 60 (32/8/20) | 0.983 | +0.3 [-0.4, +0.8] (n 60) | 0.27 | 4.4 | +0.5 [-0.1, +1.4] (n 12) | +0.5 [+0.1, +0.8] (n 60) | 0 (60) | -37.6 | 0.32 |

순위 (T/T_ref 경로 중앙, 빠른 순): C1 ['dwa_R3', 'RPP_R27', 'DWB_D31', 'dwa_textbook_tuned', 'MPPI_P30'], C2 ['dwa_R3', 'RPP_R27', 'DWB_D31', 'dwa_textbook_tuned', 'MPPI_P30']

## 표 2 — E1 변형 + E3 CC-DWA (같은 배치, held-out D)

| 변형 | 팔 | 상태 | Δ시간 C1 % [95 %] | t0.9 중앙 [s] (n) | Δt0.9 [s] | 출발 2 s 가속 재설정 /실행 | 재설정 전체 /실행 | C3 느린 /60 (실패) | C3 Δ시간 % | O 성공 /60 (접촉) | 진입 yaw > 0.05 rad | 성공 C1 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| CC-DWA | dwa_R3 | run | – | 1.30 (60) | – | 0.28 | 0.28 | 0 (0) | – | 60 (0) | 7/60 | 60/60 |
| V-meas | dwa_v_meas | run | +21.0 [+12.2, +30.4] (n 59) | 4.15 (60) | +2.85 | 0.00 | 0.00 | 1 (0) | +32.5 [+16.0, +54.3] (n 12) | 58 (1) | 18/59 | 59/60 |
| V-thr | dwa_v_thr | run | -1.6 [-6.1, +0.9] (n 60) | 1.55 (60) | +0.25 | 1.00 | 1.00 | 0 (0) | +0.7 [+0.5, +0.9] (n 12) | 60 (0) | 5/60 | 60/60 |
| V-mdl-n | dwa_v_mdl_n | run | -1.2 [-5.5, +1.1] (n 60) | 1.90 (60) | +0.60 | 8.27 | 9.07 | 0 (0) | +1.0 [+0.5, +1.6] (n 12) | 60 (0) | 6/60 | 60/60 |
| V-mdl-i | dwa_v_mdl_i | run | -1.4 [-5.6, +1.0] (n 60) | 1.65 (60) | +0.35 | 5.58 | 6.35 | 0 (0) | +0.8 [+0.4, +1.3] (n 12) | 60 (0) | 6/60 | 60/60 |
| V-K0 | dwa_v_K0 | run | +0.2 [+0.2, +0.3] (n 59) | 1.35 (60) | +0.05 | 5.18 | 5.98 | 0 (0) | -0.1 [-0.3, +0.0] (n 12) | 60 (0) | 6/59 | 59/60 |
| V-noext | dwa_v_noext | run | +0.7 [+0.4, +1.1] (n 59) | 1.30 (60) | +0.00 | 0.82 | 0.82 | 0 (0) | +0.9 [-0.2, +2.6] (n 12) | 60 (0) | 14/59 | 59/60 |
| V-nodet | dwa_v_nodet | run | -2.1 [-6.3, +0.0] (n 60) | 1.30 (60) | +0.00 | 0.13 | 0.13 | 0 (0) | +0.0 [-0.1, +0.1] (n 12) | 11 (0) | 6/60 | 60/60 |
| V-G1only | dwa_v_G1only | run | -2.0 [-5.9, +0.0] (n 60) | 1.30 (60) | +0.00 | 0.00 | 0.02 | 0 (0) | -0.1 [-0.2, -0.0] (n 12) | 60 (0) | 5/60 | 60/60 |

E3 재현 (E3 dwa_R3 C1 대 R3 dwa_R3 C1): 경로별 |시간 차| 평균 4.212 % (평균 +2.076 %, n 60), 성공 60 대 60 → 기준 밖 (기준 ≤ 0.5 %, 성공 같음)

## E1 chain (잡음 없음, stateful false)

| 변형 | 성공 | t0.9 중앙 [s] | v_m 0.3→0.6 기울기 중앙 [m/s²] | 가속 재설정 /실행 | Δ시간 % (짝, dwa_R3 기준) |
| --- | --- | --- | --- | --- | --- |
| CC-DWA | 60/60 | 1.30 | 1.00 | 0.00 | – |
| V-meas | 60/60 | 3.98 | 0.23 | 0.00 | +20.1 [+14.2, +27.1] (n 60) |
| V-thr | 60/60 | 1.56 | 0.79 | 1.00 | +0.5 [+0.4, +0.5] (n 60) |
| V-mdl-n | 60/60 | 1.30 | 1.00 | 0.00 | +0.0 [+0.0, +0.0] (n 60) |
| V-mdl-i | 60/60 | 1.30 | 1.00 | 0.00 | +0.0 [+0.0, +0.0] (n 60) |
| V-K0 | 60/60 | 1.30 | 1.00 | 0.00 | +0.0 [+0.0, +0.0] (n 60) |
| V-noext | 60/60 | 1.30 | 1.00 | 0.00 | +0.6 [+0.3, +1.0] (n 60) |
| V-nodet | 60/60 | 1.30 | 1.00 | 0.00 | +0.0 [+0.0, +0.0] (n 60) |
| V-G1only | 60/60 | 1.30 | 1.00 | 0.00 | +0.0 [+0.0, +0.0] (n 60) |

E1 chain dwa_R3 대 R3 chain dwa_R3 (결정적): {'common': 420, 'identical_metrics': 420}

## E2 — X00 (배포 규칙 동치, K 0) × held-out D C1

성공 60/60, t0.9 중앙 1.65 s (E3 dwa_R3 대비 +0.35 s), 출발 2 s 가속 재설정 2.00/실행, 재설정 전체 2.08/실행, T/T_ref 중앙 0.991

## R3 원자료에서 내는 2.2 절 수치

- ② R3 C3 X00 느린 실행 7 건 중 첫 진입 전 3 s 안 재설정 7 건, 감속 재설정 7 건
- ③ dwa_R2 (Y11) 가속 재설정 /실행: chain 0.03, ROS C1 출발 2 s 8.15 (전체 9.07)
- 시험대 간 차 (dwa_R3, ROS C1 − chain): {'n': 60, 'time_pct': [0.24126279380780602, -0.07890376843436853, 0.8453650737888285], 'time_abs_pct_max': 18.87269681883028, 'cte_cm': [-0.0352789434652403, -0.07737139285772003, 0.014345891823768818], 'cte_abs_cm_max': 0.8941570586580603}

## E4 동치

e4eq 통과 여부: {'dwa_R3': True, 'DWB_D31': True, 'MPPI_P30': False, 'RPP_R27': True, 'dwa_textbook_tuned': True}
e4eq2 (MPPI, 순서 엇갈림): 통과 False, 경로별 (시간 %, CTE cm) {'S01_pwc': (3.933, -0.322), 'S02_pwl': (-5.926, -0.215), 'S03_pwc': (-4.891, -0.022), 'S04_pwl': (4.621, 0.171)}

## 계산 시간 — R3 C1 cpu_ms_per_cycle (E4 취소, 편차 D1)

| 팔 | 경로 | 경로 평균의 평균 [ms] | 경로 중앙 [ms] |
| --- | --- | --- | --- |
| dwa_R3 | 60 | 1.240 | 1.183 |
| dwa_textbook_tuned | 60 | 1.358 | 1.460 |
| DWB_D31 | 60 | 19.207 | 19.210 |
| MPPI_P30 | 60 | 5.971 | 4.527 |
| RPP_R27 | 60 | 0.319 | 0.324 |
- ratio_dwa_R3_over_DWB_D31: 0.065 [0.059, 0.070] (n 60)
- ratio_dwa_R3_over_MPPI_P30: 0.208 [0.182, 0.239] (n 60)
- ratio_dwa_R3_over_RPP_R27: 3.890 [3.329, 4.699] (n 60)
- ratio_dwa_R3_over_dwa_textbook_tuned: 0.913 [0.835, 1.009] (n 60)
- CC-DWA 후보·굴림: {'runs': 24, 'n_samples_med': 342.0, 'rollout_steps_med': 15.0, 'source': 'r3c/e4eq dwa/stats[1], ⌈T_sim(v)/0.1⌉'}

## E5

```json
{
 "P1_P3_chain": {
  "P1": {
   "a": 1.0,
   "j": 2.0,
   "d": 0.04,
   "tau": 0.08,
   "g_formula": 0.37,
   "gap_ctrl_max": 0.3651,
   "t_gap_ctrl_max": 1.0,
   "gap_ctrl_at_ramp_end": 0.3651,
   "gap_cont_max": 0.4049
  },
  "P3": [
   {
    "v0": 0.4,
    "v_steady": 0.4,
    "d_meas": 0.221,
    "t_stop": 1.69,
    "d_eq4_tc0": 0.1696,
    "d_trapezoid": 0.08,
    "d_eq4_tc_servo": 0.2176,
    "d_eq4_tc0.3": 0.2896,
    "err_eq4_tc0": 0.0514,
    "err_trapezoid": 0.141,
    "err_eq4_tc_servo": 0.0034
   },
   {
    "v0": 0.6,
    "v_steady": 0.6,
    "d_meas": 0.3929,
    "t_stop": 1.88,
    "d_eq4_tc0": 0.3196,
    "d_trapezoid": 0.18,
    "d_eq4_tc_servo": 0.3916,
    "d_eq4_tc0.3": 0.4996,
    "err_eq4_tc0": 0.0733,
    "err_trapezoid": 0.2129,
    "err_eq4_tc_servo": 0.0013
   },
   {
    "v0": 0.8,
    "v_steady": 0.8,
    "d_meas": 0.603,
    "t_stop": 1.68,
    "d_eq4_tc0": 0.5096,
    "d_trapezoid": 0.32,
    "d_eq4_tc_servo": 0.6056,
    "d_eq4_tc0.3": 0.7496,
    "err_eq4_tc0": 0.0934,
    "err_trapezoid": 0.283,
    "err_eq4_tc_servo": -0.0026
   },
   {
    "v0": 1.0,
    "v_steady": 1.0,
    "d_meas": 0.8547,
    "t_stop": 2.17,
    "d_eq4_tc0": 0.7396,
    "d_trapezoid": 0.5,
    "d_eq4_tc_servo": 0.8596,
    "d_eq4_tc0.3": 1.0396,
    "err_eq4_tc0": 0.1151,
    "err_trapezoid": 0.3547,
    "err_eq4_tc_servo": -0.0049
   }
  ]
 },
 "P4": [
  "[P4] bound_cycles=12.000 (theta_h 0.6, a 1.00, dt_c 0.050)",
  "[P4] g1_on g1=1 K=0 f1=0 onset=1 reset=13 cycles=12 time_s=0.60 reason=hard v_last_at_reset=0.650",
  "[P4] g1_on g1=1 K=0 f1=1 onset=1 reset=13 cycles=12 time_s=0.60 reason=hard v_last_at_reset=0.650",
  "[P4] g1_on g1=1 K=6 f1=0 onset=1 reset=13 cycles=12 time_s=0.60 reason=hard v_last_at_reset=0.650",
  "[P4] g1_on g1=1 K=6 f1=1 onset=1 reset=13 cycles=12 time_s=0.60 reason=hard v_last_at_reset=0.650",
  "[P4] g1_on g1=1 K=10 f1=0 onset=1 reset=13 cycles=12 time_s=0.60 reason=hard v_last_at_reset=0.650",
  "[P4] g1_on g1=1 K=10 f1=1 onset=1 reset=13 cycles=12 time_s=0.60 reason=hard v_last_at_reset=0.650",
  "[P4] g1_off g1=0 K=0 f1=0 onset=1 reset=7 cycles=6 time_s=0.30 reason=threshold v_last_at_reset=0.350",
  "[P4] g1_off g1=0 K=0 f1=1 onset=1 reset=9 cycles=8 time_s=0.40 reason=threshold v_last_at_reset=0.450",
  "[P4] g1_off g1=0 K=6 f1=0 onset=1 reset=12 cycles=11 time_s=0.55 reason=persist v_last_at_reset=0.600",
  "[P4] g1_off g1=0 K=6 f1=1 onset=1 reset=13 cycles=12 time_s=0.60 reason=hard v_last_at_reset=0.650",
  "[P4] g1_off g1=0 K=10 f1=0 onset=1 reset=13 cycles=12 time_s=0.60 reason=hard v_last_at_reset=0.650",
  "[P4] g1_off g1=0 K=10 f1=1 onset=1 reset=13 cycles=12 time_s=0.60 reason=hard v_last_at_reset=0.650"
 ],
 "P1_ros_start_gap_dwa_R3": {
  "med": 0.51555,
  "p10": 0.37556999999999996,
  "p90": 0.5475099999999999,
  "max": 0.5564
 },
 "P1_ros_hard_accel_resets": {
  "c1": 0,
  "c3": {
   "runs": 60,
   "hard_accel": 13
  },
  "o": {
   "runs": 60,
   "hard_accel": 5
  }
 },
 "P2": {
  "formula_ratio": 0.27,
  "a": 1.0,
  "ros_slope_med": 0.22222222222222224,
  "chain_slope_med": 0.234375,
  "ros_slope_med_dwa_R3": 0.9999999999999998,
  "chain_slope_med_dwa_R3": 1.0000000000000002
 }
}
```

## E6 그림 2 (held-out D)

```json
{
 "path": "A04",
 "score": 0.0,
 "medians": {
  "hold": 1.3,
  "thr": 1.55,
  "mdl": 1.9,
  "rpp": 1.3
 },
 "n_common": 60,
 "t09_on_path": {
  "hold": 1.3,
  "thr": 1.55,
  "mdl": 1.9,
  "mid": 1.8499999999999999,
  "rpp": 1.3
 },
 "rows": 53,
 "sources": {
  "hold": [
   "r3c/c1",
   "dwa_R3"
  ],
  "thr": [
   "r3c/c1",
   "dwa_v_thr"
  ],
  "mdl": [
   "r3c/c1",
   "dwa_v_mdl_n"
  ],
  "mid": [
   "r3c/c1",
   "dwa_v_mdl_i"
  ],
  "rpp": [
   "r3/c1",
   "RPP_R27"
  ]
 }
}
```

## 실시간 품질

| 단계 | 실행 | 명령 주기 최소 [Hz] | 18 Hz 미만 | 참값 트랙 구독 ≠ 0 |
| --- | --- | --- | --- | --- |
| r3/tuning | 7224 | 18.72 | 0 | 0 |
| r3/c1 | 720 | 19.70 | 0 | 0 |
| r3/c2 | 720 | 19.74 | 0 | 0 |
| r3/c3 | 780 | 19.73 | 0 | 0 |
| r3/o | 720 | 19.70 | 0 | 0 |
| r3c/e2 | 60 | 20.00 | 0 | 0 |
| r3c/c1 | 540 | 19.99 | 0 | 0 |
| r3c/c3 | 540 | 19.99 | 0 | 0 |
| r3c/o | 540 | 19.99 | 0 | 0 |
| r3c/e4eq | 120 | 19.80 | 0 | 0 |
| r3c/e4eq2 | 24 | 20.00 | 0 | 0 |
| r3c/smoke_timed | 5 | 20.00 | 0 | 0 |
