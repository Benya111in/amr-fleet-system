# 원고 기구 서술 점검표 (R3-C, prereg_r3c §6)

- 대상: `/home/CAPYI_AI/paper/cc-dwa/main.tex` §2–§3 의 기구 문장과 그림 캡션(`fig/chain.tex`·`startup.tex`·`terminal.tex`), 알고리즘(`fig/algo.tex`, 현재 main.tex 에 \input 되지 않음). 원고는 읽기만 했다.
- 대조 기준
  - 설정: 배포 `src/amr_navigation/config/nav2_params.yaml`(DWA 블록) 위에 R3 선택 설정(아래 §0)을 덮은 CC-DWA(= dwa_R3) 블록. 연구 스위치 기본값은 `src/amr_nav_research/README.md` 표.
  - 코드: `src/amr_nav_research/src/dwa_x.cpp` (DwaPlannerX, 빌드 eab6f8e = R3·R3-C ROS 빌드). 줄 번호는 커밋 88e8e3b 기준.
  - 시험대: `src/amr_navigation/scripts/kinematic_sim.py`, `launch/kinematic_sim.launch.py`, `config/sensors.yaml`·`robot_params.yaml`, `src/amr_navigation/config/velocity_profiler.yaml`, 이미지 amr-ai/rl:0.3 의 Nav2 패키지.
  - 실측: R3-C E5 (P1·P3·P4, `runs/r3c/e5`, 봉인), smoke_timed (봉인).
- 판정: **일치** / **부분** (뜻은 맞으나 빠진 조건·단서가 있음) / **불일치** (원고를 고쳐야 함).

## 0. 선택 설정 (tuning_selection_r3.json)

@@SELECTION@@

## 1. 이미 알려진 불일치 (prereg_r3c §6, 설계 중 발견) — 현재 원고 상태

| # | 원고 위치 | 원래 문장 | 사실 (키·코드) | 현재 main.tex |
| --- | --- | --- | --- | --- |
| K1 | 3.2 | "굴림은 가속·저크 한계로 그 명령까지 올라가는 궤적" | `sustained_sampling: false` (nav2_params.yaml:135, Z 격자·선택 모두) → `rollout()` 등속 원호 (dwa_x.cpp:613–625, 분기 1236–1263). `rolloutRamp` 은 쓰이지 않는다 | **고쳐짐**: "등속 원호로 굴린다" |
| K2 | 3.2 | "v_des 로 제한한다" | `maxSpeedForStop(d_g, decel, jerk, approach_latency)` 값은 속도 항 \|v_des − v\|/v_span 의 목표(부드러운 항)로만 쓰인다 (dwa_x.cpp:1171–1177, 1393). 하드 제한 아님 | **고쳐짐**: "속도 항의 목표 속도는 … 로 둔다" |
| K3 | 5.5 | "저크 램프를 적분하는 굴림은 원호 굴림보다 무겁다" | K1 과 같은 이유로 성립하지 않음 | **고쳐짐**: 비교 대상이 교과서 DWA·MPPI (E4) |
| K4 | 3.2 식 (4) | t_c | `approach_latency: 0.3` (nav2_params.yaml:140) | **일치**: "t_c = 0.3 s" |

## 2. §2 구동 사슬

| # | 원고 문장 (요지) | 키·값 / 코드 | 판정 | 비고 |
| --- | --- | --- | --- | --- |
| 2-1 | 풋프린트 0.60 × 0.40 m 차동 구동 | robot_params.yaml footprint_length 0.60·width 0.40; chain_sim_x kHalfL 0.30·kHalfW 0.20 | 일치 | |
| 2-2 | 선속도 0–1.0 m/s (후진 없음), ω 1.5 rad/s, a = 1.0 m/s² | DWA max_vel_x 1.0·min_vel_x 0.0; limits max_angular_velocity 1.5·max_linear_acceleration 1.0 (robot_params.yaml:59–60) | 일치 | 하드웨어 상한 2.0 m/s 는 운용 1.0 으로 묶임 |
| 2-3 | 제어 Δt_c = 0.05 s (20 Hz) | controller_frequency 20.0 | 일치 | 실측 명령 주기 표: results_r3c.md |
| 2-4 | 저크 j = 2 m/s³ 의 50 Hz 프로파일러 | velocity_profiler.yaml rate 50.0; limits max_linear_jerk 2.0 | 일치 | 프로파일러에는 PI 보정·재동기화도 있다 (velocity_profiler.hpp) — 원고는 생략 |
| 2-5 | 100 Hz 서보 (지연 0.04 s, 1차 0.08 s, 가속 제한) | kinematic_sim.py rate 100, servo_delay 0.04, servo_time_constant 0.08, max_linear_accel 1.0 (줄 120–127) | 일치 | chain_sim_x Servo(0.08, 0.04, 1.0) 같음 |
| 2-6 | 목표 판정 위치 0.10 m·방위 0.05 rad 같은 순간 | general_goal_checker xy 0.10·yaw 0.05·stateful False | 일치 | C2 는 stateful true (4.3 에 정의) |
| 2-7 | 식 (1) g(a) = a²/2j + a(d+τ) = 0.37 m/s | 개루프 램프 실측 (P1, chain): 램프 끝 제어 주기 괴리 0.365 m/s | 일치 | ROS 분포는 results_r3c.md E5 P1 |
| 2-8 | 측정 기준 창은 괴리를 aΔt_c 로 묶어 a_e ≈ 0.27a (식 해) | 식: a_e²/4 + 0.12a_e = 0.05 → 0.2675 | 일치 (식) | 실측 (P2): results_r3c.md |
| 2-9 | 임계 재설정 규칙 \|v_c − v_m\| > θ 이면 측정 기준 | 원본 규칙 = K 0: windowCenterX 비-F1 분기 (dwa_x.cpp:182–197) | 부분 | 원본 규칙은 ω 조건(\|w_c − w_m\| > 0.5)도 함께 본다 (dwa_x.cpp:183) |
| 2-10 | ③ 모형 \|v_m − v̂\| 판정 | F1 분기 (dwa_x.cpp:141–179): 후보 = \|v_m − v̂\| > θ **또는** \|w_m − ŵ\| > θ_w, hard 는 원래 차 | 일치 | |

## 3. §3 CC-DWA

| # | 원고 문장 (요지) | 키·값 / 코드 | 판정 | 비고 |
| --- | --- | --- | --- | --- |
| 3-1 | 창 중심 v̄ = 직전 명령, 식 (2) 일 때만 측정값 | windowCenterX (dwa_x.cpp:126–214) | @@31@@ | |
| 3-2 | H_k = (v_c^{k-1} > v_c^{k-2}) ∧ (v_c^{k-1} > v_m) ∧ (\|·\| ≤ θ_h) | dwa_x.cpp:139–140 (`accel_hold && has_v_last_prev && v_last > v_last_prev && v_last > v_meas && \|gap\| ≤ reset_hard_v`) | 일치 | 직전 두 주기 명령을 모르면(시작·예외 뒤) 유예하지 않는다 |
| 3-3 | 막힘·와인드업은 큰 괴리 규칙 θ_h = 0.6 이 잡는다 | reset_hard_v 0.6 (기본, Z 격자에서 안 바꿈); G1 켜면 K 와 무관하게 즉시 hard (dwa_x.cpp:194–195) | 일치 | P4: 측정 0 고정에서 12 주기(0.60 s) 뒤 hard (명령 0.65 m/s), 상한 θ_h/(aΔt_c) = 12 |
| 3-4 | **감속 중에는 유예하지 않는다** | (a) 명령 증가 조건 (dwa_x.cpp:139) — 명령이 줄거나 같으면 hold false | 일치 | test_dwa_x_round3 DoesNotInterveneWhileDecelerating: 무작위 4 000 판정에서 G1 끔과 같고, 잡음 폐루프 감속 주기 유예 0 |
| 3-5 | **각속도 괴리는 유예 대상이 아니다** (θ_ω 0.5 rad/s) | `within = (v_in \|\| hold) && w_in` (dwa_x.cpp:184) — ω 차가 window_reset_w 를 넘으면 유예 조건이어도 후보 | 일치 | 원고 식 (2) 에는 ω 항이 없다 → 식 (2) 아래 "ω 차 > 0.5 rad/s 도 같은 후보로 센다" 한 줄 권고 |
| 3-6 | K 주기 지속 조건이 잡음의 순간 괴리를 거른다 | reset_persist_cycles (R1), 연속 수는 차가 임계 안으로 들거나 유예된 주기에 0 (dwa_x.cpp:186–192) | 부분 | 유예된 주기는 "건너뜀" 이 아니라 연속 수를 0 으로 되돌린다 — 식 (2) 의 "K주기 연속 ∧ ¬H_k" 표기보다 강한 규칙 |
| 3-7 | θ = 0.3 m/s (ω 0.5 rad/s), K = □ | window_reset_v·window_reset_w, research.reset_persist_cycles | @@37@@ | |
| 3-8 | 와인드업은 늦어도 θ_h/a = 0.6 s 안에 큰 괴리 규칙이 재설정 | P4 gtest (test_r3c_windup): G1 켬 K 0·6·10 × F1 끔·켬 모두 와인드업 시작 뒤 12 주기 = 0.60 s 에 hard | 일치 | 재설정은 13 번째 주기(차 0.65 > 0.6, 엄격 부등호) — "0.6 s 안" 은 시작 주기부터 잰 경과 시간 |
| 3-9 | 정상 가속 괴리 0.37 < θ_h 라 큰 괴리 규칙이 가속 중 잘못 걸리지 않는다 | P1 ROS: E3 dwa_R3 C1 가속 국면 hard 재설정 수 | @@39@@ | prereg_r3c §3 E5: 이 문장은 이 수가 0 일 때만 남긴다 |
| 3-10 | 후보는 지평 clip(\|v\|/a + 0.5, 1.5, 2.5) s 등속 원호 | simTime (dwa_x.cpp:77–81), rollout (613–625), sim_dt 0.1 | 부분 | v ≤ v_max = 1.0, a = 1.0 이면 \|v\|/a + 0.5 ≤ 1.5 → 지평은 **늘 1.5 s (15 스텝)** (E4 굴림 스텝 중앙 15). 식은 맞지만 실효 상수임을 밝히는 편이 정확 |
| 3-11 | 창 [v̄ ± aΔt_c] × [ω̄ ± ω̇_max Δt_c] (알고리즘 1) | dynamicWindow (dwa_x.cpp:528–556): v 하한은 decel_lim_x(= 1.0) | 일치 | 후보 = vx 11 × vth 31 (+ ω 0 열) + 제동 1 → 주기당 342 안팎 (E4 실측 중앙) |
| 3-12 | 속도 항 목표 v_des = d_stop⁻¹(d_g) | dwa_x.cpp:1171–1177; 그 뒤 헤딩 cos 감속(1189)·정렬 구간 0(1193)·v_cap 상한 | 부분 | v_des 는 접근 상한 외에 헤딩 오차 감속(×cos)도 받는다 — 원고에 없음 |
| 3-13 | 식 (4) d_stop(v) = v(t_c+t_R) − j t_R³/6 + (v − a²/2j)²/2a, t_R = a/j | SpeedProfile::stoppingDistance (speed_profile.cpp:28–51) | 일치 | P3 (chain, t_c=0): v=1.0 실측 0.855 m, 식 (4) 0.740 (+ 서보 몫 0.12 → 0.860), v²/2a 0.500 |
| 3-14 | v²/2a 는 v = 1 에서 지연 몫을 빼고도 0.24 m 과소 추정 | 식 (4)(t_c=0) − v²/2a = 0.7396 − 0.5 = 0.240; 실측 − 서보 몫(0.12) − 0.5 = 0.235 | 일치 | |
| 3-15 | 사슬 지연·저크는 굴림 대신 정지 제약에 넣는다 | approach_latency 0.3 = 저크 램프 a/2j 0.25 + 서보 ≈ 0.3 (nav2_params.yaml:140 주석) | 일치 | t_c 는 d+τ(0.12) 가 아니라 램프 지연을 묶은 0.3 |
| 3-16 | 목표 거리 D 안에서 방향·경로 항 목표점을 경로 끝 접선 연장선 위로 | goal_extension_dist; ext_on (dwa_x.cpp:969–972), goalExtension (487–503); 목표점 = 투영 + ℓ (경로 끝을 넘을 때만 연장선 위) | 일치 | |
| 3-17 | 목표를 지나친 뒤에는 끈다 | ext_on 조건 (목표 yaw 축 위치 < 0) | 일치 | |
| 3-18 | 로봇 앞 3 m 경로 점에 발자국 충돌 → 막힘 | blocked_lookahead 3.0 (기본); detourPath 검사 구간 [s_robot − 0.3, s_robot + 3.0] (dwa_x.cpp:331–345, 조건 334·337) | 일치 | 뒤 0.3 m 여유 포함 |
| 3-19 | **비용이 낮은 쪽**으로 최대 S m 옆으로 민다 | 쪽마다 d = 0.05…S 중 비충돌·중심 비용 최대가 가장 낮은 d (dwa_x.cpp:404–446); 두 쪽이 가능하면 비용 낮은 쪽, 단 **직전 주기 쪽이 가능하면 그 쪽 유지** (450–455) | 부분 | 이력(쪽 유지) 단서가 원고에 없다 |
| 3-20 | **앞뒤를 부드러운 경사로 합류** | smoothstep 램프, 길이 max(1, 2\|d\|) m, 목표 쪽 램프는 목표 전에 끝 (dwa_x.cpp:378–402) | 일치 | |
| 3-21 | **양쪽이 모두 막히면 제동 후보로 멈춘다** | 양쪽 불가 → `return false` = 원래 경로 사용 (dwa_x.cpp:447–449). 정지는 DWA 충돌 검사(1265–1282)가 후보를 버리고, 유효 후보가 없으면 제동 후보(1601–1606) | 부분 | 우회 규칙 자체에는 제동 분기가 없다 — "원래 경로로 돌아가고, 충돌 검사가 후보를 거르므로 제동 후보로 멈춘다" 가 정확. 목표 자체가 막힌 경우도 우회하지 않는다 (374–376) |
| 3-22 | 우회 경로를 그 주기의 추종 기준으로 | applyDetour 가 path 를 바꾼다 (dwa_x.cpp:947–950) — 헤딩·경로·여유 기준 모두 | 일치 | 우회 끝 점·d 를 기억해 되돌아오는 램프가 끝날 때까지 잇는다 (memory) |
| 3-23 | 평가 비용 = 방향·경로·이탈·상대 여유·속도 항의 가중합 | c.cost (dwa_x.cpp:1489–1492) = heading·clearance(상대)·velocity·path·**oscillation**·**dynamic**·off_path·escape | 부분 | 진동 항(oscillation_weight 0.5, 방향·회전 뒤집힘에만 1)·동적 항(참값 트랙 차단으로 시험대에서 0)·통로 탈출 항이 빠져 있다 |
| 3-24 | 알고리즘 1 "유효 후보 없으면 제동 명령" | dwa_x.cpp:1601–1606 (brake_candidate true) | 일치 | |

## 4. §4 실험 방법 (시험대)

| # | 원고 문장 | 근거 | 판정 | 비고 |
| --- | --- | --- | --- | --- |
| 4-1 | ROS 2 Humble · Nav2 1.1.20 | 이미지 amr-ai/rl:0.3: nav2_controller·nav2_mppi_controller·nav2_regulated_pure_pursuit_controller·dwb_core package.xml 1.1.20 | 일치 | |
| 4-2 | 실제 controller_server 에 플러그인 | 시험대 = navigation.launch.py 의 controller_server, 팔마다 controller_id | 일치 | |
| 4-3 | 프로파일러 노드와 같은 서보 모형 | velocity_profiler_node (50 Hz) → kinematic_sim (cmd_topic cmd_vel_smoothed, 서보 0.04/0.08) | 일치 | 안전 노드는 사슬에 없다 (원고 한계에 있음) |
| 4-4 | LiDAR σ = 0.03 m 코스트맵 | sensors.yaml lidar.noise_stddev 0.03, 720 빔 → kinematic_sim scan_noise_std | 일치 | |
| 4-5 | 명령 주기 □ Hz 이상 | results_r3c.md §실시간 품질 | (값) | |
| 4-6 | 원판 반지름 0.25–0.40 m | scenes_obstacles.py RADIUS = (0.25, 0.40) | 일치 | |
| 4-7 | 장애물 위치 참값을 어느 제어기에도 주지 않음 | params_x: DWA 계열 tracked_obstacles_topic·scan_filter tracks_topic → /pr_unused_tracks; 실행마다 tracks_subs 기록 | 일치 | 감싼 팔(E4)도 같다 (params_r3c.timed_block) |
| 4-8 | CC-DWA 는 설계 반복으로 만든 설정 36 개에서 골랐다 | 누적 X 12 + Y 12 + Z 12 = 36 이지만 **마지막 선택 후보는 Z00–Z11 + Y11 의 13 개** (prereg_r3 §2) | 부분 | "누적 36 설정의 설계 반복 끝에 마지막 13 후보에서" 가 정확 |
| 4-9 | 같은 규칙 (성공 수 최대, 동률이면 오차·시간·장애물 점수 합 최소) | analyze_r2 J''·S'' (analyze_r3.select) | 일치 | |
| 4-10 | C3 σ_v = σ_ω = 0.02, 12 경로 × 시드 5 | orchestrate_r3 c3 (odom_noise_v·w 0.02) | 일치 | |

## 5. 그림 캡션

| # | 캡션 문장 | 근거 | 판정 | 비고 |
| --- | --- | --- | --- | --- |
| F1-a | 그림 1: 창 기준은 직전 명령, 식 (2) 참일 때만 측정값 | windowCenterX | 일치 | F1 이 들어간 설정이면 판정식이 다르다 (§0) |
| F1-b | 정지 제약은 프로파일러·서보의 저크·가속 한계와 지연을 따른다 | maxSpeedForStop(d, decel 1.0, jerk 2.0, t_c 0.3) | 부분 | 지연은 묶은 상수 t_c 0.3 하나 (서보 d·τ 를 따로 넣지 않음) |
| F2 | 그림 2: 튜닝 경로 20 개 중 중앙값 경로, 도달 1.30/1.55/1.90 s | E6 로 held-out D 경로·값으로 교체 (results_r3c.md E6) | 교체 필요 | 새 자료 `analysis/fig2_startup_D.dat` |
| F3-a | 그림 3(a): D 안에서 목표점을 연장선 위로 | 3-16 | 일치 | |
| F3-b | 그림 3(b): 앞 3 m 막히면 최대 S 옆으로, 앞뒤 경사 합류 | 3-18·3-19·3-20 | 일치 | |
| A1 | 알고리즘 1 (main.tex 에 \input 안 됨) 2 행 "(ω 도 같다)" | 재설정 시 v·ω 함께 측정값 (dwa_x.cpp:210–212) | 일치 | |
| A1-b | 알고리즘 1 4 행 "d_g < D 이면 목표점을 연장선 위로" | 3-16 | 일치 | 목표를 지나치면 끈다는 조건 생략 |

@@SELDEP@@
