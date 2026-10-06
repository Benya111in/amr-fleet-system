# A2-4 Gazebo 단일 로봇 스모크 — 학습 정책 v12_s1

- 시각: 2026-10-06 18:59 KST · 에이전트: b1-plugin · 단계: A2
- 코드: 1 회차 `0a4b474`, 2 회차 `f5f6c8d` 이후 (각 실행 디렉터리 `HEAD.txt`)
- 관련 실행 id: `A2_b1plugin_gz_smoke_a` (1 회차), 2 회차는 아래

## 질문

Isaac 에서 학습한 정책(`/home/CAPYI_AI/runs/r7-isaac12/train/v12_s1/policy_final.onnx`, 계약 v1.2.0, 비교군 B,
순수 PPO, DR 켬, 경로 3–15 m 로 학습, Isaac held-out 3–15 m 성공 0.982 · 3–30 m 0.978 [측정, r7 `eval.json`])이
Gazebo 의 실제 Nav2 스택 안에서 **로드되고, 폐루프로 로봇을 몰고, 목표에 닿는가**? 폴백(DWA)이 언제·왜 끼어드는가?
이것은 스모크다 — RL 대 DWA 본 비교(D5)는 A5·C2 몫이고 여기서는 성공률을 주장하지 않는다.

## 가설과 예측 (결과를 보기 전에 적는다)

- 로드: configure 가 메타데이터(contract_version 1.2.0, arm B, obs_dim 291, dt 0.1, normalization inside)를
  받아들이고 `RLController 'DWA' 로드 … arm B, contract_version 1.2.0 … ORT 1.23.2` 를 남긴다.
- 폐루프: 정책 결정이 결정의 대부분이고(> 90 %), DWA 폴백 사유는 거의 "입력 무효"(스캔 지연 > 0.3 s) 뿐이다.
- 도달: 정적 장애물만 있는 맵에서 목표 5 개 중 **과반을 엄격 기준(Nav2 goal checker xy 0.10 m, yaw 0.05 rad)으로
  도달**한다고 예측한다. 실패가 나면 가장 그럴듯한 원인 후보 (결과 전에 적는 목록):
  (1) 목표 직전 정지 — Isaac 실패의 주 모드(정체, 목표 최소거리 중앙값 수 cm–수십 cm)와 같은 꼴,
  (2) BT 가 경로를 1 Hz 로 재계획 → 경로 관측·이탈 상태가 학습(고정 경로) 분포와 다름 (r3 위험 (1)),
  (3) safety 관측이 diagnostics 1 Hz 라 최대 1 s 늦음 (r3 위험 (3)),
  (4) 실제 Gazebo 구동 응답(1 차 지연·지연)이 DR 범위(T_m 0.048–0.112 s, 지연 0.02–0.08 s) 밖,
  (5) AMCL 위치추정 오차가 0.10 m 성공 기준의 큰 몫,
  (6) 경로 길이: 학습 3–15 m 대 일부 목표 > 15 m.
- 이 예측이 틀리면(과반 실패) 결과를 숨기지 않고 원인 후보를 로그로 가른다 — 같은 실패를 세 번 되풀이하지 않는다.

## 설정

- 월드: `warehouse.sdf` 에서 **움직이는 장애물(actor 6, 지게차·셔틀 모델 2)을 뺀 사본** (정적 구조물은 그대로, 이름
  `warehouse` 유지 — 스폰이 월드 이름으로 create 를 부른다). 만드는 스크립트는 커밋한다
  (`src/amr_rl_controller/scripts/make_static_world.py`), 사본은 커밋하지 않는다 (원자료 디렉터리).
  지도(AMCL)는 원래 `maps/warehouse` — 움직이는 장애물은 지도에 없으므로 같다.
- 스택: `amr_bringup system.launch.py with_navigation:=false …` + `amr_rl_controller rl_navigation.launch.py
  replace_dwa:=true mode:=B` (BT 기본 제어기 'DWA' 자리를 RL 이) + lifecycle watchdog. 스폰 (0, 0, 0).
- 목표 (지도 좌표, 차례로, 앞 목표에서 출발): A (10, 0, 0) · B (16, −8, π/2) · C (−6, −7, π) · D (−12, 0, π) ·
  E (0, 0, 0). 각 navigate_to_pose 시간 상한 180 s. 마지막에 follow 5 m 한 번 (BT 없이 FollowPath — 재계획 없음).
- 기록 (목표마다): status, 경과(벽시계·sim), 끝 자세와 목표 거리(AMCL·지면 진실), 결정 출처(정책/DWA/정렬),
  DWA 사유, 틱당 |Δv|·|Δω|, infer_ms·cycle_ms, 최소 정적 여유(가능하면). RTF 는 /clock 대 벽시계로 잰다.
- 자원: `/home/CAPYI_AI/env/gz_slot.sh b1-plugin`, DOMAIN 31, IGN_PARTITION `amr_ai_b1p`, `ROS_LOCALHOST_ONLY=1`,
  컨테이너 `amr-ai-b1-plugin-gz`, 시작·끝 loadavg. 원자료 `/home/CAPYI_AI/runs/bonus1/b1-plugin/A2_gz_smoke/`.
- 판정: 스모크 합격 = 로드 확인 + 폐루프 명령 + 폴백 사유 카운터 기록. 도달 수는 **보고만** (n = 5 라 성공률 주장 없음).

## 1 회차 중간 결과 (2026-10-06 19:53–20:05 KST) — 2 회차 전에 적는다

[측정, `/home/CAPYI_AI/runs/bonus1/b1-plugin/A2_gz_smoke/smoke/gz_v12s1_static_a/`, loadavg 11.7 → 29.2]
- **로드**: `RLController 'DWA' 로드: 모델 /runs/models/v12_s1.onnx (arm B, contract_version 1.2.0, …, seed 1,
  train_steps 39321600, …, ORT 1.23.2) | 모드 B, 결정 0.10 s = 틱 0.050 s x 2, 추론 상한 25.0 ms …` — lifecycle 4/4 5 s 안.
- **하네스 결함 (내 실수)**: 목표 C (−6, −7)·D (−12, 0) 은 `--goal -6.0,…` 를 argparse 가 옵션으로 읽어 **시도되지 않았다**.
  `--goal=…` 로 고쳤다 (`f5f6c8d`).
- 목표 A (10, 0, 0) · B (16, −8, π/2) · E (0, 0, 0) (B 다음 바로 E): **셋 다 성공 (status 4)**, 최종 위치 오차
  0.079 / 0.043 / 0.081 m (map TF) · 0.084 / 0.044 / 0.062 m (지면 진실), sim 22.5 / 14.7 / 23.0 s, RTF 0.75 / 0.71 / 0.66.
  결정은 전부 정책·정렬 (정책 278·174·307, 정렬 24·31·42), **DWA 폴백 0, 사유 카운터 전부 0**, 틱당 최대 |Δv| 0.049 ·
  |Δω| 0.100, 플러그인 안 추론 p50 0.15–0.17 ms · p99 0.28–0.50 ms, 한 틱 전체(cycle) p99 0.36–0.62 ms. follow 5 m 도 성공.
- **이상**: 성공 셋의 끝 yaw 가 요청 yaw 와 0.244 · 0.247 · 0.162 rad 다르다 (map TF 와 지면 진실이 같은 값). 배포 goal
  checker 는 `yaw_goal_tolerance: 0.05` (nav2_params.yaml:99) 인데 0.25 rad 가까이에서 "Reached the goal!" 이 났다.
  0.25 는 Nav2 SimpleGoalChecker 의 **기본값**이다.

### 2 회차 사전 등록 (결과 전) — yaw 이상의 원인 가르기

- 가설 Y1: controller_server 에 `general_goal_checker.yaw_goal_tolerance` 가 0.05 로 실리지 않고 기본값 0.25 로 돈다.
  → `ros2 param get` 이 0.25 를 내면 Y1.
- 가설 Y2: 파라미터는 0.05 인데 전역 경로 끝 자세의 방향이 요청 yaw 와 다르다 (플러그인·goal checker 모두 경로 끝 방향을
  쓴다). → 경로 끝 yaw 와 요청 yaw 의 차가 끝 yaw 오차와 비슷하면 Y2.
- 둘 다 아니면 원인을 로그(정렬 결정의 ω·yaw 시계열, `series.json`)로 더 판다. 같은 이상이 DWA 에도 생기는지는 이 단계에서
  재지 않는다 (판정 경로가 제어기와 무관하면 DWA 도 같다 — 근거를 적는다).
- 2 회차 판정: 목표 5 개 모두 시도, 도달 수·오차·폴백 사유를 보고 (스모크 합격 기준은 그대로).

## 2 회차 결과 (2026-10-06 20:56–21:13 KST) — 3·4 회차 전에 적는다

[측정, `…/smoke/gz_v12s1_static_b/`, loadavg 15.9 → 20.2 (다른 에이전트의 Isaac 학습·변이 검사와 겹침)]
- yaw 이상: **Y1 기각** (`general_goal_checker.yaw_goal_tolerance` = 0.05 로 실렸다, `params.txt`), **Y2 기각** (경로 끝 yaw
  = 요청 yaw: 목표 1 끝 (10.0, 0.0, 0.0)). 이번 목표 1 의 끝 yaw 오차는 0.118 rad. 원인 미확인 — 아래 3 회차에서 자세 시계열로 본다.
- **목표 1 은 map 기준 "성공" (0.034 m) 이었으나 지면 진실은 x 28.1 (map 10.0 과 18 m 차)**: 위치추정이 B–C 통로를 따라
  18 m (랙 주기 6 m 의 3 배) 미끄러졌다. 같은 순간 odom EKF(`odometry/filtered`)는 지면 진실과 진행 방향이 약 90° 어긋났다
  (11:00:36–52 의 세 표본: 지면 진실 +y 로, odom EKF +x 로 움직임). 목표 2–5 는 실제 위치 (28.5, 16.7) 근처에서 막혀 모두 시간
  초과 (180 s, `Failed to make progress` · spin · backup 복구 반복), follow 5 m 는 status 6.
- 플러그인 쪽: 다섯 목표 모두 **DWA 폴백 0, 사유 카운터 0**, 정책 결정 1529–1794 회/목표 — 제어기는 계속 정책으로 돌았다.
- 하네스: `elapsed_sim_s` 는 rl_smoke 노드가 /clock 을 받기 전 시각(0)에서 재기 시작해 목표 1 에서 109.7 s 로 부풀려졌다 —
  무효 (고침: /clock 을 받을 때까지 기다린다).

### 3·4 회차 사전 등록 (결과 전)

- 질문: 2 회차의 위치추정 붕괴가 **RL 정책 고유**인가 (예: 급가속·급회전 → 바퀴 미끄럼·오도메트리 오차), 아니면 **스택·부하
  문제**인가?
- 3 회차 = 같은 RL 설정 재실행, 4 회차 = **대조: 같은 스택에서 제어기만 원래 DWA** (`BASELINE_DWA=1` — replace_dwa 없이,
  BT 기본 'DWA'). 둘 다 자세 시계열(5 Hz: 지면 진실 · odom EKF · TF map · TF odom)을 남긴다.
- 판정 규칙: 각 목표 끝의 map 대 지면 진실 차 (> 0.5 m 면 "위치추정 붕괴"), 목표마다 odom EKF 진행 방향 대 지면 진실 진행
  방향의 차. RL 만 붕괴하면 RL 고유 쪽으로, 둘 다 붕괴하면 스택·부하 쪽으로 기운다. 둘 다 안 붕괴하면 2 회차는 부하 하
  간헐 현상으로 적고 반복률은 미측정으로 남긴다 (n 이 작다 — 결론이 아니라 방향).

## 결과

(실행 뒤 채운다)

## 해석

(실행 뒤 채운다)

## 다음

(실행 뒤 채운다)
