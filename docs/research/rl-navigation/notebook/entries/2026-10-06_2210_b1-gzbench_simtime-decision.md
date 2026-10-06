# A5-1 플러그인 결정 주기를 sim 시각 기준으로 — 고치기와 RTF 별 확인

- 시각: 2026-10-06 22:10 KST (이 절까지는 결과 전에 쓰고 커밋한다) · 에이전트: b1-gzbench · 단계: A5
- 코드: 고치기 전 `e812acf` (= b1-plugin `152d871` + 통합 브랜치 계획 §6), 고친 뒤 커밋은 결과 절에 적는다
- 관련 실행 id: (결과 절에 적는다)

## 질문

RL 플러그인(`src/amr_rl_controller`)의 결정 엔진은 **틱 수**로 결정 주기를 센다 (`ticks_per_decision_ = 2`,
ZOH 램프 `tick_in_period_ × control_period`). Nav2 Humble controller_server 는 **벽시계** 20 Hz 로 부르므로
(b1-plugin 실측: 벽시계 20.04–20.08 Hz, RTF 0.94 에서 sim 기준 21.2–21.4 Hz → 결정 sim 0.094 s,
`notebook/entries/2026-10-06_1859_b1-plugin_gazebo-smoke-v12s1.md` 결과 4 번), Gazebo RTF < 1 이면 sim 기준 결정 주기가
0.1·RTF s 로 줄고 명령 가속도가 1/RTF 배가 된다. 학습(Isaac·2D)은 sim 0.1 s 마다 결정하고 그 사이 가속도를 붙잡는다
(계약 §3, `rl/isaac/amr_isaac/env.py` 1–7행). **결정을 sim 시각 0.1 s 마다, 램프 적분을 실제 sim dt 로 바꾸면 RTF 와
무관하게 sim 기준 결정 주기가 0.1 s 가 되는가?**

## 가설과 예측 (결과를 보기 전에 적는다)

설계 (고칠 내용 — 코드는 이 항목 커밋 뒤에 쓴다):

1. **결정 경계 = sim 시각 격자.** 다음 경계 b(k+1) = b(k) + 0.1 (격자에 고정 — 결정이 늦거나 일러도 오차가 쌓이지
   않는다). 틱 시각 t 가 `t ≥ b − dt_est/2` 이면 결정 (경계에 가장 가까운 틱). 경계보다 반 주기 넘게 뒤처지면
   (멈춤·공백) 지금 틱으로 격자를 다시 잡는다. dt_est = 직전 틱 간격(sim, 양수일 때만 갱신, [1 ms, 0.1 s] 로 자름),
   첫 틱은 control_period(0.05).
2. **램프 = 실제 sim 경과.** 결정 시각부터의 경과 e 에서 v_cmd + a·min(0.1, e + dt_est) 를 낸다 (계약 §3 의
   "틱이 다음 틱까지 붙잡을 값" — 20 Hz·RTF 1 이면 지금과 똑같이 0.05·0.1). 주기가 0.1 보다 짧게 끝나면 다음 결정의
   v_cmd = 마지막으로 낸 명령 (끊김 없음), 길면 0.1 에서 끝 명령을 붙잡는다.
3. 이탈 상태 타이머(`devUpdate`)와 DWA 주기의 실행 가속도(prev_actions)는 실제 결정 간격(sim)으로.
4. **바꾸지 않는 것**: DWA 폴백 틱의 계약 동적 창 자름(`commitDwa`)과 폴백 끔 제동(`brakeStep`)은 공칭 control_period
   (0.05) 그대로 — 내부 DWA 의 창도 공칭 주기를 가정하므로(N:75 주석 "DWA 동적 창 Δt = 0.05 s") 같은 단위로 자른다.
   sim dt 로 자르면 RTF < 1 에서 DWA 의 정상 가속이 매 틱 잘려 폴백 DWA 가 비교군 A(DWA 단독)와 달라진다.
   (DWA 단독도 RTF < 1 에서 sim 가속도가 1/RTF 배다 — 하류 velocity_profiler 가 a 1.0 으로 묶는다. 이 비대칭은
   전이 점검 표에 적는다.)
5. stats 에 필드를 덧붙인다: 26 tick_dt(sim), 27 now(sim), 28 직전 결정 주기 길이(sim), 29 dt_est, 30 근접 인계 중.

예측:

- **P1 [계산, 단위 시험]** 엔진에 sim 틱 간격 0.025·0.035·0.05·0.07·0.08 s 와 무작위 U[0.02, 0.07] s 를 넣으면
  결정 200 회의 평균 주기가 0.100 ± 0.002 s, 각 주기가 [0.1 − dt_max, 0.1 + dt_max] 안, 한 램프 안 틱당 명령 변화 =
  a·dt (1e-9 안), 결정 경계의 명령 변화도 |a|·dt_est + |a'|·dt 를 넘지 않는다.
- **P2 [계산, 변이]** (a) 틱 수 결정으로 되돌림, (b) 램프를 공칭 0.05 로 되돌림, (c) 이탈 타이머를 0.1 고정으로 되돌림,
  (d) 격자 대신 "결정 틱 + 0.1" 로 다시 잡기 — 넷 다 새 시험 하나 이상이 잡는다. (d) 는 평균 주기가 0.1 + 평균 넘침이 되어
  RTF 0.8 근처에서 1 % 넘게 틀어지는 것으로 잡는다.
- **P3 [측정, Gazebo]** 같은 정책(v12_s1, 모드 B)·같은 목표로 (i) RTF 목표 1.0(월드 기본, 공유 부하 하 실제 RTF 기록),
  (ii) RTF 목표 0.5 (정적 월드 사본의 `<real_time_factor>` 만 0.5) 두 조건에서, 플러그인 stats 의 결정 틱 sim 시각
  차분 = 결정 주기의 **평균 ∈ [0.097, 0.103] s, 중앙값 ∈ [0.095, 0.105] s** (두 조건 모두). 실제 RTF 는 /clock 대 벽시계로.
- **P4 [측정, 대조]** 고치기 전 플러그인(`e812acf` 의 amr_rl_controller, 별도 덧씌움 작업공간)을 같은 RTF 0.5 조건에
  띄우면 결정 주기 평균이 0.1·RTF_실제 ± 10 % (≈ 0.05 s) — 결함이 측정으로 재현되고 고친 뒤 사라지는지 같은 조건에서 본다.
- **기각**: P3 에서 한 조건이라도 평균이 [0.097, 0.103] 밖이면 원인(/clock 해상도, 벽시계 틱 지터, dt_est 추정)을
  stats 로 가른 뒤 고친다 — 문턱을 결과에 맞춰 바꾸지 않는다. 같은 실패가 세 번이면 멈추고 진단만 남긴다.

## 설정

- 단위 시험: `src/amr_rl_controller/test/test_decision_engine.cpp` (가짜 정책, ROS 비의존), 플러그인 시험
  `test_rl_controller_plugin.cpp` 는 **sim 시계(/clock 발행)로 바꿔** 틱마다 0.05 s 씩 진행 (벽시계로 돌면 틱 사이가 수 ms
  라 sim 시각 결정이 거의 일어나지 않는다 — 시험이 배포와 같은 시간 원천을 쓰게 한다).
- 변이 검사: 위 (a)–(d) 를 하나씩 적용한 격리 작업공간(git archive)에서 -Werror 빌드 + gtest. `cpu_slot`.
- Gazebo: `gz_slot.sh b1-gzbench`, DOMAIN 33, IGN_PARTITION `amr_ai_b1g`, `ROS_LOCALHOST_ONLY=1`, 컨테이너
  `amr-ai-b1-gzbench-gz`, 이미지 `amr-ai/rl:0.3`. 정적 월드 사본(`make_static_world.py`) — RTF 0.5 사본은 같은 사본에서
  `<real_time_factor>1.0` → `0.5` 한 줄만 바꾼다. 정책 `/home/CAPYI_AI/runs/r7-isaac12/train/v12_s1/policy_final.onnx`
  (모드 B). 목표: b1-plugin 스모크와 같은 A (10,0,0) · B (16,−8,π/2) · C (−6,−7,π) · D (−12,0,π) · E (0,0,0) 차례로
  (`smoke_gazebo.sh` + `rl_smoke.py`, stats 원자료 `series.json`). 시작·끝 loadavg, `HEAD.txt`.
- 판정 대상: 플러그인 stats 의 new_decision 틱의 now(sim) 차분 (목표마다 첫 결정 제외 — 목표 사이 공백은 재동기로 끊긴다).
  RTF = 같은 구간의 sim 경과 / 벽시계 경과.
- 원자료: `/home/CAPYI_AI/runs/bonus1/b1-gzbench/A5_simtime/`.

## 결과

(결과 뒤에 쓴다)

## 해석

## 다음
