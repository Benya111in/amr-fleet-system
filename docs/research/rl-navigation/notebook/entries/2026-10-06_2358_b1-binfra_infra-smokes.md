# B 단계 기반 스모크 — DR 끔 학습, PID 라그랑주 λ 반응, SAC 학습 신호, ONNX

- 시각: 2026-10-06 23:58 KST · 에이전트: b1-binfra · 단계: B0 (B2·B3·B4 기반)
- 코드: `1cc4887` (브랜치 `feature/b1-binfra`) — 이 항목은 아래 스모크를 돌리기 **전에** 커밋한다. 스모크는 이 커밋의
  `git archive` 사본 `/home/CAPYI_AI/runs/bonus1/b1-binfra/src/1cc4887/` 에서 돈다 (worktree 수정과 섞이지 않게)
- 관련 실행 id: B0_smoke_dr_off, B0_smoke_pid_d0p2, B0_smoke_pid_d200, B0_smoke_sac_s0, B0_smoke_sac_s0_eval, B0_onnx_sac, B0_onnx_pid,
  B0_pytest_cpu, B0_pytest_isaac_ppo_lag
- 원자료: `/home/CAPYI_AI/runs/bonus1/b1-binfra/smoke/<run_id>/` (HEAD.txt·시작/끝 시각·loadavg·rc·run.log + 학습/평가 산출)

## 질문

B 실행자(b1-sweep·b1-sac·b1-dr·b1-ppo10-eval)가 코드 수정 없이 돌릴 기능 셋이 Isaac 안에서 실제로 동작하는가?
(1) `--dr off` 가 정말 명목값으로 학습하는가, (2) 제약 PPO 의 λ 가 예산에 반응하는가, (3) SAC 가 이 env 에서 학습 신호를 내는가,
(4) SAC·제약 PPO 정책이 onnx_io.md 와 같은 입출력으로 내보내지는가. **스모크는 성능 주장이 아니다** (1 시드, 짧은 예산).

## 가설·예측과 판정 (결과 전에 고정)

1. **DR 끔** (B0_smoke_dr_off): `train_ppo.py --dr off --phi canonical --num_envs 512 --iterations 2 --path_len 3:15 --seed 0`.
   통과 = `dr_snapshot.json` 의 모든 축 min = max = 명목값 (T_m 0.08, T_d 0.04, 가속 1, σ 0.03, 드롭 0, 이득 1, 관측 지연 0) 이고
   summary `dr` = false. 대조: PID 스모크(DR 켬)의 dr_snapshot 은 학습 범위 안에서 퍼져야 한다 (min < max).
2. **PID 라그랑주 λ 반응** (B0_smoke_pid_d0p2·_d200): 제약 = 개입(계약 §7.1), 정본 φ, DR 켬, 3–15 m, 2048 env × 40 반복 (5.24M),
   시드 0, 기본 이득 Kp 0.5·Ki 0.05·Kd 0.1, λ 상한 10, 미분 지연 5, EMA (0, 0.5). 예산 d = **0.2** (B2 훑기의 작은 끝) 와 **200** (묶이지 않음).
   - 예측: 학습 초반 롤아웃 개입은 크다 (A3c 무제약 학습 반복 10 에서 42/에피소드 [측정, A3c curve.jsonl]) → d 0.2 는 e = (J − d)/d 가 수백이라
     λ 가 첫 반복부터 상한 10 에 붙는다. d 200 은 J < 200 인 동안 λ = 0 그대로.
   - 통과 = d 0.2 의 λ 가 처음 5 반복 안에 > 0 이 되고 40 반복 중 > 0 인 반복이 대부분 (≥ 30) · d 200 의 λ 가 40 반복 모두 0.
     λ 가 예산에 반응하지 않으면 (두 run 의 λ 궤적이 같으면) 결함.
   - 함께 적는 것 (판정 아님): 반복마다 J·λ·P·I·D (lagrange_history.jsonl), 롤아웃 성공·개입, 비용 critic 손실. d 0.2 에서 λ 가 상한에 붙으면
     정책은 비용 이익이 지배해 정지 쪽으로 갈 수 있다 — 정지는 개입 0 이라 Slater 증인이다 (예산을 만족하는 정책이 있다). 그 대가(성공 하락)는
     B2 의 Pareto 곡선이 잰다.
3. **SAC 학습 신호** (B0_smoke_sac_s0): `train_sac.py` 기본값 (1024 env, 배치 8192, env 스텝당 갱신 4, lr 3e-4, α₀ 0.05, 목표 엔트로피 −2,
   γ 0.995, τ 0.005, 버퍼 10⁶, learning_starts 50), 정본 φ, DR 켬, 3–15 m, 시드 0, **10M env-step** (PPO 판정 예산 39.3M 의 약 1/4).
   - 신호 판정 (사전): 학습 롤아웃 성공률(확률 행동, DR 켬)의 마지막 5 chunk (chunk = 64 결정 스텝 × 1024 env) 평균 ≥ 0.05 이면 "신호 있음".
     보조: 마지막 정책의 결정론 평가 (B0_smoke_sac_s0_eval: 네 집합, 정본 φ, seed 4242) 의 main_3_15 엄격 성공.
   - 예측 (자신 낮음): PPO 는 2.6M 에서 롤아웃 성공 10 % 를 넘었다. off-policy SAC 는 이 env 에서 잰 적이 없다 — 10M 에서 0.05–0.5 로 본다.
   - **신호가 없으면** (< 0.05) 진단(α·Q·log σ 궤적)을 적고, 하이퍼파라미터 변형은 사전에 정한 둘까지만: (a) α₀ 0.01, (b) 배치 4096·갱신 8.
     세 번(기본 + 둘) 모두 신호가 없으면 더 돌리지 않고 진단을 b1-sac 에 넘긴다 (COORDINATION §7 의 "같은 실패 세 번이면 진단").
4. **ONNX** (B0_onnx_sac·_pid): `export_onnx.py --run <smoke run>` (amr-ai/rl:0.3). 통과 = fails 0 (입출력 이름·모양·dtype·opset 17·필수 메타데이터·
   algo 주장 일치·ORT 대 torch ≤ 1e-5). SAC 는 `algo` 가 SAC 이고 `constraint` = none, PID 는 `constraint` = "intervention <= d ...".
5. **시험** (B0_pytest_cpu·B0_pytest_isaac_ppo_lag): CPU (amr-ai/rl:0.3) 전체 + Isaac 이미지 파이썬의 test_ppo_lag 3 개.

## 결과
(스모크 뒤 채운다)

## 해석
(스모크 뒤)

## 다음
- b1-sweep (B2) 는 PID 이득을 이 스모크 결과를 보고 고르지 않는다 — 기본 이득 그대로 예산 {0.2, 0.5, 1.0} × 5 시드 (바꾸려면 그쪽 사전 등록에서).
