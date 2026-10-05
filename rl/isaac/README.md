# rl/isaac — Isaac Lab 2.3.2 학습 환경 (보너스 1, 비교군 B)

명세 5장 보너스 1 (`31_*.md` 323행): "Isaac Sim의 강화학습 환경을 활용하여 PPO/SAC 기반 내비게이션 정책을
학습시킨다." 이 디렉터리는 그 Isaac 트랙이다. 관측·행동·보상·비용·종료는 `rl/contract/` 계약 v1.1.0 을 따른다.
결과 수치와 실패 기록은 `/home/CAPYI_AI/logs/agents/r1-isaac/REPORT.md` 에 있다 (이 문서는 구조와 사용법).

## 구성

| 파일 | 내용 | Isaac 필요 |
|---|---|---|
| `amr_isaac/contract_torch.py` | 계약 기준 구현(numpy)의 배치 torch 판 — 관측 291, 보상 6 항, 비용 5, 이탈 상태기계, `integrate_b`, 프로파일러, safety_lite | 아니오 |
| `amr_isaac/warehouse_geom.py` | `warehouse.sdf`(읽기 전용) + include 모델 → 충돌 상자 229 개 → 삼각형 메시 / 2D 사각형 129 개 / 0.05 m 점유 격자 | 아니오 |
| `amr_isaac/paths.py`, `assets.py` | 전역 코스트맵 팽창 + A* 간선 비용으로 비용 최단 경로 풀 (캐시 `/home/CAPYI_AI/runs/r1-isaac/assets`, 커밋 안 함) | 아니오 |
| `amr_isaac/env_cfg.py`, `env.py`, `tasks.py` | `DirectRLEnv` — `AMR-Isaac-Nav-B-v0` (학습), `AMR-Isaac-Nav-B-Eval-v0` (held-out 경로·명목 동역학) | 예 |
| `amr_isaac/agents/rsl_rl_ppo_cfg.py` | rsl_rl PPO 설정 (γ 0.995, λ 0.97, rollout 64) | 예 |
| `amr_isaac/policy_export.py` | rsl_rl state dict → 계약형 `PolicyB` → ONNX (`onnx_io.md`) | 아니오 |
| `scripts/` | `build_assets` · `smoke_env` · `check_golden_in_sim` · `train_ppo` · `eval_policy` · `export_onnx` · `plot_curves` · `gpu_monitor.sh` | 일부 |
| `tests/` | golden 31 대조, 기준 구현과 무작위 차분, 창고 변환기·경로 풀 | 아니오 |
| `run_isaac.sh` | Isaac 컨테이너 실행기 (`env/verify/verify_isaac.sh` 의 마운트·환경변수, 이름 `amr-ai-r1-isaac-<태그>`) | — |

## 실행

```bash
WT=/home/CAPYI_AI/wt/r1-isaac; R=/home/CAPYI_AI/runs/r1-isaac
# 1) 순수 시험 (CPU, Isaac 불필요)
docker run --rm -v $WT:/ws -w /ws/rl/isaac amr-ai/rl:0.3 python3 -m pytest -q tests
# 2) 창고 기하·경로 풀 캐시 (없으면 env 가 스스로 만든다 — 한 프로세스로 수 분)
docker run --rm -v $WT:/ws -v $R:$R -w /ws amr-ai/rl:0.3 python3 rl/isaac/scripts/build_assets.py --goals 512
# 3) env 점검·Isaac 안 golden 주입 시험
rl/isaac/run_isaac.sh smoke  rl/isaac/scripts/smoke_env.py --num_envs 256
rl/isaac/run_isaac.sh golden rl/isaac/scripts/check_golden_in_sim.py
# 4) 학습 → 평가 → ONNX
rl/isaac/run_isaac.sh train rl/isaac/scripts/train_ppo.py --num_envs 1024 --iterations 400 --run <run> \
    --curriculum 4:30:250 --vcap_curriculum 0.15:0.4:40:250:0.5
rl/isaac/run_isaac.sh eval rl/isaac/scripts/eval_policy.py --policy $R/train/<run>/policy_final.pt
docker run --rm -v $WT:/ws -v $R:$R -w /ws amr-ai/rl:0.3 python3 rl/isaac/scripts/export_onnx.py --run $R/train/<run>
```

앱 종료(`app.close()`)가 CPU 100 % 로 멈춘 적이 있어 모든 Isaac 스크립트는 끝에서 `os._exit` 한다.

## 전이 한 번 (Δt = 0.1 s = 물리 0.02 s × 5)

raw 행동 → `u = tanh(raw)` → `integrate_b` (ray 클램프) → 결정 주기 안 선형 명령 램프 → [50 Hz × 5]
프로파일러 기준 경로 → safety_lite φ → 플랜트(순수 지연 + 1 차 지연 + 가속 한계) → 정확 원호 적분 →
풋프린트 대 정적 사각형 분리축 충돌 판정 (접촉 = 그 자리 정지·종단) → Warp `RayCaster` 720 빔 → 계약 관측.
버퍼에는 rsl_rl 이 낸 원출력(raw)과 그 log-prob 가 들어간다 (contract §5). 결정론적 배포 행동 = `tanh(mu)` = ONNX `action`.

## 계약과 다르게, 또는 계약이 정하지 않아 여기서 정한 것

1. **safety_lite 의 접근 거리 D** — 계약은 D 를 입력으로 받는다(§7.4). 여기서는 정본 `SweptFootprint` 대신
   근사: 직진(|v| ≥ 1e-3)은 진행 방향 앞·차폭 띠(|y| ≤ 0.20) 안 점까지의 세로 거리, 제자리 회전은 외접원 밖
   반지름 거리, 정지는 +inf, 풋프린트 안 점은 0. 원호·꼬리 직선·가설 여러 개·강건 통계·지연 보정 일부가 빠졌다.
   STOP 래치는 lite 규칙대로 없다 (확정 전 STOP = CRITICAL). r2(2D) 와 같은 φ 가 아니다 — SRCC 보고에 적을 것.
2. **하류 사슬 순서** — contract §5.4 는 "프로파일러 → 플랜트 → φ" 로 적었다. 여기서는 실제 배선(프로파일러가
   낸 cmd_vel 을 safety_node 가 거르고 DiffDrive 가 응답)대로 **프로파일러 → φ → 플랜트** 다. 개입 비용의
   (u, φ(u)) 는 계약 §7.1 대로 (프로파일러 출력, φ 출력) 이다.
3. **실행 가속도 관측** — `a_v_meas` = 프로파일러 선속도 필터 상태 `a`, `a_w_meas` = 프로파일러 `w_ref` 의
   서브스텝 차분 (계약 State 주석 "프로파일러 상태").
4. **동적 장애물 없음** — 트랙 블록은 늘 0, `min_ttc` 없음. 학습 분포에서 트랙 특징이 상수라 배포에서 트랙이
   생기면 정규화 뒤 값이 커진다 (분산 하한 1e-2 로 ±10 이하이지만 학습되지 않은 입력이다). 다음 물결 과제.
5. **맵 다양성(DR 축 1) 없음** — 창고 한 장. held-out 은 같은 맵의 다른 시작·목표(경로 풀 seed 1)다.
6. **DR 구현분** — 플랜트 T_m·T_d·가속 한계 배율, LiDAR σ·빔 드롭아웃. 관측·행동 지연(축 4)과 트랙 누락(축 5)
   은 없다. 순수 지연은 0.02 s 단위로 양자화 (1–4 서브스텝).
7. **A\* 경로** — `amr_navigation` C++ 플래너를 부르지 않고 같은 비용 규칙의 Dijkstra(같은 최적 비용, 동률 경로
   선택은 다를 수 있다). C++ 평활기(숏컷 + Gauss–Seidel) 대신 숏컷만 근사한 뒤 0.05 m 재표본.
8. **관측 정규화** — rsl_rl `EmpiricalNormalization` 의 통계 갱신을 쓰고 적용 식만 계약식으로 바꿨다
   (`train_ppo.ContractNormalization`). 분산 하한 1e-2 를 두었고 ONNX 에도 같은 σ² 를 넣는다.
9. **학습 전용 커리큘럼** — 경로 길이 상한(구조 파라미터, 20 % 는 전체 풀 균일, contract §10)과 **v_cap**
   (관측된다; 평가·비교는 v_cap = 1.0). v_cap 커리큘럼은 계약에 없는 [설계 선택]이다 — 이유는 REPORT §5.

## Isaac 함정 (이 호스트, Isaac Sim 5.1 / Isaac Lab 2.3.2 / RTX 5090 / Rocky 8.10)

- `kinematic_enabled=True` 강체에 GPU PhysX `set_transforms` → `CUDA error 700 (illegal memory access)`
  `GpuRigidBodyView.cpp:512`. 충돌체가 하나도 없는 장면도 첫 step 에서 CUDA 700 (narrowphase). → 동적 강체
  (중력·충돌 끔) + 바닥 평면. 재현 스크립트 `/home/CAPYI_AI/runs/r1-isaac/probe/probe_kin.py` (변형 A–J).
- `RayCasterCfg.ray_alignment='yaw'` 는 시작점만 돌리고 방향은 세계 고정(높이 스캐너용) → 2D LiDAR 는 `'base'`.
- `RayCaster.data.pos_w` 는 부착 강체 원점이다. `cfg.offset` 은 광선 시작점에만 더해진다.
- 쓴 자세는 같은 step 안에서 RayCaster 가 바로 읽는다 (리셋 직후 원점 오차 7.7e-7 m, smoke 점검).
