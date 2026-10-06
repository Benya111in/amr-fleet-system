# 1차 물결 r0 — RL 계약 v1.0.0 → v1.1.0 (관측·행동·보상·비용·golden)

> **소급 기록** (작성 2026-10-06 19:08 KST 첫 커밋, b1-backfill, 단계 A4). 원 작업: r0-contract, 2026-10-06 06:19–07:32 KST
> (첫 커밋 766424e 06:19, 판 올림 b1eb0c7 07:22, 리뷰 07:32). 학습·평가 run 은 없고 시험만 있다. 수치는
> `/home/CAPYI_AI/logs/agents/r0-contract/` 의 로그에서 다시 읽었고, 로그가 없는 계산 값은 [문서주장] 으로 적었다.

- 시각: 2026-10-06 06:19 KST (원 작업) · 에이전트: r0-contract (기록 b1-backfill) · 단계: W1 (1차 물결)
- 코드: `feature/rl-contract` 766424e … ffc93af (v1.0.0), 5b04c1c·1274aa8·17d30fc·b1eb0c7 (v1.1.0)
- 관련 실행 id: `W1_r0_crosscheck_cpp_698585d`, `W1_r0_crosscheck_cpp_b1eb0c7`, `W1_r0_pytest_ffc93af`, `W1_r0_pytest_b1eb0c7`

## 질문
Isaac(r1)·2D(r2)·C++ 플러그인(r3)·평가(r4)가 **같은 관측·행동·보상·비용·에피소드**를 쓰도록, 숫자마다 출처가 있는
규범 계약과 기준 구현·golden 벡터를 만들 수 있는가. 그리고 그 기준 구현이 팀 C++ 원본(DWA·VO·프로파일러·SafetyGate)과
같은 값을 내는가.

## 당시 가설과 예측
- 보상은 PBRS(Φ = −κ·d_arc, 종단 Φ = 0)로 두면 최적 정책을 바꾸지 않고 탐색만 돕는다 (「이론」). Δd 꼴은 최적 정책을
  바꾼다 — 사슬 MDP 로 확인할 수 있다.
- 충돌 벌점 C = 30, 시간 비용 c = 0.02, γ = 0.995 면 "충돌 ≺ 대기 ≺ 성공" 순서가 선다.
- 비용 5 채널(충돌·개입 보정 크기·1 m 이탈·5 s 복귀·트랙 CTE)을 제약으로 쓰고 예산 d > 0 으로 Slater 조건을 지킨다.
  개입 예산 d₁ 은 DWA 실측 × 1.2 로 정하되, 측정 전에는 {0.02, 0.05, 0.1, 0.2} 를 훑는다 (contract §7.2).
- (기록 없음) 성공 조건(0.10 m, |v| ≤ 0.05 m/s)이 탐색을 막을 위험은 이때 가설로 다루지 않았다 — 뒤에 r1·r2 가 찾았다.

## 설정
- 문서: `rl/contract/contract.md` (관측 291 = LiDAR 180 섹터·자기 상태·A\* 부분 목표 5·목표·safety·이탈 상태기계·트랙 8),
  결정 0.1 s, 행동은 가속도. 비교군 A(DWA)·B(연속 PPO/SAC)·C(99 셀 + 폴백 마스크)·D(SMDP, 선택).
- 기준 구현 `rl/contract/amr_rl_contract/` (numpy), golden `rl/contract/golden/*.json` (v1.0.0 28 개 → v1.1.0 31 개),
  `onnx_io.md`, C++ 대조 `tools/crosscheck_cpp.sh`.
- 시험 환경: `amr-ai/rl:0.3` 컨테이너, `git archive` 사본.

## 결과
| 실행 | 커밋 | 결과 | 표기 |
|---|---|---|---|
| C++ 원본 대조 (v ∈ [0, 1.0]) | 698585d **(추정)** + 미커밋 tools/ — 로그의 HEAD 줄이 비어 있다 (아래 「차이」) | vo 2000·dwa 300·prof 600·saf 1000 건 불일치 0 | [측정] `crosscheck_cpp.log` (커밋은 추정) |
| C++ 원본 대조 (v ∈ [0, 2.0], 리뷰 반영) | b1eb0c7 | 같은 3900 건 불일치 0 (θ 를 감은 차이로 비교한 뒤) [측정]. 범위를 넓힌 첫 실행의 "dwa 6 건 불일치 (C++ `geometry.cpp:191` θ 감기 표현 차이)" 는 WAVE 진술뿐이고 로그가 없다 [문서주장] | [측정] `crosscheck_cpp_review.log` (6 건은 [문서주장]) |
| pytest v1.0.0 | ffc93af | 79 passed, golden 28 | [측정] `final_ffc93af.log` |
| pytest v1.1.0 | b1eb0c7 | 88 passed, golden 31, colcon list 의 rl/ 항목 0 | [측정] `final_review.log` |
| 변이 검사 (리뷰 반영 7 종) | b1eb0c7 | 대조(변이 없음) 88 passed; HEAD 변이 7 종 모두 잡힘 (3–4 failed); 그중 무회전·역회전·VO 지도속도·포화표시 4 종을 리뷰 전 ffc93af 에 넣으면 79 passed 로 놓침 | [측정] `mutation_review.log` |
| 사슬 MDP: Δd 는 16.8 m 넘는 133 상태에서 위험한 돌진을 고르고 PBRS 는 기저와 같다; p\* (γ 0.995) 11.7 % | — | — | [계산, 문서주장 — 계산 스크립트 출력 로그 없음, 리뷰가 재계산 일치 보고] |

리뷰(07:32)가 찾은 차단 결함 3 개와 수정:
1. golden 이 트랙 속도 좌표 변환·비포화 VO 를 검증하지 못함 → d16·d17 추가 (1274aa8).
2. 마스크 사전필터가 |v′| > 1.0 에서 마지막 롤아웃 자세의 점을 버림 (기준 구현 결함 → 곧 규범) → 사전필터 제거 (5b04c1c), golden d15.
3. BC 레이블 규칙(격자 최근접 → 마스크 밖이면 폴백)이 DWA 의 정상 가속·선회 시작을 "제동"으로 레이블 → 마스크 안 최근접(폴백 포함)으로 변경 (17d30fc). 리뷰 재확인: 무작위 400 장면 폴백 레이블 371 → 65 [측정, `recheck/probe.log` 를 인용, 이 기록에서 다시 돌리지 않음].

## 해석
- 계약·기준 구현·golden 의 **일치성**은 강하게 확인됐다 (C++ 원본 대조 3900 건 0, golden 이 회전·VO·마지막 자세를 고정).
- 계약이 정한 **보상의 학습 가능성**은 이 단계에서 전혀 재지 않았다. 이후 r1·r2 의 순수 PPO 9 run 이 모두 실패하고(대기·표류
  끌개), 2a 에서 v1.1.0 의 비성공 궤적 가치가 진행과 무관하다는 것이 [측정] 으로 드러났다 → 계약 v1.2.0. 계약의 "PBRS 는
  최적 정책을 바꾸지 않는다" 는 맞았지만, **최적 정책이 정지 문턱을 맞추는 성공 사건 하나에만 걸려 탐색 신호가 없다**는 점을 보지 못했다.
- 개입 예산 d₁ 을 "훑는다" 고 정해 두었는데 r1 은 그 범위 밖(5.0)을 썼다 (`2026-10-06_0832_…bc-ppo-lagrangian.md`).

## 무엇을 바꾸게 했나
- v1.1.0 (같은 날 07:22): 마스크·BC 레이블·golden 31.
- 이후 계약을 쓰는 모든 구현(r1·r2·r3)이 golden 재생 시험을 자기 경로에 둔다 — 2D·Isaac·C++ 의 관측·보상이 1e-5·1e-4 안에서 같다는 근거.

## 다음 (당시 남은 것)
- d₁ 미측정(DWA 2D 개입 분포 필요), d₂–d₄ 근거 없음, safety_lite 는 SafetyGate 일부만, U 턴 경로의 호길이 점프(Φ 불연속), Gazebo 가속 응답 미측정.

## 원자료
- `/home/CAPYI_AI/logs/agents/r0-contract/` — `crosscheck_cpp*.log`, `final_*.log`, `mutation_*.{log,json}`, `recheck/`
- 보고: `/home/CAPYI_AI/logs/agents/r0-contract/WAVE1_RESULT.md`

## WAVE 보고와 원자료의 차이
- 시험 수와 대조 건수는 로그와 같다.
- **(리뷰 반영 정정)** 처음에 "`HEAD.txt` 와 `baseline_698585d.log` 가 git 오류 문자열을 담는다" 고 적었는데 **사실과 반대였다**.
  - `HEAD.txt` (mtime 06:38:12) 의 내용은 `698585d5517145e8a073c71b3cebe77eb175551b` 이고,
    `baseline_698585d.log` (06:38:28) 는 `79 passed, 4 warnings in 5.23s / flake8-ok / No problems found` 다.
  - git 오류 두 줄 (`fatal: not a git repository: /home/CAPYI_AI/repo/.git/worktrees/r0-contract`) 과 빈 `HEAD  + 미커밋 tools/` 줄은
    **`crosscheck_cpp.log` 머리**에 있다.
  - 그래서 첫 C++ 대조(`W1_r0_crosscheck_cpp_698585d`)의 커밋은 로그로 확인할 수 없다. 커밋은 다음 셋으로 **추정**했다:
    2 분 앞(06:38:12)의 `HEAD.txt` 698585d, 로그 끝 줄 "커밋 b6ca451 에 들어간 tools/ 로 실행한 결과 (실행 시점엔 미커밋, 내용 동일)",
    b6ca451 커밋 시각 06:40:23 (로그 mtime 06:40:23 과 같은 초 — 끝 줄은 커밋 직후에 덧붙인 것으로 보인다).
- WAVE 의 "범위를 넓힌 첫 실행은 dwa 6 건 불일치" 는 로그가 없다. `crosscheck_cpp_review.log` 에는 비교기를 고친 뒤의 0 건만 있다 → [문서주장].

## 실행 목록에 등록한 run (`runs/b1-backfill.jsonl`, entry = 이 파일)

| run_id | 종류 | 시뮬 | 커밋 | 시드 | 표기 |
|---|---|---|---|---|---|
| `W1_r0_crosscheck_cpp_698585d` | test | offline | 698585d | n/a (결정적 입력) | [측정] |
| `W1_r0_crosscheck_cpp_b1eb0c7` | test | offline | b1eb0c7 | n/a (결정적 입력) | [측정] |
| `W1_r0_pytest_ffc93af` | test | offline | ffc93af | n/a (결정적 입력) | [측정] |
| `W1_r0_pytest_b1eb0c7` | test | offline | b1eb0c7 | n/a (결정적 입력) | [측정] |
