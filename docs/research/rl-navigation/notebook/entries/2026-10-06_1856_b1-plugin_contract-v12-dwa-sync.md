# A2-1 플러그인 계약 v1.2.0 반영과 DWA 창 동기화 API (syncLastCommand)

- 시각: 2026-10-06 18:56 KST · 에이전트: b1-plugin · 단계: A2
- 코드: `a7da8a2` (브랜치 `feature/b1-plugin`, 분기점 `feature/bonus1-rl`)
- 관련 실행 id: `A2_b1plugin_baseline_try0`(실패), `A2_b1plugin_baseline`, `A2_b1plugin_dwa_ref_pre_api`,
  `A2_b1plugin_test_sync_t1`(실패: 서식), `A2_b1plugin_test_sync_t2`, `A2_b1plugin_test_sync_t3`, `A2_b1plugin_planwin_probe`,
  `A2_b1plugin_mutation_r1`, `A2_b1plugin_mutation_r2`, `A2_b1plugin_kin_*` 12 건 (그중 1 건 실패: 목표 응답 유실)

## 질문

1. C++ 플러그인(`src/amr_rl_controller`, 1차 물결 r3, 계약 v1.1.0 기준)이 계약 v1.2.0 의 golden 32 개 중
   플러그인이 구현하는 부분과 그대로 일치하는가?
2. `amr_navigation::DWAController` 에 공개 `syncLastCommand(v, w)` 를 더해도 **그 API 를 부르지 않는 기존 경로의
   출력이 비트 단위로 같은가**, 그리고 그 API 로 r3 의 `activate()` 우회(게이트 상태까지 지우는 부작용)를 걷어낼 수
   있는가?

## 가설과 예측 (결과를 보기 전에 적는다)

- **H1 (계약)**: v1.2.0 은 보상만 바꿨으므로 decision 17·integrate 1·profiler 1·safety 1 golden 은 `contract_version`
  필드 말고 같다. [계산, 2026-10-06 18:50, `git show b1eb0c7:` 대 `c3c97de:` 의 JSON 비교 — 20 개 파일이 버전 필드 말고
  같고, s01–s11 은 `expected` 의 `reward`·`terms`·`phi_pre`·`phi_post` 만 다르며 플러그인이 쓰는 `dev`·`e` 는 같다,
  s12 는 새 사례]. 예측: C++ 은 버전 상수·사례 수(step 11 → 12)만 고치면 전부 통과한다. **기각 조건**: 버전 말고
  다른 이유로 실패하는 golden 사례가 하나라도 있으면 원인을 찾을 때까지 다음 단계로 가지 않는다.
- **H2 (비트 동일)**: `syncLastCommand` 는 새 메서드 하나이고 기존 함수 본문을 바꾸지 않으므로, API 를 부르지 않는
  DWA 의 `computeVelocityCommands` 출력은 a7da8a2 와 비트 단위로 같다. 예측: (a) `amr_navigation` 기존 시험 전부 통과,
  (b) a7da8a2 코드로 만든 결정적 시나리오의 출력 기록(double 의 16진 비트)을 새 코드가 그대로 재현.
  **기각 조건**: 한 비트라도 다르면 API 를 되돌리고 원인을 찾는다.
- **H3 (동기화 = 스스로 낸 것과 같음)**: `syncLastCommand(v, w)` 뒤의 DWA 출력은 "DWA 가 직전에 스스로 (v, w) 를
  낸 경우" 의 출력과 같다 (동적 장애물 없음 — 양보 상한 기억 `yield_limit_last_` 는 이 API 가 건드리지 않으므로).
  ω 만 어긋난 경우도 포함한다 (r3 변이 R3 생존 — "ω 비교 생략" 을 가르는 시험이 없었다).
- **H4 (플러그인)**: 플러그인이 DWA 틱마다 실제로 낸 직전 명령으로 `syncLastCommand` 를 부르면, r3 의 전환 시험
  (정책 → DWA 틱당 |Δv| ≤ a_max·Δt_c, |Δω| ≤ α_max·Δt_c, 속도 제한, 벽 앞 제동)은 그대로 통과하고, 측정이 명령보다
  늦어도 `window_reset_v` 0.3 m/s · `window_reset_w` 0.5 rad/s 안이면 **자름(commitDwa)이 필요 없어진다**
  (DWA 창 = 계약 창: 둘 다 a 1.0·α 2.0, Δt_c 0.05 — `nav2_params.yaml:106-107` 가 robot_params 의 한계를 쓴다).
  자름은 계약 불변식으로 남기고, 측정이 0.3 m/s 넘게 늦어 DWA 가 측정값으로 창을 다시 잡는 경우에만 걸린다고 예측한다.
  `activate()` 를 더는 부르지 않으므로 게이트 상태(`gate_state_`, `gate_hist_`)가 보존된다.

## 설정

- 이미지 `amr-ai/rl:0.3` (ROS Humble, ORT C++ 1.23.2 `/opt/onnxruntime`), `colcon build -Werror`
  (`-Wall -Wextra -Wpedantic -Werror`, Release), `--packages-up-to amr_rl_controller`, DOMAIN 31, `ROS_LOCALHOST_ONLY=1`.
- 소유 경로: `src/amr_rl_controller/**`, `src/amr_navigation/include/amr_navigation/dwa_controller.hpp`,
  `src/amr_navigation/src/ros/dwa_controller.cpp`, `src/amr_navigation/test/test_dwa_sync*` (새 파일).
  **주의**: `src/amr_navigation/CMakeLists.txt` 는 내 소유가 아니고 gtest 를 이름 목록으로 등록한다 (glob 아님) →
  `test_dwa_sync.cpp` 는 `amr_navigation` 빌드에 저절로 들어가지 않는다. 시험 소스는 `amr_navigation/test/` 에 두고,
  빌드·실행은 내 소유인 `amr_rl_controller/CMakeLists.txt` 에서 그 소스를 가리켜 한다. `amr_navigation` 소유자에게
  등록 한 줄을 요청한다.
- 비트 동일 기준 기록: 같은 시험 소스를 `git archive a7da8a2` 의 `amr_navigation` 에 대고 빌드해(동기화 시험 제외
  매크로) 출력 기록을 만들고, 그 기록을 `src/amr_navigation/test/test_dwa_sync_reference.json` 으로 커밋한다.
  새 코드의 시험이 그것과 비트 비교한다. 별도로 a7da8a2 와 새 코드 두 빌드의 출력을 직접 대조한다 [측정].
- 변이 검사 (판정 규칙 — 결과 전에 고정): API 가드(비유한 입력 → 창을 측정값으로), `has_last_ = true` 생략,
  v·w 뒤바꿈, ω 무시(동기화에서 w 생략), 플러그인의 동기화 호출 생략, 플러그인이 낡은 값(내부 거울)으로 동기화.
  **각 변이를 적어도 한 시험이 잡아야 한다**. 놓치면 시험을 보강하고 다시 돌린다.

### 추가 사전 등록 (커밋 `67012e4` 2026-10-06 19:33:48, 결과 전 — 첫 운동학 실행 19:34:11) — 운동학 폐루프 재실행

(정정: 원래 "2026-10-06 19:55" 로 잘못 적음. 커밋 시각과 실행 목록 `A2_b1plugin_kin_*` 의 started 로 고쳤다. 사전 등록 순서는 그대로다 —
12 실행 모두 19:34:11 이후에 시작했다.)

r3 리뷰 반영 때의 운동학 폐루프 스모크(`/home/CAPYI_AI/runs/r3-controller/smoke/rev2_kin_*_to014_fixed`: heuristic_c,
`max_inference_ms` 0.14 로 간헐 시간초과 → 정책↔DWA 전환 반복)를 동기화 API 판으로 다시 돈다.
r3 실측 (activate 우회 + 자름, ffd359b): follow 6 m 자름 33 회 / DWA 67 틱, navigate (3, −6) 자름 80 회 / DWA 195 틱,
틱당 최대 |Δv| 0.050 · |Δω| 0.100.
- 예측: 틱당 |Δv| ≤ 0.050, |Δω| ≤ 0.100 그대로, 목표 도달 그대로, **자름 횟수는 DWA 틱의 5 % 이하**로 준다
  (운동학 시뮬레이터의 측정이 명령을 window_reset 0.3 m/s · 0.5 rad/s 넘게 늦게 따라가는 일이 드물다고 본다).
  자름이 5 % 를 넘으면 그 틱의 측정-명령 차를 로그에서 보고 원인을 적는다.
- 같은 조건에서 학습 정책 v12_s1 (모드 B) 도 시간초과를 일으켜(상한 0.012 ms — B 추론 p50 0.014 ms [측정, A2-2]) 돌린다.
- 시간·횟수는 공유 부하 하 예비값 (loadavg 기록). 시간초과가 벽시계라 실행마다 전환 횟수가 다르다.

## 결과

### 계약 v1.2.0 (H1) — 지지

- 기준 실행 (a7da8a2, 고치기 전) [측정, `A2_b1plugin_baseline`]: amr_navigation ctest 25/25 통과,
  amr_rl_controller 12 중 3 실패 — **셋 다 판 문자열·사례 수** (golden `contract_version` 1.2.0 ≠ "1.1.0", step 사례 12 ≠ 11,
  pytest `kContractVersion`, ONNX 고정물 메타데이터). 계산 실패는 없었다 = H1 예측대로.
- 고친 뒤 [측정, `A2_b1plugin_test_sync_t2`·`_t3`, 커밋 `850d6b2`·`162f8ea`]: `-Werror` 경고 0, 715 시험 실패 0
  (건너뜀 96 = 전부 cppcheck 2.7 자동 생략). golden 시험 11 개: 32 사례의 판·종류 목록 (decision 17·step 12·
  integrate 1·profiler 1·safety 1), d01–d17 관측(1e-5)·마스크(정확), s01–s12 이탈 상태, i01, **p01 을 amr_navigation 의
  `VelocityProfiler` 코어(PI 끔)로 대조 — 28 주기 × 5 서브스텝 1e-6 안 통과** (Gazebo 의 velocity_profiler_node 가 학습
  환경의 기준 모델과 같다는 전이 점검). f01 (safety_lite) 은 학습 환경 쪽 근사 φ 라 플러그인 대상이 아니다 — 목록 시험에만 센다.
- 판 검사: 같은 주 버전의 1.1.0·1.9.0 모델은 받고 0.9.0·2.0.0 은 거부 (시험 모델 4 개 추가).

### syncLastCommand — 기존 경로 비트 동일 (H2) — 지지

- API 추가 전 코드(git archive a7da8a2)로 결정적 시나리오 7 개(통로 추종, 곡선+추종 지연, 속도 제한 변경, 벽 막힘,
  목표 접근, 경로 밖 출발, 적재 25 kg) × 설정 2 개(코드 기본값, 배포 `nav2_params.yaml`+`robot_params.yaml` 을
  `--params-file` 로)를 3 회 기록 → 세 기록이 바이트 동일 [측정, `A2_b1plugin_dwa_ref_pre_api`].
- 새 코드의 `DwaSync.OutputsWithoutSyncAreBitIdenticalToPreApiReference`: **1180 틱 모두 double 비트 동일**.
  amr_navigation 기존 시험 25/25 + 린트 통과.
- 주의: 이 시나리오들에서는 두 설정의 출력이 서로도 같았다 (배포 파라미터가 실렸음은 시험이 확인 —
  `DWA.path_horizon` 4.0 대 기본 6.0 — 차이가 출력에 닿지 않았다). 동적 장애물 시나리오는 넣지 못했다 (트랙 나이가 벽시계라 실행마다 다르다).

### 동기화 의미 (H3·H4) — 지지, 단 변이 검사에서 시험 공백 1 개를 찾아 고쳤다

- `SyncToOwnCommandEqualsSelfEmitted` (곡선, ω 가 틱마다 바뀜): A 가 스스로 계산, B 는 그 틱만 A 의 명령으로 sync,
  C 는 갓 activate 한 DWA 에 sync → 이후 30 틱 출력이 A 와 비트 동일. `SyncSetsWindowCenterIncludingOmega`: v·ω, ω 만,
  v 만, 갓 activate 한 DWA. `NonFiniteOrFarSyncFallsBackToMeasuredCenter`.
- 플러그인 시험 15 개 통과: 1차 물결 전환 시험 3 개 (낡은 낮은 중심 → 이제 **자름 없이** 틱당 한계 안, 속도 제한, 벽 앞 제동),
  지연 0.15 m/s (< window_reset 0.3) → 자름 없음, 지연 0.35 m/s (> 0.3) → DWA 가 측정값으로 창을 다시 잡아 자름 1 회 + 다음
  틱 맞춤, ω 만 어긋난 전환 (0.3–0.45 rad/s) → 자름 없음, 긴 정책 구간(20 m) 뒤 폴백.
- **변이 검사** [측정, 격리 작업공간, cpu_slot, `A2_mutation/`]: 15 종.
  1 회차 12 잡힘 · 2 생존 · 1 빌드 오류. 생존 N4 (비유한 가드 제거) = **등가 변이** — 코어 `windowCenter` 의 NaN 비교가
  거짓이라 가드가 없어도 측정값 중심으로 떨어진다 (예측대로, 가드는 방어용으로 둔다). 생존 **P3 (플러그인이 v 만 sync, ω 는
  DWA 기억)** = 시험 공백: ω 차 문턱 0.2 가 DWA 창 폭(±0.1)과 정확히 맞물려 낡은 창의 끝 값이 계약 창 경계에 닿았다 →
  문턱 0.3 으로 고쳤다 (`863d4ff`). O1 은 변이 정의가 `-Werror` 미사용 변수로 컴파일되지 않아 다시 정의했다.
  2 회차 (P3·O1·N3): 3/3 잡힘. **최종 15 중 14 잡힘, 남은 1 은 등가 변이.**
  잡은 시험: N1 (has_last_ 안 켬) ← 갓 activate 한 DWA 검사 하나뿐 (`162f8ea` 에서 더함 — 그 전에는 이것도 생존했을 것이다, 미측정).
- **운동학 폐루프 재실행** [측정, Gazebo 없음, `A2_kin/`, loadavg 4–10, 커밋 `863d4ff` 내용]:

| 실행 | 정책 | 시간초과 상한 | 결과 | 결정 (정책/폴백셀/DWA/정렬) | DWA 틱 | 전환 | 자름 | 틱당 최대 \|Δv\| / \|Δω\| |
|---|---|---|---|---|---|---|---|---|
| r3 rev2 follow (ffd359b, activate 우회) | heuristic_c | 0.14 ms | 성공 | 61/14/34/0 | 67 | 22 | **33** | 0.050 / 0.100 |
| r3 rev2 nav (3,−6) (ffd359b) | heuristic_c | 0.14 ms | 성공 | 27/38/98/10 | 195 | 42 | **80** | 0.050 / 0.100 |
| follow to014 | heuristic_c | 0.14 | 성공 | 114/0/1/0 | 1 | 0 | 0 | 0.050 / 0 |
| follow to0085 | heuristic_c | 0.085 | 성공 | 1/0/71/0 | 141 | 1 | 2 | 0.050 / 0 |
| nav to0085 | heuristic_c | 0.085 | 성공 | 1/2/85/15 | 170 | 3 | 3 | 0.050 / 0.100 |
| follow to010 | heuristic_c | 0.10 | 성공 | 80/7/18/0 | 35 | 9 | 1 | 0.050 / 0 |
| nav to010 | heuristic_c | 0.10 | 성공 | 24/28/53/18 | 105 | 21 | 0 | 0.050 / 0.100 |
| follow to009 | heuristic_c | 0.09 | 성공 | 0/1/71/0 | 140 | 1 | 2 | 0.050 / 0.100 |
| nav to009 | heuristic_c | 0.09 | 성공 | 0/0/94/14 | 187 | 0 | 4 | 0.050 / 0.100 |
| follow v12_s1 | **학습 정책 v12_s1 (B)** | 25 (기본) | 성공 9.5 s | 76/0/0/9 | 0 | 0 | 0 | 0.049 / 0.100 |
| nav v12_s1 | v12_s1 | 25 | 성공 10.1 s | 84/0/0/17 | 0 | 0 | 0 | 0.048 / 0.100 |
| follow v12_s1 to014 | v12_s1 | 0.14 | 성공 | 25/0/48/0 | 95 | 16 | 2 | 0.050 / 0.100 |
| nav v12_s1 to014 | v12_s1 | 0.14 | **실패 (하네스)** — bt_navigator 목표 응답 유실, 로봇 정지 | — | — | — | — | — |
| nav v12_s1 to014 재시도 | v12_s1 | 0.14 | 성공 | 81/0/8/16 | 15 | 7 | 3 | 0.050 / 0.100 |

  - 동기화 판 합계: DWA 틱 889 중 자름 17 (**1.9 %**, r3 판 262 중 113 = 43 %). 예측 "5 % 이하" 는 합계로는 맞고,
    실행별로는 0–20 % (v12 nav 재시도 3/15 가 넘었다). 자름 경고 로그 (2 s 제한) 9 건을 보면 전부 **측정이 명령보다
    window_reset 를 넘게 늦은 틱** — 출발 가속 중 0.40 명령에 DWA 가 0.135 를 내거나 (측정 ≈ 0.1 로 창을 다시 잡음),
    제자리 회전 중 ω 명령 −0.7 에 DWA 가 −0.17–−0.30 을 낸 경우다. 즉 남은 자름은 동기화 결함이 아니라 **DWA 자체가
    측정 중심으로 창을 다시 잡는 규칙** 때문이고, 플러그인 없이 DWA 만 쓰면 이런 틱에 한 틱 −5 m/s² 급 명령 변화가
    그대로 나간다 (하류 저크 제한 프로파일러가 완화) — 시간초과로 전부 DWA 인 실행(to009·to0085)에서도 1–2 % 자름이 났다.
  - 시간초과 상한은 벽시계 추론 시간과 비교하므로 같은 상한에서도 부하에 따라 전환 수가 크게 달랐다 (0.085·0.09 는 거의 전부
    DWA). 시간·전환 수는 공유 부하 하 예비값이다.
  - 학습 정책 v12_s1 이 운동학 시뮬레이터에서 follow 6 m · navigate (3, −6) 를 모두 정책 결정만으로 성공했다 (플러그인 안
    추론 p50 0.14 ms / p99 0.31–0.35 ms).

### 1차 물결 r3 변이 검사 재실행 (코어·엔진·ONNX)

r3 의 하네스(`/home/CAPYI_AI/runs/r3-controller/mutation/mutate.py`, 경로만 바꾼 사본)를 현재 코드(`83b29a3`)에 다시 돌렸다
[측정, `A2_b1plugin_r3mutation_rerun`, cpu_slot, loadavg 5.6 → 5.2]: **r3 의 28 종 28 잡힘** (M1–M15 관측·마스크·적분, E1–E8 엔진,
P1–P5 ONNX 검사) + A2-5 정렬 변이 3 종 3 잡힘. 계약 v1.2 반영(판 문자열·golden 시험 확장)이 기존 시험의 검출력을 잃지 않았다.

### 긴 정책 구간 뒤 폴백의 경로 창 (A2 중 새로 의심한 위험) — 문제 아님

내부 DWA 의 `PlanWindow` 색인은 DWA 가 돈 틱에만 앞으로 가서 (직전 색인에서 앞쪽 200 점 = 10 m 만 찾음), 정책이 10 m 넘게
몬 뒤 폴백하면 첫 틱에 로봇 뒤쪽 점을 최근접으로 잡는다고 의심했다. 30 m 고정 경로, 정책 20 m 뒤 폴백 시험 [측정,
`A2_b1plugin_planwin_probe`]: 20 틱 동안 감속·회전 없음 — 색인이 호출마다 200 점씩 따라잡아 둘째 틱부터 맞고, 직선에서는 첫
틱의 영향이 보이지 않았다. BT navigate 는 1 Hz 재계획으로 경로를 새로 보내므로 더 짧다. 회귀 시험으로 남겼다
(`DwaAfterLongPolicyStretchTracksPathAhead`). 곡선 직전 폴백은 재지 않았다 (미측정).

## 해석

- 플러그인은 계약 v1.2.0 의 플러그인 해당 golden 31 개와 일치하고 (f01 은 대상 아님), 하류 프로파일러 코어도 학습 기준 모델과 같다.
- `syncLastCommand` 는 부르지 않으면 비트 단위로 무해하고, 부르면 "DWA 가 스스로 낸 것" 과 비트 단위로 같다. 1차 물결의
  activate() 우회를 걷어냈으므로 게이트 상태(`gate_state_`, `gate_hist_`)를 지우지 않는다 — 단 **게이트를 켠 상태의 시험은
  없다** (배포 `yield_gate: false`, 게이트 상태가 보존되는지 직접 관측하는 시험은 만들지 못했다 — 코드상 sync 가 그 멤버를
  건드리지 않는 것만 확인).
- 자름은 거의 없어졌지만 남는다 — DWA 의 window_reset 규칙 때문이고, 계약 불변식(틱당 a·Δt)은 자름이 지킨다.
- `amr_navigation/CMakeLists.txt` 가 내 소유가 아니어서 `test_dwa_sync` 는 amr_rl_controller 가 빌드한다 → amr_navigation 만
  시험하는 CI 에서는 이 시험이 돌지 않는다 (요청으로 넘긴다).

## 다음

- amr_navigation 소유자에게: `CMakeLists.txt` gtest 목록에 `test_dwa_sync` 등록 (ROS 의존이라 `test_ros_plugins` 와 같은 방식,
  `NAV2_PARAMS_FILE`·`ROBOT_PARAMS_FILE`·`DWA_SYNC_REFERENCE_FILE` 정의, nlohmann_json 링크). 등록되면 amr_rl_controller 의
  임시 블록을 지운다.
- `yield_limit_last_` (양보 상한 기억)는 sync 가 건드리지 않는다 — 정책 구간 동안 낡을 수 있으나 상승만 a·Δt 로 묶는
  보수적 방향이다 (미측정, 동적 장애물 단계에서 확인).
- 게이트(yield_gate) 를 켜고 RL 을 쓸 계획이 생기면 게이트 상태 보존 시험을 먼저 만든다.

## 리뷰 반영 (2026-10-06 21:29 KST 이후 작성, 이 절을 담은 커밋 시각이 상한)

- 리뷰는 코드·측정 주장을 모두 재현했다. 재현 항목: 깨끗한 빌드 -Werror 경고 0 · 723 시험 0 실패, ORT 없는 경로 700 시험 0 실패,
  a7da8a2 기준 비트 동일, 운동학 12 실행 DWA 틱 889 · 자름 17 (1.9 %).
- 리뷰는 가드를 되돌린 변이 9 종도 돌렸고, 모두 잡혔다 (`/home/CAPYI_AI/runs/bonus1/b1-plugin/review/out_mut/`).
- 이 항목에서 고친 것은 "추가 사전 등록" 절의 시각 하나다. 원래 "19:55" 로 적었고, 실제는 커밋 `67012e4` 19:33:48 이다.
  사전 등록이 결과보다 먼저였다는 점은 커밋 시각으로 확인된다.
