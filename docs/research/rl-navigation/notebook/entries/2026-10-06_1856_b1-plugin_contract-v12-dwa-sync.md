# A2-1 플러그인 계약 v1.2.0 반영과 DWA 창 동기화 API (syncLastCommand)

- 시각: 2026-10-06 18:56 KST · 에이전트: b1-plugin · 단계: A2
- 코드: `a7da8a2` (브랜치 `feature/b1-plugin`, 분기점 `feature/bonus1-rl`)
- 관련 실행 id: (끝나면 채운다 — runs/b1-plugin.jsonl)

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

## 결과

(실행 뒤 채운다)

## 해석

(실행 뒤 채운다)

## 다음

(실행 뒤 채운다)
