# P5b 구역 진입 몸체 판정 뒤 Gazebo 재확인 (안전 · F1 C1 · F5 F5L) — 사전 등록

- 시각: 2026-10-16 01:20 KST (작성) · 에이전트: b4-p5b · 단계: P5b (`bonus4_completion_plan.md` §3 F1 · F3 · F5, P5 의 미측정 · 미달 메우기)
- 코드: 고침 `effed5b` (구역 진입 몸체 판정), 도구 `6b82afe`, 이 항목을 커밋한 `feature/b4-p5b` 커밋 = **사전 등록 커밋**. Gazebo 실행은
  이 커밋의 `git archive` 사본(`/home/CAPYI_AI/runs/b24/b4-p5b/ws_new`, `ws_old`)으로만 한다 — 드라이버 · 분석기도 그 사본의 것을 쓴다
  (`ws_new/tests/integration/fleet_p5/p5b_launch.sh`). 사본 커밋은 갈래마다 `HEAD.txt`.
- 근거: P5 결과 `p5.md` §0-0 · §3.0 · §5-1 (원판 동시 점유 15 건과 두 기제), P5 사전 등록 `…_1540_b4-p5_gazebo-confirm-prereg.md` (C1 · F5L 설계 ·
  판정 규칙 · 부하 규칙 · 「이어받기」의 운영 규칙), COORDINATION §12 (메모리) · §13 (이 과제).
- 관련 실행 id: `P5b_*` (`runs/b4-p5b.jsonl`), 원자료 `/home/CAPYI_AI/runs/b24/b4-p5b/raw/<태그>/` (태그는 P5 와 같은 꼴 `p5_<실험>_p<짝>_<갈래>[r<n>]`
  — 원자료 루트가 다르다).

## 질문

1. **(a) 안전 재확인**: 구역 진입을 몸체 원판으로 판정하게 고친 코드(`effed5b`)로 P5 에서 사건이 났던 묶음(C2 분산 갈래 · C3 두 갈래 — 사건이
   몰린 `dock_b_approach` 를 지나는 시나리오 12 계획 그대로)을 다시 돌면, 실제 자세(정답 odom 원판 r 0.36 m) 기준 구역 동시 점유와 결정점
   정지 침범이 P5 보다 줄었나, 0 인가?
2. **(b) C1 — F1 회귀**: 같은 계획 8 짝에서 분산 모드가 중앙 모드보다 완료 작업 수 · makespan 이 나빠지는가 (P5 미측정)?
3. **(c) F5L — F5 도크 배치**: 옛 배치(`fd17e49` 의 `traffic_zones.yaml`, 도크 ±1 m · approach_m 1.5) 대 새 배치(±0.75 m · 2.0), 중앙 모드 8 짝 —
   새 배치가 도크 구역 동시 점유 · 완료 · 도킹을 나쁘게 하지 않는가 (P5 미측정)?

## 무엇을 고쳤나 (`effed5b`, Gazebo 전 — 결과 전)

- `traffic_zones.zone_visits(radius)` · `request_chain(held)` · `traffic_geometry.discs_touch_polygon`: 방문 = 원판이 구역과 겹치는 구간, 요청 ·
  막힘 거리 = 원판이 처음 닿는 곳까지, 허가 없이 이미 걸친 구역은 거리 0 으로 요청 · 막음 (fail-closed), 몸체 점유도 원판으로.
- **분산 경로 게이트와 중앙 traffic_manager 가 같은 함수를 쓴다 → 중앙 모드에도 적용된다** (`traffic.yaml zones.body_entry: true`, 중앙
  `TrafficConfig` 기본값 true). 영향: 중앙 관리자의 토큰 요청 구역 · hold 시작 거리 · 몸체 점유 등록 · 양보 포켓 경로의 미리 받는 구역이 모두
  원판 기준이 된다. 그래서 (b) C1 과 (c) F5L 의 중앙 갈래도 고친 코드다 (P5 중앙 갈래와 코드가 다르다).
- 시험 [측정, `P5b_test_amr_fleet_effed5b` · `P5b_mut_gate_effed5b`]: `test_body_entry.py` 22 개, amr_fleet colcon test 939 / 실패 0 / 건너뜀 5
  (amr_bringup 미source 4 — 따로 4 통과, 연구 스크립트 없음 1), 가드 변이 11/11 KILLED.
- 결정점 거리: 몸체 기준이면 결정점 식(p0.md §5)의 차체 앞 절반 0.30 이 빠져 1.55 m ≤ 2.0. 배포 배치에서 묶이지 않는 이웃 구역 사이 몸체
  간격(g − 2r) 최소는 도크 같은 쪽 쌍 1.78 m ≥ 1.55 (시험 고정). 중심 기준 `spacing_conflicts` 는 바꾸지 않았다.

## 가설과 예측 (결과를 보기 전에)

- **H-a1 (원판 동시 점유)**: P5 의 두 기하 기제(요청 없는 스침 · 막힌 자리의 몸체 침범)는 사라진다. **0 을 예측하지는 않는다** — 남는 길:
  (i) 서 있다가 출발할 때 이미 원판 진입 0.5–0.7 m 안 (오프라인 재판정 3 건, 아래), (ii) plan 없음 fail-open (`p3.md` §10.8), (iii) 게이트를 거치지
  않는 PERCEIVING · DOCKING 단계의 양보 이동, (iv) 판정은 추정 자세(AMCL) · 사건은 정답 자세, (v) 중앙 교통 관리자의 결정(분산 모드에서도
  서버가 살아 있으면 hold 를 따른다). 예측: 0–2 건, 비율 비교는 "P5 보다 적다" (같은 노출이면 ≤ 1 건에서 단측 p < 0.05).
  버리는 조건: 사건 ≥ 3 이거나 사건 기제가 여전히 "요청 없음" 이면 고침이 그 기제를 막지 못한 것이다.
- **H-a2 (결정점 정지 침범, `gate_stops_before_grant` clear_m < 0)**: 줄 것이다. 그러나 P5 같은 묶음의 기준이 **2/149** 라 새 쪽이 0 이어도
  Fisher 단측 p ≈ 0.25 — **이 지표로는 가를 수 없다 (검정력 없음)**. "가르지 못함" 을 예측하고, 값은 기술로 본다 (P5 전체 21/374 를 맥락으로).
- **H-a3 (불리한 예측 — 활성)**: 새 판정은 더 일찍 · 더 많은 구역을 요청하고, 허가 없이 걸친 구역에서는 멈춘다 → 대기 · 교착이 늘 수 있다:
  막힘 거리 0 사건 > 0, traffic DEADLOCK · UNRESOLVED 증가, 완료 감소 가능. 특히 도크 접근 구역 서쪽 변을 따라 남북으로 지나는 로봇은 다른 로봇이
  도크에 있으면 이제 기다린다 (P5 사건 15 건의 자리). 기술만 한다 (P5 와 시각 · 부하가 다르고 짝이 아니다).
- **H-b (C1)**: P5 H1 그대로 — 분산 모드는 완료 작업 수에서 회귀하지 않는다 (서버가 살아 있으면 분산 모드도 중앙 할당), makespan 은 분산이
  조금 길 것 (투표 왕복 · 확정 왕복). 버리는 조건: 완료 수 부호 검정 p < 0.05 로 분산 < 중앙.
- **H-c (F5L)**: P5 H5 그대로 — 새 배치의 도크 구역 동시 점유(고정 지표 = 옛 큰 상자)는 옛 배치보다 적거나 같다. 덧붙임: 몸체 판정에서는 옛
  배치의 approach_m 1.5(몸체) = 중심 ≈ 1.86 m 이라 옛 배치의 결정점 미달도 거의 풀린다 → 차이는 작을 것 (가르지 못함 예측, 확신 낮음).

## 설정 (P5 와 같은 것 · 다른 것)

| 항목 | 값 |
|---|---|
| 같은 것 | P5 사전 등록의 시스템 · 서버 컨테이너 배치 · 작업 묶음 · 시각(t_base) · 준비 판정 · 끝 조건 · 기록기 · 지표 정의(`p5_analyze.py`, 바꾸지 않음 — sha256 아래) · 짝(같은 계획 · 같은 장애 일정, 먼저 띄울 갈래 · 도메인 번갈아 (21, 23)/(25, 27)) · **차례로**(파일럿 결정) · T_START 60 s · DDS SHM 기본 · 이미지 `amr-ai/rl:0.3` |
| 부하 규칙 (P5 와 같다) | 짝 시작 조건 loadavg 1 분 < 30 · 5 분 < 50 · 15 분 < 80 두 번 연속 (대기 상한 3 h — 넘으면 그대로 시작하고 기록). 갈래 중 loadavg 5 분 > 120 이 10 분이면 부하 중단 → **그 갈래만** 시작 조건을 다시 기다린 뒤 새 태그로 (연속 수에 넣지 않음). 작업 게시 · 예정 장애 전 무효(CLOCK_STALL · READY_TIMEOUT)는 그 갈래만 다시 (같은 갈래 연속 6 번이면 그 실험 멈춤 · 진단). 작업 게시 뒤 무효는 짝 무효 · 짝째로 다시 (연속 3 번이면 그 실험 멈춤). 짝 안 갈래는 차례로. |
| 다른 것 (운영 — 지표 · 판정 무관) | 원자료 루트 `/home/CAPYI_AI/runs/b24/b4-p5b/raw`, 컨테이너 이름 · gz 슬롯 id `b4-p5b`, run_id 머리 `P5b_`, 스택 `docker --memory 28g` · 서버 `6g`, 짝 시작 조건에 MemAvailable ≥ 32 GB 더함 (COORDINATION §12). 측정 시간대 `CPU_SLOTS=0 exclusive.sh b4-p5b` (gz 슬롯 둘). **b2-rerec 의 `RECORDING_DONE` 이 생긴 뒤에만** (`p5b_launch.sh` 가 기다린다) |
| 빌드 | 이 커밋의 `git archive` → `ws_new` (새 배치), `ws_old` = 같은 사본 + `fd17e49` 의 `src/amr_fleet/config/traffic_zones.yaml` (차이 그 파일 하나). `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic -Werror" --packages-up-to amr_bringup`, cpu_slot |

### 실험 (`p5_plan.EXPERIMENTS` — 표와 코드가 다르면 코드가 실행된 것이다)

| id | 짝 | 갈래 a · b | 작업 · 장애 | 목적 | 노출 기준 (P5) |
|---|---|---|---|---|---|
| SC2 | 4 | 분산 · 분산 (C2 분산 갈래 복제 `dist_s2a` · `dist_s2b`) | C2 그대로: A @0, B @90, 서버 kill @60, 새 서버 @300 | (a) | P5 C2 분산 8 갈래: 진입 418, 사건 3 |
| C3 | 8 | `dist_dead`(서버 kill @−30) · `dist_alive` | P5 C3 그대로 (lead 40 s) | (a) + C3 보충 | P5 C3 16 갈래: 진입 562, 사건 5 |
| C1 | 8 | `central` · `distributed` | A @0, 무장애 | (b) F1 회귀 | 미측정 |
| F5L | 8 | `central_old` · `central_new` | A @0, 무장애 | (c) F5 배치 | 미측정 |

### 순서와 줄이는 규칙 (결과 전에 고정 — 그대로 한다)

1. **순서 = 우선순위**: `SC2,C3,C1,F5L` (과제 지시 (a) → (b) → (c)). 짝 안 먼저 띄울 갈래 · 도메인은 짝 번호에 묶인 그대로.
2. **마감**: 새 짝(재시도 포함)은 **드라이버 첫 시작 + 40 h** 뒤에 시작하지 않는다 (`p5b_launch.sh` 가 처음 한 번 `raw/DEADLINE.txt` 에 적고
   다시 띄워도 그 값을 쓴다). 진행 중인 짝은 끝낸다. 남은 짝은 "미측정 (줄이는 규칙)". 어림 [추정 — P5 유효 갈래 벽시계, RTF 0.2–0.3]:
   SC2 4 짝 × 2 × 45 분 ≈ 6 h, C3 8 짝 × 2 × 31 분 ≈ 8.3 h, C1 · F5L 각 8 짝 × 2 × 33 분 ≈ 8.8 h — 합 ≈ 32 h + 무효 · 부하 대기.
3. **n 은 줄이지 않는다**. 마감에 걸려 n < 8 로 끝난 비교는 얻은 n 으로 분석하되 그 n 의 부호 검정 최소 양측 p 를 먼저 적는다 (n = 7: 0.0156,
   6: 0.031, 5: 0.0625 — **n ≤ 5 는 어떤 결과로도 기각하지 못한다**, "가르지 못함" 만 쓸 수 있다).
4. 결과 값을 보고 순서 · 마감 · n 을 바꾸지 않는다. 오케스트레이터가 더 일찍 멈추라고 하면 같은 순서에서 자른다.

### 중단 규칙 (코드 결함 감지 — 지표 값으로 판정 규칙을 바꾸지 않는다)

- **SC2 짝 1 의 두 갈래가 모두 유효하고 둘 다 완료 ≤ 2/8 이면** 드라이버를 멈추고(STOP) 진단한다 (P5 C2 분산 갈래 완료 6–8/8 — 고친 판정이
  체계적으로 진행을 막는 결함 의심). 고치면 그 전 갈래는 "결함 발견" 자료로 남기고 노트에 "결과 뒤 변경 (코드)" 로 적은 뒤 SC2 를 처음부터.
  (감독 에이전트가 살아 있을 때만 적용된다 — 드라이버는 이 규칙을 모른다.)
- 드라이버의 연속 무효 규칙은 위 「설정」 그대로.

## 판정 규칙

- **(a) 안전 재확인** — `docs/research/fleet-distributed/tools/p5b_analyze.py safety --root <P5b raw> --sets SC2,C3 --baseline-root
  /home/CAPYI_AI/runs/b24/b4-p5/raw --baseline-sets C2:distributed,C3`. 갈래는 각 원자료의 `pairs/<실험>_p<k>.done` 태그만. 기준과 새 값은 **같은
  함수**로 센다 (지표 `p5_analyze` metrics, 기제 분류 `p5_review_check.incidents`).
  - **안전 불변식 (1차)**: 원판 동시 점유 · 믿은 허가 겹침 · 픽업 중복 · 낮은 epoch 수용 중 하나라도 > 0 이면 **미달** — 사건 표(시각 · 구역 ·
    로봇 · 자세 · 모드 · 단계 · 깊이 · 기제)를 결과 문서 앞에. 0 이면 노출(진입 수 E)로 rule of three 상한 3/E 를 적는다.
  - **비율 (1차)**: 원판 사건 / 진입, 새 대 P5 같은 묶음 — 조건부 정확 이항 검정 단측 p = P(X ≤ x_new), X ~ Bin(x_new + x_old,
    E_new/(E_new + E_old)). p < 0.05 면 "P5 보다 적다", 아니면 "가르지 못함".
  - **결정점 정지 침범 (2차)**: clear_m < 0 비율, Fisher 정확 검정 단측. 위 H-a2 대로 검정력이 없음을 함께 적는다.
  - **기술만**: 기제별 사건 수 · 시간, 중심 기준 사건, 허가 없는 진입 초, 막힘 거리 0 사건 수, 완료 · makespan · DEADLOCK · UNRESOLVED · RTF ·
    부하. "P5 보다 나빠졌다/좋아졌다" 를 판정하지 않는다 (시각 · 부하 · 짝 아님).
  - C3 보충: P5 의 C3 판정 규칙(makespan · 평균 작업 시간 · 이동 거리 부호 검정, Holm)을 고친 코드로 그대로 적용해 적는다 (`p5_analyze.py exp C3`).
- **(b) C1**: `p5_analyze.py exp C1` (P5 사전 등록 규칙 그대로). 짝 차 b − a = 분산 − 중앙. 완료 수 부호 검정 p < 0.05 이고 분산 < 중앙이면
  "회귀". p ≥ 0.05 면 "가르지 못함" 이고, 95 % CI 하한 > −1 작업이면 "비열등(마진 1 작업) 충족" 을 덧붙인다 (모든 짝 차가 같아 CI 가 정의되지
  않으면 그렇게 적는다). makespan 은 같은 검정 (보조). 두 모드 모두 고친 코드다.
- **(c) F5L**: `p5_analyze.py exp F5L` + `p5b_analyze.py f5l`. 짝 차 b − a = central_old − central_new. 도크 구역 동시 점유(고정 지표 원판 · 중심, 각
  배치 자기 상자) 합 옛 대 새, 완료 · makespan 부호 검정, 도킹 ≤ 2 cm · 1° 비율과 오차 중앙값 · 최대. **"새 배치가 나쁘지 않다" 는 고정 지표 원판
  사건 새 ≤ 옛 이고 완료 회귀(부호 검정 p < 0.05 로 옛 > 새)가 없을 때만** 쓴다.
- 중앙 모드 안전 (기술): `p5b_analyze.py central --sets C1:central,F5L` — 서버가 살아 있는 중앙 갈래의 원판 · 중심 사건과 노출. 중앙 모드는
  정족수 안전을 주장하지 않는다 (P5 사전 등록). P5 맥락: C2 중앙 8 갈래 원판 18 · 중심 17 (15 건이 서버 장애 구간), 파일럿 중앙 3 갈래 0.
- 유효 실행 · 문구: P5 와 같다 — 기록기 DONE · 감시 중단 없음. 차이를 가르지 못하면 "가르지 못함" ("같다" · "효과 없음" 을 쓰지 않는다).
  공유 서버 값은 "공유 부하 하 예비값" (loadavg 함께).

## 기준 — P5 같은 묶음 [계산, `P5b_offline_baseline_p5`, `/home/CAPYI_AI/runs/b24/b4-p5b/offline/baseline_p5/`]

P5 원자료를 `p5b_analyze.py` 로 다시 센 값 (P5 문서 값과 같다 — 도구 검증): C2 분산 8 + C3 16 = **24 갈래 · 진입 980 · 원판 사건 8 (중심 1)**,
기제 요청 없음 7 (14.7 s) · 요청 · 막힘 · 구역 안 대기 1 (60.6 s), 결정점 정지 149 중 음수 2 (최소 −0.17 m), 허가 없는 진입 103.5 s, 완료 137/144,
DEADLOCK 2 · UNRESOLVED 3, RTF 0.141–0.278, 막힘 거리 0 사건 0 (옛 판정에는 없는 동작). 중앙 C2 8 갈래: 진입 573 · 원판 18.
검정력: 새 쪽 노출이 같을 때 사건 0 → 단측 p = 0.0039, 1 → 0.0195, 2 → 0.055 (기각 못 함).

## 오프라인 재판정 — P5 사건 15 건 [계산, 근사, `P5b_offline_replay_p5_incidents`, `tools/p5b_replay.py`]

P5 에는 plan 전체가 없어 허가 없는 로봇의 실제 정답 궤적을 plan 대신 넣고 옛 · 몸체 판정을 0.2 s 마다 냈다. 옛 판정 재현은 P5 `gate` 기록과
맞다 (요청 없음 13 건은 재판정도 요청 0, 막힌 2 건은 기록보다 0.7–0.8 s 이르게 막음 — 궤적 근사 차이). **몸체 판정은 15/15 건을 원판이 닿기
전에 막는다**; 막힌 자리 → 실제 닿은 자리 궤적 길이 중앙 1.89 m, 12/15 가 P5 에서 잰 막힌 뒤 정지 거리 0.81 m 를 넘는다. **나머지 3 건(0.55 ·
0.58 · 0.72 m — C3 p6 alive · P p1 a · S p2 a 의 amr_05)은 사건 30 s 전부터 dock_b 북서 모서리 0.4 m 옆 (26.06, 13.96) 에 서 있던(MOVING 단계,
속력 0) 로봇이 출발한 경우**다 — 출발하자마자 막히므로 순항 중 정지 거리 0.81 m 가 맞지 않지만, 막힘 신호 지연 동안 움직이면 원판이 살짝
걸칠 수 있다 [추정] (H-a1 의 (i)). 근사의 한계: 막혔다면 궤적 자체가 달랐을 것이고, 실제 plan 은 재계획으로 다를 수 있다.

## 분석 스크립트 · 도구 (결과 전에 고정) — 사전 등록 커밋 = 이 항목의 커밋

| 파일 | sha256 |
|---|---|
| `docs/research/fleet-distributed/tools/p5b_analyze.py` | `d08d5a6667e0c0beb476c0d0cdf911053122c62f8732166407a6ea439f8a2048` |
| `docs/research/fleet-distributed/tools/p5_review_check.py` (P5 리뷰, 바꾸지 않음) | `ee9d7b8de382f5402f65dc8471155fb06863d5c9b5f0fc83c83a5a564eea71b3` |
| `tests/integration/fleet_p5/p5_analyze.py` (P5 마지막 판 `30e78dad…` 와 같음) | `30e78dad39cd48f84bb7af9bb0c182f528684b435fe6a981e9a67b9c2d1ae9c3` |
| `tests/integration/fleet_p5/p5_driver.py` (P5b 더하기 — 기본값 P5 동작) | `66846970b9a5c92ae7a7cb4dd5779156f3a6ac79cb3374b4acb71849bef3e072` |
| `tests/integration/fleet_p5/p5_plan.py` (SC2 더함) | `64b0653f74e2a8195a4551998d5f466bcada7ee18685db755520d488be0dc2e0` |
| `tests/integration/fleet_p5/p5_recorder.py` (P5 와 같음) | `d0ff03fa9492afe7b5b102ddb8b378104b805c58d7ad6e648b10d3b83153eec6` |
| `tests/integration/fleet_p5/p5_container.sh` · `p5_server.sh` (P5 와 같음) | `d9549bda…2b80` · `6ef269ab…b23b` |
| `tests/integration/fleet_p5/p5_agent_ctl.py` (P5 와 같음, R 안 씀) | `586e19a4bba00f1540cb689a2beafc740dd87a6f9f75a63fe33822dcaf6f01dd` |
| `tests/integration/fleet_p5/p5b_launch.sh` (새) | 이 커밋 (`git show <커밋>:tests/integration/fleet_p5/p5b_launch.sh \| sha256sum`) |
| `tests/integration/unit/test_fleet_p5b.py` | `830e7e7ee51af000ce70846680b344fe1d5dd54fb403b861d8069f524fdf6b86` |
| `src/amr_fleet/test/test_body_entry.py` | `d2a5e97faa967487e35e188d894b20bc0214962b17230cc857a35e883ea81a0b` |

- 시험 [측정]: tests/integration/unit 159 통과 (`P5b_test_unit_6b82afe` — test_fleet_p5b 11 개 추가), 분석기 가드 변이 8/8 KILLED (조건부 이항
  방향 · Fisher 방향 · 불변식 문턱 · 모드 거르기 · 막힘 거리 0 조건 · F5L 회귀 방향 · F5L ≤ · 판정 문턱).
- 파이프라인 시험 [측정, Gazebo 없음, 셈 밖, `P5b_pipeline_dry_6b82afe`, `/home/CAPYI_AI/runs/b24/b4-p5b/dry/`]: 실험 `dry` 두 갈래 유효, 장애 4 개
  주입 (pause 15.05 · unpause 25.05 · kill 35.05 · 새 서버 45.55 sim — 예정 대비 ≤ 0.55 s), 컨테이너 `amr-ai-b4-p5b-*`, run_id `P5bdry_*`, 짝 시작
  조건 기록에 MemAvailable 101.9 GB, 서버 traffic_manager 시작 줄 `body_entry=True`.

## 결과

(Gazebo 실행 뒤 채운다 — 결과 뒤 변경은 시각 · 근거 · 해시와 함께 아래에 덧붙인다.)

## 해석

## 다음
