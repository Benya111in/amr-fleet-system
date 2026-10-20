# 보너스 4 결과 — 분산 플릿 관리 (명세 3 줄 ↔ F1–F6)

- 작성 2026-10-16 · 에이전트 b4-p5b · 브랜치 `feature/b4-p5b` (`feature/bonus4-fleet` `a00722b` 에서). 완료 정의
  [`bonus4_completion_plan.md`](bonus4_completion_plan.md) (F1–F6, §5 판정 원칙, §6 변경 이력).
- 이 문서의 목적: 나중에 다시 조사할 때 이 한 문서로 **무엇을 했고, 무엇이 나왔고, 무엇을 주장할 수 있고 없는지, 원자료가 어디 있는지**를
  찾게 한다. 세부 표 · 재현 명령은 원천 문서에 있다 (§7 기록 지도). 수치마다 원천(문서 · 절)을 적었다.
- 새 측정은 P5b(§2.8, 원천 `p5.md` §6) 하나다. 나머지 수치는 원천 문서에서 옮겼다.
- 표기: **[측정]** 명령 · 원자료 있음 / **[계산]** 원자료의 오프라인 집계 / **[추정]** / **미측정** / **[문서주장]**. Gazebo 값은 모두
  **공유 부하 하 예비값**이다 (원천에 loadavg). "가르지 못함" 을 "같다" · "효과 없음" 으로 쓰지 않는다.

⟨§0 결론 · §1 판정표 — P5b 결과 뒤⟩

## 2. 단계별 요약

### 2.1 P0 — 사전 정비 [`p0.md`, `hold_failsafe.md` §0–§7.8]

- 통신 모의(`comm_emulator.py` — 링크별 결정적 지연 · 손실 · 버스트 · 분할 · 단절, 로봇 ↔ 로봇 포함), 플릿 노드 시간 축(노드 시계),
  어댑터 분리(`launch_adapters:=false`), 작업 게시판(`/fleet/task_board` TRANSIENT_LOCAL) · epoch 필드, 결정점 거리(교차로 1.5 → 2.0 m).
- 불리한 것: 도크 구역은 배치 간격 때문에 1.5 m 로 남아 결정점 거리 1.85 m 에 **미달** (→ F5), 재기동한 관리자가 게시판 이력으로 끝난 작업을
  다시 접수할 수 있다 (**완료 대조 없음** — P5 에서 실제로 났다, §2.7).
- traffic hold fail-open 결함(D1–D8)과 DDS 늦은 참여자 대응(D9–D14)을 고쳤다 — 시험 · 변이 [측정], Gazebo 짝 확인은 F4 (§2.6).

### 2.2 P1 · P2 — 투표 코어 · TLA+ · 협상 코어 [`protocol.md`, `verification.md`, `formal/fleet/README.md`, `negotiation.md`]

- 구역 투표 코어(5 대 중 3 대 정적 과반, 2 단계 진입, epoch · incarnation): Hypothesis 상태기계 5 대 모의 — CI 400 예제 · 깊은 탐색 5 시드 ×
  3 000 예제 **위반 0** (허가 상태에 닿은 단계 약 19 %), 보고서가 기각한 "생존 멤버 전원 REPLY" 변형은 10/10 시드에서 반례 [측정,
  `verification.md` §2]. 변이 31 개 모두 잡힘 — 다만 상태기계 단독의 힘은 경계 수준 (M8 은 손으로 만든 결정적 시험이 잡는다).
  주장 수준은 "시험됨 (n 회, 위반 0)" 이다.
- TLA+ (TLC, **유한 범위 모델 검사 — 증명 아님**): 기각된 초안 `RA_LiveMembers` 는 신뢰 채널에서도 반례(한쪽 견해 어긋남만으로 두 대 진입),
  정족수 투표 `QuorumVote` · 작업 잠금 `TaskLock` 은 표의 범위에서 위반 없음 [`formal/fleet/README.md`].
- CBAA 협상 코어: 오프라인 동기 라운드 모의 [계산] — 엄격 모드(이상적 정족수 잠금 가정) 중복 0/1 000 사례, 낙관 모드(잠금 없음)는 버스트
  손실에서 중복 0.1–0.2 % → 잠금 없는 운용은 쓰지 않는다 [`negotiation.md` §0].

### 2.3 P3 — ROS 통합 [`p3.md` §0–§9]

- 로봇마다 `fleet_agent`(순수 코어 + 얇은 노드), 메시지 8 종 · srv 1, 모드 다섯(NORMAL · HOLD_NEW · DEGRADED · ISOLATED · RESYNC), 작업 ID
  잠금(투표 코어 task 채널, 확정 뒤 풀지 않음), 에포크 펜스, 존 BT 노드. launch 시험 ①–④c 시드 1–3 통과 [측정, 가짜 실행기]: 허가 구간 겹침 0,
  낮은 명령 수용 0/9, 같은 작업 두 로봇 확정 경합에서 HELD 한 대, 분할 중 소수 쪽 픽업 0 · 중복 0. 변이 순수 37/37, ROS 수준 RM2 SURVIVED.
- 리뷰가 찾은 결함(중앙 작업 프록시 경로 누수, 원래 ④ 의 경합 0 = 공허)을 고쳤다.

### 2.4 P4 — 운동학 플릿 모의 (빠른 계층) [`p4.md` §0 · §4]

- **불리한 것**: 현행 hold 의미는 서버가 살아 있고 손실이 없어도 도크 구역을 침범했다 (A1h 6/30 · A5h 14/30 실행, 깊이 ≤ 0.18 m) — 도크 요청 거리
  1.5 < 결정점 1.85 m. 손실 0.4 에서 분산 교통이 느리다 (A4 − A1 +5.0 [3.2, 6.8] s). 모형 충실도 한계 (구역 밖 차체 막힘 없음, A/A ρ 0.96–0.99 —
  Gazebo 로 옮기면 안 된다).
- 제안 방식(A9) 대 현행 hold 의미(A1h): 완료 +15.4 [14.0, 16.7], 30/30 [측정, 모의].
- 모의의 투표 모드는 "차체 접촉 구간(s_touch)" 으로 정지선을 정했다 — 실제 경로 게이트는 그 대신 중심 방문을 썼고 (§2.7 의 원인), P5b 가 몸체
  판정을 실제 코드에 넣었다.

### 2.5 W — 실제 실행기 · BT 연결 [`p3.md` §10]

- 관리자 하트비트 · 존 노드 등록 · 분산 전용 트리(PickupLocked) · 서비스 리매핑 · 설치, `fleet_mode:=distributed` 인자 하나 (기본 central — 중앙
  모드 시험 전부 그대로). ZoneMoveTo 대신 **에이전트 경로 게이트**(plan → 사슬 사전 요청, 결정점 안 미허가면 정지).
- F5 를 배치로: 도크 구역 ±1 → ±0.75 m (같은 쪽 간격 2.0 → 2.5 m, 요청 거리 기본 2.0) — 근거는 운동학 모의(침범 6/30 · 14/30 → 0/30).
- 실제 실행기 5 + 에이전트 5 + 실제 관리자, 가짜 세계 launch W1 · W2 세 시드 판정 통과, 변이 12/12 KILLED, 회귀(W-regress3) 경고 0 · 실패 0
  [측정]. 불리한 것: P3 ④c launch 시험의 공허 검사는 기준 코드에서도 가끔 실패하는 경계 경주 (기준 1/20 · HEAD 2/20, 가르지 못함).

### 2.6 S12 — P0 hold 수정 Gazebo 짝 (F4) [`../fleet-traffic-deadlock/hold_failsafe.md` §7.9]

- 시나리오 12 짝 8 쌍(5 대 Gazebo 둘을 동시에), **정보 있는 짝 3 쌍뿐** (사전 등록 목표 6 — 16 갈래 중 8 갈래가 하네스 정지로 판정 전 멈춤).
- 판정 (a) 문자 그대로 "나빠지지 않음 (2 ≤ 8)" 이지만 대칭 변형은 2 대 2 — **수정의 효과는 보이지 않았다**. (b) 평가 가능 로봇 0/3 → 가르지 못함,
  (c) 3 로봇에서 성립 (관리자 쪽 DDS 정지 후보 — gdb 스냅숏 하나에 기댄 추정). 시나리오 12 통과율 수정 전 0/8 · 수정 후 1/8 (판정에 안 씀).
  하네스 정지 기제는 SHM 송신 경로 정지로 보였다 (gdb).

### 2.7 P5 — Gazebo 5 대 확증 (F1 · F2 · F3 · F5) [`p5.md` §0–§5]

- 줄이는 규칙(마감)으로 **C2 8 · C3 8 · P 2 · S 2 · X 2 짝** 만 돌았고 R · C1 · A/A 3–6 · F5L 은 미측정 (재개 뒤 69 갈래, 48.6 h, loadavg 50–200 에서 RTF 0.14–0.30).
- **안전 불변식 미달**: 최종 코드 분산 36 갈래 · 구역 진입 1 560 에 실제 자세 원판 동시 점유 14 건 (+ 고치기 전 1), 모두 도크 접근 구역, 중심 기준 1 건.
  기제 — 요청 없는 경계 스침 13 · 요청했으나 막힌 자리에서 몸체가 구역 안 2 (사건 시간의 72 %) — 둘 다 **진입을 경로 중심점으로 잰** 결과.
  믿은 허가 겹침 0, 픽업 중복 0 (최종 코드 — 고치기 전 3 건을 P5 가 찾아 고쳤다).
- C2(서버 kill): 1차 "장애 구간 완료" **가르지 못함** (p 0.375), 보조 전체 완료 분산 > 중앙 8/8 (p 0.0078), 분산 5 대 DEGRADED 4.94–5.39 s ·
  NORMAL 복귀 40/40. 중앙 재기동 관리자가 끝난 작업을 다시 실행 (합 38 픽업). 분산은 복귀 뒤 재할당 고리 (task_taken 154–406 번/갈래).
- C3(협상 대 중앙 할당): 세 지표 가르지 못함, 할당 8 짝 모두 결정적으로 같음, 협상 낙찰 1.6–3.4 s (P4 와 같은 방향 · 크기).
- **공허**: P 의 일부러 만든 작업 잠금 경합 K (경합 0 번), S 의 정지 뒤 재개 · split-brain (옛 서버가 재개 뒤 아무것도 못 보냄), 펜싱(낮은 epoch
  명령 도착 0). P 부분 분할 소수 쪽 ISOLATED · 복귀 4/4, X 통신 스트레스 HOLD_NEW 오탐 55–66/갈래 · DEGRADED 오탐 0.

⟨§2.8 P5b — 결과 뒤⟩

## 7. 기록 지도

| 무엇 | 어디 |
|---|---|
| 완료 정의 · 변경 이력 | [`bonus4_completion_plan.md`](bonus4_completion_plan.md) |
| 설계 근거 (리서치 보고서 사본) | [`report-2026-10-06.md`](report-2026-10-06.md) (원본 `/home/CAPYI_AI/research/reports/분산 플릿 관리 협상 할당과 폴백.md`) |
| 프로토콜 · 코어 검증 · TLA+ | [`protocol.md`](protocol.md), [`verification.md`](verification.md), `formal/fleet/README.md`, [`negotiation.md`](negotiation.md) |
| P0 · hold 수정 · S12 (F4) | [`p0.md`](p0.md), `../fleet-traffic-deadlock/hold_failsafe.md` (§7.9 = S12) |
| P3 · W 연결 | [`p3.md`](p3.md) (§10 = W) |
| P4 운동학 모의 | [`p4.md`](p4.md) |
| P5 · P5b Gazebo | [`p5.md`](p5.md) (§6 = P5b) |
| 노트 항목 (사전 등록 · 결과 뒤 변경) | `notebook/entries/` — P5 `2026-10-08_1540_b4-p5_gazebo-confirm-prereg.md`, P5b `2026-10-16_0120_b4-p5b_body-entry-recheck-prereg.md`, S12 `2026-10-08_1010_b4-s12_s12-pairs-prereg.md`, W `2026-10-08_1102_…`, P4 `2026-10-06_2246_…` |
| 실행 목록 | `notebook/runs/{b4-p3,b4-p4,b4-wire,b4-s12,b4-p5,b4-p5b}.jsonl` (`python3 scripts/notebook.py --nb docs/research/fleet-distributed/notebook check --paths`) |
| 원자료 | `/home/CAPYI_AI/runs/b24/<과제>/` — P5 `b4-p5/raw/`, P5b `b4-p5b/raw/` (+ `offline/` · `tests/` · `dry/`), S12 `b4-s12/raw/`, P4 `b4-p4/`, W `b4-wire/` |
| 분석 도구 | `tests/integration/fleet_p5/` (기록기 · 드라이버 · 분석 · P5b 띄우기), `docs/research/fleet-distributed/tools/` (`p5_review_check.py`, `p5b_analyze.py`, `p5b_replay.py`) |
| 코드 | `src/amr_fleet/amr_fleet/{agent,distributed,negotiation,fleetsim}/`, `traffic_zones.py` · `traffic_manager.py` (구역 진입 몸체 판정 `effed5b`), `src/amr_behavior` 존 노드 · 분산 트리 |
