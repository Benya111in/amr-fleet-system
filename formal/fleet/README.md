# formal/fleet — 구역 투표 프로토콜 모델 검사 (TLA+ / TLC)

보너스 4(분산 Fleet Management) P1 의 "모델 검사 층"이다. 근거 설계는 리서치 보고서
「분산 플릿 관리 협상 할당과 폴백」 74–122 행(구역 투표), 214–217 행(검증 세 층), 실행 계획 뒤의
주장 표다. 보고서는 저장소 밖(`/home/CAPYI_AI/research/reports/`)에 있다.

## 0. 먼저 읽을 것 — 이 결과가 말하지 **않는** 것

- **유한 범위 모델 검사다. 증명이 아니다.** 위반이 없다는 결과는 아래 표에 적은 범위
  (로봇 수·구역 수·요청 수·크래시 수) 안에서만 뜻이 있다. 범위를 넓히면 상태 수가 빠르게 늘어
  보고서가 제안한 범위(N = 3/5, 구역 ≤ 2, 요청 ≤ 3) 중 **N = 5 는 구역 1·요청 2 까지만** 돌렸다
  (§6 표).
- **활성(교착·기아·표 갈림 라이브락)은 검사하지 않았다.** `NoStrandedCommit`·`NoDeadZone` 은
  "어떤 후속 실행에서도 풀리지 않는 확정 표"라는 **특정한** 활성 결함을 안전 성질 꼴로 잡은 것뿐이다.
  공정성 가정 아래의 `<>` 성질은 다음 물결 일이다.
- 형식 범위는 **"구역 안(inside)은 과반 확정 뒤에만"** 이라는 논리 단계까지다. 제동 거리,
  위치추정, 점대칭 별칭, "언제 RELEASE 를 보내나"(진출 판정)는 모델 밖이다. 모델은 진입·진출을
  원자적이고 정확한 사건으로 둔다. 실제 차체의 동시 점유는 Gazebo 시험으로만 주장할 수 있다.
- 모델은 **보고서의 설계**를 옮긴 것이다. f1-vote 의 Python 구현(`vote_core.py`)을 옮긴 것이
  아니고, 구현 로그를 스펙에 대조하는 trace validation 도 하지 않았다.
- wall 시간은 공유 서버 부하(1 분 loadavg 5–100) 아래에서 잰 **예비값**이다. 상태 수는 부하와
  무관하다 (같은 cfg 를 다시 돌리면 같은 값이 나와야 한다 — 단 트레이스는 작업자 수에 따라 다를 수 있다).

## 1. 도구와 재현

| 항목 | 값 |
|---|---|
| TLC | `TLC2 Version 2.19 of 08 August 2024 (rev: 5a47802)` = tla2tools **v1.7.4** (2024-08-05, GitHub 최신 비프리릴리스) |
| jar | `/home/CAPYI_AI/downloads/tla/tla2tools-1.7.4.jar`, sha256 `936a262061c914694dfd669a543be24573c45d5aa0ff20a8b96b23d01e050e88` (저장소에 넣지 않는다, 출처·크기·sha256 은 `/home/CAPYI_AI/downloads/tla/PROVENANCE.txt`) |
| Java | OpenJDK 11.0.32.1 (이미지 `amr-ai/rl:0.3` 의 `/usr/bin/java`) |
| 실행 | `--network none --cpus 4`, `-workers 4 -deadlock -cleanup`, 힙 8 GB |

`-deadlock` 은 TLC 의 교착(다음 상태 없음) 검사를 **끈다**. 요청 수·크래시 수에 상한을 둔
유한 모델이라 모든 실행이 결국 멈추기 때문이다. 교착 성질은 이것으로 아무것도 검사하지 않는다.

```bash
# 한 설정 (로그: /home/CAPYI_AI/logs/agents/f2-tla/tlc/<cfg 이름>.log, 입력 사본: .../tlc/inputs/<cfg 이름>/)
formal/fleet/run_tlc.sh RA_LiveMembers cfg/RA_report_N5.cfg
formal/fleet/run_tlc.sh QuorumVote cfg/QV_N3_Z2_R3_C1_view.cfg
# 여러 설정을 차례로 (한 번에 하나)
formal/fleet/run_queue.sh QuorumVote:cfg/QV_N5_Z1_R2_C1_view.cfg TaskLock:cfg/TL_N3_T2_R3_C1.cfg
# 위반 트레이스 뽑기 (traces/<cfg 이름>.txt — 바뀐 변수만 보이는 요약 + TLC 원문)
formal/fleet/extract_trace.sh /home/CAPYI_AI/logs/agents/f2-tla/tlc/RA_report_N5.log
# 결과 표 (§6 은 이 출력을 붙인 것이다)
python3 formal/fleet/summarize.py /home/CAPYI_AI/logs/agents/f2-tla/tlc
```

환경 변수: `WORKERS`(≤ 4), `HEAP`, `TLC_TIMEOUT`(초), `TLA_JAR`, `OUT_DIR`, `IMAGE`. 로그 머리에
jar sha256, `git rev-parse HEAD`, 스펙·cfg sha256, 시작·끝 시각과 loadavg 를 남긴다.

## 2. 공통 가정 (세 모델)

| 가정 | 모델에서 |
|---|---|
| 비잔틴 고장 없음 | 메시지 위조·거짓 투표 없음. 모든 로봇이 규칙대로 움직인다 |
| 네트워크 | 보낸 메시지를 집합 `msgs` 에 넣고 **지우지 않는다** (tlaplus/Examples `Paxos.tla` 와 같은 방식). 수신은 비결정 선택이라 유실(끝내 안 받음)·중복(여러 번 받음)·재정렬·임의 지연이 모두 들어간다. 안전 성질(유한 접두사로 깨지는 성질)에 대해서는 이것으로 충분하다. **RA 모델만 예외**로 신뢰·비FIFO 채널이다 (§3) |
| 크래시-재시작 | 크래시와 재시작을 한 단계로 묶는다. 내려가 있는 동안은 아무 동작도 하지 않는 것과 같으므로 안전에 대해 일반성을 잃지 않는다. 영속 변수는 남고 휘발 변수는 초기화된다 |
| 시간 | 정수 시각이 없다. 잠정 표의 기한 만료는 **투표자별로 아무 때나 일어나는 비결정 사건**이다. 즉 어떤 시계 진행률·RTF·시계 역행에서도 안전한지를 본다 (기한 값 튜닝은 모델 밖) |
| 진입·진출 | 원자적이고 정확하다. `phase = "inside"` 가 곧 차체가 구역 안에 있다는 뜻 |
| 대칭 | 로봇·구역(작업)은 모델 값이고 순열 대칭(`SYMMETRY Symm`)으로 상태를 줄인다. 불변식은 모두 대칭이다 |

## 3. 모델 1 — `RA_LiveMembers.tla` (기각된 초안의 반례 재현)

보고서 74–80 행이 기각한 초안 — 앞선 노트 `decentralized_traffic.md` 권장안 A 의 진입 조건
"현재 생존 멤버 전원의 REPLY 를 받았고, 그 멤버가 5대 중 3대 이상" — 을 그대로 옮겼다.

- 요청 순서는 Ricart–Agrawala 원형: (번호, id) 사전식, 번호는 본 최댓값 + 1.
- REPLY 는 RA 규칙대로 — 안에 있거나, 나도 요청 중이고 내가 앞서면 미루고, 아니면 바로 보낸다.
- REPLY 에 답하는 요청 번호를 싣고 요청자는 **지금 요청의 답만** 센다. 초안보다 강하게 잡았다
  (그래도 깨지는지 보려고).
- **채널은 유실 없는 신뢰 채널**이다. 반례가 메시지 유실 없이 "생존 견해 불일치" 하나만으로
  나온다는 것을 보이려고 일부러 유실을 넣지 않았다.
- `view[r]`(로봇 r 이 믿는 생존 멤버)는 하트비트 오탐을 추상화해 아무 때나 바뀐다. `ViewModel` 로 범위를 고른다.

| cfg | 견해 범위 | 기대 | 결과 |
|---|---|---|---|
| `RA_report_N5` | A(1) 는 {A,B,C}, E(5) 는 {C,D,E} 로만 (보고서 장면 그대로) | 반례 | §6 |
| `RA_dropOne_N5` | A·E 가 각자 남 **하나**까지만 빠뜨림 (최소 불일치) | 반례 | §6 |
| `RA_any_N5` | r 포함, 크기 ≥ 3 인 아무 집합 | 반례 | §6 |
| `RA_fixed_N5` | 늘 전원 (= RA 원형). 위 셋과 범위가 같다 — **대조군** | 위반 없음 | §6 |
| `RA_fixed_N5_r3`, `RA_fixed_N3` | 늘 전원, 요청자를 넓힘 | 위반 없음 | §6 |

반례 셋은 모두 A(1)·E(5) 만 요청하고(다른 로봇은 응답만), 요청은 1 회씩이다.

**`RA_report_N5` 반례** (`traces/RA_report_N5.txt`, 15 상태): A 가 요청 → A 의 견해가 {1,2,3} 으로
바뀜 → B·C 가 REPLY → A 진입. E 가 요청 → C·D 가 REPLY (C 는 쉬고 있으므로 RA 규칙상 바로 답한다 —
REPLY 는 "나는 원하지 않는다" 일 뿐 "남이 안에 있지 않다" 가 아니다) → E 의 견해가 {3,4,5} 로 바뀜 → E 진입.
A·E 가 동시에 안에 있다. 보고서 본문의 장면과 같다.

**`RA_dropOne_N5` 반례** (19 상태)는 더 약한 조건을 보인다: E 가 **이미 구역 안에 있는 A 하나만**
오탐(견해 {2,3,4,5})하면 E 는 A 의 REPLY 를 기다리지 않고 들어간다. 견해 불일치는 한쪽만 생겨도 된다.

## 4. 모델 2 — `QuorumVote.tla` (정적 과반 배타 투표 + 2단계 진입)

보고서 82–98 행의 전이 표와 요청자 규칙, 100–102 행의 나머지 규칙을 옮겼다.
로봇 r 은 요청자이자 투표자다. 요청 id 는 `<<r, incarnation, seq>>`.

| 보고서 전이 | 모델 동작 | 비고 |
|---|---|---|
| FREE → TENTATIVE(r) : REQUEST(r,S), S 의 모든 표가 비어 있음 | `GrantVote(v)` | 사슬 S 전체에 원자적으로. "결정 규칙상 r 이 최우선" 은 넣지 않았다 — 비결정 선택이 **모든 순서**를 덮으므로 안전 결과는 어떤 결정 규칙에도 성립한다 |
| TENTATIVE → FREE : 기한 경과, ABANDON·RELEASE | `ExpireVote(v)`, `HandleAbandon(v)`, `HandleRelease(v)` | 기한은 아무 때나 |
| TENTATIVE(r) → COMMITTED(r) : ENTER(r) 때 여전히 TENTATIVE(r) | `CommitVote(v)` | ENTER 에 S 를 싣고 S 전체가 T(r) 일 때만 |
| COMMITTED → FREE : RELEASE 때만 | `HandleRelease(v)` | 시간·멤버십으로는 회수 없음 |
| 요청자: 과반 VOTE → ENTER | `SendEnter(r)` | |
| 과반 ENTER_ACK 뒤에만 진입 | `EnterZone(r)` | ACK 는 **지금 rid** 의 것만 센다 (자기 펜싱 (1)(2)) |
| 기한 안에 못 받으면 ABANDON | `GiveUp(r)` | 아무 때나 |
| 진출 → RELEASE | `Exit(r)` | 재전송은 메시지 집합이 대신한다 |
| 투표 상태 fsync, incarnation 영속·증가 | `CrashRestart(r)` | 영속: 표·incarnation·진입 사실·진행 중 rid. 휘발: seq. 진행 중이던 요청에는 재시작 후 ABANDON |

**불변식**: `MutualExclusion` — 모든 구역에서 안에 있는 로봇 ≤ 1 (보고서 214–217 행의 불변식).
`InsideBacked` — 안에 있는 로봇은 과반 투표자의 C(rid) 를 S 전체에 대해 쥐고 있다 (보고서의 정족수 교차
논증이 기대는 보조 성질).

**모델에 넣지 않은 것**: epoch(시간 리셋·모드 전환 번호). 보고서·노트의 epoch 규칙이 상태를 바꾸는
방법은 "TENTATIVE 를 버린다"(= `ExpireVote` 가 아무 때나 하는 일)와 "낮은 epoch 메시지를 무시한다"
(= 그 메시지를 받지 않는 것, 즉 유실)뿐이므로, epoch 를 넣은 프로토콜의 행동은 이 모델의 행동 집합 안에
있다고 **추론**한다 (TLC 로 확인한 것은 아니다). 우선순위 권고·INQUIRE/YIELD·백오프는 활성 장치라 넣지 않았다.

### 4.1 가드 변이 (Mutation) — 가드를 하나씩 되돌리면 반례가 나오는가

| Mutation | 되돌린 가드 | 기대 |
|---|---|---|
| `forget` | 재시작 때 표를 잊는다 (영속 없음) | 반례 |
| `expireCommitted` | 기한 만료가 C 표도 회수 (리스형 fail-open) | 반례 |
| `smallQuorum` | 정족수 N div 2 | 반례 |
| `onePhase` | 과반 VOTE(잠정 표)만으로 진입 | 반례 |
| `noRidCheck` | ENTER_ACK 를 rid 없이 로봇으로만 셈 (자기 펜싱 (2) 제거) | 반례 |
| `volatileInc` | 재시작해도 incarnation 을 안 올림 (rid 재사용) | 반례 |
| `overwriteTentative` | 투표자가 남의 **잠정** 표를 덮어씀 (C 가 아니면 부여) | 판정 대상 |

결과는 §6.

### 4.2 VIEW 축약

요청 id 는 로봇마다 단조 증가한다 (incarnation 영속·증가, seq 증가). 그래서 요청자의 지금 rid 가
아닌 VOTE/ACK 는 다시는 어떤 동작의 가드에도 쓰이지 않는다 (`SendEnter`·`EnterZone`·`EnterOnePhase` 는
`cur[r]` 의 것만 센다). `View` 는 그런 죽은 VOTE/ACK 만 빼고 상태를 비교한다. REQ/ENTER/ABANDON/RELEASE
는 투표자가 늦게 받을 수 있으므로 남긴다. 단조 전제가 깨지는 `noRidCheck`·`volatileInc` 변이에는
VIEW 를 쓰지 않는다. 같은 범위(`QV_N3_Z1_R2_C1`)에서 VIEW 없이·있이 둘 다 위반 없음이고 distinct 는
136,276 → 30,223 이다 (§6).

### 4.3 활성 결함 — ABANDON 이 확정 표를 남긴다

보고서 표대로라면 ABANDON 은 TENTATIVE 만 회수하고 COMMITTED 는 RELEASE 로만 풀린다. 그런데 요청자는
ENTER 를 보낸 뒤 과반 ACK 를 기한 안에 못 받으면 ABANDON 한다 — 그 사이 일부 투표자는 이미 확정했을 수
있다. 그 rid 의 RELEASE 는 영영 오지 않는다.

- `NoStrandedCommit`: "어떤 후속 실행에서도 풀리지 않는 C 표가 없다". 풀 수 있는 메시지는 RELEASE(rid)
  (그리고 변형에 따라 ABANDON(rid))뿐이고, 요청자의 rid 는 앞으로만 가므로, 요청자가 그 rid 로 진입 중이거나
  안에 있지 않고 투표자가 아직 처리하지 않은 해제 메시지도 없으면 그 표는 영원히 남는다. 해제 메시지를
  **한 번 처리한 뒤**에는 요청자가 재전송을 멈춘다고 본다 (모든 투표자의 ACK 를 받으면 멈추는 규칙).
- `NoDeadZone`: 그런 표가 N − 과반 개를 넘어 남은 투표자로 과반을 못 만드는 구역이 없다.

| Variant | ABANDON 처리 | 기대 |
|---|---|---|
| `report` | T 만 회수 (보고서 표) | 위반 |
| `fixA` | T·C 모두 회수 | 판정 대상 |
| `fixB` | fixA + 투표자가 이미 끝난(RELEASE/ABANDON 을 처리한) rid 의 REQUEST/ENTER 를 거부 (`done`, 영속) | 판정 대상 |

ABANDON 이 C 를 풀어도 안전한 이유: 요청자는 그 rid 로 진입하지 않았고(진입했으면 ABANDON 이 아니라
RELEASE 를 보낸다) 앞으로도 하지 않는다 (cur 이 앞으로만 간다). TLC 로는 `fixA`·`fixB` 에서도
`MutualExclusion` 을 함께 검사했다 (§6).

## 5. 모델 3 — `TaskLock.tla` (작업 ID 정족수 잠금, 중복 실행 0)

보고서 1 행 요약("실제 픽업을 시작할 실행권은 구역과 같은 투표 모듈로 작업 ID 를 잠가 확정한다"),
쓸 것 표의 "실행권" 행("작업 ID 정족수 잠금(잠정 → 픽업 시작 시 확정, 픽업 후 고장은 FAILED)"),
주장 표의 "손실이 있어도 중복 실행이 0이다(엄격 모드에서 설계상 참)"를 옮겼다.

- QuorumVote 의 2단계 투표를 구역 대신 **작업 id** 에 쓴다. 요청은 작업 하나 (사슬 없음).
- 진입 대신 **픽업 시작**(`Start`). 되돌릴 수 없고 `execCount[t]` 를 1 올린다 (역사 변수).
- 시작한 작업의 잠금은 풀지 않는다 — 완료든 크래시(FAILED)든 확정 표가 **묘비**로 남는다.
  시작 전에는 내려놓을 수 있다 (`Drop` → RELEASE: 협상이 다른 로봇에게 준 경우).
- 어느 로봇이 어느 작업을 잠그려 하는지는 비결정이다. 협상(CBAA)이 손실·지연으로 두 로봇에게 같은
  작업을 "낙찰" 시키는 경우, 늦게 도착한 옛 낙찰로 끝난 작업을 다시 청하는 경우를 모두 덮는다.
- ABANDON 은 시작 전 요청에만 오므로 C 도 푼다 (§4.3 의 fixA 와 같음).
- 불변식 `AtMostOnce`: 모든 작업에서 픽업 시작 횟수 ≤ 1 (같은 로봇이 두 번 시작하는 것도 센다).

| Mutation | 되돌린 가드 | 기대 |
|---|---|---|
| `forget` | 재시작 때 잠금 표를 잊는다 | 반례 |
| `expireCommitted` | 기한 만료가 확정 잠금도 회수 | 반례 |
| `releaseAfterDone` | 완료 후 잠금을 푼다 (묘비 없음) | 반례 |

`releaseAfterDone` 반례(`traces/TL_N3_T1_R2_C0_releaseAfterDone.txt`, 19 상태)는 r2 가 r1 의 작업 t1 을
**r1 이 끝내기 전에** 청해 두었다가, r1 이 완료하고 잠금을 풀자 그 옛 요청으로 과반을 모아 t1 을 다시
시작한다. 늦게 도착한 낙찰이 끝난 작업을 되살리는 장면이다. 그래서 **완료한 작업의 잠금은 풀면 안 된다.**
f3-cbaa 의 `negotiation/agent.py:263-268`(`_drop_assigned`)은 시작 전 `pending_lock` 만 release 한다
(코드 읽기, 2026-10-06 그 브랜치 HEAD `30d997d`) — 이 설계와 같다.

## 6. 결과

[측정] 모든 수치는 `summarize.py` 가 로그(`/home/CAPYI_AI/logs/agents/f2-tla/tlc/<cfg>.log`)에서 뽑은 것이다.
cfg 이름: `<모델>_N<로봇>_Z<구역>|T<작업>_R<전체 요청 상한>_C<전체 크래시 상한>[_<변이/변형>][_view]`.
"wall s" 는 공유 서버(32 코어, 시작 1 분 loadavg 열 참고) 위 4 작업자 기준 **예비값**이다. "깊이" 는 위반이면 반례까지의
BFS 깊이, 위반 없음이면 상태 그래프 전체의 깊이다. spec sha256 열의 스펙 판은 커밋으로 찾을 수 있다 (§6.7).

### 6.1 RA_LiveMembers (생존 멤버 RA 변형)

| cfg | 결과 | states generated | distinct | 깊이 | wall s | 시작 loadavg(1m) | spec sha256[:12] |
|---|---|---|---|---|---|---|---|
| RA_report_N5 | 위반 MutualExclusion (트레이스 15 상태) | 248021 | 40664 | 15 | 2 | 13.32 | e88070dac980 |
| RA_dropOne_N5 | 위반 MutualExclusion (트레이스 19 상태) | 4050753 | 341984 | 19 | 6 | 13.45 | e88070dac980 |
| RA_any_N5 | 위반 MutualExclusion (트레이스 15 상태) | 22787970 | 1271852 | 15 | 20 | 13.74 | e88070dac980 |
| RA_fixed_N5 | 위반 없음 (전수) | 53747 | 10784 | 23 | 1 | 5.52 | e88070dac980 |
| RA_fixed_N5_r3 | 위반 없음 (전수) | 9398163 | 1310604 | 34 | 14 | 5.15 | e88070dac980 |
| RA_fixed_N3 | 위반 없음 (전수) | 165184 | 51518 | 43 | 2 | 13.24 | e88070dac980 |

### 6.2 QuorumVote 설계 (Mutation none, Variant report) — 안전

| cfg | 결과 | states generated | distinct | 깊이 | wall s | 시작 loadavg(1m) | spec sha256[:12] |
|---|---|---|---|---|---|---|---|
| QV_N3_Z1_R2_C1 | 위반 없음 (전수) | 1514103 | 136276 | 32 | 6 | 27.61 | 1a25ba07b50a |
| QV_N3_Z1_R2_C1_view | 위반 없음 (전수) | 293094 | 30223 | 29 | 6 | 67.02 | 886be4a88678 |
| QV_N3_Z1_R2_C2_view | 위반 없음 (전수) | 930738 | 90981 | 30 | 8 | 36.85 | 1ab2620fb436 |
| QV_N3_Z2_R3_C1_view | 위반 없음 (전수) | 410010263 | 26007001 | 42 | 1768 | 62.45 | 886be4a88678 |
| QV_N5_Z1_R2_C1_view | 위반 없음 (전수) | 13523949 | 913764 | 41 | 402 | 35.28 | 1ab2620fb436 |
| QV_N5_Z2_R2_C1_view | 로그 없음 | | | | | | |

### 6.3 QuorumVote 가드 변이

| cfg | 결과 | states generated | distinct | 깊이 | wall s | 시작 loadavg(1m) | spec sha256[:12] |
|---|---|---|---|---|---|---|---|
| QV_N3_Z1_R2_C1_forget | 위반 MutualExclusion (트레이스 16 상태) | 537926 | 65813 | 17 | 3 | 23.74 | 1a25ba07b50a |
| QV_N5_Z1_R2_C1_forget_view | 위반 MutualExclusion (트레이스 20 상태) | 6699614 | 598727 | 21 | 206 | 44.06 | 1ab2620fb436 |
| QV_N3_Z1_R2_C0_expireCommitted | 위반 MutualExclusion (트레이스 16 상태) | 205479 | 29027 | 17 | 3 | 23.76 | 1a25ba07b50a |
| QV_N3_Z1_R2_C0_smallQuorum | 위반 MutualExclusion (트레이스 11 상태) | 21169 | 4080 | 11 | 2 | 24.34 | 1a25ba07b50a |
| QV_N3_Z1_R2_C0_onePhase | 위반 MutualExclusion (트레이스 10 상태) | 7437 | 1319 | 11 | 1 | 24.34 | 1a25ba07b50a |
| QV_N3_Z1_R3_C0_noRidCheck | 위반 MutualExclusion (트레이스 24 상태) | 11535546 | 1046002 | 24 | 38 | 54.75 | 1ab2620fb436 |
| QV_N3_Z1_R3_C1_volatileInc | 위반 MutualExclusion (트레이스 21 상태) | 20729069 | 1813987 | 22 | 63 | 42.48 | 1ab2620fb436 |
| QV_N3_Z2_R2_C0_overwriteTentative | 위반 없음 (전수) | 3524076 | 242646 | 34 | 16 | 38.07 | 1ab2620fb436 |
| QV_N3_Z2_R3_C1_overwriteTentative_view | 로그 없음 | | | | | | |

### 6.4 QuorumVote 활성 결함 (ABANDON 처리)

| cfg | 결과 | states generated | distinct | 깊이 | wall s | 시작 loadavg(1m) | spec sha256[:12] |
|---|---|---|---|---|---|---|---|
| QV_N3_Z1_R2_C1_strand_report_view | 위반 NoStrandedCommit (트레이스 7 상태) | 3973 | 1219 | 7 | 2 | 60.57 | 886be4a88678 |
| QV_N3_Z1_R2_C1_strand_fixA_view | 위반 NoStrandedCommit (트레이스 9 상태) | 35652 | 8002 | 9 | 2 | 60.57 | 886be4a88678 |
| QV_N3_Z1_R2_C1_strand_fixB_view | 위반 없음 (전수) | 796304 | 96598 | 25 | 11 | 62.29 | 886be4a88678 |
| QV_N3_Z1_R2_C1_dead_report_view | 위반 NoDeadZone (트레이스 8 상태) | 13643 | 3571 | 8 | 1 | 48.74 | 1ab2620fb436 |
| QV_N3_Z1_R2_C1_dead_fixA_view | 위반 NoDeadZone (트레이스 12 상태) | 266875 | 42896 | 12 | 5 | 48.74 | 1ab2620fb436 |
| QV_N3_Z1_R2_C1_dead_fixB_view | 위반 없음 (전수) | 796304 | 96598 | 25 | 10 | 47.72 | 1ab2620fb436 |
| QV_N3_Z2_R3_C1_fixB_view | 미완 (시간 상한/중단, rc=없음) | - | >=2,934,527 | >=13 |  | 72.40 | 1ab2620fb436 |

### 6.5 TaskLock

| cfg | 결과 | states generated | distinct | 깊이 | wall s | 시작 loadavg(1m) | spec sha256[:12] |
|---|---|---|---|---|---|---|---|
| TL_N3_T1_R2_C1 | 위반 없음 (전수) | 2527897 | 364626 | 32 | 18 | 90.21 | 290b7fafd0fa |
| TL_N3_T1_R2_C1_view | 위반 없음 (전수) | 192652 | 28400 | 26 | 3 | 41.49 | 7ec69a1f8222 |
| TL_N5_T1_R2_C1_view | 위반 없음 (전수) | 5383998 | 528047 | 37 | 114 | 39.61 | 7ec69a1f8222 |
| TL_N3_T2_R3_C1_view | 미완 (시간 상한/중단, rc=없음) | - | >=4,213,891 | >=20 |  | 38.75 | 7ec69a1f8222 |
| TL_N3_T1_R2_C1_forget | 위반 AtMostOnce (트레이스 18 상태) | 930304 | 170069 | 18 | 9 | 100.71 | 290b7fafd0fa |
| TL_N3_T1_R2_C0_expireCommitted | 위반 AtMostOnce (트레이스 18 상태) | 317816 | 58407 | 18 | 4 | 92.57 | 290b7fafd0fa |
| TL_N3_T1_R2_C0_releaseAfterDone | 위반 AtMostOnce (트레이스 19 상태) | 210067 | 40247 | 20 | 4 | 86.92 | 290b7fafd0fa |

### 6.6 중단한 실행 (결과로 쓰지 않는다)

| cfg | 결과 | states generated | distinct | 깊이 | wall s | 시작 loadavg(1m) | spec sha256[:12] |
|---|---|---|---|---|---|---|---|
| QV_N3_Z2_R3_C1.noview-killed | 미완 (시간 상한/중단, rc=143) | - | >=23,599,723 | >=19 | 953 | 23.74 | 1a25ba07b50a |
| QV_N3_Z2_R3_C0_noRidCheck.outer-timeout-killed | 미완 (시간 상한/중단, rc=없음) | - | >=11,787,877 | >=21 |  | 26.72 | 1a25ba07b50a |
| QV_N3_Z2_R3_C1_volatileInc.outer-timeout-killed | 미완 (시간 상한/중단, rc=없음) | - | >=14,264,863 | >=18 |  | 68.00 | 1a25ba07b50a |
| TL_N3_T2_R3_C1.noview-killed | 미완 (시간 상한/중단, rc=143) | - | >=27,262,191 | >=19 | 638 | 50.76 | 290b7fafd0fa |
| TL_N5_T1_R2_C1.noview-killed | 미완 (시간 상한/중단, rc=143) | - | >=15,725 | >=11 | 12 | 43.68 | 7ec69a1f8222 |

### 6.7 스펙 판과 실행 메모

| spec sha256[:12] | 파일 @ 커밋 | 앞 판과의 차이 |
|---|---|---|
| e88070dac980 | RA_LiveMembers.tla @ ba96132 | — |
| 1a25ba07b50a | QuorumVote.tla @ b9a534b | — |
| 886be4a88678 | QuorumVote.tla @ 68fcc1d | `LiveMsg`·`View` 정의 추가만 (Init/Next 동일) |
| 1ab2620fb436 | QuorumVote.tla @ e65d6b5 | `NoDeadZone` 정의 추가만 (Init/Next 동일) |
| 290b7fafd0fa | TaskLock.tla @ 68fcc1d | — |
| 7ec69a1f8222 | TaskLock.tla @ ab7cdaf | `LiveMsg`·`View` 정의 추가만 (Init/Next 동일) |

판 사이의 차이는 `git diff` 로 확인했고 지운 줄이 없다 (추가만). 그래서 같은 cfg 의 상태 수는 판과 무관하다.

- `QV_N3_Z2_R3_C1_view` 로그 끝의 `rc=127` 은 TLC 가 아니라 **실행 스크립트** 쪽이다: 실행 중에 `run_tlc.sh` 를 고쳐서
  bash 가 바뀐 파일의 엉뚱한 줄(`h: command not found`)을 읽었다. TLC 자신은 "Model checking completed. No error has been
  found." 와 최종 통계를 찍고 끝났다 (29 분 26 초). 이 실행은 입력 사본(`inputs/`) 기능이 들어가기 전이라 사본이 없고,
  스펙은 sha 로 68fcc1d 의 파일과 같다.
- 같은 실행의 지문 충돌 확률 추정: optimistic 5.4E-4, actual fingerprints 1.6E-5 (TLC 출력). 다른 실행의 값은 각 로그에 있다.
- VIEW 없이 돌린 `QV_N3_Z1_R2_C1`(136,276) 과 VIEW 판(30,223)은 둘 다 위반 없음이다. 깊이가 32 → 29 로 줄어드는 것은
  VIEW 가 같다고 본 상태를 합치기 때문이다.

## 7. 이 결과로 쓸 수 있는 문장 · 쓸 수 없는 문장

보고서 주장 표(실행 계획 뒤)의 행에 맞춰 적는다. 수치는 §6 표의 [측정] 값이다.

### 쓸 수 있는 문장

| 보고서 주장 | 이 디렉터리로 보탤 수 있는 문장 |
|---|---|
| "서버가 죽어도 한 구역에 두 대가 동시에 '허가'되지 않는다" (조건부 정리) | "보고서의 2단계 투표 설계(정적 과반, 영속 투표, 요청 id·incarnation 으로 묶은 ENTER_ACK)를 TLA+ 로 모델링해 TLC 로 전수 검사했다. 로봇 3대·구역 2·요청 3·크래시-재시작 1, 메시지 유실·중복·재정렬·임의 지연, 투표자별 임의 시점 기한 만료 범위에서 MutualExclusion 위반 없음 (distinct 26,007,001, 깊이 42). 로봇 5대는 구역 1·요청 2·크래시 1 범위에서 위반 없음 (distinct 913,764)." — **유한 범위 모델 검사**이고 증명이 아니다. 정족수 교차 논증(수작업)과 함께 쓴다 |
| (같은 행, 모델이 공허하지 않다는 근거) | "설계의 가드 6 개 — 투표 영속, 확정 표 비회수, 과반 정족수, 2단계, ACK 의 rid 일치, incarnation 증가 — 를 하나씩 되돌리면 TLC 가 각각 두 대 동시 진입 반례를 낸다 (10–24 상태)." |
| 앞선 노트의 "생존 멤버 RA" 기각 | "생존 멤버 전원 REPLY + 멤버 ≥ 3 규칙은 유실 없는 채널에서도 생존 견해가 어긋나면 두 대를 들여보낸다. 보고서의 장면({A,B,C} 대 {C,D,E})을 TLC 가 15 상태 반례로 재현했고, 구역 안의 로봇 하나를 한 대가 오탐하는 것만으로도 반례가 나온다 (19 상태). 견해가 늘 전원이면(RA 원형) 같은 범위에서 위반 없음." |
| "손실이 있어도 중복 실행이 0이다" (엄격 모드에서 설계상 참) | "작업 id 정족수 잠금 모델에서 로봇 3대·작업 1·요청 2·크래시 1 (및 로봇 5대·작업 1·요청 2·크래시 1) 범위, 같은 고장 모델로 픽업 시작이 작업당 2 회 이상인 실행 없음." (작업 2·요청 3 은 §6 표 참고) |
| "split-brain 이 없다" — 구역·실행권 부분 | 위 두 문장이 그 범위다. 서버 명령의 epoch 펜싱은 **모델에 없다** |

### 쓸 수 없는 문장

- "증명했다", "모든 N·모든 요청 수에서 안전하다" — 유한 범위다. 기계 증명(TLAPS)이나 귀납 불변식 검사(Apalache)는 하지 않았다.
- "교착이 없다", "언젠가 들어간다", "표 갈림이 풀린다" — 활성은 검사하지 않았다. 오히려 §4.3 처럼 보고서 표 그대로는 **구역이 영구히 막히는** 실행이 있다.
- "구현(vote_core.py)이 옳다" — 모델은 설계를 옮긴 것이다. 구현과의 대응은 Hypothesis 상태기계 시험(f1-vote)과 로그 재생이 맡는다.
- "실제 차체가 한 구역에 동시에 있지 않다" — 제동·위치추정·별칭·진출 판정은 모델 밖이다. Gazebo 실제 자세 계수로만 주장한다.
- "epoch 펜싱을 TLC 로 검증했다" — epoch 는 넣지 않았다 (§4 의 추론만 있다).
- "보고서가 제안한 범위(N = 5, 구역 2, 요청 3)를 전수 검사했다" — N = 5 는 §6 표에 적은 범위까지만이다.

## 8. 보고서에 대한 발견 (설계 쪽으로 넘길 것)

1. **ABANDON 이 확정 표를 남긴다 (활성 결함, 안전 무관).** 보고서 전이 표는 ABANDON 이 TENTATIVE 만 회수하게 한다.
   ENTER 뒤 ACK 를 기한 안에 못 받아 포기하면 이미 확정한 투표자의 C 표는 영영 풀리지 않는다. N = 3 에서 8 상태 만에
   과반 형성이 불가능해져 그 구역은 **영구히** 막힌다 (`traces/QV_N3_Z1_R2_C1_dead_report_view.txt`).
   ABANDON 이 C 도 풀게 해도(fixA) 투표자가 이미 처리한 요청의 늦은 REQUEST/ENTER 를 다시 받아들이면 같은 일이 생긴다
   (12 상태). **끝난 요청 id 를 거부**하면(fixB) 같은 범위에서 사라진다. f1-vote 의 `vote_core.py` 는 이미
   "확정 표는 RELEASE/ABANDON 으로 회수" 와 tombstone 거부(REQUEST·ENTER 에 `R_FINISHED` NACK)를 한다
   (코드 읽기, 그 브랜치 HEAD `3eedf5c` 의 `_on_request`·`_on_enter`·`_on_release`) — fixB 와 같은 방향이다.
   tombstone 은 최근 256 개까지만 남지만(`tombstone_cap`), 같은 코드의 REQUEST 기한 검사(`deadline <= now` 면 NACK)와
   "같은 로봇의 더 새 시도가 있으면 무시" 가 밀려난 옛 요청의 재부여를 막는 것으로 읽힌다. 기한 검사는 sim 시각 비교라
   활성에만 영향을 준다 (추론, 시험하지 않음).
2. **잠정 표끼리의 배타성은 안전에 필요하지 않다 (좁은 범위의 관찰).** 투표자가 남의 잠정 표를 덮어써도
   (`overwriteTentative`, 확정 표는 덮어쓰지 않음) 위반이 없다 (범위는 §6 표 — 설계 검사보다 좁을 수 있다).
   보고서 95–96 행의 안전 논증도 실제로는 "확정 표 C(r_A) 를 쥔 투표자는 r_B 에 표를 줄 수 없다" 만 쓰고, 잠정 표끼리의
   배타성은 논증에 들어가지 않는다. 잠정 표 배타성은 같은 구역의 확정이 갈려 아무도 과반을 못 얻는 일을 줄이는
   **활성 장치**로 보는 것이 맞다. 구현에서 이 가드를 느슨하게 하라는 뜻은 아니다.
3. **완료한 작업의 잠금은 풀면 안 된다.** `releaseAfterDone` 반례 (§5).
4. **incarnation 영속·증가와 ACK 의 rid 일치는 둘 다 필요하다.** 하나만 빠져도 옛 VOTE/ACK 로 확정 없이 진입한다
   (`volatileInc` 21 상태, `noRidCheck` 24 상태).

## 9. 다음 단계

- 활성: 요청 수를 묶지 않은 모델(seq 를 유한 순환 또는 추상화)에서 공정성 아래 "요청하면 언젠가 들어가거나 포기한다",
  표 갈림 라이브락의 존재 확인. 지금은 요청 수 상한 때문에 모든 실행이 유한해 `<>` 성질이 뜻이 없다.
- 로봇당 rid 여럿: 지금 모델은 로봇마다 진행 중 요청 하나(사슬은 S 로)다. f1-vote 처럼 "보유 여러 개 + 시도 1 개"
  (구역을 쥔 채 다음 사슬을 청하는 hold-and-wait)를 넣으면 VIEW 의 단조 전제도 다시 따져야 한다.
- epoch·서버 권고 순서를 넣은 모델, 그리고 f1-vote 의 이벤트 로그를 이 스펙의 행동으로 재생하는 경량 trace 검사.
- N = 5 범위 넓히기 (구역 2·요청 3): 상태 수가 커서 이번에는 못 했다.
