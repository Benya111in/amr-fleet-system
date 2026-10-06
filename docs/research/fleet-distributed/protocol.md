# 구역 투표 프로토콜 명세 (보너스 4 P1 — 투표 코어)

- 상태: P1 (순수 Python 코어 + 시험). ROS 통합(P3)은 다음 물결.
- 코드: `src/amr_fleet/amr_fleet/distributed/` (rclpy 비의존)
- 근거: 「분산 플릿 관리 협상 할당과 폴백」 — 사본 `report-2026-10-06.md`(머리 8 줄이 출처 주석이라
  원본 행 번호 + 8). 원본 기준 74–134행(구역 투표·교착), 135–158행(장애 감지·모드), 218–276행(P1).
- 명세 원문: `31_*.md` 5장 4번 "분산 Fleet Management" — "로봇 간 직접 통신을 통한 협상 기반 작업 할당",
  "중앙 서버 장애 시에도 로봇들이 자율적으로 동작하는 폴백 로직". 이 문서는 그중 **구역 안전(상호배제)**
  부분의 프로토콜만 정한다. 협상 할당(CBAA)은 f3 의 `negotiation.md`, 모델 검사는 f2 의 `formal/fleet/`.

## 1. 한 문장 요약

용량 1 자원(교차로·1차선 통로·교착 해소 포켓·작업 ID)마다 플릿 5대 **전원이 투표자**이고, 요청자는
**정적 과반 3/5** 의 배타 표를 모은 뒤(1단계, 잠정) 다시 **과반의 확정 응답(ENTER_ACK)** 을 받은 뒤에만
진입선을 넘는다(2단계). 잠정 표만 시간으로 회수하고, 확정 표는 보유자의 RELEASE 로만 풀린다.

## 2. 안전 논증 (수작업 증명 스케치 — 기계 검증 아님)

불변식 S: 어느 자원 z 에 대해서도 "z 를 허가받은(authorized) 로봇" 은 동시에 1 대 이하.

1. 요청자 A 가 z 에 대해 허가 상태이면, 과반 M_A(|M_A| ≥ 3) 의 투표자가 z 에 대해 `COMMITTED(r_A)` 를
   **디스크에** 갖고 있다 (투표자는 영속화한 뒤에 ENTER_ACK 를 보내고, 요청자는 과반 ACK 를 받은 뒤에만
   허가 상태가 된다).
2. `COMMITTED(r_A)` 는 r_A 의 RELEASE/ABANDON 으로만 풀린다. 요청자는 허가를 내려놓은 **뒤에** RELEASE 를
   보낸다. 시간·하트비트·멤버십·epoch 변경은 `COMMITTED` 를 풀지 않는다.
3. B 가 허가 상태가 되려면 과반 M_B 의 투표자가 z 에 대해 먼저 `TENTATIVE(r_B)` 를 거쳐 `COMMITTED(r_B)`
   가 되어야 한다. 투표자 한 명은 z 에 대해 한 번에 한 요청에만 표를 준다 (배타).
4. |M_A| + |M_B| ≥ 6 > 5 이므로 공통 투표자 w 가 있다. w 는 `COMMITTED(r_A)` 를 쥔 동안 r_B 에 표를 줄 수
   없다. 모순.

시간은 3 번에서 "w 가 r_B 의 잠정 표를 **언제** 회수하나" 에만 나온다. 그래서 시계가 느리든 빠르든,
투표자마다 다르든 안전은 그대로이고 활성만 달라진다. 재시작한 투표자는 디스크에서 표를 되살리므로
1 번의 "디스크에" 가 핵심이다 (영속화 누락 = 이중 투표).

이 논증이 기대는 가정: 비잔틴 없음(거짓 메시지 없음), 투표 상태 영속화(쓰기 후 fsync, 응답 전), 정적 멤버
5 대, 요청 ID 전역 유일. 물리 계층(제동·위치추정·점대칭 별칭)은 범위 밖이다 — "허가 없이 진입선을 넘지
않는다" 는 로봇 쪽 게이트(P3, `ZoneGuard`)와 Gazebo 시험으로만 주장한다.

## 3. 식별자

| 이름 | 정의 | 비고 |
|---|---|---|
| `robot_id` | 정적 멤버 문자열 (`amr_01` …) | 멤버 집합은 설정으로 고정. 동적 멤버십 없음 |
| `incarnation` | 로봇별 재시작 번호. 기동마다 +1, **첫 송신 전에** 디스크에 기록 | 재시작 전 요청의 늦은 응답이 재시작 후 요청과 섞이지 않게 |
| `ReqId` | `(robot_id, incarnation, seq)` | seq 는 같은 incarnation 안 단조 증가(휘발). 재시도마다 새 ReqId |
| 자원 ID | 문자열. 접두사로 종류를 가른다: `zone:<id>`, `pocket:<id>`, `task:<task_id>` | 투표 코어는 종류를 구별하지 않는다 ("자원 ID → 배타 표") |
| 사슬 S | 한 요청이 묶어 받는 자원 집합 (`frozenset`) | 교차로→통로처럼 붙은 구역 + 필요하면 포켓 |
| `epoch` | 플릿 전역 단조 정수. 로봇마다 `max_epoch_seen` 을 영속화 | 시간 리셋·모드 전환 번호 (6 절) |
| `Stamp` | `(sec, nanosec)` sim 시각. 비교는 **항상 (sec, nanosec) 전체** | Open-RMF #551(nanosec 만 비교) 교훈. `ordering.py` 단위 시험 필수 |
| `lamport` | 로봇별 Lamport 시계 | 대기 순서(오래 기다린 순)에 쓴다. 안전과 무관 |

## 4. 메시지 (`VoteMsg`)

모든 메시지는 같은 필드를 갖는다 (안 쓰는 필드는 기본값).

| 필드 | 형 | 뜻 |
|---|---|---|
| `kind` | 아래 8 종 | |
| `src`, `dst` | robot_id | 1:1 전송 (방송은 송신 측이 5 개로 쪼갠다 — 자기 자신 포함) |
| `epoch` | int | 송신자의 `max_epoch_seen` |
| `req` | ReqId | 이 메시지가 가리키는 요청 |
| `resources` | frozenset[str] | REQUEST/ENTER: 사슬 S 전체. RELEASE: 이번에 푸는 부분집합. 응답: 질의한 집합 그대로 |
| `key` | PriorityKey 또는 None | REQUEST 에만. 투표자의 대기열 순서 (7 절) |
| `deadline` | Stamp 또는 None | REQUEST 에만. 잠정 표의 기한 (요청자의 sim 시각 기준 절대값) |
| `lamport` | int | 송신 시 Lamport 시각 |
| `reason` | str | NACK 사유: `busy`(확정 표가 막음) `contended`(다른 요청의 잠정 표가 막음) `expired` `finished` `stale_epoch` `not_tentative` `amnesic` |

| kind | 방향 | 보내는 때 |
|---|---|---|
| `REQUEST` | 요청자 → 투표자 전원 | 새 시도 시작, 그리고 `retry_s` 마다 아직 답 없는 투표자에게 재전송 |
| `VOTE` | 투표자 → 요청자 | 잠정 표를 **영속화한 뒤** |
| `NACK` | 투표자 → 요청자 | 지금은 표를 못 줌(사유 포함). `busy` 는 대기열에 넣었다는 뜻 — 나중에 VOTE 가 올 수 있다 |
| `ENTER` | 요청자 → VOTE 를 준 투표자 | 과반 VOTE 를 모은 뒤 (그리고 ENTERING 상태를 영속화한 뒤) |
| `ENTER_ACK` | 투표자 → 요청자 | `TENTATIVE(r)` → `COMMITTED(r)` 를 **영속화한 뒤** |
| `ABANDON` | 요청자 → 투표자 전원 | 기한 안에 과반 ACK 를 못 받음 / 과반이 불가능해짐 / 취소 |
| `RELEASE` | 요청자 → 투표자 전원 | 허가를 내려놓은 뒤 (진출 확인, 경로 변경, 재시작 후 정리). 부분집합 가능 |
| `RELEASE_ACK` | 투표자 → 요청자 | RELEASE/ABANDON 처리 뒤. 요청자는 전원 ACK 까지 재전송 |

ABANDON 과 RELEASE 는 투표자에게 같은 의미다: "요청 r 의 표(잠정이든 확정이든)를 푼다". 보유자 자신만
보낼 수 있는 메시지이므로 확정 표를 풀어도 안전하다. 구분은 로그·지표용이다.

## 5. 상태와 전이

### 5.1 투표자 (자원 z 마다 표 1 장)

| 현재 | 사건 | 다음 | 동작 |
|---|---|---|---|
| FREE | REQUEST(r, S): S 의 모든 자원이 FREE, r 보다 앞선 기아 요청의 예약 없음, 기한 미경과 | S 전체 `TENTATIVE(r, deadline)` | 영속화 → VOTE |
| FREE/점유 | REQUEST(r, S): S 중 하나라도 다른 요청이 쥠 | 그대로, r 을 대기열에 | 막는 표가 모두 확정이면 NACK(busy), 아니면 NACK(contended) |
| `TENTATIVE(r)` | 자기 sim 시각 ≥ deadline | FREE | 대기열에서 다음 요청에 투표 시도 |
| `TENTATIVE(r)` | 같은 로봇의 더 새 요청 r' > r 의 REQUEST | FREE, r 묘비 | 요청자는 활성 시도가 하나뿐이므로 r 은 끝났다 — 이른 만료와 같아 안전 |
| `TENTATIVE(r)` | ENTER(r, S) 이고 S 전체가 r 에 묶여 있음 | `COMMITTED(r)` | 영속화 → ENTER_ACK |
| `COMMITTED(r)` | ENTER(r, S) 중복 | 그대로 | ENTER_ACK 재전송 (멱등) |
| 그 밖 | ENTER(r, S) | 그대로 | NACK(not_tentative) |
| `TENTATIVE(r)`/`COMMITTED(r)` | RELEASE/ABANDON(r, R) | R 의 표 FREE, r 묘비(tombstone) | 영속화 → RELEASE_ACK → 대기열 재평가 |
| 아무 상태 | 더 높은 epoch 의 메시지 | `TENTATIVE` 전부 FREE, `COMMITTED` 유지 | `max_epoch_seen` 갱신·영속화, 메시지 처리 계속 |
| 아무 상태 | 더 낮은 epoch 의 메시지 | 그대로 (**수용 0**) | NACK(stale_epoch, 자기 epoch 실음) |

- **배타**: 투표자의 표는 자원마다 하나이고 하나의 ReqId 에만 묶인다.
- **원자 사슬**: S 의 일부만 비어 있으면 아무 것도 주지 않는다 (전부-아니면-전무). 해제는 자원별로 할 수 있다.
- **fail-closed**: `COMMITTED` 는 시간·하트비트·멤버십·epoch 로 절대 회수하지 않는다. 구역 안에서 죽은 로봇의
  표는 남는다 (운영자 해제 절차 — P3).
- **묘비**: RELEASE/ABANDON 을 받은 ReqId 는 묘비 목록(상한 `tombstone_cap`, FIFO)에 넣고, 그 ReqId 의
  늦게 온 REQUEST/ENTER 는 NACK(finished). 중복·재정렬된 옛 REQUEST 가 좀비 표를 만드는 것을 막는다
  (활성 보호. 안전은 묘비 없이도 성립 — 변이 검사로 확인).
- **기억 상실**: 영속 파일이 손상돼 읽을 수 없으면 그 투표자는 `amnesic` 이 되어 **어떤 표도 주지 않는다**
  (NACK(amnesic)). 5 대 중 1 대가 빠져도 과반 3 은 남는다. 복구는 운영자 몫.

### 5.2 요청자 (로봇 1 대에 활성 시도 1 개 + 보유 여러 개)

| 단계 | 들어가는 때 | 나가는 때 |
|---|---|---|
| IDLE | — | `request(S)` → VOTING |
| VOTING(r) | 새 ReqId 로 REQUEST 방송 | 과반 VOTE → ENTERING 영속화 → ENTER 송신 / 기한 경과, 또는 NACK(contended) > N−Q (표 갈림), 또는 NACK(expired·finished·amnesic) > N−Q → ABANDON + 백오프. NACK(busy) 는 기다린다 (점유가 풀리면 대기열에서 VOTE 가 온다) |
| ENTERING(r) | 과반 VOTE | 과반 ENTER_ACK(같은 req·같은 epoch) → **HELD 영속화** / 기한·과반 불가·더 높은 epoch → ABANDON + 백오프 |
| HELD(r, S) | 과반 ENTER_ACK | `release(R)` — 진출 확인한 자원만 내려놓고 RELEASE |
| BACKOFF | ABANDON 직후 | 무작위 `U(backoff_min, backoff_max) × 2^min(시도-1, 3)` 뒤 새 ReqId 로 재시도 |
| RELEASING(r, R) | RELEASE/ABANDON 송신 | 투표자 5 명 전원이 RELEASE_ACK |

- **진입 게이트(자기 펜싱)**: `may_enter(z)` 는 z 가 HELD 에 있을 때만 참. HELD 는 "현재 ReqId 에 대해, 현재
  epoch 로 온 ENTER_ACK 의 투표자 집합 크기 ≥ 3" 일 때만 만들어진다. 다른 ReqId·낮은 epoch 의 ACK 는 버린다.
- **요청 없는 응답 정리**: 활성도 보유도 아닌 ReqId 에 대한 VOTE/ENTER_ACK 가 오면 그 투표자에게 RELEASE 를
  보낸다 (좀비 표 청소).
- **재시작**: incarnation+1 을 먼저 영속화한다. 디스크의 HELD 중 물리적으로 안에 있지 않은 자원, 그리고
  ENTERING 이던 시도는 RELEASE 한다. 물리적으로 안에 있는 자원은 HELD 로 유지한다 (판단 근거는 로봇의
  위치추정 + 오도메트리 이중 조건 — P3).
- **기아 방지**: 같은 논리 요청의 재시도는 처음 시도의 Lamport 시각(`since`)과 시도 횟수를 이어 간다.
  시도 횟수가 `max_bypass` 를 넘으면 `starved` 표시를 단다 (7 절 순서의 첫 기준).

## 6. 시간·epoch 규칙

- 기한·재전송·백오프·하트비트는 모두 **sim-time**(`/clock`, `use_sim_time`)으로 잰다. wall-clock 은 쓰지 않는다.
  sim-time 이 0(미초기화)이면 요청하지 않는다 (P3 노드 책임).
- 기한(`tentative_s`, 기본 sim 15 s)은 활성 파라미터다. 너무 짧으면 확정 전에 회수돼 재요청이 반복되고, 너무
  길면 죽은 요청자의 잠정 표가 그만큼 묶인다. 안전과 무관하다.
- 시간 역행(시뮬레이션 리셋)을 보면 그 로봇은 `epoch = max_epoch_seen + 1` 로 올려 영속화하고, 자기 투표자의
  `TENTATIVE` 를 모두 풀고(`COMMITTED` 유지), 활성 시도를 ABANDON 한다.
- 더 높은 epoch 의 메시지를 받으면 같은 처리(채택)를 한 뒤 메시지를 처리한다.
- 더 낮은 epoch 의 메시지는 **어떤 상태도 바꾸지 않는다** (RELEASE 포함). NACK(stale_epoch)에 자기 epoch 를
  실어 보내 송신자가 따라오게 한다. RELEASE 를 거절해도 안전하다 (fail-closed) — 송신자는 epoch 를 채택하고
  재전송한다.
- 서버 명령(순서 권고 등, P3)도 같은 `EpochFence` 로 거른다: `(epoch, leader_id)` 가 본 최댓값보다 작으면 거부.

## 7. 순서 규칙 (투표자의 대기열)

표가 풀리면 투표자는 대기열에서 다음 키 순으로 첫 요청에 표를 준다 (`traffic_zones.py` 의 중앙 규칙을
메시지에 실은 키로 옮긴 것 — 모든 투표자가 같은 키로 정렬하므로 표 갈림이 줄어든다).

1. `starved` 먼저 — 기아끼리는 `since`(Lamport, 오래된 것 먼저)
2. 작업 우선순위 높은 순
3. 마감 `Stamp` 이른 순 (없으면 가장 늦음)
4. `since` 작은 순 (오래 기다린 순 — 중앙 규칙의 "대기 시간" 을 Lamport 시각으로 대체)
5. `robot_id`, `incarnation`, `seq`

기아 요청이 대기열에서 막혀 있으면 그 자원은 그 요청에 **예약**되어 더 나쁜 키의 요청에 표를 주지 않는다.
중앙 서버가 살아 있을 때의 순서 권고는 P3 에서 이 키 앞에 붙인다 — 안전 논증이 "임의의 순서" 에 대해
성립하므로 누가 순서를 정하든 안전은 그대로다.

## 8. 장애 감지 (`failure_detector.py`) — 안전에 쓰지 않는다

- 서버 하트비트를 sim-time 으로 잰다: 마지막 수신 뒤 `suspect_s`(1.5 s, 2 Hz 3 회 누락) → SUSPECT,
  `dead_s`(5 s) → DEAD.
- 자기 지연 점검(Lifeguard 식): 자기 감시 주기가 늦게 돌면(간격 > 기대 × 2) 지역 건강 점수를 올리고,
  타임아웃에 `(1 + 점수)` 를 곱한다. 자기 정지 직후의 첫 판정은 "자기 탓" 으로 보고 DEAD 를 내지 않는다.
- 과반 의심: 플릿 DEGRADED 는 비컨에 실린 "서버 DEAD" 표시가 **신선한 비컨 기준 3/5 이상**일 때만.
- 감지는 틀릴 수 있다 (FLP). 오탐은 할당 방식이 바뀌는 효율 문제일 뿐 구역 안전과 무관하다.

## 9. 교착 (`wfg.py`) — 복제 대기 그래프

- 각 로봇이 자기 out-edge(`token`·`crossing`·`block`, 시각)를 비컨에 싣고, 모두가 같은 그래프에 같은 결정적
  Tarjan(`deadlock.py` 재사용)을 돌린다.
- `token`·`crossing` 은 AND 대기 → 크기 ≥ 2 SCC 면 교착 후보. `block` 이 낀 SCC 는 Nav2 가 우회할 수 있는
  OR 대기라 **knot**(SCC 밖으로 나가는 간선 없음)일 때만 후보.
- 확정: 후보 구성원 전원의 경로상 진행량이 `confirm_s`(1 s) 동안 ε 미만.
- 희생자: (우선순위 낮은 순, 같으면 **처음 요청 시각이 늦은 순**, 같으면 robot_id 큰 순)의 첫 로봇. 고정 시각이라
  같은 쌍에서 방향이 뒤집히지 않는다 (wait-die). 같은 입력이면 모든 복제가 같은 희생자를 고른다.
- "교착 없음" 은 주장하지 않는다. 사슬 내부 순환 대기는 원자 부여로 없애고, 나머지는 탐지·해소한다.

## 10. 보고서와 다른 점 (구현 결정)

| 보고서 | 구현 | 이유 |
|---|---|---|
| 투표자 전이 표: TENTATIVE → FREE 는 "기한 경과, 또는 ABANDON·RELEASE" | 같음 + **더 높은 epoch 채택 시** TENTATIVE 회수 | 보고서 원본 143 행(시간 리셋 → TENTATIVE 무효화, COMMITTED 유지)을 전이 표에 합쳤다 |
| ABANDON 은 잠정 표 회수 | ABANDON = RELEASE (확정 표도 푼다) | ENTERING 중 기한이 지나면 일부 투표자는 이미 COMMITTED 일 수 있다. 보유자만 보내므로 안전 |
| (없음) | 묘비, 요청 없는 응답 → RELEASE | 중복·재정렬된 옛 REQUEST+ENTER 가 영구 좀비 COMMITTED 를 만드는 활성 결함을 막는다 |
| (없음) | 기억 상실 투표자는 영구 기권 | 손상된 영속 파일 = 표를 잊음. 다시 투표하면 이중 투표 |
| 대기 순서의 "대기 시간" | Lamport `since` | 투표자마다 다른 벽시계·sim 시각을 쓰지 않고 메시지에 실은 값으로 같은 순서를 얻는다 |
| INQUIRE/YIELD (Maekawa) | 넣지 않음. 기한 + 무작위 지수 백오프 + 과반 불가 조기 포기 | P1 범위. 표 갈림 빈도는 P4 빠른 계층에서 잰다 |
| 확정 표는 RELEASE 로만 | 같음. 단 **요청자의 epoch 가 바뀌면 ENTERING 시도를 포기** | 낮은 epoch ACK 를 거부하므로 그 시도는 더는 과반을 못 모은다 |

### 10.1 검증으로 드러난 것 (verification.md)

- **안전을 지는 것은 확정 표의 배타다.** 잠정 표 배타를 지운 변이(M2)와 사슬 부분 부여 변이(M13)는 안전을 깨지
  못했다 — ENTER 때 "S 전체가 그 요청의 잠정 표에 묶여 있나" 를 확인하므로 확정 단계에서 배타가 다시 선다.
  2 절 논증 3 번의 "w 는 r_B 에 투표할 수 없다" 는 "w 는 r_B 를 확정할 수 없다" 로 읽어야 정확하다.
  잠정 표 배타는 헛된 ENTER 를 줄이는 활성 장치다.
- **incarnation 은 필수다 — 다만 보고서가 든 이유와 다른 경로로.** 보고서는 "재시작 전 요청의 늦은 ACK" 를 들지만,
  그것은 요청자의 요청 ID · 자원 집합 확인이 이미 막는다. 깊은 탐색(3000 × 100)이 incarnation 고정 변이(M15)에서 찾은
  경로는 **재시작 때 보낸 옛 요청 정리 RELEASE 가 늦게 도착해, 같은 ID 를 다시 쓴 새 요청의 확정 표를 푸는 것**이다
  (투표자는 같은 ID 의 확정 표를 "중복 요청" 으로 보고 곧바로 다시 허가한다). 결과는 확정 표 0 개로 허가된 로봇.
  결정적 시험 `test_restart_reuse_of_request_id_cannot_be_freed_by_old_cleanup` 으로 고정했다.
  **요청 ID 확인(M8)도 필수**다 — 결정적 반례 시험이 있다.
- **기각 변형의 반례는 보고서보다 약한 조건에서 나온다**: 대칭 관측 어긋남({A,B,C} 대 {C,D,E})이 아니라
  한 로봇의 관측 오류 하나(r3 이 r1 을 못 본다고 믿음)로 충분하다.
- **기억 상실 투표자를 셈에 넣어야 한다**: 확정 표를 쥔 투표자가 디스크를 잃으면 "확정 표 ≥ 3" 은 깨지지만,
  영구 기권하므로 남이 쓸 수 있는 투표자는 n − 확정 − 기권 < q 로 여전히 모자라다.

## 11. 시험으로 주장하는 범위

- "Python 코어를 무작위 고장 시나리오(메시지 손실·중복·재정렬·지연, 투표자 crash-restart, 시계 독립 진행·리셋,
  멤버십 관측 어긋남)로 n 회·단계 ≤ m 시험, 불변식 위반 0" — n·m 은 REPORT 와 시험 파일에 적힌 설정값.
- 같은 하네스에서 보고서가 기각한 "생존 멤버 전원 REPLY" 변형은 반례가 나와야 한다 (시험의 힘 대조).
- 변이: 가드를 하나씩 되돌려 시험이 잡는지. 결과는 REPORT 에.
- 모델 검사(TLC)는 f2 의 몫이다. 이 문서의 상태·메시지 이름은 f2 스펙과 다음 물결에서 맞춘다.
