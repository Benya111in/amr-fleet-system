# C2·C3 Gazebo 학습 정책 대 팀 DWA 본 측정 (사전 등록)

- 시각: 2026-10-07 10:55 KST (아래 "설정" 까지는 본 측정 결과를 보기 **전에** 쓰고 커밋한다) · 에이전트: b1-gzc · 단계: C (D4·D5)
- 코드: 측정은 `bece745` 의 고정 사본(`git archive`, Release `-Wall -Wextra -Wpedantic -Werror` — A5 와 같은 빌드 설정)으로 돈다.
  이 항목·실행기·분석 스크립트는 브랜치 `feature/b1-gzc` 에 커밋한다 (런타임 경로 `src/`·`scenarios_v1.json` 은 `bece745` 와 같아야 하고,
  시도마다 그 차이를 `HEAD.txt` 에 남긴다).
- 관련 실행 id: 결과 절에 적는다 (`C_b1gzc_*`, 실행 목록 `notebook/runs/b1-gzc.jsonl`)
- 원자료: `/home/CAPYI_AI/runs/bonus1/b1-gzc/` — `c2/pilot/`, `c2/batches/<팔>_b<k>/a<n>/`, `c2/sampler.tsv`, `ws/` (고정 사본), `driver/`

## 질문

Isaac 에서 학습한 PPO 대표 정책(B1 정본 φ, 대표 정책 규칙으로 고른 시드 하나)을 C++ Nav2 플러그인으로 Gazebo 폐루프에 넣었을 때, 팀 DWA 와
**같은 시나리오 400 개(주 200 + 좁은 목표 스트레스 200)·같은 순서·같은 스택**에서 엄격 성공(지면 진실)이 다른가? 그리고 근접 인계(목표 0.4 m 안
내부 DWA)를 붙이면 RL 단독·DWA 와 어떻게 다른가? (완성 계획 D5, §6 21:55 정정: 주 집합과 스트레스 집합을 같은 비중으로.)

## 이미 본 것 (사전 등록의 한계 — 숨기지 않는다)

- A5 스모크·독점 재측정(스모크 20 = 이 400 의 처음 묶음 안): DWA 주 10/10·스트레스 5/10, RL(v12_s1 — 이번 대표 정책이 아니다) 주 10/10·
  스트레스 8/10 (`2026-10-06_2335_b1-gzbench_smoke-dwa-rl.md` 독점 재측정 절). 근접 인계(v12_s1) 6 개 중 스트레스 2 개가 DWA 제자리 정렬
  막힘으로 Nav2 시간 초과.
- b1-ppo10-eval 의 Isaac 집합 평가 10 개 중 5 개(시드 0·2·4·6·8)의 main_3_15·stress_3_15·main_3_30 엄격 성공 수를 대표 정책 규칙을
  미리 계산하려고 읽었다 (~~10:58~~ **10:52** — 정정: 파일 시각 기준, `/home/CAPYI_AI/runs/bonus1/b1-gzc/tmp/select_partial_1100.json`). 나머지 5 시드는 아직 없다.
- 그래서 아래 예측은 맹검이 아니다. 판정 규칙(통계·제외·일정)은 그 값들과 무관하게 과제 지시와 A5 의 지표 정의를 따른다.

## 팔 (세 팔 모두 같은 스택 — `run_bench.sh`: system.launch 정적 월드 + rl_navigation.launch, Nav2 파라미터 동일)

| 팔 | 제어기 | 정책 | 근거 |
|---|---|---|---|
| DWA | BT 기본 'DWA' = 팀 DWA, 배포 파라미터 그대로 (`nav2_params.yaml`: controller_frequency 20 Hz, goal checker xy 0.10·yaw 0.05) | — (RL 플러그인은 'RL' 로 실리되 쓰지 않음 — A5 와 같게 ONNX 는 v12_s1) | A5 |
| RL | `replace_dwa` — 'DWA' 자리에 RL 플러그인 (모드 B, ONNX) | **대표 정책 단독** | 아래 「대표 정책」 |
| RL+인계 | RL + `handover_radius 0.4` (목표 0.4 m 안은 내부 DWA) | 같은 대표 정책 | A5 와 같은 반경 — 바꾸지 않는다 |

## 시나리오·순서·일정

- `rl/eval/gazebo/scenarios_v1.json` (`gz_static_v1`, sha256 은 시도마다 `HEAD.txt`) 의 `exec_order` 400 전부. 묶음 k (k = 0…7) =
  `exec_order[50k : 50k+50]` (주 25 + 스트레스 25 — 실제로 확인함). **세 팔 모두 같은 400 개, 같은 순서**(`--select all --start 50k --count 50`).
- **레인 둘** (ROS_DOMAIN_ID 35·36, IGN_PARTITION `b1gzc_l35`·`b1gzc_l36`, ROS_LOCALHOST_ONLY, 기본 브리지 네트워크, 컨테이너 `amr-ai-b1-gzc-l35/36`).
- 고정 일정 `rl/eval/gazebo/stage_c/schedule_c.tsv`: 레인 35 = dwa_b0, rl_b0, dwa_b2, rl_b2, dwa_b4, rl_b4, dwa_b6, rl_b6, rlho_b0, rlho_b2, rlho_b4, rlho_b6;
  레인 36 = 같은 꼴로 홀수 묶음. 각 팔이 각 레인에서 묶음 4 개씩. 같은 k 의 DWA·RL 묶음은 **같은 레인에서 잇달아** 돈다 (짝 차이에서 레인·시간대
  효과를 줄인다). 근접 인계 묶음은 레인마다 DWA·RL 뒤.
- **문 규칙 (결과 전에 고정)**: DWA 묶음은 대표 정책과 무관하므로 이 항목 커밋 뒤 바로 시작한다. RL·근접 인계 묶음은 문
  (`gate/rl_ready.json` — 아래 대표 정책 대조 일치 + C1 일치 검사 통과 뒤 내가 쓴다)이 열려야 돈다. 레인의 다음 묶음이 문에 막히면 **같은 레인 목록의
  다음 DWA 묶음을 먼저** 돈다 (일정 순서에서 벗어난 실제 순서는 레인 로그 `c2/lane*.log` 에 남고 결과에 적는다). 대표 정책 대조가 어긋나면
  RL 팔을 띄우지 않고(`gate/STOP_RL`) `ALERT_b1-gzc.txt` 에 쓰며 DWA 묶음만 계속한다.
- **파일럿 (셈에 넣지 않음)**: 본 측정 전에 smoke 20 (주 10 + 스트레스 10) 을 **레인 둘에서 함께 DWA 로** 돌린다. 레인별 에피소드 RTF 중앙(오류 아닌
  에피소드)이 **둘 다 ≥ 0.75 면 레인 둘, 하나라도 0.75 미만이면 레인 하나**(35)로 간다 — 그때는 일정의 pos 순으로 35·36 목록을 번갈아 합친 순서.
  파일럿에서 레인마다 20 행이 아니거나 오류 > 2 면 본 측정을 띄우지 않고 진단한다. 결정은 `c_state.py pilot-decide` 가 기계적으로 내린다.
- 실행기: `rl/eval/gazebo/stage_c/` (c_master.sh → 파일럿 → c_lane.sh × 레인 → c_batch.sh). 전체를 `exclusive.sh b1-gzc` **한 번** 안에서
  `nohup setsid` 로 돈다 (레인 둘이 같은 독점을 쓴다 — 교착 없음). 재개 가능: 끝난 묶음(DONE 표지 = [start, start+50) 의 exec_index 전부)은 건너뛰고,
  끊긴 묶음은 첫 빠진 exec_index 부터 새 시도(a2, a3 …)로 잇는다. 진전 없는 시도 3 번이면 그 묶음을 FAILED 로 두고 ALERT (같은 실패 세 번 — 진단).
  묶음 상한 3 h (넘으면 그 시도를 끊고 이어 돈다). 같은 드라이버 두 개는 잠금(`locks/*.lock`)이 막는다.
- 측정 코드 고정: 고정 사본 `runs/bonus1/b1-gzc/ws` (bece745) 을 **읽기 전용**으로 마운트 — 측정 중 작업 트리를 고쳐도 측정은 바뀌지 않는다. 시도마다
  `HEAD.txt` 에 사본 커밋, 작업 트리 HEAD·status·`diff --stat`, 사본 대비 런타임 경로 차이, 모델·월드·시나리오 sha256, 동시 컨테이너를 쓴다.
  에피소드 행의 `commit` 열 = 사본 커밋. (에피소드마다 따로 쓰지 않는 이유: 한 시도 = 한 실행기 프로세스 = 고정 사본이라 에피소드 사이에 코드가
  바뀔 수 없다.) 월드: `make_static_world.py` 로 다시 만든 정적 사본 — A5 의 `world/rtf10/warehouse.sdf` 와 sha256 같음 (`edf913f4…`).

## 대표 정책 (COORDINATION §8.4 — b1-ppo10-eval 의 사전 등록 규칙으로만)

- 규칙: b1-ppo10-eval 0915 항목 「C 단계 대표 정책 선택 규칙」 (최고 시드 고르기 금지 — main_3_15 엄격 성공 수의 중앙값에 가장 가까운 시드,
  동률은 stress_3_15 → main_3_30 → 시드 번호). 내가 원자료(`b1-ppo10-eval/eval/B1e_ppo10_s*_final_sets/eval.json`, 10 개)로
  `rl/eval/gazebo/select_rep_policy.py` 를 돌려 **스스로 계산**하고 `analysis/selection.json` 과 대조한다. 같으면 문을 연다, 다르면 STOP_RL·ALERT.
- 10 시드가 다 나오기 전 값(10:52, 5 시드)으로는 시드 2 가 1 순위였다 — **이것으로 정하지 않는다** (중앙값이 10 시드로 바뀐다).

## C1 전이 점검 재확인 (대표 정책에 걸리는 것만, 작게 — 판정 규칙)

- C1-a ONNX 출력 일치: A2 도구(`tools/parity/run_parity.sh` — 고정 사본에서 컨테이너 이름만 `amr-ai-b1-gzc-*` 로)로 대표 정책의 Python ORT 대
  C++ `OnnxPolicy`. 통과 = A2 의 P1 (최대 |Δaction| < 1e-5, 모든 묶음) **과** P4 (C++ 관측 대 numpy 관측 ≤ 1e-5) **과** P5 (정규화 한 번). 실패면 RL 팔을
  띄우지 않는다 (ALERT).
- C1-b 결정 주기 (sim): RL 에피소드마다 `decision_period_mean_s` ∈ [0.099, 0.101] (A5 0.0998–0.1001). 벗어난 에피소드 비율이 RL 팔의 5 % 를 넘으면
  원인을 진단하고 보고한다 (주 결론 표는 그대로 내되 그 사실을 먼저 적는다).
- C1-c scan→cmd 지연: RL 에피소드별 `scan_to_cmd_p50_s` 의 중앙이 ≤ 0.10 s (A5 독점 0.071), `scan_age_p50_s` 중앙을 A5 (공유 0.084 · 독점 0.064)
  와 나란히 적는다. 판정은 C1-a 만 문을 막고, b·c 는 기술 + 경보다.
- 모두 본 측정 에피소드의 열로 잰다 (따로 Gazebo 를 돌리지 않는다 — 독점 시간대를 레인 둘이 쓴다).

## 지표와 통계 (결과 전에 고정)

지표 정의는 `src/amr_rl_bench/amr_rl_bench/metrics.py` 머리말 (A5-2 + A5-5 고침 `0fd7157`) 그대로. 지표를 바꿔야 하면 Gazebo 를 다시 돌리지 않고
`recompute.py` 로 다시 계산해 새 접미사로 남긴다 (원래 CSV 는 지우지 않는다).

### 제외 규칙 — '전체' 와 '유효' 두 가지로 보고

에피소드 제외 분류 (버리지 않고 팔·집합별로 센다):
- **E1 하네스 오류**: `setup_ok` 거짓 또는 `outcome_contract = error` (준비 실패, 목표 수락 실패, 지표 계산 실패).
- **E2 시작 위치추정 미확립 (H4 실패)**: `loc_err_start_m > 0.10` 또는 `lost_at_start` 참.
- **E3 시뮬레이터 사고**: `cancel_reason = watchdog` (sim 시계 30 s 정지) 또는 에피소드 RTF < 0.5.
- **E4 주행 중 위치추정 붕괴**: `loc_collapse` 참 (추정–정답 > 0.5 m) 또는 `lost_events > 0`.

- **유효** 짝 = 두 팔 에피소드가 모두 E1–E4 가 아닌 시나리오. **전체** 짝 = 모든 시나리오, E1–E3 에피소드는 그 팔의 **실패**로 센다 (두 팔 대칭),
  E4 는 지면 진실 결과를 그대로 쓴다 (정답 기준 결과는 붕괴와 무관하게 잴 수 있다).
- E4 는 제어기 때문일 수 있다 (예: 정책의 주행이 AMCL 을 흔든다) — 팔별 E4 수를 따로 적고, 두 팔 차가 ≥ 3 이면 "제어기 관련 가능" 으로 먼저 쓴다.
- **오류율 규칙**: 한 팔의 E1+E3 이 그 팔 400 의 5 % (20) 를 넘으면 원인을 진단하고, E1+E3 이 ≥ 3/50 인 그 팔의 묶음을 새 시도 디렉터리(`rerun`)로
  다시 돈다. 분석은 다시 돈 묶음을 쓰고 원래 묶음 값도 함께 적는다. 이 규칙은 E2·E4 에는 쓰지 않는다.

### 주 결론 (확증 — 이것 하나)

- 대상: **주 집합 200, 유효 짝**, `strict_gt` (지면 진실 엄격 성공: 목표 xy ≤ 0.10 m ∧ |v| ≤ 0.05 m/s 가 계약 종단 사건 중 첫째).
- 효과: 짝 차이 Δ = mean(RL_i − DWA_i) (= RL 성공률 − DWA 성공률, 같은 짝). 95 % 구간 = **짝 부트스트랩** (시나리오를 복원 추출, B = 10 000,
  seed 20261007, 백분위).
- 검정: **McNemar 정확 검정 (양측)** — b = (RL 성공, DWA 실패) 짝 수, c = (RL 실패, DWA 성공); p = min(1, 2·P(X ≤ min(b, c))), X ~ Bin(b + c, ½);
  b + c = 0 이면 p = 1. α = 0.05. p < 0.05 면 "차이가 있다 (방향 = Δ 의 부호)", 아니면 "차이를 보이지 못했다" + 구간.
- **강건성 규칙**: 같은 계산을 '전체' 짝으로도 한다. 유효와 전체의 판정(p < 0.05 여부 또는 Δ 부호)이 갈리면 주 결론을 "강건하지 않음" 으로 쓴다.

### 부결론 (기술 — 구간만, p 값을 내지 않는다)

주 집합·스트레스 집합을 **같은 표에 같은 비중으로** (열: 주 | 스트레스, 행: 지표 × 팔·짝 차이). 비교: RL − DWA, RL+인계 − DWA, RL+인계 − RL (각각 짝).
- 성공 층위: `strict_est` (추정 기준 엄격), `nav2compat_gt` (0.25 m), `system_success` (Nav2 SUCCEEDED) — 성공률과 짝 차이, 짝 부트스트랩 95 %.
- 최종 거리 `final_dist_gt_m` 분포: 5/25/50/75/95 % 와 > 0.10·> 0.25 m 비율; 짝 차이의 중앙(Hodges–Lehmann 아님 — 단순 짝 차 중앙)과 부트스트랩 95 %.
- 주행 시간 `nav_time_s`·경로 길이 `path_length_m`: **둘 다 strict_gt 성공한 짝**에서 짝 차이 평균·중앙과 부트스트랩 95 %.
- CTE `cte_active_mean_m`·`max_dev_m`: 두 팔 모두 오류가 아닌 짝 (성공 무관), 같은 방식.
- safety 개입: `interventions` (건)·`intervention_time_s`·`zone_stop_s` — 에피소드 평균·합, 짝 차이 평균과 구간.
- 접촉: `contacts_static + contacts_dynamic > 0` 인 에피소드 수 (팔·집합별, Wilson 95 %), `outcome_contract = collision` 수.
- 목표 직전 정지: `near_goal_stop_max_s ≥ 2 s` 에피소드 수 (A5 와 같은 정의).
- 최종 yaw 오차 `|final_yaw_err_gt|`: strict_gt 성공 에피소드의 중앙·95 %·최대.
- 결과 분류 `outcome_contract` (success/stall/timeout/collision/error) 와 `outcome_system` 의 교차표.
- **스트레스 집합 구간**: 시나리오 부트스트랩과 함께 목표 군집 부트스트랩(목표 xy 를 1.0 m 단일 연결로 묶은 군집을 복원 추출)도 낸다 — 주장은 둘 중
  넓은 구간으로. (주 집합도 같은 두 구간을 내지만 목표가 거의 다 따로라 차이가 작을 것.)

### 부하·시간·실효 주기 (모두 에피소드마다)

- RTF (`rtf`), loadavg (`load_start`·`load_end`), GPU 사용률·메모리·동시 Isaac 컨테이너 수 (`c2/sampler.tsv` 10 s 표집을 에피소드 벽시계 구간으로 평균).
- **DWA 의 sim 기준 실효 제어 주기**: DWA 에피소드의 `ctrl_dt_sim_mean` (cmd_vel_nav 간격, sim). controller_server 는 벽시계 20 Hz 로 돌므로 예측은
  0.05·RTF s — sim 1 초당 결정 수가 공칭(20)의 1/RTF 배. 이 편향을 숫자로: 팔·레인별 중앙, sim 초당 결정 수, 그리고 DWA 성공·주행 시간을 RTF 삼분위로 나눈
  표 (기술). RL 은 `decision_period_mean_s` (sim 0.1 s 고정이어야 한다)와 `tick_dt_sim_mean`.
- 벽시계: 에피소드·묶음·팔별, 공유 Isaac 부하 아래 값임을 적는다 ("독점" = Gazebo·CPU 슬롯 독점이지 기계 유휴가 아니다 — §8.4).

### 2D·Isaac 과의 방향 비교 (기술)

- Isaac: 대표 정책의 집합 평가(b1-ppo10-eval) `main_3_30`·`stress_3_30` (길이 분포가 Gazebo 3–30 m 에 가장 가깝다) 과 `main_3_15`·`stress_3_15` 의
  엄격 성공을 Gazebo RL 의 주·스트레스 strict_gt 와 나란히. 방향 일치 = (i) 두 시뮬레이터 모두 주 > 스트레스, (ii) Gazebo − Isaac 차이 (전이 간극,
  Gazebo 쪽 Wilson 95 %). Isaac 에는 DWA 가 없어 RL − DWA 의 방향은 Isaac 과 비교할 수 없다 → "미측정".
- 2D: 같은 정책의 2D 교차 평가(B5)나 2D-DWA 결과가 분석 때 있으면 RL − DWA 부호를 비교하고, 없으면 "미측정" 으로 적는다 (이 과제에서 2D 를 새로 돌리지 않는다).
- **같은 이름, 다른 정의** (표로 결과 문서에 싣는다): 엄격 성공 — Isaac 은 관측 자세 = 정답(위치추정 없음), Gazebo 는 AMCL 추정으로 주행하고 정답으로
  판정(strict_gt) / 추정으로 판정(strict_est); 경로 — Isaac 은 풀의 고정 A* 경로, Gazebo 는 Nav2 계획기(같은 팀 A* 코어)가 주행 중 다시 계획;
  종단 — Isaac 은 계약 사건만, Gazebo 는 Nav2 BT(회복 행동·goal checker 가 먼저 끝낼 수 있음)가 끼고 정착 구간까지 봄; 접촉 — Isaac 은 풋프린트–장애물,
  Gazebo 는 정적 상자 부호 거리 + collision_monitor; 개입 — Isaac `cost_intervention_per_ep` (φ 비용)와 Gazebo `interventions` (safety 출력 ≠ 입력 구간 수)는
  단위가 다르다; 플랜트·지연 — Isaac DR 끔 명목값(T_m 0.08, 지연 0.04)과 Gazebo 측정값(T_m ≈ 0.004, scan_age ≈ 0.06–0.08 s)이 다르다 (A5-3).

## 가설과 예측 (결과 전에 — 낮은 확신, 위 「이미 본 것」 때문에 맹검 아님)

- P1 (주 결론): 주 집합 strict_gt 는 DWA ≥ 0.95, RL ≥ 0.95, 짝 차이의 95 % 구간이 0 을 포함하고 McNemar p ≥ 0.05 ("차이를 보이지 못함").
  어긋나 RL 이 유의하게 낮으면 원인 후보: 목표 직전 정지(b1-plugin A2), 위치추정 오차가 0.10 m 경계를 가르는 것 (A5-5 갈림 꼴).
- P2 (스트레스, 기술): strict_gt RL > DWA, 짝 차이 ≥ +0.15 (A5 독점 스모크 8/10 대 5/10 의 방향). DWA 실패는 STOP 존에 막힌 목표 0.10–0.20 m 앞 정지·Nav2 시간 초과.
- P3: Nav2 호환 0.25 m 은 세 팔 모두 두 집합에서 ≥ 0.97.
- P4: RL 의 CTE 평균이 DWA 보다 크다 (A5: 0.40 대 0.015 m). 주행 시간은 주 집합에서 ±15 % 안, 스트레스에서 RL 이 짧다.
- P5: 개입·STOP 존 시간은 스트레스에서 DWA ≫ RL. 접촉은 세 팔 모두 0–1 건.
- P6 (근접 인계): 주 집합은 RL 과 비슷 (±0.03), 스트레스는 RL 단독보다 낮다 (DWA 정렬 막힘 — A5 6 개 중 2 개).
- P7 (실효 주기): DWA `ctrl_dt_sim_mean` 중앙 ≈ 0.05 × RTF 중앙 (± 0.003). RL `decision_period_mean_s` 0.100 ± 0.001.
- P8 (Isaac 방향): 주 > 스트레스가 두 시뮬레이터에서 같다. Gazebo RL 주 strict_gt 는 Isaac main_3_30 보다 0–0.05 낮다.
- P9 (소요): 레인 둘이면 본 측정(DWA·RL·인계 각 400) 약 9–11 h (A5 독점 단일 레인 추정 DWA 3.0 h·RL 2.1 h /200 의 두 배를 레인 둘로 나누고 RTF 저하를 더함).

## 설정

- 시뮬레이터 Gazebo Fortress (정적 월드, RTF 목표 1.0, max_step 0.001), 이미지 `amr-ai/rl:0.3`, 계약 v1.2.0, 플러그인 모드 B, safety node 켬,
  지도 `maps/warehouse` (AMCL). 고정 사본 빌드 [측정]: `runs/bonus1/b1-gzc/logs/build.log` (13 패키지, 경고 0, 1 min 56 s).
- 측정 명령 (주 드라이버): `nohup setsid /home/CAPYI_AI/env/exclusive.sh b1-gzc bash /home/CAPYI_AI/runs/bonus1/b1-gzc/driver/c_master.sh`
  (`driver/` = 이 커밋의 `rl/eval/gazebo/stage_c/` 사본 — 실행 중 스크립트를 고치지 않으려고).
- 동시 부하: Isaac(B 단계 Phase2 — b1-sac·b1-dr·b1-ppo10-eval 등)은 isaac 슬롯이라 함께 돈다. 그 수를 `sampler.tsv` 로 남긴다.
- 기록: 묶음이 끝날 때마다 실행 목록(`C_b1gzc_<묶음>`)을 등록하고 이 항목에 짧은 중간 기록을 덧붙여 커밋한다. **주 결론 값은 모든 묶음이 끝나기 전에
  쓰지 않는다** (중간 집계는 실행 목록·로그에만).
- C4 (동적 장애물 스트레스)는 본 측정 뒤 DOMAIN 37 로, 별도 항목에서 사전 등록한다.

## 결과

(주 결론·부결론 표는 모든 묶음이 끝난 뒤에 쓴다. 아래는 중간 기록.)

### 중간 기록 1 — 파일럿과 레인 결정 (2026-10-07 11:25)

- [측정, `C_b1gzc_pilot_l35`·`_l36`, `c2/pilot/l{35,36}/a1/`, 11:01:16–11:25:19 KST] smoke 20 을 레인 35·36 에서 DWA 로 **함께** 돌렸다
  (셈에 넣지 않음). 두 레인 모두 20/20 완주, 오류 0, H4 위반 0. 에피소드 RTF 중앙 **0.623 · 0.623** (범위 0.563–0.677) — 사전 등록 문턱 0.75 미만이라
  규칙대로 **레인 하나(35)** 로 간다 (`c2/pilot/DECISION.json`, `c_state.py pilot-decide` 가 기계적으로 결정). 조건: Isaac 컨테이너 4–5 개 동시
  (GPU 사용률 92–94 %), loadavg(1 분) 7 → 40–50 (Gazebo 스택 둘 + Isaac).
- 레인 하나 일정 = `schedule_c.tsv` 의 pos 순으로 35·36 목록을 번갈아 합친 순서 (dwa_b0, dwa_b1, rl_b0, rl_b1, dwa_b2, …, rlho_b7 — `c2/lane35.list`).
  소요 어림이 레인 둘 9–11 h 에서 **약 15–17 h** 로 늘어난다 (A5 독점 단일 레인 추정: DWA 400 ≈ 6 h, RL ≈ 4.1 h, 근접 인계 ≈ 6.6 h).
- 파일럿 자체의 관찰 (기술, 셈에 넣지 않음): 두 레인의 결과가 20 개 모두 같았다 (계약 결과 success 15 · stall 5, 같은 시나리오에서). 주 10/10,
  스트레스 strict_gt 5/10 — A5 독점 스모크(RTF 0.88)와 같은 수. DWA 의 sim 기준 제어 간격 `ctrl_dt_sim_mean` 은 0.028–0.035 s (RTF 0.56–0.68) —
  공칭 0.05 s 의 약 0.62 배, 즉 sim 1 초당 결정 약 32 번 (공칭 20). 사전 등록한 실효 주기 편향이 이 부하에서 크다는 첫 숫자다.

### 중간 기록 2 — 대표 정책 대조와 C1-a, RL 문 열림 (2026-10-07 11:34)

- [계산, `C_b1gzc_select_rep`, `runs/bonus1/b1-gzc/c1/selection_recomputed.json`] 10 시드 집합 평가 원자료로 규칙을 스스로 계산: 후보 10 (모두
  main_3_15 ≥ 0.90, ONNX fails 빈 것), 중앙값 main_3_15 2041.5 · stress_3_15 454.5 · main_3_30 2037.5. 1 순위 |Δ| 가 시드 7 (2043) 과 시드 2 (2040)
  모두 1.5 로 동률 → 2 순위 stress_3_15 |Δ| 10.5 (444) 대 19.5 (474) 로 **시드 7**. b1-ppo10-eval 의 `analysis/selection.json` (11:32:57,
  `chosen_seed` 7, tie_level 1) 과 **같다**, ONNX sha256 `26fec594…` 도 같다. (처음 대조는 내 스크립트가 그쪽 키 `chosen_seed` 를 읽지 못해
  '불일치' 로 찍혔다 — 키 읽기만 고쳐 다시 대조했다. 선택 값 자체는 처음부터 7 이었다.)
- 10:52 의 5 시드 중간 계산에서는 시드 2 가 1 순위였다 — 10 시드로 중앙값이 바뀌어 7 이 됐다. 그 중간 값으로 정하지 않는다는 사전 등록대로.
- [측정, `C_b1gzc_c1_parity_s7`, `c1/out_s7/report.json`] **C1-a 통과**: 관측 9 745 개 (golden 17 · 무작위 계약 상태 4 096 · 표준정규 4 096 · Isaac
  기록 1 536) 에서 C++ `OnnxPolicy` 대 Python ORT 최대 |Δaction| **0** (비트 동일 전부), C++ 관측 대 numpy 관측 최대 **0** (1 191 936 값),
  정규화 한 번 (첫 Gemm 앞 Sub·Div·Clip 하나, "두 번 정규화" 경로와의 차 1.68–1.92), 학습 때 rsl_rl (CPU) 대 ORT 최대 9.2e-7, Isaac 기록 CUDA 출력 대
  ORT 3.9e-7, integrate_b 세 구현 차 0. A2 의 v12 정책 결과와 같은 그림.
- 문 열림 11:33:48 (`gate/rl_ready.json` — 모델 `models/B1_ppo10_s7_policy_final.onnx`, sha256 대조). 레인 35 는 dwa_b0 를 돌고 있고, 목록 순서상 첫 RL
  묶음(rl_b0)은 dwa_b1 뒤라 **문 때문에 일정 순서가 바뀐 묶음은 없다**.
- 단일 레인 RTF (dwa_b0 처음 9 에피소드, 기술): 0.83–0.88 — 파일럿(레인 둘) 0.56–0.68 보다 높고 A5 독점 0.84–0.90 과 비슷하다.

### 중간 기록 3 — C1-b·c 첫 RL 묶음 (rl_b0, 2026-10-07 13:20)

- [측정, `C_b1gzc_rl_b0`] 플러그인이 대표 정책을 실었다 (`nav.log`: "RLController 'DWA' 로드: 모델 /models/B1_ppo10_s7_policy_final.onnx (arm B,
  contract_version 1.2.0, obs_dim 291, algo PPO (rsl_rl 3.1.2) …").
- **C1-b 통과 (이 묶음)**: 에피소드별 `decision_period_mean_s` 0.09985–0.10014 s, [0.099, 0.101] 밖 0/50. 틱 간격 sim 은 0.05·RTF 꼴 (0.037–0.042).
- **C1-c**: `scan_to_cmd_p50_s` 에피소드 중앙 0.072 s (≤ 0.10 — A5 독점 0.071 과 같다). `scan_age_p50_s` 에피소드 중앙 **0.081 s**, p95 중앙 0.111 s —
  A5 독점 0.064 · 공유 0.084 사이 (부하에 따라 움직인다는 A5-5 의 미설명 관찰과 같은 방향). DWA 폴백 0, 계약 폴백 셀 0, 추론 p95 최대 0.21 ms.
- 판정 b·c 는 사전 등록대로 RL·근접 인계 전 묶음이 끝난 뒤 analyze_c.py 로 다시 센다.

### 묶음 진행 기록 (운영 정보만 — 결과 값은 실행 목록 `runs/b1-gzc.jsonl` 에 있고 여기에는 쓰지 않는다)

| 묶음 | 시각 | n | 시도 | 제외 E1/E2/E3/E4 | RTF 중앙 | 에피소드 벽시계 합 [min] | loadavg 시작 → 끝 |
|---|---|---|---|---|---|---|---|
| dwa_b0 | 11:25:19–12:06:36 | 50 | 1 | 0/0/1/0 | 0.833 | 38.7 | 40.18 → 30.42 |
| dwa_b1 | 12:06:41–12:42:16 | 50 | 1 | 0/0/1/0 | 0.798 | 33.0 | 28.86 → 29.19 |
| rl_b0 | 12:42:21–13:17:48 | 50 | 1 | 0/0/0/0 | 0.82 | 32.9 | 28.53 → 24.05 |
| rl_b1 | 13:17:54–13:52:39 | 50 | 1 | 0/0/0/0 | 0.815 | 32.2 | 22.61 → 33.22 |

- 진행 메모 (12:10): dwa_b0 의 E3 1 건은 exec_index 39 (`s033`) — watchdog 이 아니라 **에피소드 RTF 0.469 < 0.5** 로 분류됐다. 그때 loadavg 가
  36.6 → 52.0 으로 뛰었다 (11:57–12:00, 다른 사용자의 `openroad`·`ruby`·`python` 프로세스가 CPU 상위 — 우리 것이 아니고 건드리지 않는다).
  사전 등록 규칙(E3 = watchdog 또는 RTF < 0.5)은 바꾸지 않는다 — 결과 문서에 "RTF 만으로 E3 가 된 에피소드" 의 수와 그 결과를 따로 적는다
  ('전체' 에서는 규칙대로 실패로 센다). 같은 시간대 에피소드 RTF 0.63–0.72.

## 해석

(본 측정 뒤에 쓴다)

## 다음

(본 측정 뒤에 쓴다)
