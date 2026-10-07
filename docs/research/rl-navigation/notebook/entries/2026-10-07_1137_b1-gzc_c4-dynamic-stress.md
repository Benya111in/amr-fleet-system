# C4 동적 장애물 스트레스 — 시나리오 08 왕복에서 DWA 대 RL (사전 등록, 주장 없음)

- 시각: 2026-10-07 11:37 KST (아래 "설정" 까지는 C4 를 돌리기 **전에** 쓰고 커밋한다) · 에이전트: b1-gzc · 단계: C4
- 코드: 고정 사본 `bece745` (본 측정과 같은 `runs/bonus1/b1-gzc/ws`, Release), 실행기 `rl/eval/gazebo/stage_c/c4_run.sh`, 시나리오 생성
  `make_c4_scenarios.py` → `rl/eval/gazebo/stage_c/scenarios_c4_s08.json` (이 항목과 함께 커밋)
- 관련 실행 id: 결과 절에 적는다 (`C_b1gzc_c4_dwa`, `C_b1gzc_c4_rl`)
- 원자료: `/home/CAPYI_AI/runs/bonus1/b1-gzc/c4/` (`runs/{dwa,rl}/a<n>/`, `sampler.tsv`, `c4.log`)

## 질문

완성 계획 §2 "범위 밖으로 정직하게 남기는 것": 동적(비반응) 장애물 회피는 2b 실측으로 현재 방법이 배우지 못했다 (`dyn_obstacles.md` §1 —
순수 RL 후보 중 회피를 배운 것 없음, BC + KL 은 장애물 무시 추종기와 결말 92.5 % 같음). **정적 지도로만 학습한 대표 정책(B1 시드 7)을 동적
장애물이 있는 원본 창고 월드의 시나리오 08 왕복에 넣으면, 팀 DWA 와 비교해 접촉·성공·이격이 숫자로 얼마인가?** 한계를 숫자로 남기는 측정이다 —
**어떤 주장도 하지 않는다** (n 30 씩, 장애물 배치가 팔 사이에 짝지어지지 않는다).

## 가설과 예측 (결과 전 — 낮은 확신)

- DWA: 접촉 에피소드 0–1/30 (1차 물결 08 기준선 0/240 — `W0_dwa_s08_*`, 단 하네스·시작 방식이 다르다), 엄격 성공 ≥ 25/30.
- RL (B1 시드 7, 정적 학습): 접촉 에피소드 ≥ 3/30 (회피 학습 없음 — 2b), 그래도 엄격 성공은 ≥ 20/30 (작업자가 경로를 가로지르는 시간이 짧다).
  safety node 가 마지막 방어선이라 RL 쪽 개입·STOP 존 시간이 DWA 보다 크다.
- 이 예측이 틀려도 결론을 바꾸지 않는다 (주장 없음). 두 팔 모두 접촉 0 이면 "이 30 시행의 조우 조건에서 접촉이 나오지 않았다" 로만 쓴다
  (조우 수를 함께 — 조우가 적으면 이 측정은 회피를 시험하지 못한 것이다).

## 설정

- 월드: **원본** `src/amr_simulation/worlds/warehouse.sdf` (actor 6 — worker_crossing 포함, forklift_main, shuttle_amr; 고정 사본의 파일, sha256 은
  시도마다 `HEAD.txt`). 지면 진실 노드(collision_monitor, obstacle_truth)는 warehouse.launch 기본값대로 뜬다.
- 시나리오 `scenarios_c4_s08.json` (생성 결과 [계산]): 시행 30 = A(−4, −5, −0.46) → B(4, −9) 와 B(4, −9, 2.68) → A(−4, −5) 를 번갈아.
  팀 A* 경로 길이 AB 9.185 m · BA 9.273 m, 시간 상한 min(120, 20 + 3L) = 47.6 · 47.8 s sim, 목표 yaw = 경로 끝 접선 (−0.809 · 3.130 rad),
  목표 직진 여유 무한 (주 집합 조건 충족). 실행기가 시행마다 시작 자세로 순간 이동·위치추정 초기화 (본 측정과 같은 절차).
- **08 과 다른 점** (결과에 그대로 적는다): (1) 08 은 순간 이동 없이 연속 왕복, 여기는 시행마다 순간 이동; (2) 08 의 목표 yaw 는 다른 점의 yaw
  (목표에서 제자리 반 바퀴), 여기는 경로 끝 접선; (3) 08 시행 상한 90 s, 여기는 계약 식 47.6 s; (4) 접촉 판정 원천은 같은 collision_monitor 지만
  여기는 정적 기하 접촉에서만 실행기가 즉시 취소하고 동적 접촉은 기록만 한다 (계약 종단 사건으로는 collision); (5) **actor·지게차 위상은 세션의
  누적 sim 시각을 따르므로 두 팔이 같은 시행에서 같은 장애물 배치를 보지 않는다** — 짝 비교를 하지 않고 팔별 수만 적는다.
- 팔: DWA (배포 파라미터) 30 → RL (대표 정책 B1 시드 7, `gate/rl_ready.json` 의 모델, 모드 B, 근접 인계 없음) 30. 같은 고정 사본·이미지·Nav2 파라미터.
- 자원: 본 측정이 끝난 뒤 (`c2/master_ended.txt`) `exclusive.sh b1-gzc` 안, ROS_DOMAIN_ID 37, IGN_PARTITION `b1gzc_c4`, 컨테이너 `amr-ai-b1-gzc-c4`.
  대기자를 `nohup setsid bash c4_run.sh --after-main` 으로 미리 띄운다 (세션이 끊겨도 이어지게). 동시 Isaac 부하는 `c4/sampler.tsv`.
- 보고 (팔별, 기술 — 구간은 Wilson 95 %): 접촉 에피소드 (`contacts_dynamic > 0` · `contacts_static > 0` 따로), 계약 결과 분포, strict_gt·nav2compat_gt·
  system_success, 조우 (`min_clear_dynamic_m ≤ 2.0 m`) 수, `min_clear_dynamic_m` 5/50/95 %, 개입 (건·시간)·STOP 존 시간, `max_dev_m`·`return_max_s`
  (명세 190·191 행 꼴의 참고값 — 정의가 08 과 같은 활성 계획 기준), 주행 시간 (성공), RTF. 제외 분류 E1–E4 는 본 측정과 같다 ('전체' 와 '유효').
- 시나리오 08 기준선(1차 물결 DWA 0/240, 다른 하네스)과는 같은 표에 섞지 않고 따로 적는다.

## 결과

(C4 뒤에 쓴다)

## 해석

(C4 뒤에 쓴다)

## 다음

(C4 뒤에 쓴다)
