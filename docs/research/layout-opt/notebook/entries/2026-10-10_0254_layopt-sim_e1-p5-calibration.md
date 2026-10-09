# E1 보정 (실험 0 오프라인): P5 작업량을 E1 로 재현해 Gazebo 7 회와 비교

- 시각: 2026-10-10 02:54 KST · 에이전트: layopt-sim · 단계: E1-calib
- 코드: `60e6614` (브랜치 `feature/layopt-sim`)
- 관련 실행 id: (보정 실행 뒤 채운다 — runs/layopt-sim.jsonl)

## 질문
E1 이 P5 작업량(5 건 동시 게시, 로봇 5 대, 대기 슬롯 출발·복귀)에서 Gazebo 유효 7 회의 makespan · 평균 작업 시간 ·
게이트 대기(실행당 · 정지당) · 정지 위치를 재현하는가 (design §6 실험 0, 게이트 B), 어느 쪽으로 치우치는가.

## 가설과 예측
결과를 보기 전에 적는다. 단, **미리 본 것**을 밝힌다: Gazebo 원자료 수치(인벤토리 + 내가 다시 뽑은 gate 구간 ·
phase 지속시간)와 E1 개발 중 P5 스모크(지금 파일 시드 0·1 — makespan 283–325 s, 평균 작업 229–243 s,
dock_1_approach 대기 88–131 s, 교착 2/실행)를 이미 봤다. 그래서 아래 예측은 "눈먼" 예측이 아니다.
- makespan: E1 중앙값 280–340 s, Gazebo 범위 [280, 552] 안. 긴 꼬리(483 · 552 s)는 E1 에 거의 없다 —
  Gazebo 꼬리는 DWA 머뭇거림 · 미해결 교착(234 s hold) · 측위 실패에서 오고 E1 에는 그 기제가 약하다 → **낙관**.
- 평균 작업 시간: E1 중앙값 225–250 s (Gazebo 236–309, 중앙값 ≈ 244) — 아래쪽으로 치우침(**낙관**).
- 게이트 대기/실행: E1 80–150 s (Gazebo 분산 49–155). 정지 위치: dock_1_approach 가 가장 크다 (dock_1 을 두 작업이
  집고 한 작업이 내려놓는다). E1 은 교차로(x_bc_2 · x_bc_3) 정지가 Gazebo 보다 적다 — 분산 모드의 투표 게이트는
  결정점 정지가 잦았다 (중앙 모드와 기제가 다르다).
- 교착: E1 실행당 0–2 (도크 출구 HEAD_ON). Gazebo 중앙 2 회 중 1 회 HEAD_ON 미해결, 분산 5 회는 중앙 교통 관리자가
  없어 비교 불가.
- 로봇당 주행: E1 ≈ 150 m (Gazebo 152–164) — DWA 옆 비킴 · 도킹 재시도가 없어 3–5 % 짧다.
판정 규칙 (게이트 B, 이 항목에서 고정): makespan · 평균 작업 시간 각각 E1 중앙값이 Gazebo [최솟값, 최댓값] 안이고
E1 시드의 50 % 이상이 그 범위 안이면 "재현", 아니면 "미달". 게이트 대기/실행도 같은 규칙. 정지 위치는 상위 2 구역이
같으면 "일치". 두 E1 판(지금 파일 · 기준 묶음)을 따로 판정한다. 결과를 보고 E1 파라미터를 바꾸면 그 단계마다
일지에 남기고 바꾼 판을 별도 run_id 로 다시 돈다 (원래 판은 지우지 않는다).

## 설정
- E1: layopt/sim (이 항목을 커밋하는 커밋), amr-ai/rl:0.3, dt 0.1 s, 실제 TrafficManager(traffic.yaml),
  팀 A*(amr_core_py), actor 8 개(시드별 위상), 도킹 시간 [측정] 값(perceive 3.3 · dock U(21, 24.5) · undock 15.8 s),
  P5 작업 5 건(t = 60 s, small 2 kg, 우선순위 100…96), 시작 잔량 100 %(충전 없음), 지평 900 s, 끝나면 멈춤.
- 두 판: (a) 지금 파일 `current` (maps/warehouse.yaml SLAM 지도 + src/amr_fleet/config/traffic_zones.yaml),
  (b) 기준 묶음 `v10-v20-v30-v40-v50` (래스터 지도, 생성기 2b71c30). 시드 0–49 (n = 50 씩).
- Gazebo: /home/CAPYI_AI/runs/b24/b4-p5/raw/p5_* 중 metrics.json valid: true 7 회 (분산 5 · 중앙 2).
- 지표 정의: layopt/sim/calib.py 머리말 (게이트 구간 · 정지 ≥ 1 s · 정지 위치 ≥ 5 s · 교착).
- 명령: `layopt/sim/run_sweep.sh p5calib_current -- --bundles current --seeds 0:49 --workload p5 --horizon 900
  --soc 100 --end-when-done --stage E1-calib`, 같은 것을 기준 묶음으로; 비교는
  `python3 -m layopt.sim.calib gazebo …` · `… compare …` (원자료 /home/CAPYI_AI/runs/layopt/e1/<run_id>/).

## 결과
(표: 시드별 값, 실패 시드 수. [측정]/[계산]/미측정 표기. 원자료 경로)

## 해석
(결과가 말하는 것과 말하지 않는 것. 공유 부하 하 예비값 여부)

## 다음
(이 결과로 바뀌는 결정, 남은 질문)
