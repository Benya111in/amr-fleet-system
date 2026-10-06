# 1차 물결 r3 리뷰 — 정책→DWA 폴백 전환의 속도 점프와 수정 (자름 + 창 재설정)

> **소급 기록** (작성 2026-10-06 19:13 KST 첫 커밋, b1-backfill, 단계 A4). 원 작업: r3 리뷰(scratchpad 재현), 수정 cdead19·45aaf06 (09:28)·
> ffd359b (09:42), 폐루프 확인 09:29–09:46, 재확인. 수치는 `smoke/rev*_kin_*/summary.json` 의 `tick_step` 에서 다시 읽었다.

- 시각: 2026-10-06 09:11 KST (원 작업; 첫 리뷰 빌드 로그 rev_build1 09:11) · 에이전트: r3 리뷰어·수정 에이전트 (기록 b1-backfill) · 단계: W1
- 코드: 고치기 전 dc56d03, 1 회차 45aaf06 (자름만), 2 회차 ffd359b (창 재설정 + 자름)
- 관련 실행 id: `W1_r3_rev_kin_{follow,nav}_to014_{fixed,prefix}`, `W1_r3_rev2_kin_{follow,nav}_to014_{fixed,prefix}`

## 질문
정책 구간 뒤 DWA 폴백으로 넘어갈 때, 내부 DWAController 의 창 중심(v_last_, w_last_)이 DWA 자신이 마지막으로 낸 명령에 머물러 있어
첫 명령이 계약 가속 한계(a_max·Δt = 0.05 m/s, α_max·Δt = 0.10 rad/s / 50 ms 틱)를 넘는가. 고치면 사라지는가.

## 당시 가설과 예측
- (리뷰어) 넘는다 — 리뷰 전용 gtest 에서 v_meas 0.30 → DWA 출력 0.15 (dv −0.15, a = −3.0 m/s²) 재현.
- (수정 에이전트) 첫 DWA 틱만 자르면 둘째 틱에 다시 뛴다 → 매 DWA 틱 자르기 + 창 중심이 어긋나면 `dwa_->activate()` 로 측정 속도에 재설정.

## 설정
- Nav2 + kinematic_sim (Gazebo 아님), heuristic_c 모드 C, `rl.max_inference_ms` 0.14 로 간헐 시간초과를 일으켜 정책→DWA 전환을 반복.
- 같은 워크트리에서 고친 판(fixed)과 고치기 전 플러그인(prefix, overlay `/pre` = dc56d03)을 번갈아 돌림. follow 6 m, navigate (3,−6).
- 지표: 틱당 명령 변화의 최대값 (전체·DWA 틱·전환 틱).

## 결과
[측정, `tick_step`; 시간초과가 무작위라 실행마다 전환 수가 다르다; loadavg 55–82, 공유 부하 하 예비값]

| 회차 | 실행 | 판 | 전환 틱 수 | 전환 max \|Δv\| | 전환 max \|Δω\| | 전체 max \|Δω\| | 결과 |
|---|---|---|---|---|---|---|---|
| 1 (45aaf06) | follow | 고친 판 | 21 | **0.050** | 0.100 | 0.100 | 성공 |
| 1 | follow | 고치기 전 | 20 | **0.360** | 0.100 | 0.100 | 성공 |
| 1 | navigate | 고친 판 | 39 | 0.050 | 0.100 | 0.100 | 성공 |
| 1 | navigate | 고치기 전 | 36 | 0.350 | 0.454 | **0.482** | 성공 |
| 2 (ffd359b) | follow | 고친 판 | 22 | 0.050 | 0.100 | 0.100 | 성공 |
| 2 | follow | 고치기 전 | 17 | 0.350 | 0.100 | 0.100 | 성공 |
| 2 | navigate | 고친 판 | 42 | 0.050 | 0.100 | 0.100 | 성공 |
| 2 | navigate | 고치기 전 | 29 | 0.330 | 0.474 | 0.474 | 성공 |

- 8 회 모두 목표 도달(status 4). 고친 판의 DWA 자름(`dwa_limited`) 33–95 회 [측정]; 창 재설정 ≥ 44 회 [문서주장 — summary.json 에 없음].
- 리뷰 재확인 [문서주장, scratchpad `r3recheck` 로그 인용]: 재확인 gtest 5/5 가 HEAD 에서 통과(max |dv| 0.05), 고치기 전 dc56d03 에서 3/5 실패.
- 수정 쪽 변이 14 중 13 잡힘, R3 (동기 판정에서 ω 비교 생략) 생존.

## 해석
- 폴백 전환 점프는 실제였고 고쳐졌다 — 단 **운동학 시뮬에서만** 확인했고, 고친 뒤 Gazebo 스모크는 다시 돌리지 않았다.
- `activate()` 우회는 DWA 의 게이트 상태와 진동 항(dir_flip/spin_flip)도 지운다. 배포 설정 yield_gate 가 꺼져 있어 지금은 무해하다.
  근본 해결은 amr_navigation 의 공개 API(`syncLastCommand(v, w)`) — 완성 계획 A2 가 b1-plugin 소유 경로에 이 API 와 시험을 넣었다.
- 자른 DWA 명령은 DWA 가 충돌 검사를 한 후보가 아닐 수 있다 (계약상 최대 감속과 같아 물리적으로 나빠지지는 않음).
- 플러그인 gtest 간헐 정지 2 회 관측, 반복 170 회에서 재현 0 — 원인 미확인.

## 무엇을 바꾸게 했나
- 완성 계획 A2: 플러그인을 계약 v1.2 로 + amr_navigation 동기화 API + Python/C++ ORT 일치.

## 원자료
- `/home/CAPYI_AI/runs/r3-controller/smoke/rev{,2}_kin_*/` (summary.json, HEAD.txt, pkg_prefix.txt, load.txt, launch.log)
- 로그 `/home/CAPYI_AI/logs/agents/r3-controller/rev_*` (빌드·시험·변이·정지 반복)

## WAVE 보고와 원자료의 차이
- WAVE fix 의 "nav |Δω| 0.48·0.47" 은 **전체 틱** 최대(0.482·0.474)다. 전환 틱만 보면 1 회차 0.454 다 (정의 차이, 결론 불변).
- 그 밖(0.36/0.35/0.35/0.33, 전환 21·22·39·42 회, 8/8 도달)은 원자료와 같다.

## 실행 목록에 등록한 run (`runs/b1-backfill.jsonl`, entry = 이 파일)

| run_id | 종류 | 시뮬 | 커밋 | 시드 | 표기 |
|---|---|---|---|---|---|
| `W1_r3_rev_kin_follow_to014_fixed` | test | offline | 45aaf06 | n/a (시간초과 무작위) | [측정] |
| `W1_r3_rev_kin_follow_to014_prefix` | test | offline | dc56d03 | n/a (시간초과 무작위) | [측정] |
| `W1_r3_rev_kin_nav_to014_fixed` | test | offline | 45aaf06 | n/a (시간초과 무작위) | [측정] |
| `W1_r3_rev_kin_nav_to014_prefix` | test | offline | dc56d03 | n/a (시간초과 무작위) | [측정] |
| `W1_r3_rev2_kin_follow_to014_fixed` | test | offline | ffd359b | n/a (시간초과 무작위) | [측정] |
| `W1_r3_rev2_kin_follow_to014_prefix` | test | offline | dc56d03 | n/a (시간초과 무작위) | [측정] |
| `W1_r3_rev2_kin_nav_to014_fixed` | test | offline | ffd359b | n/a (시간초과 무작위) | [측정] |
| `W1_r3_rev2_kin_nav_to014_prefix` | test | offline | dc56d03 | n/a (시간초과 무작위) | [측정] |
