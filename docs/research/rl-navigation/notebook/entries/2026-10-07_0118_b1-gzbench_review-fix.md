# A5-5 리뷰 반영 — strict_est 지표 결함 수정·재계산과 독점 시간대 재측정

- 시각: 2026-10-07 01:18 KST (아래 "설정"까지는 코드를 고치고 측정하기 전에 쓰고 커밋한다) · 에이전트: b1-gzbench · 단계: A5
- 코드: 시작 `33cfd0d` (브랜치 `feature/b1-gzbench`). 고침 커밋은 결과 절에 적는다
- 근거: A5 적대적 검증 (판정 fix_needed, 리뷰 사본 `/home/CAPYI_AI/runs/bonus1/b1-gzbench/review/`) 의 차단 문제 2 개
- 관련 실행 id: 결과 절에 적는다

## 질문

1. (차단 1) `strict_est` (추정 자세 기준 엄격 성공) 의 두 결함 — (a) 정착 구간에서 낡은 추정 자세를 끌어 쓴다, (b) strict_gt 와 달리
   정체·시간 상한 순서를 보지 않는다 — 을 고치면 A5-3 표 6 의 "정답·추정 갈림 수" 와 A5-4 표의 "엄격 정답 / 추정" 이 얼마로 바뀌는가?
   고친 실행기는 정착 구간까지 추정 자세를 실제로 남기는가?
2. (차단 2, COORDINATION §8.1 "측정 시간대 독점 (22:59 추가)") 부하에 민감한 [측정] — RTF, scan→cmd·scan_age, 틱·결정 주기, 플랜트
   T_m·T_d 와 적합 RMSE, 에피소드 벽시계 — 을 `exclusive.sh` 로 다시 재면 공유 부하 하 예비값과 얼마나 다른가? 단계 C 소요 추정은?

## 리뷰 판단 (결과 전에 적는다)

- 차단 1 **동의**. 리뷰 스크립트 `review/scripts/strict_est_fix.py` 를 원자료(`A5_smoke/*/episodes/*.json.gz`)에 다시 돌려 같은 수를
  얻었다 [계산, 01:15 KST]: smoke_dwa 의 strict_est 가 마지막 추정 표본 뒤 시각에서 판정된 에피소드 11/20, 갈림 3 → 1 (m035·s114 가
  인공물, s089 만 진짜), smoke_rl 2 → 2, smoke_rlho 1 → 1, final_dwa m040 1 → 0. 원자료의 추정 표본 간격 [계산, `tmp/fix/est_gaps.py`]:
  감시 루프 안 p50 0.020 s · p99 0.029–0.031 s · 최대 0.165 s, 기록 끝 − 마지막 추정 표본 0.37–0.80 s (정착 구간 = 추정 없음).
- 차단 2 **동의**. §8.1 의 독점 규칙은 22:59 에 추가됐고 A5-3·A5-4 사전 등록(23:35)과 측정(23:38–00:46)보다 앞선다. 그 절을 다시 읽지
  않고 gz_slot 만 썼다 — 내 잘못이다. `/home/CAPYI_AI/logs/exclusive.log` 가 없다(독점 실행 0)는 것도 확인했다.

## 가설과 예측 (결과를 보기 전에 적는다)

### 지표 정의 고침 (고정 — 이 정의로 코드를 고친다)

- D1 실행기: 정착 대기(지면 진실 정지 또는 3 s sim, 이어 0.2 s 벽시계) 동안에도 감시 루프와 같은 방식(TF map→base, 약 50 Hz 벽시계)으로
  추정 자세를 표집한다. 마지막 표본 = `final.est`, 그 sim 시각 = `final.t`. 정착 표집은 rclpy 비의존 함수로 떼어 가짜 시계로 시험한다.
- D2 보간: 추정 자세는 **실제 표본 사이 틈이 0.25 s sim 이하인 곳에서만** 선형 보간한다. 표본 구간 밖과 더 긴 틈은 "미정" — 성공·Nav2
  호환·위치추정 오차 판정에 쓰지 않는다 (np.interp 의 끝값 고정 금지). 0.25 s 는 위 감시 루프 최대 틈 0.165 s 보다 크다.
  `final.est` 는 그 측정 시각(`final.t`, 옛 원자료에는 없으니 원자료의 마지막 지면 진실 시각 = 기록 끝)의 표본으로 덧붙인다 — 시간 상한
  뒤면 판정 구간 밖이다 (시간 초과 뒤 정착에서 잰 추정이 상한 전 성공으로 끼지 않게).
- D3 계약 순서: `strict_est` 도 `strict_gt` 와 같은 규칙 — {접촉, 추정 기준 엄격 성공, 정체, 시간 상한} 중 시간순 첫째가 성공이어야 한다
  (같은 시각이면 접촉 → 성공 → 정체 → 상한, 사건 없이 끝나면 strict_gt 와 같은 대체 규칙). 접촉·정체·상한은 물리 사건이라 두 기준 모두
  지면 진실로 본다 (Nav2 progress checker 는 추정 자세를 쓰지만 차이가 몇 cm 라 정체 시각 차는 무시 — 정의로 고정한다).
- D4 새 열 (gzbench/v1 에 덧붙임, 기존 열 뜻은 그대로): `outcome_contract_est` (추정 기준 계약 결과), `est_end_gap_s` (기록 끝 − 마지막
  실제 추정 표본, 덧붙인 final 제외).
- D5 옛 원자료 4 세션(smoke_dwa·smoke_rl·smoke_rlho·final_dwa)은 정착 구간 추정이 없으므로 recompute 는 **근사** — 정착 구간의 추정
  판정은 기록 끝 한 점(final.est)으로만 한다. 새 CSV 접미사 `_recomputed_estfix` (앞의 `_recomputed` 는 그대로 둔다).

### 예측

재계산 (리뷰 수를 본 뒤라 맹검이 아니다 — 리뷰 수를 그대로 예측으로 적는다):

- E1 strict_est 갈림: smoke_dwa 1/20 (s089), smoke_rl 2/20 (s012·s178), smoke_rlho 1/6 (s021), final_dwa 0/3.
  DWA 엄격 추정: 주 10/10, 스트레스 5/10. RL 은 그대로 (주 10/10, 스트레스 10/10).
- E2 outcome_contract·strict_gt·system_success·final_* 등 추정과 무관한 열은 바뀌지 않는다 (바뀌면 진단한다). 바뀌어도 되는 열:
  strict_est, nav2compat_est, loc_err_max_m·loc_err_p95_m(표본 구간 밖 제외, 차이 ≤ 0.01 m 예상), 새 열 2 개.

독점 재측정 (판정 규칙):

- X1 (합격 조건 — 고침 확인) 새 원자료의 오류 아닌 에피소드 **모두** `est_end_gap_s ≤ 0.10 s`, 그리고 같은 원자료를 recompute 하면 실행 중
  CSV 와 바뀐 칸 0.
- X2 (합격 조건) A5-4 의 H1–H4 를 정의 그대로 두 팔 모두 통과.
- X3 RL 결정 주기 에피소드 평균 0.100 ± 0.001 s (A5-1b — RTF 와 무관해야 한다).
- X4 RTF: 독점 에피소드 RTF 중앙 ≥ 0.90 (공유 0.81–0.83). 0.85 미만이면 "독점으로도 RTF 가 오르지 않았다" 로 적고 sampler 기록으로
  원인을 진단한다 (슬롯 밖 부하 — 아래 한계).
- X5 sim 시각 지표는 부하와 무관해야 한다: scan→cmd p50·scan_age p50 의 에피소드 중앙, 플랜트 T_m·T_d 가 공유 값과 0.01 s 안. 플랜트
  적합 RMSE 는 독점 탐침 둘 다 선 ≤ 0.005 (공유 3 회차의 0.024 는 부하 인공물이라는 A5-3 해석의 검증 — 넘으면 그 해석을 버린다).
- X6 틱 간격 sim ≈ 0.05 · RTF 이므로 독점에서 RTF 를 따라 커지고, 벽시계 틱 간격 0.0499 s 는 그대로.
- X7 에피소드 벽시계: 독점 평균이 공유보다 짧다 (주 집합 10 % 넘게). 단계 C 소요(팔당 주 100 + 스트레스 100)를 독점 값으로 다시 내어
  공유 값(DWA 3.0 h · RL 2.1 h)과 나란히 적는다.
- X8 고친 지표의 갈림 (새 원자료): 팔당 ≤ 2/20 (A5-3 의 원래 "≤ 10 %" 예측 그대로).

## 설정

- 고침: `src/amr_rl_bench` (metrics·실행기·정착 표집 모듈·시험), `rl/eval/gazebo` (recompute 접미사·분석 열·README). 단위 시험을 더하고
  가드마다 되돌려 시험이 잡는지 변이 검사 (cpu_slot). 린트(flake8 99·pep257) 통과 뒤에만 커밋.
- 재계산: `python3 rl/eval/gazebo/recompute.py A5_smoke/{smoke_dwa,smoke_rl,smoke_rlho,final_dwa} --suffix _recomputed_estfix` →
  `analyze_smoke.py --suffix _recomputed_estfix`. 실행 목록에 offline eval 로 등록.
- 독점 측정: 고침 커밋에서 **한 번의** `/home/CAPYI_AI/env/exclusive.sh b1-gzbench` 안에서 `A5_smoke/run_session.sh` 두 세션을 연달아
  — `excl_dwa` (dwa, 탐침 scan×2·step, smoke 20) → `excl_rl` (rl v12_s1 모드 B, 탐침 scan×2·step, smoke 20). 월드·이미지·시나리오·순서는
  A5-4 와 같다 (DOMAIN 33, IGN_PARTITION amr_ai_b1g, LOCALHOST_ONLY). 근접 인계 세션은 다시 돌지 않는다 (합격 조건 밖).
- 동시 기록: 10 s 마다 loadavg · nvidia-smi(사용률·메모리) · docker 컨테이너 이름 → `A5_excl/sampler.log`.
- **한계 (먼저 적는다)**: `exclusive.sh` 는 gz·cpu 슬롯만 잡는다. Isaac 슬롯(B 단계 학습 — 지금 PPO 3 개 + SAC 1 개, GPU 사용률 약 89 %)과
  다른 사용자 작업은 막지 않는다. 그래서 "독점" 은 "다른 Gazebo·CPU 집약 에이전트 작업 없음" 이지 기계 유휴가 아니다. 동시 Isaac
  작업은 sampler 로 남기고, 그 영향은 이 실험으로 분리하지 못한다.
- 분석: `analyze_smoke.py` (H1–H4·전이 3·4·6·소요), `analyze_step.py` (플랜트), `analyze_scan.py` (스캔 규약 — 부하 무관 확인용).
  공유 값은 지우지 않고 A5-3·A5-4 표에 독점 값을 나란히 적는다.

## 결과

(고침·재계산·독점 측정 뒤에 쓴다)

## 해석

(측정 뒤에 쓴다)

## 다음

(측정 뒤에 쓴다)
