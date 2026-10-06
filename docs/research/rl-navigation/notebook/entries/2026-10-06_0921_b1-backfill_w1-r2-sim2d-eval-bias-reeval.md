# 1차 물결 r2 리뷰 — "먼저 끝난 에피소드" 평가 편향의 발견과 체크포인트 재평가

> **소급 기록** (작성 2026-10-06 19:12 KST 첫 커밋, b1-backfill, 단계 A4). 원 작업: r2 리뷰(09:21 이전), 수정 a8afe77·5b42ca8
> (09:32–09:33), 재평가 09:34–09:47, 재확인 09:58–10:08. 수치는 리뷰 `diag*_summary.log`, 각 run 의 `reeval.jsonl`,
> 재확인 `crosscheck_reeval_vs_review.log` 에서 다시 읽었다.

- 시각: 2026-10-06 09:21 KST (원 작업, 리뷰 증거 복사 시각) · 에이전트: r2 리뷰어·수정 에이전트 (기록 b1-backfill) · 단계: W1
- 코드: 리뷰 658aa73 사본, 수정 a8afe77 (평가), a6660f3 (env 시험), 5b42ca8 (재평가 도구), 486fdce (README)
- 관련 실행 id: `W1_r2_review_diag2_unbiased`, `W1_r2_{smoke_B1,B_main1,B_lag1,B_bc1,B_bc2,B_bc3,B_noobs1,C_scratch1,C_scratch2}_reeval`

## 질문
학습 중 평가(`ppo.evaluate`)가 보고한 성공률(B_bc3 0.415 → 0.429, "SE 0.022")은 정책의 실제 성공률을 재는가.

## 당시 가설과 예측
- (리뷰어) 512 env 를 굴려 먼저 끝난 에피소드 500 개에서 멈추는 추정량은 짧게 끝나는 결말(충돌) 쪽으로 치우친다. 실제 성공률은
  다르고, 치우침 방향은 결말 분포에 따라 바뀐다.

## 설정
- 리뷰 진단: 같은 체크포인트를 (a) 옛 추정량과 (b) env 마다 리셋 직후 첫 에피소드만 세는 추정량으로 비교, 장애물 mixed 와 none.
- 수정(a8afe77): evaluate() 가 env 마다 첫 에피소드 하나만 센다, 라운드마다 env.gen 재시드(공통 난수), max_steps 는 t_limit 에서,
  요약에 unfinished·success_se·estimator. 재평가 도구 `tools/reeval_runs.py`(5b42ca8)로 9 run 의 고유 체크포인트 42 개 (B_bc3 는 seed 2 개라 체크포인트 × seed 52 회) + 휴리스틱 3 행을
  n 2048, 평가 seed 1000 (B_bc3 는 2000 도), unfinished 0 으로 다시 잼. 재학습 없음.

## 결과
**편향** [측정, `review/diag_summary.log`]

| 정책 · 장애물 | 옛 추정량 (먼저 끝난 ~500) | 첫 에피소드 (n 512) |
|---|---|---|
| B_bc3 최종 · mixed | 성공 0.408, 충돌 0.592, 정체 0.000, 경로 8.60 m | 성공 **0.555**, 충돌 0.412, 정체 0.033, 경로 9.81 m |
| B_bc3 최종 · none | 1.000 (경로 9.10 m) | **0.957** (정체 0.043) |
| 휴리스틱 · mixed | 0.417 | 0.557 |
| 휴리스틱 · none | 1.000 | 0.986 |

평가 풀 평균 경로 9.92 m — 옛 추정량은 짧은 경로·빠른 결말을 더 많이 셌다. 장애물이 있으면 성공을 **낮게**(충돌이 빨리 끝남),
없으면 **높게**(정체·긴 에피소드가 덜 잡힘) 쟀다 → 편향 방향이 결말 분포에 따라 뒤집힌다.

**재평가** [측정, `reeval.jsonl`, n 2048, unfinished 0]: B_bc3 BC 0.5381 → 갱신 15–105 0.5347–0.5425 → 최종 0.5317 (seed 2000:
0.5327 → 0.5332–0.5415 → 0.5327), 휴리스틱 0.5347/0.5317. B_bc1·B_bc2 0.538 → 0.009/0.012. 순수 PPO run 은 모두 0
(B_noobs1 정체 1.000, 휴리스틱 0.9927), C 0 → 0.0005. 재확인: 리뷰어 독립 추정량과 수정 도구의 10 (체크포인트, seed) 쌍이
1e-4 안에서 일치, 불일치 0 [측정, `recheck/crosscheck_reeval_vs_review.log`].

리뷰 차단 2 (env 종료 조건 시험 부재): 리뷰 변이 R2–R6(장애물 접촉 무시, 정체·시간 상한 제거, 1 스캔 STOP, 장애물 관측 지연 무시)이
53 passed 로 살아남았다 → 시험 5 개 추가 뒤 변이 10/10 잡힘 [측정, `fix/mut_fix_summary.log`]. 재확인 변이 15 개 중 X5(라운드마다
같은 seed 재사용) 1 개 생존.

## 해석
- **1차 물결 2D 의 학습 중 성공률 수치는 모두 편향 값이었다.** 결론("PPO 가 성공률을 올리지 못했다")은 편향을 없애도 그대로지만,
  수치(0.415·0.429, SE 0.022)는 틀렸다 — 편향 없는 값 0.53–0.54, SE 0.011.
- Isaac 1차 물결의 `eval_policy.py` 도 같은 꼴의 추정량(끝난 에피소드만)이었다 → 2b r7 이 `eval_unbiased.py` 로 고쳤다.
- 학습 롤아웃 성공률(train.jsonl)도 끝난 에피소드 비율이라 짧은 에피소드 쪽으로 치우친다 — 판정에 쓰지 않는다.

## 무엇을 바꾸게 했나
- 완성 계획 §4-3 "평가는 편향 없는 방식 (env 마다 첫 에피소드, 고정 평가 seed, unfinished 0)" 의 직접 근거.
- 2a·2b 모든 판정 평가가 이 추정량(n 2048, 평가 seed 4242)을 쓴다.

## 원자료
- 리뷰 `/home/CAPYI_AI/logs/agents/r2-sim2d/review/` (diag.py, diag*.json, diag*_summary.log, mut_*), 수정 `fix/` (reeval_all.log, mut_fix_*), 재확인 `recheck/`
- 재평가 결과 `/home/CAPYI_AI/runs/r2-sim2d/<run>/reeval.jsonl`, 도구 사본 `/home/CAPYI_AI/runs/r2-sim2d/reeval_src/`

## WAVE 보고와 원자료의 차이
- WAVE fix 의 재평가 수치는 원자료와 같다. 리뷰 증거 파일의 mtime 은 모두 09:21:16 (복사 시각)이라 리뷰 실행 시각은 기록되지 않았다.
- WAVE fix "9 개 run 의 체크포인트 47 개" — `reeval.jsonl` 9 개를 세면 고유 체크포인트 42, 체크포인트 × seed 52, 휴리스틱 포함 55 행이다.
  47 은 어느 셈법과도 맞지 않는다 (리뷰 반영으로 더함 — 처음엔 이 숫자를 그대로 옮겼다).

## 실행 목록에 등록한 run (`runs/b1-backfill.jsonl`, entry = 이 파일)

| run_id | 종류 | 시뮬 | 커밋 | 시드 | 표기 |
|---|---|---|---|---|---|
| `W1_r2_smoke_B1_reeval` | eval | 2d | 5b42ca8 | 1000 | [측정] |
| `W1_r2_B_main1_reeval` | eval | 2d | 5b42ca8 | 1000 | [측정] |
| `W1_r2_B_lag1_reeval` | eval | 2d | 5b42ca8 | 1000 | [측정] |
| `W1_r2_B_noobs1_reeval` | eval | 2d | 5b42ca8 | 1000 | [측정] |
| `W1_r2_C_scratch1_reeval` | eval | 2d | 5b42ca8 | 1000 | [측정] |
| `W1_r2_C_scratch2_reeval` | eval | 2d | 5b42ca8 | 1000 | [측정] |
| `W1_r2_B_bc1_reeval` | eval | 2d | 5b42ca8 | 1000 | [측정] |
| `W1_r2_B_bc2_reeval` | eval | 2d | 5b42ca8 | 1000 | [측정] |
| `W1_r2_B_bc3_reeval` | eval | 2d | 5b42ca8 | 1000,2000 | [측정] |
| `W1_r2_review_diag2_unbiased` | eval | 2d | 658aa73 | 1000,2000 | [측정] |
