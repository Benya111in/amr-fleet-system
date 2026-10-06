# A4 소급 기록 — 방법, 판정 규칙, 원자료와 다르게 적힌 보고 수치

- 시각: 2026-10-06 18:56 KST · 에이전트: b1-backfill · 단계: A4
- 코드: `a7da8a2` (브랜치 `feature/b1-backfill`)
- 관련 실행 id: `runs/b1-backfill.jsonl` 의 모든 run_id (이 작업은 새 학습·평가를 하지 않는다 — 원자료 읽기와 산술만)
- **정직성 메모**: 이 항목의 질문·판정 규칙은 원자료 일부(r1·r2·r3·r5·r6·r7 의 curve/eval/config)를 이미 훑어본 뒤에
  썼다. 그래서 엄밀한 사전 등록이 아니다. 규칙은 "보고 수치와 원자료가 다른가"를 기계적으로 가르는 것이라 결과에
  맞춰 고를 여지가 작지만, 그 점을 밝혀 둔다.

## 질문
1차 물결(r0–r4)·2a(r5)·2b(r6·r7)의 RL 실험을 원자료(eval.jsonl·eval.json·curve.jsonl·config.json·HEAD 파일·로그)로
다시 읽으면, WAVE 보고(`/home/CAPYI_AI/logs/agents/*/WAVE*_RESULT.md`)에 적힌 수치와 얼마나 어긋나는가 — 그리고
"무엇을 시도했고 무엇이 왜 실패했나"를 한 곳(노트 항목 + 실행 목록)에서 찾을 수 있게 정리할 수 있는가.

## 가설과 예측
- 대부분의 핵심 수치(시드별 성공률, n, 실패 시드 수)는 WAVE 보고와 같다 — 각 물결에 적대적 리뷰와 재확인이 있었다.
- 어긋남은 반올림(0.0005 를 0.000 으로), 범위 표기(최솟값·최댓값 일부 누락), 스텝 수 표기(BC·critic 예열 포함 여부),
  추정량 차이(편향 있는 평가)에서 나올 것이다.
- 커밋 출처가 없는 run 이 1차 물결 초기에 몇 개 있을 것이다 (리뷰가 이미 B_main1 등을 지적했다).
- 가설을 버리는 조건: 시드별 성공률·실패 시드 수 같은 **결론 수준 수치**가 원자료와 0.01 넘게 어긋나는 사례가 나오면
  "WAVE 보고는 대체로 정확하다"를 버린다.

## 설정
- 대상 원자료: `/home/CAPYI_AI/runs/{r1-isaac,r2-sim2d,r3-controller,r5-reward,r6-dyn,r7-isaac12}/`,
  `/home/CAPYI_AI/logs/agents/{r0-contract,r1-isaac,r2-sim2d,r3-controller,r4-eval,r5-reward,r6-dyn,r7-isaac12}/`,
  DWA 기준선 Gazebo 로그 (원본 `/tmp/claude-0/-home-CAPYI/c8bfdadc-8698-4d09-aac1-e3c1224c9abc/scratchpad/wt-integ/logs/`
  읽기 전용 — 이 경로는 다른 세션의 scratchpad 라 지워질 수 있어 `/home/CAPYI_AI/runs/bonus1/b1-backfill/archive/` 에
  사본을 두고 sha256 목록을 남긴다).
- 읽기 스크립트: `/home/CAPYI_AI/runs/bonus1/b1-backfill/tmp/` (저장소에 넣지 않는다). 컨테이너·GPU 사용 없음
  (호스트 python3 표준 라이브러리 + 원자료 JSON/CSV 읽기). 예외: `.npz`·`.pt` 는 읽지 않는다 — 그 안의 수치는 리뷰 기록을 인용하고 [문서주장] 으로 적는다.
- 판정 규칙 (어긋남 분류):
  - **결론 수준**: 시드별 성공률·실패 시드 수·n·판정 대상이 0.01 넘게 다르거나 다른 run 을 가리킴.
  - **수치 표기**: 반올림·범위·스텝 수 표기가 원자료 값과 다름 (결론 불변).
  - **출처**: 커밋·미커밋 여부·시각을 원자료로 확인할 수 없음.
  - 원자료 값이 이기고, 차이는 이 항목 「결과」 표에 보고 위치와 함께 적는다.
- 실행 목록 등록 규칙: 원자료 디렉터리가 있는 모든 학습·평가 run (+ 노트 항목이 인용하는 탐침·Gazebo·스모크). 커밋은
  HEAD 파일 → config `git_commit` 순서로 읽고, 없으면 `unknown-<이유>` 로 쓰고 config 필드에 그 사실을 적는다.
  시각은 HEAD/start/end 파일, 없으면 파일 mtime (KST) 이다 — mtime 으로 정한 것은 config 필드에 "mtime 추정" 으로 적는다.

## 결과
(작업 끝에 채운다)

## 해석
(작업 끝에 채운다)

## 다음
(작업 끝에 채운다)
