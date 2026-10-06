# 보너스 1 실험 노트 — 쓰는 법

이 노트는 **나중에 다시 조사할 때** 쓰려고 남긴다. 결과만이 아니라 "왜 그 실험을 했고, 무엇을 예상했고,
무엇이 나왔고, 그래서 무엇을 바꿨나"를 남긴다. 실패한 실험이 더 중요하다 — 같은 실패를 되풀이하지 않게 한다.

## 세 가지 기록

| 기록 | 위치 | 누가 | 언제 |
|---|---|---|---|
| **노트 항목** | `entries/<날짜>_<시각>_<에이전트>_<주제>.md` | 실험한 에이전트 | 실험 **시작 전**(질문·가설·설정)에 만들고, 끝나면 결과·해석·다음을 채운다 |
| **실행 목록** | `runs.jsonl` (한 줄 = 실행 한 건) | 실행한 에이전트 | 학습·평가·탐침·Gazebo 실행이 **끝날 때마다** (실패한 실행도) |
| **색인** | `index.md` | 도구가 생성 | 단계가 끝날 때 오케스트레이터가 다시 만든다 |

도구: `python3 rl/tools/notebook.py {new-entry|register|index|check}` (표준 라이브러리만, 컨테이너 밖에서도 돈다).

```bash
python3 rl/tools/notebook.py new-entry --agent b1-isaac --stage B1 --slug ppo-10seeds --title "Isaac v1.2 PPO 10 시드"
python3 rl/tools/notebook.py register --file /home/CAPYI_AI/runs/bonus1/B1/ppo_s3/registry.json
python3 rl/tools/notebook.py check --paths
```

## 노트 항목 규칙

- 틀(`new-entry`)의 절을 지운다면 이유를 적는다: 질문 / 가설과 예측 / 설정 / 결과 / 해석 / 다음.
- **가설과 판정 규칙은 결과를 보기 전에 쓰고 커밋한다** (2b 리뷰 교훈: 판정 기준이 결과 뒤에 커밋돼 사전 등록을 확인할 수 없었다).
- 수치마다 표기: **[측정]** (명령·로그 경로 있음) / **[계산]** / **미측정** / **[문서주장]**. 공유 서버 부하 하 수치는 "예비값"과 loadavg.
- 시드별 값을 모두 적고 실패 시드 수를 함께 쓴다 (2a 교훈: 시드 2 개로 낸 결론이 독립 시드에서 뒤집혔다).
- 원자료는 저장소에 넣지 않는다 — `/home/CAPYI_AI/runs/bonus1/<단계>/<run_id>/` 에 두고 경로를 적는다.
- 다른 기록(1·2차 물결 결과, 리서치 보고서)을 근거로 쓰면 경로를 적는다.

## 실행 목록 필수 필드

`run_id, stage, agent, kind(train|eval|probe|gazebo|test), sim(2d|isaac|gazebo|offline), commit, dirty, config, seed,
started, ended, raw_dir(절대 경로), result(dict — 판정 지표와 n), label, load`. 하나라도 비면 `register` 가 거부한다.

## 이전 기록 (이 노트 이전)

| 기록 | 위치 |
|---|---|
| 1차 물결 RL 과제 결과 (r0–r4) | `/home/CAPYI_AI/logs/agents/{r0-contract,r1-isaac,r2-sim2d,r3-controller,r4-eval}/WAVE1_RESULT.md` |
| 2a 보상 수정 | `/home/CAPYI_AI/logs/agents/r5-reward/WAVE2A_RESULT.md`, `../reward_v12.md` |
| 2b 동적 장애물·Isaac v1.2 | `/home/CAPYI_AI/logs/agents/{r6-dyn,r7-isaac12}/WAVE2B_RESULT.md`, `../dyn_obstacles.md` |
| 리서치 보고서 | `/home/CAPYI_AI/research/reports/` (실무편·이론편·실측 기반 후속 기술 조사) |
| 원자료 | `/home/CAPYI_AI/runs/{r1-isaac,r2-sim2d,r5-reward,r6-dyn,r7-isaac12}/` |
