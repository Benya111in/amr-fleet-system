# rl/eval — 보너스 1 평가 도구 (학습 정책 대 DWA)

ROS 패키지가 아니다 (`COLCON_IGNORE`). 순수 Python(numpy·scipy, rliable 은 선택)이라 `amr-ai/rl:0.3` 이나
아무 Python 3.10 환경에서 돈다.

| 경로 | 내용 |
| --- | --- |
| [`PROTOCOL.md`](PROTOCOL.md) | **사전 등록 평가 프로토콜** — 시나리오·시드·결과 CSV 열·판정 규칙. 결과를 내기 전에 읽는다 |
| [`baseline/dwa_baseline.md`](baseline/dwa_baseline.md) | 비교군 A(DWA) Gazebo 기준선 — 측정 로그에서 다시 계산, 모든 수치에 출처 |
| `baseline/dwa_gz_s08_episodes.csv` | 같은 기준선 240 시행을 결과 스키마로 옮긴 것 |
| `amr_rl_eval/` | 통계 라이브러리 (`binomial` `noninferiority` `sequential` `aggregate` `paired` `multiplicity` `srcc` `cvar` `planning`), 결과 스키마 (`schema`), 판정 (`report`), 그림 (`figures`), 합성 데이터 (`synth`), 기준선 추출 (`baseline`) |
| `scripts/evaluate.py` | 결과 CSV → 판정표(`report.md`)·`report.json`·그림 4 장 |
| `scripts/dwa_baseline.py` | 측정 로그 → `baseline/` 재생성 |
| `tests/` | pytest — 이론편 수치 재현·리서치 calc 스크립트 교차 확인·판정 분기 |

결과를 붓는 쪽(r1·r2·r3)이 할 일:

1. 에피소드마다 한 행, `PROTOCOL.md` §5 의 열로 CSV 를 쓴다. `amr_rl_eval.schema.read_csv(<파일>)` 가 통과해야 한다.
2. `python3 rl/eval/scripts/evaluate.py <파일들> --out-dir <출력>` 으로 판정표를 만든다.
3. 판정 기준(`report.PREREG`)은 명령행으로 바꿀 수 없다. 바꿔야 하면 실행 전에 PROTOCOL 개정을 커밋한다.

```bash
docker run --rm --network none -v <worktree>:/ws -w /ws/rl/eval --entrypoint bash amr-ai/rl:0.3 \
  -lc 'python3 -m pytest -q tests'
```
