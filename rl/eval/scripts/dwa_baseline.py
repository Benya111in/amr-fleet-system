#!/usr/bin/env python3
"""
비교군 A (DWA) Gazebo 기준선 표 생성 — rl/eval/baseline/{dwa_baseline.md, dwa_gz_s08_episodes.csv}.

사용:
  python3 rl/eval/scripts/dwa_baseline.py [--log-root <logs>] [--out-dir rl/eval/baseline]

로그는 읽기만 한다. 출력 표의 모든 수치에 로그 경로·태그·HEAD.txt 상태를 붙인다.
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from amr_rl_eval import baseline as BL  # noqa: E402
from amr_rl_eval import schema  # noqa: E402


def _f(x, nd=2):
    return '—' if x is None else f'{x:.{nd}f}'


def _head(r):
    return f'`{r["head"]}`' if r.get('head') else '—'


def render(s08, exc, s07, root):
    """세 요약 → 마크다운."""
    p = s08['pooled']
    cr = p['contact_rate']
    runs = s08['runs']
    lines = []
    w = lines.append
    w('# 비교군 A (팀 DWA) Gazebo 기준선')
    w('')
    w('> 생성: `python3 rl/eval/scripts/dwa_baseline.py` — 손으로 고치지 말 것. 로그 루트 '
      f'`{root}` (읽기 전용, 이하 `logs/`).')
    w('> 표기: **[측정]** = 아래 로그 파일에서 이 스크립트가 다시 계산한 값, **[계산]** = 그 값으로 낸 통계,')
    w('> **미측정** = 로그에 그 지표를 계산할 자료가 없다. 모든 행에 출처 열이 있다.')
    w('')
    w('## 0. 불리한 사실 먼저')
    w('')
    w('1. **safety node 개입 횟수·시간은 미측정이다.** 시나리오 08 하네스는 safety node 의 입력'
      '(`cmd_vel_smoothed`)·출력(`cmd_vel`)을 구독하지만 시행별로 저장하지 않고 접촉 순간'
      '(`contacts.csv`) 에만 쓴다. `safety/zone` 은 구독하지 않는다 (`test_08_dynamic_obstacles.py:411-413`). '
      '그래서 기준선에 개입 지표가 없다 — PROTOCOL.md §5 가 새 실행부터 이 열을 필수로 요구한다.')
    w(f'2. **{p["n"]} 시행은 독립 시나리오가 아니다.** {BL.ACTOR_SEED_NOTE}. 같은 월드·같은 actor 루프에서 '
      f'A↔B 왕복 30 회를 {p["runs"]} 번 돌렸다. 접촉 0/{p["n"]} 의 CP 95 % 상한은 시행 독립을 가정하면 '
      f'**{100 * cr["cp_upper"]:.2f} %**, 실행 간 위상이 재현돼 실효 표본이 30 이면 '
      f'**{100 * p["contact_ub_if_30_effective"]:.2f} %** 다 (이론편).')
    failed = p['harness_failed_runs']
    w(f'3. **{len(failed)} / {p["runs"]} 실행이 실행 당시 하네스 판정에서 실패했다** '
      f'({", ".join("`" + t + "`" for t in failed)}). 실패 검사는 전부 '
      '`return to path (worst of active/original plan)` > 5 s 다. `performance.md` §2 는 이 실행들을 '
      f'활성 계획 기준 복귀(실행별 최대 {min(r["return_max"] for r in runs):.2f}~'
      f'{max(r["return_max"] for r in runs):.2f} s)로 다시 판정해 통과로 적는다 — 판정 기준이 실행 뒤에 '
      '바뀐 셈이므로 RL 비교에서는 **두 기준을 다 보고**하고, 새 비교의 기준은 실행 전에 PROTOCOL.md 로 고정한다.')
    exc_c = [r for r in exc if r['contacts'] > 0]
    same = [r for r in exc_c if r['head'] in {q['head'] for q in runs}]
    tail = ''
    if same:
        n_all = p['n'] + sum(r['n'] for r in same)
        c_all = p['contact_trials'] + sum(r['contacts'] for r in same)
        tail = (f' 커밋이 240 묶음과 같은 {", ".join("`" + r["tag"] + "`" for r in same)} 를 넣으면 '
                f'접촉 {c_all}/{n_all} 이다.')
    w(f'4. **{p["n"]} 에 넣지 않은 08 실행 {len(exc_c)} 개에 접촉이 있다** — 아래 §4.{tail}')
    mx = ' 대 '.join(f'{100 * r["cte"]["직선"]["max"]:.1f} cm (`{r["tag"]}`)' for r in s07
                    if '직선' in r['cte'])
    w(f'5. **CTE 는 실행 {len(s07)} 개뿐이고 실행 사이 차이가 크다** (§3: 직선 최대 {mx}). '
      '실행 간 변동을 잴 반복이 없다.')
    lo_l = min(r['loadavg_start'] for r in runs)
    hi_l = max(r['loadavg_end'] for r in runs)
    w(f'6. 공유 서버 측정이다 — 시작·끝 1 분 loadavg {lo_l:.1f}~{hi_l:.1f} (32 코어). 시간 지표는 '
      '**공유 부하 하 예비값**이다.')
    w('')
    w('## 1. 실행별 출처 (시나리오 08 동적 장애물, 240 시행 묶음)')
    w('')
    w('| 태그 | HEAD.txt | 커밋 | 미커밋 | HEAD.txt→시작 [s] | loadavg 시작/끝 | 시행 | 도달 | 접촉 | '
      '하네스 판정(당시) | avoidance.csv sha256[:12] |')
    w('| --- | --- | --- | --- | ---: | --- | ---: | ---: | ---: | --- | --- |')
    shas = dict(s08['sources'])
    for r in runs:
        src = os.path.join(root, r['tag'], BL.SCEN_08, 'avoidance.csv')
        if r['uncommitted'] is None:
            unc = '없음'
        else:
            unc = f'{len(r["uncommitted"])} 줄'
        verdict = '통과' if r['harness_passed'] else '실패: ' + '; '.join(r['failed_checks'])
        w(f'| `{r["tag"]}` | {r["head_status"]} | {_head(r)} | {unc} | '
          f'{_f(r["head_lag_s"], 0)} | {r["loadavg_start"]}/{r["loadavg_end"]} | {r["n"]} | '
          f'{r["reached"]} | {r["contacts"]} | {verdict} | `{shas[src]}` |')
    w('')
    w('## 2. 시나리오 08 묶음 지표 — 비교군 A 의 Gazebo 기준선')
    w('')
    w(f'출처: `logs/<태그>/{BL.SCEN_08}/avoidance.csv` × 8 (§1 의 태그). 시행 단위 열 정의는 '
      '`tests/integration/test_08_dynamic_obstacles.py:9-24`.')
    w('')
    w('| 지표 | 값 | 표기 | 정의·출처 열 |')
    w('| --- | --- | --- | --- |')
    w(f'| 시행 | {p["n"]} (실행 {p["runs"]} × 30) | [측정] | 행 수 |')
    w(f'| 목표 도달 | {p["reached"]}/{p["n"]} (CP 95 % 하한 {100 * p["reached_cp_lower"]:.2f} %) | '
      '[측정]/[계산] | `reached` |')
    w(f'| 접촉 시행 | {p["contact_trials"]}/{p["n"]} | [측정] | `contacts` > 0 |')
    w(f'| 접촉률 95 % 상한 CP / Wilson / Jeffreys / 3/n | {100 * cr["cp_upper"]:.2f} / '
      f'{100 * cr["wilson_upper"]:.2f} / {100 * cr["jeffreys_upper"]:.2f} / '
      f'{100 * cr["rule3"]:.2f} % (독립 가정) | [계산] | `amr_rl_eval.binomial.rate_summary` |')
    w(f'| 접촉률 95 % 상한 (실효 표본 30) | {100 * p["contact_ub_if_30_effective"]:.2f} % | [계산] | '
      '실행 간 위상 재현 시 |')
    w(f'| 주행 시간 중앙 / IQM / p90 / 최대 | {p["time_median"]:.2f} / {p["time_iqm"]:.2f} / '
      f'{p["time_p90"]:.2f} / {p["time_max"]:.2f} s | [측정] | `time_s` (A↔B 한 방향, 약 9 m) |')
    w(f'| 최소 이격 최소 / p5 / 중앙 | {p["min_clearance_min"]:.3f} / {p["min_clearance_p5"]:.3f} / '
      f'{p["min_clearance_median"]:.3f} m | [측정] | `min_distance_m` (풋프린트↔장애물 부호 거리) |')
    w(f'| 최소 TTC 최소 / p5 | {p["min_ttc_min"]:.2f} / {p["min_ttc_p5"]:.2f} s | [측정] | '
      '`min_ttc_s` (지면 진실 등속) |')
    w(f'| 최대 이탈 (활성 계획) 최대 / p95 | {p["max_dev_max"]:.3f} / {p["max_dev_p95"]:.3f} m | '
      '[측정] | `max_deviation_m` (명세 ≤ 1 m) |')
    w(f'| 복귀 시간 (활성 계획) 최대 | {p["return_max"]:.2f} s | [측정] | `return_s` (명세 ≤ 5 s) |')
    w(f'| 첫 계획 기준 이탈 > 1 m 시행 | {p["dev_first_over_1m"]}/{p["n"]} | [측정] | `max_dev_first_m` |')
    w(f'| 첫 계획 기준 복귀 > 5 s 시행 | {p["return_first_over_5s"]}/{p["n"]} | [측정] | '
      '`return_first_s` |')
    w(f'| 재계획 시행 (`n_plans` > 1) | {p["replanned"]}/{p["n"]} '
      f'({100 * p["replanned"] / p["n"]:.1f} %) | [측정] | `n_plans` |')
    w('| safety node 개입 횟수·시간 | — | **미측정** | 시행별 기록 없음 (§0-1) |')
    w('| CTE (동적 장애물 시나리오) | — | **미측정** | 08 은 CTE 를 기록하지 않는다 — §3 의 07 참조 |')
    w('| 선·각 저크 | — | **미측정** | 시행별 명령 이력은 접촉 시행만 저장 |')
    w('')
    w('실행별 범위 (같은 열, 실행 단위 최대):')
    w('')
    w('| 태그 | 주행 시간 중앙/최대 [s] | 최대 이탈 [m] | 복귀 최대 [s] | 첫 계획 복귀 최대 [s] | '
      '최소 이격 [m] | 최소 TTC [s] | 재계획 시행 |')
    w('| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |')
    for r in runs:
        w(f'| `{r["tag"]}` | {r["time_median"]:.2f}/{r["time_max"]:.2f} | {r["max_dev"]:.3f} | '
          f'{r["return_max"]:.2f} | {r["return_first_max"]:.2f} | {r["min_clearance"]:.3f} | '
          f'{r["min_ttc"]:.2f} | {r["replanned"]} |')
    w('')
    w('## 3. 시나리오 07 경로 추종 (CTE·주행 시간) — 실행 2 개')
    w('')
    w('| 태그 | HEAD.txt | 커밋 | 구간 | n | 평균 / p95 / 최대 \\|CTE\\| [cm] | result.json 과 일치 | '
      'cte_amr_01.csv sha256[:12] |')
    w('| --- | --- | --- | --- | ---: | --- | --- | --- |')
    for r in s07:
        for seg, v in sorted(r['cte'].items()):
            w(f'| `{r["tag"]}` | {r["head_status"]} | {_head(r)} | {seg} | {v["n"]} | '
              f'{100 * v["mean"]:.2f} / {100 * v["p95"]:.2f} / {100 * v["max"]:.2f} | '
              f'{"예" if r["cte_matches_result_json"] else "**아니오**"} | `{r["source_sha"]}` |')
    w('')
    w('| 태그 | 목표별 주행 시간 [s] (목표 순서 (15,0)·(10,6)·(−15,6)·(−10,−6)·(0,0)) | '
      '주행 시간 예측 오차 최대 / 평균 [%] |')
    w('| --- | --- | --- |')
    for r in s07:
        times = ' · '.join(f'{t:.1f}' for _, t, _ in r['goals'])
        eta = r['eta']
        eta_s = f'{eta["max_err_pct"]:.2f} / {eta["mean_err_pct"]:.2f}' if eta else '기록 없음'
        w(f'| `{r["tag"]}` | {times} | {eta_s} |')
    w('')
    w('출처: `logs/<태그>/07_path_tracking_cte/{cte_amr_01.csv, result.json}`. CTE 는 |cte| 를 '
      '`segment` 열(직선/곡선)로 묶어 다시 계산했다. 명세 기준(평균): 직선 ≤ 5 cm, 곡선 ≤ 10 cm.')
    w('')
    w('## 4. 240 에 넣지 않은 시나리오 08 실행')
    w('')
    w('| 태그 | HEAD.txt | 커밋 | 시행 | 접촉 | 제외 사유 | 하네스 판정(당시) |')
    w('| --- | --- | --- | ---: | ---: | --- | --- |')
    for r in exc:
        verdict = '통과' if r['harness_passed'] else '실패: ' + '; '.join(r['failed_checks'])
        w(f'| `{r["tag"]}` | {r["head_status"]} | {_head(r)} | {r["n"]} | {r["contacts"]} | '
          f'{r["reason"]} | {verdict} |')
    w('')
    w('## 5. 결과 CSV 로 옮긴 기준선')
    w('')
    w(f'`dwa_gz_s08_episodes.csv` — §2 의 {p["n"]} 시행을 결과 스키마 `{schema.SCHEMA_VERSION}` 로 옮겼다. '
      '`scenario_seed` = 왕복 순번(0~29) 이고 **독립 시나리오 시드가 아니다** (§0-2). 학습 비교군의 '
      '새 Gazebo 평가는 이 묶음이 아니라 PROTOCOL.md §3 의 새 시드 묶음과 짝짓는다.')
    w('')
    return '\n'.join(lines) + '\n'


def main(argv=None):
    """명령행 진입점."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--log-root', default=BL.DEFAULT_LOG_ROOT)
    ap.add_argument('--out-dir', default=os.path.join(os.path.dirname(HERE), 'baseline'))
    a = ap.parse_args(argv)
    s08 = BL.summarize_08(a.log_root)
    exc = BL.summarize_excluded_08(a.log_root)
    s07 = BL.summarize_07(a.log_root)
    os.makedirs(a.out_dir, exist_ok=True)
    md = os.path.join(a.out_dir, 'dwa_baseline.md')
    with open(md, 'w', encoding='utf-8') as f:
        f.write(render(s08, exc, s07, a.log_root))
    csv_path = os.path.join(a.out_dir, 'dwa_gz_s08_episodes.csv')
    schema.write_csv(csv_path, s08['episodes'])
    schema.read_csv(csv_path)          # 계약 검사 — 어긋나면 여기서 실패
    print(md)
    print(csv_path, len(s08['episodes']), '행')
    return 0


if __name__ == '__main__':
    sys.exit(main())
