"""
통합 테스트 결과 집계 (rclpy 비의존, scripts/run_integration.sh 가 마지막에 부른다).

    python3 -m amr_itest.report --log-dir logs/itest [--scenarios 01_sensor_topics ...]
    python3 -m amr_itest.report --list            # 시나리오 목록 (번호 id 백엔드 상태)
    python3 -m amr_itest.report --readme-table    # tests/integration/README.md 표

입력  logs/itest/<id>/junit.xml (러너), logs/itest/<id>/result.json (하네스),
      logs/itest/unit/junit.xml (하네스 단위 테스트, 있으면)
출력  logs/itest/junit.xml     모든 스위트를 합친 JUnit (CI 리포터용)
      logs/itest/summary.json  시나리오별 상태·사유·판정 항목·측정 요약·소요 시간·호스트 부하
      logs/itest/summary.md    사람이 읽는 표
종료 코드  0: 실패/에러/누락 없음, 그리고 실패로 세는 skip 없음
          1: 하나라도 failed / error / missing, 또는 실패로 세는 skip·partial
skip 정책 (skip 은 통과가 아니다)
  기본          구현된 시나리오가 needs 패키지가 모두 설치된 워크스페이스에서 skip(전부) 또는 partial(일부 skip)
                이면 실패로 센다 — GPU 없음·백엔드/구성 불가·대역 금지처럼 환경 때문에 못 돈 것도 포함.
                needs 패키지가 설치되지 않았으면(부분 워크스페이스) skip 으로만 남는다.
  --fail-on-skip  모든 skip·partial 을 실패로 (needs 설치 여부 무관)
  --allow-skip    skip·partial 을 실패로 세지 않는다 (하네스 개발용 — 집계에 그대로 표시)
실행·skip 수는 항상 출력한다 (시나리오 수와 테스트 케이스 수).
"""

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Sequence

from amr_itest import catalog
from amr_itest import junit
from amr_itest import requirements as req
from amr_itest import results
from amr_itest.config import repo_root

FAIL_ON_SKIP_DEFAULT = 'default'   # needs 가 설치된 구현 시나리오의 skip 만 실패
FAIL_ON_SKIP_ALL = 'all'
FAIL_ON_SKIP_NONE = 'none'


SUMMARY_JSON = 'summary.json'
SUMMARY_MD = 'summary.md'
AGGREGATE_XML = 'junit.xml'
UNIT_DIR = 'unit'


def missing_needs(scenario) -> List[str]:
    """시나리오 needs 중 설치되지 않은 패키지."""
    return [p for p in scenario.needs if not req.has_package(p)]


def scenario_entry(log_root: Path, scenario) -> Dict[str, Any]:
    """시나리오 하나의 요약 (JUnit 상태 + result.json)."""
    d = log_root / scenario.id
    xml = d / 'junit.xml'
    status = junit.status_of(xml if xml.is_file() else None)
    res = results.load(d / results.RESULT_FILE)
    reason = res.get('skip_reason') or (junit.skip_message(xml) if status == junit.SKIPPED
                                        else '')
    checks = res.get('checks', [])
    failed = [c for c in checks if not c.get('passed')]
    if status == junit.PARTIAL and not reason:
        reason = junit.skip_message(xml)
    return {
        'id': scenario.id, 'number': scenario.number, 'title': scenario.title,
        'status': status, 'skip_reason': reason,
        'cases': junit.case_counts(xml if xml.is_file() else None),
        'missing_needs': missing_needs(scenario),
        'implemented': scenario.implemented, 'backend': res.get('backend'),
        'profile': res.get('measurements', {}).get('profile'),
        'components': res.get('components', {}),
        'threshold': scenario.threshold,
        'checks': checks, 'failed_checks': failed,
        'measurements': res.get('measurements', {}),
        'duration_s': res.get('duration_s'),
        'host': res.get('host', {}),
        'junit': str(xml) if xml.is_file() else None,
    }


def collect(log_root: Path, ids: Optional[Sequence[str]] = None) -> Dict[str, Any]:
    """선택한 시나리오(None 이면 로그 디렉토리가 있는 것 전부, [] 면 없음)의 요약 + 합계."""
    log_root = Path(log_root)
    if ids is not None:
        chosen = [catalog.by_id(i) for i in ids]
    else:
        chosen = [s for s in catalog.SCENARIOS if (log_root / s.id).is_dir()]
    entries = [scenario_entry(log_root, s) for s in chosen]
    unit_xml = log_root / UNIT_DIR / 'junit.xml'
    unit_status = junit.status_of(unit_xml) if unit_xml.is_file() else None
    counts: Dict[str, int] = {}
    for e in entries:
        counts[e['status']] = counts.get(e['status'], 0) + 1
    return {'log_root': str(log_root), 'scenarios': entries, 'counts': counts,
            'unit': {'status': unit_status, 'junit': str(unit_xml) if unit_status else None},
            'host_now': results.host_load()}


def skip_is_failure(entry: Dict[str, Any], policy: str = FAIL_ON_SKIP_DEFAULT) -> bool:
    """skip·partial 을 실패로 셀까 (모듈 설명의 skip 정책)."""
    if entry['status'] not in (junit.SKIPPED, junit.PARTIAL):
        return False
    if policy == FAIL_ON_SKIP_ALL:
        return True
    if policy == FAIL_ON_SKIP_NONE:
        return False
    return bool(entry['implemented']) and not entry['missing_needs']


def counts_line(summary: Dict[str, Any], policy: str = FAIL_ON_SKIP_DEFAULT) -> str:
    """실행·skip 수 한 줄 (시나리오 수 + 테스트 케이스 수)."""
    entries = summary['scenarios']
    executed = [e for e in entries if e['status'] not in (junit.SKIPPED, junit.MISSING)]
    skipped = [e for e in entries if e['status'] == junit.SKIPPED]
    partial = [e for e in entries if e['status'] == junit.PARTIAL]
    as_fail = [e['id'] for e in entries if skip_is_failure(e, policy)]
    cases = {k: sum(e['cases'][k] for e in entries) for k in ('total', 'executed', 'skipped')}
    return (f"시나리오 {len(entries)}: 실행 {len(executed)} (그중 일부 skip {len(partial)}), "
            f"전부 skip {len(skipped)}, 실패로 센 skip {len(as_fail)}"
            + (f" {as_fail}" if as_fail else '')
            + f" | 테스트 케이스 {cases['total']}: 실행 {cases['executed']}, skip {cases['skipped']}")


def _short(entry: Dict[str, Any], limit: int = 160) -> str:
    """표의 비고 칸: skip 사유 / 실패 판정 / 통과 판정 요약."""
    if entry['status'] == junit.SKIPPED:
        text = entry['skip_reason']
    elif entry['status'] == junit.PARTIAL and not entry['failed_checks']:
        text = f"일부 skip: {entry['skip_reason']}"
    elif entry['failed_checks']:
        text = '; '.join(f"FAIL {c['name']}={c['value']}" for c in entry['failed_checks'])
    else:
        text = '; '.join(f"{c['name']}={c['value']}" for c in entry['checks'][:4])
    text = text.replace('|', '/').replace('\n', ' ')
    return text if len(text) <= limit else text[:limit - 1] + '…'


def to_markdown(summary: Dict[str, Any]) -> str:
    lines = ['# 통합 테스트 결과', '',
             f"로그 루트: `{summary['log_root']}`  ",
             f"합계: {', '.join(f'{k} {v}' for k, v in sorted(summary['counts'].items()))}  ",
             f"{counts_line(summary, summary.get('skip_policy', FAIL_ON_SKIP_DEFAULT))}  ",
             f"하네스 단위 테스트: {summary['unit']['status'] or '실행 안 함'}  ",
             f"호스트 부하(집계 시점): {summary['host_now']['loadavg']} / "
             f"{summary['host_now']['cpus']} CPU", '',
             '| # | 시나리오 | 상태 | 구성 / 백엔드 | 케이스 실행/skip | 소요 [s] | 비고 |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    policy = summary.get('skip_policy', FAIL_ON_SKIP_DEFAULT)
    for e in summary['scenarios']:
        status = e['status'] + (' (실패로 셈)' if skip_is_failure(e, policy) else '')
        lines.append(f"| {e['number']} | {e['title']} | {status} | "
                     f"{e.get('profile') or '-'} / {e['backend'] or '-'} | "
                     f"{e['cases']['executed']}/{e['cases']['skipped']} | "
                     f"{e['duration_s'] if e['duration_s'] is not None else '-'} | "
                     f"{_short(e)} |")
    lines.append('')
    return '\n'.join(lines)


def write(summary: Dict[str, Any], log_root: Path) -> Dict[str, Any]:
    """summary.json / summary.md / 합친 junit.xml 을 쓰고 JUnit 합계를 돌려준다."""
    log_root = Path(log_root)
    log_root.mkdir(parents=True, exist_ok=True)
    xmls = [Path(e['junit']) for e in summary['scenarios'] if e['junit']]
    if summary['unit']['junit']:
        xmls.insert(0, Path(summary['unit']['junit']))
    totals = junit.aggregate(xmls, log_root / AGGREGATE_XML)
    summary['junit_totals'] = totals
    (log_root / SUMMARY_JSON).write_text(
        json.dumps(results.jsonable(summary), ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    (log_root / SUMMARY_MD).write_text(to_markdown(summary), encoding='utf-8')
    return totals


def exit_code(summary: Dict[str, Any], policy: str = FAIL_ON_SKIP_DEFAULT) -> int:
    bad = {junit.FAILED, junit.ERROR, junit.MISSING}
    if any(e['status'] in bad for e in summary['scenarios']):
        return 1
    if any(skip_is_failure(e, policy) for e in summary['scenarios']):
        return 1
    if summary['unit']['status'] in (junit.FAILED, junit.ERROR):
        return 1
    return 0


def list_text() -> str:
    rows = []
    for s in catalog.SCENARIOS:
        state = 'implemented' if s.implemented else 'skeleton'
        if s.long_running:
            state += ',long'
        rows.append(f'{s.number:>2}  {s.id:<24} {",".join(s.backends):<18} {state:<18} {s.title}')
    return '\n'.join(rows)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='amr_itest.report', description=__doc__.split('\n')[1])
    p.add_argument('--log-dir', default=str(repo_root() / 'logs' / 'itest'))
    p.add_argument('--scenarios', nargs='*', default=None, help='집계할 시나리오 id (기본: 전부)')
    p.add_argument('--fail-on-skip', action='store_true',
                   help='모든 skip·partial 을 실패로 셈 (needs 설치 여부 무관)')
    p.add_argument('--allow-skip', action='store_true',
                   help='skip·partial 을 실패로 세지 않음 (기본: needs 가 설치된 구현 시나리오의 skip 은 실패)')
    p.add_argument('--list', action='store_true', help='시나리오 목록만 출력')
    p.add_argument('--readme-table', action='store_true', help='README 시나리오 표 출력')
    p.add_argument('--resolve', nargs='*', default=None,
                   help='시나리오 지정(번호/id/slug)을 id 로 바꿔 한 줄씩 출력 (셸 스크립트용)')
    p.add_argument('--default-set', action='store_true',
                   help='기본 실행 대상 id (장시간 시나리오 제외) 를 한 줄씩 출력')
    p.add_argument('--synthetic', nargs=3, metavar=('ID', 'STATUS', 'MESSAGE'),
                   help='launch_test 가 JUnit 을 못 남긴 시나리오(시간 초과·크래시)의 JUnit 을 대신 쓴다')
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list:
        print(list_text())
        return 0
    if args.readme_table:
        print(catalog.table_markdown(), end='')
        return 0
    if args.resolve is not None:
        try:
            for key in args.resolve:
                print(catalog.by_id(key).id)
        except KeyError as exc:
            print(f'알 수 없는 시나리오: {exc}', file=sys.stderr)
            return 2
        return 0
    if args.default_set:
        print('\n'.join(s.id for s in catalog.SCENARIOS if not s.long_running))
        return 0
    log_root = Path(args.log_dir)
    if args.synthetic:
        sid, status, message = args.synthetic
        junit.synthetic(log_root / sid / 'junit.xml', sid, 'launch_test', status, message)
        return 0
    if args.fail_on_skip and args.allow_skip:
        print('--fail-on-skip 과 --allow-skip 은 함께 쓸 수 없다', file=sys.stderr)
        return 2
    policy = (FAIL_ON_SKIP_ALL if args.fail_on_skip else
              FAIL_ON_SKIP_NONE if args.allow_skip else FAIL_ON_SKIP_DEFAULT)
    summary = collect(log_root, args.scenarios)
    summary['skip_policy'] = policy
    totals = write(summary, log_root)
    print(to_markdown(summary))
    print(f"JUnit 합계: tests {totals['tests']}, failures {totals['failures']}, "
          f"errors {totals['errors']}, skipped {totals['skipped']} → {log_root / AGGREGATE_XML}")
    print(counts_line(summary, policy))
    return exit_code(summary, policy)


if __name__ == '__main__':
    sys.exit(main())
