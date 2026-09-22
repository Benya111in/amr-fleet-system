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
종료 코드  0: 실패/에러/누락 없음 (skip 은 통과로 셈, --fail-on-skip 이면 실패)
          1: 하나라도 failed / error / missing
"""

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional, Sequence

from amr_itest import catalog
from amr_itest import junit
from amr_itest import results
from amr_itest.config import repo_root

SUMMARY_JSON = 'summary.json'
SUMMARY_MD = 'summary.md'
AGGREGATE_XML = 'junit.xml'
UNIT_DIR = 'unit'


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
    return {
        'id': scenario.id, 'number': scenario.number, 'title': scenario.title,
        'status': status, 'skip_reason': reason,
        'implemented': scenario.implemented, 'backend': res.get('backend'),
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


def _short(entry: Dict[str, Any], limit: int = 160) -> str:
    """표의 비고 칸: skip 사유 / 실패 판정 / 통과 판정 요약."""
    if entry['status'] == junit.SKIPPED:
        text = entry['skip_reason']
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
             f"하네스 단위 테스트: {summary['unit']['status'] or '실행 안 함'}  ",
             f"호스트 부하(집계 시점): {summary['host_now']['loadavg']} / "
             f"{summary['host_now']['cpus']} CPU", '',
             '| # | 시나리오 | 상태 | 백엔드 | 소요 [s] | 비고 |',
             '| --- | --- | --- | --- | --- | --- |']
    for e in summary['scenarios']:
        lines.append(f"| {e['number']} | {e['title']} | {e['status']} | {e['backend'] or '-'} | "
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


def exit_code(summary: Dict[str, Any], fail_on_skip: bool = False) -> int:
    bad = {junit.FAILED, junit.ERROR, junit.MISSING}
    if fail_on_skip:
        bad.add(junit.SKIPPED)
    if any(e['status'] in bad for e in summary['scenarios']):
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
    p.add_argument('--fail-on-skip', action='store_true', help='skip 도 실패로 셈')
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
    summary = collect(log_root, args.scenarios)
    totals = write(summary, log_root)
    print(to_markdown(summary))
    print(f"JUnit 합계: tests {totals['tests']}, failures {totals['failures']}, "
          f"errors {totals['errors']}, skipped {totals['skipped']} → {log_root / AGGREGATE_XML}")
    return exit_code(summary, args.fail_on_skip)


if __name__ == '__main__':
    sys.exit(main())
