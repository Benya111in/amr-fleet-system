#!/usr/bin/env python3
"""
xunit XML 집계 · 비교 (b4-wire 회귀 판정용, 표준 라이브러리만).

  python3 xunit_summary.py summary <run_dir> [--pkgs amr_fleet amr_behavior amr_bringup]
  python3 xunit_summary.py compare <기준 run_dir> <새 run_dir> [--pkgs ...]

run_dir 아래 <pkg>_test_results/**/*.xml (regress.sh 가 남긴 것)을 읽는다. 시험 사례 키 = (pkg, xml 파일 이름,
classname, name). 상태 = pass · fail · error · skip.
compare 판정 (회귀 = 기존 시험 무변화): 기준에서 pass 였던 사례가 새 실행에서 pass 가 아니거나 사라지면 회귀.
기준의 skip 이 새 실행에서 다른 상태가 된 것 · 새로 생긴 사례는 따로 적는다 (회귀로 세지 않는다).
종료 코드: 0 회귀 없음, 1 회귀 있음, 2 입력 오류.
"""

import argparse
import collections
import json
import pathlib
import sys
import xml.etree.ElementTree as ET

DEFAULT_PKGS = ('amr_msgs', 'amr_fleet', 'amr_behavior', 'amr_bringup')


def cases(run_dir: pathlib.Path, pkgs):
    out = {}
    for pkg in pkgs:
        base = run_dir / f'{pkg}_test_results'
        if not base.is_dir():
            continue
        for xml in sorted(base.rglob('*.xml')):
            try:
                root = ET.parse(xml).getroot()
            except ET.ParseError:
                out[(pkg, xml.name, '<parse>', '<parse>')] = 'error'
                continue
            for tc in root.iter('testcase'):
                if tc.find('failure') is not None:
                    st = 'fail'
                elif tc.find('error') is not None:
                    st = 'error'
                elif tc.find('skipped') is not None:
                    st = 'skip'
                else:
                    st = 'pass'
                out[(pkg, xml.name, tc.get('classname', ''), tc.get('name', ''))] = st
    return out


def totals(cs):
    t = collections.defaultdict(collections.Counter)
    for (pkg, *_), st in cs.items():
        t[pkg][st] += 1
        t[pkg]['tests'] += 1
    return {k: dict(v) for k, v in sorted(t.items())}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('summary')
    s.add_argument('run_dir')
    s.add_argument('--pkgs', nargs='+', default=list(DEFAULT_PKGS))
    c = sub.add_parser('compare')
    c.add_argument('base')
    c.add_argument('new')
    c.add_argument('--pkgs', nargs='+', default=list(DEFAULT_PKGS))
    a = ap.parse_args(argv)
    if a.cmd == 'summary':
        cs = cases(pathlib.Path(a.run_dir), a.pkgs)
        bad = sorted('/'.join(k) for k, v in cs.items() if v in ('fail', 'error'))
        print(json.dumps({'totals': totals(cs), 'failed': bad}, ensure_ascii=False, indent=1))
        return 0 if not bad else 1
    base = cases(pathlib.Path(a.base), a.pkgs)
    new = cases(pathlib.Path(a.new), a.pkgs)
    if not base or not new:
        print('입력 오류: 시험 사례가 없다', file=sys.stderr)
        return 2
    regress = sorted('/'.join(k) + f' {base[k]}→{new.get(k, "없음")}' for k in base
                     if base[k] == 'pass' and new.get(k) != 'pass')
    skip_changed = sorted('/'.join(k) + f' skip→{new.get(k, "없음")}' for k in base
                          if base[k] == 'skip' and new.get(k) != 'skip')
    added = sorted(k for k in new if k not in base)
    added_bad = sorted('/'.join(k) + f' {new[k]}' for k in added if new[k] in ('fail', 'error'))
    out = {'base_totals': totals(base), 'new_totals': totals(new),
           'regressions': regress, 'skip_changed': skip_changed,
           'added': len(added), 'added_by_pkg': dict(collections.Counter(k[0] for k in added)),
           'added_failed': added_bad,
           'base_not_pass': sorted('/'.join(k) + f' {v}' for k, v in base.items()
                                   if v in ('fail', 'error'))}
    print(json.dumps(out, ensure_ascii=False, indent=1))
    return 1 if regress or added_bad else 0


if __name__ == '__main__':
    sys.exit(main())
