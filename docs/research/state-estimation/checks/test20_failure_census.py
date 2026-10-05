"""test_20_forced_deadlock 의 실패를 **사유별로** 가른다 (읽기 전용, 수 초).

왜 필요한가: "15 회 중 6 회 실패(40 %)" 를 측위 별칭 하나로 귀속한 적이 있는데 틀렸다.
실패 메시지와 트레이스백 줄 번호로 가르면 사유가 넷이고, 줄 번호가 서로 달라
**테스트 코드 버전도 다르다**. 빈도를 근거로 쓰려면 사유를 먼저 갈라야 한다.

    python3 docs/research/state-estimation/checks/test20_failure_census.py

읽는 것: logs/<태그>/12_multi_robot_deadlock/{junit.xml,result.json}, logs/<태그>/HEAD.txt
"""
import json
import os
import re
import xml.etree.ElementTree as ET
from collections import Counter
from glob import glob
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]      # .../checks/ -> 저장소 루트

# 사유 분류 — 실패 메시지의 고유 문구로 가른다. 순서가 곧 우선순위다.
PATTERNS = [
    ('측위 별칭', '배치 뒤 위치 추정이 안정되지 않음'),
    ('busy 작업 거절', 'busy'),
    ('IDLE 아님', 'IDLE'),
    ('교착 미해소', 'UNRESOLVED'),
]


def classify(msg: str) -> str:
    for name, needle in PATTERNS:
        if needle in msg:
            return name
    return '기타'


def main() -> int:
    rows = []
    for f in sorted(glob(str(REPO / 'logs/*/12_multi_robot_deadlock/junit.xml'))):
        tag = Path(f).parts[-3]
        started = '?'
        rj = Path(f).with_name('result.json')
        if rj.exists():
            try:
                started = (json.loads(rj.read_text()).get('started_at') or '?')[:16]
            except Exception:
                pass
        hp = REPO / 'logs' / tag / 'HEAD.txt'
        head = hp.read_text().strip()[:7] if hp.exists() else '—'
        try:
            tree = ET.parse(f)
        except ET.ParseError:
            continue
        for tc in tree.iter('testcase'):
            if 'forced_deadlock' not in (tc.get('name') or ''):
                continue
            bad = [c for c in tc if c.tag in ('failure', 'error')]
            if not bad:
                rows.append((started, tag, head, '통과', '', ''))
                continue
            msg = ((bad[0].get('message') or '') + ' ' + (bad[0].text or '')).replace('\n', ' ')
            m = re.search(r'test_12_multi_robot_deadlock\.py", line (\d+)', msg)
            rows.append((started, tag, head, '실패', classify(msg), m.group(1) if m else '?'))

    rows.sort()
    print(f'{"시작":17s} {"실행":9s} {"HEAD":8s} {"결과":5s} {"사유":14s} 코드줄')
    for r in rows:
        print(f'{r[0]:17s} {r[1]:9s} {r[2]:8s} {r[3]:5s} {r[4]:14s} {r[5]}')

    fails = [r for r in rows if r[3] == '실패']
    print()
    print(f'test_20 을 돌린 실행 {len(rows)} 회, 실패 {len(fails)} 회 '
          f'({100.0 * len(fails) / len(rows):.1f} %)')
    for name, n in Counter(r[4] for r in fails).most_common():
        print(f'  {n} 건 ({100.0 * n / len(rows):4.1f} %)  {name}')

    lines = sorted({r[5] for r in fails})
    print()
    print(f'실패 지점의 코드 줄: {", ".join(lines)} — 서로 다르면 테스트 코드 버전이 다르다는 뜻이다.')
    print('현 코드의 settle 검사 위치는 아래와 같다:')
    src = (REPO / 'tests/integration/test_12_multi_robot_deadlock.py').read_text().split('\n')
    for i, line in enumerate(src, 1):
        if '안정되지 않음' in line:
            print(f'  test_12_multi_robot_deadlock.py:{i}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
