"""시나리오 12 의 **모든** 실행을 사유별로 가른다 (읽기 전용, 수 초).

왜 있는가: 시나리오 12 는 실패율이 높은데 그 실패가 **한 원인이 아니다**. 사유를 가르지 않고
횟수만 세면 틀린 결론이 나온다 — 실제로 한 번 그렇게 적었다가 바로잡았다
(performance.md §4.5, "별칭 40 %" → 13.3 %).

    python3 docs/research/fleet-traffic-deadlock/checks/scenario12_census.py

읽는 것 (실행 하나당):
  logs/<태그>/12_multi_robot_deadlock/result.json   구조화된 판정 (checks[].passed)
  logs/<태그>/12_multi_robot_deadlock/junit.xml     테스트 단위 단언 (check 가 아닌 실패)
  logs/<태그>/12_multi_robot_deadlock/launch.log    보조 신호 (응답 유실 등)
  logs/<태그>/HEAD.txt                              측정 시점 커밋

**분류할 때 조심할 것** (둘 다 내가 실제로 틀렸던 것이다):
  1. `logs/<태그>/junit.xml` 은 전수 회귀에서 **모든 시나리오 합본**이다.
     반드시 `12_multi_robot_deadlock/junit.xml` 을 읽어라.
  2. 트레이스백에 파일명 `test_12_multi_robot_deadlock.py` 가 들어 있어 'deadlock' 으로
     분류하면 전부 교착이 된다. **단언 메시지만** 보고 가른다.
"""
import json
import os
import re
import xml.etree.ElementTree as ET
from collections import Counter
from glob import glob
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]

#: 단언 메시지에서 사유를 가르는 고유 문구. 순서가 우선순위다.
PATTERNS = [
    ('settle(측위 별칭)', '배치 뒤 위치 추정이 안정되지 않음'),
    ('작업 미완', 'tasks completed'),
    ('미해소 교착(전체)', 'detected deadlocks resolved'),
    ('미해소 교착(강제)', 'forced deadlock resolved'),
    ('강제 교착 미탐지', 'forced deadlock detected'),
    ('busy 작업 거절', 'busy'),
    ('IDLE 아님', 'IDLE'),
    ('map 정합', 'map frame == world frame'),
    ('CPU 초과', 'system CPU'),
    ('TF', 'TF frame'),
    ('프로세스 크래시', 'process crash'),
    ('러너 타임아웃', 'timeout after'),
]


def classify(text: str) -> str:
    """단언 메시지(트레이스백 제외)에서 사유 한 가지."""
    tail = text.split('AssertionError')[-1] if 'AssertionError' in text else text
    for name, needle in PATTERNS:
        if needle in tail:
            return name
    return '기타: ' + re.sub(r'\s+', ' ', tail).strip()[:50]


def one_run(tag: str) -> dict:
    d = REPO / 'logs' / tag / '12_multi_robot_deadlock'
    out = {'tag': tag, 'status': '?', 'head': '—', 'duration': None,
           'failed_checks': [], 'failed_tests': [], 'signals': {}}
    rj = d / 'result.json'
    if rj.exists():
        try:
            r = json.loads(rj.read_text())
        except Exception:
            r = {}
        out['duration'] = r.get('duration_s')
        out['failed_checks'] = [c['name'] for c in r.get('checks', []) if not c.get('passed')]
    sj = REPO / 'logs' / tag / 'summary.json'
    if sj.exists():
        try:
            for s in json.loads(sj.read_text())['scenarios']:
                if s['id'].startswith('12'):
                    out['status'] = s['status']
        except Exception:
            pass
    hp = REPO / 'logs' / tag / 'HEAD.txt'
    if hp.exists():
        out['head'] = hp.read_text().strip()[:7]
    jf = d / 'junit.xml'                       # ← 시나리오 전용. 합본을 읽으면 안 된다
    if jf.exists():
        try:
            for tc in ET.parse(jf).iter('testcase'):
                for b in tc:
                    if b.tag in ('failure', 'error'):
                        msg = (b.get('message') or '') + ' ' + (b.text or '')
                        out['failed_tests'].append((tc.get('name', '?'), classify(msg)))
        except ET.ParseError:
            pass
    lg = d / 'launch.log'
    if lg.exists():
        txt = lg.read_text(errors='replace')
        out['signals'] = {
            'resp_loss': txt.count('failed to send response'),
            'exhausted': txt.count('recovery exhausted'),
            'reinit': txt.count('global re-init'),
            'completed': txt.count(': COMPLETED'),
        }
    return out


def main() -> int:
    tags = sorted(p.split('/')[-2] for p in glob(str(REPO / 'logs/*/12_multi_robot_deadlock')))
    runs = [one_run(t) for t in tags]
    runs.sort(key=lambda r: (r['status'] != 'passed', r['tag']))

    print(f'{"실행":10s} {"HEAD":8s} {"판정":7s} {"소요":>6s} {"유실":>4s} {"exh":>4s} '
          f'{"완료":>4s}  실패 사유')
    for r in runs:
        s = r['signals']
        why = '; '.join(f'{n}:{k}' for n, k in r['failed_tests']) or '—'
        dur = f"{int(r['duration'])}" if r['duration'] else '—'
        print(f'{r["tag"]:10s} {r["head"]:8s} {r["status"]:7s} {dur:>6s} '
              f'{s.get("resp_loss", 0):>4} {s.get("exhausted", 0):>4} '
              f'{s.get("completed", 0):>4}  {why[:72]}')

    ok = sum(1 for r in runs if r['status'] == 'passed')
    print(f'\n실행 {len(runs)} 회 — 통과 {ok} / 실패 {len(runs) - ok} ({100.0 * ok / len(runs):.0f} %)')

    print('\n실패 사유 (테스트 단언 기준, 한 실행이 여러 개를 낼 수 있다):')
    c = Counter(k for r in runs for _, k in r['failed_tests'])
    for k, n in c.most_common():
        print(f'  {n:3d}  {k}')

    print('\n실패한 판정 이름 (result.json checks):')
    cc = Counter(n for r in runs for n in r['failed_checks'])
    for k, n in cc.most_common():
        print(f'  {n:3d}  {k}')

    print('\n응답 유실이 난 실행과 그 결과 (lifecycle 사슬의 영향 범위):')
    for r in runs:
        if r['signals'].get('resp_loss'):
            print(f'  {r["tag"]:10s} 유실 {r["signals"]["resp_loss"]} · '
                  f'exhausted {r["signals"]["exhausted"]} → {r["status"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
