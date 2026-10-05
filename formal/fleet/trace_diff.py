#!/usr/bin/env python3
r"""
TLC 위반 트레이스를 상태마다 "바뀐 변수만" 보이게 줄인다.

사용법: python3 formal/fleet/trace_diff.py <TLC 로그 또는 traces/*.txt>
표준 라이브러리만 쓴다. TLC 출력 형식(1.7.x)의 "State n: <동작 ...>" 와
"/\\ 변수 = 값" 줄을 읽는다.
"""
import re
import sys


def parse(path):
    states = []
    cur = None
    var = None
    with open(path, encoding='utf-8') as fh:
        lines = fh.read().splitlines()
    try:
        start = next(i for i, ln in enumerate(lines) if ln.startswith('Error: Invariant'))
    except StopIteration:
        sys.exit('위반 트레이스가 없다: ' + path)
    for ln in lines[start:]:
        m = re.match(r'^State (\d+): <(\S+)', ln)
        if m:
            cur = {'n': int(m.group(1)), 'act': m.group(2), 'vars': {}}
            states.append(cur)
            var = None
            continue
        if cur is None:
            continue
        if re.search(r'states generated', ln):
            break
        m = re.match(r'^/\\ (\w+) = (.*)$', ln)
        if m:
            var = m.group(1)
            cur['vars'][var] = m.group(2).strip()
        elif var and ln.strip():
            cur['vars'][var] += ' ' + ln.strip()
    return states


REC = re.compile(r'\[[^\[\]]*\]')


def show_set_delta(name, old, new):
    """메시지 집합처럼 레코드 집합인 변수는 더해진/빠진 원소만 보인다."""
    old_set = set(REC.findall(old or ''))
    new_set = set(REC.findall(new))
    for rec in sorted(new_set - old_set):
        print('    %s + %s' % (name, rec))
    for rec in sorted(old_set - new_set):
        print('    %s - %s' % (name, rec))


def main():
    states = parse(sys.argv[1])
    prev = {}
    for st in states:
        changed = {k: v for k, v in st['vars'].items() if prev.get(k) != v}
        print('State %d: %s' % (st['n'], st['act']))
        for k in sorted(changed):
            if k == 'msgs' and prev:
                show_set_delta(k, prev.get(k), changed[k])
            else:
                print('    %s = %s' % (k, changed[k]))
        prev = st['vars']


if __name__ == '__main__':
    main()
