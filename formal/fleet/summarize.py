#!/usr/bin/env python3
"""
TLC 로그 머리·꼬리를 읽어 결과 표(Markdown)를 만든다.

사용법: python3 formal/fleet/summarize.py <로그 디렉터리> [cfg 이름 ...]
cfg 이름을 주면 그 순서로, 안 주면 로그 전부를 이름순으로 싣는다.
손으로 옮겨 적지 않으려고 둔다 (수치는 모두 로그 원문에서 온다).
"""
import datetime
import os
import re
import sys


def field(text, pat, default=''):
    m = re.search(pat, text, re.M)
    return m.group(1) if m else default


def summarize(path):
    with open(path, encoding='utf-8', errors='replace') as fh:
        text = fh.read()
    name = os.path.basename(path)[:-4]
    start = field(text, r'^# start=(\S+)')
    end = field(text, r'^# end=(\S+)')
    rc = field(text, r'^# end=\S+ rc=(\d+)')
    load = field(text, r'^# start=\S+ loadavg=(\S+)')
    secs = ''
    if start and end:
        t0 = datetime.datetime.fromisoformat(start)
        t1 = datetime.datetime.fromisoformat(end)
        secs = str(int((t1 - t0).total_seconds()))
    gen = field(text, r'^([\d,]+) states generated, [\d,]+ distinct')
    dist = field(text, r'^[\d,]+ states generated, ([\d,]+) distinct')
    depth = field(text, r'depth of the complete state graph search is (\d+)')
    viol = field(text, r'^Error: Invariant (\w+) is violated')
    sim = 'Running Random Simulation' in text
    opts = field(text, r'^# tlc_opts=(.*)$')
    if sim:
        found = re.findall(r'^Progress: ([\d,]+) states checked', text, re.M)
        checked = found[-1] if found else ''
        checked = checked or field(text, r'^The number of states generated: ([\d,]+)', '')
        gen = checked or '-'
        dist = '(시뮬레이션: distinct 없음)'
        depth = field(text, r'-depth (\d+)', '?')
        depth = '경로 상한 ' + depth
    if viol:
        nstates = len(re.findall(r'^State \d+:', text, re.M))
        verdict = '위반 %s (트레이스 %d 상태)' % (viol, nstates)
        if sim:
            verdict = '시뮬레이션 ' + verdict
    elif sim and end and rc == '0':
        verdict = '시뮬레이션 위반 없음 (%s, 전수 아님)' % (opts.strip() or '옵션 미기록')
    elif sim:
        verdict = '시뮬레이션 중단, 그때까지 위반 못 찾음 (rc=%s; %s)' % (rc or '없음', opts.strip())
    elif 'No error has been found' in text:
        verdict = '위반 없음 (전수)'
    elif rc == '124' or not end or not gen:
        last = re.findall(r'^Progress\((\d+)\).*?([\d,]+) distinct states found', text, re.M)
        verdict = '미완 (시간 상한/중단, rc=%s)' % (rc or '없음')
        if last:
            depth = '>=' + last[-1][0]
            dist = '>=' + last[-1][1]
        gen = gen or '-'
    else:
        err = field(text, r'^(Error: .*)$')
        verdict = '오류: ' + err
    spec = field(text, r'^# spec_sha256=(\w{12})')
    return '| %s | %s | %s | %s | %s | %s | %s | %s |' % (
        name, verdict, gen, dist, depth, secs, load, spec)


def main():
    d = sys.argv[1]
    names = sys.argv[2:] or sorted(f[:-4] for f in os.listdir(d)
                                   if f.endswith('.log') and '.' not in f[:-4])
    print('| cfg | 결과 | states generated | distinct | 깊이 | wall s | 시작 loadavg(1m) '
          '| spec sha256[:12] |')
    print('|---|---|---|---|---|---|---|---|')
    for n in names:
        p = os.path.join(d, n + '.log')
        if os.path.exists(p):
            print(summarize(p))
        else:
            print('| %s | 로그 없음 | | | | | | |' % n)


if __name__ == '__main__':
    main()
