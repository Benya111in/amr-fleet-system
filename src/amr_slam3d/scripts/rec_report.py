#!/usr/bin/env python3
"""
녹화 시도 기록·표 (보너스 2 G 단계) — 호스트에서 표준 라이브러리만으로 돈다 (rec_driver.sh 가 부른다).

    rec_report.py registry <시도 디렉터리> --run-id B2rec-<태그> --scenario <id> --attempt <k> \
        --rc <종료 코드> --out registry.json
        실행 목록(notebook runs/b2-rec.jsonl) 한 줄 — notebook.py register 입력
    rec_report.py table <녹화 루트> --out table.md
        bag 마다 토픽별 메시지 수·sim 주기·간격 최대·공백 수 표 (사전 등록 분석)

표의 값 (bag_meta.json = bag_tools.py stats, gate.json = bag_tools.py gate):
  메시지 수, sim 주기 rate_hz = (n − 1)/(끝 − 처음) [header 스탬프], 간격 최대 dt_max [s],
  공백 = gate 규칙(> 1.5 주기 + 1 µs) 수 (검사 토픽만; 그 밖 토픽은 stats 의 중앙값 기준 gaps 를 참고값으로),
  RTF = /clock sim 구간 / 기록 벽시계 구간.
"""

import argparse
import datetime as dt
import json
import os
import re
import sys

GATED = ('/clock', 'ekf_tf', '/amr_01/lidar3d/points_raw', '/amr_01/lidar3d/points',
         '/amr_01/imu/data_raw', '/amr_01/imu/data')


def _load(path):
    if not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def _env(path):
    out = {}
    if not os.path.exists(path):
        return out
    for line in open(path, encoding='utf-8', errors='replace'):
        m = re.match(r'^([a-z_]+): (\S+)(?: loadavg: (.*))?$', line.strip())
        if m:
            out[m.group(1)] = {'value': m.group(2), 'load': (m.group(3) or '').strip()}
    return out


def rtf(meta):
    """RTF = /clock header 구간 / /clock 기록 벽시계 구간 (같은 토픽)."""
    if not meta or '/clock' not in meta.get('topics', {}):
        return None
    c = meta['topics']['/clock']
    h, w = c.get('header', {}), c.get('recv_wall', {})
    if h.get('span_s') and w.get('span_s'):
        return h['span_s'] / w['span_s']
    return None


def summary(d):
    """시도 디렉터리 → 핵심 결과 dict."""
    gate = _load(os.path.join(d, 'gate.json'))
    meta = _load(os.path.join(d, 'bag_meta.json'))
    env = _env(os.path.join(d, 'env.txt'))
    res = {'gate_pass': None if gate is None else bool(gate['pass'])}
    if gate is not None:
        res['gate_reasons'] = gate['reasons'][:6]
        res['n_gap'] = {k: v.get('n_gap') for k, v in gate['topics'].items()}
        res['start_still_s'] = round(gate['protocol'].get('start_still_s', float('nan')), 3)
        res['end_still_s'] = round(gate['protocol'].get('end_still_s', float('nan')), 3)
        res['depart_t'] = gate['protocol'].get('depart_t')
        leads = [v['max_s'] for v in gate.get('clock_lead', {}).values()]
        res['clock_lead_max_s'] = round(max(leads), 4) if leads else None
    if meta is not None:
        r = rtf(meta)
        res['rtf'] = None if r is None else round(r, 3)
        clk = meta['topics'].get('/clock', {}).get('header', {})
        res['sim_span_s'] = round(clk.get('span_s', 0.0), 2)
        res['bag_mb'] = round(sum(t['bytes'] for t in meta['topics'].values()) / 1e6, 1)
        pts = meta['topics'].get('/amr_01/lidar3d/points', {})
        res['n_points'] = pts.get('count')
    for k in ('record_subscribed', 'record_stop_clean', 'gate_rc', 'derive_rc'):
        if k in env:
            res[k] = env[k]['value']
    derive = _load(os.path.join(d, 'derive', 'derive.json'))
    if derive is not None:
        res['derive_n_out'] = derive.get('n_out')
    return res


def cmd_registry(ns) -> int:
    d = os.path.abspath(ns.dir)
    head = open(os.path.join(d, 'HEAD.txt'), encoding='utf-8').read().strip()
    status = open(os.path.join(d, 'status.txt'), encoding='utf-8').read().strip() \
        if os.path.exists(os.path.join(d, 'status.txt')) else ''
    started = open(os.path.join(d, 'started.txt'), encoding='utf-8').read().strip()
    load0 = open(os.path.join(d, 'loadavg_start.txt'), encoding='utf-8').read().strip()
    load1 = open(os.path.join(d, 'loadavg_end.txt'), encoding='utf-8').read().strip() \
        if os.path.exists(os.path.join(d, 'loadavg_end.txt')) else 'n/a'
    res = summary(d)
    res['rc'] = ns.rc
    res['attempt'] = ns.attempt
    ok = ns.rc == 0 and res.get('gate_pass') is True
    text = open(os.path.join(d, 'env.txt'), encoding='utf-8', errors='replace').read() \
        if os.path.exists(os.path.join(d, 'env.txt')) else ''
    m_route = re.search(r'route=(\S+)', text)
    m_var = re.search(r'variant=(\S+)', text)
    m_rtf = re.search(r'^rtf_target: (\S+)', text, re.M)
    route = os.path.splitext(os.path.basename(m_route.group(1)))[0] if m_route else '?'
    rec = {'run_id': ns.run_id, 'stage': 'G', 'agent': 'b2-rec', 'kind': 'gazebo', 'sim': 'gazebo',
           'commit': head[:7], 'dirty': bool(status),
           'config': f'scenario={ns.scenario} attempt={ns.attempt} route={route} '
                     f"world={m_var.group(1) if m_var else '?'} "
                     f"rtf_target={m_rtf.group(1) if m_rtf else '?'} "
                     f'(record_bag.sh, 원시 IMU 만, 공백 검사 recgate)',
           'seed': ns.seed, 'started': started,
           'ended': dt.datetime.now().astimezone().isoformat(timespec='seconds'),
           'raw_dir': d, 'result': res,
           'label': '[측정] 공유 부하 하 예비값' if ok else '실패',
           'load': f'{load0} → {load1}'}
    with open(ns.out, 'w', encoding='utf-8') as f:
        json.dump(rec, f, ensure_ascii=False, indent=1)
    print(json.dumps({'run_id': ns.run_id, 'ok': ok, 'gate_pass': res.get('gate_pass')},
                     ensure_ascii=False))
    return 0


def _first(path):
    """파일 첫 줄의 앞 세 낱말 (loadavg 1·5·15 분) — 없으면 '—'."""
    if not os.path.exists(path):
        return '—'
    return ' '.join(open(path, encoding='utf-8').read().split()[:3])


def _fmt(v, nd=3):
    if v is None:
        return '—'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def cmd_table(ns) -> int:
    root = os.path.abspath(ns.root)
    state = os.path.join(root, 'state')
    lines = ['| bag (시나리오) | 시도 | 판정 | sim 구간 s | RTF | 크기 MB | 처음/끝 정지 s | 출발 sim s |',
             '|---|---|---|---|---|---|---|---|']
    topic_lines = []
    for name in sorted(os.listdir(state)) if os.path.isdir(state) else []:
        if not name.endswith('.ok'):
            continue
        sid = name[:-3]
        tag = open(os.path.join(state, name), encoding='utf-8').read().strip()
        d = os.path.join(root, 'rec', tag)
        s = summary(d)
        tries = len([x for x in os.listdir(os.path.join(root, 'rec')) if x.startswith(sid + '_a')])
        lines.append(f"| `{tag}` ({sid}) | {tries} | {'통과' if s.get('gate_pass') else '실패'} | "
                     f"{_fmt(s.get('sim_span_s'), 1)} | {_fmt(s.get('rtf'))} | "
                     f"{_fmt(s.get('bag_mb'), 0)} | {_fmt(s.get('start_still_s'), 1)} / "
                     f"{_fmt(s.get('end_still_s'), 1)} | {_fmt(s.get('depart_t'), 2)} |")
        meta = _load(os.path.join(d, 'bag_meta.json')) or {'topics': {}}
        gate = _load(os.path.join(d, 'gate.json')) or {'topics': {}}
        topic_lines.append(f'\n**`{tag}`** — 토픽별 [측정] (bag_meta.json·gate.json)\n')
        topic_lines.append('| 토픽 | 수 | sim 주기 Hz | 간격 최대 s | 공백 (gate 규칙) | MB |')
        topic_lines.append('|---|---|---|---|---|---|')
        for tp, t in sorted(meta['topics'].items()):
            h = t.get('header', {})
            g = gate['topics'].get(tp, {})
            gap = g.get('n_gap') if tp in GATED else f"({_fmt(h.get('gaps'), 0)})"
            topic_lines.append(f"| `{tp}` | {t['count']} | {_fmt(h.get('rate_hz'))} | "
                               f"{_fmt(h.get('dt_max'))} | {_fmt(gap, 0)} | "
                               f"{t['bytes'] / 1e6:.1f} |")
        if 'ekf_tf' in gate['topics']:
            g = gate['topics']['ekf_tf']
            topic_lines.append(f"| `/tf` 안 EKF odom→base_footprint | {g['n']} | "
                               f"{_fmt((g['n'] - 1) / (g['last_s'] - g['first_s']))} | "
                               f"{_fmt(g['dt_max_s'])} | {g['n_gap']} | — |")
    att = ['\n**모든 시도** (실패 포함, 시도 디렉터리 순)\n',
           '| 시도 | rc | 공백 검사 | 실패 사유 (처음 2) | RTF | loadavg 시작 → 끝 |',
           '|---|---|---|---|---|---|']
    rec = os.path.join(root, 'rec')
    for tag in sorted(os.listdir(rec)) if os.path.isdir(rec) else []:
        d = os.path.join(rec, tag)
        s = summary(d)
        rc = open(os.path.join(d, 'rc.txt')).read().strip() \
            if os.path.exists(os.path.join(d, 'rc.txt')) else '(끊김)'
        why = '; '.join(s.get('gate_reasons', [])[:2]).replace('|', '/')
        l0 = _first(os.path.join(d, 'loadavg_start.txt'))
        l1 = _first(os.path.join(d, 'loadavg_end.txt'))
        verdict = {True: '통과', False: '실패', None: '—'}[s.get('gate_pass')]
        att.append(f"| `{tag}` | {rc} | {verdict} | {why or '—'} | {_fmt(s.get('rtf'))} | "
                   f'{l0} → {l1} |')
    text = '\n'.join(lines + topic_lines + att) + '\n'
    with open(ns.out, 'w', encoding='utf-8') as f:
        f.write(text)
    print(text)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('registry')
    r.add_argument('dir')
    r.add_argument('--run-id', required=True)
    r.add_argument('--scenario', required=True)
    r.add_argument('--attempt', type=int, required=True)
    r.add_argument('--rc', type=int, required=True)
    r.add_argument('--seed', default='n/a (경로 고정)')
    r.add_argument('--out', required=True)
    t = sub.add_parser('table')
    t.add_argument('root')
    t.add_argument('--out', required=True)
    ns = ap.parse_args(argv)
    return {'registry': cmd_registry, 'table': cmd_table}[ns.cmd](ns)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
