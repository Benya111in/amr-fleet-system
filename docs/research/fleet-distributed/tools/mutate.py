#!/usr/bin/env python3
"""
변이 검사: 가드를 하나씩 되돌린 사본에서 시험이 실패하는지 본다 (COORDINATION §4, handover §5).

패키지 밖 도구라 CI 린트 대상이 아니다 (치환 문자열이 소스와 정확히 같아야 해서 긴 줄이 있다).

사용 (컨테이너 안, 저장소 루트에서):
  python3 docs/research/fleet-distributed/tools/mutate.py --out /logs/mutation [--only M1,M3]
각 변이마다 src/amr_fleet 을 임시 디렉터리에 복사하고 문자열 치환(정확히 1 회 일치해야 함)을 적용한 뒤
시험 묶음별로 pytest 를 돌린다. 시험이 하나라도 실패하면 "잡힘(killed)".
묶음: unit = 단위 시험 파일들, stateful = Hypothesis 상태기계(불변식 I1–I5),
      stateful_noI4 = 같은 상태기계에서 보조 불변식 I4(허가 ⇒ 과반 확정)를 끈 것 (핵심 불변식만으로 잡히나).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

VC = 'amr_fleet/distributed/vote_core.py'
ST = 'amr_fleet/distributed/store.py'
FD = 'amr_fleet/distributed/failure_detector.py'
WF = 'amr_fleet/distributed/wfg.py'

MUTATIONS = [
    ('M1', '과반 조건: 정족수 n//2 (2/5) + 검증 제거', VC, [
        ('q = len(members) // 2 + 1 if self.quorum is None', 'q = len(members) // 2 if self.quorum is None'),
        ('if not 2 * q > len(members) or q > len(members):', 'if q > len(members):'),
    ]),
    ('M2', '잠정 표 배타 제거: 남의 잠정 표를 덮어쓴다 (예상: 안전 무관 — 2단계 확정이 배타를 다시 세운다)', VC, [
        ('free = all(z not in self.votes for z in p.resources)',
         'free = all(z not in self.votes or not self.votes[z].committed for z in p.resources)'),
    ]),
    ('M2b', '배타 투표 제거: 확정 표까지 덮어쓴다', VC, [
        ('free = all(z not in self.votes for z in p.resources)', 'free = True'),
    ]),
    ('M3', '확정 표도 기한 만료 (리스 회수형)', VC, [
        ('gone = [z for z, v in self.votes.items() if not v.committed and v.deadline <= now]',
         'gone = [z for z, v in self.votes.items() if v.deadline <= now]'),
    ]),
    ('M4', '표를 영속화하지 않음 (재시작하면 표를 잊음)', VC, [
        ("for z, v in sorted(self.votes.items())},", "for z, v in sorted(self.votes.items()) if False},"),
    ]),
    ('M5', 'fsync 순서: handle 응답 뒤로 영속화 미룸 (tick 때만 저장)', VC, [
        ('''            self._on_release_ack(msg)
        self._flush()
        return out''', '''            self._on_release_ack(msg)
        return out'''),
    ]),
    ('M6', '1단계 진입: 과반 VOTE 만으로 허가 (ENTER_ACK 생략)', VC, [
        ('''        if a.phase == VOTING and len(a.votes) >= self.q:
            a.phase = ENTERING''', '''        if a.phase == VOTING and len(a.votes) >= self.q:
            self.held.setdefault(a.req, set()).update(a.resources)
            self.attempt = None
            self.logical = None
            self._dirty = True
            return []'''),
    ]),
    ('M7', 'epoch 펜스 제거: 낮은 epoch 메시지도 처리', VC, [
        ('        if msg.epoch < self.max_epoch:\n', '        if False:\n'),
    ]),
    ('M8', '요청자: ENTER_ACK 의 요청 ID 를 확인하지 않음', VC, [
        ('if a is None or msg.req != a.req or a.phase != ENTERING:',
         'if a is None or a.phase != ENTERING:'),
    ]),
    ('M9', '요청자: ENTER_ACK 정족수 하나 모자라도 허가', VC, [
        ('if len(a.acks) >= self.q:', 'if len(a.acks) >= self.q - 1:'),
    ]),
    ('M10', '투표자: RELEASE 가 다른 요청의 표도 푼다', VC, [
        ('''            v = self.votes.get(z)
            if v is not None and v.req == req:
                del self.votes[z]''', '''            v = self.votes.get(z)
            if v is not None:
                del self.votes[z]'''),
    ]),
    ('M11', '투표자: ENTER 가 다른 요청에 묶인 표도 확정', VC, [
        ('if msg.resources and all(v is not None and v.req == req for v in bound):',
         'if msg.resources and all(v is not None for v in bound):'),
    ]),
    ('M12', 'epoch 채택 때 확정 표까지 버림', VC, [
        ('for z in [z for z, v in self.votes.items() if not v.committed]:',
         'for z in list(self.votes):'),
    ]),
    ('M13', '사슬 부분 부여: 빈 자원만 표를 준다', VC, [
        ('free = all(z not in self.votes for z in p.resources)',
         'free = any(z not in self.votes for z in p.resources)'),
        ('''                for z in p.resources:
                    self.votes[z] = _Vote(req, False, p.deadline)''',
         '''                for z in [z for z in p.resources if z not in self.votes]:
                    self.votes[z] = _Vote(req, False, p.deadline)'''),
    ]),
    ('M14', '묘비 비활성 (활성 보호 — 안전과 무관할 것으로 예상)', VC, [
        ('            self.tombstones[req] = None\n', '            pass\n'),
    ]),
    ('M15', 'incarnation 을 재시작 때 올리지 않음', VC, [
        ('        agent.incarnation += 1\n', '        agent.incarnation += 0\n'),
    ]),
    ('M16', '재시작: 손상 저장소를 빈 상태로 보고 다시 투표 (기억 상실 무시)', VC, [
        ('            agent.amnesic = True\n            # 옛 incarnation',
         '            agent.amnesic = False\n            # 옛 incarnation'),
    ]),
    ('M17', 'FileStore: rename 전 파일 fsync 생략', ST, [
        ('            os.fsync(fd)                  # 1) 내용이 디스크에 닿은 뒤에만\n', ''),
    ]),
    ('M18', 'FileStore: 디렉터리 fsync 생략', ST, [
        ('            os.fsync(dfd)                 # 3) 디렉터리 항목(rename)도 영속화\n',
         '            pass\n'),
    ]),
    ('M19', 'FileStore: fsync 를 rename 뒤로', ST, [
        ('            os.fsync(fd)                  # 1) 내용이 디스크에 닿은 뒤에만\n', ''),
        ('        os.replace(tmp, self.path)        # 2) 이름을 바꾸고\n',
         '        os.replace(tmp, self.path)        # 2) 이름을 바꾸고\n'
         '        _f = os.open(self.path, os.O_RDONLY)\n        os.fsync(_f)\n        os.close(_f)\n'),
    ]),
]

MUTATIONS += [
    ('F1', '감지: 과반 대신 1 대의 DEAD 로 DEGRADED', FD, [
        ('return len(self.dead_votes(now)) >= self.quorum', 'return len(self.dead_votes(now)) >= 1'),
    ]),
    ('F2', '감지: 자기 정지 직후 DEAD 보류 제거', FD, [
        ('            if self.health is not None and self.health.late:\n',
         '            if False:\n'),
    ]),
    ('F3', '감지: 옛 · 중복 하트비트도 갱신', FD, [
        ('        if self.last_key is not None and key <= self.last_key:\n',
         '        if False:\n'),
    ]),
    ('F4', '감지: 지역 건강 배수 무시', FD, [
        ('mult = self.health.multiplier() if self.health is not None else 1.0', 'mult = 1.0'),
    ]),
    ('F5', 'EpochFence: 낮은 epoch 도 수용', FD, [
        ('        if self.max is not None and key < self.max:\n', '        if False:\n'),
    ]),
    ('F6', '과반 의심: 비컨 신선도 무시', FD, [
        ('                if b.rx <= now and _sec(b.rx, now) <= self.ttl_s}', '                if True}'),
    ]),
    ('W1', 'WFG 희생자: 어린 쪽 대신 늙은 쪽', WF, [
        ('    return (-int(info.priority), ts, info.robot)', '    return (-int(info.priority), -ts, info.robot)'),
    ]),
    ('W2', 'WFG: block SCC 의 knot 조건 제거', WF, [
        ('                continue          # OR 대기', '                pass          # OR 대기'),
    ]),
    ('W3', 'WFG: 묵은 비컨도 판정에 씀', WF, [
        ('            if rx <= now and (now.total_ns() - rx.total_ns()) / 1e9 <= self.ttl_s:',
         '            if True:'),
    ]),
    ('W4', 'WFG: 진행이 있어도 확정 타이머를 다시 재지 않음', WF, [
        ('            if base is None or moved:', '            if base is None:'),
    ]),
    ('W5', 'WFG: 재정렬된 옛 비컨으로 덮어씀', WF, [
        ('        if old is not None and (info.incarnation, info.seq) <= (old[1].incarnation, old[1].seq):',
         '        if False:'),
    ]),
]

SUITES = {
    'unit': ['test/test_dist_vote_core.py', 'test/test_dist_store.py', 'test/test_dist_ordering.py',
             'test/test_dist_failure_detector.py', 'test/test_dist_wfg.py',
             'test/test_dist_rejected_variant.py'],
    'stateful': ['test/test_dist_vote_stateful.py::test_vote_core_state_machine'],
    'stateful_noI4': ['test/test_dist_vote_stateful.py::test_vote_core_state_machine'],
}


def run_suite(pkg, suite, env_extra, timeout):
    files = [f for f in SUITES[suite] if os.path.exists(os.path.join(pkg, f.split('::')[0]))]
    if not files:
        return None, 0.0, 'no files'
    env = dict(os.environ, PYTHONPATH=pkg, **env_extra)
    if suite == 'stateful_noI4':
        env['AMR_DIST_SKIP_I4'] = '1'
    t0 = time.time()
    try:
        p = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-x', '-p', 'no:cacheprovider']
                           + files, cwd=pkg, env=env, capture_output=True, text=True,
                           timeout=timeout)
        rc, out = p.returncode, p.stdout[-3000:] + p.stderr[-1000:]
    except subprocess.TimeoutExpired:
        rc, out = -9, 'timeout'
    return rc, time.time() - t0, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default='src/amr_fleet')
    ap.add_argument('--out', required=True)
    ap.add_argument('--only', default='')
    ap.add_argument('--suites', default='unit,stateful,stateful_noI4')
    ap.add_argument('--timeout', type=int, default=900)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    only = set(filter(None, args.only.split(',')))
    suites = args.suites.split(',')
    env_extra = {k: v for k, v in os.environ.items() if k.startswith('AMR_DIST_HYP_')}
    results = []
    # 기준선: 변이 없는 사본은 모든 묶음이 통과해야 한다
    todo = [('M0', '변이 없음 (기준선)', None, [])] + MUTATIONS
    for mid, desc, path, subs in todo:
        if only and mid not in only and mid != 'M0':
            continue
        tmp = tempfile.mkdtemp(prefix=f'mut_{mid}_')
        pkg = os.path.join(tmp, 'amr_fleet')
        shutil.copytree(args.src, pkg)
        ok = True
        if path is not None:
            fp = os.path.join(pkg, path)
            s = open(fp, encoding='utf-8').read()
            for old, new in subs:
                n = s.count(old)
                if n != 1:
                    ok = False
                    print(f'{mid}: 치환 대상 {n} 회 일치 — 건너뜀: {old[:60]!r}', flush=True)
                    break
                s = s.replace(old, new)
            if ok:
                open(fp, 'w', encoding='utf-8').write(s)
        row = {'id': mid, 'desc': desc, 'file': path, 'applied': ok, 'suites': {}}
        if ok:
            # 감지기 · WFG 변이는 투표 상태기계와 무관하다 — 단위 시험만
            my_suites = ['unit'] if path in (FD, WF) else suites
            for suite in my_suites:
                rc, dt, out = run_suite(pkg, suite, env_extra, args.timeout)
                row['suites'][suite] = {'rc': rc, 'killed': None if rc is None else rc != 0,
                                        'sec': round(dt, 1)}
                with open(os.path.join(args.out, f'{mid}_{suite}.log'), 'w') as f:
                    f.write(out)
                print(f'{mid} {suite}: rc={rc} {dt:.1f}s', flush=True)
        results.append(row)
        shutil.rmtree(tmp, ignore_errors=True)
    with open(os.path.join(args.out, 'mutation_results.json'), 'w') as f:
        json.dump(results, f, ensure_ascii=False, indent=1)
    print('| id | 변이 | ' + ' | '.join(suites) + ' |')
    print('|---|---|' + '---|' * len(suites))
    for row in results:
        cells = []
        for suite in suites:
            r = row['suites'].get(suite)
            if r is None or r['killed'] is None:
                cells.append('—')
            else:
                cells.append(('잡힘' if r['killed'] else '**놓침**') if row['id'] != 'M0'
                             else ('통과' if not r['killed'] else '**실패**'))
        print(f"| {row['id']} | {row['desc']} | " + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    main()
