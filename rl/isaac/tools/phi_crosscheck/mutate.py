#!/usr/bin/env python3
"""
정본 safety φ 이식 변이 검사 — 규칙·가드를 하나씩 되돌린 사본에서 교차 시험(작은 묶음)이 실패하는가.

    python3 mutate.py --src <저장소 스냅숏> --binary <gate_runner> --work <임시 디렉터리> --out <json>

스냅숏의 rl/isaac 를 변이마다 임시 디렉터리로 복사해 safety_gate_torch.py 한 곳만 바꾼다 (원본·worktree 는 건드리지
않는다). 판정: crosscheck.py --quick 이 rc ≠ 0 이면 잡힘. 살아남으면 단위 시험(tests/test_safety_gate_torch.py)도
돌려 본다. 미리 동치로 예측한 변이(EXPECT_EQUIV)는 살아남아야 맞다 — 그 이유를 결과에 적는다.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

F = 'amr_isaac/safety_gate_torch.py'
# (이름, 원본 조각, 변이 조각, 되돌리는 규칙의 원본 위치)
MUTANTS = [
    ('M01 의도 유지 끔', 'use_intent = ~mv_c & ~use_meas & self.have_intent',
     'use_intent = torch.zeros_like(mv_c)', 'motionHypotheses :526-528'),
    ('M02 측정 운동 가설 끔', 'reg_b = Region(self.meas_v, self.meas_w, use_meas, p)',
     'reg_b = Region(self.meas_v, self.meas_w, torch.zeros_like(use_meas), p)',
     'motionHypotheses :520-524'),
    ('M03 STOP 확정 1 프레임', '((self.ch_cnt >= p.stop_confirm_frames) |',
     '((self.ch_cnt >= 1) |', 'confirmed :550'),
    ('M04 즉시 확정 여유 끔', '(self.ch_val <= thr - p.immediate_stop_margin))',
     '(self.ch_val < -1.0))', 'confirmed :551'),
    ('M05 해제 거리 = 정지 거리',
     'lidar_clear = ~has(CAUSE_LIDAR) | (lidar_main > p.stop_release_distance)',
     'lidar_clear = ~has(CAUSE_LIDAR) | (lidar_main > p.emergency_stop_distance)', ':788'),
    ('M06 존 히스테리시스 끔', '(d_main > boundary + h)', '(d_main > boundary)', ':817'),
    ('M07 창 가지치기 없음(창 통계의 최소)', 'm = torch.where(fin, torch.maximum(values, s), inf_t)',
     'm = torch.where(fin, s, inf_t)', 'physicalWindowStatistic :255'),
    ('M08 가드 창 순위 0.6', '0.5, self.circular, self.half_cap)',
     '0.6, self.circular, self.half_cap)', 'robustBeams :367'),
    ('M09 자기 차체 필터 끔', 'valid = torch.isfinite(rx) & torch.isfinite(ry) & ~self._is_self(rx, ry)',
     'valid = torch.isfinite(rx) & torch.isfinite(ry)', ':702'),
    ('M10 지연 보정 끔', 'use = self._meas_fresh(now) & (stamp >= 0.0)',
     'use = torch.zeros_like(self.meas_stamp, dtype=torch.bool)', 'latencyCorrection :381-390'),
    ('M11 탈출 허용오차 0', 'ok = ok & ~(g < g_now - ESCAPE_TOLERANCE)', 'ok = ok & ~(g < g_now)',
     'escapeAllowed :575'),
    ('M12 탈출 예측 끝 단계만', '''        ok = torch.ones(self.n, dtype=torch.bool, device=self.device)
        fin = torch.isfinite(now_d)
        for step in range(1, ESCAPE_STEPS + 1):''', '''        ok = torch.ones(self.n, dtype=torch.bool, device=self.device)
        fin = torch.isfinite(now_d)
        for step in range(ESCAPE_STEPS, ESCAPE_STEPS + 1):''', 'escapeAllowed :568'),
    ('M13 후퇴에 점군 신선 조건 없음',
     'ret_need = ~stale & ~self.prox & ttc_binds & p.allow_escape & cloud_fresh & \\',
     'ret_need = ~stale & ~self.prox & ttc_binds & p.allow_escape & \\', ':930'),
    ('M14 접근 판정의 해제 거리 항 없음',
     '((has(CAUSE_LIDAR) | has(CAUSE_DEPTH)) & (d_main <= p.stop_release_distance)) | \\',
     '\\', ':908'),
    ('M15 예외 여유 상한 없음',
     'ucx = torch.minimum(ucx, clearance_speed_limit(d_excl, p.exclusion_stop_distance, p))',
     'ucx = ucx', ':836-838'),
    ('M16 낡은 필수 점군 전진 상한 없음',
     'v_fwd = torch.where(cloud_stale, torch.full_like(u_cap, p.degraded_mode_max_speed), inf_n)',
     'v_fwd = inf_n', ':844-847'),
    ('M17 옆 여유 안 점 b_real 규칙 없음 (swept_margin 0 이라 동치 예측)',
     'bsel = torch.where(beside & (py.abs() <= b), torch.full_like(py, b_real),',
     'bsel = torch.where(beside & (py.abs() <= b) & False, torch.full_like(py, b_real),',
     'approach :179'),
    ('M18 꼬리 직선 없음', 's_arc = torch.where(torch.isfinite(s_arc), s_arc, s_tail)',
     's_arc = s_arc', 'approach :185-192'),
    ('M19 원호 각 상한 없음',
     's_arc = torch.where((best > col(self.arc_angle)) | (rho < 1e-9), inf, rho * best)',
     's_arc = torch.where(rho < 1e-9, inf, rho * best)', 'arcApproach :163'),
    ('M20 영역 길이 상한 없음', 's = torch.where(s <= col(self.total), s, inf)', 's = s',
     'approach :194'),
    ('M21 측정 저역 통과 끔', 'direct = (dt <= 0.0) | (dt > p.measured_velocity_timeout) | (tau <= 0.0)',
     'direct = torch.ones_like(dt, dtype=torch.bool)', 'setMeasuredVelocity :414'),
    ('M22 명령 낡음 끔',
     'cmd_fresh = (self.input_stamp >= 0.0) & (now - self.input_stamp <= p.command_timeout)',
     'cmd_fresh = self.input_stamp >= 0.0', ':670'),
    ('M23 깊이 해제에 점군 신선 조건 없음',
     'depth_clear = ~has(CAUSE_DEPTH) | (cloud_fresh & (cloud_main > p.stop_release_distance))',
     'depth_clear = ~has(CAUSE_DEPTH) | (cloud_main > p.stop_release_distance)', ':789-790'),
    ('M24 새 프레임 아니어도 계수',
     'new_scan = (self.scan_stamp >= 0.0) & (self.scan_stamp != self.counted_scan_stamp)',
     'new_scan = self.scan_stamp >= 0.0', ':738'),
    ('M25 양보 최소 이득 없음', 'res = ok & (prev >= start + p.yield_escape_min_gain)', 'res = ok',
     'yieldEscapeAllowed :641'),
    ('M26 창 순위 −1e-9 없음',
     'k = torch.ceil(support_ratio * length.to(ranges.dtype) - 1e-9).long().clamp(min=1)',
     'k = torch.ceil(support_ratio * length.to(ranges.dtype)).long().clamp(min=1)', ':262'),
    ('M27 TTC 정적 트랙 포함', '''        if p.ttc_only_dynamic:
            use = use & (self.tracks[..., 1] > 0.5)
        mt = ''', '''        mt = ''', 'minTrackTtc :428'),
    ('M28 새 래치에서 옛 원인 유지',
     'base = torch.where(self.prox, self.causes, torch.zeros_like(self.causes))',
     'base = self.causes', ':777-779'),
]
EXPECT_EQUIV = {
    'M17 옆 여유 안 점 b_real 규칙 없음 (swept_margin 0 이라 동치 예측)':
        '배포 설정 swept_margin = 0 이면 b = b_real 이라 두 갈래가 같다 (safety_gate.cpp:70, 179).',
}


def main():
    """변이마다 사본 → 교차 시험."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True, help='저장소 스냅숏 루트 (rl/isaac 가 있는 곳)')
    ap.add_argument('--binary', required=True)
    ap.add_argument('--work', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=4242)
    ap.add_argument('--only', default='')
    a = ap.parse_args()
    base = os.path.join(a.src, 'rl', 'isaac')
    orig = open(os.path.join(base, F), encoding='utf-8').read()
    results = []
    for name, old, new, where in MUTANTS:
        if a.only and not name.startswith(a.only):
            continue
        n = orig.count(old)
        if n != 1:
            raise SystemExit(f'{name}: 원본 조각이 {n} 번 나온다')
        d = os.path.join(a.work, name.split()[0])
        shutil.rmtree(d, ignore_errors=True)
        shutil.copytree(base, os.path.join(d, 'rl', 'isaac'),
                        ignore=shutil.ignore_patterns('__pycache__'))
        shutil.copytree(os.path.join(a.src, 'rl', 'contract'), os.path.join(d, 'rl', 'contract'),
                        ignore=shutil.ignore_patterns('__pycache__'))
        for sub in ('src', 'config'):
            if os.path.isdir(os.path.join(a.src, sub)):
                os.symlink(os.path.join(a.src, sub), os.path.join(d, sub))
        with open(os.path.join(d, 'rl', 'isaac', F), 'w', encoding='utf-8') as f:
            f.write(orig.replace(old, new))
        t0 = time.time()
        xc = os.path.join(d, 'rl', 'isaac', 'tools', 'phi_crosscheck', 'crosscheck.py')
        r = subprocess.run([sys.executable, xc, '--binary', a.binary, '--quick', '--seed',
                            str(a.seed), '--out', os.path.join(d, 'xcheck.json')],
                           capture_output=True, text=True, env={**os.environ, 'XCHECK_TMP': d})
        rec = {'mutant': name, 'where': where, 'xcheck_rc': r.returncode,
               'xcheck_s': round(time.time() - t0, 1)}
        try:
            rep = json.load(open(os.path.join(d, 'xcheck.json')))
            rec['mismatches'] = {k: v for k, v in rep['mismatches'].items() if v}
        except (OSError, ValueError):
            rec['stderr_tail'] = r.stderr[-600:]
        caught = r.returncode != 0
        if not caught:
            u = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
                                'tests/test_safety_gate_torch.py', '-k', 'not cpp'],
                               cwd=os.path.join(d, 'rl', 'isaac'), capture_output=True, text=True)
            rec['unit_rc'] = u.returncode
            rec['unit_tail'] = u.stdout.strip().splitlines()[-1:] if u.stdout else []
            caught = u.returncode != 0
        rec['caught'] = caught
        rec['expected_equivalent'] = EXPECT_EQUIV.get(name)
        results.append(rec)
        tag = 'CAUGHT  ' if caught else 'SURVIVED'
        print(f'{tag} {name} ({rec["xcheck_s"]} s)', flush=True)
        shutil.rmtree(d, ignore_errors=True)
    summary = {'n': len(results), 'caught': sum(r['caught'] for r in results),
               'survived': [r['mutant'] for r in results if not r['caught']],
               'survived_unexpected': [r['mutant'] for r in results
                                       if not r['caught'] and not r['expected_equivalent']],
               'results': results}
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=1, ensure_ascii=False)
    print(json.dumps({k: summary[k] for k in ('n', 'caught', 'survived')}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
