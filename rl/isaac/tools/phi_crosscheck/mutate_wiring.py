#!/usr/bin/env python3
"""
정본 φ env 결선·평가 지표 변이 (A3d) — git archive 사본의 rl/isaac 에 변이 하나를 넣는다.

    mutate_wiring.py --list
    mutate_wiring.py --root <사본 루트> --apply E3

리뷰 vacuity 검사(`runs/bonus1/b1-isaac-phi/review/vacuity/vac.py`)와 같은 치환이다
(E5 는 지표를 eval_metrics.nav2_update 로 옮겨서 그 함수와 호출 줄 두 곳으로 나눴다).
검출기: E1–E4 는 Isaac 기록 + replay_env.py (env 안 교차 시험), E2·E3·E4·E5·G3·G4 는 CPU pytest
(tests/test_safety_gate_torch.py). 치환 대상 문자열이 정확히 한 번 나오지 않으면 실패한다
(코드가 바뀌어 변이가 조용히 무효가 되는 것을 막는다).
"""

import argparse
import os
import sys

MUTANTS = {
    'G3': ('해제 거리 = 정지 거리', 'amr_isaac/safety_gate_torch.py',
           'lidar_clear = ~has(CAUSE_LIDAR) | (lidar_main > p.stop_release_distance)',
           'lidar_clear = ~has(CAUSE_LIDAR) | (lidar_main > p.emergency_stop_distance)'),
    'G4': ('지연 보정 끔', 'amr_isaac/safety_gate_torch.py',
           'use = self._meas_fresh(now) & (stamp >= 0.0)',
           'use = torch.zeros_like(self.meas_stamp, dtype=torch.bool)'),
    'E1': ('env: 래치 비트를 관측에 안 넣음', 'amr_isaac/env.py',
           "            self.prox_stop = out['prox']\n", ''),
    'E2': ('결선: 측정 속도 대신 명령', 'amr_isaac/env_math.py',
           'self.gate.set_measured(v_meas, w_meas, now)',
           'self.gate.set_measured(v_ref, w_ref, now)'),
    'E3': ('결선: 스캔 시각을 한 결정 앞으로', 'amr_isaac/env_math.py',
           'self.gate.set_scan(self.beams(ranges.to(F64)), stamp)',
           'self.gate.set_scan(self.beams(ranges.to(F64)), stamp - 0.1)'),
    'E4': ('결선: 리셋 때 게이트 안 비움', 'amr_isaac/env_math.py',
           '        self.gate.reset(ids)\n', '        pass\n'),
    'E5a': ('Nav2 지표 함수: 충돌 스텝 제외 없음', 'amr_isaac/eval_metrics.py',
            'return nav2 | (~got & (d_goal <= radius) & ~collision)',
            'return nav2 | (~got & (d_goal <= radius))'),
    'E5b': ('Nav2 지표 호출: 함수 대신 충돌 제외 없는 식', 'scripts/eval_unbiased.py',
            "nav2 = EV.nav2_update(nav2, got, sd['d_goal'], sd['collision'], args.nav2_radius)",
            "nav2 = nav2 | ((~got) & (sd['d_goal'] <= args.nav2_radius))"),
}


def apply(root, name):
    """root/rl/isaac 의 대상 파일에 변이 name 을 넣는다."""
    _desc, rel, old, new = MUTANTS[name]
    path = os.path.join(root, 'rl', 'isaac', rel)
    with open(path, encoding='utf-8') as f:
        txt = f.read()
    n = txt.count(old)
    if n != 1:
        raise SystemExit(f'{name}: 치환 대상이 {n} 번 나온다 ({rel})')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt.replace(old, new))


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--root')
    ap.add_argument('--apply')
    a = ap.parse_args()
    if a.list:
        for k, (d, rel, _o, _n) in MUTANTS.items():
            print(f'{k}\t{rel}\t{d}')
        return 0
    if not a.root or a.apply not in MUTANTS:
        ap.error('--root 와 --apply <이름> 이 필요하다')
    apply(a.root, a.apply)
    print(f'applied {a.apply}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
