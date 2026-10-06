#!/usr/bin/env python3
"""
B 단계 기반 코드(b1-binfra)의 가드·불변식 변이 검사 — 하나씩 되돌린 사본에서 시험이 실패하는가.

    python3 mutate_binfra.py --src <저장소 스냅숏> --work <임시 디렉터리> --out <json> [--image cpu|isaac]

스냅숏의 rl/ 을 변이마다 임시 디렉터리로 복사해 파일 한 곳만 바꾼다 (원본·worktree 는 건드리지 않는다). 판정: 지정한
시험 파일들의 pytest 가 실패하면(rc ≠ 0) 잡힘. --image cpu 는 amr-ai/rl:0.3 안에서 (rsl_rl 없음 → ppo_lag 변이는 건너뜀),
--image isaac 은 Isaac 이미지 파이썬에서 ppo_lag 변이만 돈다. 컨테이너 안에서 이 스크립트를 부르면 된다 (README 「시험」).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

# (이름, 파일(rl/isaac 기준), 원본 조각, 변이 조각, 시험 파일들, 이미지)
MUTANTS = [
    ('E1 문턱 경계 > 로', 'amr_isaac/eval_sets.py', 'main = c >= threshold', 'main = c > threshold',
     ['tests/test_eval_sets.py'], 'cpu'),
    ('E2 겹침 0 처리 없음', 'amr_isaac/eval_sets.py', 'return np.where(overlap, 0.0, best)',
     'return best', ['tests/test_eval_sets.py'], 'cpu'),
    ('E3 앞 변 조건 x ≥ 0', 'amr_isaac/eval_sets.py', 'ok = (np.abs(ly) <= half_w) & (lx >= half_l)',
     'ok = (np.abs(ly) <= half_w) & (lx >= 0.0)', ['tests/test_eval_sets.py'], 'cpu'),
    ('E4 모서리 광선 경우 없음', 'amr_isaac/eval_sets.py',
     '    for cy in (-half_w, half_w):\n        cross', '    for cy in ():\n        cross',
     ['tests/test_eval_sets.py'], 'cpu'),
    ('E5 max_dist 컷 없음', 'amr_isaac/eval_sets.py',
     'best = np.where(best <= max_dist, best, inf)', 'best = best', ['tests/test_eval_sets.py'],
     'cpu'),
    ('E6 끝 접선 1 점 앞', 'amr_isaac/eval_sets.py', 'TANGENT_BACK = 5', 'TANGENT_BACK = 1',
     ['tests/test_eval_sets.py'], 'cpu'),
    ('E7 나머지 복원 추출', 'amr_isaac/eval_sets.py',
     'rng.choice(ids, size=rem, replace=False)', 'rng.choice(ids, size=rem, replace=True)',
     ['tests/test_eval_sets.py'], 'cpu'),
    ('E8 주·스트레스 마스크 뒤바뀜', 'amr_isaac/eval_sets.py',
     "(('main', main_mask, n_main), ('stress', stress_mask, n_stress))",
     "(('main', stress_mask, n_main), ('stress', main_mask, n_stress))",
     ['tests/test_eval_sets.py'], 'cpu'),
    ('E9 풀 해시가 점을 빼먹음', 'amr_isaac/eval_sets.py', 'for a in (pts, offs, length):',
     'for a in (offs, length):', ['tests/test_eval_sets.py'], 'cpu'),
    ('E10 군집 대신 에피소드 부트스트랩', 'amr_isaac/eval_sets.py',
     'boots = s[draw].sum(1) / c[draw].sum(1)',
     'boots = x[rng.integers(0, x.size, (reps, x.size))].mean(1)',
     ['tests/test_eval_sets.py'], 'cpu'),
    ('P1 학습 범위 밖 판정 늘 거짓', 'amr_isaac/perturb.py',
     'return value < lo - tol or value > hi + tol', 'return False', ['tests/test_perturb.py'],
     'cpu'),
    ('P2 edge 규칙 없음', 'amr_isaac/perturb.py', "if g == 'edge' and out:", 'if False:',
     ['tests/test_perturb.py'], 'cpu'),
    ('P3 플랜트 지연 격자 검사 없음', 'amr_isaac/perturb.py',
     'if abs(k - round(k)) > 1e-9 or not 1 <= round(k) <= delay_buffer_steps:', 'if False:',
     ['tests/test_perturb.py'], 'cpu'),
    ('P4 지정 안 한 축 = 0', 'amr_isaac/perturb.py', '    out = dict(NOMINAL)\n',
     '    out = {k: 0.0 for k in NOMINAL}\n', ['tests/test_perturb.py'], 'cpu'),
    ('L1 적분 0 아래 허용', 'amr_isaac/pid_lagrange.py',
     'self.integral = min(max(self.integral + self.ki * e, 0.0), self.lmax)',
     'self.integral = min(self.integral + self.ki * e, self.lmax)',
     ['tests/test_pid_lagrange.py'], 'cpu'),
    ('L2 λ 상한 없음', 'amr_isaac/pid_lagrange.py', 'self.lam = min(max(raw, 0.0), self.lmax)',
     'self.lam = max(raw, 0.0)', ['tests/test_pid_lagrange.py'], 'cpu'),
    ('L3 미분항 음수 허용', 'amr_isaac/pid_lagrange.py',
     'D = max(0.0, self.hist[-1] - self.hist[0]) / self.d',
     'D = (self.hist[-1] - self.hist[0]) / self.d', ['tests/test_pid_lagrange.py'], 'cpu'),
    ('L4 오차 정규화 없음', 'amr_isaac/pid_lagrange.py', 'e = (J - self.d) / self.d',
     'e = (J - self.d)', ['tests/test_pid_lagrange.py'], 'cpu'),
    ('L5 비용 이익 표준화', 'amr_isaac/pid_lagrange.py', 'a_c = adv_c - adv_c.mean()',
     'a_c = (adv_c - adv_c.mean()) / (adv_c.std() + 1e-8)', ['tests/test_pid_lagrange.py'],
     'cpu'),
    ('L6 끝나지 않은 env 비용도 합산', 'amr_isaac/pid_lagrange.py',
     'self.sum += float(self.acc[d].sum())', 'self.sum += float(self.acc.sum())',
     ['tests/test_pid_lagrange.py'], 'cpu'),
    ('S1 tanh 보정 없음', 'amr_isaac/sac_skrl.py', 'return (lp - corr).sum(dim=-1, keepdim=True)',
     'return lp.sum(dim=-1, keepdim=True)', ['tests/test_sac_skrl.py'], 'cpu'),
    ('S2 Q 가 raw 를 받음', 'amr_isaac/sac_skrl.py', "a = torch.tanh(inputs['taken_actions'])",
     "a = inputs['taken_actions']", ['tests/test_sac_skrl.py'], 'cpu'),
    ('S3 종단도 현재 관측으로', 'amr_isaac/sac_skrl.py',
     'cut = (truncated.reshape(-1) & ~terminated.reshape(-1)).unsqueeze(-1)',
     'cut = truncated.reshape(-1).unsqueeze(-1)', ['tests/test_sac_skrl.py'], 'cpu'),
    ('S4 고정 통계 갱신', 'amr_isaac/sac_skrl.py', '        if self.frozen:\n            return\n',
     '', ['tests/test_sac_skrl.py'], 'cpu'),
    ('S5 log_prob 가 늘 표본 raw', 'amr_isaac/sac_skrl.py',
     "taken = inputs.get('taken_actions', raw)", 'taken = raw', ['tests/test_sac_skrl.py'], 'cpu'),
    ('X1 PID 끝 λ 늘 활성', 'amr_isaac/policy_export.py', "'active_at_end': lam > 0.0,",
     "'active_at_end': True,", ['tests/test_policy_meta.py'], 'cpu'),
    ('X2 SAC 형식 분기 없음', 'amr_isaac/policy_export.py', "if ck.get('format') == 'amr_sac_v1':",
     'if False:', ['tests/test_sac_skrl.py'], 'cpu'),
    ('G1 승수 이름 lam (GAE λ 덮어씀)', 'amr_isaac/ppo_lag.py',
     '        nz = policy.critic_obs_normalizer\n',
     '        self.lam = self.multiplier\n        nz = policy.critic_obs_normalizer\n',
     ['tests/test_ppo_lag.py'], 'isaac'),
    ('G2 비용 잘림 bootstrap 없음', 'amr_isaac/ppo_lag.py',
     "            cb += self.gamma * self._cv * extras['time_outs']",
     "            cb += 0.0 * extras['time_outs']", ['tests/test_ppo_lag.py'], 'isaac'),
    ('G3 결합에 승수 0', 'amr_isaac/ppo_lag.py',
     'st.advantages = combine_advantages(adv_r, adv_c, self.multiplier)',
     'st.advantages = combine_advantages(adv_r, adv_c, 0.0)', ['tests/test_ppo_lag.py'], 'isaac'),
    ('G4 PID 갱신 안 씀', 'amr_isaac/ppo_lag.py', 'self.multiplier = self.pid.update(J)',
     'self.pid.update(J)', ['tests/test_ppo_lag.py'], 'isaac'),
    ('G5 비용 critic 이 전역 난수', 'amr_isaac/ppo_lag.py',
     'perm = torch.randperm(B, device=self.device, generator=self._gen)',
     'perm = torch.randperm(B, device=self.device)', ['tests/test_ppo_lag.py'], 'isaac'),
]


def main():
    """변이마다 사본·적용·시험."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True, help='저장소 스냅숏 (rl/ 이 있는 곳)')
    ap.add_argument('--work', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--image', default='cpu', choices=('cpu', 'isaac'))
    ap.add_argument('--python', default=sys.executable)
    a = ap.parse_args()
    res = []
    for name, f, old, new, tests, img in MUTANTS:
        if img != a.image:
            continue
        w = os.path.join(a.work, name.split()[0])
        if os.path.exists(w):
            shutil.rmtree(w)
        shutil.copytree(os.path.join(a.src, 'rl'), os.path.join(w, 'rl'),
                        ignore=shutil.ignore_patterns('__pycache__'))
        p = os.path.join(w, 'rl', 'isaac', f)
        s = open(p, encoding='utf-8').read()
        n = s.count(old)
        if n != 1:
            res.append({'mutant': name, 'status': f'원본 조각 {n} 번 (적용 실패)'})
            continue
        open(p, 'w', encoding='utf-8').write(s.replace(old, new))
        t0 = time.time()
        r = subprocess.run([a.python, '-m', 'pytest', '-q', '-x', '-p', 'no:cacheprovider',
                            *tests], cwd=os.path.join(w, 'rl', 'isaac'), capture_output=True,
                           text=True, env={**os.environ, 'PYTHONDONTWRITEBYTECODE': '1'})
        tail = (r.stdout.strip().splitlines() or [''])[-1]
        res.append({'mutant': name, 'file': f, 'tests': tests, 'rc': r.returncode,
                    'caught': r.returncode != 0, 'summary': tail,
                    'seconds': round(time.time() - t0, 1)})
        print(('CAUGHT  ' if r.returncode else 'SURVIVED') + f' {name}: {tail}', flush=True)
        shutil.rmtree(w)
    caught = sum(1 for x in res if x.get('caught'))
    out = {'image': a.image, 'mutants': len(res), 'caught': caught, 'results': res}
    with open(a.out, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, ensure_ascii=False, indent=1)
    print(f'MUTATION {a.image}: {caught}/{len(res)} 잡힘', flush=True)
    return 0 if caught == len(res) else 1


if __name__ == '__main__':
    sys.exit(main())
