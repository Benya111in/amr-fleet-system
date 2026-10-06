"""
합성 결과 데이터 — 집계·판정·그림 코드를 실제 결과 없이 시험하기 위한 것 (**측정값이 아니다**).

참값을 알고 만든다: 비교군별 성공률·접촉률·복귀 시간을 인자로 주면, 판정 코드가 그 참값을
되찾는지 tests/test_report.py 가 본다. 시나리오 시드를 비교군 사이에 공유해 짝 설계를 흉내 내고,
시드 공통 잠재 난도 z 로 성공의 양의 상관(짝짓기 이득의 원천)을 만든다.
"""

import numpy as np
from scipy import stats

from . import schema

DIFFICULTIES = ('open', 'narrow', 'dynamic_low', 'dynamic_high')


def _row(**kw):
    r = {k: None for k in schema.NAMES}
    r['schema_version'] = schema.SCHEMA_VERSION
    r['safety_node'] = 'on'
    r.update(kw)
    return r


def _policies(arm, n_seeds):
    if arm == 'A_dwa':
        return [('dwa@synthetic', None)]
    return [(f'{arm}/s{s}/best', s) for s in range(n_seeds)]


def make_dataset(seed=0, n_seeds=5, levels=40, eps_per_level=50, gz_n=300,
                 gz_collision=None, gz_success=None, gz_return_mean=None,
                 sim2d_skill=None, sim2d_collision=None, isaac=True, isaac_eps=20,
                 exec_index=True, gz_error_rate=0.0, gz_secondary=True):
    """
    합성 데이터셋 → 정규화 행 리스트.

    gz_*: Gazebo 확증 묶음(gz_new600)의 비교군별 참값 dict (A_dwa·C_hybrid·B_ppo).
    sim2d_*: 2D 의 비교군별 기본 성공률·접촉률.
    """
    rng = np.random.default_rng(seed)
    gz_collision = {'A_dwa': 0.0, 'C_hybrid': 0.0, 'B_ppo': 0.01, **(gz_collision or {})}
    gz_success = {'A_dwa': 0.90, 'C_hybrid': 0.95, 'B_ppo': 0.85, **(gz_success or {})}
    gz_return_mean = {'A_dwa': 2.0, 'C_hybrid': 1.2, 'B_ppo': 2.5, **(gz_return_mean or {})}
    sim2d_skill = {'A_dwa': 0.90, 'B_ppo': 0.86, 'C_hybrid': 0.93, **(sim2d_skill or {})}
    sim2d_collision = {'A_dwa': 0.0005, 'B_ppo': 0.01, 'C_hybrid': 0.0003,
                       **(sim2d_collision or {})}
    rows = []
    hard = rng.normal(0.0, 0.06, levels)                   # 레벨 난도 (2D·Isaac 공통)
    diff = [DIFFICULTIES[i % len(DIFFICULTIES)] for i in range(levels)]
    skills = {}
    # ---- 2D: 레벨 × 에피소드, 시나리오 시드는 비교군 공통 (짝)
    for arm in ('A_dwa', 'B_ppo', 'C_hybrid'):
        for pol, s in _policies(arm, n_seeds):
            eff = 0.0 if s is None else rng.normal(0.0, 0.02)
            skills[pol] = sim2d_skill[arm] + eff
            for lv in range(levels):
                p = float(np.clip(skills[pol] - hard[lv], 0.01, 0.999))
                for e in range(eps_per_level):
                    coll = rng.random() < sim2d_collision[arm]
                    ok = (rng.random() < p) and not coll
                    rows.append(_row(
                        arm=arm, policy_id=pol, train_seed=s, simulator='sim2d',
                        scenario_set='sim2d_heldout', scenario_id=f'L{lv:03d}',
                        scenario_seed=lv * 10000 + e, difficulty=diff[lv], episode=0,
                        run_tag='synthetic', commit='synthetic',
                        outcome='collision' if coll else ('success' if ok else 'timeout'),
                        contacts=int(coll), time_s=float(15 + rng.gamma(2.0, 1.5)),
                        min_clearance_m=float(rng.uniform(0.1, 1.5)),
                        interventions=int(rng.poisson(0.3))))
    # ---- Isaac: 같은 레벨 일부, 정책 순위는 2D 와 같은 기술 + 잡음
    if isaac:
        for arm in ('A_dwa', 'B_ppo', 'C_hybrid'):
            for pol, s in _policies(arm, n_seeds):
                for lv in range(0, levels, 2):
                    p = float(np.clip(skills[pol] - hard[lv] - 0.03 + rng.normal(0, 0.01),
                                      0.01, 0.999))
                    for e in range(isaac_eps):
                        ok = rng.random() < p
                        rows.append(_row(
                            arm=arm, policy_id=pol, train_seed=s, simulator='isaac',
                            scenario_set='isaac_heldout', scenario_id=f'L{lv:03d}',
                            scenario_seed=lv * 10000 + e, difficulty=diff[lv], episode=0,
                            run_tag='synthetic', commit='synthetic',
                            outcome='success' if ok else 'timeout', contacts=0,
                            time_s=float(15 + rng.gamma(2.0, 1.5))))
    # ---- Gazebo 확증 묶음: 시나리오 시드 공통, 잠재 난도 z 공유
    z = rng.normal(0.0, 1.0, gz_n)
    arms = ('A_dwa', 'C_hybrid') + (('B_ppo',) if gz_secondary else ())
    for arm in arms:
        pol = 'dwa@synthetic' if arm == 'A_dwa' else f'{arm}/s0/best'
        q = float(stats.norm.ppf(gz_success[arm]))     # lat ~ N(0, 1) 이므로 P(lat < q) = 성공률
        for i in range(gz_n):
            lat = 0.8 * z[i] + 0.6 * rng.normal()
            err = rng.random() < gz_error_rate
            coll = (not err) and rng.random() < gz_collision[arm]
            ok = (not err) and (not coll) and lat < q
            outcome = 'error' if err else ('collision' if coll else
                                           ('success' if ok else 'timeout'))
            rows.append(_row(
                arm=arm, policy_id=pol, train_seed=None if arm == 'A_dwa' else 0,
                simulator='gazebo', scenario_set='gz_new600', scenario_id=f'G{i % 30:02d}',
                scenario_seed=10_000 + i, difficulty=diff[i % levels], episode=0,
                run_tag='synthetic', exec_index=i if exec_index else None,
                commit='synthetic', outcome=outcome, contacts=int(coll),
                time_s=float(15 + rng.gamma(2.0, 1.5)),
                max_dev_m=float(rng.uniform(0.0, 0.9)),
                return_max_s=float(rng.exponential(gz_return_mean[arm])),
                min_clearance_m=float(rng.uniform(0.05, 1.5)),
                min_ttc_s=float(rng.uniform(0.5, 9.0)),
                interventions=int(rng.poisson(0.5)), cte_mean_m=float(rng.uniform(0.01, 0.04))))
    return rows
