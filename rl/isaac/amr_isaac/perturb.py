"""
DR 축 목록과 교란 평가 묶음 (Isaac 비의존 — numpy/torch).

학습 DR (env_cfg.DomainRandCfg, 계약 §11 의 부분집합) — 리셋마다 env 별 균일 표본:
  축 2 가속도 동역학: 플랜트 1 차 지연 T_m, 순수 지연 T_d (0.02 s 단위 양자화), 플랜트 가속 한계 배율
  축 5 인지 잡음:     LiDAR 거리 잡음 σ, 빔 드롭아웃 확률
  (계약 §10 시작 yaw 잡음 ±0.5 rad 는 DR 이 아니라 에피소드 생성 규칙이라 DR 을 꺼도 남는다)
학습에 없는 축: 1 맵 다양성(창고 한 장 — 지도 변형 교란도 만들 수 없다), 3 장애물 행동(정적 지도), 4 관측·행동 지연
(행동 지연은 플랜트 순수 지연으로만), 5 의 트랙 누락·위치/속도 잡음(트랙 없음), 적재, 구동 이득, 속도 측정 잡음.

교란 묶음 파일 (rl/isaac/eval_sets/b1_perturb_suite_v1.json) 의 조건마다 params 를 env.set_perturbation 으로 넣는다
(평가 env 는 DR 을 끈 명목값에서 시작해 params 만 덮어쓴다). 'unseen' 조건은 적어도 한 축이 학습 DR 범위 밖이어야
한다 (validate_suite 가 확인 — "학습 때 보지 않은" 을 코드로 지킨다).
"""

import json
import math

import numpy as np

PHYS_DT = 0.02          # 플랜트 서브스텝 (계약 PROFILER_DT)
DECISION_DT = 0.1       # 결정 주기 (계약 DT)
SUITE_FORMAT = 'amr_isaac_perturb_suite_v1'

# 명목값 (DR 끔) — env_cfg.DomainRandCfg 의 nominal_* 와 같아야 한다 (tests/test_perturb.py 가 원문 대조)
NOMINAL = {
    'plant_time_constant_s': 0.08,
    'plant_delay_s': 0.04,
    'accel_limit_scale': 1.0,
    'lidar_sigma_m': 0.03,
    'beam_dropout_p': 0.0,
    'drive_gain': 1.0,
    'obs_scan_delay_s': 0.0,
}
# 학습 DR 범위 (env_cfg.DomainRandCfg) — DR 이 없는 축은 명목값 한 점이 "학습 범위" 다
TRAIN_DR_RANGES = {
    'plant_time_constant_s': (0.048, 0.112),
    'plant_delay_s': (0.02, 0.08),
    'accel_limit_scale': (0.84, 1.0),
    'lidar_sigma_m': (0.01, 0.06),
    'beam_dropout_p': (0.0, 0.02),
    'drive_gain': (1.0, 1.0),
    'obs_scan_delay_s': (0.0, 0.0),
}
KEYS = tuple(NOMINAL)
GROUPS = ('nominal', 'unseen', 'edge')


def delay_steps(seconds, max_steps):
    """플랜트 순수 지연 [s] → 서브스텝 수 (반올림, 1 … max_steps) — env._sample_dr 와 같은 양자화."""
    k = np.rint(np.asarray(seconds, dtype=np.float64) / PHYS_DT).astype(np.int64)
    return np.clip(k, 1, max_steps)


def obs_delay_steps(seconds, max_steps):
    """정책 관측 스캔 지연 [s] → 결정 스텝 수 (반올림, 0 … max_steps)."""
    k = np.rint(np.asarray(seconds, dtype=np.float64) / DECISION_DT).astype(np.int64)
    return np.clip(k, 0, max_steps)


def outside_train_range(key, value, tol=1e-9):
    """값이 학습 DR 범위 밖인가."""
    lo, hi = TRAIN_DR_RANGES[key]
    return value < lo - tol or value > hi + tol


def validate_suite(doc, delay_buffer_steps=10, obs_delay_max_steps=3):
    """
    교란 묶음 문서 확인 — 문제 목록 (빈 목록이면 통과).

    규칙: 형식, 이름 유일, group ∈ GROUPS, params 키 ⊂ KEYS, 'unseen' 은 한 축 이상 학습 범위 밖,
    'edge' 는 모든 축이 학습 범위 안, 'nominal' 은 params 없음, 지연은 버퍼 안·양자화 오차 ≤ 1e-9.
    """
    errs = []
    if doc.get('format') != SUITE_FORMAT:
        errs.append(f'format {doc.get("format")!r}')
    names = [c.get('name') for c in doc.get('conditions', [])]
    if len(set(names)) != len(names):
        errs.append('조건 이름 중복')
    for c in doc.get('conditions', []):
        nm, g, p = c.get('name'), c.get('group'), c.get('params', {})
        if g not in GROUPS:
            errs.append(f'{nm}: group {g!r}')
        bad = set(p) - set(KEYS)
        if bad:
            errs.append(f'{nm}: 모르는 축 {sorted(bad)}')
            continue
        out = [k for k, v in p.items() if outside_train_range(k, v)]
        if g == 'unseen' and not out:
            errs.append(f'{nm}: unseen 인데 학습 범위 밖 축이 없다')
        if g == 'edge' and out:
            errs.append(f'{nm}: edge 인데 학습 범위 밖 축 {out}')
        if g == 'nominal' and p:
            errs.append(f'{nm}: nominal 은 params 가 없어야 한다')
        if 'plant_delay_s' in p:
            v = p['plant_delay_s']
            k = v / PHYS_DT
            if abs(k - round(k)) > 1e-9 or not 1 <= round(k) <= delay_buffer_steps:
                errs.append(f'{nm}: 플랜트 지연 {v} 가 0.02 s 격자·버퍼 밖')
        if 'obs_scan_delay_s' in p:
            v = p['obs_scan_delay_s']
            k = v / DECISION_DT
            if abs(k - round(k)) > 1e-9 or not 0 <= round(k) <= obs_delay_max_steps:
                errs.append(f'{nm}: 관측 지연 {v} 가 0.1 s 격자·버퍼 밖')
        for k in ('accel_limit_scale', 'drive_gain', 'plant_time_constant_s'):
            if k in p and not p[k] > 0:
                errs.append(f'{nm}: {k} ≤ 0')
        for k in ('lidar_sigma_m', 'beam_dropout_p'):
            if k in p and not 0 <= p[k] < (1.0 if k == 'beam_dropout_p' else math.inf):
                errs.append(f'{nm}: {k} 범위')
    return errs


def load_suite(path, **kw):
    """교란 묶음 파일 → dict (validate_suite 통과해야)."""
    with open(path, encoding='utf-8') as f:
        doc = json.load(f)
    errs = validate_suite(doc, **kw)
    if errs:
        raise ValueError('교란 묶음 오류: ' + '; '.join(errs))
    return doc


def resolved_params(cond):
    """조건 → 7 축 전부의 값 (지정 안 한 축 = 명목값)."""
    out = dict(NOMINAL)
    out.update(cond.get('params', {}))
    return out


def override_arrays(conds, cond_index):
    """
    조건 번호 cond_index [N] (env 별) → 축마다 덮어쓸 값 [N] (numpy float64).

    모든 축을 채운다 (지정 안 한 축 = 명목값) — 평가 env 는 DR 을 끈 명목값이라 결과는 같고, 실제로 쓴 값이 기록에 남는다.
    """
    idx = np.asarray(cond_index, dtype=np.int64)
    table = np.array([[resolved_params(c)[k] for k in KEYS] for c in conds], dtype=np.float64)
    return {k: table[idx, j] for j, k in enumerate(KEYS)}
