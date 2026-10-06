"""
동적 장애물 진단 (2차 물결 2b, r6-dyn) — 학습 없이 재는 것들. 근거와 결과: docs/research/rl-navigation/dyn_obstacles.md.

- first_episodes: env 마다 리셋 직후 첫 에피소드 하나의 결말을 env 번호 순서로 모은다 (ppo.evaluate 와 같은 추정량,
  같은 seed 면 정책이 달라도 시작·목표·DR·장애물 배치가 같다 → 정책 사이 짝 비교·"모든 정책이 맞는가" 를 셀 수 있다).
- continuing_values: truncate=False env 에서 정책을 S 스텝 굴려 첫 종단(성공·충돌)까지 할인 보상을 **항별로** 모은다
  (S 를 넘기면 γ^S 꼬리를 버린다). truncation 부트스트랩의 정확한 기준값(무한 지평 가치 V^π(s₀))과 그 분해다.
- ESCAPE: 피할 수 없는 에피소드를 세는 스크립트 정책 묶음 (장애물을 보지 않는 열린 고리 기동). 묶음의 **모든** 정책이
  맞는 에피소드의 비율은 "어떤 정책도 못 피한다" 의 상한 추정이다 (묶음 밖의 더 나은 정책이 피할 수 있으므로).
"""

import math

import torch

from . import heuristic as H
from . import scripted as SP
from ._contract import C

OUTCOME_CODES = ('success', 'collision', 'stall', 'timeout', 'unfinished')


def _vw_u(v_t, w_t, obs):
    """목표 속도 (v*, ω*) → 가속도 행동 u (scripted._u 와 같은 규칙)."""
    return SP._u(v_t, w_t, obs)


def _const(obs, v, w):
    z = torch.zeros(obs.shape[0], device=obs.device, dtype=obs.dtype)
    return z + v, z + w


def follow_v(v_des):
    """경로 추종기 (속도 v_des, 목표 근처 감속)."""
    def pol(obs, t):
        return _vw_u(*SP._follow_target(obs, v_des=v_des), obs)
    return pol


def straight(v):
    """시작 yaw 로 직진 (ω = 0)."""
    def pol(obs, t):
        return _vw_u(*_const(obs, v, 0.0), obs)
    return pol


def turn_go(sign, turn_s=1.0, v=1.0):
    """제자리 회전 ω = ±1.5 를 turn_s 초 → 직진 v."""
    def pol(obs, t):
        tt = t.to(obs.dtype)
        turning = tt < turn_s
        v_t = torch.where(turning, torch.zeros_like(tt), torch.full_like(tt, v))
        w_t = torch.where(turning, torch.full_like(tt, sign * 1.5), torch.zeros_like(tt))
        return _vw_u(v_t, w_t, obs)
    return pol


def follow_stop(t_stop, v_des=H.V_DES):
    """t_stop 초까지 추종 → 정지."""
    def pol(obs, t):
        v, w = SP._follow_target(obs, v_des=v_des)
        stop = t.to(obs.dtype) >= t_stop
        return _vw_u(torch.where(stop, torch.zeros_like(v), v),
                     torch.where(stop, torch.zeros_like(w), w), obs)
    return pol


def gauss(mu_v, mu_w, sigma=math.exp(-0.5), seed=0):
    """
    PPO 초기 정책 꼴의 확률 정책: z ~ N((μ_v, μ_ω), σ²), u = tanh(z) (ActorCriticB 의 log σ₀ = −0.5, μ ≈ 0).

    난수는 env 난수기와 따로 둔다 (env 의 감지 잡음 난수열을 바꾸지 않게).
    """
    g = {}

    def pol(obs, t):
        if 'gen' not in g:
            g['gen'] = torch.Generator(device=obs.device)
            g['gen'].manual_seed(seed)
        n = obs.shape[0]
        z = torch.randn(n, 2, generator=g['gen'], device=obs.device, dtype=obs.dtype) * sigma
        z = z + torch.tensor([mu_v, mu_w], device=obs.device, dtype=obs.dtype)
        return torch.tanh(z)
    return pol


ESCAPE = {
    'idle': SP.idle,
    'follow_0.3': follow_v(0.3), 'follow_0.8': follow_v(0.8), 'follow_1.0': follow_v(1.0),
    'straight_0.5': straight(0.5), 'straight_1.0': straight(1.0),
    'left_go': turn_go(1.0), 'right_go': turn_go(-1.0),
    'follow_stop1': follow_stop(1.0), 'follow_stop2': follow_stop(2.0),
    'follow_stop4': follow_stop(4.0),
}


@torch.no_grad()
def first_episodes(env, policy, seed, max_steps=None):
    """
    env(truncate=True) 의 env 마다 첫 에피소드 결말 → env 번호 순 dict of [N] 텐서.

    policy(obs, t) → 행동 (B: [N,2], C: [N] long — C 면 policy(obs, t, mask)).
    키: outcome (OUTCOME_CODES 번호), time, collision_moving, collision_turning, level, n_obst,
    min_sep, length, progress, spin_frac, cost [N,5].
    """
    env.gen.manual_seed(int(seed))
    obs = env.reset()
    N, dev = env.N, env.dev
    out = {'outcome': torch.full((N,), 4, dtype=torch.long, device=dev),
           'time': torch.full((N,), math.nan, dtype=torch.float64, device=dev)}
    got = torch.zeros(N, dtype=torch.bool, device=dev)
    keys = ('collision_moving', 'collision_turning', 'level', 'n_obst', 'min_sep', 'length',
            'cost', 'd_start', 'd_final', 'spin_steps', 'path_len')
    store = {}
    limit = max_steps or int(math.ceil(float(env.t_limit.max()) / C.DT)) + 5
    for _ in range(limit):
        if env.cfg.arm == 'C':
            u = policy(obs, env.t, env.mask)
        else:
            u = policy(obs, env.t)
        obs, _r, _te, _tr, info = env.step(u)
        if 'done_idx' in info:
            di = info['done_idx']
            new = ~got[di]
            if bool(new.any()):
                e = info['episode']
                ix = di[new]
                oc = torch.full_like(ix, 3)
                oc = torch.where(e['stalled'][new], torch.full_like(oc, 2), oc)
                oc = torch.where(e['collision'][new], torch.full_like(oc, 1), oc)
                oc = torch.where(e['success'][new], torch.full_like(oc, 0), oc)
                out['outcome'][ix] = oc
                out['time'][ix] = e['time'][new].double()
                for k in keys:
                    v = e[k][new]
                    if k not in store:
                        store[k] = torch.zeros((N,) + tuple(v.shape[1:]), dtype=v.dtype,
                                               device=dev)
                    store[k][ix] = v
                got[ix] = True
            if bool(got.all()):
                break
    out.update(store)
    out['finished'] = got
    if 'd_start' in out:
        out['progress'] = 1.0 - out['d_final'].double() / out['d_start'].double().clamp_min(1e-6)
        out['spin_frac'] = out['spin_steps'].double() / out['length'].double().clamp_min(1.0)
    return out


TERMS = ('time', 'shape', 'progress', 'smooth', 'track', 'dev', 'goal', 'collision')


@torch.no_grad()
def continuing_values(env, policy, seed, steps=1200, gamma=C.GAMMA):
    """
    env(truncate=False) 에서 첫 종단(성공·충돌)까지 할인 보상의 항별 합 → dict of [N].

    키: 'V' (합), TERMS 각 항, 'coll' (충돌로 끝남), 'succ', 'disc_end' (종단 스텝의 γ^k, 열림 = 0),
    'interv' (비할인 개입 비용 합), 'spin' (제자리 회전 스텝 비율), 'open' (steps 안에 끝나지 않음).
    충돌 벌점 C 만 바꾼 값은 V + (C₀ − C')·disc_end·coll 로 다시 계산할 수 있다 (C₀ = env 의 spec).
    """
    assert not env.cfg.truncate, 'truncate=False env 에서 잰다'
    env.gen.manual_seed(int(seed))
    obs = env.reset()
    N, dev = env.N, env.dev
    acc = {k: torch.zeros(N, dtype=torch.float64, device=dev) for k in TERMS}
    ended = torch.zeros(N, dtype=torch.bool, device=dev)
    coll = torch.zeros_like(ended)
    succ = torch.zeros_like(ended)
    disc_end = torch.zeros(N, dtype=torch.float64, device=dev)
    interv = torch.zeros(N, dtype=torch.float64, device=dev)
    spin = torch.zeros(N, dtype=torch.float64, device=dev)
    nstep = torch.zeros(N, dtype=torch.float64, device=dev)
    disc = 1.0
    for k in range(steps):
        alive = ~ended
        u = policy(obs, env.t)
        obs, _r, term, _tr, info = env.step(u)
        a = alive.double()
        for name in TERMS:
            if name in info['terms']:
                acc[name] += disc * info['terms'][name].double() * a
        interv += info['interv'].double() * a
        spin += ((env.w.abs() > 1.0) & (env.v.abs() < 0.1)).double() * a
        nstep += a
        now = term & alive
        coll |= now & info['collision']
        succ |= now & info['success']
        disc_end = torch.where(now, torch.full_like(disc_end, disc), disc_end)
        ended |= term
        disc *= gamma
        if bool(ended.all()):
            break
    out = dict(acc)
    out['V'] = sum(acc.values())
    out.update({'coll': coll, 'succ': succ, 'disc_end': disc_end, 'interv': interv,
                'spin': spin / nstep.clamp_min(1.0), 'open': ~ended})
    return out


def summarize_first(rec):
    """first_episodes 결과 → 비율 요약."""
    oc = rec['outcome']
    n = oc.numel()
    s = {'n': n, 'unfinished': int((oc == 4).sum())}
    for i, nm in enumerate(OUTCOME_CODES[:4]):
        s[nm] = float((oc == i).double().mean())
    coll = oc == 1
    if 'collision_moving' in rec:
        s['collision_moving'] = float((coll & rec['collision_moving']).double().mean())
        s['collision_stationary'] = float((coll & ~rec['collision_moving']).double().mean())
        s['collision_turning'] = float((coll & rec['collision_turning']).double().mean())
        ms = rec['min_sep'].double()
        fin = torch.isfinite(ms)
        if bool(fin.any()):
            s['min_sep_median'] = float(ms[fin].median())
        s['interv_mean'] = float(rec['cost'][:, 1].double().mean())
        s['spin_frac'] = float(rec['spin_frac'].mean())
        s['progress'] = float(rec['progress'].mean())
        s['n_obst_mean'] = float(rec['n_obst'].double().mean())
    return s
