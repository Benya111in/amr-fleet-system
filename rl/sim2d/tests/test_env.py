"""
환경 대조 — 시뮬레이션이 만든 실제 상태에서 env 의 관측·마스크·보상이 numpy 기준 구현과 같은가.

golden 은 손으로 만든 17+11 사례뿐이다. 여기서는 env 를 무작위 행동·장애물·DR 로 굴려 생긴 상태를 계약 State 로
바꿔 기준 구현(amr_rl_contract)에 넣고 env 출력과 대조한다.

뒤쪽 시험은 env 수준의 종료 조건을 하나씩 고정한다 (2026-10-06 리뷰 변이 R2·R3·R4·R6 이 놓쳤던 것):
장애물 접촉 = 종단(이동·정지 장애물), 정체 truncation, 시간 상한 truncation 과 final_obs, 스캔·트랙의 장애물
관측 지연.
"""

import math

import numpy as np
import pytest
import torch

from amr_sim2d import kernels as K
from amr_sim2d import obstacles as OB
from amr_sim2d.env import EnvConfig, Sim2DEnv
from amr_sim2d.state_io import states_from_bstate

from amr_rl_contract import mask as RM
from amr_rl_contract import observation as RO
from amr_rl_contract import reward as RR

DEV = 'cuda' if torch.cuda.is_available() else 'cpu'
pytestmark = pytest.mark.skipif(DEV != 'cuda', reason='env 시험은 GPU 에서')


def make(arm='B', n=48, seed=5, compile_=False, mode='mixed', **kw):
    cfg = EnvConfig(num_envs=n, device=DEV, seed=seed, arm=arm, obstacle_mode=mode,
                    path_len=(3.0, 12.0), compile=compile_, n_paths=512, **kw)
    return Sim2DEnv(cfg, log=lambda *a: None)


def act_b(env, g):
    return torch.rand(env.N, 2, device=DEV, generator=g) * 2 - 1


def act_c(env, g):
    logits = torch.rand(env.N, 100, device=DEV, generator=g)
    logits = torch.where(env.mask, logits, torch.full_like(logits, -1.0))
    return logits.argmax(1)


@pytest.mark.parametrize('arm', ['B', 'C'])
def test_obs_and_mask_match_reference_on_sim_states(arm):
    env = make(arm)
    obs = env.reset()
    g = torch.Generator(device=DEV)
    g.manual_seed(0)
    checked = 0
    for k in range(36):
        a = act_b(env, g) if arm == 'B' else act_c(env, g)
        obs, *_ = env.step(a)
        if k % 6 != 5:
            continue
        sts = states_from_bstate(env.bstate(), range(0, env.N, 3))
        for j, st in zip(range(0, env.N, 3), sts):
            ref = RO.build_observation(st)
            err = np.abs(obs[j].double().cpu().numpy() - ref)
            assert err.max() <= 1e-5, (k, j, int(err.argmax()), float(err.max()))
            if arm == 'C':
                m = RM.compute_mask(st)['mask']
                assert env.mask[j].cpu().tolist() == [bool(x) for x in m], (k, j)
            checked += 1
    assert checked >= 90


def test_reward_cost_match_reference():
    env = make('B', mode='random')
    env.reset()
    g = torch.Generator(device=DEV)
    g.manual_seed(1)
    n_cmp = 0
    for k in range(40):
        pre = states_from_bstate(env.bstate())
        obs, rew, term, trunc, info = env.step(act_b(env, g))
        done = (term | trunc).cpu()
        post = states_from_bstate(env.bstate())
        for j in range(env.N):
            if done[j]:
                continue
            a_exec = post[j].prev_actions[0]
            iv = float(info['interv'][j])
            r = RR.step(pre[j], post[j], a_exec, bool(info['collision'][j]),
                        [((math.sqrt(iv), 0.0), (0.0, 0.0))])
            assert float(rew[j]) == pytest.approx(r.reward, abs=1e-4), (k, j)
            assert np.allclose(info['cost'][j].cpu().numpy(), r.cost, atol=1e-6)
            assert bool(term[j]) == r.terminated
            assert post[j].dev.in_dev == r.dev.in_dev
            assert post[j].dev.t_since_peak == pytest.approx(r.dev.t_since_peak, abs=1e-9)
            n_cmp += 1
    assert n_cmp > 1000


def test_terminal_events_and_autoreset():
    env = make('B', n=128, mode='random', n_obstacles=(4, 8))
    env.reset()
    g = torch.Generator(device=DEV)
    g.manual_seed(2)
    seen = {'collision': 0, 'trunc': 0}
    for _ in range(150):
        obs, rew, term, trunc, info = env.step(act_b(env, g))
        assert torch.isfinite(obs).all() and torch.isfinite(rew).all()
        coll = info['collision']
        assert bool((term[coll]).all())                         # 접촉 = 즉시 종단
        assert torch.allclose(info['terms']['collision'][coll].float(),
                              torch.full_like(rew[coll], -30.0))
        assert not bool((term & trunc).any())
        if 'done_idx' in info:
            di = info['done_idx']
            assert bool((env.ep_len[di] == 0).all())             # 리셋됨
            assert info['final_obs'].shape == (di.numel(), obs.shape[1])
            assert torch.allclose(obs[di, 213], (env.d_arc[di] / 10).clamp(max=1).float(),
                                  atol=1e-5)                    # 새 에피소드 관측
        seen['collision'] += int(coll.sum())
        seen['trunc'] += int(trunc.sum())
    assert seen['collision'] > 0


def test_seed_determinism():
    a, b = make('B', seed=9), make('B', seed=9)
    oa, ob = a.reset(), b.reset()
    g1 = torch.Generator(device=DEV)
    g1.manual_seed(3)
    for _ in range(15):
        u = act_b(a, g1)
        oa, ra, *_ = a.step(u)
        ob, rb, *_ = b.step(u)
        assert torch.equal(oa, ob) and torch.equal(ra, rb)


@pytest.mark.slow
def test_compiled_matches_eager():
    """torch.compile 경로 = eager 경로 (inductor argmax 동률 버그 회귀 방지, contract_torch.first_true)."""
    e0, e1 = make('B', compile_=False, seed=4), make('B', compile_=True, seed=4)
    o0, o1 = e0.reset(), e1.reset()
    g = torch.Generator(device=DEV)
    g.manual_seed(4)
    for _ in range(25):
        u = act_b(e0, g)
        o0, r0, *_ = e0.step(u)
        o1, r1, *_ = e1.step(u)
        assert float((o0 - o1).abs().max()) < 1e-4
        assert float((e0.pose - e1.pose).abs().max()) < 1e-9


# ---------------------------------------------------------------- env 수준 종료 조건

def put_obstacle(env, a, b, speed, slot=0):
    """모든 env 의 슬롯 slot 에 작업자(원 r 0.30)를 a→b 왕복(속력 speed, 위상 0)으로 둔다."""
    o = env.obst
    o.active[:, slot] = True
    o.otype[:, slot] = OB.TYPE_WORKER
    o.kind[:, slot] = OB.KIND_CIRCLE
    o.r[:, slot] = OB.WORKER_R
    o.mot[:, slot] = OB.MOT_PINGPONG
    o.a[:, slot] = a
    o.b[:, slot] = b
    o.speed[:, slot] = speed
    o.phase[:, slot] = 0.0


def heading(env):
    th = env.pose[:, 2]
    return env.pose[:, :2].clone(), torch.stack([torch.cos(th), torch.sin(th)], 1)


def first_done(env, act, steps):
    """정해진 스텝 수만큼 굴려 env 마다 처음 끝난 스텝(1 부터, 없으면 0)과 그 스텝의 결말을 모은다."""
    n = env.N
    k_done = torch.zeros(n, dtype=torch.long)
    rec = {}
    for k in range(1, steps + 1):
        _obs, _rew, term, trunc, info = env.step(act)
        if 'done_idx' not in info:
            continue
        new = [int(j) for j in info['done_idx'].cpu() if int(k_done[int(j)]) == 0]
        for j in new:
            k_done[j] = k
            pos = int((info['done_idx'] == j).nonzero()[0, 0])
            rec[j] = {'term': bool(term[j]), 'trunc': bool(trunc[j]),
                      'collision': bool(info['collision'][j]),
                      'collision_moving': bool(info['collision_moving'][j]),
                      'stalled': bool(info['stalled'][j]), 'timeout': bool(info['timeout'][j]),
                      'success': bool(info['success'][j]),
                      'r_coll': float(info['terms']['collision'][j]),
                      'ep': {kk: v[pos] for kk, v in info['episode'].items()},
                      'final_obs': info['final_obs'][pos]}
    return k_done, rec


def test_moving_obstacle_contact_terminates_stationary_robot():
    """서 있는 로봇에 비반응 작업자가 와서 닿으면 그 스텝에 충돌 종단 (−30), 로봇은 안 움직였으니 이동 충돌 아님."""
    kw = dict(mode='none', n=16, seed=21, yaw_noise=0.0)
    env, ctrl = make('B', **kw), make('B', **kw)
    env.reset()
    ctrl.reset()
    xy, hd = heading(env)
    put_obstacle(env, xy + 2.5 * hd, xy, 1.0)        # 2.5 m 앞에서 1 m/s 로 다가온다
    zero = torch.zeros(env.N, 2, device=DEV)
    k_done, rec = first_done(env, zero, 30)
    k_ctrl, _ = first_done(ctrl, zero, 30)
    assert bool((k_ctrl == 0).all())                  # 장애물이 없으면 30 스텝 동안 아무 일도 없다
    # 중심 거리 2.5 − t ≤ 0.30(반길이) + 0.30(반경) ⇔ t ≥ 1.9 s → 19–20 번째 스텝
    assert bool(((k_done >= 19) & (k_done <= 20)).all()), k_done.tolist()
    for j, r in rec.items():
        assert r['collision'] and r['term'] and not r['trunc'] and not r['success'], (j, r)
        assert not r['collision_moving'], j
        assert r['r_coll'] == pytest.approx(-30.0), j
        assert bool(r['ep']['collision']) and int(r['ep']['length']) == int(k_done[j])


def test_static_obstacle_contact_terminates_driving_robot():
    """φ 를 끄고 정지 장애물로 곧장 달리면 장애물 접촉으로 종단 (이동 중 충돌) — 격자 접촉과 구별한다."""
    kw = dict(mode='none', n=32, seed=22, yaw_noise=0.0, safety=False)
    env, ctrl = make('B', **kw), make('B', **kw)
    env.reset()
    ctrl.reset()
    xy, hd = heading(env)
    c = xy + 1.0 * hd
    put_obstacle(env, c, c, 0.0)                      # 중심 1.0 m 앞 → 0.4 m 가면 닿는다
    fwd = torch.tensor([[1.0, 0.0]], device=DEV).repeat(env.N, 1)
    k_done, rec = first_done(env, fwd, 40)
    k_ctrl, _ = first_done(ctrl, fwd, 40)
    # 같은 시드·같은 행동이면 접촉 전 궤적이 같다. 장애물 없는 쪽이 그 스텝까지 안 끝난 env 만 본다
    free = [j for j in range(env.N) if k_ctrl[j] == 0 or k_ctrl[j] > k_done[j] > 0]
    assert len(free) >= env.N // 2, (k_done.tolist(), k_ctrl.tolist())
    for j in free:
        assert int(k_done[j]) > 0, (j, k_done.tolist())       # 장애물 쪽은 끝나야 한다
        r = rec[j]
        assert r['collision'] and r['term'] and r['collision_moving'], (j, r)
        assert r['r_coll'] == pytest.approx(-30.0)
        assert 3 <= int(k_done[j]) <= 25, j


def test_stall_truncation_after_20s_without_motion():
    """행동 0, 장애물 없음 → 정체 20 s 를 넘는 스텝에 truncation (종단 아님, 시간 상한 아님)."""
    env = make('B', mode='none', n=16, seed=23)
    env.reset()
    assert float(env.t_limit.min()) > 25.0               # 경로 ≥ 3 m → 상한 ≥ 29 s
    zero = torch.zeros(env.N, 2, device=DEV)
    k_done, rec = first_done(env, zero, 205)
    k = int(k_done[0])
    assert k in (200, 201) and bool((k_done == k).all()), k_done.tolist()
    for j, r in rec.items():
        assert r['trunc'] and r['stalled'] and not r['term'] and not r['timeout'], (j, r)
        assert not r['collision'] and bool(r['ep']['stalled']) and int(r['ep']['length']) == k
        assert r['final_obs'].shape == (291,) and bool(torch.isfinite(r['final_obs']).all())


def test_timeout_truncation_keeps_final_obs():
    """시간 상한을 1 s 로 줄이면 1 s 를 넘는 스텝에 timeout truncation, final_obs = 리셋 전 관측."""
    kw = dict(mode='none', n=16, seed=24)
    env, twin = make('B', **kw), make('B', **kw)
    env.reset()
    twin.reset()
    env.t_limit[:] = 1.0
    zero = torch.zeros(env.N, 2, device=DEV)
    k_hit, fin, obs_twin = None, None, None
    for k in range(1, 16):
        _o, _r, term, trunc, info = env.step(zero)
        o_tw, *_rest, info_tw = twin.step(zero)
        assert 'done_idx' not in info_tw
        if 'done_idx' in info:
            k_hit, fin, obs_twin = k, info, o_tw
            assert bool(trunc.all()) and not bool(term.any())
            break
    assert k_hit in (10, 11), k_hit
    assert info['done_idx'].numel() == env.N
    assert bool(fin['timeout'].all()) and not bool(fin['stalled'].any())
    assert bool(fin['episode']['timeout'].all())
    # 쌍둥이 env (같은 시드·행동, 상한만 다름) 의 같은 스텝 관측 = 끝난 env 의 final_obs (관측은 상한에 무관)
    assert torch.equal(fin['final_obs'], obs_twin[fin['done_idx']])
    assert bool((env.ep_len == 0).all())                 # 리셋됨


def test_obstacle_scan_and_tracks_use_observation_delay():
    """관측 지연 0.1 s 면 트랙의 장애물 위치 = 0.1 s 전 위치 (스캔과 같은 지연 시각에서 계산)."""
    cfg = EnvConfig(num_envs=16, device=DEV, compile=False, n_paths=512, obstacle_mode='none',
                    path_len=(3.0, 12.0), seed=25)
    cfg.dr_ranges = dict(cfg.dr_ranges, obs_delay=(0.1, 0.1), lidar_sigma=(0.0, 0.0),
                         beam_dropout=(0.0, 0.0), track_drop=(0.0, 0.0),
                         track_pos_sigma=(0.0, 0.0), track_vel_sigma=(0.0, 0.0))
    env = Sim2DEnv(cfg, log=lambda *a: None)
    env.reset()
    xy, hd = heading(env)
    lat = torch.stack([-hd[:, 1], hd[:, 0]], 1)
    put_obstacle(env, xy + 3.0 * hd - 4.0 * lat, xy + 3.0 * hd + 4.0 * lat, 1.0)   # 3 m 앞 가로지름
    zero = torch.zeros(env.N, 2, device=DEV)
    for _ in range(6):
        env.step(zero)
    assert bool((env.obs_delay == 5).all())
    op = env._obst_params()
    past = K.obstacle_state(op, env.t - 0.1)['xy'][:, 0]
    now = K.obstacle_state(op, env.t)['xy'][:, 0]
    assert bool(env.trk_valid[:, 0].all())
    assert float((env.trk[:, 0, :2] - past).abs().max()) < 1e-9
    assert float((now - past).norm(dim=-1).min()) > 0.09        # 1 m/s × 0.1 s
