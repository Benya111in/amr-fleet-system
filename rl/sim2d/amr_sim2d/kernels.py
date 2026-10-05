"""
env 의 뜨거운 경로를 순수 함수로 — torch.compile 이 원소별 연산을 몇 개의 커널로 묶을 수 있게.

모든 함수는 텐서 dict 를 받아 새 dict 를 돌려준다 (입력을 바꾸지 않는다). 같은 함수를 컴파일 없이 부르면
기준 동작이고 (작은 부분집합 리셋, CPU 시험), env 는 고정 N 호출만 컴파일한다.
"""

import math

import torch

from . import contract_torch as T
from . import geometry2d as G
from ._contract import C

INF = math.inf
SUB_DT = C.PROFILER_DT


def profiler_step(p, v_target, w_target, dt=SUB_DT):
    """ProfilerBatch.step 의 순수판. p = dict(lin_v, lin_a, ang_v, ang_a, w_ref, curv)."""
    v_t = v_target.clamp(-0.5, C.V_MAX_HW)
    w_t = w_target.clamp(-C.W_MAX, C.W_MAX)
    lin_v, lin_a = T.jlf_step(p['lin_v'], p['lin_a'], v_t, dt, -0.5, C.V_MAX_HW, C.A_MAX,
                              C.JERK_MAX)
    curv = v_t.abs() > 0.05
    v_t_safe = torch.where(curv, v_t, torch.ones_like(v_t))
    w_des = ((w_t / v_t_safe) * lin_v).clamp(-C.W_MAX, C.W_MAX)
    dw = C.ALPHA_MAX * dt
    w_curv = torch.minimum(torch.maximum(w_des, p['w_ref'] - dw), p['w_ref'] + dw)
    ang_v0 = torch.where(p['curv'], p['w_ref'], p['ang_v'])
    ang_a0 = torch.where(p['curv'], torch.zeros_like(p['ang_a']), p['ang_a'])
    w_nc, a_nc = T.jlf_step(ang_v0, ang_a0, w_t, dt, -C.W_MAX, C.W_MAX, C.ALPHA_MAX,
                            C.ANG_JERK_MAX)
    w_ref = torch.where(curv, w_curv, w_nc)
    return {'lin_v': lin_v, 'lin_a': lin_a, 'ang_v': w_ref,
            'ang_a': torch.where(curv, torch.zeros_like(a_nc), a_nc), 'w_ref': w_ref,
            'curv': curv}


def obstacle_state(o, t):
    """장애물 매개변수 dict o (active, kind, r, box, mot, a, b, speed, phase) 의 시각 t [N] 상태."""
    tt = t[:, None] + o['phase']
    ab = o['b'] - o['a']
    L = torch.linalg.norm(ab, dim=-1)
    Ls = torch.where(L > 1e-6, L, torch.ones_like(L))
    u = ab / Ls[..., None]
    dist = torch.remainder(o['speed'] * tt, 2.0 * Ls)
    fwd = dist <= Ls
    d = torch.where(fwd, dist, 2.0 * Ls - dist)
    sgn = torch.where(fwd, torch.ones_like(d), -torch.ones_like(d))
    moving = (L > 1e-6) & (o['speed'] > 0)
    xy_p = o['a'] + torch.where(moving, d, torch.zeros_like(d))[..., None] * u
    vel_p = torch.where(moving, sgn * o['speed'], torch.zeros_like(d))[..., None] * u
    yaw_p = torch.where(L > 1e-6, torch.atan2(u[..., 1], u[..., 0]), torch.zeros_like(L))
    rc = o['b'][..., 0]
    rcs = torch.where(rc > 1e-6, rc, torch.ones_like(rc))
    om = o['speed'] / rcs
    ang = om * tt
    xy_c = o['a'] + rc[..., None] * torch.stack([torch.cos(ang), torch.sin(ang)], -1)
    vel_c = (rc * om)[..., None] * torch.stack([-torch.sin(ang), torch.cos(ang)], -1)
    circ = o['mot'] == 1
    return {'xy': torch.where(circ[..., None], xy_c, xy_p),
            'vel': torch.where(circ[..., None], vel_c, vel_p),
            'yaw': torch.where(circ, ang + 0.5 * math.pi, yaw_p),
            'kind': o['kind'], 'r': o['r'], 'box': o['box'], 'active': o['active']}


def safety_tick(S, X, v_ref, w_ref, first):
    """안전 필터 φ 한 틱 — 반환 (φ 출력 dict, 갱신된 S 일부)."""
    sp = G.SAFETY
    mv_c = (v_ref.abs() > sp['motion_epsilon_v']) | (w_ref.abs() > sp['motion_epsilon_w'])
    mv_m = (S['v'].abs() > sp['motion_epsilon_v']) | (S['w'].abs() > sp['motion_epsilon_w'])
    intent = torch.where(mv_c[:, None], torch.stack([v_ref, w_ref], 1), S['intent'])
    have = S['have_intent'] | mv_c
    hv = torch.where(mv_c, v_ref, intent[:, 0])
    hw = torch.where(mv_c, w_ref, intent[:, 1])
    use_h = mv_c | (~mv_m & have)
    f32 = torch.float32
    d1 = G.swept_approach(X['scan_px'], X['scan_py'], X['scan_ok'], hv.to(f32), hw.to(f32))
    d1 = torch.where(use_h, d1.to(v_ref.dtype), torch.full_like(v_ref, INF))
    d2 = G.swept_approach(X['scan_px'], X['scan_py'], X['scan_ok'], S['v'].to(f32),
                          S['w'].to(f32))
    d2 = torch.where(mv_m, d2.to(v_ref.dtype), torch.full_like(v_ref, INF))
    D = torch.minimum(d1, d2)
    cnt = torch.where(D <= C.E_STOP_DIST, S['stop_count'] + 1, torch.zeros_like(S['stop_count']))
    cnt = torch.where(first, cnt, S['stop_count'])
    prox = S['prox'] | (first & (cnt >= 2))
    prox = prox & ~(D > sp['stop_release_distance'])
    out = T.safety_filter(v_ref, w_ref, D, X['min_ttc'], prox)
    return out, {'intent': intent, 'have_intent': have, 'stop_count': cnt, 'prox': prox}


def plant_step(S, X, vs, ws):
    """순수 지연(버퍼 보간) → 1 차 지연 → 가속 한계 → 정확 원호 적분 (충돌한 env 는 정지)."""
    frozen = S['collided']
    dbuf = torch.cat([torch.stack([vs, ws], 1)[:, None, :], S['dbuf'][:, :-1]], dim=1)
    nb = dbuf.shape[1]
    dsteps = (X['td'] / SUB_DT).clamp(0.0, nb - 1.001)
    i0 = torch.floor(dsteps).long()
    fr = (dsteps - i0.to(dsteps.dtype))[:, None]
    b0 = dbuf.gather(1, i0[:, None, None].expand(-1, 1, 2))[:, 0]
    b1 = dbuf.gather(1, (i0 + 1)[:, None, None].expand(-1, 1, 2))[:, 0]
    tgt = b0 + fr * (b1 - b0)
    alpha = 1.0 - torch.exp(-SUB_DT / X['tm'])
    lim_v = C.A_MAX * SUB_DT * X['acc_scale']
    lim_w = C.ALPHA_MAX * SUB_DT * X['acc_scale']
    dv = torch.maximum(torch.minimum((tgt[:, 0] - S['v']) * alpha, lim_v), -lim_v)
    dw = torch.maximum(torch.minimum((tgt[:, 1] - S['w']) * alpha, lim_w), -lim_w)
    v0, w0 = S['v'], S['w']
    v = torch.where(frozen, torch.zeros_like(v0), v0 + dv)
    w = torch.where(frozen, torch.zeros_like(w0), w0 + dw)
    x, y, th = T.integrate_arc(0.5 * (v0 + v), 0.5 * (w0 + w), SUB_DT)
    pose = S['pose']
    c, s = torch.cos(pose[:, 2]), torch.sin(pose[:, 2])
    newp = torch.stack([pose[:, 0] + c * x - s * y, pose[:, 1] + s * x + c * y,
                        T.wrap_angle(pose[:, 2] + th)], dim=1)
    return {'dbuf': dbuf, 'v': v, 'w': w, 'pose': torch.where(frozen[:, None], pose, newp)}


def substep(S, X, kf, first, safety_on: bool):
    """
    50 Hz 서브스텝 한 번. kf = k/5 (0-d 텐서), first = 첫 서브스텝 (0-d bool 텐서, 새 스캔 프레임).

    S 키: pose v w lin_v lin_a ang_v ang_a w_ref curv w_ref_prev dbuf hist intent have_intent
          stop_count prox collided coll_moving interv zone speed_limit
    X 키: v_cmd w_cmd v1 w1 scan_px scan_py scan_ok min_ttc tm td acc_scale t obst grid map_meta
          map_idx
    """
    out = dict(S)
    prof = {k: S[k] for k in ('lin_v', 'lin_a', 'ang_v', 'ang_a', 'w_ref', 'curv')}
    out['w_ref_prev'] = S['w_ref']
    prof = profiler_step(prof, X['v_cmd'] + kf * (X['v1'] - X['v_cmd']),
                         X['w_cmd'] + kf * (X['w1'] - X['w_cmd']))
    out.update(prof)
    v_ref, w_ref = prof['lin_v'], prof['w_ref']
    if safety_on:
        sf, upd = safety_tick(S, X, v_ref, w_ref, first)
        out.update(upd)
        vs, ws = sf['v'], sf['w']
        out['interv'] = S['interv'] + ((v_ref - vs) / C.INTERVENTION_V_NORM) ** 2 + \
            ((w_ref - ws) / C.INTERVENTION_W_NORM) ** 2
        out['zone'] = sf['zone']
        out['speed_limit'] = sf['speed_limit']
    else:
        vs, ws = v_ref, w_ref
    out.update(plant_step(S, X, vs, ws))
    t_sub = X['t'] + kf * C.DT
    ob = obstacle_state(X['obst'], t_sub)
    hit = G.rect_hits_grid(X['grid'], X['map_meta'], X['map_idx'], out['pose']) | \
        G.rect_hits_obstacles(out['pose'], ob)
    new = hit & ~S['collided']
    out['coll_moving'] = S['coll_moving'] | (new & (out['v'].abs() > C.GOAL_STOP_SPEED))
    collided = S['collided'] | hit
    out['collided'] = collided
    out['v'] = torch.where(collided, torch.zeros_like(out['v']), out['v'])
    out['w'] = torch.where(collided, torch.zeros_like(out['w']), out['w'])
    out['hist'] = torch.cat([out['pose'][:, None, :], S['hist'][:, :-1]], dim=1)
    return out


def min_ttc(st):
    """안전 필터의 동적 트랙 최소 TTC — 관측과 같은 계약 TTC (전방 직진 v_meas)."""
    _o, ok, _d, p, v, _dyn, _sp = T.select_tracks(st, C.TRACK_RANGE)
    ttc = T.track_ttc(p[..., 0], p[..., 1], v[..., 0], v[..., 1], st.v_meas[:, None])
    ttc = torch.where(ok, ttc, torch.full_like(ttc, INF))
    if ttc.shape[1] == 0:
        return torch.full_like(st.v_meas, INF)
    return ttc.min(dim=1).values


def post_step(st, P, a_prev, a_exec, collided, interv):
    """
    전이 끝: 투영·보상·비용·이탈 갱신·정체·시간 상한·관측.

    st: 전이 뒤 BState (dev 는 전이 전 값), P: dict(d_arc, base_xy, base_t, t, t_limit).
    반환 dict: obs, reward, cost, terminated, truncated, success, stalled, timeout, dev(in, peak, t),
    d_arc, base_xy, base_t, proj.
    """
    proj = T.project_onto_path(st.path_pts, st.path_n, st.path_cum, st.pose[:, :2])
    d_post = (st.path_len - proj[0]).clamp_min(0.0)
    gdist = torch.linalg.norm(st.pose[:, :2] - T.goal_xy(st.path_pts, st.path_n), dim=-1)
    r = T.reward_step(P['d_arc'], d_post, proj[1], gdist, st.v_meas, a_prev, a_exec, collided,
                      (st.dev_in, st.dev_peak, st.dev_t), interv)
    moved = torch.linalg.norm(st.pose[:, :2] - P['base_xy'], dim=-1) >= C.STALL_RADIUS
    base_xy = torch.where(moved[:, None], st.pose[:, :2], P['base_xy'])
    base_t = torch.where(moved, P['t'], P['base_t'])
    term = r['terminated']
    stalled = ((P['t'] - base_t) > C.STALL_TIME) & ~term
    timeout = (P['t'] > P['t_limit']) & ~term
    st2 = T.BState(**{**st.__dict__, 'dev_in': r['dev'][0], 'dev_peak': r['dev'][1],
                      'dev_t': r['dev'][2]})
    obs = T.build_observation(st2, proj)
    return {'obs': obs, 'reward': r['reward'], 'cost': r['cost'], 'terminated': term,
            'truncated': stalled | timeout, 'success': r['success'], 'stalled': stalled,
            'timeout': timeout, 'dev': r['dev'], 'd_arc': d_post, 'base_xy': base_xy,
            'base_t': base_t, 'proj': proj, 'terms': r['terms']}


def track_obs(ob, psig, vsig, pdrop, noise_xy, noise_v, u_drop):
    """지면 진실 장애물 → 트랙 (위치·속도 잡음, Bernoulli 누락). 난수는 밖에서 넣는다."""
    xy = ob['xy'] + psig[:, None, None] * noise_xy
    vel = ob['vel'] + vsig[:, None, None] * noise_v
    drop = u_drop < pdrop[:, None]
    return torch.cat([xy, vel], dim=-1), ob['active'] & ~drop, ob['active']
