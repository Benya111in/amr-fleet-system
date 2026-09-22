"""c06: kinematic prototype of the revised PVT-DWA (v2) to test the brief's DEFAULT parameters for
freezing / collisions / deviation / return time in set-piece scenarios S1-S6.
NOT a substitute for the Gazebo evaluation: no LiDAR, no costmap latency, no localisation error,
tracker = GT + Gaussian noise at 10 Hz, obstacles are non-reactive scripts.

Variants
  pvt   : revised design (min(HP,BX) bound, passive-safety hard check on brake tail, TTC/risk costs)
  det   : Sigma == 0 (deterministic predictive, GVO-style)             -> ablation A2
  hp    : half-plane bound only (bound of brief v1)                     -> ablation A5
  now   : no prediction, obstacles frozen at current position (costmap-only behaviour, DWB/MPPI-like)
usage: python3 c06_proto_sim.py [n_seeds] [procs]
"""
import sys, json, time
import numpy as np
from scipy.special import ndtr
from scipy.ndimage import distance_transform_edt
from multiprocessing import Pool

# ---------------- parameters (the brief's defaults, config/robot_params.yaml limits) ----------------
V_MIN, V_MAX, W_MAX, A_V, ALPHA_W, JERK = -0.5, 2.0, 1.5, 1.0, 2.0, 2.0
DT_C, NV, NW = 0.05, 11, 31
DT_S, T_PRED, T_COMMIT = 0.1, 5.0, 0.20          # rollout step, GVO prediction horizon, commit (dt_c + t_react)
R_ROBOT_C, ROBOT_OFF = 0.25, 0.15                 # robot 2-circle cover (exact cover of 0.60 x 0.40)
M_HARD = 0.30                                     # hard chance check margin = safety.emergency_stop_distance
M_SOFT = 0.50                                     # soft TTC/risk margin     = safety.critical_zone_distance
CLS = {"person": dict(r=0.25, q=0.2, offs=(0.0,)), "forklift": dict(r=0.60, q=0.1, offs=(-0.667, 0.0, 0.667))}
SP, SV = 0.05, 0.2                                # model sigma_p, sigma_v (class default fallback)
DELTA_HARD, DELTA_TTC, T_ACT, T_RISK = 0.05, 0.05, 2.5, 1.5
BAND, BAND_RELAX = 0.8, 1.0
W = dict(h=0.8, c=1.0, v=0.4, p=1.2, t=1.5, r=1.0)
M_RISK = 0.30                                     # risk index margin (= hard margin)
CONT_PARALLEL = True                              # GVO rollout: arc for T_sim, then path-parallel
RES = 0.05
CLEAR_REL = True                                  # J_clear = excess over the reference path's own cost
WINDOW_ON_CMD = True                              # V_d centred on last command (not measured velocity)
FP = np.array([(x, y) for x in np.linspace(-0.3, 0.3, 7) for y in (-0.2, 0.2)] +
              [(x, y) for x in (-0.3, 0.3) for y in (-0.1, 0.0, 0.1)])      # 20 perimeter pts


def tsim(v): return min(max(abs(v) / A_V + 0.5, 1.5), 2.5)


class World:
    def __init__(self, rects, xlim=(-5, 45), ylim=(-12, 12)):
        self.x0, self.y0 = xlim[0], ylim[0]
        nx, ny = int((xlim[1] - xlim[0]) / RES), int((ylim[1] - ylim[0]) / RES)
        occ = np.zeros((nx, ny), bool)
        xs = self.x0 + (np.arange(nx) + 0.5) * RES; ys = self.y0 + (np.arange(ny) + 0.5) * RES
        for (a, b, c, d) in rects:
            occ[np.ix_((xs >= a) & (xs <= c), (ys >= b) & (ys <= d))] = True
        self.occ = occ; self.edt = distance_transform_edt(~occ) * RES; self.nx, self.ny = nx, ny

    def idx(self, x, y):
        i = np.clip(((x - self.x0) / RES).astype(int), 0, self.nx - 1)
        j = np.clip(((y - self.y0) / RES).astype(int), 0, self.ny - 1)
        return i, j

    def occ_at(self, x, y): i, j = self.idx(x, y); return self.occ[i, j]
    def dist_at(self, x, y): i, j = self.idx(x, y); return self.edt[i, j]


# ---------------- obstacles (ground truth) ----------------
class Obs:
    def __init__(self, cls, fn, shape):
        self.cls, self.fn, self.shape = cls, fn, shape       # fn(t) -> (pos(2), vel(2))

    def state(self, t): return self.fn(t)


def line(p0, u, t0=0.0):
    p0, u = np.array(p0, float), np.array(u, float)
    return lambda t: (p0 + u * max(t - t0, 0.0), u.copy() if t >= t0 else np.zeros(2))


def circle_path(c, r, speed, phase0, ccw=True):
    c = np.array(c, float); om = speed / r * (1 if ccw else -1)
    def f(t):
        ph = phase0 + om * t
        p = c + r * np.array([np.cos(ph), np.sin(ph)])
        v = r * om * np.array([-np.sin(ph), np.cos(ph)])
        return p, v
    return f


def random_walker(p0, speed, seed, robot_ref, T=80.0, dt=0.1, t_start=0.0, dturn=np.pi / 3):
    rng = np.random.default_rng(seed); n = int(T / dt) + 1
    P = np.zeros((n, 2)); V = np.zeros((n, 2)); p = np.array(p0, float)
    hd = (np.pi / 2 if p0[1] < 0 else -np.pi / 2) + rng.uniform(-np.pi / 4, np.pi / 4)   # initially toward the path
    state = {"P": P, "V": V, "k": 0, "p": p, "hd": hd}
    def f(t):
        k = min(int(max(t - t_start, 0.0) / dt), n - 1)
        while state["k"] < k:           # lazily generate so the rejection rule can see the robot
            kk = state["k"]
            if kk % 10 == 0:            # heading change every 1 s, reject headings aimed at the robot
                for _ in range(20):
                    cand = state["hd"] + (rng.uniform(-dturn, dturn) if kk > 0 else 0.0)
                    u = speed * np.array([np.cos(cand), np.sin(cand)])
                    rp = robot_ref["p"]
                    tt = np.linspace(0, 2.0, 21)
                    if np.min(np.linalg.norm(state["p"] + np.outer(tt, u) - rp, axis=1)) > 0.36 + 0.25 + 0.5:
                        break
                state["hd"] = cand
            u = speed * np.array([np.cos(state["hd"]), np.sin(state["hd"])])
            state["p"] = state["p"] + u * dt; P[kk + 1] = state["p"]; V[kk + 1] = u; state["k"] += 1
        return (P[k] if k > 0 else np.array(p0, float)), (V[k] if k > 0 else np.zeros(2))
    return f


# ---------------- geometry for GT metrics ----------------
RP = np.array([(x, y) for x in np.linspace(-0.3, 0.3, 13) for y in (-0.2, 0.2)] +
              [(x, y) for x in (-0.3, 0.3) for y in np.linspace(-0.2, 0.2, 9)])


def rect_pts(cx, cy, th, L, Wd, n=12):
    xs = np.linspace(-L / 2, L / 2, n); ys = np.linspace(-Wd / 2, Wd / 2, max(n // 2, 5))
    loc = np.array([(x, s * Wd / 2) for x in xs for s in (-1, 1)] + [(s * L / 2, y) for s in (-1, 1) for y in ys])
    c, s = np.cos(th), np.sin(th)
    return np.c_[cx + c * loc[:, 0] - s * loc[:, 1], cy + s * loc[:, 0] + c * loc[:, 1]]


def inside_rect(pts, cx, cy, th, L, Wd):
    c, s = np.cos(th), np.sin(th); d = pts - [cx, cy]
    lx = c * d[:, 0] + s * d[:, 1]; ly = -s * d[:, 0] + c * d[:, 1]
    return np.any((np.abs(lx) <= L / 2) & (np.abs(ly) <= Wd / 2))


def edge_dist(rx, ry, rth, ob, t):
    p, v = ob.state(t)
    rpts = rect_pts(rx, ry, rth, 0.6, 0.4, 13)
    if ob.shape[0] == "circle":
        r = ob.shape[1]
        if inside_rect(p[None, :], rx, ry, rth, 0.6, 0.4): return -r
        return np.min(np.linalg.norm(rpts - p, axis=1)) - r
    L, Wd = ob.shape[1], ob.shape[2]; th = np.arctan2(v[1], v[0]) if np.linalg.norm(v) > 1e-3 else ob.shape[3]
    opts = rect_pts(p[0], p[1], th, L, Wd, 21)
    if inside_rect(opts, rx, ry, rth, 0.6, 0.4) or inside_rect(rpts, p[0], p[1], th, L, Wd): return 0.0
    return np.min(np.linalg.norm(rpts[:, None, :] - opts[None, :, :], axis=2))


# ---------------- planner ----------------
def brake_profile(v0, tgrid):
    """speed along track: hold v0 for T_COMMIT, then jerk-limited decel (a: 0 -> -A_V at JERK)."""
    sgn = np.sign(v0); v0 = abs(v0); vs = np.empty_like(tgrid); v, a = v0, 0.0; dt = tgrid[1] - tgrid[0]
    for i, t in enumerate(tgrid):
        vs[i] = v
        if t >= T_COMMIT and v > 0:
            a = max(a - JERK * dt, -A_V); v = max(v + a * dt, 0.0)
    return sgn * vs


class Planner:
    def __init__(self, variant):
        self.variant = variant; self.recover_until = -1; self.clear_cnt = 0; self.active = False; self.pass_side = None
        self.last_cmd = None
        self.tg = np.arange(0, 2.62, 0.02)                    # tail integration grid
        self.tail_idx = np.arange(5, len(self.tg), 5)         # tail poses every 0.1 s

    def bound(self, d, sig, R, prob=True):
        if not prob or self.variant in ("det", "now"): return (d < R).astype(float)
        hp = ndtr((R - d) / sig)
        if self.variant == "hp": return hp
        return np.minimum(hp, (hp - ndtr((-R - d) / sig)) * (2 * ndtr(R / sig) - 1))   # isotropic box bound

    def dmin(self, X, Y, TH, T, tracks):
        """per track: min centre distance over robot/obstacle cover circles, sigma(t), base radius r_R + r_j."""
        out = []
        cx = [X + s * ROBOT_OFF * np.cos(TH) for s in (1, -1)]; cy = [Y + s * ROBOT_OFF * np.sin(TH) for s in (1, -1)]
        for tr in tracks:
            c = CLS[tr["cls"]]; tt = T + tr["age"]
            if self.variant == "now": tt = 0 * tt
            sig = np.sqrt(SP ** 2 + tt ** 2 * SV ** 2 + c["q"] * tt ** 3 / 3)
            u = tr["u"]; sp = np.linalg.norm(u); hd = u / sp if sp > 0.2 else np.array([1.0, 0.0])
            offs = list(c["offs"]); rb = R_ROBOT_C + c["r"]
            if len(offs) > 1 and sp <= 0.2: offs = [0.0]; rb = R_ROBOT_C + 1.12          # stationary forklift: 1 circle
            D = np.full(X.shape, np.inf)
            for o in offs:
                ox = tr["p"][0] + u[0] * tt + o * hd[0]; oy = tr["p"][1] + u[1] * tt + o * hd[1]
                for k in range(2): D = np.minimum(D, np.hypot(ox - cx[k], oy - cy[k]))
            out.append((D, sig, rb))
        return out

    def plan(self, st, tracks, world, v_cap, x_goal, tnow):
        x0, y0, th0, va, wa = st["x"], st["y"], st["th"], st["v"], st["w"]
        if WINDOW_ON_CMD and self.last_cmd is not None:
            if abs(self.last_cmd[0] - va) <= 0.3: va = self.last_cmd[0]
            if abs(self.last_cmd[1] - wa) <= 0.3: wa = self.last_cmd[1]
        vdes_nom = min(V_MAX, v_cap)
        vlo = max(V_MIN, va - A_V * DT_C); vhi = max(min(v_cap, V_MAX, va + A_V * DT_C), vlo)
        Vs = np.linspace(vlo, vhi, NV); Ws = np.linspace(max(-W_MAX, wa - ALPHA_W * DT_C), min(W_MAX, wa + ALPHA_W * DT_C), NW)
        V, Wg = np.meshgrid(Vs, Ws, indexing="ij"); V = V.ravel(); Wg = Wg.ravel()
        bv = max(va - A_V * DT_C, 0.0) if va >= 0 else min(va + A_V * DT_C, 0.0)
        bw = wa - np.sign(wa) * min(abs(wa), ALPHA_W * DT_C)
        V = np.r_[V, bv]; Wg = np.r_[Wg, bw]; M = len(V); ib = M - 1
        # cruise rollouts (constant v,w), 30 poses
        t = DT_S * np.arange(1, int(round(T_PRED / DT_S)) + 1); K = len(t)
        TH = th0 + Wg[:, None] * t[None, :]
        small = np.abs(Wg) < 1e-3; Wsafe = np.where(small, 1.0, Wg)
        X = np.where(small[:, None], x0 + V[:, None] * t * np.cos(th0), x0 + (V / Wsafe)[:, None] * (np.sin(TH) - np.sin(th0)))
        Y = np.where(small[:, None], y0 + V[:, None] * t * np.sin(th0), y0 - (V / Wsafe)[:, None] * (np.cos(TH) - np.cos(th0)))
        # beyond T_sim(v): continue parallel to the path at the reached lateral offset (Frenet d = const)
        Ns_v = np.array([int(round(tsim(v) / DT_S)) for v in V])
        if CONT_PARALLEL:
            kk_ = np.arange(K)[None, :]; beyond = kk_ >= Ns_v[:, None]
            idx = np.minimum(Ns_v - 1, K - 1)
            xs_, ys_ = X[np.arange(M), idx], Y[np.arange(M), idx]
            Xc = xs_[:, None] + np.abs(V)[:, None] * np.sign(V)[:, None] * (t[None, :] - t[idx][:, None])
            Xp = np.where(beyond, Xc, X); Yp = np.where(beyond, ys_[:, None], Y); THp = np.where(beyond, 0.0, TH)
        else:
            Xp, Yp, THp = X, Y, TH
        # static collision along cruise (footprint points)
        c, s = np.cos(TH)[..., None], np.sin(TH)[..., None]
        PX = X[..., None] + c * FP[:, 0] - s * FP[:, 1]; PY = Y[..., None] + s * FP[:, 0] + c * FP[:, 1]
        coll = world.occ_at(PX, PY).any(-1)                                    # (M,K)
        kc = np.where(coll.any(1), coll.argmax(1), K)                           # first colliding pose
        # brake tails (vectorised): speed profile per unique v, constant curvature while braking
        G = len(self.tg); dtg = np.diff(self.tg)
        VP = np.empty((M, G))
        for key in np.unique(np.round(V, 5)):
            VP[np.isclose(V, key, atol=1e-5)] = brake_profile(key, self.tg)
        SD = np.concatenate([np.zeros((M, 1)), np.cumsum(0.5 * (VP[:, 1:] + VP[:, :-1]) * dtg, 1)], 1)
        mov = np.abs(V) > 0.02
        kap = np.where(mov, Wg / np.where(mov, V, 1.0), 0.0)
        wprof = np.where(self.tg[None, :] < T_COMMIT, Wg[:, None],
                         np.sign(Wg)[:, None] * np.maximum(np.abs(Wg)[:, None] - ALPHA_W * (self.tg[None, :] - T_COMMIT), 0))
        THs = th0 + np.concatenate([np.zeros((M, 1)), np.cumsum(0.5 * (wprof[:, 1:] + wprof[:, :-1]) * dtg, 1)], 1)
        THP = np.where(mov[:, None], th0 + kap[:, None] * SD, THs)
        cx_, sx_ = VP * np.cos(THP), VP * np.sin(THP)
        XS = x0 + np.concatenate([np.zeros((M, 1)), np.cumsum(0.5 * (cx_[:, 1:] + cx_[:, :-1]) * dtg, 1)], 1)
        YS = y0 + np.concatenate([np.zeros((M, 1)), np.cumsum(0.5 * (sx_[:, 1:] + sx_[:, :-1]) * dtg, 1)], 1)
        TX, TY = XS[:, self.tail_idx], YS[:, self.tail_idx]; TT = np.broadcast_to(self.tg[self.tail_idx], TX.shape)
        stopped = VP == 0; tstop = np.where(stopped.any(1), self.tg[stopped.argmax(1)], self.tg[-1])
        tmask = TT <= tstop[:, None] + 1e-9; tmask[:, 0] = True
        s_stop = np.abs(SD[:, -1]); thT_tail = THP[:, self.tail_idx]
        # admissibility vs static obstacles (Fox et al.: stop before the first collision)
        s_c = np.abs(V) * t[np.minimum(kc, K - 1)] * (kc < K) + 1e9 * (kc >= K)
        adm = np.where(np.abs(V) > 0.02, s_c > s_stop + 0.05, (kc >= K) | (t[np.minimum(kc, K - 1)] > 0.3))
        # cost poses: first N_s = T_sim(v)/dt poses and before static collision
        Ns = np.array([int(round(tsim(v) / DT_S)) for v in V])
        kk = np.arange(K)[None, :]; cmask = (kk < Ns[:, None]) & (kk < kc[:, None]); cmask[:, 0] = True
        dmask = kk < kc[:, None]; dmask[:, 0] = True
        e = np.abs(Y); e0 = abs(y0)
        emax = np.where(cmask, e, 0).max(1)
        # dynamic terms
        if tracks:
            Dc = self.dmin(Xp, Yp, THp, t[None, :] + 0 * X, tracks)
            Dt = self.dmin(TX, TY, thT_tail, TT, tracks)
            if self.variant == "pvt3s":        # brief-v1-like soft terms: probabilistic TTC_delta and risk over 3 s
                m3 = dmask & (t[None, :] <= 3.0 + 1e-9)
                Pc = [self.bound(D, s, rb + M_SOFT) * m3 for (D, s, rb) in Dc]
                hit = np.max(np.stack(Pc), 0) >= DELTA_TTC
                risk = 1 - np.prod([1 - p.max(1) for p in Pc], axis=0)
            else:                              # deterministic GVO-TTC (mean prediction) + short-horizon risk index
                hit = np.max(np.stack([(D < rb + M_SOFT) for (D, s, rb) in Dc]), 0) & dmask
                rmask = dmask & (t[None, :] <= T_RISK + 1e-9)
                risk = 1 - np.prod([1 - (self.bound(D, s, rb + M_RISK) * rmask).max(1) for (D, s, rb) in Dc], axis=0)
            ttc = np.where(hit.any(1), t[hit.argmax(1)], np.inf)
            ptail = np.max(np.stack([np.where(tmask, self.bound(D, s, rb + M_HARD), 0) for (D, s, rb) in Dt]), 0).max(1)
            thr = DELTA_HARD if self.variant not in ("det", "now") else 0.5
            hard_ok = ptail < thr
            if not hard_ok.any():                              # least-risk fallback (lexicographic): nothing is safe
                hard_ok = ptail <= ptail.min() + 0.02
        else:
            ttc = np.full(M, np.inf); risk = np.zeros(M); hard_ok = np.ones(M, bool); ptail = np.zeros(M)
        # pass-offset reference y_ref (Frenet-style lateral target, hysteresis) for an obstacle moving ALONG the corridor
        # trigger: in corridor, |u_y|<0.3, closing time along the path < T_PRED; need = R_hard + 1.64 sigma(t_eval),
        # t_eval = 1.2 s if co-moving (|u_x - v_pass| < 0.5, stays alongside), else 0.5 s (passes quickly)
        y_ref = 0.0; keep = None; V_PASS = 0.5
        for tr in tracks:
            c = CLS[tr["cls"]]; R = R_ROBOT_C + c["r"] + M_HARD
            ahead = tr["p"][0] - x0; close = max(vdes_nom - tr["u"][0], 0.1)
            if -(R + 0.5) < ahead and ahead / close < T_PRED and abs(tr["p"][1]) < R + 0.3 and abs(tr["u"][1]) < 0.3 \
                    and (tr["u"][0] < 0.5 * max(vdes_nom, 0.1)):
                tp = 1.2 if abs(tr["u"][0] - V_PASS) < 0.5 else 0.5
                need = R + 1.64 * np.sqrt(SP ** 2 + tp ** 2 * SV ** 2 + c["q"] * tp ** 3 / 3)
                cand = [tr["p"][1] + need, tr["p"][1] - need]
                if self.pass_side is not None: cand = [cand[0] if self.pass_side > 0 else cand[1]]
                cand = [yc for yc in cand if abs(yc) <= BAND - 0.05 + 0.05 * (self.pass_side is not None)]   # hysteresis
                if cand: keep = min(cand, key=abs)
        if keep is not None:
            y_ref = keep; self.pass_side = np.sign(keep) if keep != 0 else self.pass_side
        else:
            self.pass_side = None
        # costs
        ell = np.clip(0.8 * np.abs(V) + 0.5, 0.6, 2.0)
        iN = np.minimum(Ns, kc) - 1; iN = np.maximum(iN, 0)
        yN, thN = Y[np.arange(M), iN], TH[np.arange(M), iN]
        Jh = np.abs((np.arctan2(y_ref - yN, ell) - thN + np.pi) % (2 * np.pi) - np.pi) / np.pi
        dgoal = max(x_goal - x0, 0.0); vdes = min(V_MAX, v_cap, np.sqrt(2 * A_V * max(dgoal - 0.2, 0)))
        Jv = np.abs(vdes - V) / (V_MAX - V_MIN)
        Jp = (np.where(cmask, np.minimum(np.abs(Y - y_ref) / BAND, 1), 0).sum(1) / cmask.sum(1))
        dc = world.dist_at(X, Y); cc = np.exp(-3.0 * np.maximum(dc - 0.2, 0))
        cpath = np.exp(-3.0 * np.maximum(world.dist_at(X, 0 * Y) - 0.2, 0))        # cost on the reference path, same station
        Jc = np.where(cmask, np.maximum(cc - cpath, 0) if CLEAR_REL else cc, 0).max(1)
        Jt = np.where(np.isfinite(ttc), 1 - ttc / T_PRED, 0.0)
        wp, wh = W["p"], W["h"]
        if tnow < self.recover_until: wp, wh = 1.5 * wp, 1.2 * wh
        J = wh * Jh + W["c"] * Jc + W["v"] * Jv + wp * Jp + W["t"] * Jt + W["r"] * risk
        valid = adm & hard_ok & (emax <= max(BAND, e0 + 0.02))
        if not valid[:ib].any():
            valid = adm & hard_ok & (emax <= max(BAND_RELAX, e0 + 0.02))
        if not valid.any(): valid[ib] = True                 # brake candidate as last resort
        Jm = np.where(valid, J, np.inf)
        b = int(np.argmin(Jm))
        self.last = dict(y_ref=y_ref, nvalid=int(valid[:ib].sum()), nadm=int(adm[:ib].sum()), nhard=int(hard_ok[:ib].sum()),
                         nband=int((emax[:ib] <= max(BAND, e0 + 0.02)).sum()), b=b, ib=ib, v=float(V[b]), w=float(Wg[b]),
                         ttc=float(ttc[b]), ptail_b=float(ptail[b]),
                         J=dict(h=float(Jh[b]), c=float(Jc[b]), v=float(Jv[b]), p=float(Jp[b]), t=float(Jt[b]), r=float(risk[b])))
        # avoidance state machine / weight schedule
        if np.isfinite(ttc[b]) and ttc[b] < T_ACT: self.active = True; self.clear_cnt = 0
        elif self.active:
            self.clear_cnt += 1
            if self.clear_cnt >= 3: self.active = False; self.recover_until = tnow + 5.0
        if self.recover_until > tnow and abs(y0) < 0.1: self.recover_until = tnow
        self.last_cmd = (float(V[b]), float(Wg[b]))
        return V[b], Wg[b], dict(nvalid=int(valid[:ib].sum()), brake=(b == ib))


# ---------------- scenarios ----------------
def aisle(x0, x1, half, openings=()):
    rects = []
    xs = [x0] + [v for o in openings for v in o] + [x1]
    for a, b in zip(xs[0::2], xs[1::2]):
        rects += [(a, half, b, half + 0.3), (a, -half - 0.3, b, -half)]
    return rects


_NOM = None
def nominal_curve():
    """x(t) of the unobstructed robot with the same planner/profiler (used to time the conflicts)."""
    global _NOM
    if _NOM is None:
        st = dict(x=0.0, y=0.0, th=0.0, v=0.0, w=0.0, a=0.0); pl = Planner("blind"); dt = 0.01; t = 0.0; cmd = (0.0, 0.0)
        world = World([]); L = []
        while t < 30 and st["x"] < 29.7:
            if int(round(t / dt)) % 5 == 0: v, w, _ = pl.plan(st, [], world, V_MAX, 30.0, t); cmd = (v, w)
            a_des = np.clip((cmd[0] - st["v"]) / 0.15, -A_V, A_V); st["a"] += np.clip(a_des - st["a"], -JERK * dt, JERK * dt)
            nv = st["v"] + st["a"] * dt
            if (nv - cmd[0]) * (st["v"] - cmd[0]) < 0: nv = cmd[0]; st["a"] = 0.0
            st["v"] = nv; st["x"] += st["v"] * dt; t += dt; L.append((t, st["x"]))
        _NOM = np.array(L)
    return _NOM


def make(sid, seed, robot_ref):
    rng = np.random.default_rng(1000 + seed); j = lambda s: 1 + rng.uniform(-s, s)
    NC = nominal_curve(); T_nom = lambda x: float(np.interp(x, NC[:, 1], NC[:, 0]))
    if sid == "S1":   # person crossing at 1.0 m/s in open floor
        xc = 14.0; sp = 1.0 * j(0.2); tc = T_nom(xc) + rng.uniform(-0.5, 0.5)
        return World([]), [Obs("person", line((xc, -sp * tc), (0, sp)), ("circle", 0.25))], 30.0
    if sid == "S2":   # head-on forklift 1.5 m/s in 3 m aisle, keeps to its lane (axis 0.8 m from wall)
        sp = 1.5 * j(0.2); xm = 16.0 + rng.uniform(-2, 2); tm = T_nom(xm)
        return World(aisle(-2, 40, 1.5)), [Obs("forklift", line((xm + sp * tm, -0.7), (-sp, 0)), ("box", 2.0, 1.0, np.pi))], 30.0
    if sid == "S3":   # overtaking a 0.3 m/s person walking near the rack side
        return World(aisle(-2, 40, 1.5)), [Obs("person", line((8.0 + rng.uniform(-1, 1), -0.9 + rng.uniform(-0.05, 0.05)), (0.3 * j(0.2), 0)), ("circle", 0.25))], 30.0
    if sid == "S4":   # two random walkers 0.5-1.2 m/s, heading change each 1 s (non-adversarial rule)
        obs = []
        for i, xc in enumerate((10.0, 18.0)):
            xi = xc + rng.uniform(-2, 2); sp = rng.uniform(0.5, 1.2); side = rng.choice([-1, 1]); ts = T_nom(xi) - 2.5 + rng.uniform(-0.5, 0.5)
            obs.append(Obs("person", random_walker((xi, side * 2.5 * sp), sp, 10 * seed + i + 1, robot_ref, t_start=ts, dturn=np.pi / 4), ("circle", 0.25)))
        return World([]), obs, 30.0
    if sid == "S5":   # forklift on a 4 m radius curve crossing the path twice
        sp = 1.2 * j(0.2); cx = 14.0; tc = T_nom(cx - 3.12) + rng.uniform(-0.8, 0.8)
        ph_hit = np.pi + np.arcsin(2.5 / 4.0)                 # CCW crossing of y=0 at x = 14 - 3.12 (moving -y)
        om = sp / 4.0; phase0 = ph_hit - om * tc
        return World([]), [Obs("forklift", circle_path((cx, 2.5), 4.0, sp, phase0), ("box", 2.0, 1.0, 0.0))], 30.0
    if sid == "S6":   # 0.6 m narrow passage (x 12..16) behind a cross-aisle where a person crosses
        rects = aisle(-2, 12, 1.5, openings=((9.5, 11.5),)) + [(12, 0.3, 16, 1.8), (12, -1.8, 16, -0.3)] + aisle(16, 30, 1.5)
        sp = 1.0 * j(0.2); tc = T_nom(10.5) + rng.uniform(-0.5, 0.5)
        return World(rects), [Obs("person", line((10.5, -sp * tc), (0, sp)), ("circle", 0.25))], 24.0
    raise ValueError(sid)


def run(args):
    sid, seed, variant = args
    rng = np.random.default_rng(seed * 7 + sum(map(ord, variant)) % 97)   # stable across processes
    robot_ref = {"p": np.zeros(2)}
    world, obs, x_goal = make(sid, seed, robot_ref)
    st = dict(x=0.0, y=0.0, th=0.0, v=0.0, w=0.0, a=0.0)
    pl = Planner(variant); dt = 0.01; t = 0.0; cmd = (0.0, 0.0); tracks = []; estop = False
    log = []; min_edge = 9.0; collided = False; n_estop = 0; n_crit = 0; freeze = 0.0; tcomp = []
    zone_prev = 0
    while t < 70.0:
        robot_ref["p"] = np.array([st["x"], st["y"]])
        # GT safety zone (dynamic obstacles only, footprint-edge distance)
        ed = min([edge_dist(st["x"], st["y"], st["th"], o, t) for o in obs] + [9.0])
        min_edge = min(min_edge, ed); collided |= ed <= 0.0
        if ed <= 0.30: estop = True
        elif estop and ed > 0.50: estop = False
        zone = 3 if estop else (2 if ed <= 0.5 else (1 if ed <= 1.0 else 0))
        if zone == 3 and zone_prev != 3: n_estop += 1
        if zone == 2 and zone_prev < 2: n_crit += 1
        zone_prev = zone
        if variant == "blind": estop = False; zone = 0
        v_cap = {0: V_MAX, 1: 0.5, 2: 0.2, 3: 0.0}[zone]
        k = int(round(t / dt))
        if k % 10 == 0:   # tracker 10 Hz
            tracks = []
            for o in obs:
                p, u = o.state(t)
                if np.hypot(p[0] - st["x"], p[1] - st["y"]) < 15.0:
                    tracks.append(dict(p=p + rng.normal(0, 0.03, 2), u=u + rng.normal(0, 0.08, 2), cls=o.cls, t=t))
        if k % 5 == 0:    # planner 20 Hz
            trk = [] if variant == "blind" else [dict(tr, age=t - tr["t"] + 0.05) for tr in tracks]
            t0 = time.perf_counter(); v_c, w_c, info = pl.plan(st, trk, world, v_cap, x_goal, t); tcomp.append(time.perf_counter() - t0)
            cmd = (v_c, w_c)
        # velocity profiler (jerk-limited) + safety gate (bypasses profiler when clamping)
        if estop:
            st["a"] = 0.0; st["v"] = max(st["v"] - A_V * dt, 0.0) if st["v"] >= 0 else min(st["v"] + A_V * dt, 0.0)
            st["w"] = st["w"] - np.sign(st["w"]) * min(abs(st["w"]), ALPHA_W * dt)
        else:
            tgt = min(cmd[0], v_cap)
            if st["v"] > v_cap + 1e-6:
                st["a"] = -A_V; st["v"] = max(st["v"] - A_V * dt, v_cap)
            else:
                a_des = np.clip((tgt - st["v"]) / 0.15, -A_V, A_V)
                st["a"] += np.clip(a_des - st["a"], -JERK * dt, JERK * dt)
                nv = st["v"] + st["a"] * dt
                if (nv - tgt) * (st["v"] - tgt) < 0: nv = tgt; st["a"] = 0.0
                st["v"] = nv
            st["w"] += np.clip(cmd[1] - st["w"], -ALPHA_W * dt, ALPHA_W * dt)
        st["x"] += st["v"] * np.cos(st["th"]) * dt; st["y"] += st["v"] * np.sin(st["th"]) * dt; st["th"] += st["w"] * dt
        if world.occ_at(np.array([st["x"]]), np.array([st["y"]]))[0]: collided = True
        if t > 1.0 and abs(st["v"]) < 0.05 and st["x"] < x_goal - 0.5: freeze += dt
        log.append((t, st["x"], st["y"], st["th"], st["v"]))
        t += dt
        if st["x"] >= x_goal - 0.3: break
    L = np.array(log)
    # GT-based avoidance window / return time (planner independent)
    dev = np.abs(L[:, 2]); tt = L[:, 0]
    maxdev = dev.max()
    t_start = tt[np.argmax(dev > 0.10)] if (dev > 0.10).any() else None
    ret = None
    if t_start is not None:
        def clear(ti):
            for s in np.arange(ti, ti + 3.0, 0.1):
                xr = np.interp(s, tt, L[:, 1])            # corridor moves with the robot's GT position at s
                for o in obs:
                    p, _ = o.state(s); rad = 0.25 if o.shape[0] == "circle" else 1.12
                    if xr - 1 - rad <= p[0] <= xr + 5 + rad and abs(p[1]) <= 1.0 + rad: return False
            return True
        t_clear = next((ti for ti in np.arange(t_start, tt[-1], 0.1) if clear(ti)), None)
        if t_clear is not None:
            ok = (dev < 0.10) & (np.abs(L[:, 3]) < np.radians(10))
            idx = np.where(tt >= t_clear)[0]
            t_ret = None
            for i in idx:
                j2 = i + 100
                if ok[i] and (j2 >= len(ok) or ok[i:j2].all()): t_ret = tt[i]; break
            ret = (t_ret - t_clear) if t_ret is not None else float("inf")
    return dict(sid=sid, seed=seed, variant=variant, collided=bool(collided), min_edge=round(float(min_edge), 3),
                n_estop=n_estop, n_crit=n_crit, maxdev=round(float(maxdev), 3), ret=None if ret is None else round(float(ret), 2),
                freeze=round(freeze, 2), t_end=round(float(tt[-1]), 2), reached=bool(L[-1, 1] >= x_goal - 0.3),
                tcomp_ms_p50=round(1e3 * float(np.median(tcomp)), 1))


if __name__ == "__main__":
    ns = int(sys.argv[1]) if len(sys.argv) > 1 else 5; procs = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    variants = sys.argv[3].split(",") if len(sys.argv) > 3 else ["pvt", "det", "hp", "now"]
    scen = sys.argv[4].split(",") if len(sys.argv) > 4 else ["S1", "S2", "S3", "S4", "S5", "S6"]
    jobs = [(s, k, v) for v in variants for s in scen for k in range(ns)]
    with Pool(procs) as p: res = p.map(run, jobs)
    out = "c06_results.json" if len(sys.argv) <= 5 else sys.argv[5]
    json.dump(res, open(out, "w"), indent=0)
    for v in variants:
        for s in scen:
            R = [r for r in res if r["variant"] == v and r["sid"] == s]
            rets = [r["ret"] for r in R if r["ret"] is not None]
            print("%-4s %s coll=%d estop=%d crit=%d minEdge=%.2f maxDev=%.2f ret(max)=%s freeze(max)=%.1f reached=%d/%d t_end(mean)=%.1f" % (
                v, s, sum(r["collided"] for r in R), sum(r["n_estop"] for r in R), sum(r["n_crit"] for r in R), min(r["min_edge"] for r in R),
                max(r["maxdev"] for r in R), (max(rets) if rets else "-"), max(r["freeze"] for r in R), sum(r["reached"] for r in R), len(R),
                np.mean([r["t_end"] for r in R])))
