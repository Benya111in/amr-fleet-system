"""c08: one-factor-at-a-time attribution of the three prototype changes made between run2 and run3/run4
(S2 head-on forklift e-stops, S3 overtaking never reached), plus the window-centre acceleration check of §3.2.

Configs (all start from the run4 defaults of c06_proto_sim.py, variant 'pvt', same seeds/scenarios):
  F    : final (path-parallel GVO extension, closing-time pass trigger, risk index on hard radius, w_r=1)
  A    : F but constant-curvature arc extension to T_pred           (CONT_PARALLEL=False)
  B    : F but first-draft pass trigger 'obstacle ahead < 6 m'       (PASS_TRIGGER='dist6')
  C    : F but risk index on soft radius (margin 0.50) with w_r=2   (M_RISK=0.5, W['r']=2)
  ABC  : all three reverted (approximates run2; run2's exact source was not archived)
usage: python3 c08_attrib.py [n_seeds] [procs]
"""
import sys, json
import numpy as np
from multiprocessing import Pool
import c06_proto_sim as S

CFG = {"F": {}, "A": {"CONT_PARALLEL": False}, "B": {"PASS_TRIGGER": "dist6"},
       "C": {"M_RISK": 0.5, "W_r": 2.0},
       "ABC": {"CONT_PARALLEL": False, "PASS_TRIGGER": "dist6", "M_RISK": 0.5, "W_r": 2.0}}


def job(args):
    cfg, sid, seed = args
    for k, v in CFG[cfg].items():
        if k == "W_r": S.W["r"] = v
        else: setattr(S, k, v)
    r = S.run((sid, seed, "pvt")); r["cfg"] = cfg
    return r


def accel_check():
    """time to 1.95 m/s and to x = 29.7 m for the unobstructed robot, window centred on the command vs the measurement."""
    out = {}
    for on in (True, False):
        S.WINDOW_ON_CMD = on; S._NOM = None
        st = dict(x=0.0, y=0.0, th=0.0, v=0.0, w=0.0, a=0.0); pl = S.Planner("blind"); dt = 0.01; t = 0.0; cmd = (0.0, 0.0)
        world = S.World([]); t_v = None
        while t < 40 and st["x"] < 29.7:
            if int(round(t / dt)) % 5 == 0: v, w, _ = pl.plan(st, [], world, S.V_MAX, 30.0, t); cmd = (v, w)
            a_des = np.clip((cmd[0] - st["v"]) / 0.15, -S.A_V, S.A_V); st["a"] += np.clip(a_des - st["a"], -S.JERK * dt, S.JERK * dt)
            nv = st["v"] + st["a"] * dt
            if (nv - cmd[0]) * (st["v"] - cmd[0]) < 0: nv = cmd[0]; st["a"] = 0.0
            st["v"] = nv; st["x"] += st["v"] * dt; t += dt
            if t_v is None and st["v"] >= 1.95: t_v = t
        out["cmd_centre" if on else "meas_centre"] = dict(t_to_1p95=None if t_v is None else round(t_v, 2), t_to_29p7m=round(t, 2))
    S.WINDOW_ON_CMD = True; S._NOM = None
    return out


if __name__ == "__main__":
    print("accel check:", accel_check(), flush=True)
    ns = int(sys.argv[1]) if len(sys.argv) > 1 else 8; procs = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    jobs = [(c, s, k) for c in CFG for s in ("S2", "S3") for k in range(ns)]
    with Pool(procs, maxtasksperchild=1) as p: res = p.map(job, jobs, chunksize=1)
    json.dump(res, open("c08_results.json", "w"), indent=0)
    for c in CFG:
        for s in ("S2", "S3"):
            X = [r for r in res if r["cfg"] == c and r["sid"] == s]
            rets = [r["ret"] for r in X if r["ret"] is not None and np.isfinite(r["ret"])]
            print("%-4s %s estop_runs=%d/%d crit_runs=%d coll=%d reached=%d/%d minEdge=%.2f maxDev=%.2f ret_max=%s freeze_max=%.1f t_end_mean=%.1f" % (
                c, s, sum(r["n_estop"] > 0 for r in X), len(X), sum(r["n_crit"] > 0 for r in X), sum(r["collided"] for r in X),
                sum(r["reached"] for r in X), len(X), min(r["min_edge"] for r in X), max(r["maxdev"] for r in X),
                ("%.2f" % max(rets)) if rets else "-", max(r["freeze"] for r in X), np.mean([r["t_end"] for r in X])), flush=True)
