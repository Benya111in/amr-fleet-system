"""c08b: does the absolute clearance cost max_k c(p_k) (brief v1) stop the robot in front of the 0.6 m passage (S6)?
Runs variant 'pvt' with run4 defaults except CLEAR_REL=False, S6 x 8 seeds, plus the static-only case (no person: 'blind').
usage: python3 c08b_clear.py [n_seeds] [procs]"""
import sys, json
from multiprocessing import Pool
import c06_proto_sim as S

def job(a):
    rel, variant, seed = a
    S.CLEAR_REL = rel
    r = S.run(("S6", seed, variant)); r["clear_rel"] = rel
    return r

if __name__ == "__main__":
    ns = int(sys.argv[1]) if len(sys.argv) > 1 else 8; procs = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    jobs = [(rel, v, k) for rel in (False,) for v in ("pvt", "blind") for k in range(ns)]
    with Pool(procs, maxtasksperchild=1) as p: res = p.map(job, jobs, chunksize=1)
    json.dump(res, open("c08b_results.json", "w"), indent=0)
    for v in ("pvt", "blind"):
        X = [r for r in res if r["variant"] == v]
        print("CLEAR_REL=False %-5s S6 reached=%d/%d freeze_max=%.1f t_end_mean=%.1f coll=%d" % (
            v, sum(r["reached"] for r in X), len(X), max(r["freeze"] for r in X), sum(r["t_end"] for r in X) / len(X), sum(r["collided"] for r in X)))
