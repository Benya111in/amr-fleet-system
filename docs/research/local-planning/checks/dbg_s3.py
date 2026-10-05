import sys
import numpy as np, c06_proto_sim as S
orig = S.Planner.plan
rec = []
def plan(self, st, tracks, world, v_cap, x_goal, tnow):
    out = orig(self, st, tracks, world, v_cap, x_goal, tnow)
    if abs(tnow*4 - round(tnow*4)) < 1e-6 and float(sys.argv[4]) <= tnow < float(sys.argv[5]):
        L = self.last; tr = tracks[0] if tracks else None
        print("t=%5.1f x=%5.2f y=%5.2f v=%4.2f vcap=%.1f | obs x=%5.2f y=%5.2f | yref=%5.2f nval=%3d adm=%3d hard=%3d band=%3d brake=%d ttc=%.2f ptail=%.3f J=%s" % (
            tnow, st["x"], st["y"], st["v"], v_cap, tr["p"][0] if tr else 0, tr["p"][1] if tr else 0, L["y_ref"], L["nvalid"], L["nadm"], L["nhard"], L["nband"],
            L["b"] == L["ib"], L["ttc"], L["ptail_b"], {k: round(v, 2) for k, v in L["J"].items()}))
    return out
S.Planner.plan = plan
import sys; print(S.run((sys.argv[1], int(sys.argv[2]), sys.argv[3])))
