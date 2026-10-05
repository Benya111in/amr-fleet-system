"""Summarise c06_results.json into markdown tables + pooled paired comparison (min edge distance)."""
import json, sys, numpy as np
from scipy.stats import wilcoxon
R = json.load(open(sys.argv[1] if len(sys.argv) > 1 else "c06_results.json"))
V = ["pvt", "det", "hp", "pvt3s", "now", "blind"]; S = ["S1", "S2", "S3", "S4", "S5", "S6"]
name = {"pvt": "v2", "det": "A2 Σ≡0", "hp": "A5 HP만", "pvt3s": "A6 v1식 3 s 확률비용", "now": "현재위치만(costmap형)", "blind": "무시(충돌 확인용)"}
print("| 변형 | 시나리오 | 충돌 런 | e-stop 진입 런 | Critical 진입 런 | 최소 가장자리 [m] | 최대 이탈 [m] | 복귀 최대 [s] [측정 런] | 멈춤 최대 [s] | 도달 | 평균 완료 [s] |")
print("|---|---|---|---|---|---|---|---|---|---|---|")
for v in V:
    for s in S:
        X = [r for r in R if r["variant"] == v and r["sid"] == s]
        if not X: continue
        rets = [r["ret"] for r in X if r["ret"] is not None]
        rmax = (("%.2f" % max(rets)) if all(np.isfinite(rets)) else "∞(%d)" % sum(1 for q in rets if not np.isfinite(q))) if rets else "–"
        ntr = sum(1 for r in X if r.get("ret_trunc")); rmax += (" (끝 절단 %d)" % ntr) if ntr else ""
        rmax += " [%d/%d]" % (len(rets), len(X)) if rets else ""
        print("| %s | %s | %d/%d | %d | %d | %.2f | %.2f | %s | %.1f | %d/%d | %.1f |" % (
            name[v], s, sum(r["collided"] for r in X), len(X), sum(r["n_estop"] > 0 for r in X), sum(r["n_crit"] > 0 for r in X),
            min(r["min_edge"] for r in X), max(r["maxdev"] for r in X), rmax, max(r["freeze"] for r in X),
            sum(r["reached"] for r in X), len(X), np.mean([r["t_end"] for r in X])))
print()
key = lambda r: (r["sid"], r["seed"])
base = {key(r): r for r in R if r["variant"] == "pvt"}
for v in ["det", "hp", "pvt3s", "now"]:
    pairs = [(base[key(r)]["min_edge"], r["min_edge"]) for r in R if r["variant"] == v and key(r) in base]
    pairs_t = [(base[key(r)]["t_end"], r["t_end"]) for r in R if r["variant"] == v and key(r) in base]
    if not pairs: continue
    a = np.array(pairs); d = a[:, 0] - a[:, 1]; b = np.array(pairs_t); dt = b[:, 0] - b[:, 1]
    p = wilcoxon(d, alternative="two-sided").pvalue if np.any(d != 0) else 1.0
    pt = wilcoxon(dt, alternative="two-sided").pvalue if np.any(dt != 0) else 1.0
    print("v2 vs %-22s n=%d  min-edge diff median %+.3f m (p=%.3g) | completion-time diff median %+.2f s (p=%.3g)" % (name[v], len(d), np.median(d), p, np.median(dt), pt))
t = [r["tcomp_ms_p50"] for r in R if r["variant"] == "pvt"]; print("numpy planner p50 per cycle (median over runs): %.1f ms (Python, loaded host; not representative of C++)" % np.median(t))
