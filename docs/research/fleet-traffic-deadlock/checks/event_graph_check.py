"""§5.1.1/§5.1.3: event graph P on (robot, resource) enter/exit events.

type-1: enter(i,k) -> enter(i,k+1) -> exit(i,k)
type-2: for consecutive users (i then j) of resource r: exit(i,r) -> enter(j,r)
tau(enter(i,k)) = t_in, tau(exit(i,k)) = clear time = t_out + (L/2+eps)/v.
"""
import random

L, EPS = 0.6, 0.08


def build(routes):
    """routes: {robot: [(res, t_in, t_out, v), ...]} -> (nodes, edges, tau)."""
    E, tau = [], {}
    users = {}
    for i, seq in routes.items():
        for k, (r, tin, tout, v) in enumerate(seq):
            tau[("en", i, k)] = tin
            tau[("ex", i, k)] = tout + (L / 2 + EPS) / v
            users.setdefault(r, []).append((tin, i, k))
            if k + 1 < len(seq):
                E.append((("en", i, k), ("en", i, k + 1)))
                E.append((("en", i, k + 1), ("ex", i, k)))
    robot_edges = set()
    for r, us in users.items():
        us.sort()
        for (t1, i, k), (t2, j, m) in zip(us, us[1:]):
            if i != j:
                E.append((("ex", i, k), ("en", j, m)))
                robot_edges.add((i, j))
    return E, tau, robot_edges


def has_cycle(E):
    adj = {}
    for a, b in E:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, [])
    color = {u: 0 for u in adj}

    def dfs(u):
        color[u] = 1
        for w in adj[u]:
            if color[w] == 1 or (color[w] == 0 and dfs(w)):
                return True
        color[u] = 2
        return False
    return any(color[u] == 0 and dfs(u) for u in list(adj))


def robot_cycle(robot_edges):
    return any((j, i) in robot_edges for (i, j) in robot_edges)


def windows_conflict(routes, pad):
    """padded-window overlap on any resource (open-interval predicate a1<b2 and a2<b1)."""
    occ = {}
    for i, seq in routes.items():
        for r, tin, tout, v in seq:
            occ.setdefault(r, []).append((tin - pad(v), tout + pad(v), i))
    for r, ws in occ.items():
        for x in range(len(ws)):
            for y in range(x + 1, len(ws)):
                a1, b1, i = ws[x]
                a2, b2, j = ws[y]
                if i != j and a1 < b2 and a2 < b1:
                    return True
    return False


def main():
    v = 1.0
    # (a) contiguous swap, i first on r1, j first on r2, zero pads (delta_t=0) -> predicate accepts
    swap = {"i": [("r1", 0, 1, v), ("r2", 1, 2, v)], "j": [("r2", 0, 1, v), ("r1", 1, 2, v)]}
    E, tau, re = build(swap)
    print("(a) swap with delta_t=0: predicate conflict =", windows_conflict(swap, lambda v: 0.0),
          "| event-graph cycle =", has_cycle(E))
    Dr = lambda v: ((L + 0.3 + EPS) / v + 0.35) / 2
    print("    same plan with delta_t=Delta_r/2: predicate conflict =", windows_conflict(swap, Dr))
    # (b) non-contiguous: robot-level 2-cycle but time-feasible; event graph must be acyclic
    nc = {"i": [("r1", 0, 1, v), ("r3", 1, 6, v), ("r2", 6, 7, v)],
          "j": [("r2", 0, 1, v), ("r4", 1, 6, v), ("r1", 6, 7, v)]}
    E, tau, re = build(nc)
    print("(b) non-contiguous i:r1->r3->r2, j:r2->r4->r1: robot-level 2-cycle =", robot_cycle(re),
          "| predicate conflict (Delta_r/2) =", windows_conflict(nc, Dr),
          "| event-graph cycle =", has_cycle(E))
    # (c) random prioritized plans with strict separation -> DAG and tau strictly increasing on edges
    rng = random.Random(1)
    bad = 0
    acc = 0
    for trial in range(2000):
        R = [f"r{x}" for x in range(6)]
        busy = {r: [] for r in R}
        routes = {}
        for i in range(4):
            path = rng.sample(R, rng.randint(2, 5))
            t = rng.uniform(0, 5)
            seq = []
            for r in path:
                vv = rng.choice([0.5, 1.0, 1.5])
                T = rng.uniform(1, 4)
                pad = Dr(vv)
                # earliest start with strict separation vs existing padded windows (prioritized)
                while True:
                    ok = all(not (t - pad < b and a < t + T + pad) for a, b in busy[r])
                    if ok:
                        break
                    t = max(b for a, b in busy[r] if (t - pad < b and a < t + T + pad)) + pad + 1e-6
                    # waiting happens in previous resource: extend previous window
                    if seq:
                        pr, ptin, _, pv = seq[-1]
                        seq[-1] = (pr, ptin, t, pv)
                seq.append((r, t, t + T, vv))
                t = t + T
            # re-check previous-window extension did not create conflicts; if so skip plan
            for r, a, b, vv in seq:
                busy[r].append((a - Dr(vv), b + Dr(vv)))
            routes[i] = seq
        if windows_conflict(routes, Dr):
            continue  # extension of a held window collided: SIPP would reject; ignore sample
        acc += 1
        E, tau, _ = build(routes)
        if has_cycle(E) or any(not tau[a] < tau[b] for a, b in E):
            bad += 1
    print("(c) random strictly separated prioritized plans: accepted", acc, "/ 2000 ; cyclic or non-monotone =", bad)


if __name__ == "__main__":
    main()
