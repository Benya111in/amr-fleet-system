"""§5.1.1-5 / §5.1.3 Theorem 3: what a priority switch (i->j)_r -> (j->i)_r must do to the event graph.

The sparse type-2 construction keeps, per resource r, a chain of consecutive users ... p -> i -> j -> s ...
with edges exit(p)->enter(i), exit(i)->enter(j), exit(j)->enter(s).
Swapping the adjacent users i, j must rewire THREE edges:
    remove exit(p)->enter(i), exit(i)->enter(j), exit(j)->enter(s)
    add    exit(p)->enter(j), exit(j)->enter(i), exit(i)->enter(s)
The audited text (rev3 before audit) only did  P - e + (exit(j)->enter(i)),  i.e. single-edge reversal.
This script counts, on random strictly separated prioritized plans, how often the single-edge test
(A) disagrees with the full adjacent-swap test (B), and how often (A) leaves j unordered w.r.t. p.

Result (audit, 2026-09-22): the acyclicity VERDICTS of A and B coincide (0 disagreements) -- expected, because
in any DAG the chain gives ord(exit p) < ord(enter i) < ord(exit i) < ord(enter j) < ord(exit j) < ord(enter s),
so the two extra edges of B (exit p -> enter j, exit i -> enter s) point forward and cannot close a cycle; only
exit(j) -> enter(i) can.  So "DFS for a path enter(i) ~> exit(j) in P - e" is a valid O(|E_P|) test.
But the UPDATE must be B: applying A literally leaves enter(j) without any type-2 predecessor on r (and gives
i and s the same predecessor exit(j)), so the per-resource total order that Theorem 2 relies on is lost.
"""
import random

from event_graph_check import build, has_cycle, windows_conflict, L, EPS  # noqa: F401


def chains(routes):
    users = {}
    for i, seq in routes.items():
        for k, (r, tin, tout, v) in enumerate(seq):
            users.setdefault(r, []).append((tin, i, k))
    for r in users:
        users[r].sort()
    return users


def ex(i, k):
    return ("ex", i, k)


def en(i, k):
    return ("en", i, k)


def main():
    Dr = lambda v: ((L + 0.3 + EPS) / v + 0.35) / 2
    rng = random.Random(7)
    n_pairs = a_acc_b_cyc = a_cyc_b_acc = a_unordered = 0
    n_plans = 0
    while n_plans < 3000:
        R = [f"r{x}" for x in range(5)]
        busy = {r: [] for r in R}
        routes = {}
        for i in range(5):
            path = rng.sample(R, rng.randint(2, 4))
            t = rng.uniform(0, 3)
            seq = []
            for r in path:
                vv = rng.choice([0.5, 1.0, 1.5])
                T = rng.uniform(1, 3)
                pad = Dr(vv)
                while True:
                    clash = [b for a, b in busy[r] if (t - pad < b and a < t + T + pad)]
                    if not clash:
                        break
                    t = max(clash) + pad + 1e-6
                    if seq:
                        pr, ptin, _, pv = seq[-1]
                        seq[-1] = (pr, ptin, t, pv)
                seq.append((r, t, t + T, vv))
                t += T
            for r, a, b, vv in seq:
                busy[r].append((a - Dr(vv), b + Dr(vv)))
            routes[i] = seq
        if windows_conflict(routes, Dr):
            continue
        n_plans += 1
        E, _, _ = build(routes)
        assert not has_cycle(E)
        Eset = set(E)
        for r, us in chains(routes).items():
            for x in range(len(us) - 1):
                (_, i, ki), (_, j, kj) = us[x], us[x + 1]
                if i == j:
                    continue
                p = us[x - 1] if x >= 1 else None
                s = us[x + 2] if x + 2 < len(us) else None
                if (p and p[1] in (i, j)) or (s and s[1] in (i, j)):
                    continue
                n_pairs += 1
                e = (ex(i, ki), en(j, kj))
                A = (Eset - {e}) | {(ex(j, kj), en(i, ki))}
                B = set(Eset)
                B.discard(e)
                B.add((ex(j, kj), en(i, ki)))
                if p:
                    B.discard((ex(p[1], p[2]), en(i, ki)))
                    B.add((ex(p[1], p[2]), en(j, kj)))
                if s:
                    B.discard((ex(j, kj), en(s[1], s[2])))
                    B.add((ex(i, ki), en(s[1], s[2])))
                ca, cb = has_cycle(list(A)), has_cycle(list(B))
                if not ca and cb:
                    a_acc_b_cyc += 1
                if ca and not cb:
                    a_cyc_b_acc += 1
                if not ca and p:
                    # in A, j has no type-2 predecessor on r at all -> order p vs j on r left to runtime race
                    if not any(b == en(j, kj) and a[0] == "ex" for a, b in A if a[1] != j):
                        a_unordered += 1
    print(f"plans={n_plans}, adjacent swap candidates={n_pairs}")
    print(f"single-edge test ACCEPTS but full swap is CYCLIC (unsafe accept): {a_acc_b_cyc}")
    print(f"single-edge test REJECTS but full swap is acyclic (needless reject): {a_cyc_b_acc}")
    print(f"single-edge accepts with a predecessor p present -> j left unordered after p on r: {a_unordered}")
    print("=> verdict test (path enter(i)~>exit(j) in P-e) is valid; the graph update must rewire all 3 edges")


if __name__ == "__main__":
    main()
