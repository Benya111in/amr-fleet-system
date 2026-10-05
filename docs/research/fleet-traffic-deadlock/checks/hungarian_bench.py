"""§3.1: rectangular Hungarian (shortest augmenting path, O(n^2 m), n<=m) in pure Python.
Correctness vs brute force (n<=5, m<=7) and timing for 5x50 and a padded 50x50 square.
Run inside amr-fleet-system:wf-final as well to compare with scipy linear_sum_assignment.
"""
import itertools
import random
import time

INF = float("inf")


def hungarian_rect(c):
    n, m = len(c), len(c[0])
    assert n <= m
    u = [0.0] * (n + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)       # p[j] = row matched to column j (1-based), 0 = free
    way = [0] * (m + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [INF] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta, j1 = INF, 0
            ci = c[i0 - 1]
            ui0 = u[i0]
            for j in range(1, m + 1):
                if not used[j]:
                    cur = ci[j - 1] - ui0 - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    assign = [None] * n
    for j in range(1, m + 1):
        if p[j]:
            assign[p[j] - 1] = j - 1
    return assign, sum(c[i][assign[i]] for i in range(n))


def brute(c):
    n, m = len(c), len(c[0])
    return min(sum(c[i][cols[i]] for i in range(n)) for cols in itertools.permutations(range(m), n))


def main():
    rng = random.Random(0)
    for _ in range(200):
        n = rng.randint(1, 5)
        m = rng.randint(n, 7)
        c = [[rng.uniform(0, 100) for _ in range(m)] for _ in range(n)]
        _, val = hungarian_rect(c)
        assert abs(val - brute(c)) < 1e-6
    print("rectangular Hungarian == brute force on 200 random n<=5, m<=7: OK")

    def bench(n, m, reps=200):
        cs = [[[rng.uniform(0, 600) for _ in range(m)] for _ in range(n)] for _ in range(reps)]
        t = time.perf_counter()
        for c in cs:
            hungarian_rect(c)
        return (time.perf_counter() - t) / reps * 1e3
    t_rect = bench(5, 50)
    t_sq = bench(50, 50, reps=20)
    print(f"5x50 rectangular: {t_rect:.3f} ms/solve ; 50x50 square (padding approach): {t_sq:.2f} ms/solve")
    try:
        import numpy as np
        from scipy.optimize import linear_sum_assignment
        for _ in range(300):
            c = np.random.rand(5, 50) * 600
            r, cc = linear_sum_assignment(c)
            _, val = hungarian_rect(c.tolist())
            assert abs(val - c[r, cc].sum()) < 1e-6
        print("rectangular Hungarian == scipy linear_sum_assignment on 300 random 5x50: OK")
    except ImportError as e:
        print("scipy not available here:", e)


if __name__ == "__main__":
    main()


def audit_2026_09_22():
    """Correlated costs (all robots prefer the same few tasks) force multi-step augmentations: worst-ish case."""
    rng = random.Random(3)
    reps = 300
    cs = []
    for _ in range(reps):
        base = [rng.uniform(0, 600) for _ in range(50)]
        cs.append([[b + rng.uniform(0, 5) for b in base] for _ in range(5)])
    t = time.perf_counter()
    for c in cs:
        hungarian_rect(c)
    print(f"5x50 correlated costs: {(time.perf_counter() - t) / reps * 1e3:.3f} ms/solve (budget <= 2 ms)")


if __name__ == "__main__":
    audit_2026_09_22()
