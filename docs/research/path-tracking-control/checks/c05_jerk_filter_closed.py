"""Closed-form discrete 3rd-order filter (one-step landing on the switching surface), proposed as the
replacement of the brief's pseudocode. Verifies |a|<=A, |j|<=J, no overshoot, 0->2 m/s time,
jerk sign changes, and diagnoses where the brief's version violates the jerk bound."""
import numpy as np, importlib.util, sys, os
A, J = 1.0, 2.0
here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("c04", os.path.join(here, "c04_jerk_filter.py"))
c04 = importlib.util.module_from_spec(spec); sys.stdout = open(os.devnull, "w"); spec.loader.exec_module(c04); sys.stdout = sys.__stdout__

def step_closed(v, a, vr, dt):
    if abs(vr - v) <= J*dt*dt and abs(a) <= J*dt:          # jerk-feasible terminal snap (|j| <= J)
        return vr, 0.0
    c = vr - v - a*dt/2                                   # target for a1|a1|/(2J) + a1*dt/2
    a1s = np.sign(c)*J*(-dt/2 + np.sqrt(dt*dt/4 + 2*abs(c)/J))
    jj = np.clip((a1s - a)/dt, -J, J)
    a1 = np.clip(a + jj*dt, -A, A)
    v1 = v + 0.5*(a + a1)*dt
    return v1, a1

def run(vref_fn, T, dt, v0=0.0, a0=0.0):
    v, a = v0, a0; V, Acc, Jk = [], [], []
    for k in range(int(round(T/dt))):
        a_prev = a; v, a = step_closed(v, a, vref_fn(k*dt), dt)
        V.append(v); Acc.append(a); Jk.append((a - a_prev)/dt)
    return np.array(V), np.array(Acc), np.array(Jk)

if __name__ == "__main__":
    for dt in [0.01, 0.02]:
        V, Acc, Jk = run(lambda t: 2.0, 5.0, dt)
        k = np.argmax(np.abs(V - 2.0) < 1e-6); sc = np.sum(np.abs(np.diff(np.sign(np.round(Jk[int(3/dt):], 9)))) > 0)
        print(f"CLOSED dt={dt}: t(|v-2|<1e-6)={(k+1)*dt:.3f}s max|a|={np.abs(Acc).max():.4f} max|j|={np.abs(Jk).max():.4f} "
              f"overshoot={V.max()-2:.1e} final a={Acc[-1]:.1e} jerk sign changes after 3 s={sc}")
        for name, f, kw in [("2.0->1.1", lambda t: 1.1, dict(v0=2.0)), ("reversal", lambda t: 2.0 if t < 0.8 else 0.5, {}),
                            ("ramp+stop", lambda t: min(2.0, 0.8*t) if t < 4 else 0.0, {})]:
            V2, A2, J2 = run(f, 8.0, dt, **kw)
            print(f"CLOSED dt={dt} {name}: max|j|={np.abs(J2).max():.4f} max|a|={np.abs(A2).max():.4f} final v={V2[-1]:.5f} final a={A2[-1]:.1e}")
        # random step sequences
        rng = np.random.default_rng(0); worst_j = 0; worst_a = 0
        for trial in range(200):
            times = np.sort(rng.uniform(0, 10, 6)); vals = rng.uniform(-0.5, 2.0, 6)
            f = lambda t, times=times, vals=vals: vals[np.searchsorted(times, t) - 1] if t >= times[0] else 0.0
            V2, A2, J2 = run(f, 14.0, dt)
            worst_j = max(worst_j, np.abs(J2).max()); worst_a = max(worst_a, np.abs(A2).max())
        print(f"CLOSED dt={dt}: 200 random step sequences: max|j|={worst_j:.4f}, max|a|={worst_a:.4f}")

    # diagnose the brief's violation
    V, Acc, Jk = c04.filt_brief(lambda t: 2.0, 5.0, 0.01)
    k = np.argmax(np.abs(Jk)); print(f"BRIEF: max|j| {abs(Jk[k]):.2f} at t={k*0.01:.2f}s, a before {Acc[k-1]:.4f} -> after {Acc[k]:.4f}, v {V[k]:.4f}")
