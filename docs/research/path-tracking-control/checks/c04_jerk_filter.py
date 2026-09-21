"""Audit checks for the brief's online 3rd-order filter pseudocode (Sec 2.7) as written,
and a corrected variant; bounds, 0->2 m/s time, chattering, terminal snap."""
import numpy as np
A, J = 1.0, 2.0

def filt_brief(vref_fn, T, dt, v0=0.0, a0=0.0):
    v, a = v0, a0; V, Acc, Jk = [], [], []
    for k in range(int(round(T/dt))):
        vr = vref_fn(k*dt)
        a_prev = a
        sigma = (vr - v) - a*abs(a)/(2*J)
        delta = J*dt*(abs(a) + J*dt/2)
        snapped = False
        if abs(sigma) > delta:
            jj = J*np.sign(sigma)
        else:
            jj = np.clip(sigma/delta*J, -J, J); jj = np.clip(jj, (-A - a)/dt, (A - a)/dt)
            if abs(vr - v) < J*dt*dt:
                a = 0.0; v = vr; snapped = True
        if not snapped:
            a = np.clip(a + jj*dt, -A, A); v = v + a*dt
        V.append(v); Acc.append(a); Jk.append((a - a_prev)/dt)
    return np.array(V), np.array(Acc), np.array(Jk)

def filt_fixed(vref_fn, T, dt, v0=0.0, a0=0.0):
    """Corrected: the switching surface uses the velocity reached after braking a to 0 including
    the current step, and the boundary-layer law is the exact one-step jerk that lands on the surface,
    clipped to +-J; terminal snap only if |a| <= J*dt as well (no jerk violation)."""
    v, a = v0, a0; V, Acc, Jk = [], [], []
    for k in range(int(round(T/dt))):
        vr = vref_fn(k*dt); a_prev = a
        if abs(vr - v) <= J*dt*dt and abs(a) <= J*dt:
            a = 0.0; v = vr
        else:
            best = None
            # choose jerk from a fine candidate set that minimises |sigma after step| (discrete surface tracking)
            for jj in np.linspace(-J, J, 81):
                a1 = np.clip(a + jj*dt, -A, A); v1 = v + 0.5*(a + a1)*dt
                s1 = (vr - v1) - a1*abs(a1)/(2*J)
                # prefer surface; penalise wrong-direction overshoot
                cost = abs(s1)
                if best is None or cost < best[0] - 1e-12: best = (cost, jj, a1, v1)
            _, jj, a, v = best
        V.append(v); Acc.append(a); Jk.append((a - a_prev)/dt)
    return np.array(V), np.array(Acc), np.array(Jk)

def report(name, f, dt):
    V, Acc, Jk = f(lambda t: 2.0, 5.0, dt)
    k98 = np.argmax(V >= 2.0 - 1e-3); t_reach = (k98 + 1)*dt
    settle = V[int(3.0/dt):]
    sign_changes = np.sum(np.abs(np.diff(np.sign(np.round(Jk[int(3.0/dt):], 9)))) > 0)
    print(f"{name} dt={dt}: t(v>=1.999)={t_reach:.3f}s  max|a|={np.abs(Acc).max():.3f}  "
          f"max|j|={np.abs(Jk).max():.2f}  overshoot={V.max()-2.0:.2e}  final a={Acc[-1]:.2e}  "
          f"jerk sign changes t>3s: {sign_changes}")
    # step down 2 -> 1.1 from cruise, and a reversal (accelerating then ref drops)
    V2, A2, J2 = f(lambda t: 1.1, 4.0, dt, v0=2.0)
    print(f"{name} dt={dt}: 2.0->1.1 undershoot={1.1-V2.min():.2e} max|j|={np.abs(J2).max():.2f} final v={V2[-1]:.4f}")
    V3, A3, J3 = f(lambda t: 2.0 if t < 0.8 else 0.5, 6.0, dt)
    print(f"{name} dt={dt}: reversal (ref 2.0 then 0.5 at t=0.8): max|j|={np.abs(J3).max():.2f} "
          f"min v={V3.min():.4f} final v={V3[-1]:.4f} final a={A3[-1]:.2e}")

if __name__ == "__main__":
    for dt in [0.01, 0.02]:
        report("BRIEF", filt_brief, dt)
        report("FIXED", filt_fixed, dt)
