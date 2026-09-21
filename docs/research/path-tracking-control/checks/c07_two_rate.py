"""(1) closed-form braking distance vs the filter itself; (2) contract two-rate architecture on an
integer 100 Hz base clock: controller 20 Hz (step targets + braking trigger using a mirror of the
profiler filter) -> velocity_profiler_node 50 Hz (closed-form jerk filter) with 0/1/3 profiler-tick
transport delay; (3) DWA emitting window-clamped velocities (ramp semantics) vs targets into the downstream jerk filter."""
import numpy as np, importlib.util, os, sys
here = os.path.dirname(os.path.abspath(__file__))
def load(n):
    sp = importlib.util.spec_from_file_location(n, os.path.join(here, n + ".py")); m = importlib.util.module_from_spec(sp)
    so = sys.stdout; sys.stdout = open(os.devnull, "w"); sp.loader.exec_module(m); sys.stdout = so; return m
c05 = load("c05_jerk_filter_closed"); c06 = load("c06_envelope")
step, brake_dist, envelope = c05.step_closed, c06.brake_dist, c06.envelope
ds = 0.05
for (v, a, vt) in [(2.0, 0.0, 1.1), (1.5, 0.8, 1.1), (1.8, -0.5, 1.0), (1.2, 1.0, 0.3), (2.0, 0.0, 0.0), (0.4, 0.0, 0.0)]:
    vv, aa, d = v, a, 0.0
    for _ in range(100000):
        v1, a1 = step(vv, aa, vt, 1e-3); d += 0.5*(vv + v1)*1e-3; vv, aa = v1, a1
        if vv == vt and aa == 0.0: break
    print(f"brake_dist({v},{a}->{vt}): closed {float(brake_dist(v, a, vt)):.4f} m, filter-simulated {d:.4f} m")

def binding_points(vcap):
    vpe = envelope(vcap, range(len(vcap))); return np.where(np.abs(vpe - vcap) < 1e-9)[0]

def two_rate(vcap, delay_ticks=1, margin_t=0.05):
    n = len(vcap); sg = np.arange(n)*ds; B = binding_points(vcap)
    s = v = a = 0.0; target = 0.0; queue = [0.0]*delay_ticks; worst, ws = -9.0, 0.0; worst_mid = -9.0
    for k in range(40000):                      # 100 Hz base clock
        t = k*0.01
        if k % 5 == 0:                          # controller 20 Hz: mirror = last profiler state (resync)
            vm, am = v, a
            vT = np.interp(s, sg, vcap)
            ahead = B[(sg[B] > s) & (vcap[B] < vm)]
            if len(ahead):
                hit = sg[ahead] - s <= brake_dist(vm, am, vcap[ahead]) + vm*margin_t
                if hit.any(): vT = min(vT, vcap[ahead][hit].min())
            target = vT
        if k % 2 == 0:                          # profiler 50 Hz
            queue.append(target); tgt = queue.pop(0)
            v1, a1 = step(v, a, tgt, 0.02); s_new = s + 0.5*(v + v1)*0.02; v, a, s = v1, a1, s_new
            exc = v - np.interp(min(s, sg[-1]), sg, vcap)
            if exc > worst: worst, ws = exc, s
            if s < sg[-1] - 0.10:
                worst_mid = max(worst_mid, exc)
            if s >= sg[-1] - 0.02 and v == 0.0: return worst, ws, t, v, s, worst_mid
            if v == 0.0 and target == 0.0 and k > 10: return worst, ws, t, v, s, worst_mid
    return worst, ws, t, v, s, worst_mid

for name, vc in c06.cases.items():
    for d in [0, 1, 3]:
        w, ws, T, vend, send, wm = two_rate(vc, delay_ticks=d)
        print(f"2-rate {name} delay {d}: max(v - v_cap) {w:+.3f} m/s at s={ws:.2f}; excluding last 0.1 m {wm:+.4f}; stop at s={send:.3f} (goal {(len(vc)-1)*ds:.2f}), T={T:.2f} s")

for centre in ["window-clamped output v_f + a*dT (ramp semantics)", "target semantics (step to v*)"]:
    v = a = 0.0; tgt = 0.0; tt = None
    for k in range(3000):
        if k % 5 == 0: tgt = min(2.0, v + 1.0*0.05) if centre.startswith("window") else 2.0
        if k % 2 == 0:
            v, a = step(v, a, tgt, 0.02)
            if tt is None and v >= 1.999: tt = k*0.01
    print(f"DWA output = {centre}: t(v>=1.999) = {tt:.2f} s (jerk-limited optimum 2.47 s)")

# trigger margin must include transport delay: margin_t = T_ctrl(0.05) + delay
for name, vc in c06.cases.items():
    w, ws, T, vend, send, wm = two_rate(vc, delay_ticks=3, margin_t=0.05 + 0.06)
    print(f"2-rate {name} delay 3, margin 0.11 s: max(v - v_cap) {w:+.3f} m/s at s={ws:.2f}; excluding last 0.1 m {wm:+.4f}; stop at s={send:.3f}, T={T:.2f} s")
