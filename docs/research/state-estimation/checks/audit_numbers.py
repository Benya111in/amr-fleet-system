"""Audit re-derivation of the closed-form numbers in state-estimation.md (revised).
Run: python3 audit_numbers.py
"""
import math
import numpy as np
from scipy.stats import chi2

def hdr(s):
    print("\n== " + s)

# sensors.yaml / robot_params.yaml
sg, sacc = 2e-4, 0.017           # gyro, accel white sigma
bg, ba = 0.01, 0.10              # static bias means
ks, r, b = 0.01, 0.0825, 0.36
dt_enc, dt_imu = 0.02, 0.01

hdr("M1 gyro bias / heading budget (sec 2.1, 2.2)")
print("uncal bias heading 10 s, 60 s:", bg * 10, bg * 60, "rad =", math.degrees(bg * 60), "deg")
for T_cal in (10, 60):
    se = sg / math.sqrt(T_cal / dt_imu)
    print(f"T_cal={T_cal}s residual bias SE = {se:.2e} rad/s")
se10 = sg / math.sqrt(1000)
th_b = se10 * 60
th_w = sg * math.sqrt(dt_imu * 60)
print("post-cal heading sigma @60s: bias %.2e white %.2e total %.2e" % (th_b, th_w, math.hypot(th_b, th_w)))
# lateral error on 20 m in 60 s (v = 1/3 m/s): bias -> b v T^2/2 ; white RW -> D sigma_end/sqrt3
v = 20 / 60
lat_b = se10 * v * 60 ** 2 / 2
lat_w = 20 * th_w / math.sqrt(3)
print("20 m/60 s lateral: bias %.4f m, white %.4f m, rss %.4f m, upper bound D*theta_end %.4f m" % (lat_b, lat_w, math.hypot(lat_b, lat_w), 20 * math.hypot(th_b, th_w)))
print("encoder-only heading RW 2 m/s 10 s:", 0.0786 * math.sqrt(10 * dt_enc))
print("R_gyro = sg^2 =", sg ** 2, "; + residual^2 =", sg ** 2 + se10 ** 2)

hdr("G6 nominal drift 100 m @ 1 m/s (sec 2.1)")
D, vv = 100.0, 1.0
ds = vv * dt_enc
n = D / ds
sig_dth = ks * math.sqrt(2) * ds / b
sth = sig_dth * math.sqrt(n)
print("encoder heading RW sigma_theta end = %.4f rad (%.2f deg)" % (sth, math.degrees(sth)))
print("lateral = D sigma/sqrt3 = %.2f m" % (D * sth / math.sqrt(3)))
sD_doc = ks * math.sqrt(ds * D)
sD_true = ks * ds / math.sqrt(2) * math.sqrt(n)
print("distance scale sigma: doc k_s sqrt(ds D) = %.4f ; correct k_s sqrt(ds D/2) = %.4f" % (sD_doc, sD_true))
T = D / vv
for T_cal in (10, 60):
    se = sg / math.sqrt(T_cal / dt_imu)
    lat_bias = se * vv * T ** 2 / 2
    lat_white = D * sg * math.sqrt(dt_imu * T) / math.sqrt(3)
    tot = math.sqrt(lat_bias ** 2 + lat_white ** 2 + sD_true ** 2)
    print(f"gyro-fused, T_cal={T_cal}s: lateral bias {lat_bias:.4f} white {lat_white:.4f} scale {sD_true:.4f} -> total {tot:.4f} m ({100*tot/D:.3f} %)")

hdr("M2 AMCL alpha (variance units)")
a3_phys = (ks / math.sqrt(2)) ** 2
print("alpha3_phys = (k_s/sqrt2)^2 = %.2e" % a3_phys)
print("kappa 5..10 -> alpha = %.2e .. %.2e" % (25 * a3_phys, 100 * a3_phys))
print("alpha=0.05 -> sigma frac %.3f = %.1fx physical" % (math.sqrt(0.05), math.sqrt(0.05) / (ks / math.sqrt(2))))
sth_upd = sg * math.sqrt(dt_imu * 0.1)
print("gyro heading sigma per 0.1 m update @1 m/s: %.2e -> alpha2_phys %.1e" % (sth_upd, (sth_upd / 0.1) ** 2))
srot = math.sqrt(2e-3) * 0.1
print("alpha2=2e-3, 0.1 m: sigma_rot %.2e rad -> 10 m endpoint %.3f m (%.2f sigma_hit@0.05)" % (srot, 10 * srot, 10 * srot / 0.05))

hdr("M4 sigma_map / sigma_hit")
smap = 0.03 / math.sqrt(2 / math.pi)
shit = math.sqrt(0.03 ** 2 + 0.05 ** 2 / 12 + smap ** 2)
print("sigma_map %.4f sigma_hit %.4f ; single-update width sigma_hit/sqrt3 %.4f" % (smap, shit, 0.05 / math.sqrt(3)))

hdr("M5 uniform re-init expected hits in basin (3-D pose space)")
A_free = 1000.0
def frac(sh, r_rep=5.0):
    rad = 3 * sh
    dth = 3 * sh / r_rep
    return (math.pi * rad ** 2 / A_free) * (2 * dth / (2 * math.pi))
for sh, N in ((0.05, 5000), (0.2, 5000), (0.2, 20000), (0.2, 50000)):
    f = frac(sh)
    lam = N * f
    print(f"sigma_hit={sh} N={N}: frac {f:.2e}, E[hits] {lam:.3f}, P(>=1) {1-math.exp(-lam):.2f}")

hdr("M3 resampling cost (naive sampler, avg N^2/2 comparisons)")
for N in (500, 750, 5000, 20000, 100000):
    c = N * N / 2
    print(f"N={N}: {c:.2e} comparisons -> {c*1e-9*1e3:.1f}..{c*2e-9*1e3:.1f} ms @1-2 ns/cmp")
print("tracking update: 500x180 lookups =", 500 * 180)
print("recovery N=20000: lookups", 20000 * 180, "; resample ~0.2-0.4 s; spin 6.3 s @1 rad/s, 10 Hz scans")
# effective AMCL update count in GLOBAL rotate: CPU-bound
per_upd = 0.2 + 20000 * 180 * 20e-9   # resample + lookups (20 ns/lookup, cache-missy)
print("per-update (resample every update) ~ %.2f s -> ~%.0f updates in 6.3 s; with resample_interval 2 avg %.2f s -> ~%.0f updates" % (per_upd, 6.3 / per_upd, (per_upd + 20000*180*20e-9) / 2, 6.3 / ((per_upd + 20000*180*20e-9) / 2)))

hdr("M17 NEES bands (5 dof)")
print("single-run 95%%: [%.2f, %.2f]" % (chi2.ppf(0.025, 5), chi2.ppf(0.975, 5)))
print("10-run average: [%.2f, %.2f]" % (chi2.ppf(0.025, 50) / 10, chi2.ppf(0.975, 50) / 10))
print("chi2 0.999 m=1,2,3:", [round(chi2.ppf(0.999, m), 1) for m in (1, 2, 3)], "n_sigma:", [round(math.sqrt(chi2.ppf(0.999, m)), 2) for m in (1, 2, 3)])

hdr("M16 Q equivalence: r_l Q_param,xx random walk vs CWNA")
for qp in (0.05, 1e-3, 1e-5):
    print(f"r_l Q_xx={qp}: added pos sigma over 0.1 s AMCL interval = {math.sqrt(qp*0.1):.4f} m ; over 1 s = {math.sqrt(qp):.4f} m")
qv = 0.005
print("CWNA q_v=0.005: pos sigma from process over 0.1 s = %.2e m (q dt^3/3)" % math.sqrt(qv * 0.1 ** 3 / 3))

hdr("M15 CUSUM occlusion (unmasked)")
for w, d in ((0.5, 0.5), (0.45, 0.5), (0.6, 1.5), (0.4, 0.5)):
    ang = 2 * math.degrees(math.atan(w / 2 / d))
    print(f"object width {w} m at {d} m: {ang:.0f} deg = {ang/360*100:.0f} % of beams")
print("30%% drop, 20 scans, delta 0.15: g = %.1f" % (20 * (0.30 - 0.15)))

hdr("M6 SEED grid")
print("coarse hypotheses 1000 x 24 =", 1000 * 24, "; lookups x90 beams =", 1000 * 24 * 90)
print("fine per candidate 11x11x9x180 =", 11 * 11 * 9 * 180, "; x20 =", 20 * 11 * 11 * 9 * 180)
print("worst coarse offsets: pos %.3f m, heading 7.5 deg -> endpoint @5 m %.2f m, @10 m %.2f m, @20 m %.2f m" % (math.sqrt(0.5), 5 * math.sin(math.radians(7.5)), 10 * math.sin(math.radians(7.5)), 20 * math.sin(math.radians(7.5))))
print("EDT float32 1200x800 = %.2f MB" % (1200 * 800 * 4 / 1e6))
