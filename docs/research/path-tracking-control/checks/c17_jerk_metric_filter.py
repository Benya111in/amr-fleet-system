"""Audit-2 check (SG7): does the jerk metric itself push an at-limit (j = j_max = 2) profile over the
P99 <= 2 m/s^3 target?  S1-like run: 0 -> 2 m/s S-curve, cruise, 2 -> 0 (a 1, j 2), total 30 m, GT twist at
50 Hz (noise-free).  Metric pipelines: (i) 2nd-order Butterworth 5 Hz filtfilt (brief v2.1), (ii) zero-phase
Gaussian kernel sigma = sqrt(ln 2)/(2 pi 5 Hz) = 26.5 ms (-3 dB at 5 Hz, non-negative kernel => filtered
|j| <= max |j| exactly).  Also the IMU noise floor (sigma_a 0.017 m/s^2, 100 Hz) for both filters."""
import numpy as np
from scipy import signal
from scipy.ndimage import gaussian_filter1d

def s1_profile(fs):
    dt = 1/fs; t = np.arange(0, 22, dt); j = np.zeros_like(t)
    # accel: +j 0.5 s, 0 1.5 s, -j 0.5 s (0->2 m/s in 2.5 s, 2.5 m); cruise 25/2 = 12.5 s; decel mirror
    seg = [(0.5, 2.0), (1.5, 0.0), (0.5, -2.0), (12.5, 0.0), (0.5, -2.0), (1.5, 0.0), (0.5, 2.0)]
    t0 = 1.0
    for d, jj in seg:
        j[(t >= t0) & (t < t0 + d)] = jj; t0 += d
    a = np.cumsum(j)*dt; v = np.cumsum(a)*dt
    return t, v, a, j

sig_t = np.sqrt(np.log(2))/(2*np.pi*5.0)
for fs in [50.0, 100.0]:
    t, v, a, j = s1_profile(fs)
    moving = v > 1e-3
    b, aa = signal.butter(2, 5.0/(fs/2))
    jb = np.diff(signal.filtfilt(b, aa, v), 2)*fs*fs                      # from GT velocity: 2 differences
    jg = np.diff(gaussian_filter1d(v, sig_t*fs, mode="nearest"), 2)*fs*fs
    for name, jj in [("Butterworth-2 5 Hz", jb), ("Gaussian 26.5 ms", jg)]:
        m = moving[1:-1]
        print(f"fs={fs:.0f} {name:18s}: max|j| {np.abs(jj).max():.3f}, P99|j| (moving samples) {np.percentile(np.abs(jj[m]), 99):.3f}, "
              f"share of moving samples with |j|>2.0: {100*np.mean(np.abs(jj[m]) > 2.0):.2f} %")
rng = np.random.default_rng(1)
x = rng.normal(0, 0.017, 400000); fs = 100.0
b, aa = signal.butter(2, 5.0/(fs/2))
for name, y in [("Butterworth-2 5 Hz", signal.filtfilt(b, aa, x)), ("Gaussian 26.5 ms", gaussian_filter1d(x, sig_t*fs))]:
    jn = np.diff(y)*fs
    print(f"IMU noise floor {name:18s}: sigma_j {jn.std():.3f}, P99 {np.percentile(np.abs(jn), 99):.3f} m/s^3")
print(f"Gaussian sigma_t = {1000*sig_t:.1f} ms")

# start-up step (a0 = 0.3 m/s^2, optional exemption) through the Gaussian metric, GT velocity at 50 Hz
for fs in [50.0, 100.0]:
    dt = 1/fs; t = np.arange(0, 4, dt); acc = np.zeros_like(t); a_ = 0.0
    for i, ti in enumerate(t):
        if ti >= 1.0: a_ = 0.3 if a_ == 0.0 else min(1.0, a_ + 2.0*dt)
        acc[i] = a_
    v = np.cumsum(acc)*dt
    jg = np.diff(gaussian_filter1d(v, sig_t*fs, mode="nearest"), 2)*fs*fs
    inner = (t[1:-1] > 0.5) & (t[1:-1] < 3.5)          # exclude the array-end artefact of mode="nearest"
    print(f"start-up step a0=0.3, fs={fs:.0f}, Gaussian metric: peak |j| {np.abs(jg[inner]).max():.2f} m/s^3")
