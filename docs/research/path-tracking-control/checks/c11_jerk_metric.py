"""Spec gap 7: noise floor of the IMU-derived jerk metric (sensors.yaml accel_noise_stddev 0.017 m/s^2
at 100 Hz) through a 2nd-order Butterworth 5 Hz zero-phase filter + first difference; effect of the
0.3 m/s^2 start-up acceleration step on the filtered jerk; GT source ground_truth/odom at 50 Hz."""
import numpy as np
from scipy import signal
rng = np.random.default_rng(1)
for fs, sa, label in [(100.0, 0.017, "IMU 100 Hz, sigma_a=0.017 (config)"), (100.0, 0.02, "IMU 100 Hz, sigma_a=0.02 (brief)")]:
    b, a = signal.butter(2, 5.0/(fs/2)); x = rng.normal(0, sa, 400000)
    y = signal.filtfilt(b, a, x); jn = np.diff(y)*fs
    print(f"SG7 {label}: sigma_j={jn.std():.3f} m/s^3, P99|j|={np.percentile(np.abs(jn), 99):.3f}")
# start-up step: a jumps 0 -> 0.3 then jerk-limited ramp to 1.0; filtered jerk peak (GT 50 Hz)
for fs in [50.0, 100.0]:
    dt = 1/fs; t = np.arange(0, 4, dt); acc = np.zeros_like(t); a_ = 0.0
    for i, ti in enumerate(t):
        if ti >= 1.0:
            a_ = 0.3 if a_ == 0.0 else min(1.0, a_ + 2.0*dt)
        acc[i] = a_
    b, a = signal.butter(2, 5.0/(fs/2)); af = signal.filtfilt(b, a, acc); j = np.diff(af)*fs
    b0 = acc.copy(); b0[t >= 1.0] = np.minimum(1.0, 2.0*(t[t >= 1.0] - 1.0)); jb = np.diff(signal.filtfilt(b, a, b0))*fs
    print(f"SG7 fs={fs:.0f}: filtered |j| peak with start-up step {np.abs(j).max():.2f} m/s^3 vs without {np.abs(jb).max():.2f}")
