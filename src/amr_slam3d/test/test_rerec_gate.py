"""
재녹화(b2-rerec) 녹화 결함 검사 시험 — 기동 바이어스 줄(규칙 5), EKF yaw 표류율(규칙 6), 정착 판정, evaluate 통합.

로그 줄은 실제 녹화 launch.log 의 줄을 그대로 옮겼다 (runs/b24/b2-rec/rec/<태그>/launch.log).
"""

import math

from amr_slam3d import recgate
import numpy as np
import pytest

START = ('[imu_filter_node-15] [INFO] [1791449422.736816968] [amr_01.imu_filter_node]: '
         'imu filter: lpf 20.0 Hz, startup bias estimation 60.0 s, gyro_bias [0, 0, 0], '
         'accel_bias [0, 0, 0]')
FALLBACK = ('[imu_filter_node-15] [WARN] [1791449423.120276882] [amr_01.imu_filter_node]: motion '
            'detected during startup bias estimation; falling back to parameter bias')
FULL = ('[imu_filter_node-15] [INFO] [1791451698.225499637] [amr_01.imu_filter_node]: '
        'startup bias estimated from 12001 samples: '
        'gyro_bias [0.0100030319, 0.0100013736, 0.00999077543] '
        '(SE [2.56120951e-06, 2.58686358e-06, 2.58790314e-06]) accel_bias [0.101801772, '
        '0.0998983653, 0.0993245009] (SE [0.000507374158, 0.000218456045, 0.000219668779]) '
        'over 60.0 s')
BEFORE = ('[imu_filter_node-15] [INFO] [1791445264.013616691] [amr_01.imu_filter_node]: motion '
          'detected during startup bias estimation; using 5167 samples collected before motion: '
          'gyro_bias [0.00998985175, -0.0100126076, -0.0100127603] (SE [3.93604895e-06, '
          '3.8942549e-06, 3.9531087e-06]) accel_bias [-0.0941185608, -0.1002512, 0.101568047] '
          '(SE [0.00277893535, 0.000331194851, 0.000329276214]) over 25.8 s')
OTHER = ('[scan_filter_node-16] [INFO] [1.0] [amr_01.scan_filter_node]: scan filter: '
         'range [0.10, 25.00] m')


# ---------------------------------------------------------------- 규칙 5: 기동 바이어스
def test_bias_full_estimate_passes_and_parses():
    b = recgate.bias_outcome('\n'.join([OTHER, START, FULL]))
    assert b['ok'] and b['kind'] == 'full' and b['samples'] == 12001
    assert b['gyro_bias'] == pytest.approx([0.0100030319, 0.0100013736, 0.00999077543])
    assert b['duration_s'] == pytest.approx(60.0)
    assert b['fallback'] == 0 and b['n_outcome'] == 1


def test_bias_before_motion_estimate_passes():
    b = recgate.bias_outcome('\n'.join([START, BEFORE]))
    assert b['ok'] and b['kind'] == 'before_motion' and b['samples'] == 5167
    assert b['gyro_bias'][2] == pytest.approx(-0.0100127603)


def test_bias_fallback_fails():
    """결함 녹화 3 bag 의 꼴: 기동 0.4 s 안 파라미터 바이어스."""
    b = recgate.bias_outcome('\n'.join([START, FALLBACK]))
    assert not b['ok'] and b['kind'] == 'fallback' and b['fallback'] == 1
    assert b['samples'] is None and '첫 결과가 파라미터 바이어스' in b['why']


def test_bias_fallback_anywhere_fails_even_after_success():
    b = recgate.bias_outcome('\n'.join([START, FULL, FALLBACK]))
    assert not b['ok'] and b['kind'] == 'full' and b['fallback'] == 1


def test_bias_missing_line_fails():
    b = recgate.bias_outcome('\n'.join([OTHER, START]))
    assert not b['ok'] and b['n_outcome'] == 0 and b['kind'] is None


def test_bias_too_few_samples_fails():
    few = BEFORE.replace('using 5167 samples', 'using 25 samples')
    b = recgate.bias_outcome(few)
    assert not b['ok'] and b['samples'] == 25
    assert recgate.bias_outcome(few, min_samples=20)['ok']


def test_bias_ignores_other_nodes_lines():
    alien = FALLBACK.replace('imu_filter_node', 'other_node')
    b = recgate.bias_outcome('\n'.join([alien, FULL]))
    assert b['ok'] and b['fallback'] == 0


# ---------------------------------------------------------------- 규칙 6: EKF yaw 표류율
def _gt_yaw(n=5000, rate=50.0, t0=12.0, turn=True):
    t = t0 + np.arange(n) / rate
    yaw = np.full(n, math.pi / 2)
    if turn:
        yaw = math.pi / 2 + 0.3 * np.sin(0.05 * (t - t0))
    return t, yaw


def _wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def test_drift_defect_rate_fails_and_clean_passes():
    t, g = _gt_yaw()
    bad = recgate.yaw_drift(t, _wrap(g + 0.3 + 0.0100 * (t - t[0])), t, g)
    assert not bad['ok'] and bad['rate_rad_s'] == pytest.approx(0.0100, abs=1e-6)
    ok = recgate.yaw_drift(t, _wrap(g - 0.2 - 0.0004 * (t - t[0])), t, g)
    assert ok['ok'] and ok['rate_rad_s'] == pytest.approx(-0.0004, abs=1e-6)
    assert ok['yaw_err_max_deg'] == pytest.approx(math.degrees(0.0004 * (t[-1] - t[0])), rel=1e-3)


def test_drift_threshold_is_strict_inequality():
    t, g = _gt_yaw(turn=False)
    at = recgate.yaw_drift(t, _wrap(g + 0.00201 * (t - t[0])), t, g)
    assert not at['ok']                                   # |rate| < 0.002 만 통과
    neg = recgate.yaw_drift(t, _wrap(g - 0.00201 * (t - t[0])), t, g)
    assert not neg['ok']                                  # 부호와 무관 (|rate|)
    below = recgate.yaw_drift(t, _wrap(g + 0.0019 * (t - t[0])), t, g)
    assert below['ok']


def test_drift_handles_wraparound_of_both_yaws():
    """정답·odom yaw 가 ±π 를 넘나들어도 (펼쳐서) 표류 0."""
    n = 4000
    t = np.arange(n) / 50.0
    g = _wrap(math.pi - 0.2 + 0.3 * t)                    # 여러 바퀴
    d = recgate.yaw_drift(t, _wrap(g + 1.0), t, g)
    assert d['ok'] and abs(d['rate_rad_s']) < 1e-9


def test_drift_interpolates_gt_and_sorts_odom():
    t, g = _gt_yaw()
    to = t[::2] + 0.004                                   # 다른 스탬프, 뒤섞인 순서
    yo = np.interp(to, t, g) + 0.01 * (to - to[0])
    perm = np.random.default_rng(3).permutation(len(to))
    d = recgate.yaw_drift(to[perm], _wrap(yo[perm]), t, g)
    assert d['rate_rad_s'] == pytest.approx(0.01, abs=1e-5) and not d['ok']


def test_drift_sorts_odom_before_unwrapping():
    """기록 순서가 뒤섞인 odom 이 여러 바퀴 돌면, 정렬하지 않고 펼치면 기울기가 틀린다."""
    n = 3000
    t = np.arange(n) / 50.0
    g = 0.2 * t                                           # 60 s 동안 12 rad — 여러 바퀴
    yo = _wrap(g + 0.01 * t)
    perm = np.random.default_rng(7).permutation(n)
    d = recgate.yaw_drift(t[perm], yo[perm], t, _wrap(g))
    assert d['rate_rad_s'] == pytest.approx(0.01, abs=1e-6)


def test_drift_needs_samples_inside_gt_span():
    t, g = _gt_yaw(n=100)
    d = recgate.yaw_drift(t + 1000.0, g, t, g)
    assert not d['ok'] and d['n'] == 0
    assert not recgate.yaw_drift([1.0, 2.0], [0.0, 0.0], t, g)['ok']


# ---------------------------------------------------------------- 정착 판정
def _imu(n=400, gyro_sd=2e-4, acc_sd=0.03, acc_bias=(0.1, -0.1, 0.1), seed=0):
    r = np.random.default_rng(seed)
    gy = 0.01 + gyro_sd * r.standard_normal((n, 3))
    ac = np.array([0.0, 0.0, 9.8]) + np.array(acc_bias) + acc_sd * r.standard_normal((n, 3))
    return gy, ac


def test_still_robot_passes():
    gy, ac = _imu()
    s = recgate.imu_still(gy, ac)
    assert s['ok'] and s['n'] == 400 and s['accel_norm'] == pytest.approx(9.9, abs=0.05)


def test_settling_bounce_fails_on_accel_std():
    gy, ac = _imu()
    ac[50:80, 2] += 4.0 * np.sin(np.arange(30))            # 바닥에 닿아 튀는 동안
    s = recgate.imu_still(gy, ac)
    assert not s['ok'] and s['accel_std_max'] > recgate.SETTLE_ACCEL_STD


def test_free_fall_fails_on_norm():
    gy, ac = _imu(acc_bias=(0, 0, 0))
    ac[:, 2] -= 9.8                                        # 자유 낙하: ‖ā‖ ≈ 0
    s = recgate.imu_still(gy, ac)
    assert not s['ok'] and '‖ā‖' in s['why']


def test_rocking_fails_on_gyro_std():
    gy, ac = _imu()
    gy[:, 1] += 0.02 * np.sin(np.linspace(0, 6 * math.pi, len(gy)))
    s = recgate.imu_still(gy, ac)
    assert not s['ok'] and s['gyro_std_max'] > recgate.SETTLE_GYRO_STD


def test_still_needs_two_samples():
    assert not recgate.imu_still([[0, 0, 0]], [[0, 0, 9.8]])['ok']


def test_settle_thresholds_are_stricter_than_imu_filter():
    """imu_filter 정지 판정(config/imu_filter.yaml: 0.01, 0.2, 0.5)의 절반 — 정착 뒤 첫 20 표본이 그 판정을 통과한다."""
    assert recgate.SETTLE_GYRO_STD <= 0.01 / 2
    assert recgate.SETTLE_ACCEL_STD <= 0.2 / 2
    assert recgate.SETTLE_NORM_TOL <= 0.5 / 2


# ---------------------------------------------------------------- evaluate 통합
def _clean_series(seconds=30.0, t0=5.0):
    out = {}
    for tp, per in recgate.GATE_PERIODS_NS.items():
        out[tp] = [int(t0 * 1e9) + i * per for i in range(int(seconds * 1e9 // per) + 1)]
    return out


def _gt(t0=5.0, rate=50.0, still0=12.0, move=10.0, still1=8.0):
    n = int((still0 + move + still1) * rate) + 1
    t = t0 + np.arange(n) / rate
    x = np.clip(t - t0 - still0, 0.0, move) * 0.5 + 18.0
    return t, x, np.full(n, -16.0), np.full(n, math.pi / 2)


def test_evaluate_without_new_inputs_keeps_old_rules():
    r = recgate.evaluate(_clean_series(), _gt())
    assert r['pass'] and 'bias' not in r and 'odom_drift' not in r
    assert 'drift_max_rad_s' not in r['rule']


def test_evaluate_passes_with_good_bias_and_no_drift():
    gt = _gt()
    r = recgate.evaluate(_clean_series(), gt, bias_log='\n'.join([START, BEFORE]),
                         odom=(gt[0], gt[3] + 0.5))
    assert r['pass'], r['reasons']
    assert r['bias']['ok'] and r['odom_drift']['ok']
    assert r['rule']['drift_max_rad_s'] == recgate.DRIFT_MAX_RAD_S
    assert r['rule']['bias_min_samples'] == recgate.BIAS_MIN_SAMPLES


def test_evaluate_fails_on_fallback_and_on_drift():
    gt = _gt()
    r = recgate.evaluate(_clean_series(), gt, bias_log='\n'.join([START, FALLBACK]),
                         odom=(gt[0], _wrap(gt[3] + 0.01 * (gt[0] - gt[0][0]))))
    assert not r['pass']
    assert any(s.startswith('기동 바이어스') for s in r['reasons'])
    assert any(s.startswith('EKF yaw 표류') for s in r['reasons'])
    assert not r['bias']['ok'] and not r['odom_drift']['ok']


def test_evaluate_drift_without_gt_fails():
    r = recgate.evaluate(_clean_series(), None, odom=([1.0, 2.0, 3.0], [0.0, 0.0, 0.0]))
    assert not r['pass'] and not r['odom_drift']['ok']


def test_evaluate_empty_odom_fails():
    """--odom-check 인데 bag 에 EKF odom 이 없으면 (빈 열) 통과시키지 않는다."""
    r = recgate.evaluate(_clean_series(), _gt(), odom=([], []))
    assert not r['pass'] and not r['odom_drift']['ok']
