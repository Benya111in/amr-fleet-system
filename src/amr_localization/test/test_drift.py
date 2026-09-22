"""드리프트 실험 시나리오(운동학 시뮬레이터 되먹임)와 분석 테스트."""

import csv
import math

from amr_localization import drift_analysis as da
from amr_localization.drift_scenarios import (
    build_scenario, MotionLimits, ScenarioRunner, Segment, wrap)
import numpy as np
import pytest


def simulate(segments, dt=0.02, t_max=200.0, limits=None, start=(0.0, 0.0, 0.0)):
    """단일 적분 운동학 로봇으로 시나리오 실행. 반환: (자세 목록, 기록 여부 목록)."""
    runner = ScenarioRunner(segments, limits or MotionLimits())
    x, y, th = start
    t = 0.0
    poses, logging = [], []
    while not runner.done and t < t_max:
        v, w = runner.step(t, (x, y, th))
        poses.append((x, y, th))
        logging.append(runner.logging)
        x += v * dt * math.cos(th + 0.5 * w * dt)
        y += v * dt * math.sin(th + 0.5 * w * dt)
        th = wrap(th + w * dt)
        t += dt
    assert runner.done
    assert runner.step(t, (x, y, th)) == (0.0, 0.0)
    return (x, y, th), poses, logging


def test_straight_segment_reaches_distance():
    end, _, _ = simulate([Segment('straight', 10.0)])
    assert end[0] == pytest.approx(10.0, abs=0.01)
    assert end[1] == pytest.approx(0.0, abs=1e-6)


def test_backward_straight_and_rotation():
    end, _, _ = simulate([Segment('straight', -2.0), Segment('rotate', -math.pi / 2)])
    assert end[0] == pytest.approx(-2.0, abs=0.01)
    assert end[2] == pytest.approx(-math.pi / 2, abs=0.01)


def test_full_turn_unwraps():
    end, _, _ = simulate([Segment('rotate', 2.0 * math.pi)], start=(1.0, 1.0, 3.0))
    assert abs(wrap(end[2] - 3.0)) < 0.01
    assert end[:2] == pytest.approx((1.0, 1.0))


def test_pause_and_logging_flags():
    segs = [Segment('rotate', math.pi, log=False), Segment('pause', 0.5),
            Segment('straight', 1.0)]
    _, _, logging = simulate(segs)
    assert logging[0] is False       # 방향 되돌리기 구간은 기록 안 함
    assert True in logging           # 이후 구간은 기록


@pytest.mark.parametrize('name', ['square_ccw', 'square_cw'])
def test_rectangles_return_to_start(name):
    end, poses, logging = simulate(build_scenario(name, 0, rect=(3.0, 2.0), settle=0.2))
    assert end[:2] == pytest.approx((0.0, 0.0), abs=0.03)
    assert abs(wrap(end[2])) < 0.02
    xs = [p[0] for p, lg in zip(poses, logging) if lg]
    ys = [p[1] for p, lg in zip(poses, logging) if lg]
    assert max(xs) == pytest.approx(3.0, abs=0.03)
    assert max(ys) == pytest.approx(2.0, abs=0.03)
    assert min(ys) > -0.03                      # 두 방향 모두 같은 직사각형 (y ≥ 0)


def test_build_scenario_variants():
    assert build_scenario('straight', 0)[0].kind == 'pause'
    assert build_scenario('straight', 1)[0].kind == 'rotate'
    assert build_scenario('straight', 1)[0].log is False
    assert build_scenario('straight', 2)[0].kind == 'rotate'
    assert build_scenario('rotate', 1)[1].amount < 0.0
    with pytest.raises(ValueError):
        build_scenario('zigzag', 0)
    runner = ScenarioRunner([Segment('hop', 1.0)])
    with pytest.raises(ValueError):
        runner.step(0.0, (0.0, 0.0, 0.0))


def test_repeated_plan_stays_in_bounded_area():
    # 회귀: 실험 노드 순서(rep 마다 전체 시나리오)로 3 회 반복해도 직진이 왕복해 시작점 주변에 머문다
    # (이전에는 rep 2 직진이 rep 1 방향으로 다시 10 m 나가 Gazebo 창고 벽에 막혔다)
    pose = (0.0, 0.0, 0.0)
    xs, ys = [], []
    for rep in range(3):
        for name in ('straight', 'rotate', 'square_ccw', 'square_cw'):
            pose, poses, _ = simulate(build_scenario(name, rep), start=pose, t_max=400.0)
            xs += [p[0] for p in poses]
            ys += [p[1] for p in poses]
    assert min(xs) > -5.1 and max(xs) < 15.1
    assert min(ys) > -4.1 and max(ys) < 4.1
    assert abs(pose[0] - 10.0) < 0.1 and abs(pose[1]) < 0.1   # rep 2 끝: 동쪽 끝


# ------------------------------------------------------------------ 분석
def synthetic_run(n=501, length=10.0, lateral_drift=0.002, yaw_drift=0.001,
                  start=(3.0, 4.0, 0.7)):
    """GT 는 월드 좌표 직진, odom 은 원점 시작 + 거리 비례 횡 드리프트."""
    s = np.linspace(0.0, length, n)
    c, sn = math.cos(start[2]), math.sin(start[2])
    data = {k: np.zeros(n) for k in da.COLUMNS}
    data['timestamp'] = np.linspace(0.0, 20.0, n)
    data['gt_x'] = start[0] + c * s
    data['gt_y'] = start[1] + sn * s
    data['gt_yaw'] = np.full(n, start[2])
    data['odom_x'] = s
    data['odom_y'] = lateral_drift * s
    data['odom_yaw'] = yaw_drift * s
    data['odom_var_x'] = 1e-6 + 1e-6 * s
    data['odom_var_y'] = 1e-6 + 4e-6 * s
    data['odom_var_yaw'] = 1e-7 + 1e-7 * s
    for k in ('ekf_x', 'ekf_y', 'ekf_yaw'):
        data[k] = data['odom_' + k[4:]].copy()
    return data


def test_relative_track_and_wrap():
    x, y, th = da.relative_track(np.array([1.0, 1.0]), np.array([1.0, 2.0]),
                                 np.array([math.pi / 2, math.pi / 2]))
    assert (x[1], y[1], th[1]) == pytest.approx((1.0, 0.0, 0.0))
    assert da.total_rotation(np.array([0.0, 3.0, -3.0])) == pytest.approx(3.0 + (2 * math.pi - 6))
    assert da.total_rotation(np.array([1.0])) == 0.0


def test_analyze_run_metrics():
    m = da.analyze_run(synthetic_run(), 'straight', '0')
    assert m.path_length == pytest.approx(10.0)
    assert m.final_y_error == pytest.approx(0.02)
    assert m.final_pos_error == pytest.approx(0.02)
    assert m.growth_rate == pytest.approx(0.002, rel=1e-3)
    assert m.drift_percent == pytest.approx(0.2)
    assert m.yaw_drift_deg_per_m == pytest.approx(math.degrees(0.001))
    assert m.pred_sigma_pos == pytest.approx(math.sqrt(5e-5))
    # NEES = e_y² / Var(y) + e_θ² / Var(θ) (e_x = 0)
    assert m.nees == pytest.approx(0.02 ** 2 / 4e-5 + 0.01 ** 2 / 1e-6)
    assert m.ekf_final_pos_error == pytest.approx(0.02)
    with pytest.raises(ValueError):
        da.analyze_run({k: np.zeros(1) for k in da.COLUMNS}, 'x', '0')


def test_rotation_run_and_missing_ekf():
    n = 200
    data = {k: np.zeros(n) for k in da.COLUMNS}
    data['timestamp'] = np.linspace(0, 10, n)
    data['gt_yaw'] = da.wrap(np.linspace(0.0, 2 * math.pi, n))
    data['odom_yaw'] = da.wrap(np.linspace(0.0, 2 * math.pi * 1.01, n))
    data['ekf_x'][:] = math.nan
    m = da.analyze_run(data, 'rotate', '1')
    assert m.rotation == pytest.approx(2 * math.pi, rel=1e-3)
    assert m.yaw_drift_deg_per_rev == pytest.approx(3.6, rel=0.01)
    assert math.isnan(m.growth_rate) and math.isnan(m.ekf_final_pos_error)
    assert math.isnan(m.nees)  # 공분산 0 → 정의 안 됨


def test_closed_form_predictions_match_monte_carlo():
    # 직진: 주기당 곱셈 슬립 σ_s 를 두 바퀴에 독립 적용 → 헤딩 분산 2kL/b²
    rng = np.random.default_rng(3)
    sigma_s, b, v, dt, length = 0.01, 0.36, 1.0, 0.02, 20.0
    steps = int(length / (v * dt))
    ds = v * dt
    eps = rng.normal(0.0, sigma_s, size=(4000, steps, 2))
    dth = ds * (eps[:, :, 0] - eps[:, :, 1]) / b
    th = np.cumsum(dth, axis=1)
    y = np.sum(ds * th, axis=1)
    p = da.predicted_straight(length, v, sigma_s, b, dt)
    assert th[:, -1].std() == pytest.approx(p['sigma_yaw'], rel=0.05)
    assert y.std() == pytest.approx(p['sigma_y'], rel=0.05)
    # 제자리 회전: 바퀴 변위 ±ωbΔt/2
    w, angle = 0.5, 2 * math.pi
    a = w * b / 2 * dt
    n = int(angle / (2 * a / b))
    eps = rng.normal(0.0, sigma_s, size=(4000, n, 2))
    th = np.sum(a * (1 + eps[:, :, 0]) / b + a * (1 + eps[:, :, 1]) / b, axis=1)
    pr = da.predicted_rotation(angle, w, sigma_s, b, dt)
    assert th.std() == pytest.approx(pr['sigma_yaw'], rel=0.05)


def write_run(path, data):
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(da.COLUMNS)
        for i in range(data['timestamp'].size):
            w.writerow([data[c][i] for c in da.COLUMNS])


def test_directory_report_and_umbmark(tmp_path, capsys):
    write_run(tmp_path / 'straight_0.csv', synthetic_run())
    write_run(tmp_path / 'straight_1.csv', synthetic_run(lateral_drift=0.003))
    for name, drift in (('square_cw', 0.001), ('square_ccw', -0.002)):
        write_run(tmp_path / f'{name}_0.csv', synthetic_run(lateral_drift=drift, length=4.0))
    (tmp_path / 'summary_old.csv').write_text('ignored')
    (tmp_path / 'nounderscore.csv').write_text('ignored')
    assert da.main(['--input', str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert '| straight | 2 |' in out
    assert 'UMBmark' in out and 'E_max,syst = 0.0080' in out
    assert '폐형 예측' in out
    rows = list(csv.DictReader((tmp_path / 'summary.csv').open()))
    assert len(rows) == 4
    empty = tmp_path / 'empty'
    empty.mkdir()
    assert da.main(['--input', str(empty)]) == 1


def test_effective_parameters_from_scale_errors():
    # 직진: odom 거리 1 % 과대 (r_eff = r/1.01), 회전: odom 회전 2 % 과대 → b_eff = b·1.02/1.01
    n = 300
    straight = {k: np.zeros(n) for k in da.COLUMNS}
    straight['timestamp'] = np.linspace(0, 20, n)
    straight['gt_x'] = np.linspace(0.0, 10.0, n)
    straight['odom_x'] = 1.01 * straight['gt_x']
    rotate = {k: np.zeros(n) for k in da.COLUMNS}
    rotate['timestamp'] = np.linspace(0, 15, n)
    rotate['gt_yaw'] = da.wrap(np.linspace(0.0, -2 * math.pi, n))
    rotate['odom_yaw'] = da.wrap(np.linspace(0.0, -2 * math.pi * 1.02, n))
    runs = [da.analyze_run(straight, 'straight', '0'), da.analyze_run(rotate, 'rotate', '0')]
    assert runs[1].net_rotation == pytest.approx(-2 * math.pi, rel=1e-3)
    e = da.effective_parameters(runs, 0.0825, 0.36)
    assert e['distance_scale'] == pytest.approx(1.01, rel=1e-6)
    assert e['heading_scale'] == pytest.approx(1.02, rel=1e-3)
    assert e['wheel_radius_eff'] == pytest.approx(0.0825 / 1.01, rel=1e-6)
    assert e['separation_eff'] == pytest.approx(0.36 * 1.02 / 1.01, rel=1e-3)
    assert da.effective_parameters(runs[:1], 0.0825, 0.36) is None
    text = da.summarize(runs, {'wheel_radius': 0.0825, 'separation': 0.36})
    assert '체계 오차 (유효 파라미터)' in text and 'b_eff = 0.36' in text


def test_summarize_rotation_prediction_and_single_values():
    n = 100
    data = {k: np.zeros(n) for k in da.COLUMNS}
    data['timestamp'] = np.linspace(0, 10, n)
    data['gt_yaw'] = da.wrap(np.linspace(0.0, 2 * math.pi, n))
    data['odom_yaw'] = data['gt_yaw'].copy()
    m = da.analyze_run(data, 'rotate', '0')
    text = da.summarize([m], {'sigma_s': 0.01, 'separation': 0.36, 'dt': 0.02,
                              'angular_speed': 0.5})
    assert '- rotate 0' in text
    assert da.umbmark([m]) is None
    assert da.summarize([m]).startswith('# ')
