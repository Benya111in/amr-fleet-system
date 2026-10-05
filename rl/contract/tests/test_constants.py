"""계약 상수가 저장소 설정 파일 원문과 같은지 (설정이 바뀌면 여기서 깨져야 한다)."""

import os
import re

import pytest

from amr_rl_contract import constants as C

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__)))))
yaml = pytest.importorskip('yaml')


def _load(rel):
    with open(os.path.join(REPO, rel), encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_robot_params_match():
    d = _load('config/robot_params.yaml')['/**']['ros__parameters']
    assert d['robot']['footprint_length'] == C.FOOTPRINT_LENGTH
    assert d['robot']['footprint_width'] == C.FOOTPRINT_WIDTH
    L = d['limits']
    assert L['max_linear_velocity'] == C.V_MAX_HW
    assert L['max_linear_acceleration'] == C.A_MAX
    assert L['max_angular_velocity'] == C.W_MAX
    assert L['max_angular_acceleration'] == C.ALPHA_MAX
    assert L['max_linear_jerk'] == C.JERK_MAX
    s = d['safety']
    assert s['emergency_stop_distance'] == C.E_STOP_DIST
    assert s['critical_zone_distance'] == C.CRITICAL_DIST
    assert s['warning_zone_distance'] == C.WARNING_DIST
    assert s['warning_zone_max_speed'] == C.WARNING_MAX_SPEED
    assert s['critical_zone_max_speed'] == C.CRITICAL_MAX_SPEED
    assert s['reaction_latency'] == C.REACTION_LATENCY


def test_sensors_match():
    lid = _load('config/sensors.yaml')['/**']['ros__parameters']['lidar']
    assert lid['samples'] == C.LIDAR_SAMPLES
    assert lid['angle_min'] == C.LIDAR_ANGLE_MIN
    assert lid['angle_max'] == C.LIDAR_ANGLE_MAX
    assert lid['range_min'] == C.LIDAR_RANGE_MIN
    assert lid['range_max'] == C.LIDAR_RANGE_MAX
    assert lid['update_rate'] == C.LIDAR_HZ
    assert lid['noise_stddev'] == C.LIDAR_NOISE_STD
    assert lid['extrinsic']['x'] == C.LIDAR_X and lid['extrinsic']['y'] == C.LIDAR_Y
    assert abs(C.LIDAR_ANGLE_INC - 0.0087266) < 1e-6   # 0.5°


def test_nav2_dwa_match():
    d = _load('src/amr_navigation/config/nav2_params.yaml')['controller_server']['ros__parameters']
    assert d['controller_frequency'] == C.CONTROLLER_HZ
    g = d['general_goal_checker']
    assert g['xy_goal_tolerance'] == C.GOAL_XY_TOL
    p = d['progress_checker']
    assert p['required_movement_radius'] == C.STALL_RADIUS
    assert p['movement_time_allowance'] == C.STALL_TIME
    w = d['DWA']
    assert w['max_vel_x'] == C.V_MAX_OP and w['min_vel_x'] == C.V_MIN
    assert w['sim_dt'] == C.SIM_DT
    assert w['sim_time_min'] == C.SIM_TIME_MIN and w['sim_time_max'] == C.SIM_TIME_MAX
    assert w['goal_overshoot_margin'] == C.GOAL_OVERSHOOT_MARGIN
    assert w['vo_time_horizon'] == C.VO_TIME_HORIZON
    assert w['vo_margin'] == C.VO_MARGIN
    assert w['vo_max_range'] == C.VO_MAX_RANGE
    assert w['dynamic_speed_threshold'] == C.DYN_SPEED_THRESHOLD
    assert w['dynamic_fast_speed'] == C.DYN_FAST_SPEED
    assert w['vo_time_tie'] == C.VO_TIME_TIE
    assert w['obstacle_radius'] == C.OBSTACLE_RADIUS
    assert w['robot_radius'] <= 0.0   # 외접 반경 사용 → C.ROBOT_RADIUS


def test_profiler_and_perception_match():
    v = _load('src/amr_navigation/config/velocity_profiler.yaml')['/**']['ros__parameters']
    assert v['rate'] == C.PROFILER_HZ
    assert v['max_angular_jerk'] == C.ANG_JERK_MAX
    p = _load('src/amr_perception/config/perception.yaml')
    trk = [val for key, val in p.items() if 'obstacle_tracker' in key][0]['ros__parameters']
    assert trk['ttc']['horizon'] == C.TTC_HORIZON
    assert abs(trk['ttc']['robot_radius'] - C.ROBOT_RADIUS) < 1e-3


def test_safety_node_ttc_default():
    with open(os.path.join(REPO, 'src/amr_perception/src/safety_node.cpp'), encoding='utf-8') as f:
        src = f.read()
    m = re.search(r'"ttc\.critical",\s*([0-9.]+)\)', src)
    assert m and float(m.group(1)) == C.TTC_CRITICAL


def test_metrics_thresholds_match():
    with open(os.path.join(REPO, 'tests/integration/amr_itest/metrics.py'), encoding='utf-8') as f:
        src = f.read()
    m = re.search(r'out_thr: float = ([0-9.]+)', src)
    n = re.search(r'back_thr: float = ([0-9.]+)', src)
    assert float(m.group(1)) == C.DEV_OUT and float(n.group(1)) == C.DEV_BACK


def test_lattice_shape():
    assert C.N_CELLS == 99 and C.N_ACTIONS_C == 100 and C.FALLBACK_INDEX == 99
    assert C.LATTICE_AV[0] == -C.A_MAX and C.LATTICE_AV[-1] == C.A_MAX
    assert C.LATTICE_AW[0] == -C.ALPHA_MAX and C.LATTICE_AW[-1] == C.ALPHA_MAX
    gaps = [b - a for a, b in zip(C.LATTICE_AV, C.LATTICE_AV[1:])]
    assert all(abs(g - C.JERK_STEP_V) < 1e-9 for g in gaps)
    gaps_w = [b - a for a, b in zip(C.LATTICE_AW, C.LATTICE_AW[1:])]
    assert all(g <= C.JERK_STEP_W + 1e-9 for g in gaps_w)
    assert 0.0 in C.LATTICE_AV and 0.0 in C.LATTICE_AW
