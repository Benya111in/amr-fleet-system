"""
2D-DWA 팔 (B5) — 팀 DWA 코어(바인딩 amr_dwa2d_py)의 배포 설정 읽기와 env 묶음 관리.

설정은 손으로 고르지 않는다. ROS 래퍼 `src/amr_navigation/src/ros/dwa_controller.cpp` `configure()` (95–224 행) 의
`param(node, p + "<키>", <기본값>)` 을 그대로 옮긴 표 `PARAMS` 로, 배포 파일
`src/amr_navigation/config/nav2_params.yaml` `controller_server.DWA` (값이 있으면 그 값, 행 번호를 기록) →
없으면 `config/robot_params.yaml` `limits.*` (래퍼가 한계의 기본값으로 쓰는 것) → 없으면 래퍼의 기본값 순으로 정한다.
시험 `tests/test_dwa2d.py` 가 이 표를 dwa_controller.cpp 원문과 정규식으로 대조한다 (키·기본값이 바뀌면 깨진다).

지역 코스트맵 설정은 `nav2_params.yaml` `local_costmap` (rolling 8 × 8 m, 0.025 m, scan 마킹 5.5 m ·
레이트레이스 6.0 m, inflation 0.8 m · s 3.0, 내접 반경 = 풋프린트 내접 0.20 m) 과 `costmap_scan_filter_node`
(게이트 중앙값 5 · 0.15 · 3) 에서 읽는다. 제어 주기 = `controller_server.controller_frequency` (20 Hz).
"""

import glob
import importlib
import math
import os
import subprocess
import sys

import numpy as np
import yaml

from ._contract import C, REPO_DIR, SIM2D_DIR

BUILD_DIR = os.path.join(SIM2D_DIR, 'dwa_binding', 'build')
NAV2_YAML = os.path.join(REPO_DIR, 'src', 'amr_navigation', 'config', 'nav2_params.yaml')
ROBOT_YAML = os.path.join(REPO_DIR, 'config', 'robot_params.yaml')
WRAPPER_CPP = os.path.join(REPO_DIR, 'src', 'amr_navigation', 'src', 'ros', 'dwa_controller.cpp')
_MOD = None

# (DwaConfig 필드, nav2 DWA 키, 래퍼 기본값, robot_params limits 키 또는 None)
# 래퍼 기본값이 다른 param() 이면 (95–105, 127 행) 그 limits 키를 넷째 칸에 둔다.
PARAMS = [
    ('limits.max_vel_x', 'max_vel_x', None, 'max_linear_velocity'),   # min(1.0, limits) (97 행)
    ('limits.min_vel_x', 'min_vel_x', 0.0, None),
    ('limits.max_vel_theta', 'max_vel_theta', 1.5, 'max_angular_velocity'),
    ('limits.acc_lim_x', 'acc_lim_x', 1.0, 'max_linear_acceleration'),
    ('limits.decel_lim_x', 'decel_lim_x', 1.0, 'max_linear_acceleration'),
    ('limits.acc_lim_theta', 'acc_lim_theta', 2.0, 'max_angular_acceleration'),
    ('limits.jerk_lim_x', 'jerk_lim_x', 2.0, 'max_linear_jerk'),
    ('weights.heading', 'heading_weight', 0.6, None),
    ('weights.clearance', 'clearance_weight', 1.0, None),
    ('weights.velocity', 'velocity_weight', 0.4, None),
    ('weights.path', 'path_weight', 2.0, None),
    ('weights.oscillation', 'oscillation_weight', 0.5, None),
    ('weights.dynamic', 'dynamic_weight', 1.5, None),
    ('weights.off_path', 'off_path_weight', 3.0, None),
    ('weights.escape', 'escape_weight', 2.5, None),
    ('vx_samples', 'vx_samples', 11, None),
    ('vth_samples', 'vth_samples', 31, None),
    ('sustained_sampling', 'sustained_sampling', False, None),
    ('jerk_lim_theta', 'jerk_lim_theta', 6.0, 'max_angular_jerk'),
    ('sim_dt', 'sim_dt', 0.1, None),
    ('sim_time_min', 'sim_time_min', 1.5, None),
    ('sim_time_max', 'sim_time_max', 2.5, None),
    ('commit_time', 'commit_time', 0.2, None),
    ('approach_latency', 'approach_latency', 0.3, None),
    ('window_reset_v', 'window_reset_v', 0.3, None),
    ('window_reset_w', 'window_reset_w', 0.5, None),
    ('heading_lookahead_gain', 'heading_lookahead_gain', 0.4, None),
    ('heading_lookahead_offset', 'heading_lookahead_offset', 0.3, None),
    ('heading_lookahead_min', 'heading_lookahead_min', 0.4, None),
    ('heading_lookahead_max', 'heading_lookahead_max', 1.2, None),
    ('path_band', 'path_band', 0.8, None),
    ('max_path_offset', 'max_path_offset', 0.9, None),
    ('max_path_offset_hard', 'max_path_offset_hard', 0.95, None),
    ('off_path_band', 'off_path_band', 0.05, None),
    ('path_eval_time', 'path_eval_time', 0.8, None),
    ('goal_align_distance', 'goal_align_distance', 0.08, None),
    ('goal_overshoot_margin', 'goal_overshoot_margin', 0.1, None),
    ('oscillation_w_eps', 'oscillation_w_eps', 0.05, None),
    ('use_dynamic_obstacles', 'use_dynamic_obstacles', True, None),
    ('use_velocity_obstacles', 'use_velocity_obstacles', True, None),
    ('prediction_time', 'prediction_time', 5.0, None),
    ('dynamic_margin', 'dynamic_margin', 0.5, None),
    ('vo_time_horizon', 'vo_time_horizon', 2.0, None),
    ('vo_margin', 'vo_margin', 0.3, None),
    ('vo_max_range', 'vo_max_range', 6.0, None),
    ('dynamic_speed_threshold', 'dynamic_speed_threshold', 0.2, None),
    ('dynamic_steer_gain', 'dynamic_steer_gain', 0.3, None),
    ('vo_time_tie', 'vo_time_tie', 0.02, None),
    ('yield_crossing', 'yield_crossing', True, None),
    ('yield_corridor_margin', 'yield_corridor_margin', 0.50, None),
    ('yield_stop_margin', 'yield_stop_margin', 0.25, None),
    ('yield_clear_margin', 'yield_clear_margin', 1.0, None),
    ('yield_horizon', 'yield_horizon', 8.0, None),
    ('yield_nominal_speed', 'yield_nominal_speed', 1.0, None),
    ('yield_hold_standoff', 'yield_hold_standoff', 0.35, None),
    ('yield_approach_radius', 'yield_approach_radius', 3.0, None),
    ('yield_approach_horizon', 'yield_approach_horizon', 3.0, None),
    ('yield_max_zone', 'yield_max_zone', 6.0, None),
    ('yield_escape_speed', 'yield_escape_speed', 0.25, None),
    ('yield_escape_drift', 'yield_escape_drift', 0.15, None),
    ('yield_escape_v_meas', 'yield_escape_v_meas', 0.05, None),
    ('escape_min_speed', 'escape_min_speed', 0.05, None),
    ('yield_decision', 'yield_decision', True, None),
    ('yield_body_margin', 'yield_body_margin', 0.15, None),
    ('yield_go_margin', 'yield_go_margin', 0.0, None),
    ('yield_go_clearance', 'yield_go_clearance', 0.40, None),
    ('yield_go_hold_clearance', 'yield_go_hold_clearance', 0.30, None),
    ('yield_go_speed_floor', 'yield_go_speed_floor', 1.0, None),
    ('yield_hold_slack', 'yield_hold_slack', 0.3, None),
    ('yield_decision_min_sin', 'yield_decision_min_sin', 0.34, None),
    ('gate.enable', 'yield_gate', False, None),
    ('gate.pos_err', 'gate_pos_err', 0.25, None),
    ('gate.heading_unc', 'gate_heading_unc', 0.1, None),
    ('gate.v_max', 'gate_v_max', 1.0, None),
    ('gate.delay', 'gate_delay', 0.3, None),
    ('gate.hold_back', 'gate_hold_back', 1.2, None),
    ('gate.turn_time', 'gate_turn_time', 1.57, None),
    ('gate.walk_accel', 'gate_walk_accel', 0.6, None),
    ('gate.vknown', 'gate_vknown', 0.3, None),
    ('gate.static_timeout', 'gate_static_timeout', 5.0, None),
    ('gate.lookahead', 'gate_lookahead', 8.0, None),
    ('gate.group_gap', 'gate_group_gap', 1.5, None),
    ('gate.hold_tol', 'gate_hold_tol', 0.3, None),
    ('gate.hold_threat_horizon', 'gate_hold_threat_horizon', 3.0, None),
    ('gate.hold_exclusion_horizon', 'gate_hold_exclusion_horizon', 10.0, None),
    ('gate.commit_max_run', 'gate_commit_max_run', 2.0, None),
    ('gate.retreat_enable', 'gate_retreat', True, None),
    ('gate.retreat_speed', 'gate_retreat_speed', 0.5, None),
    ('gate.retreat_max', 'gate_retreat_max', 4.0, None),
    ('recenter_narrow', 'recenter_narrow', True, None),
    ('recenter_min_cost', 'recenter_min_cost', 100.0, None),
    ('recenter_max_shift', 'recenter_max_shift', 0.10, None),
    ('recenter_smooth', 'recenter_smooth', 4, None),
    ('recenter_goal_keep', 'recenter_goal_keep', 0.5, None),
]
# 래퍼 멤버 (DwaConfig 밖): (이름, 키, 기본값)
WRAPPER = [('path_horizon', 'path_horizon', 6.0), ('yield_lookahead', 'yield_lookahead', 4.0),
           ('no_valid_patience', 'no_valid_patience', 10),
           ('allow_unknown', 'allow_unknown', True), ('robot_radius', 'robot_radius', -1.0)]


def _line_index(path):
    """YAML 원문 → {(블록 경로 tuple): 행 번호} — 들여쓰기로 계층을 따라간다 (출처 표기용)."""
    out = {}
    stack = []
    with open(path, encoding='utf-8') as f:
        for ln, raw in enumerate(f, 1):
            s = raw.split('#', 1)[0].rstrip()
            if not s.strip() or ':' not in s:
                continue
            ind = len(s) - len(s.lstrip())
            key = s.strip().split(':', 1)[0].strip().strip('"')
            while stack and stack[-1][0] >= ind:
                stack.pop()
            stack.append((ind, key))
            out[tuple(k for _, k in stack)] = ln
    return out


def load_config(nav2_yaml=NAV2_YAML, robot_yaml=ROBOT_YAML):
    """
    배포 DWA 설정 → dict(cfg=DwaConfig dict, wrapper=래퍼 값, costmap=지역 코스트맵, footprint, sources).

    sources: 필드 → '파일:행' 또는 'dwa_controller.cpp 기본값' (값이 어디서 왔는지).
    """
    with open(nav2_yaml, encoding='utf-8') as f:
        nav = yaml.safe_load(f)
    with open(robot_yaml, encoding='utf-8') as f:
        rob = yaml.safe_load(f)
    limits = _find_limits(rob)
    lines_n = _line_index(nav2_yaml)
    lines_r = _line_index(robot_yaml)
    rel_n = os.path.relpath(nav2_yaml, REPO_DIR)
    rel_r = os.path.relpath(robot_yaml, REPO_DIR)
    cs = nav['controller_server']['ros__parameters']
    dwa = cs['DWA']
    cfg, src = {}, {}

    def lim_src(k):
        hit = [v for kk, v in lines_r.items() if kk[-2:] == ('limits', k)]
        return f'{rel_r}:{hit[0]}' if hit else rel_r
    for field, key, default, lim in PARAMS:
        if key in dwa:
            val = dwa[key]
            src[field] = f'{rel_n}:{lines_n[("controller_server", "ros__parameters", "DWA", key)]}'
        elif lim is not None and lim in limits:
            # 97 행: max_vel_x 의 기본값은 min(1.0, limits.max_linear_velocity)
            val = min(1.0, float(limits[lim])) if field == 'limits.max_vel_x' else limits[lim]
            src[field] = f'{lim_src(lim)} (래퍼 기본값 = limits.{lim})'
        else:
            val = 1.0 if field == 'limits.max_vel_x' else default
            src[field] = 'dwa_controller.cpp 기본값'
        cfg[field] = val
    freq = float(cs.get('controller_frequency', 20.0))
    cfg['control_period'] = 1.0 / max(1.0, freq)
    cs_line = lines_n[('controller_server', 'ros__parameters', 'controller_frequency')]
    src['control_period'] = f'{rel_n}:{cs_line} (1/controller_frequency)'
    wrap = {}
    for name, key, default in WRAPPER:
        if key in dwa:
            wrap[name] = dwa[key]
            src[name] = f'{rel_n}:{lines_n[("controller_server", "ros__parameters", "DWA", key)]}'
        else:
            wrap[name] = default
            src[name] = 'dwa_controller.cpp 기본값'
    # 221 행: yield_lookahead = min(path_horizon, yield_lookahead)
    cfg['yield_lookahead'] = min(float(wrap['path_horizon']), float(wrap['yield_lookahead']))
    src['yield_lookahead'] = src['yield_lookahead'] + ' (min(path_horizon, ·), 221 행)'
    lc = nav['local_costmap']['local_costmap']['ros__parameters']
    fp = np.array(yaml.safe_load(lc['footprint']), dtype=np.float64)
    # 210 행: recenter_step = 지역 코스트맵 해상도
    cfg['recenter_step'] = float(lc['resolution'])
    lc_key = ('local_costmap', 'local_costmap', 'ros__parameters')
    src['recenter_step'] = (f'{rel_n}:{lines_n[lc_key + ("resolution",)]}'
                            ' (dwa_controller.cpp:210 = 지역 코스트맵 해상도)')
    # 211–213 행: robot_radius ≤ 0 → 풋프린트 외접 반경
    rr = float(wrap['robot_radius'])
    cfg['robot_radius'] = rr if rr > 0 else float(np.max(np.hypot(fp[:, 0], fp[:, 1])))
    src['robot_radius'] = src['robot_radius'] + ' (≤ 0 → 풋프린트 외접 반경, 211–213 행)'
    infl = lc['inflation_layer']
    scan = lc['obstacle_layer']['scan']
    filt = nav['costmap_scan_filter_node']['ros__parameters']
    pad = float(lc.get('footprint_padding', 0.0))
    ins = _inscribed(fp) + pad
    costmap = {
        'size_m': float(lc['width']), 'resolution': float(lc['resolution']),
        'obstacle_max_range': float(scan['obstacle_max_range']),
        'raytrace_max_range': float(scan['raytrace_max_range']),
        'inscribed_radius': ins, 'inflation_radius': float(infl['inflation_radius']),
        'cost_scaling_factor': float(infl['cost_scaling_factor']),
        'footprint_clearing': True, 'denoise': True,
        'denoise_half_window': int(filt['half_window']),
        'denoise_range_gate': float(filt['range_gate']),
        'denoise_min_support': int(filt['min_support'])}
    if float(lc['width']) != float(lc['height']):
        raise ValueError('지역 코스트맵이 정사각형이 아니다 — 바인딩은 정사각형만')
    l0 = lines_n[lc_key + ('width',)]
    l1 = lines_n[lc_key + ('inflation_layer', 'inflation_radius')]
    l2 = lines_n[('costmap_scan_filter_node', 'ros__parameters', 'half_window')]
    src['costmap'] = f'{rel_n}:{l0}-{l1}, costmap_scan_filter_node {rel_n}:{l2}'
    return {'cfg': cfg, 'wrapper': wrap, 'costmap': costmap, 'footprint': fp, 'sources': src,
            'controller_frequency': freq}


def _find_limits(doc):
    """robot_params.yaml 의 limits 블록 (위치가 바뀌어도 찾는다)."""
    if isinstance(doc, dict):
        if 'limits' in doc and isinstance(doc['limits'], dict) and \
                'max_linear_velocity' in doc['limits']:
            return doc['limits']
        for v in doc.values():
            r = _find_limits(v)
            if r:
                return r
    return {}


def _inscribed(fp):
    """원점에서 다각형 변까지 최소 거리 (FootprintChecker::inscribedRadius)."""
    r = math.inf
    for i in range(len(fp)):
        a, b = fp[i], fp[(i + 1) % len(fp)]
        d = b - a
        ln2 = float(d @ d)
        t = 0.0 if ln2 < 1e-12 else min(1.0, max(0.0, -float(a @ d) / ln2))
        r = min(r, math.hypot(a[0] + t * d[0], a[1] + t * d[1]))
    return r


def module(build=True):
    """amr_dwa2d_py 를 import (없으면 build_dwa2d.sh 로 빌드)."""
    global _MOD
    if _MOD is not None:
        return _MOD
    if not glob.glob(os.path.join(BUILD_DIR, 'amr_dwa2d_py*.so')) and build:
        subprocess.run(['bash', os.path.join(SIM2D_DIR, 'dwa_binding', 'build_dwa2d.sh')],
                       check=True)
    if BUILD_DIR not in sys.path:
        sys.path.insert(0, BUILD_DIR)
    _MOD = importlib.import_module('amr_dwa2d_py')
    return _MOD


def path_poses(xy):
    """경로 점 [K,2] → 자세 [K,3] (yaw = 다음 점 방향, 끝점은 마지막 선분 방향 = 경로 끝 접선)."""
    xy = np.asarray(xy, dtype=np.float64)
    d = np.diff(xy, axis=0)
    yaw = np.arctan2(d[:, 1], d[:, 0])
    yaw = np.concatenate([yaw, yaw[-1:]]) if len(yaw) else np.zeros(len(xy))
    return np.concatenate([xy, yaw[:, None]], axis=1)


class DwaBatch:
    """env 마다 Dwa2D 하나 (지역 코스트맵·경로 창·직전 명령 기억을 env 별로 이월)."""

    def __init__(self, n, conf=None, threads=8):
        """만든다: env n 개, conf = load_config() 결과, threads = C++ 스레드 수."""
        m = module()
        self.m = m
        self.conf = conf or load_config()
        w = self.conf['wrapper']
        self.dwas = [m.Dwa2D(self.conf['cfg'], self.conf['costmap'], self.conf['footprint'],
                             float(w['path_horizon']), int(w['no_valid_patience']),
                             bool(w['allow_unknown'])) for _ in range(n)]
        self.threads = int(threads)
        self.amin = float(C.LIDAR_ANGLE_MIN)
        self.ainc = float(C.LIDAR_ANGLE_INC)
        self.rmin = float(C.LIDAR_RANGE_MIN)
        self.rmax = float(C.LIDAR_RANGE_MAX)

    def reset(self, i, path):
        """새 에피소드 (env i): 경로 [K,2] (yaw = 다음 점 방향) 또는 [K,3] (계획기 자세) + 기억·코스트맵 비우기."""
        path = np.asarray(path, dtype=np.float64)
        self.dwas[i].reset(path if path.shape[1] == 3 else path_poses(path))

    def update_costmap(self, idx, sensor, robot, ranges):
        """지역 코스트맵 갱신: env 번호 idx [n], 스캔 ranges [n,720], sensor·robot 자세 [n,3]."""
        if len(idx) == 0:
            return
        self.m.costmap_batch(self.dwas, np.asarray(idx, dtype=np.int64),
                             np.ascontiguousarray(sensor, dtype=np.float64),
                             np.ascontiguousarray(robot, dtype=np.float64),
                             np.ascontiguousarray(ranges, dtype=np.float32),
                             self.amin, self.ainc, self.rmin, self.rmax, self.threads)

    def step(self, idx, pose, v, w):
        """제어 틱 한 번 (env 번호 idx [n]) → dict(v, w, found, exception, n_valid, n_collision, ...)."""
        return self.m.step_batch(self.dwas, np.asarray(idx, dtype=np.int64),
                                 np.ascontiguousarray(pose, dtype=np.float64),
                                 np.ascontiguousarray(v, dtype=np.float64),
                                 np.ascontiguousarray(w, dtype=np.float64), self.threads)
