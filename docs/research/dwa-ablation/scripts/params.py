#!/usr/bin/env python3
"""
제어기 설정 원천: 배포 nav2_params.yaml 의 블록 + 팔(arm) 정의 (prereg.md §3).

- chain_sim 용 cfg 파일 ("key value" 줄) 을 배포 DWA·PurePursuit 블록에서 만든다.
- ROS 시험대용 nav2 파라미터 파일: 배포 파일의 controller_server.controller_plugins 를 비교 팔
  전부로 바꾸고, 팔마다 블록을 더한다 (DWB·MPPI·RPP 의 기본·격자, 우리 DWA 절제 팔, PP).
"""
import copy
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
NAV2_PARAMS = os.path.join(REPO, 'src', 'amr_navigation', 'config', 'nav2_params.yaml')

# --- 우리 DWA 절제 팔 (배포 블록 위에 덮는 값). 끈 쪽 = 교과서 (prereg.md §2) ---
DWA_ARMS = {
    'dwa_full': {},
    'dwa_no_m1': {'textbook_cost': True},
    'dwa_no_m1c': {'relative_clearance': False},
    'dwa_no_m1e': {'path_eval_time': 0.0},
    'dwa_no_m2': {'goal_approach': 'none'},
    'dwa_m2_trap': {'goal_approach': 'trapezoid'},
    'dwa_no_m3': {'window_center_measured': True},
    'dwa_no_m4': {'brake_candidate': False, 'sim_time_min': 1.5, 'sim_time_max': 1.5},
    'dwa_no_m4a': {'sim_time_min': 1.5, 'sim_time_max': 1.5},
    'dwa_no_m5': {'recenter_narrow': False},
    'dwa_textbook': {'textbook_cost': True, 'goal_approach': 'none',
                     'window_center_measured': True, 'brake_candidate': False,
                     'sim_time_min': 1.5, 'sim_time_max': 1.5, 'recenter_narrow': False,
                     'relative_clearance': False, 'path_eval_time': 0.0,
                     'use_velocity_obstacles': False, 'dynamic_weight': 0.0,
                     'yield_crossing': False},
}


# --- Nav2 비교 제어기 (prereg.md §3). "기본" = Nav2 Humble 문서·nav2_bringup 예시 블록 그대로에
#     로봇 한계(최고 1.0 m/s, 후진 없음, ω 1.5 rad/s, 가속 1.0 m/s², 각가속 2.0 rad/s²)와 목표
#     허용치(0.10 m)만 바꾼 것. 출처: nav2_bringup/params/nav2_params.yaml (humble 1.1.20, 이미지
#     amr-ai/rl:0.3), navigation2 humble 브랜치 nav2_mppi_controller/README.md ·
#     nav2_regulated_pure_pursuit_controller/README.md (runs/dwa-abl/ref/ 에 사본·sha256).
DWB_DEFAULT = {
    'plugin': 'dwb_core::DWBLocalPlanner', 'debug_trajectory_details': False,
    'min_vel_x': 0.0, 'min_vel_y': 0.0, 'max_vel_x': 1.0, 'max_vel_y': 0.0,
    'max_vel_theta': 1.5, 'min_speed_xy': 0.0, 'max_speed_xy': 1.0, 'min_speed_theta': 0.0,
    'acc_lim_x': 1.0, 'acc_lim_y': 0.0, 'acc_lim_theta': 2.0, 'decel_lim_x': -1.0,
    'decel_lim_y': 0.0, 'decel_lim_theta': -2.0, 'vx_samples': 20, 'vy_samples': 1,
    'vtheta_samples': 20, 'sim_time': 1.7, 'linear_granularity': 0.05,
    'angular_granularity': 0.025, 'transform_tolerance': 0.2, 'xy_goal_tolerance': 0.10,
    'trans_stopped_velocity': 0.25, 'short_circuit_trajectory_evaluation': True,
    'stateful': True,
    'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign', 'PathAlign',
                'PathDist', 'GoalDist'],
    'BaseObstacle.scale': 0.02, 'PathAlign.scale': 32.0, 'PathAlign.forward_point_distance': 0.1,
    'GoalAlign.scale': 24.0, 'GoalAlign.forward_point_distance': 0.1, 'PathDist.scale': 32.0,
    'GoalDist.scale': 24.0, 'RotateToGoal.scale': 32.0, 'RotateToGoal.slowing_factor': 5.0,
    'RotateToGoal.lookahead_time': -1.0}
DWB_E = {'RotateToGoal.slowing_factor': 1.0, 'RotateToGoal.lookahead_time': 0.3,
         'sim_time': 1.5, 'vx_samples': 20, 'vtheta_samples': 40, 'PathDist.scale': 48.0,
         'trans_stopped_velocity': 0.1}     # dwa.md §8.1 의 DWB_E (지난 튜닝)
DWB_GRID = {
    'D00': {},
    'D01': dict(DWB_E),
    'D02': {'sim_time': 1.0},
    'D03': {'sim_time': 1.5},
    'D04': {'PathDist.scale': 48.0},
    'D05': {'PathDist.scale': 64.0},
    'D06': {'RotateToGoal.slowing_factor': 1.0, 'RotateToGoal.lookahead_time': 0.3},
    'D07': {'trans_stopped_velocity': 0.1},
    'D08': {'vtheta_samples': 40},
    'D09': dict(DWB_E, **{'PathAlign.scale': 16.0}),
    'D10': dict(DWB_E, **{'sim_time': 1.0}),
    'D11': dict(DWB_E, **{'PathDist.scale': 64.0}),
}

MPPI_DEFAULT = {
    'plugin': 'nav2_mppi_controller::MPPIController', 'time_steps': 56, 'model_dt': 0.05,
    'batch_size': 2000, 'vx_std': 0.2, 'vy_std': 0.2, 'wz_std': 0.4, 'vx_max': 1.0,
    'vx_min': 0.0, 'vy_max': 0.5, 'wz_max': 1.5, 'iteration_count': 1, 'prune_distance': 1.7,
    'transform_tolerance': 0.1, 'temperature': 0.3, 'gamma': 0.015, 'motion_model': 'DiffDrive',
    'visualize': False,
    'critics': ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
                'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic', 'PreferForwardCritic'],
    'ConstraintCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0},
    'GoalCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0,
                   'threshold_to_consider': 1.4},
    'GoalAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.0,
                        'threshold_to_consider': 0.5},
    'PreferForwardCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0,
                            'threshold_to_consider': 0.5},
    'CostCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.81, 'critical_cost': 300.0,
                   'consider_footprint': True, 'collision_cost': 1000000.0,
                   'near_goal_distance': 1.0},
    'PathAlignCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 14.0,
                        'max_path_occupancy_ratio': 0.05, 'trajectory_point_step': 3,
                        'threshold_to_consider': 0.5, 'offset_from_furthest': 20,
                        'use_path_orientations': False},
    'PathFollowCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0,
                         'offset_from_furthest': 5, 'threshold_to_consider': 1.4},
    'PathAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 2.0,
                        'offset_from_furthest': 4, 'threshold_to_consider': 0.5,
                        'max_angle_to_furthest': 1.0, 'forward_preference': True},
}
MPPI_GRID = {
    'P00': {},
    'P01': {'PathFollowCritic.cost_weight': 10.0},
    'P02': {'PathAlignCritic.cost_weight': 20.0},
    'P03': {'PathAlignCritic.cost_weight': 20.0, 'PathFollowCritic.cost_weight': 10.0},
    'P04': {'temperature': 0.15},
    'P05': {'vx_std': 0.3, 'wz_std': 0.6},
    'P06': {'GoalAngleCritic.threshold_to_consider': 0.25,
            'GoalCritic.threshold_to_consider': 1.0},
    'P07': {'time_steps': 40},
    'P08': {'batch_size': 1000},
    'P09': {'PathAngleCritic.cost_weight': 4.0},
    'P10': {'PathAlignCritic.cost_weight': 20.0, 'PathFollowCritic.cost_weight': 10.0,
            'vx_std': 0.3, 'wz_std': 0.6},
    'P11': {'PathAlignCritic.cost_weight': 20.0, 'PathFollowCritic.cost_weight': 10.0,
            'temperature': 0.15},
}

RPP_DEFAULT = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': 1.0, 'lookahead_dist': 0.6, 'min_lookahead_dist': 0.3,
    'max_lookahead_dist': 0.9, 'lookahead_time': 1.5, 'rotate_to_heading_angular_vel': 1.5,
    'transform_tolerance': 0.1, 'use_velocity_scaled_lookahead_dist': False,
    'min_approach_linear_velocity': 0.05, 'approach_velocity_scaling_dist': 1.0,
    'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 1.0,
    'use_regulated_linear_velocity_scaling': True,
    'use_cost_regulated_linear_velocity_scaling': False,
    'regulated_linear_scaling_min_radius': 0.9, 'regulated_linear_scaling_min_speed': 0.25,
    'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': 0.785,
    'max_angular_accel': 2.0, 'max_robot_pose_search_dist': 10.0, 'use_interpolation': False,
    'cost_scaling_dist': 0.3, 'cost_scaling_gain': 1.0, 'inflation_cost_scaling_factor': 3.0,
    'allow_reversing': False}
RPP_OURS = {   # 배포 nav2_params.yaml 의 RPP 블록 (path_tracking.md §5.1 비교에 쓴 값)
    'lookahead_dist': 0.8, 'min_lookahead_dist': 0.4, 'max_lookahead_dist': 1.8,
    'lookahead_time': 0.8, 'transform_tolerance': 0.2, 'use_velocity_scaled_lookahead_dist': True,
    'approach_velocity_scaling_dist': 0.6, 'use_interpolation': True}
RPP_VS = {'use_velocity_scaled_lookahead_dist': True, 'lookahead_dist': 0.8,
          'min_lookahead_dist': 0.4, 'max_lookahead_dist': 1.2, 'lookahead_time': 0.8}
RPP_GRID = {
    'R00': {},
    'R01': dict(RPP_OURS),
    'R02': {'use_velocity_scaled_lookahead_dist': True},
    'R03': dict(RPP_VS),
    'R04': {'approach_velocity_scaling_dist': 1.5},
    'R05': {'approach_velocity_scaling_dist': 2.0, 'min_approach_linear_velocity': 0.02},
    'R06': {'regulated_linear_scaling_min_radius': 0.5},
    'R07': dict(RPP_VS, approach_velocity_scaling_dist=1.5),
    'R08': dict(RPP_VS, approach_velocity_scaling_dist=2.0, min_approach_linear_velocity=0.02),
    'R09': {'use_regulated_linear_velocity_scaling': False},
    'R10': dict(RPP_OURS, approach_velocity_scaling_dist=1.5),
    'R11': dict(RPP_OURS, approach_velocity_scaling_dist=2.0, min_approach_linear_velocity=0.02),
}


def nested_update(block, over):
    """'Critic.key' 점 표기를 MPPI 의 중첩 dict 로 반영한다 (DWB 는 점 표기가 실제 키라 그대로)."""
    out = copy.deepcopy(block)
    mppi = out.get('plugin', '').startswith('nav2_mppi')
    for k, v in over.items():
        if mppi and '.' in k:
            c, kk = k.split('.', 1)
            out[c][kk] = v
        else:
            out[k] = v
    return out


def nav2_arm_blocks():
    """controller_id → 플러그인 블록 (Nav2 비교 제어기 36 개)."""
    arms = {}
    for gid, over in DWB_GRID.items():
        arms[f'DWB_{gid}'] = nested_update(DWB_DEFAULT, over)
    for gid, over in MPPI_GRID.items():
        arms[f'MPPI_{gid}'] = nested_update(MPPI_DEFAULT, over)
    for gid, over in RPP_GRID.items():
        arms[f'RPP_{gid}'] = nested_update(RPP_DEFAULT, over)
    return arms


def ours_arm_blocks(nav2):
    arms = {}
    base = controller_block(nav2, 'DWA')
    for arm, over in DWA_ARMS.items():
        b = dict(base)
        b.update(over)
        arms[arm] = b
    arms['pp'] = dict(controller_block(nav2, 'PurePursuit'))
    return arms


def ros_params_file(fname, arm_ids, extra_blocks=None, nav2=None):
    """배포 nav2_params.yaml 에 비교 팔 블록을 더하고 controller_plugins 를 arm_ids 로 바꾼다."""
    nav2 = copy.deepcopy(nav2 or load_nav2())
    cs = nav2['controller_server']['ros__parameters']
    blocks = dict(ours_arm_blocks(nav2))
    blocks.update(nav2_arm_blocks())
    blocks.update(extra_blocks or {})
    for k in ('DWA', 'PurePursuit', 'DWB', 'RPP'):
        cs.pop(k, None)
    cs['controller_plugins'] = list(arm_ids)
    for a in arm_ids:
        cs[a] = blocks[a]
    with open(fname, 'w', encoding='utf-8') as f:
        yaml.safe_dump(nav2, f, sort_keys=False, allow_unicode=True)
    return fname


def load_nav2(path=NAV2_PARAMS):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def controller_block(nav2, name):
    return nav2['controller_server']['ros__parameters'][name]


def to_cfg_lines(block, overrides=None):
    """플러그인 블록 → chain_sim cfg 줄 (스칼라만, plugin·문자열 토픽 등 제외)."""
    b = dict(block)
    b.update(overrides or {})
    skip = {'plugin', 'tracked_obstacles_topic', 'payload_topic', 'robot_mass', 'allow_unknown',
            'transform_tolerance', 'track_timeout', 'yield_velocity_tau', 'robot_radius',
            'obstacle_radius', 'dynamic_fast_speed', 'yield_decision_dwell', 'gate_line_window',
            'gate_track_memory', 'use_rotate_to_heading_dummy'}
    lines = []
    for k, v in b.items():
        if k in skip or k.startswith('gate_') or k.startswith('yield_') and k not in (
                'yield_crossing', 'yield_decision'):
            continue
        if k == 'yield_gate' or isinstance(v, (list, dict)):
            continue
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        lines.append(f'{k} {v}')
    return lines


def dwa_cfg(arm, nav2=None):
    nav2 = nav2 or load_nav2()
    return to_cfg_lines(controller_block(nav2, 'DWA'), DWA_ARMS[arm])


def pp_cfg(nav2=None):
    nav2 = nav2 or load_nav2()
    return to_cfg_lines(controller_block(nav2, 'PurePursuit'))


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    nav2 = load_nav2()
    for arm in DWA_ARMS:
        with open(os.path.join(out, f'{arm}.cfg'), 'w', encoding='utf-8') as f:
            f.write('\n'.join(dwa_cfg(arm, nav2)) + '\n')
    with open(os.path.join(out, 'pp.cfg'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(pp_cfg(nav2)) + '\n')
    print(json.dumps(sorted(os.listdir(out))))
    print(len(nav2_arm_blocks()), 'Nav2 arms')
