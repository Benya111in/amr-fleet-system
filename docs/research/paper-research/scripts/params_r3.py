#!/usr/bin/env python3
"""
3 라운드 제어기 설정 (prereg_r3.md §2·§3). 1 라운드 params_x.py·2 라운드 params_r2.py 를 그대로 쓰고 다음을 더한다.

- Nav2 확장 격자 (각 24 개, 이유는 docs/research/paper-research/grids_r3.md):
  DWB_D12…D35, MPPI_P12…P35, RPP_R12…R35. 1 라운드 격자 D00–D11·P00–P11·R00–R11 은 그대로이고 2 라운드 튜닝
  자료를 다시 쓴다 (prereg_r3 §2 누적 36).
- DWA-R3 격자 Z00–Z11 (prereg_r3 §3 그대로, 모두 Y11 위): 팔 이름 dwa_z_Z00…, 선택된 것 dwa_R3.
  F1 ROS 보정값 d*·τ* 는 fit_f1_model.py 결과 (F1_FIT) 이다.
- 3 라운드 고정 팔: dwa_R (X03), dwa_R2 (Y11), dwa_textbook_tuned (T06).

    python3 params_r3.py <cfg 디렉터리> [<선택 Z id>]   # chain_sim_x cfg
"""
import copy
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import params_r2 as p2  # noqa: E402
import params_x as px  # noqa: E402

# --- F1 ROS 보정값 (fit_f1_model.py, 튜닝-20 의 2 라운드 ROS 궤적; analysis/f1_fit_r3.json) ---
F1_FIT = {'model_delay': 0.01, 'model_tau': 0.06}     # d*, τ* (analysis/f1_fit_r3.json; 반쪽 맞춤 0.01/0.06·0.01/0.07)

# --- G1 (pr-core, CORE R3 READY 의 이름) ---
G1 = 'accel_hold'

Y11 = dict(p2.DWA_Y_GRID['Y11'])                       # F1 θ 0.10 + F2 1.0 + F3 1.0 (research.* 이름)


def z(*over):
    """Y11 위에 덮는다. 키 'research:<이름>' 은 research.*, 'top:<이름>' 은 배포 DWA 파라미터."""
    out = {'research': dict(Y11), 'top': {}}
    for o in over:
        for k, v in o.items():
            kind, name = k.split(':', 1)
            out[kind][name] = v
    return out


F1_OFF = {'research:reset_model': False}
G1_ON = {f'research:{G1}': True}


def f1_cal(theta):
    return {'research:reset_model': True, 'research:reset_model_theta': theta,
            'research:model_delay': F1_FIT['model_delay'], 'research:model_tau': F1_FIT['model_tau']}


def gamma(g):
    return {'top:velocity_weight': g}


def build_z():
    z01 = [F1_OFF]
    z02 = [F1_OFF, G1_ON]
    z03 = [f1_cal(0.10)]
    z05 = [f1_cal(0.10), G1_ON]
    return {
        'Z00': z(),
        'Z01': z(*z01),
        'Z02': z(*z02),
        'Z03': z(*z03),
        'Z04': z(f1_cal(0.15)),
        'Z05': z(*z05),
        'Z06': z(*z02, gamma(0.6)),
        'Z07': z(*z02, gamma(0.8)),
        'Z08': z(*z02, {'research:blocked_detour_shift': 0.8}),
        'Z09': z(*z05, gamma(0.6)),
        'Z10': z(*z02, {'research:reset_persist_cycles': 10}),
        'Z11': z(*z02, {'top:window_reset_v': 0.4}),
    }


def z_arm(zid):
    return f'dwa_z_{zid}'


def dwa_z_block(nav2, zid):
    spec = build_z()[zid]
    b = px.dwa_x_block(nav2, p2.BASE_X)          # X03 블록 (research.reset_persist_cycles 6)
    b.setdefault('research', {}).update(copy.deepcopy(spec['research']))
    b.update(spec['top'])
    return b


def dwa_z_cfg(zid, nav2=None):
    spec = build_z()[zid]
    nav2 = nav2 or px.load_nav2()
    lines = px.to_cfg_lines(px.controller_block(nav2, 'DWA'), dict(spec['top']))
    research = {'reset_persist_cycles': 6}
    research.update(spec['research'])
    for k, v in research.items():
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        lines.append(f'{k} {v}')
    return lines


# --- Nav2 확장 격자 (grids_r3.md 에 이유) ---
DWB_D10 = dict(px.DWB_GRID['D10'])
DWB_D02 = dict(px.DWB_GRID['D02'])
FOOTPRINT = ['RotateToGoal', 'Oscillation', 'ObstacleFootprint', 'GoalAlign', 'PathAlign',
             'PathDist', 'GoalDist']
DWB_NEW = {
    'D12': dict(DWB_D10, **{'critics': FOOTPRINT, 'ObstacleFootprint.scale': 0.02}),
    'D13': dict(DWB_D10, **{'BaseObstacle.scale': 0.2}),
    'D14': dict(DWB_D10, **{'BaseObstacle.scale': 0.5}),
    'D15': dict(DWB_D10, **{'critics': FOOTPRINT, 'ObstacleFootprint.scale': 0.1}),
    'D16': dict(DWB_D02, **{'critics': FOOTPRINT, 'ObstacleFootprint.scale': 0.02}),
    'D17': dict(DWB_D10, **{'critics': FOOTPRINT, 'ObstacleFootprint.scale': 0.02, 'sim_time': 1.5}),
    'D18': dict(DWB_D10, **{'BaseObstacle.scale': 0.2, 'sim_time': 1.5}),
    'D19': dict(DWB_D10, **{'PathDist.scale': 64.0, 'PathAlign.scale': 16.0}),
    'D20': dict(DWB_D10, **{'PathAlign.forward_point_distance': 0.325,
                            'GoalAlign.forward_point_distance': 0.325}),
    'D21': dict(DWB_D10, **{'GoalAlign.scale': 12.0}),
    'D22': dict(DWB_D10, **{'vx_samples': 30, 'vtheta_samples': 60}),
    'D23': dict(DWB_D10, **{'linear_granularity': 0.025, 'angular_granularity': 0.0125}),
    'D24': dict(DWB_D10, **{'sim_time': 1.2}),
    'D25': dict(DWB_D10, **{'sim_time': 0.8}),
    'D26': dict(DWB_D10, **{'trans_stopped_velocity': 0.05}),
    'D27': dict(DWB_D10, **{'RotateToGoal.slowing_factor': 2.0}),
    'D28': dict(DWB_D10, **{'PathDist.scale': 64.0, 'GoalDist.scale': 32.0}),
    'D29': dict(DWB_D10, **{'critics': FOOTPRINT, 'ObstacleFootprint.scale': 0.02,
                            'PathDist.scale': 64.0}),
    'D30': dict(DWB_D02, **{'BaseObstacle.scale': 0.2}),
    'D31': dict(DWB_D10, **{'critics': FOOTPRINT + ['Twirling'], 'ObstacleFootprint.scale': 0.02,
                            'Twirling.scale': 2.0}),
    'D32': dict(DWB_D10, **{'critics': FOOTPRINT, 'ObstacleFootprint.scale': 0.05,
                            'GoalDist.scale': 32.0}),
    'D33': dict(DWB_D10, **{'BaseObstacle.scale': 0.2, 'PathDist.scale': 64.0}),
    'D34': dict(DWB_D10, **{'critics': FOOTPRINT, 'ObstacleFootprint.scale': 0.02,
                            'vx_samples': 30, 'vtheta_samples': 60}),
    'D35': dict(DWB_D10, **{'BaseObstacle.scale': 0.5, 'sim_time': 1.5, 'PathDist.scale': 64.0}),
}

MP05 = dict(px.MPPI_GRID['P05'])                  # vx_std 0.3, wz_std 0.6
MPPI_NEW = {
    'P12': dict(MP05, **{'PathFollowCritic.cost_weight': 10.0,
                         'PathFollowCritic.offset_from_furthest': 8}),
    'P13': dict(MP05, **{'PathFollowCritic.offset_from_furthest': 10}),
    'P14': dict(MP05, **{'vx_std': 0.4}),
    'P15': dict(MP05, **{'vx_std': 0.5, 'wz_std': 0.8}),
    'P16': dict(MP05, **{'time_steps': 80}),
    'P17': dict(MP05, **{'model_dt': 0.07, 'time_steps': 50}),
    'P18': dict(MP05, **{'temperature': 0.15}),
    'P19': dict(MP05, **{'temperature': 0.5}),
    'P20': dict(MP05, **{'gamma': 0.05}),
    'P21': dict(MP05, **{'iteration_count': 2}),
    'P22': dict(MP05, **{'batch_size': 3000}),
    'P23': dict(MP05, **{'PathFollowCritic.cost_weight': 15.0}),
    'P24': dict(MP05, **{'PreferForwardCritic.cost_weight': 10.0}),
    'P25': dict(MP05, **{'GoalCritic.threshold_to_consider': 1.0,
                         'GoalAngleCritic.threshold_to_consider': 0.25}),
    'P26': dict(MP05, **{'PathAngleCritic.cost_weight': 4.0}),
    'P27': dict(MP05, **{'CostCritic.cost_weight': 2.0}),
    'P28': dict(MP05, **{'vx_std': 0.4, 'PathFollowCritic.cost_weight': 10.0,
                         'PathFollowCritic.offset_from_furthest': 8}),
    'P29': dict(MP05, **{'PathAlignCritic.offset_from_furthest': 30,
                         'PathAlignCritic.max_path_occupancy_ratio': 0.07}),
    'P30': dict(MP05, **{'PathAlignCritic.cost_weight': 20.0, 'PathFollowCritic.cost_weight': 10.0,
                         'vx_std': 0.4}),
    'P31': dict(MP05, **{'vx_std': 0.4, 'temperature': 0.15}),
    'P32': dict(MP05, **{'wz_std': 0.8}),
    'P33': dict(MP05, **{'vx_std': 0.4, 'PathFollowCritic.cost_weight': 15.0,
                         'PathFollowCritic.offset_from_furthest': 10, 'time_steps': 70}),
    'P34': dict(MP05, **{'GoalCritic.cost_weight': 10.0}),
    'P35': dict(MP05, **{'vx_std': 0.5, 'PathFollowCritic.cost_weight': 10.0,
                         'PathFollowCritic.offset_from_furthest': 8, 'gamma': 0.05}),
}

RVS = dict(px.RPP_VS)          # R03 의 속도 비례 lookahead
RPP_NEW = {
    'R12': {'max_allowed_time_to_collision_up_to_carrot': 2.0},
    'R13': {'max_allowed_time_to_collision_up_to_carrot': 3.0},
    'R14': {'use_cost_regulated_linear_velocity_scaling': True, 'cost_scaling_dist': 0.6},
    'R15': {'use_cost_regulated_linear_velocity_scaling': True, 'cost_scaling_dist': 1.0},
    'R16': {'max_allowed_time_to_collision_up_to_carrot': 3.0,
            'use_cost_regulated_linear_velocity_scaling': True, 'cost_scaling_dist': 0.6},
    'R17': {'lookahead_dist': 0.8},
    'R18': {'use_velocity_scaled_lookahead_dist': True, 'lookahead_time': 1.0,
            'min_lookahead_dist': 0.5, 'max_lookahead_dist': 1.5},
    'R19': dict(RVS, lookahead_time=0.6),
    'R20': {'use_interpolation': True},
    'R21': {'approach_velocity_scaling_dist': 0.6},
    'R22': {'approach_velocity_scaling_dist': 0.8, 'min_approach_linear_velocity': 0.1},
    'R23': {'min_approach_linear_velocity': 0.1},
    'R24': {'approach_velocity_scaling_dist': 0.4},
    'R25': {'regulated_linear_scaling_min_radius': 0.7},
    'R26': {'regulated_linear_scaling_min_speed': 0.4},
    'R27': {'regulated_linear_scaling_min_radius': 1.2},
    'R28': {'rotate_to_heading_min_angle': 0.5},
    'R29': {'max_angular_accel': 3.0},
    'R30': {'max_allowed_time_to_collision_up_to_carrot': 2.0, 'use_interpolation': True},
    'R31': dict(RVS, max_allowed_time_to_collision_up_to_carrot=2.0),
    'R32': {'approach_velocity_scaling_dist': 0.6, 'min_approach_linear_velocity': 0.1},
    'R33': dict(RVS, use_cost_regulated_linear_velocity_scaling=True, cost_scaling_dist=0.6),
    'R34': {'max_allowed_time_to_collision_up_to_carrot': 2.0,
            'use_cost_regulated_linear_velocity_scaling': True, 'cost_scaling_dist': 0.6,
            'approach_velocity_scaling_dist': 0.8},
    'R35': {'max_allowed_time_to_collision_up_to_carrot': 3.0, 'lookahead_dist': 0.8,
            'use_interpolation': True},
}


def nav2_new_blocks():
    out = {}
    for gid, over in DWB_NEW.items():
        out[f'DWB_{gid}'] = px.nested_update(px.DWB_DEFAULT, over)
    for gid, over in MPPI_NEW.items():
        out[f'MPPI_{gid}'] = px.nested_update(px.MPPI_DEFAULT, over)
    for gid, over in RPP_NEW.items():
        out[f'RPP_{gid}'] = px.nested_update(px.RPP_DEFAULT, over)
    return out


def all_nav2_blocks():
    out = px.nav2_arm_blocks()
    out.update(nav2_new_blocks())
    return out


def extra_blocks(nav2=None, selected_z=None):
    nav2 = nav2 or px.load_nav2()
    out = p2.extra_blocks(nav2, 'Y11')                  # dwa_R2 = Y11 (고정)
    out.update(nav2_new_blocks())
    for zid in build_z():
        out[z_arm(zid)] = dwa_z_block(nav2, zid)
    if selected_z == 'Y11':                              # 2 라운드 선택이 그대로 뽑힌 경우
        out['dwa_R3'] = p2.dwa_y_block(nav2, 'Y11')
    elif selected_z:
        out['dwa_R3'] = dwa_z_block(nav2, selected_z)
    return out


def ros_params_file(fname, arm_ids, selected_z=None, nav2_selected=None, **kw):
    """params_x.ros_params_file + Y·Z·Nav2 확장 블록. dwa_R = X03, dwa_R2 = Y11, 교과서 튜닝 = T06 고정.
    nav2_selected: {'DWB_tuned': 'DWB_Dxx', …} 처럼 선택 팔을 원래 이름으로 쓰므로 여기서는 필요 없다."""
    nav2 = px.load_nav2()
    kw.setdefault('selected_x', p2.BASE_X)
    kw.setdefault('selected_t', 'T06')
    return px.ros_params_file(fname, arm_ids, extra_blocks=extra_blocks(nav2, selected_z), nav2=nav2,
                              **kw)


def write_cfgs(out, selected_z=None):
    names = p2.write_cfgs(out, 'Y11', 'T06')            # dwa_R (X03), dwa_R2 (Y11), dwa_textbook_tuned
    nav2 = px.load_nav2()
    for zid in build_z():
        with open(os.path.join(out, f'{z_arm(zid)}.cfg'), 'w', encoding='utf-8') as f:
            f.write('\n'.join(dwa_z_cfg(zid, nav2)) + '\n')
    if selected_z:
        with open(os.path.join(out, 'dwa_R3.cfg'), 'w', encoding='utf-8') as f:
            f.write('\n'.join(dwa_z_cfg(selected_z, nav2)) + '\n')
    return sorted(os.listdir(out))


if __name__ == '__main__':
    print(json.dumps(write_cfgs(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)))
