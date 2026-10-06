#!/usr/bin/env python3
"""
전이 점검 1 — 풋프린트·속도·가속 한계 대조표 [계산] (설정 파일 원문 → 표).

    python3 rl/eval/gazebo/transfer_limits.py [--out <json>]

열: 계약(rl/contract constants.py — Isaac·2D 가 쓰는 값) ·
배포 설정(config/robot_params.yaml, nav2 DWA, velocity_profiler) ·
Gazebo 플랜트(amr_gazebo.xacro DiffDrive — robot_params 를 읽는다) ·
Isaac 플랜트(rl/isaac env.py: 가속 한계 = 계약 a·배율,
감속도 같은 한계). 값을 파일에서 읽어 같은지 비교한다 (손으로 옮겨 적지 않는다).
"""

import argparse
import json
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'rl', 'contract'))

from amr_rl_contract import constants as C  # noqa: E402


def load_yaml(*parts):
    """저장소 YAML."""
    with open(os.path.join(REPO, *parts), encoding='utf-8') as f:
        return yaml.safe_load(f)


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rp = load_yaml('config', 'robot_params.yaml')['/**']['ros__parameters']
    lim, rob = rp['limits'], rp['robot']
    nav = load_yaml('src', 'amr_navigation', 'config', 'nav2_params.yaml')
    dwa = nav['controller_server']['ros__parameters']['DWA']
    vp = load_yaml('src', 'amr_navigation', 'config',
                   'velocity_profiler.yaml')['/**']['ros__parameters']
    xacro = open(os.path.join(REPO, 'src', 'amr_description', 'urdf', 'amr_gazebo.xacro'),
                 encoding='utf-8').read()
    dd = xacro[xacro.index('ignition-gazebo-diff-drive-system'):]
    dd = dd[:dd.index('</plugin>')]
    tags = dict(re.findall(r'<(\w+)>\$\{([^}]*)\}</\1>', dd))
    env = open(os.path.join(REPO, 'rl', 'isaac', 'amr_isaac', 'env.py'), encoding='utf-8').read()
    isaac_decel_sym = ('-C.A_MAX * self.acc_scale * PHYS_DT, C.A_MAX * self.acc_scale * PHYS_DT'
                       in env)
    cfg = open(os.path.join(REPO, 'rl', 'isaac', 'amr_isaac', 'env_cfg.py'),
               encoding='utf-8').read()
    acc_scale = re.search(r'accel_limit_scale: tuple = \(([^)]*)\)', cfg).group(1)
    rows = [
        ('풋프린트 길이 [m]', C.FOOTPRINT_LENGTH, rob['footprint_length'], 'URDF·nav2 footprint',
         '같은 계약 값 (env_math 충돌 사각형)'),
        ('풋프린트 폭 [m]', C.FOOTPRINT_WIDTH, rob['footprint_width'], '', '같은 계약 값'),
        ('운용 속도 상한 v_cap [m/s]', C.V_MAX_OP, dwa['max_vel_x'], '(DWA max_vel_x)', None),
        ('하드웨어 v [m/s]', C.V_MAX_HW, lim['max_linear_velocity'],
         tags.get('max_linear_velocity'), None),
        ('ω 한계 [rad/s]', C.W_MAX, lim['max_angular_velocity'],
         tags.get('max_angular_velocity'), None),
        ('가속 a [m/s²]', C.A_MAX, lim['max_linear_acceleration'],
         tags.get('max_linear_acceleration'), f'C.A_MAX × 배율 {acc_scale}'),
        ('감속 [m/s²]', C.A_MAX, lim['max_linear_acceleration'],
         f"{tags.get('min_linear_acceleration')} (= −emergency {lim['emergency_deceleration']})",
         'C.A_MAX × 배율 (대칭)' if isaac_decel_sym else '?'),
        ('각가속 α [rad/s²]', C.ALPHA_MAX, lim['max_angular_acceleration'],
         tags.get('max_angular_acceleration'), f'C.ALPHA_MAX × 배율 {acc_scale}'),
        ('선저크 [m/s³]', C.JERK_MAX, lim['max_linear_jerk'], '없음 (DiffDrive 에 안 넣음)',
         '프로파일러 (계약 기준 경로)'),
        ('각저크 [rad/s³]', C.ANG_JERK_MAX, vp['max_angular_jerk'], '없음', '프로파일러'),
        ('플랜트 T_m [s]', None, vp['model_time_constant'], '점검 5 측정', '0.048–0.112 (공칭 0.08)'),
        ('플랜트 T_d [s]', None, vp['model_delay'], '점검 5 측정', '0.02–0.08 (공칭 0.04)'),
    ]
    out = []
    for name, contract, deploy, gazebo, isaac in rows:
        out.append({'item': name, 'contract': contract, 'deploy': deploy, 'gazebo_plant': gazebo,
                    'isaac_plant': isaac,
                    'contract_eq_deploy': None if contract is None else
                    abs(float(contract) - float(deploy)) < 1e-9})
    res = {'rows': out, 'diffdrive_tags': tags, 'isaac_decel_symmetric': isaac_decel_sym}
    txt = json.dumps(res, ensure_ascii=False, indent=1)
    print(txt)
    if a.out:
        open(a.out, 'w', encoding='utf-8').write(txt)


if __name__ == '__main__':
    main()
