#!/usr/bin/env python3
"""
2 라운드 제어기 설정 (prereg_r2.md §2). 1 라운드 params_x.py 를 그대로 쓰고 DWA-R2 격자 Y00–Y11 만 더한다.

- DWA-R2 격자는 모두 1 라운드 선택 DWA-R = X03 (R1 reset_persist_cycles 6) 위에 덮는다.
- 새 스위치 이름(F1·F2·F3)은 pr-core 의 `src/amr_nav_research/README.md` 와 일지 `CORE R2 READY` 줄을 따른다.
  아래 F_PARAM 한 곳에서만 대응을 정한다 (ROS 는 research.<이름>, chain_sim_x cfg 는 <이름>).
- 팔 이름: 격자 dwa_y_Y00…Y11, 선택된 것 dwa_R2. 1 라운드 DWA-R(X03) 은 dwa_R 로 고정 팔이다.

    python3 params_r2.py <cfg 디렉터리> [<선택 Y id>]   # chain_sim_x cfg (1 라운드 cfg 전부 + dwa_y_* + dwa_R2)
"""
import copy
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import params_x as px  # noqa: E402

BASE_X = 'X03'          # 1 라운드 선택 DWA-R (R1 K = 6)

# prereg_r2 §0 의 스위치 → 사본 코드 파라미터 이름 (README / CORE R2 READY 를 따른다)
F_PARAM = {
    'F1_on': 'reset_model',             # bool: 사슬 모형 기반 재설정
    'F1_theta': 'reset_model_theta',    # θ [m/s]
    'F2': 'goal_extension_dist',        # D [m]
    'F3': 'blocked_detour_shift',       # S [m]
}


def f1(theta):
    return {F_PARAM['F1_on']: True, F_PARAM['F1_theta']: theta}


def f2(d):
    return {F_PARAM['F2']: d}


def f3(s):
    return {F_PARAM['F3']: s}


DWA_Y_GRID = {
    'Y00': {},
    'Y01': f1(0.10),
    'Y02': f1(0.15),
    'Y03': f1(0.20),
    'Y04': f2(0.6),
    'Y05': f2(1.0),
    'Y06': f3(0.6),
    'Y07': f3(1.0),
    'Y08': {**f1(0.15), **f2(0.6)},
    'Y09': {**f1(0.15), **f2(1.0), **f3(0.6)},
    'Y10': {**f1(0.15), **f2(0.6), **f3(1.0)},
    'Y11': {**f1(0.10), **f2(1.0), **f3(1.0)},
}


def y_arm(yid):
    return f'dwa_y_{yid}'


def dwa_y_block(nav2, yid):
    b = px.dwa_x_block(nav2, BASE_X)
    if DWA_Y_GRID[yid]:
        b.setdefault('research', {}).update(copy.deepcopy(DWA_Y_GRID[yid]))
    return b


def extra_blocks(nav2=None, selected_y=None):
    nav2 = nav2 or px.load_nav2()
    out = {y_arm(y): dwa_y_block(nav2, y) for y in DWA_Y_GRID}
    if selected_y:
        out['dwa_R2'] = dwa_y_block(nav2, selected_y)
    return out


def ros_params_file(fname, arm_ids, selected_y=None, **kw):
    """params_x.ros_params_file + Y 격자 블록. dwa_R 는 X03 (selected_x 고정)."""
    nav2 = px.load_nav2()
    kw.setdefault('selected_x', BASE_X)
    return px.ros_params_file(fname, arm_ids, extra_blocks=extra_blocks(nav2, selected_y),
                              nav2=nav2, **kw)


def dwa_y_cfg(yid, nav2=None):
    lines = px.dwa_x_cfg(BASE_X, nav2)
    for k, v in DWA_Y_GRID[yid].items():
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        lines.append(f'{k} {v}')
    return lines


def write_cfgs(out, selected_y=None, selected_t=None):
    names = px.write_cfgs(out, BASE_X)       # dwa_R.cfg = X03
    nav2 = px.load_nav2()
    for y in DWA_Y_GRID:
        with open(os.path.join(out, f'{y_arm(y)}.cfg'), 'w', encoding='utf-8') as f:
            f.write('\n'.join(dwa_y_cfg(y, nav2)) + '\n')
    if selected_y:
        with open(os.path.join(out, 'dwa_R2.cfg'), 'w', encoding='utf-8') as f:
            f.write('\n'.join(dwa_y_cfg(selected_y, nav2)) + '\n')
    if selected_t:   # 교과서 튜닝 팔 (chain): dwa_textbook cfg + T 격자 값
        over = dict(px.DWA_ARMS['dwa_textbook'], **px.TB_GRID[selected_t])
        lines = px.to_cfg_lines(px.controller_block(nav2, 'DWA'), over)
        with open(os.path.join(out, 'dwa_textbook_tuned.cfg'), 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')
    return sorted(os.listdir(out))


if __name__ == '__main__':
    print(json.dumps(write_cfgs(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)))
