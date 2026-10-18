#!/usr/bin/env python3
"""
R3-C 완성 실험 제어기 설정 (prereg_r3c.md §2·§3). 1–3 라운드 params_x·params_r2·params_r3 를 그대로 쓰고
다음만 더한다 (기존 파일은 고치지 않는다).

- CC-DWA ≡ dwa_R3 = R3 J'' 선택 설정 (tuning_selection_r3.json 의 DWA_R3.chosen_z). 블록·cfg 는 R3 와 같은
  함수(params_r3.extra_blocks / dwa_z_cfg, Y11 이면 params_r2)로 만든다 → R3 의 dwa_R3 와 바이트 동일.
- §2 변형: 선택 설정에서 **표의 키만** 덮어쓴다 (기계적). 예외:
  - 선택에 G1(accel_hold) 이 없으면 V-hold (accel_hold true, reset_model false) 를 더한다.
  - 선택에 F1(reset_model) 과 G1 이 함께 있으면 V-G1only (reset_model false, accel_hold true) 를 더한다.
    (F1 만 있고 G1 이 없으면 V-G1only 는 V-hold 와 같은 설정이라 V-hold 하나만 둔다.)
  - 덮어쓴 결과가 선택 설정과 같으면(기본값을 채운 유효 설정 비교) 그 변형은 돌지 않고 "= CC-DWA".
- E4 계산 시간 감싸개: 팔 X 를 amr_nav_research::TimedController 로 감싼 팔 tm_X. 블록은 X 의 블록 그대로에
  plugin 만 바꾸고 timed_inner_plugin (원래 plugin)·timed_csv 를 더한다. DWA 계열은 참값 트랙 차단을 그대로 둔다.

    python3 params_r3c.py show [<Z>]          # 변형 표 (선택 Z 를 주지 않으면 커밋된 tuning_selection_r3.json)
    python3 params_r3c.py manifest <out_dir>  # 변형 블록·cfg 를 쓰고 sha256 표를 낸다 (실행 전 커밋용)
"""
import copy
import hashlib
import json
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import params_r2 as p2  # noqa: E402
import params_r3 as p3  # noqa: E402
import params_x as px  # noqa: E402

PR_DIR = os.path.abspath(os.path.join(HERE, '..'))
SEL_REL = 'docs/research/paper-research/tuning_selection_r3.json'
TIMED_PLUGIN = 'amr_nav_research::TimedController'

# prereg_r3c §2 표 그대로 (키 'research:<이름>' 은 research.*, 'top:<이름>' 은 배포 DWA 파라미터)
VARIANTS = [
    ('V-meas', 'dwa_v_meas', '측정 기준 창', {'top:window_center_measured': True}),
    ('V-thr', 'dwa_v_thr', '임계 재설정', {'research:accel_hold': False, 'research:reset_model': False}),
    ('V-mdl-n', 'dwa_v_mdl_n', '모형 (명목값)',
     {'research:accel_hold': False, 'research:reset_model': True, 'research:reset_model_theta': 0.10,
      'research:model_delay': 0.04, 'research:model_tau': 0.08}),
    ('V-mdl-i', 'dwa_v_mdl_i', '모형 (식별값)',
     {'research:accel_hold': False, 'research:reset_model': True, 'research:reset_model_theta': 0.10,
      'research:model_delay': p3.F1_FIT['model_delay'], 'research:model_tau': p3.F1_FIT['model_tau']}),
    ('V-K0', 'dwa_v_K0', '지속 조건 끔', {'research:reset_persist_cycles': 0}),
    ('V-noext', 'dwa_v_noext', '말단 정렬 끔', {'research:goal_extension_dist': 0.0}),
    ('V-nodet', 'dwa_v_nodet', '우회 끔', {'research:blocked_detour_shift': 0.0}),
]
V_HOLD = ('V-hold', 'dwa_v_hold', '유예 (모형 없음)', {'research:accel_hold': True, 'research:reset_model': False})
V_G1ONLY = ('V-G1only', 'dwa_v_G1only', '유예만 (모형 끔)',
            {'research:reset_model': False, 'research:accel_hold': True})

# 연구 스위치 기본값 (src/amr_nav_research/README.md 표 = dwa_controller_x.cpp 선언값)
RESEARCH_DEFAULTS = {
    'reset_persist_cycles': 0, 'reset_hard_v': 0.6, 'jerk_consistent': False, 'jerk_lim': 2.0,
    'pp_candidate': False, 'reset_model': False, 'reset_model_theta': 0.15, 'reset_model_theta_w': -1.0,
    'model_delay': 0.04, 'model_tau': 0.08, 'goal_extension_dist': 0.0, 'blocked_detour_shift': 0.0,
    'blocked_lookahead': 3.0, 'accel_hold': False,
}
TOP_DEFAULTS = {'window_center_measured': False}


def committed_selection(ws=None):
    """ws 에 커밋된 tuning_selection_r3.json (HEAD). 없으면 None."""
    ws = ws or os.path.abspath(os.path.join(PR_DIR, '..', '..', '..'))
    r = subprocess.run(['git', '-C', ws, 'show', f'HEAD:{SEL_REL}'], capture_output=True, text=True)
    if r.returncode != 0:
        return None
    return json.loads(r.stdout)


def chosen_z(sel=None):
    sel = sel or committed_selection()
    if sel is None:
        raise SystemExit('tuning_selection_r3.json 이 아직 커밋되지 않았다')
    return sel['DWA_R3']['chosen_z']


def sel_spec(zsel):
    """선택 설정의 (research, top) — dwa_z_block / dwa_z_cfg 와 같은 구성 (X03 의 K 6 위에 덮음)."""
    if zsel == 'Y11':
        return {'reset_persist_cycles': 6, **p3.Y11}, {}
    s = p3.build_z()[zsel]
    research = {'reset_persist_cycles': 6}
    research.update(copy.deepcopy(s['research']))
    return research, dict(s['top'])


def apply_over(research, top, over):
    r, t = dict(research), dict(top)
    for k, v in over.items():
        kind, name = k.split(':', 1)
        (r if kind == 'research' else t)[name] = v
    return r, t


def effective(research, top, nav2=None):
    nav2 = nav2 or px.load_nav2()
    base = px.controller_block(nav2, 'DWA')
    e = {f'research.{k}': v for k, v in dict(RESEARCH_DEFAULTS, **research).items()}
    tops = dict(TOP_DEFAULTS)
    for k in set(top) | set(TOP_DEFAULTS):
        tops[k] = top.get(k, base.get(k, TOP_DEFAULTS.get(k)))
    e.update({f'top.{k}': v for k, v in tops.items()})
    return e


def variant_list(zsel, nav2=None):
    """[(vid, arm, 행 이름, 덮어쓰기, 상태 'run' | '= CC-DWA')] — prereg_r3c §2 의 기계적 규칙."""
    nav2 = nav2 or px.load_nav2()
    research, top = sel_spec(zsel)
    g1 = bool(research.get('accel_hold', False))
    f1 = bool(research.get('reset_model', False))
    vs = list(VARIANTS)
    if not g1:
        vs.append(V_HOLD)
    elif f1:
        vs.append(V_G1ONLY)
    e_sel = effective(research, top, nav2)
    out = []
    for vid, arm, row, over in vs:
        r, t = apply_over(research, top, over)
        same = effective(r, t, nav2) == e_sel
        out.append((vid, arm, row, over, '= CC-DWA' if same else 'run'))
    return out


def run_arms(zsel):
    return [arm for vid, arm, row, over, st in variant_list(zsel) if st == 'run']


def selection_block(nav2, zsel):
    """R3 와 같은 dwa_R3 블록 (params_r3.extra_blocks)."""
    return copy.deepcopy(p3.extra_blocks(nav2, zsel)['dwa_R3'])


def variant_blocks(nav2, zsel):
    base = selection_block(nav2, zsel)
    out = {}
    for vid, arm, row, over, st in variant_list(zsel, nav2):
        b = copy.deepcopy(base)
        for k, v in over.items():
            kind, name = k.split(':', 1)
            if kind == 'research':
                b.setdefault('research', {})[name] = v
            else:
                b[name] = v
        out[arm] = b
    return out


def timed_block(block, csv_path):
    b = copy.deepcopy(block)
    b['timed_inner_plugin'] = b['plugin']
    b['plugin'] = TIMED_PLUGIN
    b['timed_csv'] = csv_path
    if b['timed_inner_plugin'] in ('amr_navigation::DWAController', px.X_PLUGIN):
        b['tracked_obstacles_topic'] = px.UNUSED_TRACKS     # 감싸도 참값 트랙 차단은 같다
    return b


def all_blocks(nav2, zsel):
    """ros_params_file 가 아는 블록 전부 (R3 와 같은 것) + 변형."""
    blocks = dict(px.ours_arm_blocks(nav2))
    blocks.update(px.nav2_arm_blocks())
    blocks.update(p3.extra_blocks(nav2, zsel))
    blocks.update(variant_blocks(nav2, zsel))
    return blocks


def ros_params_file(fname, arm_ids, zsel, timed=None, **kw):
    """
    params_r3.ros_params_file 과 같은 파일 + 변형 블록 + 감싼 팔.
    timed: {'tm_<arm>': (<arm>, <csv 경로>)} — 감싼 팔 이름 → (안쪽 팔, CSV).
    """
    nav2 = px.load_nav2()
    extra = p3.extra_blocks(nav2, zsel)
    extra.update(variant_blocks(nav2, zsel))
    if timed:
        base = all_blocks(nav2, zsel)
        base['dwa_textbook_tuned'] = _textbook_tuned(nav2)
        for tarm, (inner, csv) in timed.items():
            extra[tarm] = timed_block(base[inner], csv)
    kw.setdefault('selected_x', p2.BASE_X)
    kw.setdefault('selected_t', 'T06')
    return px.ros_params_file(fname, arm_ids, extra_blocks=extra, nav2=nav2, **kw)


def _textbook_tuned(nav2):
    tb = dict(px.controller_block(nav2, 'DWA'))
    tb.update(px.DWA_ARMS['dwa_textbook'])
    return dict(tb, **px.TB_GRID['T06'])


# ---------------------------------------------------------------------------------------------
# chain_sim_x cfg

def _fmt(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    return str(v)


def cfg_lines(research, top, nav2=None):
    nav2 = nav2 or px.load_nav2()
    lines = px.to_cfg_lines(px.controller_block(nav2, 'DWA'), dict(top))
    for k, v in research.items():
        lines.append(f'{k} {_fmt(v)}')
    return lines


def selection_cfg(zsel, nav2=None):
    """R3 chain_inner 와 같은 dwa_R3.cfg (Y11 이면 dwa_R2.cfg 사본)."""
    nav2 = nav2 or px.load_nav2()
    if zsel == 'Y11':
        return p2.dwa_y_cfg('Y11', nav2)
    return p3.dwa_z_cfg(zsel, nav2)


def write_cfgs(out, zsel):
    os.makedirs(out, exist_ok=True)
    nav2 = px.load_nav2()
    research, top = sel_spec(zsel)
    sel_lines = selection_cfg(zsel, nav2)
    if zsel != 'Y11':
        assert sel_lines == cfg_lines(research, top, nav2), 'dwa_R3 cfg 구성이 R3 와 다르다'
    files = {'dwa_R3': sel_lines}
    for vid, arm, row, over, st in variant_list(zsel, nav2):
        r, t = apply_over(research, top, over)
        files[arm] = cfg_lines(r, t, nav2)
    for arm, lines in files.items():
        with open(os.path.join(out, f'{arm}.cfg'), 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')
    return sorted(files)


def sha(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def manifest(out, zsel):
    """변형 ROS 블록(YAML)·chain cfg 를 쓰고 sha256 표(markdown)를 돌려준다."""
    os.makedirs(out, exist_ok=True)
    nav2 = px.load_nav2()
    blocks = {'dwa_R3': selection_block(nav2, zsel)}
    blocks.update(variant_blocks(nav2, zsel))
    cfg_dir = os.path.join(out, 'cfg')
    write_cfgs(cfg_dir, zsel)
    rows = []
    for vid, arm, row, over, st in [('CC-DWA', 'dwa_R3', 'CC-DWA', {}, 'run')] + variant_list(zsel, nav2):
        yf = os.path.join(out, f'{arm}.yaml')
        with open(yf, 'w', encoding='utf-8') as f:
            yaml.safe_dump({arm: blocks[arm]}, f, sort_keys=True, allow_unicode=True)
        rows.append({'id': vid, 'arm': arm, 'row': row, 'over': over, 'status': st,
                     'yaml_sha256': sha(yf), 'cfg_sha256': sha(os.path.join(cfg_dir, f'{arm}.cfg'))})
    return rows


def show(zsel):
    research, top = sel_spec(zsel)
    print(f'선택 Z = {zsel}: research {json.dumps(research)} top {json.dumps(top)}')
    for vid, arm, row, over, st in variant_list(zsel):
        print(f'{vid:9s} {arm:14s} {st:9s} {row} {json.dumps(over, ensure_ascii=False)}')


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'show'
    if cmd == 'show':
        show(sys.argv[2] if len(sys.argv) > 2 else chosen_z())
    elif cmd == 'manifest':
        z = sys.argv[3] if len(sys.argv) > 3 else chosen_z()
        print(json.dumps(manifest(sys.argv[2], z), indent=1, ensure_ascii=False))
    else:
        raise SystemExit(__doc__)
