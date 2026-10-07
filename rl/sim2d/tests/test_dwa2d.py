"""
2D-DWA (B5) 시험 — 바인딩 amr_dwa2d_py 가 팀 DWA 코어를 배포 설정 그대로 부르는가, 2D 사슬에 맞게 붙었는가.

1. 설정: `dwa2d.PARAMS` 가 ROS 래퍼 dwa_controller.cpp configure() 의 param() 키·기본값과 같다 (정규식 대조),
   YAML → DwaConfig 왕복이 값을 잃지 않는다.
2. 골든: 같은 격자·경로·상태에서 바인딩 `compute_raw` 의 한 걸음 출력 = 바인딩을 거치지 않고 코어를 직접 부른
   독립 실행 파일(dwa_golden_main.cpp, 설정을 C++ 에 손으로 옮김)의 출력 (비트 단위, %.17g). 저장된 golden JSON 과도 대조.
   래퍼 경로(`Dwa2D.step`: 경로 창·직전 명령 기억)의 첫 걸음 = 같은 창을 넣은 compute_raw.
3. 지역 코스트맵: 마킹·레이트레이스 지우기·rolling 원점 이동·팽창·게이트 중앙값.
4. 2D 사슬: step_rl = Sim2DEnv.step (비트), step_dwa 의 두 번째 틱이 서브스텝 3 뒤 상태를 받는다,
   직선·회전·막힘 사례의 폐루프 행동.
"""

import json
import math
import os
import re
import subprocess

import numpy as np
import pytest
import torch

from amr_sim2d import dwa2d
from amr_sim2d._contract import SIM2D_DIR

HERE = os.path.dirname(os.path.abspath(__file__))
GOLDEN = os.path.join(HERE, 'golden_dwa2d.json')
MAIN = os.path.join(SIM2D_DIR, 'dwa_binding', 'build', 'dwa_golden_main')
DEV = 'cuda' if torch.cuda.is_available() else 'cpu'
FP = np.array([[0.30, 0.20], [0.30, -0.20], [-0.30, -0.20], [-0.30, 0.20]])


@pytest.fixture(scope='module')
def m():
    mod = dwa2d.module()
    if not os.path.exists(MAIN):
        subprocess.run(['bash', os.path.join(SIM2D_DIR, 'dwa_binding', 'build_dwa2d.sh')],
                       check=True)
    return mod


@pytest.fixture(scope='module')
def conf():
    return dwa2d.load_config()


# ------------------------------------------------------------------ 1. 설정
def _wrapper_params():
    src = open(dwa2d.WRAPPER_CPP, encoding='utf-8').read()
    src = re.sub(r'\s+', ' ', src)
    pat = re.compile(r'(?:c\.|L\.|W\.)([\w.]+) = param\( ?node, p \+ "(\w+)", ([^;]+?)\);')
    out = {}
    for f, key, default in pat.findall(src):
        out[key] = (f, default.strip())
    return out


def test_params_table_matches_wrapper():
    wp = _wrapper_params()
    table = {key: (field, d, lim) for field, key, d, lim in dwa2d.PARAMS}
    prefix = {'max_vel_x': 'limits.', 'min_vel_x': 'limits.', 'max_vel_theta': 'limits.',
              'acc_lim_x': 'limits.', 'decel_lim_x': 'limits.', 'acc_lim_theta': 'limits.',
              'jerk_lim_x': 'limits.'}
    assert len(wp) >= 90, f'정규식이 래퍼 param() 를 거의 못 찾았다: {len(wp)}'
    missing = sorted(set(wp) - set(table))
    assert not missing, f'래퍼에는 있고 표에는 없는 키: {missing}'
    for key, (f, d) in wp.items():
        field, default, lim = table[key]
        want = prefix.get(key, '') + f
        if key.endswith('_weight') or key in ('heading_weight',):
            want = 'weights.' + f
        assert field == want, (key, field, want)
        if d.startswith('param(') or d.startswith('std::min') or d == 'a_lin':
            assert lim is not None, f'{key}: 래퍼 기본값이 limits.* 인데 표에 limits 키가 없다'
            m_ = re.search(r'"limits\.(\w+)", ([\d.]+)', d)
            if m_:
                assert lim == m_.group(1), (key, lim, m_.group(1))
            continue
        lit = {'true': True, 'false': False}.get(d, None)
        if lit is None:
            lit = float(d)
        assert default == lit, (key, default, lit)


def test_config_values_and_roundtrip(m, conf):
    cfg = conf['cfg']
    assert cfg['control_period'] == pytest.approx(0.05)        # controller_frequency 20
    assert cfg['vx_samples'] == 11 and cfg['vth_samples'] == 31
    assert cfg['weights.oscillation'] == 0.5                      # 래퍼 기본값과 struct 기본값이 다른 키
    assert cfg['goal_align_distance'] == 0.08
    assert cfg['off_path_band'] == 1.0                            # YAML 값 (래퍼 기본 0.05 아님)
    assert cfg['recenter_step'] == 0.025
    assert cfg['robot_radius'] == pytest.approx(math.hypot(0.3, 0.2))
    assert cfg['gate.enable'] is False and cfg['yield_decision'] is False
    assert cfg['limits.max_vel_theta'] == 1.5 and cfg['limits.jerk_lim_x'] == 2.0
    rt = m.config_roundtrip(cfg)
    for k, v in cfg.items():
        assert float(rt[k]) == float(v), k
    assert conf['wrapper']['path_horizon'] == 4.0
    cm = conf['costmap']
    assert (cm['size_m'], cm['resolution'], cm['inflation_radius'], cm['cost_scaling_factor'],
            cm['obstacle_max_range'], cm['raytrace_max_range']) == (8.0, 0.025, 0.8, 3.0, 5.5, 6.0)
    assert cm['inscribed_radius'] == pytest.approx(0.20)
    for k, s in conf['sources'].items():
        assert s, k


# ------------------------------------------------------------------ 2. 골든
def _grid_case(kind):
    """합성 지역 코스트맵 (0.025 m, 원점 (−4, −4), 8 × 8 m) + 경로 + 상태."""
    res, n = 0.025, 320
    cells = np.zeros((n, n), dtype=np.uint8)
    ox = oy = -4.0

    def wall(x0, x1, y0, y1):
        ix0, ix1 = int((x0 - ox) / res), int((x1 - ox) / res)
        iy0, iy1 = int((y0 - oy) / res), int((y1 - oy) / res)
        cells[max(iy0, 0):iy1 + 1, max(ix0, 0):ix1 + 1] = 254
    s = np.arange(0, 3.5 + 1e-9, 0.05)
    if kind == 'straight':
        wall(-4, 4, 1.0, 1.05)
        wall(-4, 4, -1.05, -1.0)
        path = np.stack([s, 0 * s], 1)
        state = (0.0, 0.0, 0.0, 0.3, 0.0, 1, 0.3, 0.0, 0)
    elif kind == 'turn':
        wall(-4, 4, 2.0, 2.05)
        path = np.stack([-s * 0.7, s * 0.7], 1)          # 경로가 뒤쪽 (135°)
        state = (0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0)
    elif kind == 'blocked':
        wall(-4, 4, 1.0, 1.05)
        wall(-4, 4, -1.05, -1.0)
        wall(0.9, 1.0, -1.0, 1.0)                           # 경로를 막는 벽 (앞 0.6 m)
        path = np.stack([s, 0 * s], 1)
        state = (0.0, 0.0, 0.0, 0.5, 0.0, 1, 0.5, 0.0, 0)
    elif kind == 'goal':
        wall(-4, 4, 1.0, 1.05)
        g = s[s <= 0.6]
        path = np.stack([g, 0 * g], 1)                       # 목표 0.06 m 앞 → 정렬 구간
        state = (0.54, 0.02, 0.4, 0.02, 0.1, 1, 0.02, 0.1, 1)
    elif kind == 'narrow':
        wall(-4, 4, 0.30, 0.35)                              # 0.60 m 통로, 경로가 2 cm 비낌
        wall(-4, 4, -0.35, -0.30)
        path = np.stack([s, 0 * s + 0.02], 1)
        state = (0.0, 0.0, 0.05, 0.6, 0.05, 1, 0.6, 0.05, 0)
    else:
        raise ValueError(kind)
    return cells, res, ox, oy, dwa2d.path_poses(path), state


def _inflate(m, cells, res, conf):
    cm = conf['costmap']
    return m.inflate(cells, res, cm['inscribed_radius'], cm['inflation_radius'],
                     cm['cost_scaling_factor'])


def _run_main(cells, res, ox, oy, circ, poses, state):
    lines = [f'grid {cells.shape[1]} {cells.shape[0]} {res!r} {ox!r} {oy!r}',
             ' '.join(map(str, cells.ravel().tolist())), f'circ {circ}',
             'fp 4 ' + ' '.join(f'{v!r}' for v in FP.ravel()), 'allow_unknown 1',
             'state ' + ' '.join(repr(float(v)) if i not in (5, 8) else str(int(v))
                                 for i, v in enumerate(state)),
             f'path {len(poses)}'] + [' '.join(repr(float(v)) for v in p) for p in poses]
    out = subprocess.run([MAIN], input='\n'.join(lines) + '\n', capture_output=True, text=True,
                         check=True).stdout.split()
    keys = ('v', 'w', 'found', 'n_samples', 'n_valid', 'n_collision', 'n_recentered',
            'best_cost', 'd_goal')
    return dict(zip(keys, out))


def _fmt(d):
    keys = ('v', 'w', 'found', 'n_samples', 'n_valid', 'n_collision', 'n_recentered',
            'best_cost', 'd_goal')
    out = {}
    for k in keys:
        v = d[k]
        out[k] = str(int(v)) if isinstance(v, (bool, int, np.integer)) else '%.17g' % v
    return out


CASES = ('straight', 'turn', 'blocked', 'goal', 'narrow')


@pytest.mark.parametrize('kind', CASES)
def test_golden_binding_equals_direct_core(m, conf, kind):
    cells, res, ox, oy, poses, st = _grid_case(kind)
    g = _inflate(m, cells, res, conf)
    circ = m.inflation_cost(math.hypot(0.3, 0.2), conf['costmap']['inscribed_radius'],
                            conf['costmap']['cost_scaling_factor'])
    assert circ == 155
    direct = _run_main(g, res, ox, oy, circ, poses, st)
    x, y, th, v, w, hl, vl, wl, eg = st
    b = m.compute_raw(conf['cfg'], g, res, ox, oy, FP, circ, True, x, y, th, v, w, bool(hl),
                      vl, wl, poses, bool(eg))
    got = _fmt(b)
    py = {k: '%.17g' % float(v) if k in ('v', 'w', 'best_cost', 'd_goal') else str(int(v))
          for k, v in direct.items()}
    assert got == py, f'{kind}: 바인딩 {got} ≠ 직접 {py}'
    gold = json.load(open(GOLDEN))
    assert gold[kind] == got, f'{kind}: 저장된 golden 과 다르다 (코어가 바뀌었나?)'


def test_golden_cases_behave(m, conf):
    gold = json.load(open(GOLDEN))
    assert float(gold['straight']['v']) > 0.3 and abs(float(gold['straight']['w'])) < 1e-9
    assert float(gold['turn']['v']) < 0.05 and float(gold['turn']['w']) > 0.0   # 왼쪽 뒤로 돈다
    assert float(gold['blocked']['v']) < 0.5                                   # 막힘 → 감속
    assert float(gold['goal']['v']) <= 0.02 + 1e-9                             # 정렬 구간: 전진 안 함


def test_wrapper_first_step_equals_raw(m, conf):
    cells, res, ox, oy, poses, _ = _grid_case('straight')
    g = _inflate(m, cells, res, conf)
    d = m.Dwa2D(conf['cfg'], conf['costmap'], conf['footprint'], 4.0, 10, True)
    long_path = dwa2d.path_poses(np.stack([np.arange(0, 9, 0.05), np.zeros(180)], 1))
    d.reset(long_path)
    d.set_master_grid(g, res, ox, oy)
    o = d.step(0.31, 0.0, 0.0, 0.0, 0.0)
    # PlanWindow: 최근접 정점 6 (x 0.30) → 창 = 정점 5 부터 누적 길이가 4 m 를 넘는 정점까지 (C++ 와 같은 순서의 합)
    s = 5
    acc, j = 0.0, s
    while j + 1 < len(long_path):
        j += 1
        acc += math.hypot(long_path[j, 0] - long_path[j - 1, 0],
                          long_path[j, 1] - long_path[j - 1, 1])
        if acc > 4.0:
            break
    win = long_path[s:j + 1]
    b = m.compute_raw(conf['cfg'], g, res, ox, oy, FP, d.circumscribed_cost(), True, 0.31, 0.0,
                      0.0, 0.0, 0.0, False, 0.0, 0.0, win, False)
    assert o['window_len'] == len(win)
    assert (o['v'], o['w'], o['found'], o['n_valid']) == (b['v'], b['w'], b['found'],
                                                          b['n_valid'])
    # 두 번째 걸음은 직전 명령을 창 중심으로 쓴다 (has_last)
    o2 = d.step(0.31, 0.0, 0.0, 0.0, 0.0)
    b2 = m.compute_raw(conf['cfg'], g, res, ox, oy, FP, d.circumscribed_cost(), True, 0.31, 0.0,
                       0.0, 0.0, 0.0, True, o['v'], o['w'], win, False)
    assert (o2['v'], o2['w']) == (b2['v'], b2['w'])


def test_no_valid_patience_exception(m, conf):
    """사방이 막혀 유효 후보가 없으면 10 번째 걸음마다 예외(0 명령), 그 사이는 제동 후보."""
    res, n = 0.025, 320
    cells = np.zeros((n, n), dtype=np.uint8)
    cells[150:170, 150:170] = 254                              # 로봇 바로 위 장애물 (원점 둘레)
    g = _inflate(m, cells, res, conf)
    d = m.Dwa2D(conf['cfg'], conf['costmap'], conf['footprint'], 4.0, 10, True)
    d.reset(dwa2d.path_poses(np.stack([np.arange(0, 3, 0.05), np.zeros(60)], 1)))
    d.set_master_grid(g, res, -4.0, -4.0)
    # 측정 속도 v 0.04 (< 0.05 → 예외 조건), ω 1.4 → 제동 후보의 ω 는 0 이 아니다 (창 중심이 측정·직전 명령을 오간다)
    outs = [d.step(0.0, 0.0, 0.0, 0.04, 1.4) for _ in range(20)]
    assert not any(o['found'] for o in outs)
    exc = [i for i, o in enumerate(outs) if o['exception']]
    assert exc == [9, 19]
    assert all(outs[i]['v'] == 0.0 and outs[i]['w'] == 0.0 for i in exc)
    assert all(abs(outs[i]['w']) > 0.0 for i in range(20) if i not in exc)   # 그 사이는 제동 후보


# ------------------------------------------------------------------ 3. 지역 코스트맵
def _wall_scan(y_wall=1.5):
    ang = -math.pi + np.arange(720) * math.radians(0.5)
    with np.errstate(divide='ignore'):
        r = np.where(np.sin(ang) > 1e-3, y_wall / np.sin(ang), np.inf)
    r[r > 25.0] = np.inf
    return r.astype(np.float32), ang


def test_costmap_marks_clears_and_rolls(m, conf):
    cm = dict(conf['costmap'], denoise=False)
    d = m.Dwa2D(conf['cfg'], cm, conf['footprint'], 4.0, 10, True)
    d.reset(dwa2d.path_poses(np.stack([np.arange(0, 3, 0.05), np.zeros(60)], 1)))
    r, ang = _wall_scan(1.5)
    d.update_costmap(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, r, -math.pi, math.radians(0.5), 0.1, 25.0)
    ob = d.obstacle_grid()
    ox, oy = d.costmap_origin()
    assert (ox, oy) == (-4.0, -4.0)
    ys, xs = np.nonzero(ob == 254)
    wy = oy + (ys + 0.5) * 0.025
    wx = ox + (xs + 0.5) * 0.025
    assert np.all(np.abs(wy - 1.5) <= 0.025)                   # 벽 셀만
    assert np.all(np.hypot(wx, wy - 0.0) < 5.5 + 0.05)         # 마킹 5.5 m 안
    master = d.master_grid()
    assert np.all(master[ob == 254] == 254)
    # 팽창: 벽에서 0.5 m 아래 점의 비용은 Nav2 식 (내접 0.2, s 3.0) 의 0.5 ± 0.04 m 값 사이
    iy = int((1.0 - oy) / 0.025)
    ix = int((0.0 - ox) / 0.025)
    hi = m.inflation_cost(0.46, 0.2, 3.0)
    lo = m.inflation_cost(0.54, 0.2, 3.0)
    assert lo <= master[iy, ix] <= hi, (lo, master[iy, ix], hi)
    iy2 = int((1.4 - oy) / 0.025)
    assert master[iy2, ix] == 253                               # 내접 반경 안
    # 로봇이 1.0 m 이동 → 원점이 셀 단위로 옮겨지고 표시는 세계 좌표에 남는다
    d.update_costmap(1.0, 0.0, 0.0, 1.0, 0.0, 0.0, np.full(720, np.inf, dtype=np.float32),
                     -math.pi, math.radians(0.5), 0.1, 25.0)
    ox2, oy2 = d.costmap_origin()
    assert ox2 == pytest.approx(-3.0) and oy2 == pytest.approx(-4.0)
    ob2 = d.obstacle_grid()
    ys2, xs2 = np.nonzero(ob2 == 254)
    wx2 = ox2 + (xs2 + 0.5) * 0.025
    assert len(xs2) > 0 and set(np.round(wx2, 4)) <= set(np.round(wx, 4))
    # 벽이 사라진 스캔(벽 너머 3 m) → 지나가는 광선이 옛 표시를 지운다
    r3, _ = _wall_scan(3.0)
    d.update_costmap(1.15, 0.0, 0.0, 1.0, 0.0, 0.0, r3, -math.pi, math.radians(0.5), 0.1, 25.0)
    ob3 = d.obstacle_grid()
    ys3, _ = np.nonzero(ob3 == 254)
    wy3 = oy2 + (ys3 + 0.5) * 0.025
    assert np.sum(np.abs(wy3 - 1.5) < 0.03) < 0.1 * len(xs2)   # 대부분 지워짐
    assert np.any(np.abs(wy3 - 3.0) < 0.03)


def test_costmap_denoise_matches_core(m, conf):
    rng = np.random.default_rng(0)
    r, _ = _wall_scan(1.5)
    rn = np.where(np.isfinite(r), r + rng.normal(0, 0.03, r.shape), r).astype(np.float32)
    den = np.asarray(m.denoise_ranges(rn, 5, 0.15, 3))
    a = m.Dwa2D(conf['cfg'], conf['costmap'], conf['footprint'], 4.0, 10, True)
    b = m.Dwa2D(conf['cfg'], dict(conf['costmap'], denoise=False), conf['footprint'], 4.0, 10,
                True)
    for d_, rr in ((a, rn), (b, den)):
        d_.reset(dwa2d.path_poses(np.stack([np.arange(0, 3, 0.05), np.zeros(60)], 1)))
        d_.update_costmap(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, rr, -math.pi, math.radians(0.5), 0.1,
                          25.0)
    assert np.array_equal(a.master_grid(), b.master_grid())
    assert not np.array_equal(den, rn)


# ------------------------------------------------------------------ 4. 2D 사슬
def _world(tmp_path, block=False, res=0.05):
    """폭 3 m 통로 (x 0–12, y ±1.5) 합성 세계 → map_server yaml."""
    W, H = int(14 / res), int(6 / res)
    occ = np.zeros((H, W), dtype=bool)                          # 행 0 = y 아래, 원점 (−1, −3)

    def rect(x0, x1, y0, y1):
        ix0, ix1 = int((x0 + 1) / res), int((x1 + 1) / res)
        iy0, iy1 = int((y0 + 3) / res), int((y1 + 3) / res)
        occ[iy0:iy1, ix0:ix1] = True
    rect(-1, 13, -3, -1.5)
    rect(-1, 13, 1.5, 3)
    rect(-1, 0, -3, 3)
    rect(12, 13, -3, 3)
    if block:
        rect(4.5, 4.6, -1.5, 1.5)
    img = np.where(occ, 0, 254).astype(np.uint8)[::-1]
    pgm = tmp_path / 'w.pgm'
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H))
        f.write(img.tobytes())
    yml = tmp_path / 'w.yaml'
    yml.write_text(f'image: w.pgm\nmode: trinary\nresolution: {res}\norigin: [-1.0, -3.0, 0]\n'
                   'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n')
    return str(yml)


def _fixed_env(tmp_path, paths, yaws, block=False, seed=3, compile_=False):
    from amr_sim2d import crosseval as CE
    from amr_sim2d.env import EnvConfig
    os.environ['SIM2D_CACHE'] = str(tmp_path / 'cache')
    import amr_sim2d.paths as P
    P.CACHE_DIR = str(tmp_path / 'cache')
    cfg = EnvConfig(num_envs=len(paths), device=DEV, seed=seed, arm='B',
                    map_yaml=_world(tmp_path, block), n_paths=2, path_len=(3.0, 30.0),
                    obstacle_mode='none', obstacle_slots=0, dr=False, safety=True,
                    compile=compile_)
    return CE.FixedEpisodeEnv(cfg, paths, yaws, log=lambda *a: None)


def _line(x0, x1, y=0.0):
    s = np.arange(x0, x1 + 1e-9, 0.05)
    return np.stack([s, np.full_like(s, y)], 1)


@pytest.mark.skipif(DEV != 'cuda', reason='env 시험은 GPU 에서')
def test_step_rl_equals_base_step(tmp_path):
    from amr_sim2d.env import Sim2DEnv
    paths = [_line(1, 9), _line(2, 8, 0.5), _line(1.5, 10.5, -0.4)]
    yaws = [0.3, -0.4, 2.5]
    a = _fixed_env(tmp_path, paths, yaws)
    b = _fixed_env(tmp_path, paths, yaws)
    oa, ob = a.reset(), b.reset()
    assert torch.equal(oa, ob)
    g = torch.Generator(device=DEV)
    g.manual_seed(1)
    for k in range(120):
        u = torch.rand(3, 2, device=DEV, generator=g) * 2 - 1
        ra = a.step_rl(u)
        rb = Sim2DEnv.step(b, u)
        assert torch.equal(ra[0], rb[0]), k
        assert torch.equal(ra[1], rb[1]), k
        for name in ('pose', 'v', 'w', 'v_cmd', 'w_cmd', 'prev_actions', 'ranges', 'dev_peak'):
            assert torch.equal(getattr(a, name), getattr(b, name)), (k, name)
    assert torch.equal(a.gen.get_state(), b.gen.get_state())


class _StubDwa:
    """step_dwa 시간 순서 시험용: 받은 상태를 기록하고 정해진 명령을 낸다."""

    def __init__(self, cmds):
        self.cmds = list(cmds)
        self.seen = []

    def step(self, idx, pose, v, w):
        self.seen.append((pose.copy(), v.copy(), w.copy()))
        c = self.cmds[len(self.seen) - 1]
        n = len(idx)
        return {'v': np.full(n, c[0]), 'w': np.full(n, c[1]), 'found': np.ones(n, bool),
                'exception': np.zeros(n, bool), 'n_valid': np.ones(n, np.int64)}


@pytest.mark.skipif(DEV != 'cuda', reason='env 시험은 GPU 에서')
def test_step_dwa_ticks_twice_with_substep3_state(tmp_path):
    from amr_sim2d import kernels as K
    paths = [_line(1, 9)]
    a = _fixed_env(tmp_path, paths, [0.0])
    b = _fixed_env(tmp_path, paths, [0.0])
    a.reset()
    b.reset()
    stub = _StubDwa([(0.4, 0.2), (0.7, -0.1)] * 2)
    a.step_dwa(stub, np.ones(1, bool))
    assert len(stub.seen) == 2
    # 같은 계산을 커널로 직접: 서브스텝 0–2 는 명령 A, 3–4 는 명령 B
    S = {k: getattr(b, k) for k in b._S_KEYS}
    p = b.prof
    S.update({'lin_v': p.lin_v, 'lin_a': p.lin_a, 'ang_v': p.ang_v, 'ang_a': p.ang_a,
              'w_ref': p.w_ref, 'curv': p.curv_mode,
              'collided': torch.zeros(1, dtype=torch.bool, device=DEV),
              'coll_moving': torch.zeros(1, dtype=torch.bool, device=DEV),
              'interv': torch.zeros(1, dtype=b.fd, device=DEV)})
    mt = K.min_ttc(b.bstate())
    for k in range(5):
        c = (0.4, 0.2) if k < 3 else (0.7, -0.1)
        vc = torch.full((1,), c[0], dtype=b.fd, device=DEV)
        wc = torch.full((1,), c[1], dtype=b.fd, device=DEV)
        X = {'v_cmd': vc, 'w_cmd': wc, 'v1': vc, 'w1': wc, 'scan_px': b.scan_px,
             'scan_py': b.scan_py, 'scan_ok': b.scan_ok, 'min_ttc': mt, 'tm': b.tm, 'td': b.td,
             'acc_scale': b.acc_scale, 't': b.t, 'obst': b._obst_params(), 'grid': b.grid,
             'map_meta': b.map_meta, 'map_idx': b.map_idx}
        if k == 3:
            np.testing.assert_array_equal(stub.seen[1][0], S['pose'].cpu().numpy())
            np.testing.assert_array_equal(stub.seen[1][1], S['v'].cpu().numpy())
        S = K.substep(S, X, b._kf[k], b._first[k], True)
    assert torch.allclose(a.pose, S['pose'], atol=1e-12, rtol=0)
    # 프로파일러·플랜트 명령 버퍼는 서브스텝 3–4 의 목표(명령 B)를 바로 반영한다 (자세는 지연 때문에 둔하다)
    assert torch.equal(a.prof.lin_v, S['lin_v']) and torch.equal(a.prof.w_ref, S['w_ref'])
    assert torch.equal(a.dbuf, S['dbuf'])
    assert float(a.v_cmd[0]) == 0.7 and float(a.w_cmd[0]) == -0.1


def _run_dwa(env, n_steps=400):
    from amr_sim2d import crosseval as CE
    from amr_sim2d import dwa2d as D
    dw = D.DwaBatch(env.N, threads=2)
    for i, p in enumerate(env.fixed_paths):
        dw.reset(i, p)
    env.reset()
    sp = CE.sensor_poses(env.pose)
    dw.update_costmap(np.arange(env.N), sp.cpu().numpy(), env.pose.cpu().numpy(),
                      env.ranges.cpu().numpy())
    active = np.ones(env.N, bool)
    res = {}
    vmax = np.zeros(env.N)
    for _ in range(n_steps):
        _o, _r, _te, _tr, info = env.step_dwa(dw, active)
        vmax = np.maximum(vmax, env.v.abs().cpu().numpy())
        done = info['done'].cpu().numpy() & active
        for i in np.nonzero(done)[0]:
            keys = ('success', 'collision', 'stalled', 'timeout')
            res[i] = {k: bool(info[k][i]) for k in keys}
            res[i]['t'] = float(env.t[i])
            res[i]['max_e'] = float(env.ep_maxe[i])
        active &= ~done
        live = np.nonzero(active)[0]
        if not len(live):
            break
        sp = CE.sensor_poses(env.pose)
        li = torch.as_tensor(live, device=DEV)
        dw.update_costmap(live, sp[li].cpu().numpy(), env.pose[li].cpu().numpy(),
                          env.ranges[li].cpu().numpy())
    return res, vmax, env


@pytest.mark.skipif(DEV != 'cuda', reason='env 시험은 GPU 에서')
def test_dwa_closed_loop_straight_and_turn(tmp_path):
    paths = [_line(1, 9), _line(1, 9)]
    env = _fixed_env(tmp_path, paths, [0.0, math.pi])        # 둘째는 경로 반대 방향으로 시작
    res, vmax, env = _run_dwa(env)
    assert res[0]['success'] and res[1]['success'], res
    assert 0.95 <= vmax[0] <= 1.0 + 1e-6                     # 운용 상한 1.0 까지 가속
    assert res[0]['max_e'] < 0.05                             # 직선 추종
    assert 8.5 < res[0]['t'] < 12.0                           # 8 m, 가감속 포함
    assert res[1]['t'] > res[0]['t'] + 1.0                    # 제자리 회전에 시간이 든다


@pytest.mark.skipif(DEV != 'cuda', reason='env 시험은 GPU 에서')
def test_dwa_closed_loop_blocked_stops_without_contact(tmp_path):
    paths = [_line(1, 9)]
    env = _fixed_env(tmp_path, paths, [0.0], block=True)
    res, _vmax, env = _run_dwa(env, n_steps=300)
    assert 0 in res and not res[0]['collision'] and not res[0]['success'], res
    assert res[0]['stalled']                                  # 막힌 앞에서 서서 정체로 끝
    x = float(env.pose[0, 0])
    assert 3.4 < x < 4.5 - 0.30, x                            # 벽(4.5) 앞, 앞 끝이 벽에 닿지 않음


@pytest.mark.skipif(DEV != 'cuda', reason='env 시험은 GPU 에서')
def test_fixed_env_keeps_first_episode_only(tmp_path):
    """끝난 env 는 리셋하지 않는다 — 경로·자세가 그대로이고 난수 생성기를 더 쓰지 않는다 (두 팔 잡음열 정렬)."""
    paths = [_line(1, 9), _line(2, 8, 0.5)]
    env = _fixed_env(tmp_path, paths, [0.0, 0.0])
    env.reset()
    env.t_limit[0] = 0.05                                     # env 0 은 첫 스텝에 시간 상한
    pts0 = env.path_pts.clone()
    u = torch.zeros(2, 2, device=DEV)
    _o, _r, _te, tr, info = env.step_rl(u)
    assert bool(info['done'][0]) and not bool(info['done'][1])
    pose0 = env.pose[0].clone()
    g0 = env.gen.get_state()
    _o, _r, _te, tr, info = env.step_rl(u)
    assert torch.equal(env.path_pts, pts0)
    assert torch.allclose(env.pose[0], pose0)                 # 시작점으로 되돌아가지 않았다
    env2 = _fixed_env(tmp_path, paths, [0.0, 0.0])
    env2.reset()
    env2.step_rl(u)
    g1 = env2.gen.get_state()
    assert torch.equal(g0, g1)                                # 리셋 난수를 더 뽑지 않았다


def test_costmap_marking_range(m, conf):
    """마킹은 센서에서 obstacle_max_range (5.5 m) 안의 끝점만 — 창 안이어도 그 밖은 찍지 않는다."""
    cm = dict(conf['costmap'], denoise=False)
    d = m.Dwa2D(conf['cfg'], cm, conf['footprint'], 4.0, 10, True)
    d.reset(dwa2d.path_poses(np.stack([np.arange(0, 3, 0.05), np.zeros(60)], 1)))
    sx, sy = -3.5, -3.5                 # 창 [−7.5, 0.5]², 벽 y = 0.4 (센서에서 3.9 m)
    r, _ = _wall_scan(0.4 - sy)
    d.update_costmap(sx, sy, 0.0, sx, sy, 0.0, r, -math.pi, math.radians(0.5), 0.1, 25.0)
    ob = d.obstacle_grid()
    ox, oy = d.costmap_origin()
    ys, xs = np.nonzero(ob == 254)
    wx, wy = ox + (xs + 0.5) * 0.025, oy + (ys + 0.5) * 0.025
    dist = np.hypot(wx - sx, wy - sy)
    assert len(xs) > 100 and dist.max() < 5.5 + 0.03
    # 창 안 벽 위에서 5.5 m 밖인 곳 (|x − sx| > √(5.5² − 3.9²) = 3.88) 은 찍히지 않았다
    far = np.abs(wx - sx) > 3.92
    assert not np.any(far)
    assert np.any(np.abs(wx - sx) > 3.7)                        # 경계 가까이까지는 찍혔다
