"""
정본 safety φ 이식 시험 (safety_gate_torch, env_math.CanonicalPhi, eval_metrics) — Isaac 불필요.

C++ 원본과의 점별 대조는 tools/phi_crosscheck/ 가 맡고, 여기서는 (1) 이식의 매개변수가 safety_node 배포 설정 원문과
같은지, (2) 원본 동작을 가르는 손 사례, (3) 빔 창 통계가 원본의 순차 가지치기까지 같은지(파이썬 순차 기준판),
(4) g++·Eigen 이 있으면 작은 C++ 교차 시험, (5) env 결선·평가 지표 함수를 본다.
"""

import ast
import math
import os
import re
import shutil
import subprocess
import sys

import numpy as np
import pytest
import torch

from amr_isaac import env_math as EM
from amr_isaac import eval_metrics as EV
from amr_isaac import safety_gate_torch as SG
from amr_rl_contract import constants as C

F64 = torch.float64
INF = math.inf
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
HPP = os.path.join(REPO, 'src/amr_perception/include/amr_perception/safety_gate.hpp')
NODE = os.path.join(REPO, 'src/amr_perception/src/safety_node.cpp')
XCHECK = os.path.join(HERE, '..', 'tools', 'phi_crosscheck')
NOT_PORTED = {'estop_release_requires_reset'}     # E-stop 래치 — 시뮬레이션에 E-stop 이 없다


def t(x, dtype=F64):
    """목록 → 텐서."""
    return torch.tensor(x, dtype=dtype)


# ---------------------------------------------------------------- 설정 원문 대조
def _hpp_defaults():
    src = open(HPP, encoding='utf-8').read()
    body = src[src.index('struct SafetyParams'):]
    body = body[:body.index('\n};')]
    out = {}
    for typ, name, val in re.findall(r'^\s*(double|bool|int)\s+(\w+)\{([^}]*)\};', body, re.M):
        out[name] = {'double': float, 'int': int}.get(typ, lambda s: s == 'true')(val)
    return out


def _node_keys():
    src = open(NODE, encoding='utf-8').read()
    out = {}
    pat = r'p\.(\w+)\s*=\s*declare_parameter<\w+>\(\s*"([\w.]+)"\s*,\s*([^;]+?)\);'
    for field, key, default in re.findall(pat, src, re.S):
        out[field] = (key, default.strip())
    return out


def _lookup(d, key):
    cur = d
    for part in key.split('.'):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def test_params_match_deployed_config():
    """
    매개변수 원문 대조.

    GateParams 기본값 = safety_node 가 robot_params.yaml·perception.yaml 에서 읽는 값 (없으면 노드·hpp 기본).
    """
    yaml = pytest.importorskip('yaml')
    robot = yaml.safe_load(open(os.path.join(REPO, 'config/robot_params.yaml'),
                                encoding='utf-8'))['/**']['ros__parameters']
    perc = yaml.safe_load(open(os.path.join(REPO, 'src/amr_perception/config/perception.yaml'),
                               encoding='utf-8'))['/**/safety_node']['ros__parameters']
    hpp = _hpp_defaults()
    keys = _node_keys()
    assert len(hpp) >= 50 and len(keys) >= 40
    expected = dict(hpp)
    for field, (key, default) in keys.items():
        v = _lookup(perc, key)
        if v is None:
            v = _lookup(robot, key)
        if v is None:
            if default.startswith('p.'):
                v = hpp[default[2:]]
            else:
                v = {'true': True, 'false': False}.get(default, None)
                v = float(default) if v is None else v
        expected[field] = v
    # p.cloud_required = cloud_enabled_ = depth_cloud.enabled (safety_node.cpp:170-171)
    assert 'cloud_required = cloud_enabled_' in open(NODE, encoding='utf-8').read()
    expected['cloud_required'] = _lookup(perc, 'depth_cloud.enabled')
    gp = SG.DEPLOYED.__dict__
    assert set(hpp) == set(gp) | NOT_PORTED, set(hpp) ^ (set(gp) | NOT_PORTED)
    for k, v in gp.items():
        e = expected[k]
        assert (v == e) if isinstance(v, bool) else math.isclose(v, e, abs_tol=0), (k, v, e)
    # 배포 설정이 hpp 기본과 다른 두 값 (문서화된 차이)
    assert SG.DEPLOYED.yield_escape_enabled is False and hpp['yield_escape_enabled'] is True
    assert SG.DEPLOYED.cloud_required is True and hpp['cloud_required'] is False


# ---------------------------------------------------------------- 손 사례
def gate(n=1, **kw):
    """빔 수 작은 게이트 (원형 360 빔)."""
    return SG.SafetyGateBatch(n, 'cpu', n_beams=kw.pop('B', 360),
                              angle_increment=kw.pop('inc', 2 * math.pi / 360),
                              circular=True, origin=(0.15, 0.0), **kw)


def wall(x, B=360, y0=-0.3, y1=0.3, n=61):
    """앞 벽 점 (base_link, x 위치) [1, B, 2]."""
    pts = torch.full((1, B, 2), math.nan, dtype=F64)
    ys = torch.linspace(y0, y1, n, dtype=F64)
    pts[0, :n, 0] = x
    pts[0, :n, 1] = ys
    return pts


def run(g, now, v, w, vm=0.0, wm=0.0, beams=None, stamp=None, cloud=True):
    """한 틱 (측정 → 스캔 → 빈 점군 → 명령 → evaluate)."""
    g.set_measured(t([vm]), t([wm]), now)
    if beams is not None:
        g.set_scan(beams, now if stamp is None else stamp)
    if cloud:
        g.set_cloud(None, now, now)
    g.set_command(t([v]), t([w]), now)
    return g.evaluate(now)


def test_straight_approach_and_reverse():
    """직진 접근 = 앞 변까지 (x − 0.3), 후진은 뒤, 옆 벽은 영역 밖 (gtest ...StraightCorridorAndReverse)."""
    r = SG.Region(t([0.5, -0.3, 0.5]), t([0.0, 0.0, 0.0]), t([True, True, True], torch.bool),
                  SG.DEPLOYED)
    px = t([[1.0, -0.8, 0.1], [1.0, -0.8, 0.1], [0.5, 0.5, 0.5]])
    py = t([[0.1, 0.1, 0.25], [0.1, 0.1, 0.25], [0.21, 0.19, 0.0]])
    s = r.approach(px, py)
    assert s[0, 0] == pytest.approx(0.7) and math.isinf(s[0, 1]) and math.isinf(s[0, 2])
    assert math.isinf(s[1, 0]) and s[1, 1] == pytest.approx(0.5)
    assert math.isinf(s[2, 0]) and s[2, 1] == pytest.approx(0.2) and s[2, 2] == pytest.approx(0.2)


def test_rotation_sweeps_corners_not_circumscribed_circle():
    """제자리 회전: 꼭짓점 원 밖·안 (gtest RotationSweepsCornersOnly) — lite 의 외접원 근사와 다른 점."""
    r = SG.Region(t([0.0]), t([1.0]), t([True], torch.bool), SG.DEPLOYED)
    # 옆 0.25 m (꼭짓점 원 반지름 0.3606 안) 점은 닿는다, 0.37 m 점은 영영 안 닿는다
    s = r.approach(t([[0.0, 0.0, 0.0]]), t([[0.25, 0.37, -0.25]]))
    assert math.isfinite(float(s[0, 0])) and math.isinf(float(s[0, 1]))
    assert math.isfinite(float(s[0, 2]))


def test_zone_caps_and_clearance_limit():
    """D 0.8 → WARNING 0.5 m/s 점 속도 상한, 여유 거리 상한 (gtest ClearWarningCriticalSpeedCaps)."""
    g = gate()
    out = run(g, 1.0, 1.0, 0.0, beams=wall(0.3 + 0.8))
    assert int(out['zone'][0]) == SG.ZONE_WARNING
    at = 0.15
    lim = -at + math.sqrt(at * at + 2 * (0.8 - 0.3))
    assert float(out['speed_limit'][0]) == pytest.approx(min(0.5, lim))
    assert float(out['v'][0]) == pytest.approx(min(0.5, lim))


def test_stop_needs_two_frames_or_immediate_margin():
    """D 0.28: 첫 프레임은 CRITICAL(래치 없음, 속도 0), 둘째 프레임 STOP. D 0.24 는 한 프레임 STOP."""
    g = gate()
    o1 = run(g, 1.00, 0.3, 0.0, beams=wall(0.58))
    assert not bool(o1['prox'][0]) and int(o1['zone'][0]) == SG.ZONE_CRITICAL
    assert float(o1['v'][0]) == 0.0
    o2 = run(g, 1.02, 0.3, 0.0)                     # 같은 스캔 → 새 프레임 아님
    assert not bool(o2['prox'][0])
    o3 = run(g, 1.10, 0.3, 0.0, beams=wall(0.58))
    assert bool(o3['prox'][0]) and int(o3['causes'][0]) == SG.CAUSE_LIDAR
    g2 = gate()
    o4 = run(g2, 1.0, 0.3, 0.0, beams=wall(0.54))
    assert bool(o4['prox'][0])


def test_latch_released_when_motion_changes_but_held_by_intent():
    """STOP 뒤 명령 0 → 의도 유지로 래치 계속, 후진 명령 → 그 운동의 D = inf > 0.5 → 해제 (gtest MovingAway...)."""
    g = gate()
    run(g, 1.0, 0.3, 0.0, beams=wall(0.5))
    assert bool(run(g, 1.1, 0.3, 0.0, beams=wall(0.5))['prox'][0])
    held = run(g, 1.12, 0.0, 0.0)
    assert bool(held['prox'][0]) and int(held['zone'][0]) == SG.ZONE_STOP
    back = run(g, 1.14, -0.2, 0.0)
    assert not bool(back['prox'][0]) and float(back['v'][0]) == pytest.approx(-0.2)


def test_measured_motion_extends_region():
    """명령 0·측정 전진이면 측정 운동의 영역이 남는다 (gtest MeasuredVelocityExtendsRegion)."""
    g = gate()
    out = run(g, 1.0, 0.0, 0.0, vm=0.4, beams=wall(0.3 + 0.45))
    assert math.isfinite(float(out['min_distance'][0]))
    g2 = gate()
    out2 = run(g2, 1.0, 0.0, 0.0, vm=0.0, beams=wall(0.3 + 0.45))
    assert math.isinf(float(out2['min_distance'][0]))


def test_contact_guard_escape_only_moving_away():
    """옆 0.01 m 접촉 가드 → STOP. 벽을 따라 나아가면(가드 거리 안 줄어듦) 통과, 벽 쪽 원호는 0 (gtest ContactGuard...)."""
    B = 360
    pts = torch.full((1, B, 2), math.nan, dtype=F64)
    xs = torch.linspace(-0.25, 0.25, 51, dtype=F64)
    pts[0, :51, 0] = xs
    pts[0, :51, 1] = 0.21
    for v, w, ok in ((0.1, 0.0, True), (-0.1, 0.0, True), (0.1, 1.0, False)):
        g = gate(B=B)
        run(g, 1.0, 0.0, 0.0, beams=pts)
        o = run(g, 1.1, 0.0, 0.0, beams=pts)
        assert bool(o['prox'][0]) and int(o['causes'][0]) == SG.CAUSE_CONTACT
        e = run(g, 1.12, v, w)
        assert bool(e['escaping'][0]) == ok and bool(e['prox'][0])
        assert (float(e['v'][0]) == pytest.approx(v)) if ok else float(e['v'][0]) == 0.0


def test_stale_command_and_required_cloud():
    """명령 0.5 s 넘게 끊기면 0, 필수 점군이 없으면 전진만 0.2 m/s (gtest StaleCommandIsZeroed, Stale...Cloud)."""
    g = gate()
    o = run(g, 1.0, 0.8, 0.0, beams=wall(5.0), cloud=False)
    assert bool(o['cloud_stale'][0]) and float(o['v'][0]) == pytest.approx(0.2)
    o = run(g, 1.02, -0.4, 0.0, cloud=False)
    assert float(o['v'][0]) == pytest.approx(-0.4)
    g.set_command(t([0.5]), t([0.0]), 2.0)
    assert float(g.evaluate(2.5)['v'][0]) == pytest.approx(0.2)
    o = g.evaluate(2.5000001)
    assert bool(o['command_stale'][0]) and float(o['v'][0]) == 0.0


def test_ttc_limit_continuous_and_dynamic_only():
    """TTC 1.0 s → v ≤ a(TTC − 0.15) = 0.85, 정적 트랙 TTC 무시 (gtest TtcContinuous…·OnlyDynamic)."""
    g = gate(max_tracks=2)
    g.set_tracks(t([[[1.0, 1.0, 2.0, 0.0, -1.0, 0.0], [0.1, 0.0, 1.0, 0.0, 0.0, 0.0]]]),
                 t([[True, True]], torch.bool), 1.0, 1.0)
    o = run(g, 1.0, 1.0, 0.0, beams=wall(5.0))
    assert float(o['v'][0]) == pytest.approx(0.85) and bool(o['ttc_limited'][0])
    assert float(o['min_ttc'][0]) == pytest.approx(1.0)


def test_curvature_preserving_scale():
    """점 속도 상한은 v·ω 를 같은 비율로 (gtest PointSpeedLimitPreservesCurvature)."""
    g = gate()
    o = run(g, 1.0, 0.8, 0.6, beams=wall(0.3 + 0.8, y0=-0.05, y1=0.6, n=31))
    assert float(o['v'][0]) < 0.8 - 1e-6
    assert float(o['v'][0]) / float(o['w'][0]) == pytest.approx(0.8 / 0.6)


# ---------------------------------------------------------------- 빔 창 통계 = 원본 순차 가지치기
def _seq_reference(vals, rngs, inc, width, mw, sup, circ):
    """원본 순차판 — physicalWindowStatistic (safety_gate.cpp:241-277), 가지치기 포함."""
    n = len(vals)
    min_half = max(mw, 1) // 2
    max_half = max(min_half, (n - 1) // 2)
    dphi = max(abs(inc), 1e-6)
    best = INF
    for i in range(n):
        if not math.isfinite(vals[i]) or vals[i] >= best:
            continue
        r = rngs[i] if (math.isfinite(rngs[i]) and rngs[i] > 0) else 1.0
        half = min(max(int(math.ceil(0.5 * width / (r * dphi))), min_half), max_half)
        ln = 2 * half + 1
        k = max(1, int(math.ceil(sup * ln - 1e-9)))
        buf = []
        for j in range(i - half, i + half + 1):
            if circ:
                buf.append(vals[((j % n) + n) % n])
            else:
                buf.append(vals[j] if 0 <= j < n else INF)
        buf.sort()
        best = min(best, buf[k - 1])
    return best


@pytest.mark.parametrize('circ', [True, False])
@pytest.mark.parametrize('guard', [True, False])
def test_window_statistic_matches_sequential_reference(circ, guard):
    """이식 = 원본 순차 규칙 (가지치기 포함). 그리고 가지치기가 실제로 결과를 바꾸는 배열이 있다."""
    rng = np.random.default_rng(7 + 2 * circ + guard)
    width, mw, sup = (0.06, 9, 0.5) if guard else (0.04, 5, 0.6)
    pruned_differs = 0
    for n in (9, 37, 120):
        A = 60
        vals = rng.uniform(0, 3, (A, n))
        vals = np.where(rng.random((A, n)) < rng.uniform(0.2, 0.9, (A, 1)), np.inf, vals)
        rngs = np.exp(rng.uniform(np.log(0.1), np.log(12.0), (A, n)))
        inc = 2 * math.pi / n
        spec = SG.WindowSpec(t(rngs), inc, width, mw, sup, circ)
        got = SG.window_statistic(t(vals), spec).numpy()
        for a in range(A):
            ref = _seq_reference(vals[a], rngs[a], inc, width, mw, sup, circ)
            assert got[a] == ref or (math.isinf(got[a]) and math.isinf(ref)), (n, a, got[a], ref)
            # 가지치기 없는 "창 통계의 최소"
            spec1 = SG.WindowSpec(t(rngs[a:a + 1]), inc, width, mw, sup, circ)
            s = SG.window_kth(t(vals[a:a + 1]), spec1).numpy()[0]
            free = np.min(np.where(np.isfinite(vals[a]), s, np.inf))
            pruned_differs += int(free < ref)
    assert pruned_differs > 0


def test_window_half_cap_is_noop_for_valid_beams():
    """고정 패딩: env 의 half_cap 은 range_min 이상 빔에서 결과를 바꾸지 않는다."""
    rng = np.random.default_rng(3)
    B = 720
    vals = np.where(rng.random((8, B)) < 0.5, np.inf, rng.uniform(0, 2, (8, B)))
    rngs = rng.uniform(C.LIDAR_RANGE_MIN, 3.0, (8, B))
    cap = SG.max_half_for_range(0.99 * C.LIDAR_RANGE_MIN, C.LIDAR_ANGLE_INC)
    for width, mw, sup in ((0.04, 5, 0.6), (0.06, 9, 0.5)):
        a = SG.window_statistic(t(vals), SG.WindowSpec(t(rngs), C.LIDAR_ANGLE_INC, width, mw,
                                                       sup, True))
        b = SG.window_statistic(t(vals), SG.WindowSpec(t(rngs), C.LIDAR_ANGLE_INC, width, mw,
                                                       sup, True, half_cap=cap))
        assert torch.equal(a, b)


# ---------------------------------------------------------------- C++ 교차 시험 (작은 묶음)
@pytest.mark.skipif(shutil.which('g++') is None or not os.path.isdir('/usr/include/eigen3'),
                    reason='g++·Eigen 없음 (amr-ai/rl:0.3 에서 돈다)')
def test_cpp_crosscheck_small(tmp_path):
    """C++ 원본 SafetyGate 와 같은 입력 열에서 불일치 0 (전체 묶음은 tools/phi_crosscheck/crosscheck.sh)."""
    pe = os.path.join(REPO, 'src', 'amr_perception')
    binary = str(tmp_path / 'gate_runner')
    srcs = [os.path.join(XCHECK, 'gate_runner.cpp'), os.path.join(pe, 'src', 'safety_gate.cpp'),
            os.path.join(pe, 'src', 'geometry2d.cpp')]
    subprocess.run(['g++', '-std=c++17', '-O1', f'-I{pe}/include', '-I/usr/include/eigen3', *srcs,
                    '-o', binary], check=True)
    out = tmp_path / 'xcheck.json'
    r = subprocess.run([sys.executable, os.path.join(XCHECK, 'crosscheck.py'), '--binary', binary,
                        '--fp_motions', '120', '--fp_points', '80', '--win_arrays', '600',
                        '--random_scenarios', '80', '--batch', '80', '--ticks', '40',
                        '--out', str(out)], capture_output=True, text=True,
                       env={**os.environ, 'XCHECK_TMP': str(tmp_path)})
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]


# ---------------------------------------------------------------- env 결선
def test_canonical_phi_beams_and_cloud_modes():
    """정본 φ 결선의 빔 = 원점 + r(cos, sin), 무효 = NaN. 점군 'empty' = 상한 없음, 'none' = 0.2 m/s."""
    r = torch.full((1, C.LIDAR_SAMPLES), 20.0, dtype=F64)
    r[0, 360] = 1.0
    r[0, 10] = 0.05
    r[0, 11] = math.nan
    phi = EM.CanonicalPhi(1, 'cpu')
    b = phi.beams(r)
    # 빔 360 각 = 3.1e-9 rad (yaml 의 π 자릿수)
    assert b[0, 360, 0] == pytest.approx(C.LIDAR_X + 1.0) and abs(float(b[0, 360, 1])) < 1e-8
    assert torch.isnan(b[0, 10]).all() and torch.isnan(b[0, 11]).all()
    for mode, v_exp in (('empty', 0.9), ('none', 0.2)):
        phi = EM.CanonicalPhi(1, 'cpu', cloud_mode=mode)
        phi.set_scan(r * 0 + 20.0, 0.0)
        out = phi.step(t([0.9]), t([0.0]), t([0.0]), t([0.0]), 0.0)
        assert float(out['v'][0]) == pytest.approx(v_exp)
    with pytest.raises(ValueError):
        EM.CanonicalPhi(1, 'cpu', cloud_mode='proxy')


def test_canonical_phi_stops_before_wall_and_latches():
    """정면 벽 (D = 0.23 ≤ 0.25 → 한 프레임 확정) → 전진 0, STOP 래치, 후진은 통과."""
    r = torch.full((1, C.LIDAR_SAMPLES), 20.0, dtype=F64)
    ang = C.LIDAR_ANGLE_MIN + torch.arange(C.LIDAR_SAMPLES, dtype=F64) * C.LIDAR_ANGLE_INC
    front = ang.abs() < 0.5
    r[0, front] = (0.53 - C.LIDAR_X) / torch.cos(ang[front])
    phi = EM.CanonicalPhi(1, 'cpu')
    phi.set_scan(r, 0.0)
    o = phi.step(t([0.3]), t([0.0]), t([0.0]), t([0.0]), 0.0)
    assert float(o['v'][0]) == 0.0 and bool(o['prox'][0])
    o = phi.step(t([-0.2]), t([0.0]), t([0.0]), t([0.0]), 0.02)
    assert float(o['v'][0]) == pytest.approx(-0.2) and not bool(o['prox'][0])


def test_env_wires_canonical_phi_and_dwell():
    """env.py 가 CanonicalPhi·dwell_success 를 env_math·eval_metrics 에서 가져다 쓰고 lite 길을 남긴다."""
    src = open(os.path.join(HERE, '..', 'amr_isaac', 'env.py'), encoding='utf-8').read()
    tree = ast.parse(src)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            imported |= {a.name for a in node.names}
    assert {'CanonicalPhi', 'dwell_success', 'approach_distance'} <= imported
    assert 'self.phi.step(' in src and 'self.phi.set_scan(' in src and 'self.phi.reset(' in src
    assert 'CT.safety_filter(v_ref, w_ref, d_main, self.min_ttc, self.prox_stop)' in src


# ---------------------------------------------------------------- 평가 지표
def test_dwell_success_requires_k_consecutive_steps():
    """연속 3 스텝(k = 3)에서만 성공 종단, 끊기면 다시 센다, 충돌 종단은 그대로."""
    succ = [True, True, False, True, True, True]
    dwell = torch.zeros(1, dtype=torch.long)
    got = []
    for s in succ:
        sb = torch.tensor([s])
        s2, t2, dwell = EV.dwell_success(sb, sb.clone(), dwell, 3)
        got.append((bool(s2[0]), bool(t2[0])))
    assert got == [(False, False)] * 5 + [(True, True)]
    s2, t2, _ = EV.dwell_success(torch.tensor([False]), torch.tensor([True]),
                                 torch.zeros(1, dtype=torch.long), 3)
    assert not bool(s2[0]) and bool(t2[0])
    s0, t0, d0 = EV.dwell_success(torch.tensor([True]), torch.tensor([True]),
                                  torch.zeros(1, dtype=torch.long), 1)
    assert bool(s0[0]) and bool(t0[0]) and int(d0[0]) == 0


def test_layered_metrics_and_stop_classes():
    """층위 지표와 사전 등록한 정지 분류 (φ 차단·정책 정지·혼합)."""
    K = 20
    pairs = np.zeros((3, K, 5, 4))
    pairs[0, :, :, 0] = 0.2            # 정책 전진, φ 0 → 차단
    pairs[1, :, :, 0] = 0.0            # 정책 정지
    pairs[2, :10, :, 0] = 0.2          # 절반만 차단 → 혼합
    pairs[2, :10, :, 2] = 0.0
    ep = {'outcome': np.array([3, 3, 3, 0]), 'nav2_reach': np.array([True, True, False, True]),
          'd_end': np.array([0.15, 0.2, 0.4, 0.05]), 'cost': np.zeros((4, 5)),
          'steps': np.array([300, 300, 300, 80]), 'path_id_orig': np.arange(4),
          'diag_pairs': np.concatenate([pairs, np.zeros((1, K, 5, 4))]),
          'diag_D': np.zeros((4, K)), 'diag_guard': np.zeros((4, K)),
          'diag_zone': np.zeros((4, K), int), 'diag_prox': np.zeros((4, K), bool),
          'diag_causes': np.zeros((4, K), int)}
    lay = EV.layered(ep)
    assert lay['strict_success'] == 0.25 and lay['nav2_compatible_success'] == 0.75
    assert lay['nav2_but_not_strict'] == 2 and lay['fail_within_0p25m'] == 2
    assert lay['final_dist_fail']['median'] == pytest.approx(0.2)
    dg = EV.stop_diagnosis(ep)
    assert dg['counts'] == {'phi_blocked': 1, 'policy_stopped': 1, 'mixed': 1}


def test_goal_clearance_geometry():
    """목표 여유 3 종: 정면 상자까지 직진 접근 거리 = 상자 면 − 앞 변."""
    obb = np.array([[2.0, 0.0, 0.5, 0.5, 0.0]])               # x ∈ [1.5, 2.5]
    path = np.stack([np.linspace(0.0, 1.0, 21), np.zeros(21)], 1)
    g = EV.goal_clearances(path, obb)
    assert g['goal_point'] == pytest.approx(0.5)
    assert g['goal_footprint'] == pytest.approx(0.2)
    assert g['goal_front_straight'] == pytest.approx(0.2)
    path2 = np.stack([np.zeros(21), np.linspace(0.0, 1.0, 21)], 1)   # 위로 향함 → 상자는 옆
    g2 = EV.goal_clearances(path2, obb)
    assert math.isinf(g2['goal_front_straight'])
