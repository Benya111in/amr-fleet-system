"""
Isaac env 순수 함수 시험 (env_math, Isaac 불필요).

env.py 의 충돌·φ 접근거리·적분은 env_math 에서 온다 — test_env_uses_env_math 가 그 배선을 고정한다.
"""

import ast
import math
import os

import numpy as np
import pytest
import torch

from amr_isaac import env_math as EM
from amr_isaac import warehouse_geom as W
from amr_rl_contract import constants as C
from amr_rl_contract import geometry as G

F64 = torch.float64
INF = math.inf
ENV_PY = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'amr_isaac', 'env.py')


def t(x, dtype=F64):
    """목록 → torch 텐서."""
    return torch.tensor(x, dtype=dtype)


@pytest.fixture(scope='module')
def obb():
    return W.obstacles_2d(W.load_boxes())


# ---------------------------------------------------------------- 배선
def test_env_uses_env_math():
    """env.py 가 순수 함수를 다시 정의하지 않고 env_math 에서 가져다 쓴다 (시험 대상 = 실행 코드)."""
    tree = ast.parse(open(ENV_PY, encoding='utf-8').read())
    names = {'footprint_hits', 'integrate_pose', 'approach_distance', 'move_and_collide'}
    defined = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assert not (defined & names)
    imported = set()
    for n in tree.body:
        if isinstance(n, ast.ImportFrom) and n.module == 'env_math' and n.level == 1:
            imported |= {a.name for a in n.names}
    assert {'approach_distance', 'move_and_collide'} <= imported
    called = {n.func.id for n in ast.walk(tree)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)}
    assert {'approach_distance', 'move_and_collide'} <= called


# ---------------------------------------------------------------- footprint_hits
def _near_boundary_poses(obb, n, seed):
    """창고 사각형 경계 근처 자세 — 면에서 풋프린트 반폭(0.20)~외접 반경(0.36) 언저리."""
    rng = np.random.default_rng(seed)
    k = rng.integers(0, len(obb), n)
    cx, cy, hx, hy, yaw = obb[k].T
    side = rng.integers(0, 4, n)
    gap = rng.uniform(0.05, 0.50, n)                        # 면에서 로봇 중심까지
    along = rng.uniform(-1.1, 1.1, n)                       # 면을 따라 (모서리 밖 조금 포함)
    lx = np.where(side < 2, np.where(side == 0, 1, -1) * (hx + gap), along * hx)
    ly = np.where(side < 2, along * hy, np.where(side == 2, 1, -1) * (hy + gap))
    c, s = np.cos(yaw), np.sin(yaw)
    px = cx + c * lx - s * ly
    py = cy + s * lx + c * ly
    th = rng.uniform(-math.pi, math.pi, n)
    return np.stack([px, py, th], 1)


def test_footprint_hits_matches_numpy_reference(obb):
    """경계 근처 자세 12 000 개에서 torch 판 = numpy 기준판 (정확히 같다)."""
    pose = _near_boundary_poses(obb, 12000, seed=0)
    got = EM.footprint_hits(t(pose), t(obb)).numpy()
    ref = np.array([W.footprint_hits_obb(tuple(p), obb, C.HALF_L, C.HALF_W) for p in pose])
    # 시험이 의미 있으려면 맞음·안 맞음이 둘 다 많아야 한다
    assert 0.2 < ref.mean() < 0.8, ref.mean()
    bad = np.flatnonzero(got != ref)
    assert bad.size == 0, pose[bad[:5]].tolist()


def test_footprint_hits_obstacle_axis_only_separation():
    """
    로봇 축으로는 겹치고 장애물 축(45° 마름모)으로만 떨어진 경우 = 비충돌.

    로봇 축 투영: x 0.45 − 0.283 < 0.30, y 0.40 − 0.283 < 0.20 (겹침).
    마름모 축 투영: (0.45 + 0.40)/√2 = 0.601 > 0.354 + 0.20 (분리).
    """
    diamond = t([[0.45, 0.40, 0.20, 0.20, math.pi / 4]])
    assert not bool(EM.footprint_hits(t([[0.0, 0.0, 0.0]]), diamond)[0])
    assert not W.footprint_hits_obb((0.0, 0.0, 0.0), diamond.numpy(), C.HALF_L, C.HALF_W)
    # 조금 더 가까이 = 충돌
    near = t([[0.40, 0.35, 0.20, 0.20, math.pi / 4]])
    assert bool(EM.footprint_hits(t([[0.0, 0.0, 0.0]]), near)[0])


def test_footprint_hits_robot_axis_only_separation():
    """
    역할 바꿈: 45° 돈 정사각 풋프린트(반폭 0.20)가 축 정렬 0.60×0.40 사각형 옆 — 로봇 축으로만 분리.

    장애물 축 투영: x 0.45 < 0.30 + 0.283, y 0.40 < 0.20 + 0.283 (겹침). 로봇 축: 0.601 > 0.20 + 0.354.
    """
    box = t([[0.0, 0.0, 0.30, 0.20, 0.0]])
    hl = hw = 0.20
    assert not W.footprint_hits_obb((0.45, 0.40, math.pi / 4), box.numpy(), hl, hw)
    assert not bool(EM.footprint_hits(t([[0.45, 0.40, math.pi / 4]]), box, hl, hw)[0])
    assert bool(EM.footprint_hits(t([[0.40, 0.35, math.pi / 4]]), box, hl, hw)[0])


def test_footprint_hits_touching_is_hit():
    """경계 포함: 면이 정확히 닿으면 충돌, 2^-20 m 떨어지면 비충돌 (이진 정확 값)."""
    box = t([[1.0, 0.0, 0.5, 0.5, 0.0]])
    assert bool(EM.footprint_hits(t([[0.25, 0.0, 0.0]]), box, 0.25, 0.25)[0])
    assert not bool(EM.footprint_hits(t([[0.25 - 2.0 ** -20, 0.0, 0.0]]), box, 0.25, 0.25)[0])


def test_footprint_hits_narrow_passage(obb):
    """좁은 통로 (0, −10) 순폭 0.60 m: test_warehouse_paths 의 numpy 사례와 같은 판정."""
    cases = [((0.0, 0.0, 0.0), False), ((29.8, 0.0, 0.0), True), ((29.69, 0.0, 0.0), False),
             ((0.0, -10.0, math.pi / 2), False), ((0.0, -10.0, 0.0), True),
             ((0.0, -10.0, math.pi / 2 + 0.2), False), ((0.0, -10.0, math.pi / 2 + 0.5), True)]
    got = EM.footprint_hits(t([p for p, _ in cases]), t(obb)).tolist()
    assert got == [h for _, h in cases]


# ---------------------------------------------------------------- integrate_pose
def _arc_reference(pose, v, w, dt):
    """계약 geometry.integrate_arc (원점 기준 정확 적분) 를 세계 자세로 옮긴다."""
    out = []
    for (x, y, th), vi, wi in zip(pose, v, w):
        lx, ly, lth = G.integrate_arc(vi, wi, dt)
        c, s = math.cos(th), math.sin(th)
        out.append((x + c * lx - s * ly, y + s * lx + c * ly, G.wrap_angle(th + lth)))
    return np.asarray(out)


def _ang_err(a, b):
    return np.abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def test_integrate_pose_matches_contract_arc():
    """무작위 3 000 개 (직진 310 포함) — 계약 정확 적분과 같다."""
    rng = np.random.default_rng(1)
    n = 3000
    pose = np.stack([rng.uniform(-30, 30, n), rng.uniform(-20, 20, n),
                     rng.uniform(-math.pi, math.pi, n)], 1)
    v = rng.uniform(-0.3, 1.5, n)
    w = rng.uniform(-1.5, 1.5, n)
    w[:300] = 0.0
    w[300:310] = 1e-10                                      # 직진 분기 문턱 아래
    for dt in (C.PROFILER_DT, C.DT):
        got = EM.integrate_pose(t(pose), t(v), t(w), dt).numpy()
        ref = _arc_reference(pose, v, w, dt)
        # 작은 ω 에서 v/ω·(sin − sin) 상쇄 오차가 ~r·ε 라 위치는 1e-9 (각은 1e-12)
        assert np.abs(got[:, :2] - ref[:, :2]).max() < 1e-9
        assert _ang_err(got[:, 2], ref[:, 2]).max() < 1e-12


def test_integrate_pose_matches_fine_euler():
    """독립 대조: 2 000 분할 중점 적분과 1e-9 m 안 (원호 식의 부호·축 실수를 잡는다)."""
    rng = np.random.default_rng(2)
    n = 500
    pose = np.stack([rng.uniform(-5, 5, n), rng.uniform(-5, 5, n),
                     rng.uniform(-math.pi, math.pi, n)], 1)
    v = rng.uniform(-0.3, 1.5, n)
    w = rng.uniform(-1.5, 1.5, n)
    dt = C.PROFILER_DT
    got = EM.integrate_pose(t(pose), t(v), t(w), dt).numpy()
    x, y, th = pose[:, 0].copy(), pose[:, 1].copy(), pose[:, 2].copy()
    m = 2000
    h = dt / m
    for _ in range(m):
        thm = th + 0.5 * w * h                              # 중점 각 (2 차)
        x += v * np.cos(thm) * h
        y += v * np.sin(thm) * h
        th += w * h
    assert max(np.abs(got[:, 0] - x).max(), np.abs(got[:, 1] - y).max()) < 1e-9
    assert _ang_err(got[:, 2], th).max() < 1e-9


def test_integrate_pose_wraps_heading():
    """θ 는 [−π, π) 로 감긴다."""
    out = EM.integrate_pose(t([[0.0, 0.0, 3.1], [0.0, 0.0, -3.1]]), t([0.5, 0.5]),
                            t([1.0, -1.0]), 0.1)
    assert out[0, 2].item() == pytest.approx(3.2 - 2 * math.pi, abs=1e-12)
    assert out[1, 2].item() == pytest.approx(-3.2 + 2 * math.pi, abs=1e-12)
    assert bool(((out[:, 2] >= -math.pi) & (out[:, 2] < math.pi)).all())


# ---------------------------------------------------------------- approach_distance
R = C.ROBOT_RADIUS
APPROACH_CASES = [
    # (이름, 로봇 좌표 점, valid, v, w, 기대 D)
    ('앞', (1.0, 0.0), True, 0.5, 0.0, 1.0 - C.HALF_L),
    ('앞_원호도_직진띠', (1.0, 0.0), True, 0.5, 0.8, 1.0 - C.HALF_L),
    ('옆_띠밖', (1.0, 0.25), True, 0.5, 0.0, INF),
    ('띠_경계_포함', (1.0, C.HALF_W), True, 0.5, 0.0, 1.0 - C.HALF_L),
    ('안', (0.1, 0.1), True, 0.5, 0.0, 0.0),
    ('안_정지', (0.1, 0.1), True, 0.0, 0.0, 0.0),
    ('안_무효빔', (0.1, 0.1), False, 0.5, 0.0, INF),
    ('뒤', (-1.0, 0.0), True, 0.5, 0.0, INF),
    ('후진_뒤', (-1.0, 0.0), True, -0.2, 0.0, 1.0 - C.HALF_L),
    ('후진_앞', (1.0, 0.0), True, -0.2, 0.0, INF),
    ('제자리회전', (1.0, 0.0), True, 0.0, 0.5, 1.0 - R),
    ('제자리회전_외접원안', (0.33, 0.05), True, 0.0, -0.5, 0.0),
    ('정지', (1.0, 0.0), True, 0.0, 0.0, INF),
    ('문턱아래_속도', (1.0, 0.0), True, 5e-4, 5e-4, INF),
]


@pytest.mark.parametrize('name,pt,valid,v,w,want', APPROACH_CASES,
                         ids=[c[0] for c in APPROACH_CASES])
def test_approach_distance_cases(name, pt, valid, v, w, want):
    """한 점 사례: 앞(0.7)·옆(inf)·안(0)·뒤(inf)·후진·제자리 회전·정지."""
    got = EM.approach_distance(t([[pt]]), torch.tensor([[valid]]), t([v]), t([w]))[0].item()
    if math.isinf(want):
        assert math.isinf(got) and got > 0
    else:
        assert got == pytest.approx(want, abs=1e-12)


def test_approach_distance_batch_min_and_inside_override():
    """여러 점: 띠 안 최솟값, 무효 점 무시, 안 점이 하나라도 있으면 0 (배치에서 행별로)."""
    pts = t([[[2.0, 0.0], [0.9, -0.15], [0.5, 0.5], [0.4, 0.0]],
             [[2.0, 0.0], [0.9, -0.15], [0.5, 0.5], [0.0, 0.0]],
             [[2.0, 0.0], [0.9, -0.15], [0.5, 0.5], [0.0, 0.0]]])
    valid = torch.tensor([[True, True, True, False],
                          [True, True, True, True],
                          [True, True, True, False]])
    d = EM.approach_distance(pts, valid, t([0.5, 0.5, 0.5]), t([0.0, 0.0, 0.0])).tolist()
    assert d[0] == pytest.approx(0.6, abs=1e-12)            # (0.4, 0) 은 무효
    assert d[1] == 0.0                                      # (0, 0) 이 풋프린트 안
    assert d[2] == pytest.approx(0.6, abs=1e-12)


# ---------------------------------------------------------------- move_and_collide
def test_move_and_collide():
    """접촉한 env 는 제자리·속도 0·래치 유지, 이번에 닿은 env 는 적분 뒤 자세에서 멈춘다."""
    obb = t([[1.0, 0.0, 0.25, 1.0, 0.0]])                    # 면 x = 0.75
    dt = 0.02
    pose = t([[-3.0, 0.0, 0.0],                             # 0: 자유 주행
              [0.0, 0.0, 0.0],                              # 1: 이미 접촉 (래치)
              [0.445, 0.0, 0.0],                            # 2: 이번 서브스텝에 닿는다
              [0.0, 0.5, math.pi / 2]])                     # 3: 자유, 회전 포함
    v = t([0.5, 0.5, 0.5, 0.3])
    w = t([0.0, 0.4, 0.0, 0.7])
    col = torch.tensor([False, True, False, False])
    before = (pose.clone(), v.clone(), w.clone(), col.clone())
    p2, v2, w2, c2 = EM.move_and_collide(pose, v, w, col, obb, dt)
    for a, b in zip((pose, v, w, col), before):
        assert torch.equal(a, b)                            # 입력을 바꾸지 않는다
    assert c2.tolist() == [False, True, True, False]
    assert torch.equal(p2[1], pose[1])                      # 접촉 env 는 움직이지 않는다
    assert v2[1].item() == 0.0 and w2[1].item() == 0.0
    ref = EM.integrate_pose(pose, v, w, dt)
    assert torch.equal(p2[2], ref[2])                       # 닿은 자세에서 멈춘다
    assert p2[2, 0].item() == pytest.approx(0.455, abs=1e-12)
    assert v2[2].item() == 0.0 and w2[2].item() == 0.0
    for i in (0, 3):
        assert torch.equal(p2[i], ref[i])
        assert v2[i].item() == v[i].item() and w2[i].item() == w[i].item()
    # 래치: 다음 서브스텝에도 2 는 그대로
    p3, v3, _w3, c3 = EM.move_and_collide(p2, t([0.5, 0.5, 0.5, 0.3]), w2, c2, obb, dt)
    assert c3.tolist() == [False, True, True, False]
    assert torch.equal(p3[2], p2[2]) and v3[2].item() == 0.0
