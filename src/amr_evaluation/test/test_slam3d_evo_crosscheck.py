"""
evo 교차 확인 — amr_evaluation.slam3d.se2 의 ATE·RPE 가 evo(GPL-3.0, 도구로만 사용)와 같은 값을 내는지.

evo 가 없으면 건너뛴다 (CI·amr-ai/rl 이미지). 돌리는 법 (amr-ai/slam:0.1 의 /opt/evo venv):

    docker run --rm -v <worktree>:/ws -w /ws/src/amr_evaluation amr-ai/slam:0.1 bash -lc \
      'PYTHONPATH=/ws/src/amr_evaluation /opt/evo/bin/python -m pytest -q \
       test/test_slam3d_evo_crosscheck.py'

주의 두 가지 (evo 1.37.1 에서 확인):
- evo_rpe 의 쌍은 기본으로 **추정** 궤적의 경로 길이로 고른다. 우리 정의(GT 경로)와 같게 하려면
  --pairs_from_reference 를 줘야 한다 (주지 않으면 값이 다르다 — 아래 마지막 시험).
- main_rpe.rpe() 는 넘긴 궤적을 delta_ids 로 제자리에서 줄인다 → 호출마다 복사본을 넘긴다.
"""

import copy
import math

from amr_evaluation.slam3d import se2
import numpy as np
import pytest

evo = pytest.importorskip('evo')
from evo.core import geometry, metrics  # noqa: E402
from evo.core.trajectory import PoseTrajectory3D  # noqa: E402
from evo.main_ape import ape as _ape  # noqa: E402
from evo.main_rpe import rpe as _rpe  # noqa: E402


def ape(r, e, *a, **k):
    return _ape(copy.deepcopy(r), copy.deepcopy(e), *a, **k)


def rpe(r, e, *a, **k):
    return _rpe(copy.deepcopy(r), copy.deepcopy(e), *a, **k)


def loop_traj():
    """20 x 10 m 직사각형 루프 0.5 m/s, 20 Hz, 4 바퀴. 추정 = 축척 1.005, 방위 0.3°, 잡음 1 cm / 0.002 rad."""
    corners = [(0, 0), (20, 0), (20, 10), (0, 10), (0, 0)]
    t = np.arange(0, 240.0, 0.05)
    xy, yaw = [], []
    for tt in t:
        u, acc = (0.5 * tt) % 60.0, 0.0
        for (a, b), (c, d) in zip(corners, corners[1:]):
            seg = math.hypot(c - a, d - b)
            if u <= acc + seg:
                f = (u - acc) / seg
                xy.append((a + f * (c - a), b + f * (d - b)))
                yaw.append(math.atan2(d - b, c - a))
                break
            acc += seg
    xy, yaw = np.array(xy), np.array(yaw)
    rng = np.random.default_rng(42)
    th = math.radians(0.3)
    rot = np.array([[math.cos(th), math.sin(th)], [-math.sin(th), math.cos(th)]])
    exy = 1.005 * xy @ rot + [0.05, -0.02] + rng.normal(0, 0.01, xy.shape)
    eyaw = yaw + th + rng.normal(0, 0.002, len(t))
    return se2.Traj2D(t, xy, yaw), se2.Traj2D(t, exy, eyaw)


def to_evo(tr):
    q = np.stack([np.cos(tr.yaw / 2), 0 * tr.yaw, 0 * tr.yaw, np.sin(tr.yaw / 2)], 1)
    return PoseTrajectory3D(np.c_[tr.xy, np.zeros(len(tr))], q, tr.t)


@pytest.fixture(scope='module')
def trajs():
    g, e = loop_traj()
    return g, e, to_evo(g), to_evo(e)


@pytest.mark.parametrize('align, kw', [('none', {}), ('origin', {'align_origin': True})])
def test_ate_matches_evo(trajs, align, kw):
    g, e, r, x = trajs
    mine = se2.ate(g, e, align)
    ev = ape(r, x, metrics.PoseRelation.translation_part, align=False, correct_scale=False, **kw)
    for k in ('rmse', 'mean', 'median', 'max', 'std'):
        assert mine['trans'][k] == pytest.approx(ev.stats[k], abs=1e-12)
    ev = ape(r, x, metrics.PoseRelation.rotation_angle_deg, align=False, correct_scale=False,
             **kw)
    assert mine['yaw_deg']['rmse'] == pytest.approx(ev.stats['rmse'], abs=1e-9)


def test_se2_alignment_matches_evo_umeyama_on_xy(trajs):
    g, e, _, _ = trajs
    rot, t, _ = geometry.umeyama_alignment(e.xy.T, g.xy.T, with_scale=False)
    moved = (rot @ e.xy.T).T + t
    mine = se2.ate(g, e, 'se2')
    assert mine['trans']['rmse'] == pytest.approx(
        float(np.sqrt(np.mean(np.sum((moved - g.xy) ** 2, axis=1)))), abs=1e-12)
    assert mine['align'][2] == pytest.approx(math.atan2(rot[1, 0], rot[0, 0]), abs=1e-12)


@pytest.mark.parametrize('delta', [1.0, 5.0, 5.013, 10.007])
def test_rpe_matches_evo_with_pairs_from_reference(trajs, delta):
    g, e, r, x = trajs
    mine = se2.rpe(g, e, (delta,), all_pairs=False)[delta]
    kw = dict(delta=delta, delta_unit=metrics.Unit.meters, all_pairs=False,
              pairs_from_reference=True)
    ev = rpe(r, x, metrics.PoseRelation.translation_part, **kw)
    for k in ('rmse', 'mean', 'max'):
        assert mine['trans'][k] == pytest.approx(ev.stats[k], abs=1e-12)
    ev = rpe(r, x, metrics.PoseRelation.rotation_angle_deg, **kw)
    assert mine['rot_deg']['rmse'] == pytest.approx(ev.stats['rmse'], abs=1e-9)


def test_evo_default_pairs_differ(trajs):
    """--pairs_from_reference 없이 쓰면 값이 다르다 (기록용 — 결과 CSV 에 evo 값을 쓸 때의 함정)."""
    g, e, r, x = trajs
    mine = se2.rpe(g, e, (1.0,), all_pairs=False)[1.0]
    ev = rpe(r, x, metrics.PoseRelation.translation_part, delta=1.0,
             delta_unit=metrics.Unit.meters, all_pairs=False)
    assert abs(mine['trans']['rmse'] - ev.stats['rmse']) > 1e-4
