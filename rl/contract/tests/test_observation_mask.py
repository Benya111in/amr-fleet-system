"""관측 배치와 비교군 C 마스크 시험."""

import math

import numpy as np
import pytest

from amr_rl_contract import constants as C
from amr_rl_contract import kinematics as K
from amr_rl_contract import mask as M
from amr_rl_contract import observation as OB
from amr_rl_contract.state import empty_scan, SafetyState, State, Track


def _state(**kw):
    d = dict(pose=(0.0, 0.0, 0.0), path=[(0.0, 0.0), (10.0, 0.0)], scan_ranges=empty_scan())
    d.update(kw)
    return State(**d)


def test_layout():
    assert OB.OBS_DIM == 291
    sl = OB.obs_slices()
    assert sl['lidar_sectors'] == slice(0, 180)
    assert sl['tracks'] == slice(227, 291)
    assert sum(n for _, n in OB.OBS_LAYOUT) == OB.OBS_DIM


def test_front_beam_goes_to_sector_90():
    r = empty_scan()
    r[361] = 1.0          # +0.5°, 끝점 (1.15, 0.0087) → 앞 모서리에서 0.85
    obs = OB.build_observation(_state(scan_ranges=r))
    sec = obs[:180] * C.SECTOR_CLIP
    assert sec[90] == pytest.approx(0.85, abs=1e-4)
    assert np.all(np.delete(sec, 90) == pytest.approx(C.SECTOR_CLIP))


def test_obs_finite_and_bounded_random():
    rng = np.random.default_rng(3)
    for _ in range(30):
        r = [None if rng.random() < 0.3 else float(rng.uniform(0.0, 30.0))
             for _ in range(C.LIDAR_SAMPLES)]
        tracks = [Track(i, *rng.uniform(-12, 12, 2), *rng.uniform(-2, 2, 2),
                        bool(rng.random() < 0.7))
                  for i in range(int(rng.integers(0, 12)))]
        st = _state(pose=tuple(rng.uniform(-3, 3, 3)), scan_ranges=r, tracks=tracks,
                    v_meas=float(rng.uniform(0, 2)), w_meas=float(rng.uniform(-1.5, 1.5)),
                    track_age=float(rng.uniform(0, 1)), scan_age=float(rng.uniform(0, 1)),
                    safety=SafetyState(int(rng.integers(0, 4)), None, False, None))
        obs = OB.build_observation(st)
        assert obs.dtype == np.float32 and obs.shape == (OB.OBS_DIM,)
        assert np.all(np.isfinite(obs)) and np.all(np.abs(obs) <= 3.0)


def test_track_selection_and_order():
    tracks = [Track(1, 2.0, 0.0, 0.0, 0.0, False),     # 정적·느림 → 제외
              Track(2, 3.0, 0.0, 0.6, 0.0, False),     # 빠름 → 포함
              Track(3, 1.0, 1.0, 0.0, 0.0, True),
              Track(4, 11.0, 0.0, 0.0, 0.0, True)]     # 10 m 밖
    st = _state(tracks=tracks, track_age=0.5)
    sel = OB.select_tracks(st)
    assert [r[1] for r in sel] == [3, 2]
    assert sel[1][2] == pytest.approx((3.3, 0.0))          # 0.6 × 0.5 s 외삽
    obs = OB.build_observation(st)
    trk = obs[227:].reshape(C.K_TRACKS, C.TRACK_FEATS)
    assert trk[0, 7] == 1.0 and trk[1, 7] == 1.0 and np.all(trk[2:] == 0.0)


def test_rotated_frame():
    st = _state(pose=(1.0, 1.0, math.pi / 2), path=[(1.0, 1.0), (1.0, 6.0)])
    parts = OB.build_observation(st, return_parts=True)[1]
    # 경로가 로봇 전방: 첫 subgoal 방위 0
    assert parts['path'][1] == pytest.approx(0.0, abs=1e-9)
    assert parts['path'][2] == pytest.approx(1.0)


def test_mask_rest_cells():
    res = M.compute_mask(_state())
    valid = [i for i in range(C.N_CELLS) if res['mask'][i]]
    accs = sorted(K.cell_accel(i) for i in valid)
    assert accs == sorted((a, b) for a in (0.0, 0.2) for b in (-0.5, 0.0, 0.5))
    assert res['mask'][C.FALLBACK_INDEX]


def test_mask_fallback_always_and_nonempty_random():
    rng = np.random.default_rng(5)
    for _ in range(25):
        r = [None if rng.random() < 0.5 else float(rng.uniform(0.1, 3.0))
             for _ in range(C.LIDAR_SAMPLES)]
        st = _state(scan_ranges=r, v_cmd=float(rng.uniform(0, 1)),
                    w_cmd=float(rng.uniform(-1.5, 1.5)),
                    prev_actions=[tuple(rng.uniform([-1, -2], [1, 2]))] * 3,
                    tracks=[Track(0, *rng.uniform(-3, 3, 2), *rng.uniform(-1.5, 1.5, 2), True)])
        res = M.compute_mask(st)
        assert res['mask'][C.FALLBACK_INDEX] and res['mask'].sum() >= 1
        sel = res['speed_ok'] & res['jerk_ok'] & res['fp_ok']
        assert not np.any(res['mask'][:C.N_CELLS] & ~sel)          # 마스크 ⊆ 가행
        for i in np.flatnonzero(res['mask'][:C.N_CELLS]):          # 유효 셀은 한계 안
            v1, w1 = res['v_next'][i], res['w_next'][i]
            assert -1e-9 <= v1 <= C.V_MAX_OP + 1e-9 and abs(w1) <= C.W_MAX + 1e-9


def test_mask_blocked_by_wall_ahead():
    r = empty_scan()
    for i in range(330, 391):
        ang = C.LIDAR_ANGLE_MIN + i * C.LIDAR_ANGLE_INC
        r[i] = (0.55 - C.LIDAR_X) / math.cos(ang)       # 벽 x = 0.55 (전면 0.25 m 앞)
    res = M.compute_mask(_state(scan_ranges=r, v_cmd=0.5, prev_actions=[(0.0, 0.0)] * 3))
    assert not res['fp_ok'][[i for i in range(C.N_CELLS) if res['jerk_ok'][i]]].any()
    assert res['mask'].sum() == 1


def test_mask_saturation_keeps_latest_vo():
    st = _state(v_cmd=0.6, tracks=[Track(3, 1.8, 0.05, -1.5, 0.0, True)])
    res = M.compute_mask(st)
    assert res['saturated']
    sel = res['speed_ok'] & res['jerk_ok'] & res['fp_ok']
    t_max = np.max(res['vo_time'][sel])
    for i in np.flatnonzero(res['mask'][:C.N_CELLS]):
        assert res['vo_time'][i] >= t_max - C.VO_TIME_TIE


def test_mask_vo_ignores_slow_and_far_tracks():
    slow = Track(1, 1.5, 0.0, -0.15, 0.0, True)      # 0.15 < 0.2 → VO 제외
    far = Track(2, 7.0, 0.0, -1.5, 0.0, True)        # 6 m 밖
    res = M.compute_mask(_state(v_cmd=0.6, tracks=[slow, far]))
    assert not res['saturated'] and np.all(np.isinf(res['vo_time']))


def test_safety_block_none_means_unlimited():
    parts = OB.build_observation(_state(safety=SafetyState(C.ZONE_CLEAR, None, False, None)),
                                 return_parts=True)[1]
    assert list(parts['safety']) == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0]
    parts = OB.build_observation(_state(safety=SafetyState(C.ZONE_CRITICAL, 0.2, True, 1.0)),
                                 return_parts=True)[1]
    assert list(parts['safety']) == pytest.approx([0.0, 0.0, 1.0, 0.0, 0.1, 1.0, 0.2])


def test_goal_overshoot_exemption():
    r = empty_scan()
    for i in range(330, 391):
        ang = C.LIDAR_ANGLE_MIN + i * C.LIDAR_ANGLE_INC
        r[i] = (0.95 - C.LIDAR_X) / math.cos(ang)       # 벽 x = 0.95 (전면 0.65 m 앞)
    kw = dict(scan_ranges=r, v_cmd=0.5, prev_actions=[(0.0, 0.0)] * 3)
    straight = [i for i in range(C.N_CELLS) if K.cell_accel(i)[1] == 0.0]
    near_goal = M.compute_mask(_state(path=[(0.0, 0.0), (0.45, 0.0)], **kw))
    far_goal = M.compute_mask(_state(path=[(0.0, 0.0), (5.0, 0.0)], **kw))
    ok = [i for i in straight if near_goal['jerk_ok'][i]]
    assert ok and all(near_goal['fp_ok'][i] for i in ok)       # 목표(0.45)+0.1 너머는 안 본다
    assert not any(far_goal['fp_ok'][i] for i in ok)           # 목표가 멀면 벽에 닿는다
