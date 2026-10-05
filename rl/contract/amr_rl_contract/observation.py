"""
관측 벡터 기준 구현 (contract.md §4). 길이 OBS_DIM = 291, float32.

블록 순서는 OBS_LAYOUT 이 정한다. 정규화(아래 상수로 나누기·자르기)는 이 함수가 하고,
학습 중 추정한 running mean/std 정규화는 ONNX 모델 안에 넣는다 (onnx_io.md).
"""

import math

import numpy as np

from . import constants as C
from . import geometry as G

OBS_LAYOUT = (
    ('lidar_sectors', C.N_SECTORS),        # 180
    ('ego', 6 + 2 * C.N_PREV_ACTIONS),     # 12
    ('timing', 3),                         # scan_age, track_age, v_cap
    ('path', 3 * C.N_SUBGOALS + 3),        # 18
    ('goal', 4),
    ('safety', 7),                         # zone one-hot 4, speed_limit, proximity_stop, min_ttc
    ('dev', 3),
    ('tracks', C.K_TRACKS * C.TRACK_FEATS),  # 64
)
OBS_DIM = sum(n for _, n in OBS_LAYOUT)


def obs_slices():
    """블록 이름 → slice."""
    out, i = {}, 0
    for name, n in OBS_LAYOUT:
        out[name] = slice(i, i + n)
        i += n
    return out


def _clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _inf(v):
    return math.inf if v is None else float(v)


def lidar_sectors(scan_ranges):
    """
    720 빔 → 180 섹터: 섹터 k = 빔 4k..4k+3 의 풋프린트 여유거리 최솟값, [0, 1] 로 정규화.

    여유거리 = 빔 끝점(로봇 좌표)에서 풋프린트 사각형까지 거리 (안쪽 0).
    유효 빔이 없는 섹터 = SECTOR_CLIP. 섹터 k 의 각 범위 = [−180° + 2k, −178.5° + 2k].
    """
    pts, valid = G.scan_points(scan_ranges)
    clr = np.where(valid, G.footprint_clearance(pts), C.SECTOR_CLIP)
    clr = np.minimum(clr, C.SECTOR_CLIP)
    sec = clr.reshape(C.N_SECTORS, C.BEAMS_PER_SECTOR).min(axis=1)
    return sec / C.SECTOR_CLIP


def track_ttc(p, v_obs_robot, v_meas):
    """
    관측용 트랙 TTC [s] — 로봇이 전방 직진 v_meas, 장애물 등속일 때 원(외접 + 0.30) 첫 접촉.

    추적기 TTC(path 추종 예측·σ 팽창)를 쓰지 않는다: 모든 환경에서 같은 함수로 계산하려고.
    """
    w_rel = (v_meas - v_obs_robot[0], -v_obs_robot[1])
    return G.vo_time(p, w_rel, C.ROBOT_RADIUS + C.OBSTACLE_RADIUS, C.TTC_HORIZON)


def select_tracks(state, max_range=C.TRACK_RANGE):
    """
    관측·VO 에 쓸 트랙을 고른다.

    DWA 플러그인과 같은 선별 (is_dynamic 이거나 속력 ≥ 0.5, dwa_controller.cpp:341),
    트랙 나이만큼 등속 외삽 (dwa_controller.cpp:343-349), 로봇 좌표, 거리 ≤ max_range, (거리, id) 순.

    반환: [(dist, id, p_robot(2,), v_robot(2,), is_dynamic, speed)] 정렬된 목록.
    """
    out = []
    th = state.pose[2]
    for t in state.tracks:
        speed = math.hypot(t.vx, t.vy)
        if not t.is_dynamic and speed < C.DYN_FAST_SPEED:
            continue
        px = t.x + t.vx * max(0.0, state.track_age)
        py = t.y + t.vy * max(0.0, state.track_age)
        p = G.to_robot_frame(state.pose, [(px, py)])[0]
        v = G.rotate_to_robot(th, [(t.vx, t.vy)])[0]
        d = float(math.hypot(p[0], p[1]))
        if d > max_range:
            continue
        out.append((d, t.id, p, v, t.is_dynamic, speed))
    out.sort(key=lambda r: (r[0], r[1]))
    return out


def build_observation(state, return_parts=False):
    """State → 관측 (float32, OBS_DIM). return_parts 면 블록별 float64 사전도 돌려준다."""
    parts = {}
    parts['lidar_sectors'] = lidar_sectors(state.scan_ranges)

    ego = [state.v_meas / C.V_MAX_HW, state.w_meas / C.W_MAX,
           state.a_v_meas / C.A_MAX, state.a_w_meas / C.ALPHA_MAX,
           state.v_cmd / C.V_MAX_HW, state.w_cmd / C.W_MAX]
    prev = list(state.prev_actions)[:C.N_PREV_ACTIONS]
    prev += [(0.0, 0.0)] * (C.N_PREV_ACTIONS - len(prev))
    for a_v, a_w in prev:
        ego += [a_v / C.A_MAX, a_w / C.ALPHA_MAX]
    parts['ego'] = np.clip(np.array(ego, dtype=np.float64), -3.0, 3.0)

    parts['timing'] = np.array([
        _clip(state.scan_age / C.SCAN_AGE_NORM, 0.0, 1.0),
        _clip(state.track_age / C.TRACK_AGE_NORM, 0.0, 1.0),
        _clip(state.v_cap / C.V_MAX_HW, 0.0, 1.0)])

    x, y, th = state.pose
    s0, _dist, e_signed, seg_yaw = G.project_onto_path(state.path, (x, y))
    path_blk = []
    for k in range(1, C.N_SUBGOALS + 1):
        gx, gy = G.interpolate_at(state.path, s0 + C.SUBGOAL_SPACING * k)
        q = G.to_robot_frame(state.pose, [(gx, gy)])[0]
        d = math.hypot(q[0], q[1])
        b = math.atan2(q[1], q[0]) if d > 1e-9 else 0.0
        path_blk += [_clip(d / C.SUBGOAL_DIST_NORM, 0.0, 2.0), math.sin(b), math.cos(b)]
    e_psi = G.wrap_angle(th - seg_yaw)
    path_blk += [_clip(e_signed / C.DEV_PEAK_NORM, -2.0, 2.0), math.sin(e_psi), math.cos(e_psi)]
    parts['path'] = np.array(path_blk)

    d_arc = max(0.0, G.path_length(state.path) - s0)
    g = G.to_robot_frame(state.pose, [state.path[-1]])[0]
    dg = math.hypot(g[0], g[1])
    bg = math.atan2(g[1], g[0]) if dg > 1e-9 else 0.0
    parts['goal'] = np.array([min(d_arc, C.GOAL_CLIP) / C.GOAL_CLIP,
                              min(dg, C.GOAL_CLIP) / C.GOAL_CLIP, math.sin(bg), math.cos(bg)])

    zone = np.zeros(4)
    zone[int(state.safety.zone)] = 1.0
    sl = _inf(state.safety.speed_limit)
    mt = _inf(state.safety.min_ttc)
    parts['safety'] = np.concatenate([zone, [
        min(sl, C.V_MAX_HW) / C.V_MAX_HW,
        1.0 if state.safety.proximity_stop else 0.0,
        min(mt, C.TTC_HORIZON) / C.TTC_HORIZON]])

    if state.dev.in_dev:
        parts['dev'] = np.array([1.0, _clip(state.dev.peak / C.DEV_PEAK_NORM, 0.0, 2.0),
                                 _clip(state.dev.t_since_peak / C.DEV_TIMER_NORM, 0.0, 2.0)])
    else:
        parts['dev'] = np.zeros(3)

    trk = np.zeros((C.K_TRACKS, C.TRACK_FEATS))
    for i, (d, _id, p, v, dyn, _sp) in enumerate(select_tracks(state)[:C.K_TRACKS]):
        ttc = track_ttc(p, v, state.v_meas)
        trk[i] = [p[0] / C.TRACK_POS_NORM, p[1] / C.TRACK_POS_NORM,
                  v[0] / C.TRACK_VEL_NORM, v[1] / C.TRACK_VEL_NORM,
                  d / C.TRACK_POS_NORM, min(ttc, C.TTC_HORIZON) / C.TTC_HORIZON,
                  1.0 if dyn else 0.0, 1.0]
    parts['tracks'] = trk.reshape(-1)

    obs = np.concatenate([parts[name] for name, _ in OBS_LAYOUT]).astype(np.float32)
    assert obs.shape == (OBS_DIM,), obs.shape
    if return_parts:
        return obs, parts
    return obs
