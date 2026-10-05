"""
기하 기준 구현 — 각 함수는 C++ 원본의 의미를 그대로 옮긴다 (원본 파일:행 주석).

좌표 규약: 로봇 좌표계 = base_link (x 전방, y 좌측), 회전 중심 = 원점 (R:8).
지도 좌표계 자세 pose = (x, y, θ). 모든 함수는 numpy float64 로 계산한다.
"""

import math

import numpy as np

from . import constants as C


def wrap_angle(a):
    """각을 [−π, π) 로 감는다 (geometry.hpp wrapAngle 과 같은 의미)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def to_robot_frame(pose, xy):
    """지도 좌표 점들(N×2)을 로봇 좌표로 옮긴다."""
    x, y, th = pose
    pts = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    c, s = math.cos(th), math.sin(th)
    dx = pts[:, 0] - x
    dy = pts[:, 1] - y
    return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)


def rotate_to_robot(theta, vxy):
    """지도 좌표 벡터들(N×2)을 로봇 축으로 회전한다 (원점 이동 없음)."""
    v = np.asarray(vxy, dtype=np.float64).reshape(-1, 2)
    c, s = math.cos(theta), math.sin(theta)
    return np.stack([c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1]], axis=1)


def footprint_clearance(pts, half_l=C.HALF_L, half_w=C.HALF_W):
    """
    로봇 좌표 점들(N×2)에서 풋프린트 사각형까지의 거리 (안쪽 = 0).

    safety_gate footprintDistance 와 같은 정의 (풋프린트 모서리 기준, R:116-120).
    """
    p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    dx = np.maximum(np.abs(p[:, 0]) - half_l, 0.0)
    dy = np.maximum(np.abs(p[:, 1]) - half_w, 0.0)
    return np.hypot(dx, dy)


def integrate_arc(v, w, t):
    """
    원점·θ=0 에서 (v, w) 등속 원호를 t 초 적분한 자세 (x, y, θ).

    dwa.cpp rollout 의 integrateArc 와 같은 정확 적분 (오차 누적 없음).
    """
    if abs(w) < 1e-9:
        return v * t, 0.0, w * t
    th = w * t
    r = v / w
    return r * math.sin(th), r * (1.0 - math.cos(th)), th


def points_in_footprint(pts_robot, pose, pad=C.FP_PAD):
    """로봇 좌표 점들이 로봇 좌표계 자세 pose 의 풋프린트(+pad) 안에 있는가 (하나라도)."""
    if pts_robot.shape[0] == 0:
        return False
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx = pts_robot[:, 0] - x
    dy = pts_robot[:, 1] - y
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    inside = (np.abs(lx) <= C.HALF_L + pad) & (np.abs(ly) <= C.HALF_W + pad)
    return bool(np.any(inside))


def chord_velocity(theta, v, w, tau):
    """velocity_obstacle.cpp:18-27 chordVelocity — 원호의 τ 동안 평균 변위 속도."""
    half = 0.5 * w * tau
    mag = v
    if abs(half) > 1e-9:
        mag = v * math.sin(half) / half
    d = theta + half
    return mag * math.cos(d), mag * math.sin(d)


def vo_time(p, w_rel, radius, tau):
    """
    velocity_obstacle.cpp:46-65 velocityObstacleTime — VO 진입 시각, 없으면 +inf.

    p: 장애물 − 로봇 상대 위치, w_rel: 로봇 − 장애물 상대 속도, radius: 합성 반경.
    """
    px, py = p
    wx, wy = w_rel
    pn2 = px * px + py * py
    closing = px * wx + py * wy
    if pn2 < radius * radius:
        return 0.0 if closing > 0.0 else math.inf
    wn2 = wx * wx + wy * wy
    if wn2 < 1e-12 or closing <= 0.0:
        return math.inf
    disc = closing * closing - wn2 * (pn2 - radius * radius)
    if disc <= 0.0:
        return math.inf
    t = (closing - math.sqrt(disc)) / wn2
    return max(0.0, t) if t <= tau else math.inf


# ---------------------------------------------------------------- 경로
def path_arrays(path):
    """경로 점(N×2, 지도 좌표)과 누적 호길이."""
    pts = np.asarray(path, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] == 0:
        raise ValueError('빈 경로')
    seg = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    return pts, cum


def project_onto_path(path, p):
    """
    점 p 를 경로(폴리라인)에 투영한다 — 전역 최근접점, 동률이면 작은 호길이.

    반환: (s, dist, signed_e, seg_yaw). signed_e 는 경로 진행 방향 왼쪽이 +.
    점이 1 개뿐인 경로는 그 점까지의 거리, yaw 0.
    """
    pts, cum = path_arrays(path)
    px, py = float(p[0]), float(p[1])
    if pts.shape[0] == 1:
        d = math.hypot(px - pts[0, 0], py - pts[0, 1])
        return 0.0, d, d, 0.0
    best = None
    for i in range(pts.shape[0] - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        sx, sy = bx - ax, by - ay
        ln2 = sx * sx + sy * sy
        if ln2 < 1e-18:
            continue
        u = ((px - ax) * sx + (py - ay) * sy) / ln2
        u = min(1.0, max(0.0, u))
        qx, qy = ax + u * sx, ay + u * sy
        d = math.hypot(px - qx, py - qy)
        s = cum[i] + u * math.sqrt(ln2)
        if best is None or d < best[1] - 1e-12:
            cross = sx * (py - ay) - sy * (px - ax)
            sign = 1.0 if cross >= 0.0 else -1.0
            best = (s, d, sign * d, math.atan2(sy, sx))
    if best is None:   # 모든 구간 길이 0
        d = math.hypot(px - pts[0, 0], py - pts[0, 1])
        return 0.0, d, d, 0.0
    return best


def interpolate_at(path, s):
    """호길이 s 의 경로 점 (끝에서 자른다)."""
    pts, cum = path_arrays(path)
    if s <= 0.0:
        return float(pts[0, 0]), float(pts[0, 1])
    if s >= cum[-1]:
        return float(pts[-1, 0]), float(pts[-1, 1])
    i = int(np.searchsorted(cum, s, side='right') - 1)
    i = min(max(i, 0), pts.shape[0] - 2)
    ln = cum[i + 1] - cum[i]
    u = 0.0 if ln < 1e-18 else (s - cum[i]) / ln
    return (float(pts[i, 0] + u * (pts[i + 1, 0] - pts[i, 0])),
            float(pts[i, 1] + u * (pts[i + 1, 1] - pts[i, 1])))


def path_length(path):
    """경로 전체 호길이."""
    return float(path_arrays(path)[1][-1])


def remaining_arc(path, p):
    """d_arc = 경로 길이 − 투영 호길이 (퍼텐셜 Φ = −κ·d_arc 의 입력)."""
    s = project_onto_path(path, p)[0]
    return max(0.0, path_length(path) - s)


# ---------------------------------------------------------------- LiDAR
def beam_angles():
    """빔 i 의 각 = angle_min + i·inc (S:29-37, 빔 360 = 0°)."""
    return C.LIDAR_ANGLE_MIN + np.arange(C.LIDAR_SAMPLES) * C.LIDAR_ANGLE_INC


def scan_points(ranges):
    """
    유효 빔의 끝점(로봇 좌표, N×2)과 유효 여부(길이 720).

    유효 = 유한 && range_min ≤ r ≤ range_max. None/NaN/inf/범위 밖은 무반사로 본다.
    """
    r = np.array([np.nan if v is None else float(v) for v in ranges], dtype=np.float64)
    if r.shape[0] != C.LIDAR_SAMPLES:
        raise ValueError(f'빔 수 {r.shape[0]} != {C.LIDAR_SAMPLES}')
    valid = np.isfinite(r) & (r >= C.LIDAR_RANGE_MIN) & (r <= C.LIDAR_RANGE_MAX)
    ang = beam_angles()
    rr = np.where(valid, r, 0.0)
    pts = np.stack([C.LIDAR_X + rr * np.cos(ang), C.LIDAR_Y + rr * np.sin(ang)], axis=1)
    return pts, valid
