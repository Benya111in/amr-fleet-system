"""
경로·주행 지표 (rclpy 비의존): 경로 길이, 여유거리, CTE, 주행 시간 예측, 저크.

주행 시간 예측은 C++ core::SpeedProfile::predictTravelTime 과 같은 식이다
(include/amr_navigation/core/speed_profile.hpp):
  곡률 상한 v_cap,i = min(v_des, ω_max/|κ_i|, √(a_lat/|κ_i|)), 끝점 0,
  전진/후진 패스 v_i = min(v_cap,i, √(v_{i−1}² + 2aΔs), √(v_{i+1}² + 2dΔs)),
  T = Σ 2Δs/(v_i + v_{i+1}) + a/j + τ_rot(출발) + τ_rot(도착).
"""
from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np


@dataclass
class ProfileLimits:
    """주행 시간 예측 한계 (robot_params.yaml limits 와 제어기 설정)."""

    desired_speed: float = 1.0
    max_accel: float = 1.0
    max_decel: float = 1.0
    max_angular_vel: float = 1.5
    max_angular_accel: float = 2.0
    max_lateral_accel: float = 0.8
    max_jerk: float = 2.0
    curvature_window: float = 0.25


def path_length(xy: np.ndarray) -> float:
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))))


def cumulative_length(xy: np.ndarray) -> np.ndarray:
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) == 0:
        return np.zeros(0)
    return np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))])


def min_clearance(xy: np.ndarray, dist: np.ndarray, resolution: float,
                  origin: Sequence[float], step: float = 0.025) -> float:
    """경로(선분 보간, step 간격)가 지나는 셀 중 거리장(dist [m]) 최솟값 = 로봇 중심 여유거리."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) == 0:
        return float('nan')
    pts = [xy[:1]]
    for a, b in zip(xy[:-1], xy[1:]):
        n = max(1, int(math.ceil(math.hypot(*(b - a)) / step)))
        t = np.linspace(0.0, 1.0, n + 1)[1:, None]
        pts.append(a + t * (b - a))
    p = np.vstack(pts)
    ix = np.clip(np.floor((p[:, 0] - origin[0]) / resolution).astype(int), 0, dist.shape[1] - 1)
    iy = np.clip(np.floor((p[:, 1] - origin[1]) / resolution).astype(int), 0, dist.shape[0] - 1)
    return float(np.min(dist[iy, ix]))


def cross_track_error(xy_path: np.ndarray, x: float, y: float) -> float:
    """폴리라인 최근접 선분까지 부호 있는 수직 거리 (진행 방향 좌측 +)."""
    p = np.asarray(xy_path, dtype=float).reshape(-1, 2)
    if len(p) < 2:
        return float('nan')
    a = p[:-1]
    d = p[1:] - a
    len2 = np.maximum(np.sum(d * d, axis=1), 1e-12)
    t = np.clip(((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1]) / len2, 0.0, 1.0)
    qx = a[:, 0] + t * d[:, 0]
    qy = a[:, 1] + t * d[:, 1]
    dist2 = (x - qx) ** 2 + (y - qy) ** 2
    i = int(np.argmin(dist2))
    cross = d[i, 0] * (y - a[i, 1]) - d[i, 1] * (x - a[i, 0])
    return float(math.copysign(math.sqrt(dist2[i]), cross))


def discrete_curvature(xy: np.ndarray, window: float = 0.25) -> np.ndarray:
    """정점별 곡률 [1/m]: 앞뒤 window 떨어진 보간점으로 잰 회전각 / window (양 끝 0)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    n = len(xy)
    kappa = np.zeros(n)
    if n < 3:
        return kappa
    s = cumulative_length(xy)
    for i in range(1, n - 1):
        s0 = max(0.0, s[i] - window)
        s1 = min(s[-1], s[i] + window)
        a = np.array([np.interp(s0, s, xy[:, 0]), np.interp(s0, s, xy[:, 1])])
        c = np.array([np.interp(s1, s, xy[:, 0]), np.interp(s1, s, xy[:, 1])])
        b = xy[i]
        if np.hypot(*(b - a)) < 1e-9 or np.hypot(*(c - b)) < 1e-9:
            continue
        h0 = math.atan2(b[1] - a[1], b[0] - a[0])
        h1 = math.atan2(c[1] - b[1], c[0] - b[0])
        dh = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
        kappa[i] = dh / (0.5 * (s1 - s0))
    return kappa


def rotation_time(angle: float, w_max: float, alpha_max: float) -> float:
    angle = abs(angle)
    if angle < 1e-9 or w_max <= 0 or alpha_max <= 0:
        return 0.0
    if angle >= w_max * w_max / alpha_max:
        return angle / w_max + w_max / alpha_max
    return 2.0 * math.sqrt(angle / alpha_max)


def predict_travel_time(xy: np.ndarray, limits: Optional[ProfileLimits] = None, v0: float = 0.0,
                        start_heading_error: float = 0.0,
                        goal_heading_error: float = 0.0) -> float:
    """곡률·가감속·저크를 반영한 주행 시간 예측 [s] (C++ SpeedProfile 과 같은 식)."""
    lim = limits or ProfileLimits()
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    rot = (rotation_time(start_heading_error, lim.max_angular_vel, lim.max_angular_accel)
           + rotation_time(goal_heading_error, lim.max_angular_vel, lim.max_angular_accel))
    if len(xy) < 2:
        return rot
    s = cumulative_length(xy)
    k = np.abs(discrete_curvature(xy, lim.curvature_window))
    cap = np.full(len(xy), lim.desired_speed)
    nz = k > 1e-6
    if lim.max_angular_vel > 0:
        cap[nz] = np.minimum(cap[nz], lim.max_angular_vel / k[nz])
    if lim.max_lateral_accel > 0:
        cap[nz] = np.minimum(cap[nz], np.sqrt(lim.max_lateral_accel / k[nz]))
    cap[-1] = 0.0
    v = cap.copy()
    v[0] = min(v[0], max(0.0, v0))
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * lim.max_accel * (s[i] - s[i - 1])))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * lim.max_decel * (s[i + 1] - s[i])))
    ds = np.diff(s)
    vs = np.maximum(v[:-1] + v[1:], 1e-3)
    t = float(np.sum(np.where(ds > 0, 2.0 * ds / vs, 0.0)))
    if lim.max_jerk > 0:
        t += lim.max_accel / lim.max_jerk
    return t + rot


def jerk_from_velocity(t: np.ndarray, v: np.ndarray, smooth: int = 1) -> np.ndarray:
    """속도 시계열 → 저크 [m/s³] (2차 차분; smooth > 1 이면 이동평균 후). 길이 len(v) − 2."""
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    if smooth > 1 and len(v) >= smooth:
        kernel = np.ones(smooth) / smooth
        v = np.convolve(v, kernel, mode='same')
    if len(v) < 3:
        return np.zeros(0)
    a = np.diff(v) / np.maximum(np.diff(t), 1e-9)
    tm = 0.5 * (t[1:] + t[:-1])
    return np.diff(a) / np.maximum(np.diff(tm), 1e-9)
