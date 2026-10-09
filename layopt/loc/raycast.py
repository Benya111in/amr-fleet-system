"""
2D LiDAR 가상 스캔 (벡터화) — docs/research/state-estimation/checks/alias_margin.py raycast 와 같은 결과.

참조 구현은 빔마다 r = 0 에서 step = res/3 씩 더해 가며(r += step) 셀 floor((l + r·d − o)/res) 이
점유(≥ 65)면 멈추고, 지도를 벗어나면 무한(빔 없음)으로 둔다. 잡음은 유한한 빔에만, 빔 순서대로
rng.normal(0, σ) 를 더한다. 여기서는 같은 r 수열(np.add.accumulate 는 순차 덧셈)과 같은 math.cos/sin
방향을 써서 빔 전체를 한 번에 진행한다 — 참조와 거리값이 비트 단위로 같아야 한다 (test_e0.py 가 확인).
이웃 로봇은 레이캐스트 세계에만 넣는다 (overlay 점유 셀). 점수에 쓰는 기준 지도에는 넣지 않는다.
"""

import math
from typing import Optional, Sequence, Tuple

import numpy as np

LIDAR = (0.15, 0.0, 0.0)      # base_footprint → lidar 2D (config/sensors.yaml extrinsic)
RANGE_MAX = 25.0
N_BEAMS = 720
NOISE = 0.03
ROBOT_L, ROBOT_W = 0.60, 0.40   # config/robot_params.yaml footprint


def occupied_mask(occ: np.ndarray, extra: Optional[Sequence[Tuple[np.ndarray, np.ndarray]]] = None,
                  thresh: int = 65) -> np.ndarray:
    """레이캐스트 세계: 점유(≥ thresh) + 덧씌운 셀들(이웃 로봇)."""
    mask = occ >= thresh
    if extra:
        mask = mask.copy()
        for rr, cc in extra:
            mask[rr, cc] = True
    return mask


def raycast(occupied: np.ndarray, spec, pose, n: int = N_BEAMS, rmax: float = RANGE_MAX,
            noise: float = NOISE, seed: int = 0, lidar=LIDAR):
    """360° 스캔 (angle_min = −π). 반환 (ranges list, angle_min, angle_increment)."""
    res = spec.resolution
    bx, by, byaw = pose
    lx = bx + math.cos(byaw) * lidar[0] - math.sin(byaw) * lidar[1]
    ly = by + math.sin(byaw) * lidar[0] + math.cos(byaw) * lidar[1]
    lyaw = byaw + lidar[2]
    amin, inc = -math.pi, 2.0 * math.pi / n
    step = res / 3.0
    # 참조의 r 수열: 0, step, step+step, … (r < rmax 인 동안)
    count = int(math.ceil(rmax / step)) + 2
    rs = np.add.accumulate(np.concatenate([[0.0], np.full(count, step)]))
    rs = rs[rs < rmax]
    dx = np.array([math.cos(lyaw + amin + k * inc) for k in range(n)])
    dy = np.array([math.sin(lyaw + amin + k * inc) for k in range(n)])
    gx = np.floor((lx + rs[None, :] * dx[:, None] - spec.origin_x) / res)
    gy = np.floor((ly + rs[None, :] * dy[:, None] - spec.origin_y) / res)
    inb = (gx >= 0) & (gx < spec.width) & (gy >= 0) & (gy < spec.height)
    ci = np.where(inb, gx, 0).astype(np.int64)
    ri = np.where(inb, gy, 0).astype(np.int64)
    hit = inb & occupied[ri, ci]
    stop = hit | ~inb
    first = np.argmax(stop, axis=1)
    any_stop = stop[np.arange(n), first]
    is_hit = any_stop & hit[np.arange(n), first]
    out = np.where(is_hit, rs[first], np.inf)
    if noise:
        rng = np.random.default_rng(seed)
        finite = np.isfinite(out)
        out[finite] = out[finite] + rng.normal(0.0, noise, size=int(finite.sum()))
    return out.tolist(), amin, inc
