"""
bag·토픽 메시지 스탬프 통계 (rclpy 비의존) — 파일럿 bag 메타데이터(메시지 수·주기)와 재생 누락 판정에 쓴다.

스탬프는 header.stamp (시뮬레이션 시각) 를 쓴다. 기록 시각(bag 수신 시각)은 벽시계라 RTF 에 따라 흔들리므로
주기는 header 기준으로 낸다. header 가 없는 형(TFMessage 등)은 수신 시각만 센다.
"""

from __future__ import annotations

import struct
from typing import Dict, List, Optional, Sequence

import numpy as np


def cdr_header_stamp(raw: bytes) -> float:
    """
    직렬화된 메시지(첫 필드가 std_msgs/Header)의 header.stamp [s].

    CDR: 4 B 캡슐화 머리(리틀엔디언 0x0001) + int32 sec + uint32 nanosec. 큰 점군을 역직렬화하지 않고 센다.
    """
    if len(raw) < 12 or raw[0] != 0 or raw[1] != 1:
        raise ValueError('CDR 리틀엔디언 Header 메시지가 아니다')
    sec, nsec = struct.unpack_from('<iI', raw, 4)
    return sec + nsec * 1e-9


def interval_stats(stamps: Sequence[float],
                   nominal_hz: Optional[float] = None) -> Dict[str, float]:
    """
    스탬프 열 → 개수·구간·주기·간격 통계.

    rate_hz = (n − 1) / (마지막 − 처음). 간격 중앙값의 1.5 배를 넘는 간격을 gaps, 0.5 배 미만을 bursts 로 센다
    (nominal_hz 를 주면 1/nominal 기준). 같은 스탬프 반복(dup)과 역행(backwards)도 센다.
    """
    n = len(stamps)
    out: Dict[str, float] = {'count': float(n)}
    if n == 0:
        return out
    t = np.asarray(stamps, dtype=float)
    out.update({'first': float(t[0]), 'last': float(t[-1]), 'span_s': float(t[-1] - t[0])})
    if n < 2:
        return out
    d = np.diff(t)
    span = float(t[-1] - t[0])
    if nominal_hz:
        ref = 1.0 / nominal_hz
    else:
        ref = float(np.median(d[d > 0])) if np.any(d > 0) else 0.0
    out.update({
        'rate_hz': (n - 1) / span if span > 0 else float('nan'),
        'dt_median': float(np.median(d)),
        'dt_min': float(d.min()),
        'dt_max': float(d.max()),
        'gaps': float(np.count_nonzero(d > 1.5 * ref)) if ref > 0 else 0.0,
        'bursts': float(np.count_nonzero((d > 0) & (d < 0.5 * ref))) if ref > 0 else 0.0,
        'dup': float(np.count_nonzero(d == 0)),
        'backwards': float(np.count_nonzero(d < 0)),
    })
    return out


def matched(input_stamps: Sequence[float], output_stamps: Sequence[float],
            tol: float = 1e-6) -> Dict[str, float]:
    """
    입력 스탬프 중 출력에 같은 스탬프(±tol)가 있는 것의 수 — "처리 스캔 누락 0" 판정.

    RTAB-Map icp_odometry 는 처리한 스캔마다 같은 header.stamp 의 odom 을 낸다. missing = 입력 − 대응.
    """
    out_sorted = np.sort(np.asarray(output_stamps, dtype=float))
    hit = 0
    missing: List[float] = []
    for s in input_stamps:
        i = int(np.searchsorted(out_sorted, s - tol))
        if i < len(out_sorted) and abs(out_sorted[i] - s) <= tol:
            hit += 1
        else:
            missing.append(float(s))
    return {'input': float(len(input_stamps)), 'output': float(len(output_stamps)),
            'matched': float(hit), 'missing': float(len(missing)),
            'first_missing': missing[:5]}
