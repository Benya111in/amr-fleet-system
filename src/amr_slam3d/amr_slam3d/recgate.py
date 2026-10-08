"""
녹화 직후 bag 공백 검사 (rclpy 비의존) — 통과하지 못한 bag 은 재기록한다 (보너스 2 R 단계, replay.md §9-2·3·5).

왜: 파일럿 bag 은 sim 9.48–9.64 s 에 기록이 멈췄다 (`/clock` 틱 누락·EKF `/tf` 0.065·0.1 s 공백, 그 사이 점군은 기록됨).
그 bag 을 재생하면 RTAB-Map 이 9.6 s 장을 휠-EKF guess 조회 실패로 버렸고 재생 쪽에서는 고칠 수 없었다 (replay.md §7).
그래서 bag 마다 기록 직후 아래 규칙으로 판정하고, 실패하면 그 bag 을 쓰지 않고 다시 기록한다.

규칙 (사전 등록 — docs/research/slam3d/notebook/entries/*_b2-rec_*.md):
  1. 간격: 검사 토픽마다 기록된 sim 스탬프(정수 ns)의 이웃 간격이 **1.5 주기 + 1 µs** 를 넘으면 실패. 같은 스탬프 반복·
     역행도 실패. 주기는 명목값 (GATE_PERIODS_NS). 1 µs 허용은 부동소수 경계 오차 때문이다 — 파일럿 EKF 의 0.03 s
     (정확히 1.5 주기) 간격을 실수로 세면 53 개, 정수로 세면 11 개였다 (runs/b24/b2-rec/explore/pilot_gaps2.json).
  2. 덮음: 검사 토픽의 첫·끝 스탬프가 `/clock` 의 첫·끝과 max(1.5 주기, 0.5 s) 안. (토픽이 중간에 끊겨 끝나면 1 로는
     안 잡힌다.)
  3. 정지 구간: 정답 자세(ground_truth/odom)로 bag 첫 표본부터 처음 움직인 표본까지 ≥ 10 s, 마지막으로 움직인 표본
     뒤 bag 끝까지 ≥ 0.5 s. "움직임" = 첫(끝) 자세에서 xy 0.005 m 또는 yaw 0.2° 넘게 벗어남.
  4. (선택) 출발 시각: depart_at 을 주면 처음 움직인 sim 시각이 [depart_at, depart_at + 0.5 s] 안 (S6 변형끼리 actor
     위치를 맞추는 짝 비교용).
진단 (판정에 넣지 않음): bag 기록 순서상 직전 `/clock` 대비 센서 스탬프가 앞선 정도(clock lead), 점군 원본·전처리 수 차.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

ROBOT = 'amr_01'
EKF_TF = 'ekf_tf'          # /tf 안의 <robot>/odom → <robot>/base_footprint (휠-EKF 출력)
#: 검사 토픽 → 명목 주기 [ns]. /clock 은 물리 스텝(1 kHz, 파일럿 bag 178 618 개 / 178.8 s [측정])
GATE_PERIODS_NS: Dict[str, int] = {
    '/clock': 1_000_000,
    EKF_TF: 20_000_000,
    f'/{ROBOT}/lidar3d/points_raw': 100_000_000,
    f'/{ROBOT}/lidar3d/points': 100_000_000,
    f'/{ROBOT}/imu/data_raw': 5_000_000,
    f'/{ROBOT}/imu/data': 5_000_000,
}
GAP_FACTOR = 1.5
TOL_NS = 1_000
COVER_MIN_NS = 500_000_000
START_STILL_S = 10.0
END_STILL_S = 0.5
MOVE_XY = 0.005             # [m]
MOVE_YAW = math.radians(0.2)
DEPART_WINDOW_S = 0.5
#: clock lead 진단 대상 (센서)
LEAD_TOPICS = (f'/{ROBOT}/lidar3d/points_raw', f'/{ROBOT}/lidar3d/points',
               f'/{ROBOT}/imu/data_raw', f'/{ROBOT}/imu/data')


def interval_gaps(stamps_ns: Sequence[int], period_ns: int, factor: float = GAP_FACTOR,
                  tol_ns: int = TOL_NS, keep: int = 20) -> Dict[str, object]:
    """
    기록 순서 그대로의 스탬프 열 → 간격 검사 결과.

    gap = 이웃 간격 > factor·period + tol (정수 ns). dup = 간격 0, backwards = 간격 < 0. ok = n ≥ 2 이고 셋 다 0.
    gaps 는 처음 keep 개만 (시작 시각 s, 간격 s).
    """
    s = np.asarray(stamps_ns, dtype=np.int64)
    n = int(len(s))
    out: Dict[str, object] = {'n': n, 'period_s': period_ns * 1e-9,
                              'threshold_s': (int(factor * period_ns) + tol_ns) * 1e-9}
    if n < 2:
        out.update({'n_gap': 0, 'dup': 0, 'backwards': 0, 'dt_max_s': math.nan, 'gaps': [],
                    'ok': False, 'why': f'표본 {n} 개'})
        return out
    d = np.diff(s)
    thr = int(factor * period_ns) + tol_ns
    big = np.nonzero(d > thr)[0]
    dup = int(np.count_nonzero(d == 0))
    back = int(np.count_nonzero(d < 0))
    out.update({'first_s': int(s[0]) * 1e-9, 'last_s': int(s[-1]) * 1e-9,
                'n_gap': int(len(big)), 'dup': dup, 'backwards': back,
                'dt_max_s': int(d.max()) * 1e-9,
                'gaps': [(round(int(s[i]) * 1e-9, 6), round(int(d[i]) * 1e-9, 6))
                         for i in big[:keep]],
                'ok': len(big) == 0 and dup == 0 and back == 0})
    if not out['ok']:
        out['why'] = f'간격 초과 {len(big)}, 반복 {dup}, 역행 {back}'
    return out


def coverage(first_ns: int, last_ns: int, ref_first_ns: int, ref_last_ns: int,
             period_ns: int) -> Dict[str, object]:
    """토픽 구간이 기준(/clock) 구간을 덮는가 — 앞·뒤 모자람이 max(1.5 주기, 0.5 s) 이하."""
    lim = max(int(GAP_FACTOR * period_ns), COVER_MIN_NS)
    head, tail = first_ns - ref_first_ns, ref_last_ns - last_ns
    return {'head_s': head * 1e-9, 'tail_s': tail * 1e-9, 'limit_s': lim * 1e-9,
            'ok': head <= lim and tail <= lim}


def _moved(x: np.ndarray, y: np.ndarray, yaw: np.ndarray, i0: int) -> np.ndarray:
    dyaw = np.abs((yaw - yaw[i0] + math.pi) % (2 * math.pi) - math.pi)
    return (np.hypot(x - x[i0], y - y[i0]) > MOVE_XY) | (dyaw > MOVE_YAW)


def still_spans(t: Sequence[float], x: Sequence[float], y: Sequence[float],
                yaw: Sequence[float]) -> Dict[str, float]:
    """
    정답 자세 열 → bag 처음 정지 길이·끝 정지 길이 [s]와 처음 움직인 시각.

    처음 정지 = 첫 자세에서 벗어난 첫 표본 시각 − 첫 표본 시각 (끝까지 안 움직이면 전체 길이).
    끝 정지 = 끝 표본 시각 − 끝 자세와 다른 마지막 표본 시각.
    """
    t = np.asarray(t, dtype=float)
    x, y, yaw = (np.asarray(v, dtype=float) for v in (x, y, yaw))
    if len(t) < 2:
        return {'start_still_s': 0.0, 'end_still_s': 0.0, 'depart_t': math.nan,
                'arrive_t': math.nan}
    m0 = _moved(x, y, yaw, 0)
    m1 = _moved(x, y, yaw, len(t) - 1)
    i_dep = int(np.argmax(m0)) if m0.any() else len(t) - 1
    i_arr = int(len(t) - 1 - np.argmax(m1[::-1])) if m1.any() else 0
    return {'start_still_s': float(t[i_dep] - t[0]), 'end_still_s': float(t[-1] - t[i_arr]),
            'depart_t': float(t[i_dep]) if m0.any() else math.nan,
            'arrive_t': float(t[i_arr]) if m1.any() else math.nan}


def clock_lead(order: Sequence[Tuple[str, int]], topics: Sequence[str] = LEAD_TOPICS
               ) -> Dict[str, Dict[str, float]]:
    """
    기록 순서 (토픽, 스탬프 ns) 열 → 토픽별 '스탬프 − 기록 순서상 직전 /clock' 의 최대·p99 [s] (진단).

    파일럿 bag 의 정체 때 점군 0.12 s, IMU 0.16 s 앞섰다 (평소 p99 ≤ 0.004 s) [측정 explore/pilot_gaps.json].
    """
    last = None
    lead: Dict[str, List[int]] = {tp: [] for tp in topics}
    for topic, st in order:
        if topic == '/clock':
            last = st
        elif topic in lead and last is not None:
            lead[topic].append(st - last)
    out = {}
    for tp, v in lead.items():
        if v:
            a = np.asarray(v, dtype=np.int64) * 1e-9
            out[tp] = {'n': int(len(a)), 'max_s': float(a.max()),
                       'p99_s': float(np.percentile(a, 99))}
    return out


def evaluate(series: Dict[str, Sequence[int]], gt: Optional[Tuple[Sequence[float], ...]],
             depart_at: Optional[float] = None,
             periods: Optional[Dict[str, int]] = None,
             start_still_s: float = START_STILL_S,
             end_still_s: float = END_STILL_S) -> Dict[str, object]:
    """
    검사 토픽 스탬프 열(기록 순서, 정수 ns)과 정답 자세 (t, x, y, yaw) → 판정 dict ('pass', 'reasons', 토픽별 결과).

    series 에 없는 검사 토픽은 실패다 (기록 토픽 목록이 바뀌었으면 조용히 통과시키지 않는다).
    """
    periods = dict(GATE_PERIODS_NS if periods is None else periods)
    reasons: List[str] = []
    topics: Dict[str, object] = {}
    clk = series.get('/clock')
    ref = (int(min(clk)), int(max(clk))) if clk is not None and len(clk) else None
    if ref is None:
        reasons.append('/clock 없음')
    for tp, per in periods.items():
        st = series.get(tp)
        if st is None or len(st) == 0:
            topics[tp] = {'n': 0, 'ok': False, 'why': '토픽 없음'}
            reasons.append(f'{tp}: 없음')
            continue
        r = interval_gaps(st, per)
        if ref is not None:
            r['cover'] = coverage(int(min(st)), int(max(st)), ref[0], ref[1], per)
            if not r['cover']['ok']:
                r['ok'] = False
                reasons.append(f"{tp}: 덮음 부족 앞 {r['cover']['head_s']:.3f} s "
                               f"뒤 {r['cover']['tail_s']:.3f} s")
        if r.get('why'):
            reasons.append(f"{tp}: {r['why']} (처음 {r['gaps'][:3]})")
        topics[tp] = r
    proto: Dict[str, object] = {}
    if gt is None or len(gt[0]) < 2:
        reasons.append('정답 자세 없음')
    else:
        sp = still_spans(*gt)
        proto.update(sp)
        proto['start_ok'] = sp['start_still_s'] >= start_still_s
        proto['end_ok'] = sp['end_still_s'] >= end_still_s
        if not proto['start_ok']:
            reasons.append(f"처음 정지 {sp['start_still_s']:.2f} s < {start_still_s:g} s")
        if not proto['end_ok']:
            reasons.append(f"끝 정지 {sp['end_still_s']:.2f} s < {end_still_s:g} s")
        if depart_at is not None:
            dep = sp['depart_t']
            proto['depart_at'] = depart_at
            proto['depart_ok'] = (not math.isnan(dep)) and \
                depart_at - 1e-6 <= dep <= depart_at + DEPART_WINDOW_S
            if not proto['depart_ok']:
                hi = depart_at + DEPART_WINDOW_S
                reasons.append(f'출발 {dep:.3f} s ∉ [{depart_at:g}, {hi:g}]')
    return {'pass': not reasons, 'reasons': reasons, 'topics': topics, 'protocol': proto,
            'rule': {'gap_factor': GAP_FACTOR, 'tol_ns': TOL_NS,
                     'periods_s': {k: v * 1e-9 for k, v in periods.items()},
                     'cover_min_s': COVER_MIN_NS * 1e-9, 'start_still_s': start_still_s,
                     'end_still_s': end_still_s, 'move_xy_m': MOVE_XY,
                     'move_yaw_deg': math.degrees(MOVE_YAW),
                     'depart_window_s': DEPART_WINDOW_S}}
