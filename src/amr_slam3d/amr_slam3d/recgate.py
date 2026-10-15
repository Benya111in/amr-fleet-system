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

녹화 결함 검사 (b2-rerec, 2026-10-16 — 재녹화 결과 전에 더함, 사전 등록 notebook/entries/*_b2-rerec_*.md):
  5. (선택, 런치 로그를 주면) 기동 바이어스: imu_filter_node 의 첫 기동 결과 줄이 성공 문구(BIAS_OK_*)이고 표본
     ≥ BIAS_MIN_SAMPLES, "falling back to parameter bias" 가 어디에도 없다. 녹화 3/8 bag 이 이 줄 없이
     기동 0.1 s(sim) 안에 파라미터 바이어스(0)로 떨어져 EKF odom 이 0.01 rad/s 로 yaw 표류했다
     (bonus2_results.md §0-2).
  6. (선택, EKF odom 을 주면) yaw 표류율: EKF odom yaw 와 정답 yaw 를 첫 표본에서 맞춘 오차의 일차 기울기
     |rate| < DRIFT_MAX_RAD_S (0.002 rad/s). 결함 bag 은 0.0100, 결함 없는 bag 은 ≤ 0.0004 였다
     [측정 runs/b24/b2-eval/supp/odom_v2/odom_drift.json — 같은 정의, scripts/odom_drift.py].
  정착 판정 imu_still (settle_wait.py 가 쓴다 — 위치추정 스택을 로봇이 정지한 뒤에 띄우려고).
"""

from __future__ import annotations

import math
import re
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
             end_still_s: float = END_STILL_S,
             bias_log: Optional[str] = None,
             odom: Optional[Tuple[Sequence[float], Sequence[float]]] = None
             ) -> Dict[str, object]:
    """
    검사 토픽 스탬프 열(기록 순서, 정수 ns)과 정답 자세 (t, x, y, yaw) → 판정 dict ('pass', 'reasons', 토픽별 결과).

    series 에 없는 검사 토픽은 실패다 (기록 토픽 목록이 바뀌었으면 조용히 통과시키지 않는다).
    bias_log (런치 로그 텍스트) 를 주면 규칙 5 (기동 바이어스), odom (EKF odom 의 t, yaw) 를 주면 규칙 6 (yaw 표류율)을
    더한다 — 결과 dict 의 'bias'·'odom_drift'. 주지 않으면 예전 규칙 1–4 그대로다.
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
    extra: Dict[str, object] = {}
    rule_extra: Dict[str, object] = {}
    if bias_log is not None:
        b = bias_outcome(bias_log)
        extra['bias'] = b
        rule_extra['bias_min_samples'] = BIAS_MIN_SAMPLES
        if not b['ok']:
            reasons.append(f"기동 바이어스: {b.get('why')}")
    if odom is not None:
        if gt is None or len(gt[0]) < 2:
            d: Dict[str, object] = {'ok': False, 'why': '정답 자세 없음'}
        else:
            d = yaw_drift(odom[0], odom[1], gt[0], gt[3])
        extra['odom_drift'] = d
        rule_extra['drift_max_rad_s'] = DRIFT_MAX_RAD_S
        if not d['ok']:
            reasons.append(f"EKF yaw 표류: {d.get('why')}")
    return {'pass': not reasons, 'reasons': reasons, 'topics': topics, 'protocol': proto,
            **extra,
            'rule': {**rule_extra, 'gap_factor': GAP_FACTOR, 'tol_ns': TOL_NS,
                     'periods_s': {k: v * 1e-9 for k, v in periods.items()},
                     'cover_min_s': COVER_MIN_NS * 1e-9, 'start_still_s': start_still_s,
                     'end_still_s': end_still_s, 'move_xy_m': MOVE_XY,
                     'move_yaw_deg': math.degrees(MOVE_YAW),
                     'depart_window_s': DEPART_WINDOW_S}}


# ======================================================================== 녹화 결함 검사 (b2-rerec)
#: imu_filter_node 기동 바이어스 결과 문구
#: — amr_localization `src/imu_filter.cpp` `ImuFilter::finishStartup` 원문
BIAS_NODE = 'imu_filter_node'
BIAS_OK_FULL = 'startup bias estimated from'
BIAS_OK_BEFORE_MOTION = 'samples collected before motion'
BIAS_FALLBACK = 'falling back to parameter bias'
#: 성공 줄의 표본 하한 (200 Hz 에서 5 s). 결함 없는 녹화 5 bag 은 4 845–12 001 [측정 rec/*/launch.log]
BIAS_MIN_SAMPLES = 1000
#: EKF odom 대 정답 yaw 표류율 상한 [rad/s] (과제 지시 값)
DRIFT_MAX_RAD_S = 0.002
#: 정착 판정 — imu_filter 의 정지 판정(자이로 σ ≤ 0.01 rad/s, 가속도 σ ≤ 0.2 m/s², |‖ā‖ − g| ≤ 0.5 m/s²,
#: amr_localization config/imu_filter.yaml)의 절반 문턱. 정지한 로봇의 가속도 x σ 는 0.043–0.056 m/s² 였다
#: [계산: s6_open_a3·s6_queue_a1 launch.log 의 SE × √12001]
SETTLE_GYRO_STD = 0.005
SETTLE_ACCEL_STD = 0.1
SETTLE_NORM_TOL = 0.25
SETTLE_G = 9.8

_N_RE = re.compile(r'(?:from|using) (\d+) samples')
_GB_RE = re.compile(r'gyro_bias \[([^\]]*)\]')
_DUR_RE = re.compile(r'over ([0-9.]+) s')


def bias_outcome(text: str, min_samples: int = BIAS_MIN_SAMPLES) -> Dict[str, object]:
    """
    런치 로그 텍스트 → imu_filter_node 기동 바이어스 추정 판정 dict ('ok', 'kind', 'samples', 'gyro_bias', 'why').

    결과 줄 = imu_filter_node 줄 중 성공 문구(BIAS_OK_FULL·BIAS_OK_BEFORE_MOTION) 또는 BIAS_FALLBACK 을 담은 줄.
    통과 = 결과 줄이 있고, 첫 결과 줄이 성공이고, 그 표본 ≥ min_samples, BIAS_FALLBACK 줄이 하나도 없다.
    """
    outs = [ln.strip() for ln in text.splitlines() if BIAS_NODE in ln and (
        BIAS_OK_FULL in ln or BIAS_OK_BEFORE_MOTION in ln or BIAS_FALLBACK in ln)]
    fallback = sum(1 for ln in outs if BIAS_FALLBACK in ln)
    res: Dict[str, object] = {'n_outcome': len(outs), 'fallback': fallback, 'kind': None,
                              'samples': None, 'gyro_bias': None, 'duration_s': None,
                              'min_samples': min_samples}
    why: List[str] = []
    if not outs:
        why.append('기동 바이어스 결과 줄 없음')
    else:
        first = outs[0]
        if BIAS_FALLBACK in first:
            res['kind'] = 'fallback'
        elif BIAS_OK_FULL in first:
            res['kind'] = 'full'
        else:
            res['kind'] = 'before_motion'
        m = _N_RE.search(first)
        res['samples'] = int(m.group(1)) if m else None
        gb = _GB_RE.search(first)
        if gb:
            try:
                res['gyro_bias'] = [float(v) for v in gb.group(1).split(',')]
            except ValueError:
                res['gyro_bias'] = None
        d = _DUR_RE.search(first)
        res['duration_s'] = float(d.group(1)) if d else None
        res['line'] = first[:400]
        if res['kind'] == 'fallback':
            why.append('첫 결과가 파라미터 바이어스로 떨어짐')
        elif res['samples'] is None or res['samples'] < min_samples:
            why.append(f"표본 {res['samples']} < {min_samples}")
    if fallback:
        why.append(f'파라미터 바이어스 줄 {fallback} 개')
    res['ok'] = not why
    if why:
        res['why'] = '; '.join(why)
    return res


def yaw_drift(t: Sequence[float], yaw: Sequence[float], gt_t: Sequence[float],
              gt_yaw: Sequence[float], max_rate: float = DRIFT_MAX_RAD_S) -> Dict[str, object]:
    """
    EKF odom (t, yaw) 대 정답 (gt_t, gt_yaw) → yaw 표류율 판정 dict ('ok', 'rate_rad_s' 등).

    정답 구간 안의 odom 표본에서 정답 yaw(펼친 것)를 선형 보간하고, 두 yaw 를 각자 첫 표본에서 0 으로 맞춘 오차의
    최소제곱 일차 기울기가 rate (scripts/odom_drift.py 와 같은 정의). 통과 = |rate| < max_rate.
    """
    t = np.asarray(t, dtype=float)
    yaw = np.asarray(yaw, dtype=float)
    o = np.argsort(t, kind='stable')
    t, yaw = t[o], yaw[o]
    gt_t = np.asarray(gt_t, dtype=float)
    gt_yaw = np.asarray(gt_yaw, dtype=float)
    res: Dict[str, object] = {'n': 0, 'rate_rad_s': math.nan, 'yaw_err_max_deg': math.nan,
                              'max_rate_rad_s': max_rate, 'ok': False}
    if len(gt_t) < 2 or len(t) < 3:
        res['why'] = f'표본 부족 (odom {len(t)}, 정답 {len(gt_t)})'
        return res
    m = (t >= gt_t[0]) & (t <= gt_t[-1])
    t, yaw = t[m], yaw[m]
    res['n'] = int(len(t))
    if len(t) < 3:
        res['why'] = f'정답 구간 안 odom 표본 {len(t)}'
        return res
    g = np.interp(t, gt_t, np.unwrap(gt_yaw))
    oy = np.unwrap(yaw)
    err = (oy - oy[0]) - (g - g[0])
    rate = float(np.polyfit(t - t[0], err, 1)[0])
    res.update({'rate_rad_s': rate, 'yaw_err_max_deg': float(np.degrees(np.abs(err).max())),
                'span_s': float(t[-1] - t[0])})
    res['ok'] = bool(math.isfinite(rate) and abs(rate) < max_rate)
    if not res['ok']:
        res['why'] = f'|yaw 표류율| {abs(rate):.5f} ≥ {max_rate:g} rad/s'
    return res


def imu_still(gyro, accel, gyro_std_max: float = SETTLE_GYRO_STD,
              accel_std_max: float = SETTLE_ACCEL_STD, g: float = SETTLE_G,
              norm_tol: float = SETTLE_NORM_TOL) -> Dict[str, object]:
    """
    원시 IMU 표본 (n×3 자이로 [rad/s], n×3 가속도 [m/s²]) → 정지 판정 dict ('ok', 표준편차·평균 크기).

    정지 = 축별 표본 표준편차의 최대가 문턱 이하이고 평균 가속도 크기가 g ± norm_tol 안 (imu_filter 의 정지 판정과
    같은 꼴, 문턱은 그 절반 — 위 상수).
    """
    gy = np.asarray(gyro, dtype=float).reshape(-1, 3)
    ac = np.asarray(accel, dtype=float).reshape(-1, 3)
    n = int(min(len(gy), len(ac)))
    res: Dict[str, object] = {'n': n, 'ok': False}
    if n < 2:
        res['why'] = f'표본 {n} 개'
        return res
    gs = float(gy[:n].std(axis=0, ddof=1).max())
    acs = float(ac[:n].std(axis=0, ddof=1).max())
    norm = float(np.linalg.norm(ac[:n].mean(axis=0)))
    res.update({'gyro_std_max': gs, 'accel_std_max': acs, 'accel_norm': norm})
    why = []
    if gs > gyro_std_max:
        why.append(f'자이로 σ {gs:.4f} > {gyro_std_max:g}')
    if acs > accel_std_max:
        why.append(f'가속도 σ {acs:.3f} > {accel_std_max:g}')
    if abs(norm - g) > norm_tol:
        why.append(f'|‖ā‖ − g| {abs(norm - g):.3f} > {norm_tol:g}')
    res['ok'] = not why
    if why:
        res['why'] = '; '.join(why)
    return res
