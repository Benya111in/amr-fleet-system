"""
E2 시행 판정과 kidnap_monitor 로그 해석 (rclpy 비의존, pytest 대상).

판정 (prereg.md §3):
  재초기화가 TRACKING 을 선언한 시점, 없으면 LOST + 120 s 시점의 추정(odometry/filtered_map)과 실제
  (ground_truth/odom) 위치 오차 e 로
    correct      e < 0.5 m
    misconverge  e > 1.0 m
    ambiguous    0.5 ≤ e ≤ 1.0 m
로그 줄 형식은 kidnap_monitor_node.py / kidnap_detector.py 의 문자열 그대로다 (시험이 고정한다).
"""

import bisect
import math
import re
from typing import Dict, List, Optional, Sequence, Tuple

CORRECT_M = 0.5
MISCONVERGE_M = 1.0
RECOVERY_WINDOW_S = 120.0

_NUM = r'(-?\d+(?:\.\d+)?)'
RE_SEARCH = re.compile(r'global seed search: (\d+) hypotheses in ([\d.]+) s: (.*)$')
RE_HYP = re.compile(rf'\({_NUM}, {_NUM}, {_NUM} deg, {_NUM} m, inlier {_NUM}\)')
RE_SEED = re.compile(rf'seed #(\d+): initialpose \({_NUM}, {_NUM}, {_NUM} deg\), '
                     rf'score {_NUM} m, inlier {_NUM}')
RE_REINIT = re.compile(r'global re-init #(\d+)')
RE_LOST = re.compile(r'\[RECOVERING\] LOST: (.*)$')
RE_RECOVERED = re.compile(r'\[TRACKING\] recovered in ([\d.]+) s')
RE_GATE = re.compile(rf'alias margin (blocks convergence|bypassed by external trust): '
                     rf'margin {_NUM} <')
RE_FAILED = re.compile(r'\[FAILED\] (.*)$')
RE_AMCL_GLOBAL = re.compile(r'AMCL global re-initialization requested')
RE_REDUCE = re.compile(r'가설 #(\d+) 은 1 순위보다 인라이어가 많이 낮다')


def classify(err_m: Optional[float]) -> str:
    """위치 오차 → correct | misconverge | ambiguous | unknown (오차 없음)."""
    if err_m is None or not math.isfinite(err_m):
        return 'unknown'
    if err_m < CORRECT_M:
        return 'correct'
    if err_m > MISCONVERGE_M:
        return 'misconverge'
    return 'ambiguous'


def wrap(a: float) -> float:
    """각을 (−π, π] 로."""
    return math.atan2(math.sin(a), math.cos(a))


class Track:
    """시각순 (t, x, y, yaw) 표본 — 시각 조회 (t 이후 첫 표본 / 가장 가까운 표본)."""

    def __init__(self) -> None:
        """빈 궤적."""
        self.t: List[float] = []
        self.p: List[Tuple[float, float, float]] = []

    def add(self, t: float, x: float, y: float, yaw: float) -> None:
        """표본 추가 (시각이 뒤로 가면 무시)."""
        if self.t and t < self.t[-1]:
            return
        self.t.append(t)
        self.p.append((x, y, yaw))

    def __len__(self) -> int:
        """표본 수."""
        return len(self.t)

    def at_or_after(self, t: float, max_gap: float = 1.0) -> Optional[Tuple[float, Tuple]]:
        """t 이상 첫 표본 (max_gap 안에 없으면 t 이전 마지막 표본, 그것도 max_gap 밖이면 None)."""
        i = bisect.bisect_left(self.t, t)
        if i < len(self.t) and self.t[i] - t <= max_gap:
            return self.t[i], self.p[i]
        if i > 0 and t - self.t[i - 1] <= max_gap:
            return self.t[i - 1], self.p[i - 1]
        return None

    def nearest(self, t: float, max_gap: float = 1.0) -> Optional[Tuple[float, Tuple]]:
        """t 에 가장 가까운 표본 (max_gap 안)."""
        i = bisect.bisect_left(self.t, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.t) and abs(self.t[j] - t) <= max_gap:
                if best is None or abs(self.t[j] - t) < abs(best[0] - t):
                    best = (self.t[j], self.p[j])
        return best


def pose_error(est: Optional[Tuple],
               gt: Optional[Tuple]) -> Tuple[Optional[float], Optional[float]]:
    """(위치 오차 m, |헤딩 오차| rad). 하나라도 없으면 (None, None)."""
    if est is None or gt is None:
        return None, None
    return (math.hypot(est[0] - gt[0], est[1] - gt[1]), abs(wrap(est[2] - gt[2])))


def parse_hyps(text: str) -> List[List[float]]:
    """'(x, y, yaw deg, score m, inlier r), …' → [[x, y, yaw_rad, score, inlier], …]."""
    return [[float(m.group(1)), float(m.group(2)), math.radians(float(m.group(3))),
             float(m.group(4)), float(m.group(5))] for m in RE_HYP.finditer(text)]


def parse_monitor_line(msg: str) -> Optional[Dict]:
    """kidnap_monitor 로그 한 줄 → 사건 dict (관심 없는 줄은 None)."""
    m = RE_SEARCH.search(msg)
    if m:
        return {'ev': 'search', 'n': int(m.group(1)), 'search_s': float(m.group(2)),
                'hyps': parse_hyps(m.group(3))}
    m = RE_SEED.search(msg)
    if m:
        return {'ev': 'seed', 'attempt': int(m.group(1)), 'x': float(m.group(2)),
                'y': float(m.group(3)), 'yaw': math.radians(float(m.group(4))),
                'score': float(m.group(5)), 'inlier': float(m.group(6))}
    m = RE_LOST.search(msg)
    if m:
        return {'ev': 'lost', 'reason': m.group(1)}
    m = RE_REINIT.search(msg)
    if m:
        return {'ev': 'reinit', 'n': int(m.group(1))}
    m = RE_RECOVERED.search(msg)
    if m:
        return {'ev': 'recovered', 'after_s': float(m.group(1))}
    m = RE_GATE.search(msg)
    if m:
        return {'ev': 'gate', 'bypassed': m.group(1).startswith('bypassed'),
                'margin': float(m.group(2))}
    m = RE_FAILED.search(msg)
    if m:
        return {'ev': 'failed', 'reason': m.group(1)}
    if RE_AMCL_GLOBAL.search(msg):
        return {'ev': 'amcl_global'}
    m = RE_REDUCE.search(msg)
    if m:
        return {'ev': 'reseed_top1', 'attempt': int(m.group(1))}
    return None


def summarize_recovery(events: Sequence[Tuple[float, Dict]], t_lost: float,
                       t_end: float) -> Dict:
    """복구 구간 [t_lost, t_end] 의 사건 요약: 재초기화 수, 시드, 가설, 마진 게이트, 실패."""
    win = [(t, e) for t, e in events if t_lost - 0.5 <= t <= t_end + 0.5]
    seeds = [{'t': round(t - t_lost, 2), **{k: v for k, v in e.items() if k != 'ev'}}
             for t, e in win if e['ev'] == 'seed']
    searches = [{'t': round(t - t_lost, 2), 'hyps': e['hyps'], 'search_s': e['search_s']}
                for t, e in win if e['ev'] == 'search']
    gates = [e['margin'] for t, e in win if e['ev'] == 'gate' and not e['bypassed']]
    return {
        'n_reinit': max([e['n'] for _, e in win if e['ev'] == 'reinit'], default=0),
        'n_seed': len(seeds),
        'amcl_global': any(e['ev'] == 'amcl_global' for _, e in win),
        'reseed_top1': sum(1 for _, e in win if e['ev'] == 'reseed_top1'),
        'seeds': seeds, 'searches': searches,
        'gate_margins': gates,
        'gate_bypassed': sum(1 for _, e in win if e['ev'] == 'gate' and e['bypassed']),
        'failed': next((e['reason'] for _, e in win if e['ev'] == 'failed'), None),
        'lost_reason': next((e['reason'] for _, e in win if e['ev'] == 'lost'), None),
    }


def seed_is_true(seed: Dict, gt: Tuple[float, float, float], dist: float = 1.0) -> bool:
    """시드 위치가 실제 위치에서 dist 안인가."""
    return math.hypot(seed['x'] - gt[0], seed['y'] - gt[1]) <= dist
