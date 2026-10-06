#!/usr/bin/env python3
"""
정본 safety φ 교차 시험 — torch 이식 `amr_isaac.safety_gate_torch` 대 C++ 원본 `amr_perception::SafetyGate`.

    rl/isaac/tools/phi_crosscheck/crosscheck.sh [--quick] [--out <json>]   (amr-ai/rl:0.3, ROS 없이)

세 묶음:
  fp   점별 — 스윕 접근 거리 (sweptRegion + approach), 꼭짓점 최고 속력, 여유 거리 상한, 존
  win  배열별 — physicalWindowStatistic (빔·가드 창, 원형·비원형, 가까운 거리의 큰 창), kthSmallest
  seq  순차 — SafetyGate 상태기계 (시나리오 × 50 Hz 틱): 무작위 폐루프 세계(벽·상자·기둥 레이캐스트, 명령 열,
       측정 지연, 점군·트랙·도킹 예외·명령 끊김)와 경계 사례(직진 운동 = 사칙연산만이라 두 구현이 비트 단위로
       같은 값을 문턱 바로 위아래에 놓는다)

판정: 이산 출력 정확 일치, 연속 출력 |a − b| ≤ 1e-9·max(1, |a|, |b|) (+inf 끼리는 같다).

이진 형식 (little-endian int64 / float64, gate_runner.cpp 와 같다):
  fp  입력 int64 M, K; f64 [M, 3] (v, w, d); f64 [M, K, 2] 점 → 출력 f64 [M, 4] + f64 [M, K]
  win 입력 int64 A; 배열마다 int64 n, f64 inc, f64 width, int64 min_window, f64 support, int64 circular,
      f64 [n] 값, f64 [n] 거리; int64 Q; 질의마다 int64 n, int64 support, f64 [n] → 출력 f64 [A] + f64 [Q]
  seq 입력 int64 S, T, B, circular, NS, Cmax, Tmax, Pmax; f64 inc, ox, oy; 시나리오마다 f64 [NS, B, 2] 스캔과
      f64 [T, REC] 틱 기록 (REC 순서는 TickBatch.pack) → 출력 f64 [S, T, 20] (OUT_NAMES)
"""

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', '..')))

from amr_isaac import safety_gate_torch as G  # noqa: E402

OUT_NAMES = ('v', 'w', 'zone', 'prox', 'causes', 'escaping', 'retreat', 'yield_escaping',
             'ttc_limited', 'cloud_stale', 'command_stale', 'speed_limit', 'angular_limit',
             'min_distance', 'lidar_distance', 'cloud_distance', 'contact_distance',
             'exclusion_distance', 'min_ttc', 'estop_active')
DISCRETE = {'zone', 'prox', 'causes', 'escaping', 'retreat', 'yield_escaping', 'ttc_limited',
            'cloud_stale', 'command_stale', 'estop_active'}
LIDAR_INC = (3.13286601 + 3.14159265) / 719.0
LIDAR_MIN = -3.14159265
ORIGIN = (0.15, 0.0)
DT = 0.02
CONFIGS = {
    'deployed': {},
    'yield_on': {'yield_escape_enabled': True},
    'cloud_optional': {'cloud_required': False},
}


# ======================================================================== 비교
def close(a, b, tol=1e-9):
    """연속 비교: 둘 다 +inf(또는 −inf)면 같다."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    both_inf = np.isinf(a) & np.isinf(b) & (np.sign(a) == np.sign(b))
    both_nan = np.isnan(a) & np.isnan(b)
    with np.errstate(invalid='ignore'):
        scale = np.maximum(1.0, np.maximum(np.abs(a), np.abs(b)))
        ok = np.abs(a - b) <= tol * scale
    return ok | both_inf | both_nan


class Tally:
    """불일치 집계 (묶음·채널별) + 앞 사례 몇 개."""

    def __init__(self):
        """빈 집계."""
        self.n = {}
        self.bad = {}
        self.examples = []
        self.max_abs = {}

    def add(self, group, name, ok, a=None, b=None, context=None):
        """비교 결과(ok bool 배열)를 센다."""
        key = f'{group}/{name}'
        ok = np.asarray(ok)
        self.n[key] = self.n.get(key, 0) + int(ok.size)
        nb = int((~ok).sum())
        self.bad[key] = self.bad.get(key, 0) + nb
        if a is not None:
            fa, fb = np.asarray(a, np.float64), np.asarray(b, np.float64)
            fin = np.isfinite(fa) & np.isfinite(fb)
            if fin.any():
                d = float(np.abs(fa[fin] - fb[fin]).max())
                self.max_abs[key] = max(self.max_abs.get(key, 0.0), d)
        if nb and len(self.examples) < 40:
            idx = np.argwhere(~ok)[:3]
            for ix in idx:
                ix = tuple(int(i) for i in ix)
                ex = {'key': key, 'index': ix}
                if a is not None:
                    ex['cpp'] = float(np.asarray(a)[ix])
                    ex['torch'] = float(np.asarray(b)[ix])
                if context is not None:
                    ex['context'] = context(ix)
                self.examples.append(ex)

    def summary(self):
        """요약 dict 를 돌려준다."""
        return {'checks': self.n, 'mismatches': self.bad, 'max_abs_diff_finite': self.max_abs,
                'total_checks': int(sum(self.n.values())),
                'total_mismatches': int(sum(self.bad.values())), 'examples': self.examples}


def run_cpp(binary, mode, inp, out, extra=()):
    """gate_runner 를 부른다."""
    r = subprocess.run([binary, mode, inp, out, *extra], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f'gate_runner {mode} rc={r.returncode}: {r.stderr}')


# ======================================================================== fp
def gen_motions(rng, M):
    """접근 영역 운동 (v, w) 와 상한·존 검사용 거리 d — 직진·원호·회전·후진·ε 근처·거의 직선."""
    v = rng.uniform(-0.5, 2.0, M)
    w = rng.uniform(-1.5, 1.5, M)
    kind = rng.integers(0, 10, M)
    v = np.where(kind == 0, 0.0, v)
    w = np.where(kind == 1, 0.0, w)
    eps_v = np.array([0.01, 0.0099999, 0.0100001, -0.01, -0.0100001])
    eps_w = np.array([0.05, 0.0499999, 0.0500001, -0.05, -0.0500001])
    v = np.where(kind == 2, rng.choice(eps_v, M), v)
    w = np.where(kind == 3, rng.choice(eps_w, M), w)
    w = np.where(kind == 4, rng.choice([1e-13, -1e-13, 1e-11, 2e-6, -9e-6], M), w)   # 거의 직선
    v = np.where(kind == 5, rng.uniform(-0.02, 0.02, M), v)
    d = rng.uniform(0.0, 4.0, M)
    dk = rng.integers(0, 8, M)
    d = np.where(dk == 0, rng.choice([0.0, 0.1, 0.3, 0.5, 1.0, np.inf], M), d)
    return np.stack([v, w, d], 1)


def gen_points(rng, M, K):
    """점: 3 m 상자 균일 + 풋프린트 모서리 근처 + 원 근처."""
    pts = rng.uniform(-3.0, 3.0, (M, K, 2))
    k1 = K // 4
    edge = np.stack([rng.choice([-1, 1], (M, k1)) * (0.3 + rng.uniform(-0.05, 0.4, (M, k1))),
                     rng.uniform(-0.5, 0.5, (M, k1))], -1)
    pts[:, :k1] = edge
    k2 = K // 4
    side = np.stack([rng.uniform(-0.6, 0.6, (M, k2)),
                     rng.choice([-1, 1], (M, k2)) * (0.2 + rng.uniform(-0.03, 0.3, (M, k2)))], -1)
    pts[:, k1:k1 + k2] = side
    return pts


def check_fp(binary, tmp, rng, M, K, tally, device):
    """점별 대조."""
    mot = gen_motions(rng, M)
    pts = gen_points(rng, M, K)
    inp = os.path.join(tmp, 'fp.in')
    out = os.path.join(tmp, 'fp.out')
    with open(inp, 'wb') as f:
        np.array([M, K], dtype='<i8').tofile(f)
        mot.astype('<f8').tofile(f)
        pts.astype('<f8').tofile(f)
    run_cpp(binary, 'fp', inp, out)
    res = np.fromfile(out, dtype='<f8')
    head = res[:M * 4].reshape(M, 4)
    app = res[M * 4:].reshape(M, K)
    p = G.DEPLOYED
    v = torch.tensor(mot[:, 0], dtype=G.F64, device=device)
    w = torch.tensor(mot[:, 1], dtype=G.F64, device=device)
    d = torch.tensor(mot[:, 2], dtype=G.F64, device=device)
    mps = G.max_point_speed(0.3, 0.2, v, w).cpu().numpy()
    c30 = G.clearance_speed_limit(d, 0.30, p).cpu().numpy()
    c10 = G.clearance_speed_limit(d, 0.10, p).cpu().numpy()
    zone = G.classify_zone(d, p).cpu().numpy()
    tally.add('fp', 'max_point_speed', close(head[:, 0], mps), head[:, 0], mps)
    tally.add('fp', 'clearance_0.30', close(head[:, 1], c30), head[:, 1], c30)
    tally.add('fp', 'clearance_0.10', close(head[:, 2], c10), head[:, 2], c10)
    tally.add('fp', 'zone', head[:, 3] == zone)
    reg = G.Region(v, w, torch.ones_like(v, dtype=torch.bool), p)
    P = torch.tensor(pts, dtype=G.F64, device=device)
    s = reg.approach(P[..., 0], P[..., 1]).cpu().numpy()
    ctx = lambda ix: {'v': float(mot[ix[0], 0]), 'w': float(mot[ix[0], 1]),  # noqa: E731
                      'p': [float(pts[ix[0], ix[1], 0]), float(pts[ix[0], ix[1], 1])]}
    tally.add('fp', 'approach', close(app, s), app, s, ctx)
    return {'motions': M, 'points_per_motion': K,
            'finite_approach_frac': float(np.isfinite(app).mean())}


# ======================================================================== win
def check_win(binary, tmp, rng, A, tally, device):
    """창 통계·k 번째 값."""
    specs = []
    for _ in range(A):
        n = int(rng.choice([5, 9, 10, 37, 180, 360, 720]))
        guard = bool(rng.integers(0, 2))
        circ = bool(rng.integers(0, 2))
        specs.append((n, guard, circ))
    arrays = []
    for n, guard, circ in specs:
        vals = rng.uniform(0.0, 3.0, n)
        vals = np.where(rng.random(n) < rng.uniform(0.2, 0.95), np.inf, vals)
        if rng.random() < 0.3:     # 같은 값 묶음 (평평한 벽)
            vals = np.where(np.isfinite(vals), np.round(vals, 1), vals)
        rngs = np.exp(rng.uniform(np.log(0.1), np.log(12.0), n))
        rngs = np.where(rng.random(n) < 0.05, np.nan, rngs)
        width = 0.06 if guard else 0.04
        mw = 9 if guard else 5
        sup = 0.5 if guard else 0.6
        arrays.append((n, LIDAR_INC if n == 720 else 2 * math.pi / n, width, mw, sup, circ,
                       vals, rngs))
    Q = A // 2
    queries = []
    for _ in range(Q):
        n = int(rng.choice([0, 1, 2, 3, 5, 12, 60]))
        sup = int(rng.choice([1, 2, 3, 5]))
        vals = rng.uniform(0.0, 3.0, n)
        vals = np.where(rng.random(n) < 0.4, np.inf, vals)
        queries.append((n, sup, vals))
    inp = os.path.join(tmp, 'win.in')
    out = os.path.join(tmp, 'win.out')
    with open(inp, 'wb') as f:
        np.array([A], dtype='<i8').tofile(f)
        for n, inc, width, mw, sup, circ, vals, rngs in arrays:
            np.array([n], '<i8').tofile(f)
            np.array([inc, width], '<f8').tofile(f)
            np.array([mw], '<i8').tofile(f)
            np.array([sup], '<f8').tofile(f)
            np.array([int(circ)], '<i8').tofile(f)
            vals.astype('<f8').tofile(f)
            rngs.astype('<f8').tofile(f)
        np.array([Q], '<i8').tofile(f)
        for n, sup, vals in queries:
            np.array([n, sup], '<i8').tofile(f)
            vals.astype('<f8').tofile(f)
    run_cpp(binary, 'win', inp, out)
    res = np.fromfile(out, dtype='<f8')
    cw, cq = res[:A], res[A:]
    tw = np.zeros(A)
    groups = {}
    for i, a in enumerate(arrays):
        groups.setdefault((a[0], a[1], a[2], a[3], a[4], a[5]), []).append(i)
    for (n, inc, width, mw, sup, circ), idx in groups.items():
        vals = torch.tensor(np.stack([arrays[i][6] for i in idx]), dtype=G.F64, device=device)
        rngs = torch.tensor(np.stack([arrays[i][7] for i in idx]), dtype=G.F64, device=device)
        spec = G.WindowSpec(rngs, inc, width, mw, sup, circ)
        tw[idx] = G.window_statistic(vals, spec).cpu().numpy()
    tally.add('win', 'physical_window', close(cw, tw), cw, tw)
    tq = np.zeros(Q)
    for i, (n, sup, vals) in enumerate(queries):
        t = torch.tensor(vals.reshape(1, -1), dtype=G.F64, device=device)
        tq[i] = float(G.kth_smallest(t, sup)[0])
    tally.add('win', 'kth_smallest', close(cq, tq), cq, tq)
    return {'arrays': A, 'queries': Q, 'finite_result_frac': float(np.isfinite(cw).mean())}


# ======================================================================== seq 입력 묶음
class TickBatch:
    """시나리오 S 개 × 틱 T 개의 입력 (numpy). 같은 묶음은 같은 B·NS·Cmax·Tmax·Pmax·설정."""

    def __init__(self, S, T, B=720, NS=12, Cmax=0, Tmax=0, Pmax=0, circular=True, inc=LIDAR_INC):
        """빈 입력 (플래그 0)."""
        self.S, self.T, self.B, self.NS = S, T, B, NS
        self.Cmax, self.Tmax, self.Pmax = Cmax, Tmax, Pmax
        self.circular, self.inc = circular, inc
        self.now = np.zeros((S, T))
        self.meas = np.zeros((S, T, 4))
        self.cmd = np.zeros((S, T, 4))
        self.scanf = np.zeros((S, T, 3))
        self.scans = np.full((S, NS, B, 2), np.nan)
        self.cloud_h = np.zeros((S, T, 4))
        self.cloud = np.full((S, T, Cmax, 2), np.nan)
        self.trk_h = np.zeros((S, T, 5))
        self.trk = np.full((S, T, Tmax, 6), np.nan)
        self.excl_h = np.zeros((S, T, 3))
        self.excl = np.zeros((S, T, Pmax, 2))
        self.family = [''] * S

    def pack(self, path):
        """gate_runner seq 입력 파일."""
        S, T = self.S, self.T
        rec = np.concatenate([
            self.now[..., None], self.meas, self.cmd, self.scanf, self.cloud_h,
            self.cloud.reshape(S, T, -1), self.trk_h, self.trk.reshape(S, T, -1), self.excl_h,
            self.excl.reshape(S, T, -1)], axis=2)
        with open(path, 'wb') as f:
            np.array([S, T, self.B, int(self.circular), self.NS, self.Cmax, self.Tmax, self.Pmax],
                     '<i8').tofile(f)
            np.array([self.inc, ORIGIN[0], ORIGIN[1]], '<f8').tofile(f)
            for s in range(S):
                self.scans[s].astype('<f8').tofile(f)
                rec[s].astype('<f8').tofile(f)


def make_gate(tb, params, device):
    """묶음에 맞는 SafetyGateBatch."""
    return G.SafetyGateBatch(tb.S, device, params, n_beams=tb.B, angle_increment=tb.inc,
                             circular=tb.circular, origin=ORIGIN, max_cloud=tb.Cmax,
                             max_tracks=tb.Tmax, max_poly=tb.Pmax)


def apply_tick(gate, tb, t, device, scans_t):
    """틱 t 의 입력을 gate 에 넣는다 (gate_runner 와 같은 순서: 측정 → 스캔 → 점군 → 트랙 → 예외 → 명령)."""
    def T_(x, dt=G.F64):
        return torch.as_tensor(x, dtype=dt, device=device)
    m = tb.meas[:, t]
    gate.set_measured(T_(m[:, 1]), T_(m[:, 2]), T_(m[:, 3]), mask=T_(m[:, 0] != 0, torch.bool))
    sf = tb.scanf[:, t]
    if (sf[:, 0] != 0).any():
        idx = sf[:, 1].astype(np.int64)
        beams = scans_t[torch.arange(tb.S, device=device), T_(idx, torch.long)]
        gate.set_scan(beams, T_(sf[:, 2]), mask=T_(sf[:, 0] != 0, torch.bool))
    ch = tb.cloud_h[:, t]
    if (ch[:, 0] != 0).any():
        pts = tb.cloud[:, t].copy()
        n = ch[:, 3].astype(np.int64)
        for s in range(tb.S):
            pts[s, n[s]:] = np.nan
        gate.set_cloud(T_(pts), T_(ch[:, 1]), T_(ch[:, 2]), mask=T_(ch[:, 0] != 0, torch.bool))
    th = tb.trk_h[:, t]
    if (th[:, 0] == 1).any():
        ok = np.arange(tb.Tmax)[None, :] < th[:, 3:4]
        gate.set_tracks(T_(tb.trk[:, t]), T_(ok, torch.bool), T_(th[:, 1]), T_(th[:, 2]),
                        mask=T_(th[:, 0] == 1, torch.bool))
    if (th[:, 0] == 2).any():
        gate.set_min_ttc(T_(th[:, 4]), T_(th[:, 2]), mask=T_(th[:, 0] == 2, torch.bool))
    eh = tb.excl_h[:, t]
    if (eh[:, 0] != 0).any():
        gate.set_exclusion(T_(tb.excl[:, t]), T_(eh[:, 2], torch.long), T_(eh[:, 1]),
                           mask=T_(eh[:, 0] != 0, torch.bool))
    c = tb.cmd[:, t]
    gate.set_command(T_(c[:, 1]), T_(c[:, 2]), T_(c[:, 3]), mask=T_(c[:, 0] != 0, torch.bool))


def out_row(st):
    """출력 행 — evaluate dict → [S, 20] numpy (OUT_NAMES 순서, estop_active = 0)."""
    cols = []
    for k in OUT_NAMES:
        if k == 'estop_active':
            cols.append(torch.zeros_like(st['v']))
        else:
            cols.append(st[k].to(G.F64))
    return torch.stack(cols, 1).cpu().numpy()


def replay(tb, params, device):
    """입력 묶음을 torch 이식으로 다시 돈다 → [S, T, 20]."""
    gate = make_gate(tb, params, device)
    scans_t = torch.tensor(tb.scans, dtype=G.F64, device=device)
    out = np.zeros((tb.S, tb.T, len(OUT_NAMES)))
    for t in range(tb.T):
        apply_tick(gate, tb, t, device, scans_t)
        out[:, t] = out_row(gate.evaluate(torch.tensor(tb.now[:, t], dtype=G.F64,
                                                       device=device)))
    return out


# ======================================================================== 무작위 폐루프 세계
def raycast(origin, yaw, segs, B, inc=LIDAR_INC, amin=LIDAR_MIN):
    """세계 선분 [S, M, 4] 에 대한 2D 레이캐스트 → 거리 [S, B] (무반사 inf)."""
    ang = yaw[:, None] + amin + np.arange(B)[None, :] * inc
    dx, dy = np.cos(ang), np.sin(ang)
    ox, oy = origin[:, 0:1, None], origin[:, 1:2, None]
    x1, y1, x2, y2 = (segs[:, None, :, i] for i in range(4))
    ex, ey = x2 - x1, y2 - y1
    den = dx[..., None] * ey - dy[..., None] * ex
    with np.errstate(divide='ignore', invalid='ignore'):
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
        u = ((x1 - ox) * dy[..., None] - (y1 - oy) * dx[..., None]) / den
    hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(hit, t, np.inf)
    return t.min(axis=2)


def box_segs(cx, cy, hx, hy, yaw):
    """회전 상자 → 선분 4 개."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(cx + c * x - s * y, cy + s * x + c * y)
           for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
    return [(*pts[i], *pts[(i + 1) % 4]) for i in range(4)]


def random_world(rng, M):
    """한 시나리오의 세계 선분 (최대 M 개, 모자라면 멀리 둔 점 선분) 과 이름."""
    kind = rng.choice(['aisle', 'face', 'clutter', 'corner', 'open'])
    segs = []
    if kind == 'aisle':
        g = rng.uniform(0.0, 0.5)
        yo = 0.2 + g
        segs += [(-3, yo, 6, yo), (-3, -yo - rng.uniform(0, 0.3), 6, -yo - rng.uniform(0, 0.3))]
        if rng.random() < 0.6:
            xe = rng.uniform(0.35, 3.0)
            segs.append((xe, -2, xe, 2))
    elif kind == 'face':
        xf = rng.uniform(0.32, 1.8)
        segs.append((xf, -1.5, xf + rng.uniform(-0.2, 0.2), 1.5))
        for _ in range(int(rng.integers(0, 3))):
            segs += box_segs(rng.uniform(-1, 2), rng.uniform(-1.5, 1.5), rng.uniform(0.05, 0.4),
                             rng.uniform(0.05, 0.4), rng.uniform(-3, 3))
    elif kind == 'clutter':
        for _ in range(int(rng.integers(3, 8))):
            r = rng.uniform(0.35, 2.5)
            a = rng.uniform(-math.pi, math.pi)
            segs += box_segs(r * math.cos(a), r * math.sin(a), rng.uniform(0.02, 0.15),
                             rng.uniform(0.02, 0.15), rng.uniform(-3, 3))
    elif kind == 'corner':
        segs += [(rng.uniform(0.4, 1.5), -3, rng.uniform(0.4, 1.5), 3),
                 (-3, rng.uniform(0.25, 1.0), 3, rng.uniform(0.25, 1.0))]
    else:
        for _ in range(int(rng.integers(1, 4))):
            segs += box_segs(rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(0.1, 1.0),
                             rng.uniform(0.1, 1.0), rng.uniform(-3, 3))
    segs = segs[:M]
    while len(segs) < M:
        segs.append((100.0, 100.0, 100.0, 100.0))
    return np.array(segs, dtype=np.float64), str(kind)


def gen_random_batch(rng, S, T, params, device, Cmax=24, Tmax=4, Pmax=4):
    """
    무작위 폐루프 시나리오 S 개: torch 이식을 φ 로 써서 로봇을 움직이며 입력을 기록한다.

    기록한 입력은 교차 시험에서 C++ 과 torch 에 똑같이 들어간다 (생성에 쓴 φ 가 틀려도 대조는 유효하다).
    """
    M = 16
    NS = T // 5 + 1
    tb = TickBatch(S, T, NS=NS, Cmax=Cmax, Tmax=Tmax, Pmax=Pmax)
    worlds = np.zeros((S, M, 4))
    for s in range(S):
        worlds[s], tb.family[s] = random_world(rng, M)
    gate = make_gate(tb, params, device)
    pose = np.zeros((S, 3))
    pose[:, 2] = rng.uniform(-0.3, 0.3, S)
    vm = np.zeros(S)
    wm = np.zeros(S)
    prof = rng.integers(0, 6, S)            # 명령 꼴
    v0 = rng.uniform(-0.5, 1.2, S)
    w0 = np.where(rng.random(S) < 0.35, 0.0, rng.uniform(-1.5, 1.5, S))
    sw = rng.integers(10, T, S)             # 바꾸는 틱
    use_cloud = rng.random(S) < 0.35
    use_trk = rng.random(S) < 0.35
    use_excl = rng.random(S) < 0.25
    drop_cmd = rng.random(S) < 0.15
    drop_meas = rng.random(S) < 0.15
    meas_lag = rng.integers(0, 3, S)
    sigma = rng.uniform(0.0, 0.04, S)
    t0 = rng.uniform(0.0, 5.0, S)
    people = rng.uniform(-2.5, 2.5, (S, Tmax, 2))
    pvel = rng.uniform(-1.2, 1.2, (S, Tmax, 2))
    excl_poly = np.stack([np.stack([rng.uniform(0.2, 0.6, S), rng.uniform(-0.6, -0.1, S)], 1),
                          np.stack([rng.uniform(0.8, 2.0, S), rng.uniform(-0.6, -0.1, S)], 1),
                          np.stack([rng.uniform(0.8, 2.0, S), rng.uniform(0.1, 0.6, S)], 1),
                          np.stack([rng.uniform(0.2, 0.6, S), rng.uniform(0.1, 0.6, S)], 1)], 1)
    scans_t = None
    k_scan = 0
    for t in range(T):
        now = t0 + t * DT
        tb.now[:, t] = now
        # 측정 (휠 오도메트리, 50 Hz) — 지연 0–2 틱 뒤 값, 가끔 끊김·NaN
        send_m = ~(drop_meas & (t > sw) & (t < sw + 20))
        mv = vm + rng.normal(0, 0.005, S)
        mw = wm + rng.normal(0, 0.02, S)
        mv = np.where(rng.random(S) < 0.01, np.nan, mv)
        tb.meas[:, t] = np.stack([send_m, mv, mw, now - meas_lag * DT], 1)
        # 스캔 10 Hz
        if t % 5 == 0:
            r = raycast(np.stack([pose[:, 0] + ORIGIN[0] * np.cos(pose[:, 2]),
                                  pose[:, 1] + ORIGIN[0] * np.sin(pose[:, 2])], 1),
                        pose[:, 2], worlds, tb.B)
            r = r + rng.normal(0, 1, r.shape) * sigma[:, None]
            r = np.where(rng.random(r.shape) < 0.01, np.nan, r)
            ok = np.isfinite(r) & (r >= 0.10) & (r <= 25.0)
            ang = LIDAR_MIN + np.arange(tb.B) * LIDAR_INC
            bx = np.where(ok, ORIGIN[0] + r * np.cos(ang), np.nan)
            by = np.where(ok, ORIGIN[1] + r * np.sin(ang), np.nan)
            tb.scans[:, k_scan] = np.stack([bx, by], -1)
            stamp = now - rng.choice([0.0, 0.02, 0.04, 0.1], S)
            tb.scanf[:, t] = np.stack([np.ones(S), np.full(S, k_scan), stamp], 1)
            k_scan += 1
        # 점군 (~15 Hz): 앞쪽 스캔 점 일부 + 낮은 물체 점
        if t % 3 == 0:
            send_c = use_cloud & ~((t > sw) & (t < sw + 25) & (prof == 3))
            last = tb.scans[:, max(k_scan - 1, 0)]
            for s in np.nonzero(send_c)[0]:
                pts = last[s][np.isfinite(last[s, :, 0]) & (last[s, :, 0] > 0.3) &
                              (np.abs(np.arctan2(last[s, :, 1], last[s, :, 0] - 0.29)) < 0.76)]
                if len(pts) > Cmax - 6:
                    pts = pts[rng.choice(len(pts), Cmax - 6, replace=False)]
                low = np.stack([rng.uniform(0.3, 1.5, 6), rng.uniform(-0.3, 0.3, 6)], 1) \
                    if rng.random() < 0.3 else np.zeros((0, 2))
                pts = np.concatenate([pts, low])[:Cmax]
                tb.cloud[s, t, :len(pts)] = pts
                tb.cloud_h[s, t] = [1, now[s] - 0.03, now[s], len(pts)]
        # 트랙 (10 Hz): 사람 위치(로봇 좌표)·속도, TTC 대략(접근 속력), 가끔 기하 없는 TTC
        if t % 5 == 2:
            for s in np.nonzero(use_trk)[0]:
                n = int(rng.integers(0, Tmax + 1))
                c, sn = math.cos(pose[s, 2]), math.sin(pose[s, 2])
                for j in range(n):
                    wx = people[s, j, 0] + pvel[s, j, 0] * t * DT - pose[s, 0]
                    wy = people[s, j, 1] + pvel[s, j, 1] * t * DT - pose[s, 1]
                    px, py = c * wx + sn * wy, -sn * wx + c * wy
                    vx = c * pvel[s, j, 0] + sn * pvel[s, j, 1]
                    vy = -sn * pvel[s, j, 0] + c * pvel[s, j, 1]
                    dist = math.hypot(px, py)
                    clos = -(px * (vx - vm[s]) + py * vy) / max(dist, 1e-6)
                    ttc = max(0.0, (dist - 0.66) / clos) if clos > 0.05 else np.inf
                    if rng.random() < 0.1:
                        ttc = np.nan
                    tb.trk[s, t, j] = [ttc, float(rng.random() < 0.8), px, py, vx, vy]
                if rng.random() < 0.15:
                    tb.trk_h[s, t] = [2, now[s] - 0.05, now[s], 0, rng.uniform(0, 4)]
                else:
                    tb.trk_h[s, t] = [1, now[s] - rng.choice([0.0, 0.05, 0.3]), now[s], n, 0]
        # 도킹 예외 다각형 (5 틱마다 다시, 가끔 끊김)
        if t % 5 == 1:
            send_e = use_excl & ~((t > sw) & (prof == 4))
            for s in np.nonzero(send_e)[0]:
                tb.excl[s, t] = excl_poly[s]
                tb.excl_h[s, t] = [1, now[s], 4]
        # 명령 (50 Hz)
        v_c = np.where(t < sw, v0, -0.5 * v0)
        w_c = np.where(t < sw, w0, -w0)
        v_c = np.where(prof == 1, v0 * np.cos(0.1 * t), v_c)
        w_c = np.where(prof == 2, np.where((t // 10) % 2 == 0, 1.2, -1.2), w_c)
        v_c = np.where(prof == 2, 0.0, v_c)
        v_c = np.where((prof == 5) & (t > sw), 0.0, v_c)
        w_c = np.where((prof == 5) & (t > sw), 0.0, w_c)
        send_c = ~(drop_cmd & (t > sw) & (t < sw + 35))
        tb.cmd[:, t] = np.stack([send_c, v_c, w_c, np.full(S, now)], 1)
        # 이식으로 φ → 플랜트 (1 차 지연 0.08 s, 가속 한계) → 자세
        if scans_t is None or t % 5 == 0:
            scans_t = torch.tensor(tb.scans, dtype=G.F64, device=device)
        apply_tick(gate, tb, t, device, scans_t)
        st = gate.evaluate(torch.tensor(now, dtype=G.F64, device=device))
        vo = st['v'].cpu().numpy()
        wo = st['w'].cpu().numpy()
        a = 1.0 - math.exp(-DT / 0.08)
        vm = vm + np.clip(a * (vo - vm), -DT * 1.0, DT * 1.0)
        wm = wm + np.clip(a * (wo - wm), -DT * 2.0, DT * 2.0)
        th = pose[:, 2] + wm * DT
        small = np.abs(wm) < 1e-9
        ws = np.where(small, 1.0, wm)
        pose[:, 0] += np.where(small, vm * DT * np.cos(pose[:, 2]),
                               vm / ws * (np.sin(th) - np.sin(pose[:, 2])))
        pose[:, 1] += np.where(small, vm * DT * np.sin(pose[:, 2]),
                               -vm / ws * (np.cos(th) - np.cos(pose[:, 2])))
        pose[:, 2] = th
    return tb


# ======================================================================== 경계 사례 (직진 = 사칙연산)
def ulps(x, k):
    """값 x 에서 k ulp 떨어진 값."""
    y = float(x)
    for _ in range(abs(k)):
        y = float(np.nextafter(y, np.inf if k > 0 else -np.inf))
    return y


def wall_points(x, B, y_lo=-0.35, y_hi=0.35, extra=None):
    """수직 벽(x 위치)의 점 B 개 (base_link 좌표, 남는 빔은 NaN). extra = 추가 점 목록."""
    pts = np.full((B, 2), np.nan)
    n = min(B // 2, 141)
    ys = np.linspace(y_lo, y_hi, n)
    pts[:n, 0] = x
    pts[:n, 1] = ys
    if extra is not None:
        e = np.asarray(extra, dtype=np.float64).reshape(-1, 2)
        pts[n:n + len(e)] = e
    return pts


def side_points(g, B, side=1.0, x_lo=-0.35, x_hi=0.35):
    """옆 벽 y = side·(0.2 + g), x ∈ [x_lo, x_hi]."""
    pts = np.full((B, 2), np.nan)
    n = min(B // 2, 141)
    pts[:n, 0] = np.linspace(x_lo, x_hi, n)
    pts[:n, 1] = side * (0.2 + g)
    return pts


def gen_boundary(rng, device, params_name):
    """
    경계 사례 묶음들 (S 는 사례 수).

    대부분 직진 또는 정지 운동이고 측정 속도도 직진이라 접근 거리·가드 거리가 사칙연산만으로 나온다 — 두 구현이
    문턱 바로 위아래에서 비트 단위로 같은 값을 비교한다.
    """
    out = []
    B = 720
    T = 40
    # ---- B1: 전방 벽 접근 거리 = x − 0.3 이 문턱(0.25, 0.30, 0.50, 0.55, 1.00, 1.05) ± k ulp
    thr = [0.25, 0.30, 0.50, 0.55, 1.00, 1.05, 0.0, 0.10]
    cases = []
    for d0 in thr:
        for k in range(-3, 4):
            for vcmd in (0.3, 0.0, -0.2):
                cases.append((d0, k, vcmd))
    S = len(cases)
    NS = T // 5 + 1
    tb = TickBatch(S, T, NS=NS)
    for s, (d0, k, vcmd) in enumerate(cases):
        tb.family[s] = f'B1 d={d0} k={k} v={vcmd}'
        x = ulps(0.3 + d0, k)
        for j in range(NS):
            # 스캔 열: 멀리 → 문턱 → 해제 시험 (뒤로 물러남)
            xx = [x + 0.5, x, x, x, ulps(0.3 + 0.5, k), ulps(0.3 + 0.55, k), x, x, x][j % 9]
            tb.scans[s, j] = wall_points(xx, B)
        for t in range(T):
            now = 1.0 + t * DT
            tb.now[s, t] = now
            tb.meas[s, t] = [1, 0.0, 0.0, now]
            v = vcmd if t < 25 else -vcmd
            tb.cmd[s, t] = [1, v, 0.0, now]
            if t % 5 == 0:
                tb.scanf[s, t] = [1, t // 5, now]
    out.append(('B1_front', tb))
    # ---- B2: 옆 벽 가드 거리 = g 가 0.02·0.07 ± k ulp, 그다음 멀어지는/다가가는 명령 (탈출 판정)
    cases = []
    for g0 in (0.02, 0.07, 0.0, 0.03):
        for k in range(-2, 3):
            for cmd in ((0.15, 0.0), (0.0, 0.0), (-0.15, 0.0), (0.1, 0.4), (0.1, -0.4)):
                for side in (1.0, -1.0):
                    cases.append((g0, k, cmd, side))
    S = len(cases)
    tb = TickBatch(S, T, NS=NS)
    for s, (g0, k, cmd, side) in enumerate(cases):
        tb.family[s] = f'B2 g={g0} k={k} cmd={cmd} side={side}'
        g = ulps(0.2 + g0, k) - 0.2
        for j in range(NS):
            tb.scans[s, j] = side_points(g if j < 5 else g + 0.06, B, side)
        for t in range(T):
            now = 2.0 + t * DT
            tb.now[s, t] = now
            tb.meas[s, t] = [1, cmd[0] if t > 10 else 0.0, 0.0, now]
            tb.cmd[s, t] = [1, cmd[0] if t > 5 else 0.0, cmd[1] if t > 5 else 0.0, now]
            if t % 5 == 0:
                tb.scanf[s, t] = [1, t // 5, now]
    out.append(('B2_guard', tb))
    # ---- B3: 도킹 예외 다각형 안 전방 벽 (예외 정지 거리 0.10, 해제 0.15) ± ulp, 다각형 유효 시간 0.3 s
    cases = []
    for d0 in (0.10, 0.15, 0.30, 0.05):
        for k in range(-2, 3):
            for live in (True, False):
                cases.append((d0, k, live))
    S = len(cases)
    tb = TickBatch(S, T, NS=NS, Pmax=4)
    for s, (d0, k, live) in enumerate(cases):
        tb.family[s] = f'B3 d={d0} k={k} live={live}'
        x = ulps(0.3 + d0, k)
        for j in range(NS):
            tb.scans[s, j] = wall_points(x if j < 6 else x + 0.4, B, -0.15, 0.15,
                                         extra=[(2.5, 0.0)])
        for t in range(T):
            now = 3.0 + t * DT
            tb.now[s, t] = now
            tb.meas[s, t] = [1, 0.1 if t < 20 else -0.1, 0.0, now]
            tb.cmd[s, t] = [1, 0.1 if t < 20 else -0.1, 0.0, now]
            if t % 5 == 0:
                tb.scanf[s, t] = [1, t // 5, now]
            if t % 5 == 1 and (live or t < 12):
                tb.excl[s, t] = [(0.0, -0.3), (2.0, -0.3), (2.0, 0.3), (0.0, 0.3)]
                tb.excl_h[s, t] = [1, now, 4]
    out.append(('B3_dock', tb))
    # ---- B4: TTC 상한 (기하 없는 TTC): τ 가 2.15 ± ulp, 명령 속도 대 v_ttc, TTC 나이 0.5 s 경계, 후퇴(점군 유무)
    cases = []
    for tau in (2.15, 1.0, 0.15, 0.0, 3.0):
        for k in (-1, 0, 1):
            for cloud in (True, False):
                for vcmd in (0.5, -0.3, 0.05):
                    cases.append((tau, k, cloud, vcmd))
    S = len(cases)
    tb = TickBatch(S, T, NS=NS, Cmax=4)
    for s, (tau, k, cloud, vcmd) in enumerate(cases):
        tb.family[s] = f'B4 tau={tau} k={k} cloud={cloud} v={vcmd}'
        tt = ulps(tau, k)
        for j in range(NS):
            tb.scans[s, j] = wall_points(2.5, B, extra=[(-0.2, 0.45), (0.1, -0.5)])
        for t in range(T):
            now = 4.0 + t * DT
            tb.now[s, t] = now
            tb.meas[s, t] = [1, vcmd, 0.0, now]
            tb.cmd[s, t] = [1, vcmd, 0.0, now]
            if t % 5 == 0:
                tb.scanf[s, t] = [1, t // 5, now]
            if t in (3, 8):
                tb.trk_h[s, t] = [2, now, now, 0, tt]
            if cloud and t % 3 == 0:
                tb.cloud_h[s, t] = [1, now, now, 0]
    out.append(('B4_ttc', tb))
    # ---- B5: 명령 낡음 (0.5 s), 측정 낡음 (0.2 s), 측정 저역 통과, 의도 유지 (명령 0 뒤 영역)
    cases = []
    for gap in (25, 24, 26, 10):
        for mgap in (10, 9, 11, 3):
            cases.append((gap, mgap))
    S = len(cases)
    tb = TickBatch(S, T + 20, NS=(T + 20) // 5 + 1)
    for s, (gap, mgap) in enumerate(cases):
        tb.family[s] = f'B5 gap={gap} mgap={mgap}'
        for j in range(tb.NS):
            tb.scans[s, j] = wall_points(0.3 + 0.45, B)
        for t in range(tb.T):
            now = 5.0 + t * DT
            tb.now[s, t] = now
            if not (10 <= t < 10 + mgap):
                tb.meas[s, t] = [1, 0.2, 0.0, now]
            if t < 8:
                tb.cmd[s, t] = [1, 0.2, 0.0, now]
            elif t >= 8 + gap:
                tb.cmd[s, t] = [1, 0.0, 0.0, now]
            if t % 5 == 0:
                tb.scanf[s, t] = [1, t // 5, now]
    out.append(('B5_timing', tb))
    # ---- B6: 깊이 점군 채널 — 낮은 물체만 점군에 (깊이 STOP·해제·점군 끊김 = depth_blind), 필수 점군 낡음 → 전진 상한
    cases = []
    for d0 in (0.30, 0.25, 0.50):
        for k in (-1, 0, 1):
            for blind in (False, True):
                cases.append((d0, k, blind))
    S = len(cases)
    tb = TickBatch(S, T, NS=NS, Cmax=8)
    for s, (d0, k, blind) in enumerate(cases):
        tb.family[s] = f'B6 d={d0} k={k} blind={blind}'
        x = ulps(0.3 + d0, k)
        for j in range(NS):
            tb.scans[s, j] = wall_points(3.0, B)
        for t in range(T):
            now = 6.0 + t * DT
            tb.now[s, t] = now
            tb.meas[s, t] = [1, 0.2 if t < 25 else -0.2, 0.0, now]
            tb.cmd[s, t] = [1, 0.2 if t < 25 else -0.2, 0.0, now]
            if t % 5 == 0:
                tb.scanf[s, t] = [1, t // 5, now]
            if t % 3 == 0 and not (blind and 15 <= t < 35):
                xx = x if t < 25 else x + 0.3
                tb.cloud[s, t, :5] = [(xx, -0.1), (xx, -0.05), (xx, 0.0), (xx, 0.05), (xx, 0.1)]
                tb.cloud_h[s, t] = [1, now, now, 5]
    out.append(('B6_depth', tb))
    # ---- B7: 몸 원통 안 양보 탈출 — 트랙 기하가 로봇 몸 원통 경계(0.761 ± 여유 0.10) 근처, 진행선에서 멀어짐/다가감
    cases = []
    for r0 in (0.5, 0.86, 0.861, 0.7):
        for vy in (0.6, -0.6, 0.0, 0.1):
            for cmd in ((0.4, 0.0), (-0.3, 0.0), (0.3, 0.8)):
                cases.append((r0, vy, cmd))
    S = len(cases)
    tb = TickBatch(S, T, NS=NS, Tmax=2, Cmax=2)
    for s, (r0, vy, cmd) in enumerate(cases):
        tb.family[s] = f'B7 r={r0} vy={vy} cmd={cmd}'
        for j in range(NS):
            tb.scans[s, j] = wall_points(2.8, B)
        for t in range(T):
            now = 7.0 + t * DT
            tb.now[s, t] = now
            tb.meas[s, t] = [1, 0.0 if t < 10 else cmd[0], 0.0, now]
            tb.cmd[s, t] = [1, cmd[0], cmd[1], now]
            if t % 5 == 0:
                tb.scanf[s, t] = [1, t // 5, now]
            if t % 3 == 0:
                tb.cloud_h[s, t] = [1, now, now, 0]
            if t % 5 == 2:
                tb.trk[s, t, 0] = [0.0, 1.0, 0.0, r0, 0.0, vy]
                tb.trk[s, t, 1] = [3.0, 0.0, 2.0, -1.0, 0.0, 0.0]
                tb.trk_h[s, t] = [1, now - 0.02, now, 2, 0]
    out.append(('B7_yield', tb))
    # ---- B8: 비원형 스캔 (270°, 541 빔) — 끝 빔 근처 창이 스캔 밖(+inf)을 포함, 가드 점이 스캔 양 끝에
    Bn = 541
    incn = math.radians(270.0) / 540
    cases = [(d0, k, g) for d0 in (0.30, 0.25, 0.50) for k in (-1, 0, 1) for g in (0.015, 0.03)]
    tb = TickBatch(len(cases), T, B=Bn, NS=NS, circular=False, inc=incn)
    for s, (d0, k, g) in enumerate(cases):
        tb.family[s] = f'B8 d={d0} k={k} g={g}'
        x = ulps(0.3 + d0, k)
        for j in range(NS):
            pts = wall_points(x if j < 6 else x + 0.3, Bn)
            pts[0] = (0.0, 0.2 + g)
            pts[1] = (0.02, 0.2 + g)
            pts[2] = (0.04, 0.2 + g)
            pts[-1] = (0.0, -0.2 - g)
            pts[-2] = (0.02, -0.2 - g)
            pts[-3] = (0.04, -0.2 - g)
            tb.scans[s, j] = pts
        for t in range(T):
            now = 8.0 + t * DT
            tb.now[s, t] = now
            tb.meas[s, t] = [1, 0.15 if t < 20 else -0.15, 0.0, now]
            tb.cmd[s, t] = [1, 0.15 if t < 20 else -0.15, 0.3 if t > 30 else 0.0, now]
            if t % 5 == 0:
                tb.scanf[s, t] = [1, t // 5, now]
    out.append(('B8_noncircular', tb))
    return out


# ======================================================================== seq 대조
def compare_seq(binary, tmp, tb, params_name, tally, device, group):
    """묶음 하나를 C++ 과 torch 로 돌려 비교."""
    params = G.GateParams(**{**G.DEPLOYED.__dict__, **CONFIGS[params_name]})
    inp = os.path.join(tmp, 'seq.in')
    out = os.path.join(tmp, 'seq.out')
    tb.pack(inp)
    extra = [f'{k}={int(v) if isinstance(v, bool) else v}' for k, v in
             {'yield_escape_enabled': params.yield_escape_enabled,
              'cloud_required': params.cloud_required}.items()]
    run_cpp(binary, 'seq', inp, out, extra)
    cpp = np.fromfile(out, dtype='<f8').reshape(tb.S, tb.T, len(OUT_NAMES))
    tor = replay(tb, params, device)
    for i, name in enumerate(OUT_NAMES):
        a, b = cpp[..., i], tor[..., i]
        ok = (a == b) if name in DISCRETE else close(a, b)

        def ctx(ix, tb=tb):
            return {'family': tb.family[ix[0]], 'tick': ix[1]}
        tally.add(group, name, ok, a, b, ctx)
    cov = {
        'ticks': int(tb.S * tb.T),
        'prox': int((cpp[..., 3] > 0).sum()), 'escaping': int((cpp[..., 5] > 0).sum()),
        'retreat': int((cpp[..., 6] > 0).sum()), 'yield': int((cpp[..., 7] > 0).sum()),
        'ttc_limited': int((cpp[..., 8] > 0).sum()), 'cloud_stale': int((cpp[..., 9] > 0).sum()),
        'command_stale': int((cpp[..., 10] > 0).sum()),
        'zone': {str(z): int((cpp[..., 2] == z).sum()) for z in range(4)},
        'causes': {str(c): int(((cpp[..., 4].astype(int) & c) != 0).sum()) for c in (1, 2, 4, 8)},
        'v_changed': int((np.abs(cpp[..., 0] - tb.cmd[..., 1]) > 1e-12).sum()),
    }
    return cov, cpp, tor


def merge_cov(acc, cov):
    """적용 범위 합."""
    for k, v in cov.items():
        if isinstance(v, dict):
            acc.setdefault(k, {})
            for kk, vv in v.items():
                acc[k][kk] = acc[k].get(kk, 0) + vv
        else:
            acc[k] = acc.get(k, 0) + v


def main():
    """교차 시험 전체."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', required=True)
    ap.add_argument('--seed', type=int, default=20261006)
    ap.add_argument('--quick', action='store_true', help='변이 검사용 작은 묶음')
    ap.add_argument('--fp_motions', type=int, default=1000)
    ap.add_argument('--fp_points', type=int, default=200)
    ap.add_argument('--win_arrays', type=int, default=20000)
    ap.add_argument('--random_scenarios', type=int, default=3000)
    ap.add_argument('--batch', type=int, default=250)
    ap.add_argument('--ticks', type=int, default=50)
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--cuda_check', action='store_true', help='CPU 결과와 CUDA 결과도 비교')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    if a.quick:
        a.fp_motions, a.fp_points, a.win_arrays, a.random_scenarios = 200, 100, 2000, 250
    torch.set_num_threads(min(8, os.cpu_count() or 1))
    rng = np.random.default_rng(a.seed)
    tally = Tally()
    t0 = time.time()
    report = {'seed': a.seed, 'device': a.device, 'quick': a.quick}
    with tempfile.TemporaryDirectory(dir=os.environ.get('XCHECK_TMP')) as tmp:
        report['fp'] = check_fp(a.binary, tmp, rng, a.fp_motions, a.fp_points, tally, a.device)
        report['win'] = check_win(a.binary, tmp, rng, a.win_arrays, tally, a.device)
        cov_b = {}
        nb = 0
        for cfg in CONFIGS:
            for name, tb in gen_boundary(rng, a.device, cfg):
                cov, _c, _t = compare_seq(a.binary, tmp, tb, cfg, tally, a.device,
                                          f'seq_boundary/{cfg}')
                merge_cov(cov_b, cov)
                nb += tb.S
        report['seq_boundary'] = {'scenarios': nb, 'coverage': cov_b}
        cov_r = {}
        nr = 0
        names = list(CONFIGS)
        fam = {}
        k = 0
        while nr < a.random_scenarios:
            cfg = names[k % len(names)] if k % 5 >= 3 else 'deployed'
            k += 1
            params = G.GateParams(**{**G.DEPLOYED.__dict__, **CONFIGS[cfg]})
            S = min(a.batch, a.random_scenarios - nr)
            tb = gen_random_batch(rng, S, a.ticks, params, a.device)
            cov, cpp, tor = compare_seq(a.binary, tmp, tb, cfg, tally, a.device,
                                        f'seq_random/{cfg}')
            merge_cov(cov_r, cov)
            for f in tb.family:
                fam[f] = fam.get(f, 0) + 1
            if a.cuda_check and torch.cuda.is_available() and nr == 0:
                tg = replay(tb, params, 'cuda')
                for i, name in enumerate(OUT_NAMES):
                    ok = (tor[..., i] == tg[..., i]) if name in DISCRETE else \
                        close(tor[..., i], tg[..., i])
                    tally.add('cuda_vs_cpu', name, ok, tor[..., i], tg[..., i])
                report['cuda_check'] = {'scenarios': S, 'gpu': torch.cuda.get_device_name(0)}
            nr += S
        report['seq_random'] = {'scenarios': nr, 'ticks_per_scenario': a.ticks,
                                'world_families': fam, 'coverage': cov_r}
    report.update(tally.summary())
    report['wall_s'] = round(time.time() - t0, 1)
    txt = json.dumps(report, indent=1, ensure_ascii=False, default=str)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(txt)
    print(json.dumps({k: report[k] for k in ('total_checks', 'total_mismatches', 'mismatches',
                                             'wall_s')}, ensure_ascii=False))
    return 0 if report['total_mismatches'] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
