"""계약 State(numpy 기준 구현) ↔ BState(torch 배치) 변환 — golden 재생과 env 무작위 대조에 쓴다."""

import math

import torch

from ._contract import C
from .contract_torch import BState, path_tensors

from amr_rl_contract.state import DevState, SafetyState, State, Track  # noqa: E402


def _inf(v):
    return math.inf if v is None else float(v)


def bstate_from_states(states, device='cpu', dtype=torch.float64, scan_dtype=None, n_tracks=None):
    """State 목록 → BState."""
    scan_dtype = scan_dtype or dtype
    N = len(states)
    pts, n, cum, length = path_tensors([s.path for s in states], device, dtype=dtype)
    T = n_tracks if n_tracks is not None else max([len(s.tracks) for s in states] + [1])
    trk = torch.zeros(N, T, 4, dtype=dtype)
    dyn = torch.zeros(N, T, dtype=torch.bool)
    valid = torch.zeros(N, T, dtype=torch.bool)
    ids = torch.zeros(N, T, dtype=torch.long)
    for i, s in enumerate(states):
        for j, t in enumerate(s.tracks):
            trk[i, j] = torch.tensor([t.x, t.y, t.vx, t.vy], dtype=dtype)
            dyn[i, j] = bool(t.is_dynamic)
            valid[i, j] = True
            ids[i, j] = int(t.id)

    def col(f, dt=dtype):
        return torch.tensor([f(s) for s in states], dtype=dt, device=device)

    ranges = torch.tensor([[_inf(r) for r in s.scan_ranges] for s in states], dtype=scan_dtype)

    def prev_of(s):
        pa = list(s.prev_actions)[:C.N_PREV_ACTIONS]
        return [list(a) for a in pa + [(0.0, 0.0)] * (C.N_PREV_ACTIONS - len(pa))]
    prev = torch.tensor([prev_of(s) for s in states], dtype=dtype)
    return BState(
        pose=torch.tensor([list(s.pose) for s in states], dtype=dtype, device=device),
        path_pts=pts, path_n=n, path_cum=cum, path_len=length,
        ranges=ranges.to(device),
        v_meas=col(lambda s: s.v_meas), w_meas=col(lambda s: s.w_meas),
        a_v_meas=col(lambda s: s.a_v_meas), a_w_meas=col(lambda s: s.a_w_meas),
        v_cmd=col(lambda s: s.v_cmd), w_cmd=col(lambda s: s.w_cmd),
        prev_actions=prev.to(device), v_cap=col(lambda s: s.v_cap),
        scan_age=col(lambda s: s.scan_age), track_age=col(lambda s: s.track_age),
        trk=trk.to(device), trk_dyn=dyn.to(device), trk_valid=valid.to(device),
        trk_id=ids.to(device),
        zone=col(lambda s: s.safety.zone, torch.long),
        speed_limit=col(lambda s: _inf(s.safety.speed_limit)),
        proximity_stop=col(lambda s: s.safety.proximity_stop, torch.bool),
        min_ttc=col(lambda s: _inf(s.safety.min_ttc)),
        dev_in=col(lambda s: s.dev.in_dev, torch.bool),
        dev_peak=col(lambda s: s.dev.peak), dev_t=col(lambda s: s.dev.t_since_peak),
    )


def states_from_bstate(b, idx=None):
    """배치 상태 → State 목록 (idx 의 env 만). 무한대는 None 으로."""
    N = b.pose.shape[0]
    idx = range(N) if idx is None else idx
    out = []

    def fin(x):
        x = float(x)
        return None if not math.isfinite(x) else x
    for i in idx:
        n = int(b.path_n[i])
        path = [tuple(map(float, p)) for p in b.path_pts[i, :n].cpu().tolist()]
        tracks = []
        for j in range(b.trk.shape[1]):
            if bool(b.trk_valid[i, j]):
                x, y, vx, vy = (float(v) for v in b.trk[i, j].cpu().tolist())
                tracks.append(Track(int(b.trk_id[i, j]), x, y, vx, vy, bool(b.trk_dyn[i, j])))
        out.append(State(
            pose=tuple(float(v) for v in b.pose[i].cpu().tolist()), path=path,
            scan_ranges=[fin(r) for r in b.ranges[i].cpu().tolist()],
            v_meas=float(b.v_meas[i]), w_meas=float(b.w_meas[i]),
            a_v_meas=float(b.a_v_meas[i]), a_w_meas=float(b.a_w_meas[i]),
            v_cmd=float(b.v_cmd[i]), w_cmd=float(b.w_cmd[i]),
            prev_actions=[tuple(float(v) for v in a) for a in b.prev_actions[i].cpu().tolist()],
            v_cap=float(b.v_cap[i]), scan_age=float(b.scan_age[i]), tracks=tracks,
            track_age=float(b.track_age[i]),
            safety=SafetyState(int(b.zone[i]), fin(b.speed_limit[i]), bool(b.proximity_stop[i]),
                               fin(b.min_ttc[i])),
            dev=DevState(bool(b.dev_in[i]), float(b.dev_peak[i]), float(b.dev_t[i]))))
    return out
