"""
에피소드 지표 (rclpy 비의존) — 실행기가 모은 시계열(sim 시각)에서 gzbench/v1 행을 계산한다.

정의는 결과를 보기 전에 노트 A5-2 (`notebook/entries/*_b1-gzbench_harness-scenarios.md`) 에 적고 커밋했다.
시각은 모두 노드 시계(sim) [s]. 에피소드 시작 t0 = navigate_to_pose 목표가 수락된 시각.

층위 성공 (A3 재판정의 Isaac 정의를 Gazebo 로 옮긴 것):
  - strict (계약 §8 성공 사건): 목표 xy ≤ 0.10 m 이고 |v| ≤ 0.05 m/s 인 표본이 (계약 종단 사건 전에) 있다.
    _gt = 지면 진실 자세·속도, _est = 위치추정(TF map→base) 자세 + 지면 진실 속도.
    두 기준 모두 같은 종단 순서 규칙을 쓴다 (A5 리뷰 차단 1 의 (b)): {접촉, 그 기준의 엄격 성공, 정체, 시간 상한} 중 시간순 첫째가
    성공이어야 한다 — 접촉·정체·상한은 물리 사건이라 두 기준 모두 지면 진실로 본다. 추정 기준 결과는 outcome_contract_est.
  - 추정 자세 시계열 (A5 리뷰 차단 1 의 (a)): 실제 표본 사이 틈이 EST_MAX_GAP 이하인 곳에서만 선형 보간하고, 표본 구간 밖과
    더 긴 틈은 미정(nan — 판정에 쓰지 않는다). 최종 추정 final.est 는 그 측정 시각(final.t, 없으면 기록 끝 = 마지막 지면 진실
    시각)의 표본으로 덧붙인다. 실행기가 정착 구간을 표집하기 전의 원자료(A5-4 스모크)에서는 정착 구간의 추정 판정이 이 끝 한
    점으로만 이뤄진다 (근사 — est_end_gap_s 가 그 틈).
  - nav2compat: 목표 xy ≤ 0.25 m 인 표본이 접촉 전에 있다 (속도·yaw 무관, Nav2 SimpleGoalChecker 기본값).
  - system_success: Nav2 navigate_to_pose 가 SUCCEEDED (배포 goal checker: 추정 자세 xy 0.10·yaw 0.05).
  - final_*: 에피소드가 끝나고 정착 시간 뒤의 최종 거리·yaw 오차·속도 (분포로 보고).
계약 종단 사건 (outcome_contract — 결과 CSV episode/v1 의 outcome): 시간순 첫 사건
  collision (정적 기하 부호 거리 < 0 또는 collision_monitor 접촉 증가) → success (strict_gt) → stall
  (SimpleProgressChecker 규칙: 기준점에서 0.3 m 넘게 움직이면 기준 갱신, 기준 뒤 20 s 면 정체) → timeout
  (t − t0 > 시간 상한). 실행기·시뮬레이터 고장은 error (분석에서 빼지 않고 센다).
  접촉·성공 사건은 Nav2 가 끝난 뒤 정착 구간(기록 끝, 시간 상한 안)까지 본다 — Nav2 goal checker 는 속도를 보지 않아 DWA 가
  움직이는 채로 "성공" 을 내고 그 뒤에 선다 (A5-4 스모크에서 찾음: 그 에피소드가 'timeout' 으로 잘못 찍혔다).
  사건 없이 시스템이 먼저 끝났으면 (Nav2 성공·중단 뒤 계약 성공 조건 밖에 섰다) 로봇은 더 움직이지 않으므로 stall 로 적고
  contract_note = 'ended_without_success'. 시간 상한까지 갔으면 timeout.
"""

import math

import numpy as np

STRICT_XY = 0.10
STRICT_V = 0.05
NAV2_XY = 0.25
STALL_RADIUS = 0.3
STALL_TIME = 20.0
DEV_OUT, DEV_BACK = 0.30, 0.15          # 이탈 구간 (tests/integration/amr_itest/metrics.py 와 같다)
NEAR_GOAL_LO, NEAR_GOAL_HI = STRICT_XY, 0.50
STOP_V, STOP_W = 0.02, 0.05             # 정지 판정 (A3 eval_metrics MOVE_V·MOVE_W 와 같다)
MOVING_V = 0.05                         # CTE 표본: 이보다 빠를 때만
INTERVENTION_TOL = 0.01                 # PROTOCOL.md §5: |Δv| > 0.01 m/s 또는 |Δω| > 0.01 rad/s
# 출력과 짝지을 입력의 시각 창 [s]: safety_node 는 50 Hz 타이머로 직전 입력을 내므로 출력은 입력보다 약 20 ms 늦게,
# 그리고 기록 노드의 수신 시각은 두 토픽 사이에서 앞뒤가 바뀔 수 있다 (A5 흔들기 2: 같은 시각에 받은 다음 입력과 짝지어
# 램프 중 거짓 개입 31 건). 창 안의 입력 하나라도 출력과 같으면 개입이 아니다.
INTERVENTION_LAG = 0.06
INTERVENTION_LEAD = 0.005
CONTACT_RELEASE = 0.05                  # 접촉 사건 분리 (collision_monitor contact_release 와 같다)
LOC_COLLAPSE = 0.5                      # 위치추정 붕괴 문턱 (b1-plugin 다음 ④)
ROTATING_W = 0.1                        # goal checker 가 회전 중 성공을 낸 것 (b1-plugin 다음 ⑨)
# 추정 자세 보간 최대 틈 [s sim] — 실행기 감시 루프 표본 간격 p99 0.03, 최대 0.165 s (A5-4 원자료 49 에피소드)
EST_MAX_GAP = 0.25


def wrap(a):
    """[-π, π)."""
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def quantiles(x, ps=(50, 95)):
    """백분위 목록 (빈 입력 = None 들)."""
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return [None for _ in ps]
    return [float(np.percentile(x, p)) for p in ps]


def path_length(xy):
    """폴리라인 길이."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if len(xy) > 1 else 0.0


def polyline_distance(points, px, py):
    """점 ↔ 폴리라인 최단 거리 (tests/integration/amr_itest/worldmap.polyline_distance 와 같다)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return math.inf
    if len(pts) == 1:
        return float(math.hypot(px - pts[0, 0], py - pts[0, 1]))
    a, b = pts[:-1], pts[1:]
    ab = b - a
    den = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    t = np.clip(((px - a[:, 0]) * ab[:, 0] + (py - a[:, 1]) * ab[:, 1]) / den, 0.0, 1.0)
    return float(np.min(np.hypot(px - (a[:, 0] + t * ab[:, 0]), py - (a[:, 1] + t * ab[:, 1]))))


def deviation_episodes(times, devs, out_thr=DEV_OUT, back_thr=DEV_BACK):
    """(t_start, t_peak, peak, t_end|None) 목록 — amr_itest.metrics.deviation_episodes 와 같은 상태기계."""
    out, cur = [], None
    for t, d in zip(times, devs):
        if not math.isfinite(d):
            continue
        if cur is None:
            if d > out_thr:
                cur = [t, t, d]
            continue
        if d > cur[2]:
            cur[1], cur[2] = t, d
        if d <= back_thr:
            out.append((cur[0], cur[1], cur[2], t))
            cur = None
    if cur is not None:
        out.append((cur[0], cur[1], cur[2], None))
    return out


def first_stall(t, xy, t0, radius=STALL_RADIUS, window=STALL_TIME):
    """정체 판정(Nav2 SimpleProgressChecker 규칙)의 첫 정체 시각 (없으면 None). t·xy 는 시간순."""
    if len(t) == 0:
        return None
    base_t, base = t0, None
    for ti, p in zip(t, xy):
        if ti < t0:
            continue
        if base is None:
            base, base_t = p, ti
            continue
        if math.hypot(p[0] - base[0], p[1] - base[1]) > radius:
            base, base_t = p, ti
        elif ti - base_t > window:
            return float(ti)
    return None


def contact_events(t, clearance, release=CONTACT_RELEASE):
    """부호 거리 시계열 → 접촉 사건 시각 목록 (< 0 으로 들어간 순간; 거리가 release 이상 벌어져야 다음 사건)."""
    ev, inside = [], False
    for ti, d in zip(t, clearance):
        if not math.isfinite(d):
            continue
        if not inside and d < 0.0:
            ev.append(float(ti))
            inside = True
        elif inside and d >= release:
            inside = False
    return ev


def interventions(t_in, v_in, w_in, t_out, v_out, w_out, tol=INTERVENTION_TOL,
                  lag=INTERVENTION_LAG, lead=INTERVENTION_LEAD):
    """
    안전 노드 개입 구간을 센다 (PROTOCOL.md §5).

    출력 표본마다 시각 창 [t − lag, t + lead] 안의 입력과 비교해, 어느 입력과도 |Δv| ≤ tol 이고 |Δω| ≤ tol 이
    아니면 그 출력은 바뀐 것이다. 바뀐 출력이 연속된 구간 하나 = 1 회. 반환 (횟수, 총 시간 [s], 구간 목록).
    """
    t_in = np.asarray(t_in, dtype=float)
    v_in = np.asarray(v_in, dtype=float)
    w_in = np.asarray(w_in, dtype=float)
    if t_in.size == 0 or len(t_out) == 0:
        return 0, 0.0, []
    to_arr = np.asarray(t_out, dtype=float)
    lo = np.searchsorted(t_in, to_arr - lag, side='left')
    hi = np.searchsorted(t_in, to_arr + lead, side='right')
    segs, cur = [], None
    for k, (to, vo, wo) in enumerate(zip(t_out, v_out, w_out)):
        a, b = lo[k], hi[k]
        if b <= a:
            diff = False      # 짝지을 입력이 없다 (기록 시작·끝) — 판정하지 않는다
        else:
            same = (np.abs(vo - v_in[a:b]) <= tol) & (np.abs(wo - w_in[a:b]) <= tol)
            diff = not bool(same.any())
        if diff and cur is None:
            cur = [to, to]
        elif diff:
            cur[1] = to
        elif cur is not None:
            segs.append((cur[0], to))
            cur = None
    if cur is not None:
        segs.append((cur[0], float(t_out[-1])))
    return len(segs), float(sum(b - a for a, b in segs)), segs


def jerk_rms(t, x):
    """2 차 차분 / dt² 의 RMS (같은 시각 표본은 뺀다)."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    keep = np.concatenate([[True], np.diff(t) > 1e-6]) if t.size else np.array([], bool)
    t, x = t[keep], x[keep]
    if t.size < 3:
        return None
    v = np.diff(x) / np.diff(t)
    tm = 0.5 * (t[1:] + t[:-1])
    a = np.diff(v) / np.diff(tm)
    ta = 0.5 * (tm[1:] + tm[:-1])
    j = np.diff(a) / np.diff(ta) if a.size > 1 else np.array([])
    return float(np.sqrt(np.mean(j * j))) if j.size else None


def zone_times(t_zone, zone, t_start, t_end):
    """존(0 CLEAR, 1 WARNING, 2 CRITICAL, 3 STOP) 지속 시간 [s] — 래치 토픽이라 구간 시작 전 마지막 값부터."""
    out = {1: 0.0, 2: 0.0, 3: 0.0}
    if len(t_zone) == 0 or t_end <= t_start:
        return out
    t_zone = np.asarray(t_zone, dtype=float)
    k = int(np.searchsorted(t_zone, t_start, side='right')) - 1
    cur = int(zone[k]) if k >= 0 else 0
    t_prev = t_start
    for ti, z in zip(t_zone[k + 1:], zone[k + 1:]):
        if ti >= t_end:
            break
        if cur in out:
            out[cur] += ti - t_prev
        cur, t_prev = int(z), ti
    if cur in out:
        out[cur] += t_end - t_prev
    return out


def near_goal_stop(t, dist, v, w):
    """목표 0.10–0.50 m 안에서 정지(|v| ≤ 0.02, |ω| ≤ 0.05)한 sim 시간의 합과 가장 긴 연속 구간 [s]."""
    total, longest, run, t_prev, stopped_prev = 0.0, 0.0, 0.0, None, False
    for ti, d, vi, wi in zip(t, dist, v, w):
        stopped = NEAR_GOAL_LO < d <= NEAR_GOAL_HI and abs(vi) <= STOP_V and abs(wi) <= STOP_W
        if t_prev is not None and stopped and stopped_prev:
            total += ti - t_prev
            run += ti - t_prev
            longest = max(longest, run)
        if not stopped:
            run = 0.0
        t_prev, stopped_prev = ti, stopped
    return total, longest


def _interp_pose(track_t, track, t, max_gap=EST_MAX_GAP):
    """
    자세 시계열을 시각 t 로 (선형, yaw 는 감아서) — 표본 구간 안이고 둘러싼 표본 틈이 max_gap 이하인 곳만.

    표본 시각과 같으면 틈과 무관하게 그 표본. 그 밖(구간 밖·긴 틈)은 nan — np.interp 의 끝값 고정을 쓰지 않는다.
    반환 (x, y, yaw, valid). track_t 는 오름차순.
    """
    track_t = np.asarray(track_t, dtype=float)
    track = np.asarray(track, dtype=float).reshape(-1, 3)
    t = np.asarray(t, dtype=float)
    if track_t.size == 0:
        nan = np.full(t.shape, np.nan)
        return nan, nan.copy(), nan.copy(), np.zeros(t.shape, bool)
    x = np.interp(t, track_t, track[:, 0])
    y = np.interp(t, track_t, track[:, 1])
    yaw = wrap(np.interp(t, track_t, np.unwrap(track[:, 2])))
    n = track_t.size
    i = np.searchsorted(track_t, t, side='left')            # track_t[i-1] < t <= track_t[i]
    exact = (i < n) & (track_t[np.minimum(i, n - 1)] == t)
    j = np.clip(i, 1, max(1, n - 1))
    gap = track_t[np.minimum(j, n - 1)] - track_t[j - 1] if n > 1 else np.full(t.shape, np.inf)
    valid = (t >= track_t[0]) & (t <= track_t[-1]) & (exact | (gap <= max_gap))
    return (np.where(valid, x, np.nan), np.where(valid, y, np.nan), np.where(valid, yaw, np.nan),
            valid)


def contract_first(t_contact, t_success, t_stall, t_timeout):
    """
    계약 종단 사건 중 시간순 첫째 → (시각, 이름) 또는 None.

    같은 시각이면 접촉 → 성공 → 정체 → 시간 상한 순으로 이긴다. 정답·추정 두 기준이 같은 함수를 쓴다 (리뷰 차단 1 (b)).
    """
    ev = [(t, k, name) for k, (t, name) in enumerate(
        ((t_contact, 'collision'), (t_success, 'success'), (t_stall, 'stall'),
         (t_timeout, 'timeout'))) if t is not None]
    if not ev:
        return None
    t, _, name = min(ev)
    return t, name


def episode_metrics(ep):
    """
    에피소드 기록 → 지표 dict.

    ep 키 (sim 시각 기준 목록):
      goal (x, y, yaw), t0, t_end (Nav2 종료 또는 취소 시각), t_limit (시간 상한 [s]), nav2_status (int|None),
      cancel_reason ('' | 'timeout' | 'collision' | 'watchdog'),
      gt: [[t, x, y, yaw, v, w], …]  지면 진실 (ground_truth/odom, 자세는 월드 = map),
      est: [[t, x, y, yaw], …]       TF map→base 표본,
      clear_static: [[t, d], …]      정적 기하 풋프린트 부호 거리 (지면 진실 자세),
      cm_contacts: [[t, n], …]       collision_monitor 누적 접촉 수, cm_min: [[t, d], …],
      plans: [[t, [[x, y], …]], …]   전역 경로 (plan 토픽),
      cmd_in: [[t, v, w], …] (cmd_vel_smoothed), cmd_out: [[t, v, w], …] (cmd_vel),
      cmd_nav: [[t, t_wall, v, w], …] (cmd_vel_nav),
      zone: [[t, z], …], lost: [[t, b], …], stats: [[t, t_wall, row], …] (RL 플러그인),
      scan_stamps: [[t_recv, t_stamp], …],
      final: {'gt': [x, y, yaw, v, w], 'est': [x, y, yaw], 't': 잰 sim 시각 (없으면 기록 끝)} (정착 뒤),
      wall: {'t0', 't_end'}.
    """
    gx, gy, gyaw = ep['goal']
    t0, t_end, t_limit = ep['t0'], ep['t_end'], ep['t_limit']
    t_lim = t0 + t_limit
    gt_raw = np.asarray(ep.get('gt') or np.zeros((0, 6)), dtype=float).reshape(-1, 6)
    # 사건 판정 구간 = [t0, 기록 끝] ∩ 시간 상한 (정착 포함), 주행 지표 구간 = [t0, t_end]
    gta = gt_raw[(gt_raw[:, 0] >= t0) & (gt_raw[:, 0] <= t_lim)] if gt_raw.size else gt_raw
    gt = gta[gta[:, 0] <= t_end] if gta.size else gta
    t_rec = float(gta[-1, 0]) if gta.shape[0] else t_end
    est_raw = np.asarray(ep.get('est') or np.zeros((0, 4)), dtype=float).reshape(-1, 4)
    fin = ep.get('final') or {}
    fg, fe = fin.get('gt'), fin.get('est')
    # 최종 추정은 잰 시각의 표본으로 (실행기 final.t, 옛 원자료는 기록 끝 = 마지막 지면 진실 시각)
    t_fin = fin.get('t')
    if t_fin is None and gt_raw.shape[0]:
        t_fin = float(gt_raw[-1, 0])
    t_rec_end = float(gt_raw[-1, 0]) if gt_raw.shape[0] else t_end
    m = {'est_end_gap_s': float(t_rec_end - est_raw[-1, 0]) if est_raw.shape[0] else None}
    if fe and t_fin is not None and (est_raw.shape[0] == 0 or t_fin > est_raw[-1, 0] + 1e-9):
        est_raw = np.vstack([est_raw, [[float(t_fin), fe[0], fe[1], fe[2]]]])
    esta = est_raw[(est_raw[:, 0] >= t0) & (est_raw[:, 0] <= t_lim)] if est_raw.size else est_raw
    est = esta[esta[:, 0] <= t_end] if esta.size else esta
    tg = gt[:, 0]
    d_gt = np.hypot(gt[:, 1] - gx, gt[:, 2] - gy)
    speed = np.abs(gt[:, 4])
    tga = gta[:, 0]
    d_gta = np.hypot(gta[:, 1] - gx, gta[:, 2] - gy)
    speeda = np.abs(gta[:, 4])
    # --- 접촉 (정적 기하 + collision_monitor) — 정착 구간 포함
    cs = np.asarray(ep.get('clear_static') or np.zeros((0, 2)), dtype=float).reshape(-1, 2)
    cs = cs[(cs[:, 0] >= t0) & (cs[:, 0] <= t_lim)] if cs.size else cs
    static_ev = contact_events(cs[:, 0], cs[:, 1]) if cs.size else []
    cmc = np.asarray(ep.get('cm_contacts') or np.zeros((0, 2)), dtype=float).reshape(-1, 2)
    dyn_ev = []
    if cmc.size:
        base = cmc[cmc[:, 0] <= t0, 1]
        n0 = base[-1] if base.size else cmc[0, 1]
        inc = cmc[(cmc[:, 0] > t0) & (cmc[:, 0] <= min(t_lim, max(t_rec, t_end)))]
        prev = n0
        for ti, n in inc:
            if n > prev:
                dyn_ev += [float(ti)] * int(n - prev)
            prev = max(prev, n)
    m['contacts_static'] = len(static_ev)
    m['contacts_dynamic'] = len(dyn_ev)
    t_contact = min(static_ev + dyn_ev) if (static_ev or dyn_ev) else None
    m['min_clear_static_m'] = float(cs[:, 1].min()) if cs.size else None
    cmm = np.asarray(ep.get('cm_min') or np.zeros((0, 2)), dtype=float).reshape(-1, 2)
    cmm = cmm[(cmm[:, 0] >= t0) & (cmm[:, 0] <= t_end)] if cmm.size else cmm
    m['min_clear_dynamic_m'] = float(cmm[:, 1].min()) if cmm.size else None
    # --- 성공 사건 (정착 구간 포함)
    strict_gt = (d_gta <= STRICT_XY) & (speeda <= STRICT_V)
    t_strict_gt = float(tga[strict_gt][0]) if strict_gt.any() else None
    t_strict_est = None
    d_est_at_gt = None
    if esta.shape[0] and gta.shape[0]:
        ex, ey, _, _ = _interp_pose(esta[:, 0], esta[:, 1:4], tga)
        d_est_at_gt = np.hypot(ex - gx, ey - gy)            # 미정 = nan → 아래 비교는 거짓
        se = (d_est_at_gt <= STRICT_XY) & (speeda <= STRICT_V)
        t_strict_est = float(tga[se][0]) if se.any() else None
    t_stall = first_stall(tg, gt[:, 1:3], t0) if gt.shape[0] else None
    # --- 계약 종단 사건: 시간순 첫 (접촉 → 성공 → 정체 → 시간 상한). 같은 시각이면 접촉이 이긴다
    reason = ep.get('cancel_reason') or ''
    t_timeout = t_lim if (t_end >= t_lim - 1e-3 or reason == 'timeout') else None
    if reason == 'watchdog':
        fallback = (t_end, 'error', None)
    else:
        fallback = (t_end, 'stall', 'ended_without_success')

    def judge(t_success):
        first = contract_first(t_contact, t_success, t_stall, t_timeout)
        return (first[0], first[1], None) if first else fallback
    t_c, outcome, m['contract_note'] = judge(t_strict_gt)
    m['outcome_contract'] = outcome
    m['outcome_contract_est'] = judge(t_strict_est)[1]
    m['t_contract_s'] = float(t_c - t0)
    m['t_strict_gt_s'] = None if t_strict_gt is None else t_strict_gt - t0
    m['t_strict_est_s'] = None if t_strict_est is None else t_strict_est - t0
    m['t_stall_s'] = None if t_stall is None else t_stall - t0
    m['strict_gt'] = t_strict_gt is not None and outcome == 'success'
    m['strict_est'] = t_strict_est is not None and m['outcome_contract_est'] == 'success'
    pre = tga < (t_contact if t_contact is not None else math.inf)
    m['nav2compat_gt'] = bool(np.any(d_gta[pre] <= NAV2_XY)) if d_gta.size else False
    m['nav2compat_est'] = bool(np.any(d_est_at_gt[pre] <= NAV2_XY)) \
        if d_est_at_gt is not None else False
    m['min_dist_gt_m'] = float(d_gta.min()) if d_gta.size else None
    m['contacts_contract'] = len([x for x in static_ev + dyn_ev if x <= t0 + m['t_contract_s']])
    # --- 시스템 결과
    st = ep.get('nav2_status')
    reason = ep.get('cancel_reason') or ''
    m['nav2_status'] = st
    m['system_success'] = st == 4
    m['outcome_system'] = ('succeeded' if st == 4 else 'aborted' if st == 6 else
                           reason if reason else 'canceled' if st == 5 else 'error')
    m['nav_time_s'] = float(t_end - t0)
    m['final_dist_gt_m'] = math.hypot(fg[0] - gx, fg[1] - gy) if fg else None
    m['final_yaw_err_gt'] = abs(float(wrap(fg[2] - gyaw))) if fg else None
    m['final_speed_gt'] = abs(fg[3]) if fg else None
    m['final_dist_est_m'] = math.hypot(fe[0] - gx, fe[1] - gy) if fe else None
    m['final_yaw_err_est'] = abs(float(wrap(fe[2] - gyaw))) if fe else None
    m['final_ok_gt'] = bool(fg and m['final_dist_gt_m'] <= STRICT_XY and
                            m['final_speed_gt'] <= STRICT_V)
    m['final_ok_est'] = bool(fe and m['final_dist_est_m'] <= STRICT_XY)
    m['loc_err_end_m'] = math.hypot(fg[0] - fe[0], fg[1] - fe[1]) if (fg and fe) else None
    m['loc_err_end_yaw'] = abs(float(wrap(fg[2] - fe[2]))) if (fg and fe) else None
    # goal checker 가 회전 중에 성공을 냈나: Nav2 성공 시각 근처 |ω_gt|
    m['success_while_rotating'] = None
    if st == 4 and gt.shape[0]:
        k = int(np.argmin(np.abs(tg - t_end)))
        m['success_while_rotating'] = bool(abs(gt[k, 5]) > ROTATING_W)
        m['w_at_success'] = float(gt[k, 5])
    # --- 주행
    m['path_length_m'] = path_length(gt[:, 1:3]) if gt.shape[0] else None
    m['v_max_gt'] = float(speed.max()) if speed.size else None
    m['w_max_gt'] = float(np.abs(gt[:, 5]).max()) if gt.shape[0] else None
    plans = ep.get('plans') or []
    m['n_plans'] = len(plans)
    mv = (speed > MOVING_V) & (tg <= t0 + m['t_contract_s'])
    if plans and gt.shape[0]:
        first = np.asarray(plans[0][1], dtype=float).reshape(-1, 2)
        m['plan_first_m'] = path_length(first)
        e_first = np.array([polyline_distance(first, x, y) for x, y in gt[mv, 1:3]])
        pt = np.array([p[0] for p in plans])
        e_act = []
        for ti, x, y in gt[mv, 0:3]:
            k = max(0, int(np.searchsorted(pt, ti, side='right')) - 1)
            e_act.append(polyline_distance(plans[k][1], x, y))
        e_act = np.asarray(e_act)
        for tag, e in (('first', e_first), ('active', e_act)):
            q = quantiles(e, (50, 95))
            m[f'cte_{tag}_mean_m'] = float(e.mean()) if e.size else None
            m[f'cte_{tag}_p95_m'] = q[1]
            m[f'cte_{tag}_max_m'] = float(e.max()) if e.size else None
        dev = deviation_episodes(gt[mv, 0], e_act)
        peak = float(e_act.max()) if e_act.size else 0.0
        m['max_dev_m'] = float(max([d[2] for d in dev] + [peak]))
        t_last = float(gt[mv, 0][-1]) if mv.any() else t0
        m['return_max_s'] = float(max([(d[3] if d[3] is not None else t_last) - d[1]
                                       for d in dev] + [0.0]))
        m['dev_episodes'] = len(dev)
    # --- safety
    ci = np.asarray(ep.get('cmd_in') or np.zeros((0, 3)), dtype=float).reshape(-1, 3)
    co = np.asarray(ep.get('cmd_out') or np.zeros((0, 3)), dtype=float).reshape(-1, 3)
    ci = ci[(ci[:, 0] >= t0) & (ci[:, 0] <= t_end)] if ci.size else ci
    co = co[(co[:, 0] >= t0) & (co[:, 0] <= t_end)] if co.size else co
    n_iv, t_iv, _ = interventions(ci[:, 0], ci[:, 1], ci[:, 2], co[:, 0], co[:, 1], co[:, 2]) \
        if (ci.size and co.size) else (0, 0.0, [])
    m['interventions'] = n_iv
    m['intervention_time_s'] = t_iv
    m['jerk_lin_rms'] = jerk_rms(co[:, 0], co[:, 1]) if co.size else None
    m['jerk_ang_rms'] = jerk_rms(co[:, 0], co[:, 2]) if co.size else None
    zt = np.asarray(ep.get('zone') or np.zeros((0, 2)), dtype=float).reshape(-1, 2)
    z = zone_times(zt[:, 0], zt[:, 1], t0, t_end) if zt.size else {1: 0.0, 2: 0.0, 3: 0.0}
    m['zone_warning_s'], m['zone_critical_s'], m['zone_stop_s'] = z[1], z[2], z[3]
    # --- 목표 직전 정지
    if gt.shape[0]:
        tot, lng = near_goal_stop(tg, d_gt, gt[:, 4], gt[:, 5])
        m['near_goal_stop_s'], m['near_goal_stop_max_s'] = tot, lng
    # --- 위치추정
    if est.shape[0] and gt.shape[0]:
        ex, ey, _, ok = _interp_pose(est[:, 0], est[:, 1:4], tg)
        err = np.hypot(ex - gt[:, 1], ey - gt[:, 2])[ok]    # 표본 구간 밖·긴 틈은 빼고
        if err.size:
            m['loc_err_max_m'] = float(err.max())
            m['loc_err_p95_m'] = quantiles(err, (95,))[0]
            m['loc_collapse'] = bool(err.max() > LOC_COLLAPSE)
    lost = ep.get('lost') or []
    m['lost_events'] = sum(1 for (ti, b), (_, b0) in zip(lost[1:], lost[:-1])
                           if b and not b0 and t0 <= ti <= t_end) + \
        sum(1 for ti, b in lost[:1] if b and t0 <= ti <= t_end)
    m['lost_at_start'] = bool([b for ti, b in lost if ti <= t0][-1:] == [True])
    # --- 제어 주기·지연 (sim)
    cn = np.asarray(ep.get('cmd_nav') or np.zeros((0, 4)), dtype=float).reshape(-1, 4)
    cn = cn[(cn[:, 0] >= t0) & (cn[:, 0] <= t_end)] if cn.size else cn
    if cn.shape[0] > 2:
        dts = np.diff(cn[:, 0])
        dtw = np.diff(cn[:, 1])
        m['ctrl_dt_sim_mean'] = float(dts.mean())
        m['ctrl_dt_sim_p05'], m['ctrl_dt_sim_p95'] = quantiles(dts, (5, 95))
        m['ctrl_dt_wall_mean'] = float(dtw.mean())
    ss = np.asarray(ep.get('scan_stamps') or np.zeros((0, 2)), dtype=float).reshape(-1, 2)
    if cn.shape[0] and ss.shape[0]:
        k = np.searchsorted(ss[:, 0], cn[:, 0], side='right') - 1
        ok = k >= 0
        age = cn[ok, 0] - ss[k[ok], 1]
        m['scan_to_cmd_p50_s'], m['scan_to_cmd_p95_s'] = quantiles(age, (50, 95))
    if cn.shape[0] and co.shape[0]:
        # 사슬 지연: cmd_vel_nav 값이 바뀐 시각 → cmd_vel 이 처음 그 변화 방향으로 움직인 시각 (|Δ| > 0.005)
        lags = []
        for i in range(1, cn.shape[0]):
            dv = cn[i, 2] - cn[i - 1, 2]
            if abs(dv) < 0.005:
                continue
            j0 = int(np.searchsorted(co[:, 0], cn[i, 0], side='left'))
            base = co[j0 - 1, 1] if j0 > 0 else co[0, 1]
            for j in range(j0, min(co.shape[0], j0 + 50)):
                if (co[j, 1] - base) * dv > 0 and abs(co[j, 1] - base) > 0.002:
                    lags.append(co[j, 0] - cn[i, 0])
                    break
        m['chain_lag_p50_s'], m['chain_lag_p95_s'] = quantiles(lags, (50, 95))
    # --- RL 플러그인 stats
    stats = ep.get('stats') or []
    if stats:
        rows = [r for _, _, r in stats if r and len(r) >= 26]
        dec = [r for r in rows if r[12] > 0.5]
        src = [int(r[4]) for r in dec]
        rsn = [int(r[5]) for r in dec if int(r[4]) == 2]
        m['decisions'] = len(dec)
        for i, name in enumerate(('policy', 'fallback_cell', 'dwa', 'align')):
            m[f'src_{name}'] = src.count(i)
        for i, name in enumerate(('none', 'state', 'policy_error', 'timeout', 'nonfinite',
                                  'handover')):
            if i:
                m[f'dwa_{name}'] = rsn.count(i)
        inf = [r[3] for r in dec if int(r[4]) in (0, 1)]
        m['infer_ms_p50'], m['infer_ms_p95'] = quantiles(inf, (50, 95))
        m['infer_ms_max'] = float(max(inf)) if inf else None
        m['cycle_ms_p99'] = quantiles([r[0] for r in rows], (99,))[0]
        m['scan_age_p50_s'], m['scan_age_p95_s'] = quantiles([r[18] for r in dec], (50, 95))
        if rows and len(rows[0]) >= 31:
            tnow = np.array([r[27] for r in dec])
            per = np.diff(tnow)
            per = per[(per > 0) & (per < 0.5)]
            m['decision_period_mean_s'] = float(per.mean()) if per.size else None
            m['decision_period_p05_s'], m['decision_period_p95_s'] = quantiles(per, (5, 95))
            tdt = np.array([r[26] for r in rows])
            tdt = tdt[(tdt > 0) & (tdt < 0.5)]
            m['tick_dt_sim_mean'] = float(tdt.mean()) if tdt.size else None
    # --- RTF
    w = ep.get('wall') or {}
    if w.get('t_end') is not None and w.get('t0') is not None and w['t_end'] > w['t0']:
        m['rtf'] = float((t_end - t0) / (w['t_end'] - w['t0']))
    return m
