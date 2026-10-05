"""
행동 → 속도 명령 적분 (contract.md §5).

정책은 가속도를 낸다. 결정 주기 DT = 0.1 s 동안 가속도를 붙잡고(ZOH) 명령 속도를 선형으로 올린다.
플러그인(20 Hz)은 주기 중간에 v_cmd + a·0.05 를, 끝에 v_cmd + a·0.1 을 발행한다.
비교군 B: 연속 u ∈ [−1, 1]² → (a_v, a_w) = (u0·A_MAX, u1·ALPHA_MAX), 속도 한계는 ray 클램프.
비교군 C: 격자 셀 인덱스(0..98) 또는 폴백(99). 셀은 마스크가 유효하다고 한 것만 고른다.
"""

import math

from . import constants as C


def clamp(x, lo, hi):
    """값을 [lo, hi] 로 자른다."""
    return lo if x < lo else hi if x > hi else x


def integrate_b(u, v_cmd, w_cmd, v_cap):
    """
    비교군 B 의 행동 적분.

    u: 정책 출력 (tanh 뒤, [−1, 1]²). 범위 밖이면 먼저 자른다.
    반환: (v_new, w_new, a_v_exec, a_w_exec) — exec 는 클램프 뒤 실제 명령 가속도.
    """
    u0 = clamp(float(u[0]), -1.0, 1.0)
    u1 = clamp(float(u[1]), -1.0, 1.0)
    a_v = u0 * C.A_MAX
    a_w = u1 * C.ALPHA_MAX
    v_hi = min(v_cap, C.V_MAX_HW)
    v_new = clamp(v_cmd + a_v * C.DT, C.V_MIN, max(C.V_MIN, v_hi))
    w_new = clamp(w_cmd + a_w * C.DT, -C.W_MAX, C.W_MAX)
    return v_new, w_new, (v_new - v_cmd) / C.DT, (w_new - w_cmd) / C.DT


def cell_accel(index):
    """격자 셀 인덱스 → (a_v, a_w). index = iv·9 + iw."""
    if not 0 <= index < C.N_CELLS:
        raise ValueError(f'셀 인덱스 범위 밖: {index}')
    iv, iw = divmod(index, len(C.LATTICE_AW))
    return C.LATTICE_AV[iv], C.LATTICE_AW[iw]


def fallback_command(v_cmd, w_cmd):
    """
    폴백 = DWA 제동 후보와 같은 규칙 (dwa.cpp:163-170), 결정 주기 0.1 s 로.

    v: 최대 감속(A_MAX)으로 0 쪽, ω: ALPHA_MAX 로 0 쪽. 항상 가용 (마스크 검사 면제).
    반환: (v_new, w_new, a_v, a_w).
    """
    dv = C.A_MAX * C.DT
    if v_cmd > 0.0:
        v_new = max(0.0, v_cmd - dv)
    else:
        v_new = min(0.0, v_cmd + dv)
    dw = min(abs(w_cmd), C.ALPHA_MAX * C.DT)
    w_new = w_cmd - math.copysign(dw, w_cmd) if w_cmd != 0.0 else 0.0
    return v_new, w_new, (v_new - v_cmd) / C.DT, (w_new - w_cmd) / C.DT


def integrate_c(index, v_cmd, w_cmd):
    """
    비교군 C 의 행동 적분 — 클램프하지 않는다 (마스크가 이미 한계를 보장한다).

    반환: (v_new, w_new, a_v, a_w).
    """
    if index == C.FALLBACK_INDEX:
        return fallback_command(v_cmd, w_cmd)
    a_v, a_w = cell_accel(index)
    return v_cmd + a_v * C.DT, w_cmd + a_w * C.DT, a_v, a_w


def accel_distance(a, b):
    """정규화 가속도 거리 (Δa_v/A_MAX)² + (Δa_w/ALPHA_MAX)²."""
    return ((a[0] - b[0]) / C.A_MAX) ** 2 + ((a[1] - b[1]) / C.ALPHA_MAX) ** 2


def nearest_cell(a_v, a_w, mask=None, fallback_accel=None):
    """
    임의 가속도에 가장 가까운 행동 (정규화 거리 accel_distance 최소, 동률이면 작은 인덱스).

    mask 가 None 이면 99 셀 전부가 후보이고 폴백은 후보가 아니다 (격자 투영, v1.0.0 과 같음).
    mask(길이 N_ACTIONS_C) 를 주면 mask 가 참인 셀, 그리고 mask[99] 가 참이면 가속도
    fallback_accel 인 폴백(99)이 후보다 — 결과는 항상 마스크 안이다 (BC 레이블, bc_label).
    """
    if mask is None:
        cands = [(i, cell_accel(i)) for i in range(C.N_CELLS)]
    else:
        if len(mask) != C.N_ACTIONS_C:
            raise ValueError(f'마스크 길이 {len(mask)} != {C.N_ACTIONS_C}')
        cands = [(i, cell_accel(i)) for i in range(C.N_CELLS) if mask[i]]
        if mask[C.FALLBACK_INDEX]:
            if fallback_accel is None:
                raise ValueError('mask[99] 가 참이면 fallback_accel 이 필요하다')
            cands.append((C.FALLBACK_INDEX, tuple(fallback_accel)))
    best, best_d = None, math.inf
    for i, acc in cands:
        d = accel_distance(acc, (a_v, a_w))
        if d < best_d - 1e-12:
            best, best_d = i, d
    if best is None:
        raise ValueError('후보가 없다 (마스크가 전부 거짓)')
    return best


def bc_label(v_dwa, w_dwa, v_cmd, w_cmd, mask):
    """
    비교군 C 의 BC 레이블 (contract.md §5.2) — DWA 명령을 **마스크 안** 최근접 행동으로.

    a = ((v_dwa − v_cmd)/DT, (w_dwa − w_cmd)/DT). 후보 = 마스크가 참인 셀 + 폴백
    (가속도 = fallback_command(v_cmd, w_cmd) 의 실행 가속도). 동률이면 작은 인덱스 (폴백 99 가 마지막).
    DWA 는 계획기에서 저크를 걸지 않으므로(dwa.cpp:108-135, 저크는 하류 프로파일러) 가속 시작·선회 시작의
    DWA 가속도는 저크 마스크 밖에 있기 쉽다 — 그때 레이블은 저크가 허락하는 만큼 같은 방향으로 간 셀이다.
    """
    a_v = (v_dwa - v_cmd) / C.DT
    a_w = (w_dwa - w_cmd) / C.DT
    fb = fallback_command(v_cmd, w_cmd)
    return nearest_cell(a_v, a_w, mask, (fb[2], fb[3]))


def stop_time_and_distance(v, a=C.A_MAX, j=C.JERK_MAX, t_c=0.0):
    """
    저크 제한 정지 시간·거리 (참고용: γ·λ 유도에 쓴 2 s 정지 시간).

    가속도 0 에서 시작해 저크 j 로 −a 까지, 등감속, 다시 저크 j 로 0 (대칭 S 커브 가정).
    """
    if v <= 0.0:
        return 0.0, 0.0
    t_j = a / j
    dv_j = 0.5 * a * t_j
    if v <= 2.0 * dv_j:
        t1 = math.sqrt(v / j)
        return t_c + 2.0 * t1, v * t_c + v * t1   # 등감속 구간 없는 대칭 S 커브 (정확)
    t_flat = (v - 2.0 * dv_j) / a
    t = 2.0 * t_j + t_flat
    return t_c + t, v * t_c + 0.5 * v * t
