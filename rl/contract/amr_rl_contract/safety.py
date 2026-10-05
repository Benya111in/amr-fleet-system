"""
safety_lite — safety_gate.cpp 의 속도 상한·명령 스케일 부분 (φ) 기준 구현.

정본 φ 는 C++ SafetyGate 다 (src/amr_perception/src/safety_gate.cpp). 2D 시뮬레이터는 그것을 바인딩해
쓰는 것을 권장하고 (이론 보고서 「필터를 학습 루프에 넣을 때」), 바인딩이 없을 때 이 모듈이 최소 규칙이다.
여기 없는 것: 스윕 풋프린트 접근 거리 D 의 계산(입력으로 받는다), STOP 확정 프레임·래치·히스테리시스,
접촉 가드·탈출·후퇴·양보 탈출 특례, 센서 고장, 도킹 예외. → 그 차이는 contract.md §6.4 에 적었다.
"""

import math

from . import constants as C


def clearance_speed_limit(distance, stop_distance=C.E_STOP_DIST):
    """safety_gate.cpp:308-317 — d(v) = v·t + v²/(2a) ≤ D − d_stop 의 양의 근."""
    if distance is None or not math.isfinite(distance):
        return math.inf
    margin = max(distance - stop_distance, 0.0)
    at = C.SAFETY_DECEL * C.REACTION_LATENCY
    return -at + math.sqrt(at * at + 2.0 * C.SAFETY_DECEL * margin)


def max_point_speed(v, w, half_l=C.HALF_L, half_w=C.HALF_W):
    """safety_gate.cpp:98-108 — 풋프린트 꼭짓점 최고 속력."""
    best = 0.0
    for x in (-half_l, half_l):
        for y in (-half_w, half_w):
            best = max(best, math.hypot(v - w * y, w * x))
    return best


def classify_zone(distance):
    """safety_gate.cpp classifyZone (히스테리시스 없음)."""
    if distance is None or not math.isfinite(distance):
        return C.ZONE_CLEAR
    if distance <= C.E_STOP_DIST:
        return C.ZONE_STOP
    if distance <= C.CRITICAL_DIST:
        return C.ZONE_CRITICAL
    if distance <= C.WARNING_DIST:
        return C.ZONE_WARNING
    return C.ZONE_CLEAR


def safety_filter(v, w, d_main, min_ttc, proximity_stop):
    """
    φ(v, w) — 반환 dict: v, w, zone, speed_limit, u_cap, v_ttc.

    d_main: 접근 거리 D [m] (None = 없음), min_ttc: 동적 트랙 최소 TTC [s] (None = 없음),
    proximity_stop: 환경이 확정한 STOP 래치 (D ≤ 0.30 확정). 규칙 = safety_gate.cpp:824-874, 974.
    """
    zone = C.ZONE_STOP if proximity_stop else classify_zone(d_main)
    if zone == C.ZONE_STOP and not proximity_stop:
        zone = C.ZONE_CRITICAL      # 확정 전 STOP 은 CRITICAL 로 (safety_gate.cpp:808-812)
    u_cap = math.inf
    if zone == C.ZONE_WARNING:
        u_cap = C.WARNING_MAX_SPEED
    elif zone == C.ZONE_CRITICAL:
        u_cap = C.CRITICAL_MAX_SPEED
    u_cap = min(u_cap, clearance_speed_limit(d_main))
    v_ttc = math.inf
    if min_ttc is not None and math.isfinite(min_ttc) and min_ttc <= C.TTC_CRITICAL:
        v_ttc = max(0.0, C.SAFETY_DECEL * (min_ttc - C.REACTION_LATENCY))
    speed_limit = min(u_cap, v_ttc, C.V_MAX_HW)
    if proximity_stop:
        return {'v': 0.0, 'w': 0.0, 'zone': zone, 'speed_limit': speed_limit,
                'u_cap': u_cap, 'v_ttc': v_ttc}
    s = 1.0
    u = max_point_speed(v, w)
    if u > u_cap:
        s = min(s, u_cap / u)
    if abs(v) > v_ttc:
        s = min(s, v_ttc / abs(v))
    return {'v': v * s, 'w': w * s, 'zone': zone, 'speed_limit': speed_limit,
            'u_cap': u_cap, 'v_ttc': v_ttc}
