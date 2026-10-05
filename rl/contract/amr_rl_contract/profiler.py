"""
하류 명령 사슬의 기준 모델 — velocity_profiler 의 기준(reference) 부분을 그대로 옮긴다.

학습 환경(2D·Isaac)은 정책 명령 (v_cmd, w_cmd) 를 이 필터에 50 Hz 로 넣어 v_ref, w_ref 를 만들고,
그 뒤에 플랜트 지연(1차 지연 T_m + 순수 지연 T_d, V:16-17, DR 축)을 둔다. PI 보정·재동기화는
옮기지 않는다 (플랜트가 기준 모델과 같으면 PI 가 개입하지 않는다 — velocity_profiler.hpp 머리말).

원본: src/amr_navigation/src/core/jerk_limiter.cpp:16-60 (JerkLimitedFilter),
      src/amr_navigation/src/core/velocity_profiler.cpp:150-178 (곡률 보존).
"""

import math

from . import constants as C


class JerkLimitedFilter:
    """jerk_limiter.cpp:23-60 JerkLimitedFilter::step 의 정확한 이식."""

    def __init__(self, v_min, v_max, a_max, j_max):
        """한계 (v_min, v_max, a_max, j_max)."""
        self.v_min, self.v_max, self.a_max, self.j_max = v_min, v_max, a_max, j_max
        self.v = 0.0
        self.a = 0.0

    def reset(self, v, a):
        """상태 재설정 (jerk_limiter.cpp:16-21)."""
        self.v, self.a = float(v), float(a)

    def step(self, v_ref, dt):
        """한 스텝 — 반환: 새 속도."""
        if dt <= 0.0:
            return self.v
        v_ref = min(max(v_ref, self.v_min), self.v_max)
        a_max = max(1e-9, self.a_max)
        j = self.j_max
        if not (j > 0.0) or not math.isfinite(j):
            dv = min(max(v_ref - self.v, -a_max * dt), a_max * dt)
            self.v += dv
            self.a = dv / dt
            return self.v
        if abs(v_ref - self.v) <= j * dt * dt and abs(self.a) <= j * dt:
            self.v = v_ref
            self.a = 0.0
            return self.v
        c = v_ref - self.v - 0.5 * self.a * dt
        a_star = (1.0 if c >= 0.0 else -1.0) * j * (
            -0.5 * dt + math.sqrt(0.25 * dt * dt + 2.0 * abs(c) / j))
        jerk = min(max((a_star - self.a) / dt, -j), j)
        a1 = min(max(self.a + jerk * dt, -a_max), a_max)
        a1 = min(max(a1, self.a - j * dt), self.a + j * dt)
        self.v += 0.5 * (self.a + a1) * dt
        self.a = a1
        return self.v


class ProfilerReference:
    """
    velocity_profiler.cpp:150-178 의 기준 경로 (선속도 저크 필터 + 곡률 보존 각속도).

    한계는 VelocityProfilerConfig 기본값 = robot_params limits (R:57-62, V:11).
    """

    def __init__(self):
        """필터 두 개와 곡률 모드 상태."""
        self.lin = JerkLimitedFilter(-0.5, C.V_MAX_HW, C.A_MAX, C.JERK_MAX)
        self.ang = JerkLimitedFilter(-C.W_MAX, C.W_MAX, C.ALPHA_MAX, C.ANG_JERK_MAX)
        self.w_ref = 0.0
        self.curvature_mode = False

    def reset(self, v=0.0, a=0.0, w=0.0):
        """상태 재설정."""
        self.lin.reset(v, a)
        self.ang.reset(w, 0.0)
        self.w_ref = float(w)
        self.curvature_mode = False

    @property
    def a_ref(self):
        """선가속도 기준 (관측의 a_meas 기준값)."""
        return self.lin.a

    def step(self, v_target, w_target, dt=C.PROFILER_DT):
        """한 스텝 — 반환: (v_ref, w_ref)."""
        v_target = min(max(v_target, self.lin.v_min), self.lin.v_max)
        w_target = min(max(w_target, self.ang.v_min), self.ang.v_max)
        v_ref = self.lin.step(v_target, dt)
        curvature = abs(v_target) > 0.05     # V:13 curvature_min_speed
        if curvature:
            kappa = w_target / v_target
            w_des = min(max(kappa * v_ref, self.ang.v_min), self.ang.v_max)
            dw = self.ang.a_max * dt
            self.w_ref = min(max(w_des, self.w_ref - dw), self.w_ref + dw)
            self.curvature_mode = True
        else:
            if self.curvature_mode:
                self.ang.reset(self.w_ref, 0.0)
                self.curvature_mode = False
            self.w_ref = self.ang.step(w_target, dt)
        if curvature:
            self.ang.reset(self.w_ref, 0.0)
        return v_ref, self.w_ref


def run_decision_period(prof, v0_cmd, w0_cmd, v1_cmd, w1_cmd, period=C.DT, dt=C.PROFILER_DT):
    """
    결정 주기 하나 동안 선형 램프 명령을 프로파일러에 50 Hz 로 넣는다.

    명령은 v0 → v1 로 주기 안에서 선형 (플러그인 20 Hz 발행의 연속 극한).
    반환: 서브스텝별 (v_ref, w_ref) 목록 (길이 = period/dt = 5).
    """
    n = int(round(period / dt))
    out = []
    for k in range(1, n + 1):
        f = k / n
        out.append(prof.step(v0_cmd + f * (v1_cmd - v0_cmd), w0_cmd + f * (w1_cmd - w0_cmd), dt))
    return out
