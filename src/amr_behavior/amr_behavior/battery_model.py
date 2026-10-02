"""
배터리 잔량 모델 (ROS 비의존) — 시뮬레이션의 BMS 대역.

Gazebo 로봇에는 배터리가 없어 battery_state 를 내는 노드가 없으면 실행기는 잔량을 "미측정 = 정상" 으로
보고 Charge 서브트리가 한 번도 돌지 않는다. 이 모델은 소비 전력을 적분해 잔량을 만들고, 실행기가
charging/enable 을 켜고 정지해 있을 때만 충전한다.

    P = P_idle + (k_drive + k_payload·m_payload)·|v| + k_turn·|ω|        [W]
    SoC ← SoC − P·dt / E_cap (방전),   SoC ← SoC + P_charge·dt / E_cap (충전, 정지 중)

기본값 (24 V 20 Ah = 480 Wh 팩, 공차 47.6 kg AMR): 대기 30 W(컴퓨터·센서), 1 m/s 주행 60 W, 적재 kg 당
1 m/s 에서 +1 W, 회전 1 rad/s 에서 10 W, 충전 480 W (1 C) → 100 % 에서 대기만으로 16 h, 1 m/s 연속 주행
5.3 h. drain_scale 로 방전만 빠르게 해 충전 경로를 짧은 시험에서 돌릴 수 있다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class BatteryParams:
    """배터리 모델 파라미터 (battery_model_node 파라미터와 1:1)."""

    capacity_wh: float = 480.0          # [Wh] 팩 용량
    nominal_voltage: float = 24.0       # [V]
    idle_power_w: float = 30.0          # [W] 정지 중 소비 (컴퓨터·센서)
    drive_power_w_per_mps: float = 60.0  # [W/(m/s)] 공차 주행
    payload_power_w_per_kg_mps: float = 1.0  # [W/(kg·m/s)] 적재 질량에 비례하는 추가 주행 전력
    turn_power_w_per_radps: float = 10.0     # [W/(rad/s)] 제자리 회전
    charge_power_w: float = 480.0       # [W] 충전 (1 C)
    drain_scale: float = 1.0            # 방전 배율 (시험용 가속, 충전에는 적용하지 않는다)
    stationary_speed: float = 0.02      # [m/s] 이 이하면 정지로 보고 충전을 허용
    initial_percent: float = 100.0      # [%]


class BatteryModel:
    """상태: 잔량 [%]. step() 으로 적분한다."""

    def __init__(self, params: BatteryParams):
        self.params = params
        self.percent = min(100.0, max(0.0, float(params.initial_percent)))
        self.charging_enabled = False
        self.payload_mass = 0.0
        self.linear = 0.0
        self.angular = 0.0

    def power_draw(self) -> float:
        """현재 소비 전력 [W] (충전 전력 제외)."""
        p = self.params
        mass = max(0.0, self.payload_mass)
        return (p.idle_power_w
                + (p.drive_power_w_per_mps + p.payload_power_w_per_kg_mps * mass)
                * abs(self.linear)
                + p.turn_power_w_per_radps * abs(self.angular))

    def is_charging(self) -> bool:
        """충전 신호가 켜져 있고 정지해 있으면 충전 중."""
        return (self.charging_enabled and abs(self.linear) <= self.params.stationary_speed
                and self.percent < 100.0)

    def step(self, dt: float) -> float:
        """시간 dt [s] 만큼 적분하고 잔량 [%] 을 돌려준다."""
        if not (dt > 0.0) or not math.isfinite(dt):
            return self.percent
        p = self.params
        cap_j = max(p.capacity_wh, 1e-6) * 3600.0
        drain = self.power_draw() * max(p.drain_scale, 0.0)
        if self.is_charging():
            delta = (p.charge_power_w - self.power_draw()) * dt / cap_j * 100.0
        else:
            delta = -drain * dt / cap_j * 100.0
        self.percent = min(100.0, max(0.0, self.percent + delta))
        return self.percent

    def voltage(self) -> float:
        """단순 선형 OCV: 0 % → 0.85·V_nom, 100 % → 1.05·V_nom."""
        v = self.params.nominal_voltage
        return v * (0.85 + 0.20 * self.percent / 100.0)
