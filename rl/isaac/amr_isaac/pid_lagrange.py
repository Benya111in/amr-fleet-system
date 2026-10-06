"""
제약 PPO (PID Lagrangian) 의 순수 부분 — 승수 제어기, 비용 GAE, 이익 결합 (torch, rsl_rl·Isaac 비의존).

근거: Stooke, Achiam, Abbeel, "Responsive Safety in RL by PID Lagrangian Methods", ICML 2020
(알고리즘 2);
OmniSafe 의 PPOLag·CPPOPID 결합식 (A_r − λ·A_c)/(1 + λ) [문서주장 — 코드 원문은 이 세션에서 다시 확인하지 않았다];
후속 조사 보고서 「제약 RL」 절 (예산 {0.2, 0.5, 1.0}, (J − d)/d 정규화 + λ 상한, 10 시드·실패 시드 비율).
제약 = 계약 §7.1 개입 비용(보정 크기)의 **비할인 에피소드 합** J 의 기댓값 ≤ d (계약 §7 "제약에는 비할인 에피소드 합").

PID 갱신 (반복마다, 그 반복 롤아웃에서 끝난 에피소드의 J 평균으로):
  e   = (J − d)/d                                   정규화 오차 (예산이 달라도 같은 이득을 쓰게)
  I   ← clip(I + K_i·e, 0, λ_max)                   적분 (반-와인드업: λ 상한에서 멈춘다)
  P   = EMA_p(e)                                    비례 (α_p = 0 이면 그대로)
  D   = max(0, J̄_t − J̄_{t−k})/d                    미분 (비용이 늘 때만, J̄ = EMA_d(J), k = d_delay)
  λ   = clip(K_p·P + I + K_d·D, 0, λ_max)
λ_init 은 I 의 초기값이다 (e = 0 이면 λ = λ_init).
"""

import collections

import torch


class PIDLagrangian:
    """승수 하나의 PID 제어기 (상태: I, EMA, 지연 창)."""

    def __init__(self, kp, ki, kd, budget, lambda_max=10.0, lambda_init=0.0, d_delay=1,
                 ema_p=0.0, ema_d=0.0):
        """이득·예산·상한."""
        if not budget > 0:
            raise ValueError(f'예산 d 는 양수여야 한다 (Slater, 계약 §7.2): {budget}')
        if not lambda_max > 0:
            raise ValueError(f'λ 상한은 양수여야 한다: {lambda_max}')
        if d_delay < 1:
            raise ValueError('d_delay ≥ 1')
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.d = float(budget)
        self.lmax = float(lambda_max)
        self.ema_p, self.ema_d = float(ema_p), float(ema_d)
        self.integral = min(max(float(lambda_init), 0.0), self.lmax)
        self.p_ema = None
        self.j_ema = None
        self.hist = collections.deque(maxlen=int(d_delay) + 1)
        self.lam = self.integral
        self.last = {}

    def update(self, J):
        """끝난 에피소드 비용 평균 J 로 λ 갱신 → λ."""
        J = float(J)
        e = (J - self.d) / self.d
        self.integral = min(max(self.integral + self.ki * e, 0.0), self.lmax)
        self.p_ema = e if self.p_ema is None else self.ema_p * self.p_ema + (1 - self.ema_p) * e
        self.j_ema = J if self.j_ema is None else self.ema_d * self.j_ema + (1 - self.ema_d) * J
        self.hist.append(self.j_ema)
        D = max(0.0, self.hist[-1] - self.hist[0]) / self.d
        raw = self.kp * self.p_ema + self.integral + self.kd * D
        self.lam = min(max(raw, 0.0), self.lmax)
        self.last = {'J': J, 'error': e, 'P': self.p_ema, 'I': self.integral, 'D': D, 'raw': raw,
                     'lambda': self.lam}
        return self.lam

    def state_dict(self):
        """저장용."""
        return {'kp': self.kp, 'ki': self.ki, 'kd': self.kd, 'd': self.d, 'lmax': self.lmax,
                'ema_p': self.ema_p, 'ema_d': self.ema_d, 'I': self.integral, 'p_ema': self.p_ema,
                'j_ema': self.j_ema, 'hist': list(self.hist), 'lam': self.lam}


@torch.no_grad()
def gae(rewards, values, dones, last_values, gamma, lam):
    """
    GAE — rewards·values·dones [T, N, 1], last_values [N, 1] → (returns, advantages) [T, N, 1].

    rsl_rl RolloutStorage.compute_returns 와 같은 점화식 (dones = 종단 또는 잘림; 잘림 bootstrap 은 보상 쪽에서 더한다).
    """
    T = rewards.shape[0]
    adv = torch.zeros_like(last_values)
    returns = torch.zeros_like(rewards)
    for t in reversed(range(T)):
        nv = last_values if t == T - 1 else values[t + 1]
        nt = 1.0 - dones[t].float()
        delta = rewards[t] + nt * gamma * nv - values[t]
        adv = delta + nt * gamma * lam * adv
        returns[t] = adv + values[t]
    return returns, returns - values


@torch.no_grad()
def combine_advantages(adv_r, adv_c, lam):
    """
    (A_r 표준화 − λ·A_c 중심화)/(1 + λ) — λ 가 커지면 비용 이익이 지배하되 크기가 폭주하지 않는다.

    A_r 은 평균·표준편차로, A_c 는 평균만 뺀다 (비용 크기를 남겨 λ 가 "비용 단위당 값" 이 되게).
    """
    a_r = (adv_r - adv_r.mean()) / (adv_r.std() + 1e-8)
    a_c = adv_c - adv_c.mean()
    return (a_r - lam * a_c) / (1.0 + lam)


class EpisodeCostTracker:
    """env 별 비할인 에피소드 비용 합 — 끝난 에피소드의 합을 모아 반복마다 평균을 낸다."""

    def __init__(self, n, device):
        """버퍼 [n]."""
        self.acc = torch.zeros(n, dtype=torch.float64, device=device)
        self.sum = 0.0
        self.count = 0

    @torch.no_grad()
    def step(self, cost, dones):
        """한 스텝: cost [n] (이 전이의 비용), dones [n] (이 전이로 에피소드가 끝남)."""
        self.acc += cost.reshape(-1).to(torch.float64)
        d = dones.reshape(-1).bool()
        if bool(d.any()):
            self.sum += float(self.acc[d].sum())
            self.count += int(d.sum())
            self.acc[d] = 0.0

    def pop(self):
        """(끝난 에피소드 J 평균 또는 None, 에피소드 수) 를 돌려주고 비운다."""
        out = (self.sum / self.count if self.count else None, self.count)
        self.sum, self.count = 0.0, 0
        return out
