"""
동적 장애물 커리큘럼 (2차 물결 2b, r6-dyn) — 장애물 수 단계와 learnability p(1−p) 표본.

근거: 계약 §10, 「실무」 99행, 「이론」 295행.

수준(level) = env.ep_level: 0 … obstacle_slots 는 무작위 배치 k 개, 마지막(obstacle_slots + 1)은 Gazebo 7 개 배치.
학습 롤아웃에서 끝난 에피소드의 수준별 성공률을 EMA 로 추정하고, 허용된 수준 가운데 p(1−p) + ε 에 비례해 뽑는다.
일정 비율 u 는 허용 수준 위에서 균일하게 뽑는다 (계약 §10 "일정 비율 균일 DR"). 허용 수준은 수동 단계(장애물 상한
0 → 목표)가 정한다 — Gazebo 수준은 단계가 끝까지 올라간 뒤에만 허용한다.
"""

import torch


class LevelSampler:
    """수준별 성공 EMA → 표본 확률."""

    def __init__(self, n_levels, uniform=0.2, ema=0.2, eps=0.02, min_eps=8):
        """n_levels = obstacle_slots + 2."""
        self.n = n_levels
        self.gz = n_levels - 1
        self.uniform, self.ema, self.eps, self.min_eps = uniform, ema, eps, min_eps
        self.p = torch.zeros(n_levels, dtype=torch.float64)
        self.count = torch.zeros(n_levels, dtype=torch.long)

    def update(self, episodes):
        """끝난 에피소드 dict 목록 (env info['episode']) 으로 수준별 성공 EMA 를 갱신한다."""
        if not episodes:
            return
        lvl = torch.cat([e['level'] for e in episodes]).cpu()
        suc = torch.cat([e['success'] for e in episodes]).cpu().double()
        for k in range(self.n):
            m = lvl == k
            c = int(m.sum())
            if c < self.min_eps:
                continue
            r = float(suc[m].mean())
            self.p[k] = r if self.count[k] == 0 else (1.0 - self.ema) * self.p[k] + self.ema * r
            self.count[k] += c

    def allowed(self, max_count, full):
        """허용 수준 bool [n]: 무작위 0 … max_count, Gazebo 는 full 일 때만."""
        a = torch.zeros(self.n, dtype=torch.bool)
        a[:max_count + 1] = True
        a[self.gz] = bool(full)
        return a

    def probs(self, allowed):
        """허용 수준 위의 표본 확률: (1 − u)·(p(1−p) + ε)/Σ + u/|허용|."""
        w = (self.p * (1.0 - self.p) + self.eps) * allowed.double()
        w = w / w.sum()
        uni = allowed.double() / allowed.double().sum()
        return (1.0 - self.uniform) * w + self.uniform * uni


def mixed_probs(n_levels, n_obstacles, p_gazebo):
    """
    평가 분포(mixed)와 같은 수준 확률 — 무작위 n_obstacles[0] … [1] 균일 (1 − p_gazebo), Gazebo p_gazebo.

    env 의 mixed 배치를 수준 확률로 쓴 것과 분포가 같다 (난수열은 다르다).
    """
    p = torch.zeros(n_levels, dtype=torch.float64)
    lo, hi = n_obstacles
    p[lo:hi + 1] = (1.0 - p_gazebo) / (hi - lo + 1)
    p[n_levels - 1] = p_gazebo
    return p
