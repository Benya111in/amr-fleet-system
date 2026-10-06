"""
GPU 상주 PPO — 비교군 B(연속 가속도, tanh 가우시안)와 C(마스크 범주형). 계약 §5·§6·§9 와 onnx_io.md 를 따른다.

  - 관측 정규화 running mean/std 는 **모델 안**: x = clip((obs − μ)/√(max(σ², 0.01) + 1e-8), −10, 10)
    (onnx_io.md 식에 분산 하한을 더했다 — RunningNorm 독스트링)
  - B: z ~ N(μ, σ), u = tanh(z). 버퍼에는 원출력 z 와 log N(z) 를 둔다 — tanh 야코비안은 같은 z 에서
    비율 π_new/π_old 에서 지워진다. 결정론적 행동 = tanh(μ).
  - C: logits − 1e9·(1 − mask) (−inf 대신, onnx_io.md), 마스크를 버퍼에 저장해 갱신 때 같은 마스크로 log-prob·
    엔트로피를 다시 계산한다 (계약 §5.2 조건 2). 정책·critic 모두 마스크를 입력으로 받는다 (조건 4).
  - γ 0.995, GAE λ 0.97, 롤아웃 ≥ 64 스텝 (계약 §9). truncation(시간 상한·정체)은 V(final_obs) 로 부트스트랩,
    종단(성공·충돌)은 0 (계약 §6 "truncation 에서는 … bootstrap").
  - 보상 정규화는 할인 수익 표준편차로 **나누기만** 한다 (계약 §6).
  - clip + KL 조기 중단 (계약 §9 하이브리드 PPO).
"""

from dataclasses import asdict, dataclass
import math
import time

import torch
import torch.nn as nn

from ._contract import C

OBS_DIM = 291


class RunningNorm(nn.Module):
    """
    관측 running mean/var (Welford 병합). ONNX 로 같이 나간다.

    분산 하한 var_floor: 학습 초기에 늘 같은 값이던 특징(커리큘럼 앞 단계의 빈 트랙 칸, 이탈 블록 등)은 분산이
    1e-10 까지 내려가고, 나중에 그 특징이 0 → 1 로 바뀌면 정규화 값이 ±10 에 붙어 정책이 무너진다 (2026-10-06
    B_bc2 실측: 이탈 블록 224–226·트랙 칸 4–7 의 분산 3e-10). 계약 관측은 이미 O(1) 로 정규화돼 있으므로
    σ ≥ 0.1 (var ≥ 0.01) 하한을 둔다. 하한은 forward 그래프 안의 clamp 라 ONNX 에도 그대로 들어간다.
    """

    def __init__(self, dim, clip=10.0, eps=1e-8, var_floor=1e-2):
        """버퍼: mean, var, count."""
        super().__init__()
        self.register_buffer('mean', torch.zeros(dim))
        self.register_buffer('var', torch.ones(dim))
        self.register_buffer('count', torch.tensor(1e-4))
        self.clip, self.eps, self.var_floor = clip, eps, var_floor

    @torch.no_grad()
    def update(self, x):
        """배치 x [B, dim] 로 갱신."""
        x = x.reshape(-1, x.shape[-1]).float()
        bm, bv, bc = x.mean(0), x.var(0, unbiased=False), x.shape[0]
        delta = bm - self.mean
        tot = self.count + bc
        self.mean += delta * bc / tot
        m2 = self.var * self.count + bv * bc + delta ** 2 * self.count * bc / tot
        self.var.copy_(m2 / tot)
        self.count.copy_(tot)

    def forward(self, x):
        """정규화·자르기."""
        z = (x - self.mean) / torch.sqrt(torch.clamp(self.var, min=self.var_floor) + self.eps)
        return torch.clamp(z, -self.clip, self.clip)


def mlp(i, hidden, o, act=nn.Tanh):
    """은닉층 hidden 의 MLP."""
    layers, d = [], i
    for h in hidden:
        layers += [nn.Linear(d, h), act()]
        d = h
    layers.append(nn.Linear(d, o))
    return nn.Sequential(*layers)


class ActorCriticB(nn.Module):
    """비교군 B: tanh 가우시안, 상태 무관 log σ."""

    arm = 'B'

    def __init__(self, hidden=(256, 256), log_std0=-0.5):
        """정책·가치 MLP 와 정규화기."""
        super().__init__()
        self.norm = RunningNorm(OBS_DIM)
        self.pi = mlp(OBS_DIM, hidden, 2)
        self.v = mlp(OBS_DIM, hidden, 1)
        self.log_std = nn.Parameter(torch.full((2,), float(log_std0)))
        with torch.no_grad():
            self.pi[-1].weight.mul_(0.01)
            self.pi[-1].bias.zero_()

    def dist(self, obs, mask=None):
        """정규 분포 (z 공간)."""
        mu = self.pi(self.norm(obs))
        return torch.distributions.Normal(mu, self.log_std.exp().expand_as(mu))

    def value(self, obs, mask=None):
        """V(s)."""
        return self.v(self.norm(obs)).squeeze(-1)

    def act(self, obs, mask=None, deterministic=False):
        """→ (환경 행동 u, 버퍼 행동 z, log π(z))."""
        d = self.dist(obs)
        z = d.mean if deterministic else d.sample()
        return torch.tanh(z), z, d.log_prob(z).sum(-1)

    def evaluate(self, obs, z, mask=None):
        """→ (log π(z), 엔트로피, V)."""
        d = self.dist(obs)
        return d.log_prob(z).sum(-1), d.entropy().sum(-1), self.value(obs)

    def forward(self, obs):
        """ONNX 출력 (action, mu, log_std) — onnx_io.md 비교군 B."""
        mu = self.pi(self.norm(obs))
        return torch.tanh(mu), mu, self.log_std.expand_as(mu)


class ActorCriticC(nn.Module):
    """비교군 C: 마스크 범주형 (100 = 99 셀 + 폴백), 마스크를 입력으로."""

    arm = 'C'

    def __init__(self, hidden=(256, 256)):
        """정책·가치 MLP (입력 = 정규화 관측 291 + 마스크 100)."""
        super().__init__()
        self.norm = RunningNorm(OBS_DIM)
        self.pi = mlp(OBS_DIM + C.N_ACTIONS_C, hidden, C.N_ACTIONS_C)
        self.v = mlp(OBS_DIM + C.N_ACTIONS_C, hidden, 1)
        with torch.no_grad():
            self.pi[-1].weight.mul_(0.01)
            self.pi[-1].bias.zero_()

    def _x(self, obs, mask):
        return torch.cat([self.norm(obs), mask.float()], -1)

    def masked_logits(self, obs, mask):
        """무효 셀 = 원 logit − 1e9."""
        m = mask.float()
        return self.pi(self._x(obs, mask)) - 1e9 * (1.0 - m)

    def value(self, obs, mask):
        """V(s, mask)."""
        return self.v(self._x(obs, mask)).squeeze(-1)

    def act(self, obs, mask, deterministic=False):
        """→ (행동 인덱스, 같은 인덱스, log π)."""
        lg = self.masked_logits(obs, mask)
        d = torch.distributions.Categorical(logits=lg)
        a = lg.argmax(-1) if deterministic else d.sample()
        return a, a, d.log_prob(a)

    def evaluate(self, obs, a, mask):
        """→ (log π, 엔트로피(유효 셀만), V)."""
        lg = self.masked_logits(obs, mask)
        d = torch.distributions.Categorical(logits=lg)
        p = d.probs
        ent = -(p * torch.where(mask, d.logits, torch.zeros_like(lg))).sum(-1)
        return d.log_prob(a.long()), ent, self.value(obs, mask)

    def forward(self, obs, mask):
        """ONNX 출력 (logits, action) — onnx_io.md 비교군 C."""
        lg = self.masked_logits(obs, mask)
        return lg, torch.argmax(lg, dim=-1)


@dataclass
class PPOConfig:
    """PPO 하이퍼파라미터 (계약이 정한 것: γ, λ, 롤아웃 ≥ 64, clip + KL 조기 중단)."""

    gamma: float = C.GAMMA
    lam: float = C.GAE_LAMBDA
    rollout: int = 64
    epochs: int = 5
    minibatches: int = 8
    lr: float = 3e-4
    clip: float = 0.2
    vf_coef: float = 0.5
    ent_coef: float = 0.0
    max_grad_norm: float = 1.0
    target_kl: float = 0.03
    hidden: tuple = (256, 256)
    reward_norm: bool = True
    # 라그랑주 승수 (계약 §7): 비용 채널별 PI 갱신 λ ← max(0, λ_I + Kp·e), λ_I += Ki·e,
    # e = (끝난 에피소드의 평균 비할인 비용 합 − 예산). 예산 None 인 채널(개입, §7.2 미정)은 λ = 0.
    lagrangian: bool = False
    lag_kp: float = 0.0
    lag_ki: float = 2e-4
    lag_max: float = 2.0
    budgets: tuple = C.COST_BUDGETS


class Trainer:
    """롤아웃 수집 + PPO 갱신 (모두 GPU)."""

    def __init__(self, env, cfg: PPOConfig, seed=0):
        """모델·최적화기·버퍼."""
        self.env, self.cfg = env, cfg
        torch.manual_seed(seed)
        self.dev = env.dev
        self.arm = env.cfg.arm
        self.model = (ActorCriticB(cfg.hidden) if self.arm == 'B'
                      else ActorCriticC(cfg.hidden)).to(self.dev)
        self.opt = torch.optim.Adam(self.model.parameters(), lr=cfg.lr, eps=1e-5)
        N, T = env.N, cfg.rollout
        self.N, self.T = N, T
        dev = self.dev
        self.b_obs = torch.zeros(T, N, OBS_DIM, device=dev)
        self.b_act = torch.zeros(T, N, 2, device=dev) if self.arm == 'B' else \
            torch.zeros(T, N, dtype=torch.long, device=dev)
        self.b_mask = torch.ones(T, N, C.N_ACTIONS_C, dtype=torch.bool, device=dev) \
            if self.arm == 'C' else None
        self.b_logp = torch.zeros(T, N, device=dev)
        self.b_rew = torch.zeros(T, N, device=dev)
        self.b_done = torch.zeros(T, N, device=dev)
        self.b_val = torch.zeros(T, N, device=dev)
        self.ret_rms_mean = torch.tensor(0.0, device=dev)
        self.ret_rms_var = torch.tensor(1.0, device=dev)
        self.ret_rms_count = torch.tensor(1e-4, device=dev)
        self.disc_ret = torch.zeros(N, device=dev)
        self.obs = env.reset()
        self.steps = 0
        self.ep_hist = []
        self.lam = torch.zeros(C.N_COSTS, device=dev)
        self.lam_i = torch.zeros(C.N_COSTS, device=dev)

    def _scale(self, r, done):
        """할인 수익 표준편차로 나누기만 (계약 §6). done 이면 누적 수익 초기화."""
        if not self.cfg.reward_norm:
            return r
        self.disc_ret = self.disc_ret * self.cfg.gamma + r
        x = self.disc_ret
        bm, bv, bc = x.mean(), x.var(unbiased=False), x.numel()
        # 분산 병합 (Welford/Chan). 평균은 분산 계산에만 쓰고 보상에서 빼지 않는다
        tot = self.ret_rms_count + bc
        delta = bm - self.ret_rms_mean
        self.ret_rms_var = (self.ret_rms_var * self.ret_rms_count + bv * bc +
                            delta ** 2 * self.ret_rms_count * bc / tot) / tot
        self.ret_rms_mean = self.ret_rms_mean + delta * bc / tot
        self.ret_rms_count = tot
        self.disc_ret = torch.where(done, torch.zeros_like(self.disc_ret), self.disc_ret)
        return r / torch.sqrt(self.ret_rms_var + 1e-8)

    @torch.no_grad()
    def collect(self):
        """롤아웃 T 스텝 → 버퍼, 끝난 에피소드 통계."""
        env, m = self.env, self.model
        eps = []
        for t in range(self.T):
            mask = env.mask if self.arm == 'C' else None
            m.norm.update(self.obs)
            u, a, logp = m.act(self.obs, mask)
            v = m.value(self.obs, mask)
            obs2, r, term, trunc, info = env.step(u)
            r = r.float()
            if self.cfg.lagrangian:
                r = r - (info['cost'] * self.lam[None, :]).sum(-1)
            if 'done_idx' in info:
                di = info['done_idx']
                tr = trunc[di]
                if bool(tr.any()):        # truncation: γ V(final_obs) 부트스트랩 (원 보상 단위로)
                    fo = info['final_obs'][tr]
                    fm = info['final_mask'][tr] if self.arm == 'C' else None
                    boot = self.cfg.gamma * m.value(fo, fm)
                    if self.cfg.reward_norm:
                        boot = boot * torch.sqrt(self.ret_rms_var + 1e-8)
                    r[di[tr]] += boot
                eps.append({k: v.detach() for k, v in info['episode'].items()})
            done = term | trunc
            self.b_obs[t] = self.obs
            self.b_act[t] = a
            if self.arm == 'C':
                self.b_mask[t] = mask
            self.b_logp[t] = logp
            self.b_val[t] = v
            self.b_rew[t] = self._scale(r, done)
            self.b_done[t] = done.float()
            self.obs = obs2
        self.steps += self.T * self.N
        with torch.no_grad():
            self.last_val = m.value(self.obs, env.mask if self.arm == 'C' else None)
        return eps

    def update_lagrange(self, eps):
        """끝난 에피소드의 평균 비용 합으로 λ 를 PI 갱신 → λ 목록."""
        if not (self.cfg.lagrangian and eps):
            return self.lam.tolist()
        cost = torch.cat([e['cost'] for e in eps]).float().mean(0)
        for i, d in enumerate(self.cfg.budgets):
            if d is None:
                continue
            e = float(cost[i]) - d
            self.lam_i[i] = max(0.0, float(self.lam_i[i]) + self.cfg.lag_ki * e)
            lam = max(0.0, float(self.lam_i[i]) + self.cfg.lag_kp * e)
            self.lam[i] = min(self.cfg.lag_max, lam)
        return self.lam.tolist()

    def behavior_clone(self, demo, steps, epochs=30, batch=8192, z_clip=0.97, log=print):
        """
        시연자 demo(obs → u) 로 B 정책 평균을 회귀 초기화한다 (BC, z = atanh(clip(u))).

        시연 수집은 시연자 행동으로 env 를 굴린다 (정규화 통계도 이 관측으로 갱신). 반환: 마지막 MSE.
        """
        assert self.arm == 'B'
        env, m = self.env, self.model
        O, Z = [], []
        obs = self.obs
        with torch.no_grad():
            for _ in range(steps):
                u = demo(obs)
                O.append(obs)
                Z.append(torch.atanh(u.clamp(-z_clip, z_clip)))
                m.norm.update(obs)
                obs, *_ = env.step(u)
        self.obs = obs
        O, Z = torch.cat(O), torch.cat(Z)
        opt = torch.optim.Adam(m.pi.parameters(), lr=1e-3)
        n = O.shape[0]
        for ep in range(epochs):
            perm = torch.randperm(n, device=self.dev)
            tot = 0.0
            for i in range(0, n, batch):
                idx = perm[i:i + batch]
                loss = ((m.pi(m.norm(O[idx])) - Z[idx]) ** 2).mean()
                opt.zero_grad(set_to_none=True)
                loss.backward()
                opt.step()
                tot += float(loss) * idx.numel()
            if ep % 10 == 0 or ep == epochs - 1:
                log(f'[bc] epoch {ep} mse {tot / n:.4f} (n={n})')
        return tot / n

    def warmup_critic(self, iters):
        """정책을 고정하고 가치망만 iters 롤아웃 학습 (BC 직후 부정확한 critic 이 정책을 흔들지 않게)."""
        saved = (self.cfg.epochs, self.opt)
        vopt = torch.optim.Adam(self.model.v.parameters(), lr=self.cfg.lr, eps=1e-5)
        for _ in range(iters):
            self.collect()
            T, N, cfg = self.T, self.N, self.cfg
            adv = torch.zeros(T, N, device=self.dev)
            last = torch.zeros(N, device=self.dev)
            nv = self.last_val
            for t in reversed(range(T)):
                nonterm = 1.0 - self.b_done[t]
                delta = self.b_rew[t] + cfg.gamma * nv * nonterm - self.b_val[t]
                last = delta + cfg.gamma * cfg.lam * nonterm * last
                adv[t] = last
                nv = self.b_val[t]
            ret = (adv + self.b_val).reshape(-1)
            obs = self.b_obs.reshape(T * N, -1)
            for _ in range(cfg.epochs):
                perm = torch.randperm(T * N, device=self.dev)
                mb = T * N // cfg.minibatches
                for i in range(cfg.minibatches):
                    idx = perm[i * mb:(i + 1) * mb]
                    vl = 0.5 * ((self.model.value(obs[idx]) - ret[idx]) ** 2).mean()
                    vopt.zero_grad(set_to_none=True)
                    vl.backward()
                    vopt.step()
        self.cfg.epochs, self.opt = saved

    def update(self):
        """GAE + PPO 갱신 → 손실·KL 통계."""
        cfg, T, N = self.cfg, self.T, self.N
        adv = torch.zeros(T, N, device=self.dev)
        last = torch.zeros(N, device=self.dev)
        nv = self.last_val
        for t in reversed(range(T)):
            nonterm = 1.0 - self.b_done[t]
            delta = self.b_rew[t] + cfg.gamma * nv * nonterm - self.b_val[t]
            last = delta + cfg.gamma * cfg.lam * nonterm * last
            adv[t] = last
            nv = self.b_val[t]
        ret = adv + self.b_val
        B = T * N
        obs = self.b_obs.reshape(B, -1)
        act = self.b_act.reshape(B, -1) if self.arm == 'B' else self.b_act.reshape(B)
        mask = self.b_mask.reshape(B, -1) if self.arm == 'C' else None
        logp0 = self.b_logp.reshape(B)
        adv = adv.reshape(B)
        ret = ret.reshape(B)
        mb = B // cfg.minibatches
        stats = {'pi_loss': 0.0, 'v_loss': 0.0, 'entropy': 0.0, 'kl': 0.0, 'clipfrac': 0.0,
                 'epochs_run': 0}
        nupd = 0
        for ep in range(cfg.epochs):
            perm = torch.randperm(B, device=self.dev)
            kls = []
            for i in range(cfg.minibatches):
                idx = perm[i * mb:(i + 1) * mb]
                a_mb = adv[idx]
                a_mb = (a_mb - a_mb.mean()) / (a_mb.std() + 1e-8)
                lp, ent, v = self.model.evaluate(obs[idx], act[idx],
                                                 mask[idx] if mask is not None else None)
                ratio = torch.exp(lp - logp0[idx])
                clipped = ratio.clamp(1 - cfg.clip, 1 + cfg.clip) * a_mb
                pg = -torch.min(ratio * a_mb, clipped).mean()
                vl = 0.5 * ((v - ret[idx]) ** 2).mean()
                loss = pg + cfg.vf_coef * vl - cfg.ent_coef * ent.mean()
                self.opt.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(), cfg.max_grad_norm)
                self.opt.step()
                with torch.no_grad():
                    kl = ((ratio - 1) - torch.log(ratio)).mean()
                    kls.append(kl)
                    stats['pi_loss'] += float(pg)
                    stats['v_loss'] += float(vl)
                    stats['entropy'] += float(ent.mean())
                    stats['clipfrac'] += float(((ratio - 1).abs() > cfg.clip).float().mean())
                nupd += 1
            stats['epochs_run'] = ep + 1
            stats['kl'] = float(torch.stack(kls).mean())
            if cfg.target_kl and stats['kl'] > cfg.target_kl:
                break
        for k in ('pi_loss', 'v_loss', 'entropy', 'clipfrac'):
            stats[k] /= max(1, nupd)
        if self.arm == 'B':
            stats['std'] = self.model.log_std.exp().tolist()
        return stats


def episode_summary(eps):
    """끝난 에피소드 dict 목록 → 평균 지표."""
    if not eps:
        return {'episodes': 0}
    cat = {k: torch.cat([e[k] for e in eps]) for k in eps[0]}
    n = cat['success'].numel()
    cost = cat['cost'].double()
    out = {'episodes': n,
           'success': float(cat['success'].float().mean()),
           'collision': float(cat['collision'].float().mean()),
           'collision_moving': float(cat['collision_moving'].float().mean()),
           'stalled': float(cat['stalled'].float().mean()),
           'timeout': float(cat['timeout'].float().mean()),
           'return': float(cat['return'].mean()), 'length': float(cat['length'].float().mean()),
           'path_len': float(cat['path_len'].mean()), 'time_s': float(cat['time'].mean()),
           'fallback_rate': float((cat['fallback'].double() / cat['length'].double()).mean())}
    for i, nm in enumerate(C.COST_NAMES):
        out['cost_' + nm] = float(cost[:, i].mean())
    if 'd_start' in cat:      # 진단 (env 에피소드 기록): 진행 비율·이동 거리·제자리 회전·경로 이탈·목표 최소 거리
        d0 = cat['d_start'].double().clamp_min(1e-6)
        ln = cat['length'].double().clamp_min(1.0)
        out['progress'] = float((1.0 - cat['d_final'].double() / d0).mean())
        out['travel_ratio'] = float((cat['travel'].double() /
                                     cat['path_len'].double().clamp_min(1e-6)).mean())
        out['spin_frac'] = float((cat['spin_steps'].double() / ln).mean())
        out['offpath_1m_frac'] = float((cost[:, 2] / ln).mean())
        out['max_e'] = float(cat['max_e'].double().mean())
        out['goal_min_median'] = float(cat['goal_min'].double().median())
        out['near_goal_0p5'] = float((cat['goal_min'] <= 0.5).double().mean())
    return out


@torch.no_grad()
def evaluate(model, env, episodes, deterministic=True, seed=None, max_steps=None):
    """
    정책을 env 에서 에피소드 episodes 개로 평가 → 요약 (env 는 평가 전용 인스턴스).

    추정량: env 마다 **리셋 직후 첫 에피소드 하나만** 센다 — 끝나는 순서와 무관한 iid 표본이다. 먼저 끝난
    에피소드 k 개를 세면 일찍 끝나는 결말(초반 충돌·짧은 경로)이 과대 표집되고 정체·시간 상한처럼 늦게 끝나는
    결말이 빠진다 (2026-10-06 리뷰: B_bc3 성공 0.408 대 0.532, 편향 방향은 결말 시간 분포에 따라 바뀐다).
    episodes > env.N 이면 전체 리셋 라운드를 되풀이하고, 마지막 라운드는 앞쪽 env 몇 개만 센다 (env 는 교환
    가능하다). seed 를 주면 라운드 r 앞에서 env 난수기를 seed + r 로 다시 심는다 — 같은 seed 로 평가한
    체크포인트들은 첫 에피소드의 시작·목표·DR·장애물 배치가 같다 (공통 난수).
    max_steps None 이면 센 env 의 시간 상한 t_limit 로 정한다 (모든 에피소드는 늦어도 시간 상한 truncation 으로
    끝난다). 그래도 끝나지 않은 env 는 'unfinished' 로 센다 — 0 이 아니면 그 요약은 다시 편향될 수 있다.
    """
    eps, unfinished, counted, rnd = [], 0, 0, 0
    while counted < episodes:
        k = min(env.N, episodes - counted)
        if seed is not None:
            env.gen.manual_seed(int(seed) + rnd)
        obs = env.reset()
        dev = obs.device
        want = torch.arange(env.N, device=dev) < k
        got = torch.zeros(env.N, dtype=torch.bool, device=dev)
        limit = max_steps
        if limit is None:
            limit = int(math.ceil(float(env.t_limit[want].max()) / C.DT)) + 5
        for _ in range(limit):
            mask = env.mask if env.cfg.arm == 'C' else None
            u, _a, _lp = model.act(obs, mask, deterministic=deterministic)
            obs, _r, _te, _tr, info = env.step(u)
            if 'done_idx' in info:
                di = info['done_idx']
                new = want[di] & ~got[di]
                if bool(new.any()):
                    eps.append({key: v[new] for key, v in info['episode'].items()})
                got[di] = True
                if bool((got | ~want).all()):
                    break
        unfinished += int((want & ~got).sum())
        counted += k
        rnd += 1
    s = episode_summary(eps)
    s.update({'estimator': 'first_episode_per_env', 'unfinished': unfinished, 'rounds': rnd,
              'eval_seed': seed})
    if s['episodes']:
        p = s['success']
        s['success_se'] = math.sqrt(p * (1.0 - p) / s['episodes'])
    return s


def export_onnx(model, path, meta, opset=17):
    """onnx_io.md 대로 내보낸다 (dynamo=False, 0 번 축 동적, metadata_props)."""
    import onnx
    model = model.eval().cpu()
    obs = torch.zeros(1, OBS_DIM)
    if model.arm == 'B':
        args, names_in, names_out = (obs,), ['obs'], ['action', 'mu', 'log_std']
    else:
        args = (obs, torch.ones(1, C.N_ACTIONS_C))
        names_in, names_out = ['obs', 'mask'], ['logits', 'action']
    dyn = {n: {0: 'batch'} for n in names_in + names_out}
    torch.onnx.export(model, args, path, input_names=names_in, output_names=names_out,
                      dynamic_axes=dyn, opset_version=opset, dynamo=False)
    m = onnx.load(path)
    base = {'contract_version': C.CONTRACT_VERSION, 'arm': model.arm, 'obs_dim': str(OBS_DIM),
            'dt': str(C.DT), 'normalization': 'inside', 'simulator': 'sim2d'}
    base.update({k: str(v) for k, v in meta.items()})
    del m.metadata_props[:]
    for k, v in base.items():
        p = m.metadata_props.add()
        p.key, p.value = k, v
    onnx.save(m, path)
    return base


def config_dict(cfg):
    """설정 dataclass → dict (튜플은 목록으로)."""
    d = asdict(cfg)
    return {k: (list(v) if isinstance(v, tuple) else v) for k, v in d.items()}


class Timer:
    """간단한 경과 시간."""

    def __init__(self):
        """시작."""
        self.t0 = time.time()

    def __call__(self):
        """경과 초."""
        return time.time() - self.t0


__all__ = ['ActorCriticB', 'ActorCriticC', 'PPOConfig', 'Trainer', 'evaluate', 'export_onnx',
           'episode_summary']
