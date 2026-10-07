"""
비교군 B 정책의 계약형 모듈과 ONNX 내보내기 (rl/contract/onnx_io.md).

torch 만 쓴다 (rsl_rl·Isaac 불필요) — rsl_rl 체크포인트의 state dict 에서 같은 계산을 다시 짠다.
메타데이터(algo·train_steps·라그랑주 사실)는 학습 기록에서 만든다 (onnx_metadata, tests/test_policy_meta.py).
  x = clip((obs − μ)/√(σ² + 1e-8), −10, 10)    (σ² = max(학습 분산, VAR_FLOOR) — 학습과 같은 값)
  mu = actor_mlp(x), action = tanh(mu), log_std = 학습 매개변수 (상태 무관, 배치로 펼침)
SAC (sac_skrl, 체크포인트 형식 amr_sac_v1) 은 PolicySAC — 같은 입출력, log_std 는 상태 의존 출력.
load_policy(path) 가 체크포인트 형식으로 둘 중 하나를 고른다 (평가 스크립트는 형식을 몰라도 된다).
"""

import json
import re

import torch
from torch import nn

VAR_FLOOR = 1e-2     # 학습 중 정규화와 같은 분산 하한 (train_ppo.ContractNormalization)
NORM_EPS = 1e-8
NORM_CLIP = 10.0
OPSET = 17


def contract_normalize(x, mean, var):
    """계약 정규화 식."""
    return torch.clamp((x - mean) / torch.sqrt(torch.clamp(var, min=VAR_FLOOR) + NORM_EPS),
                       -NORM_CLIP, NORM_CLIP)


def _mlp_from_state(sd, prefix, activation='elu'):
    keys = sorted({int(m.group(1)) for k in sd for m in [re.match(prefix + r'\.(\d+)\.weight$', k)]
                   if m})
    layers = []
    for n, i in enumerate(keys):
        w = sd[f'{prefix}.{i}.weight']
        lin = nn.Linear(w.shape[1], w.shape[0])
        lin.weight.data.copy_(w)
        lin.bias.data.copy_(sd[f'{prefix}.{i}.bias'])
        layers.append(lin)
        if n < len(keys) - 1:
            layers.append({'elu': nn.ELU(), 'relu': nn.ReLU(), 'tanh': nn.Tanh()}[activation])
    return nn.Sequential(*layers)


class PolicyB(nn.Module):
    """obs [B, 291] → (action [B, 2], mu [B, 2], log_std [B, 2])."""

    kind = 'ppo_rsl_rl'

    def __init__(self, mean, var, actor, log_std):
        """정규화 통계·actor·log_std."""
        super().__init__()
        self.register_buffer('mean', mean.clone().float().reshape(1, -1))
        self.register_buffer('var', torch.clamp(var.clone().float(), min=VAR_FLOOR).reshape(1, -1))
        self.actor = actor
        self.log_std = nn.Parameter(log_std.clone().float().reshape(1, -1), requires_grad=False)

    def forward(self, obs):
        """계약 출력 셋."""
        x = torch.clamp((obs - self.mean) / torch.sqrt(self.var + NORM_EPS), -NORM_CLIP, NORM_CLIP)
        mu = self.actor(x)
        return torch.tanh(mu), mu, self.log_std.expand(obs.shape[0], -1)

    def log_std_summary(self):
        """기록용 log σ (상태 무관 매개변수)."""
        return self.log_std.flatten().tolist()

    @staticmethod
    def from_rsl_state_dict(sd, activation='elu'):
        """rsl_rl ActorCritic state dict (noise_std_type 'log', 관측 정규화 켬) → PolicyB."""
        if 'log_std' not in sd:
            raise ValueError("noise_std_type 'log' 체크포인트가 아니다 (log_std 없음)")
        return PolicyB(sd['actor_obs_normalizer._mean'].reshape(-1),
                       sd['actor_obs_normalizer._var'].reshape(-1),
                       _mlp_from_state(sd, 'actor', activation), sd['log_std'])


class PolicySAC(nn.Module):
    """SAC 짓눌린 가우시안 정책 — obs [B, 291] → (tanh(μ), μ, log σ(s)) (onnx_io.md 와 같은 이름·모양)."""

    kind = 'sac_skrl'

    def __init__(self, mean, var, trunk, mu_head, log_std_head, log_std_min, log_std_max):
        """정규화 통계·몸통·두 머리."""
        super().__init__()
        self.register_buffer('mean', mean.clone().float().reshape(1, -1))
        self.register_buffer('var', torch.clamp(var.clone().float(), min=VAR_FLOOR).reshape(1, -1))
        self.trunk = trunk
        self.mu_head = mu_head
        self.log_std_head = log_std_head
        self.log_std_min = float(log_std_min)
        self.log_std_max = float(log_std_max)

    def forward(self, obs):
        """계약 출력 셋."""
        x = torch.clamp((obs - self.mean) / torch.sqrt(self.var + NORM_EPS), -NORM_CLIP, NORM_CLIP)
        h = self.trunk(x)
        mu = self.mu_head(h)
        log_std = torch.clamp(self.log_std_head(h), self.log_std_min, self.log_std_max)
        return torch.tanh(mu), mu, log_std

    def log_std_summary(self):
        """기록용: 상태 의존이라 값이 없다."""
        return []

    @staticmethod
    def from_checkpoint(ck):
        """sac_skrl.export_state 의 dict → PolicySAC."""
        if ck.get('format') != 'amr_sac_v1':
            raise ValueError(f'SAC 체크포인트 형식이 아니다: {ck.get("format")!r}')
        act = {'elu': nn.ELU, 'relu': nn.ReLU, 'tanh': nn.Tanh}[ck['activation']]
        sizes = [ck['mean'].numel()] + list(ck['hidden'])
        layers = []
        for i in range(len(sizes) - 1):
            layers += [nn.Linear(sizes[i], sizes[i + 1]), act()]
        trunk = nn.Sequential(*layers)
        trunk.load_state_dict(ck['trunk'])
        n_act = ck['mu_head']['weight'].shape[0]
        mu_head = nn.Linear(sizes[-1], n_act)
        mu_head.load_state_dict(ck['mu_head'])
        ls_head = nn.Linear(sizes[-1], n_act)
        ls_head.load_state_dict(ck['log_std_head'])
        return PolicySAC(ck['mean'], ck['var'], trunk, mu_head, ls_head, ck['log_std_min'],
                         ck['log_std_max'])


def load_policy(path_or_ck):
    """체크포인트 (경로 또는 dict) → PolicyB (rsl_rl PPO·PPO-Lagrangian) 또는 PolicySAC."""
    ck = torch.load(path_or_ck, map_location='cpu') if isinstance(path_or_ck, str) else path_or_ck
    if ck.get('format') == 'amr_sac_v1':
        return PolicySAC.from_checkpoint(ck)
    return PolicyB.from_rsl_state_dict(ck['state_dict'], ck.get('activation', 'elu'))


def export_onnx(policy, path, metadata):
    """onnx_io.md 의 이름·모양·dtype·opset 으로 내보내고 metadata_props 를 붙인다."""
    import onnx
    policy = policy.eval().cpu()
    dummy = torch.zeros(1, policy.mean.shape[1])
    torch.onnx.export(policy, (dummy,), path, input_names=['obs'],
                      output_names=['action', 'mu', 'log_std'],
                      dynamic_axes={'obs': {0: 'batch'}, 'action': {0: 'batch'},
                                    'mu': {0: 'batch'}, 'log_std': {0: 'batch'}},
                      opset_version=OPSET, dynamo=False)
    m = onnx.load(path)
    for k, v in metadata.items():
        e = m.metadata_props.add()
        e.key = str(k)
        e.value = v if isinstance(v, str) else json.dumps(v)
    onnx.checker.check_model(m)
    onnx.save(m, path)
    return path


# ======================================================================== 메타데이터 (사실 그대로)
CONTRACT_D1_SWEEP = (0.02, 0.05, 0.1, 0.2)   # contract §7.2: d₁ 미측정 동안 훑을 개입 예산
RSL_RL = 'rsl_rl 3.1.2'


def lagrange_facts(summary, curve_rows):
    """
    학습 summary.json·curve.jsonl 행 → 라그랑주 승수가 실제로 무엇을 했는지.

    lagrange_all_zero_from_iteration = 그 기록부터 끝까지 λ 가 전부 0 인 첫 기록의 반복 번호
    (끝 λ 가 0 이 아니면 None). 반환 dict 의 키: configured, spec, d1, d1_in_contract_sweep, final,
    active_at_end, all_zero_from_iteration.
    """
    spec = summary.get('lagrange') or ''
    final = [float(x) for x in summary.get('lagrange_final') or []]
    d1 = float(spec.split('/')[2]) if spec else None
    zero_from = None
    for r in sorted(curve_rows, key=lambda r: r['iteration']):
        if all(float(x) == 0.0 for x in r.get('lagrange') or [1.0]):
            zero_from = r['iteration'] if zero_from is None else zero_from
        else:
            zero_from = None
    return {'configured': bool(spec), 'spec': spec, 'd1': d1,
            'd1_in_contract_sweep': d1 is not None and any(
                abs(d1 - x) < 1e-12 for x in CONTRACT_D1_SWEEP),
            'final': final, 'active_at_end': any(x > 0.0 for x in final),
            'all_zero_from_iteration': zero_from}


def pid_facts(summary):
    """
    B 단계 PID 라그랑주 학습 (train_ppo.py --constraint) 의 summary.json → lagrange_facts 와 같은 키 (없으면 None).

    final = [마지막 승수], active_at_end = 마지막 승수 > 0. spec 에 비용·예산·이득·상한을 적는다.
    """
    p = summary.get('pid_lagrangian')
    if not p:
        return None
    lam = float(p.get('lambda_final') or 0.0)
    d = float(p['budget_d'])
    return {'configured': True,
            'spec': (f"PID cost={p['cost']} d={d:g} Kp={p['kp']:g} Ki={p['ki']:g} Kd={p['kd']:g}"
                     f" lambda_max={p['lambda_max']:g}"),
            'd1': d if p['cost'] == 'intervention' else None,
            'd1_in_contract_sweep': any(abs(d - x) < 1e-12 for x in CONTRACT_D1_SWEEP),
            'final': [lam], 'active_at_end': lam > 0.0, 'all_zero_from_iteration': None,
            'pid': True, 'cost': p['cost'], 'budget': d}


def describe_algo(kind, summary, facts, bc_info=None):
    """정책 종류(bc·final)와 학습 기록에서 ONNX algo 문자열을 만든다 — 꺼진 라그랑주를 주장하지 않는다."""
    if kind == 'bc':
        n = (bc_info or {}).get('bc_samples', '?')
        return f'BC (expert pure pursuit, {n} samples), no RL fine-tuning'
    if str(summary.get('algo', '')).startswith('SAC'):
        return summary['algo'] + ', unconstrained (no cost constraint)'
    if facts.get('pid'):
        base = (f'PPO-PIDLagrangian ({RSL_RL} + rl/isaac ppo_lag, cost {facts["cost"]} '
                f'<= {facts["budget"]:g} undiscounted per episode')
        if facts['active_at_end']:
            return base + f', final lambda {facts["final"][0]:g})'
        return base + (', final lambda 0 = constraint not binding at end, effectively'
                       ' unconstrained at end)')
    pre = 'BC warm start + ' if summary.get('bc_steps', 0) > 0 else ''
    if not facts['configured']:
        # 승수를 켜지 않은 학습 = 비용 제약 없는 PPO (계약 §7 의 제약을 쓰지 않았다는 사실을 적는다)
        return pre + f'PPO ({RSL_RL}), unconstrained (no Lagrangian / cost constraint)'
    where = 'in' if facts['d1_in_contract_sweep'] else 'outside'
    if facts['active_at_end']:
        return pre + (f'PPO-Lagrangian ({RSL_RL}, d1={facts["d1"]:g}, {where} contract 7.2 sweep,'
                      f' final lambda {facts["final"]})')
    return pre + (f'PPO ({RSL_RL}); Lagrangian dual ascent configured (d1={facts["d1"]:g},'
                  f' {where} contract 7.2 sweep) but all multipliers 0 from iteration'
                  f' {facts["all_zero_from_iteration"]} = effectively unconstrained PPO')


def algo_claim_ok(algo, facts):
    """알고리즘 문자열이 라그랑주를 주장하면 끝 λ 가 0 이 아니거나 '제약 없음'을 함께 밝혀야 한다."""
    a = algo.lower()
    if 'lagrang' not in a:
        return True
    return facts['active_at_end'] or 'unconstrained' in a


def onnx_metadata(kind, summary, facts, head, contract_version, algo=None, bc_info=None):
    """onnx_io.md 필수·권장 키 + 학습 사실 (모두 문자열)."""
    meta = {'contract_version': contract_version, 'arm': 'B', 'obs_dim': '291', 'dt': '0.1',
            'normalization': 'inside',
            'algo': algo or describe_algo(kind, summary, facts, bc_info),
            'seed': str(summary.get('seed', '')), 'git_commit': head, 'simulator': 'isaac',
            'normalization_var_floor': str(VAR_FLOOR)}
    if kind == 'bc':
        meta['train_steps'] = '0'
        meta['bc_samples'] = str((bc_info or {}).get('bc_samples', ''))
        return meta
    meta['train_steps'] = str(summary.get('env_steps', ''))
    if summary.get('reward_version'):
        meta['reward_version'] = str(summary['reward_version'])
    if summary.get('path_len'):
        meta['train_path_len_m'] = str(summary['path_len'])
    if not facts['configured']:
        meta['constraint'] = 'none'
    if facts['configured']:
        meta['lagrange_spec'] = facts['spec']
        meta['lagrange_final'] = json.dumps(facts['final'])
        z = facts['all_zero_from_iteration']
        meta['lagrange_all_zero_from_iteration'] = '' if z is None else str(z)
        if facts['d1'] is not None:
            meta['intervention_budget_d1'] = f'{facts["d1"]:g}'
        meta['intervention_budget_in_contract_sweep'] = str(facts['d1_in_contract_sweep']).lower()
    if facts.get('pid'):
        meta['constraint'] = f'{facts["cost"]} <= {facts["budget"]:g} (undiscounted episode sum)'
    if summary.get('dr') is not None:
        meta['train_dr'] = str(bool(summary['dr'])).lower()
    if summary.get('safety_filter'):
        meta['train_safety_filter'] = str(summary['safety_filter'])
    return meta
