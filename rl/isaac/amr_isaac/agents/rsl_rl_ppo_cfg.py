"""
rsl_rl PPO 설정 (비교군 B).

계약 §9: γ 0.995, GAE λ 0.97, rollout ≥ 64 스텝, 병렬 env 1024–2048 권장. 그 밖(학습률·망 크기·배치)은
계약이 아니다 [설계 선택]. 관측 정규화는 rsl_rl EmpiricalNormalization 의 통계를 쓰되, 학습 스크립트가
forward 를 계약 식 clip((x − μ)/√(σ² + 1e-8), −10, 10) 으로 바꾼다 (onnx_io.md — 모델 안 정규화와 같게).
noise_std_type 'log' — ONNX 출력 log_std 와 같은 매개변수.
"""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class AmrNavPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """PPO 러너 설정."""

    num_steps_per_env = 64
    max_iterations = 300
    save_interval = 50
    experiment_name = 'amr_isaac_nav_b'
    obs_groups = {'policy': ['policy'], 'critic': ['policy']}
    clip_actions = None
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.6,
        noise_std_type='log',
        actor_obs_normalization=True,
        critic_obs_normalization=True,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation='elu',
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.003,
        num_learning_epochs=5,
        num_mini_batches=8,
        learning_rate=3.0e-4,
        schedule='adaptive',
        gamma=0.995,
        lam=0.97,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )
