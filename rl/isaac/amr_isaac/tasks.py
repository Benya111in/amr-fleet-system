"""
Gymnasium 과제 등록 — Isaac Sim 앱을 띄운 뒤 import 한다.

    AMR-Isaac-Nav-B-v0   : 학습 (경로 풀 seed 0, DR 켬)
    AMR-Isaac-Nav-B-Eval-v0 : 평가 (held-out 시작·목표 풀 seed 1, DR 끔 = 명목값)
"""

import gymnasium as gym

gym.register(
    id='AMR-Isaac-Nav-B-v0',
    entry_point='amr_isaac.env:AmrNavEnv',
    disable_env_checker=True,
    kwargs={
        'env_cfg_entry_point': 'amr_isaac.env_cfg:AmrNavEnvCfg',
        'rsl_rl_cfg_entry_point': 'amr_isaac.agents.rsl_rl_ppo_cfg:AmrNavPPORunnerCfg',
    },
)

gym.register(
    id='AMR-Isaac-Nav-B-Eval-v0',
    entry_point='amr_isaac.env:AmrNavEnv',
    disable_env_checker=True,
    kwargs={
        'env_cfg_entry_point': 'amr_isaac.env_cfg:AmrNavEvalEnvCfg',
        'rsl_rl_cfg_entry_point': 'amr_isaac.agents.rsl_rl_ppo_cfg:AmrNavPPORunnerCfg',
    },
)
