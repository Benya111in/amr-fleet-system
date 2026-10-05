"""
Isaac Lab DirectRLEnv 설정 — AMR 내비게이션 (계약 v1.1.0, 비교군 B).

Isaac Sim 앱이 뜬 뒤에만 import 한다 (isaaclab 모듈 의존).
시간: 물리 dt 0.02 s (프로파일러 50 Hz, V:6) × decimation 5 = 결정 주기 0.1 s (contract.md §3).
"""

from collections.abc import Callable

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import RayCasterCfg
from isaaclab.sensors.ray_caster.patterns import PatternBaseCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from .contract_torch import C, OBS_DIM

BODY_Z = 0.18           # [m] 차체 중심 높이 (상자 0.03–0.33, sensors.yaml base_link_height 0.18)
WAREHOUSE_PRIM = '/World/Warehouse'


def contract_lidar_pattern(cfg, device):
    """계약 빔 각 (angle_min + i·inc, 720 빔, 수평) — Isaac LidarPatternCfg 의 linspace 대신."""
    ang = C.LIDAR_ANGLE_MIN + torch.arange(C.LIDAR_SAMPLES, dtype=torch.float64) * \
        C.LIDAR_ANGLE_INC
    dirs = torch.stack([torch.cos(ang), torch.sin(ang), torch.zeros_like(ang)], dim=1)
    starts = torch.zeros_like(dirs)
    return starts.to(device=device, dtype=torch.float32), dirs.to(device=device,
                                                                  dtype=torch.float32)


@configclass
class ContractLidarPatternCfg(PatternBaseCfg):
    """2D LiDAR 패턴 (계약 빔 각)."""

    func: Callable = contract_lidar_pattern


@configclass
class DomainRandCfg:
    """DR 범위 (rl/contract/amr_rl_contract/dr.py DR_AXES 의 부분집합 — 1 차 구현분)."""

    enable: bool = True
    plant_time_constant_s: tuple = (0.048, 0.112)    # 축 2 (V:16 × [0.6, 1.4])
    plant_delay_s: tuple = (0.02, 0.08)              # 축 2 (V:17) — 0.02 s 단위로 양자화
    accel_limit_scale: tuple = (0.84, 1.0)           # 축 2 (R:87)
    lidar_sigma_m: tuple = (0.01, 0.06)              # 축 5
    beam_dropout_p: tuple = (0.0, 0.02)              # 축 5
    yaw_noise_rad: float = 0.5                       # contract §10 시작 yaw = 접선 + U[−0.5, 0.5]
    nominal_plant_time_constant_s: float = 0.08
    nominal_plant_delay_s: float = 0.04
    nominal_lidar_sigma_m: float = 0.03


@configclass
class AmrNavEnvCfg(DirectRLEnvCfg):
    """창고 내비게이션 env 설정."""

    decimation = 5
    episode_length_s = C.TIME_LIMIT_MAX
    action_space = 2
    observation_space = OBS_DIM
    state_space = 0
    is_finite_horizon = False      # 시간 상한 = truncation (bootstrap), contract §6

    sim: SimulationCfg = SimulationCfg(dt=C.PROFILER_DT, render_interval=5)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1024, env_spacing=0.0,
                                                     replicate_physics=True)

    # 자세를 계약 운동학으로 매 결정마다 써 넣는 강체 (중력·몸체 충돌 끔 — 충돌 판정은 지면 진실 기하).
    # kinematic_enabled=True 는 이 호스트 GPU PhysX 에서 set_transforms 가 CUDA 700 (illegal memory
    # access, GpuRigidBodyView.cpp:512) 으로 죽었다 → 동적 강체 + 바닥 평면 (README 「Isaac 함정」).
    robot: RigidObjectCfg = RigidObjectCfg(
        prim_path='/World/envs/env_.*/Robot',
        spawn=sim_utils.CuboidCfg(
            size=(C.FOOTPRINT_LENGTH, C.FOOTPRINT_WIDTH, 0.30),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False,
                                                         disable_gravity=True),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
            mass_props=sim_utils.MassPropertiesCfg(mass=60.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.95, 0.55, 0.10)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, BODY_Z)),
    )

    # Warp 레이캐스트 2D LiDAR: 원점 base_link (0.15, 0), 스캔 평면 지면 +0.20 m (S:44, sensors.yaml)
    lidar: RayCasterCfg = RayCasterCfg(
        prim_path='/World/envs/env_.*/Robot',
        mesh_prim_paths=[WAREHOUSE_PRIM],
        offset=RayCasterCfg.OffsetCfg(pos=(C.LIDAR_X, C.LIDAR_Y, 0.20 - BODY_Z)),
        # 'yaw' 는 시작점만 yaw 로 돌리고 방향은 세계 고정이다 (높이 스캐너용, ray_caster.py
        # _update_ray_infos). LiDAR 는 방향도 돌아야 하므로 'base' (몸체가 평면 운동이라 yaw 와 같은 회전).
        ray_alignment='base',
        pattern_cfg=ContractLidarPatternCfg(),
        max_distance=C.LIDAR_RANGE_MAX,
        update_period=0.0,
        debug_vis=False,
    )

    assets_dir: str = '/home/CAPYI_AI/runs/r1-isaac/assets'
    path_pool_seed: int = 0          # 0 = 학습 풀, 1 = 평가(held-out 시작·목표) 풀
    dr: DomainRandCfg = DomainRandCfg()
    lidar_source: str = 'sensor'     # 'sensor' = RayCaster 데이터, 'direct' = 같은 Warp 메시에 직접
    v_cap: float = C.V_MAX_OP
    stats_window: int = 1024         # 최근 에피소드 결과 창 (로그용)
    path_pad: int = 640              # 경로 점 패딩 (풀 최대 602)
    max_tracks: int = 16             # 트랙 버퍼 (관측은 상위 8, contract §4)


@configclass
class AmrNavEvalEnvCfg(AmrNavEnvCfg):
    """평가 설정: held-out 시작·목표 풀 (seed 1), DR 끔 (명목값)."""

    path_pool_seed: int = 1
    dr: DomainRandCfg = DomainRandCfg(enable=False)
