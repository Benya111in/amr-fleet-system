"""
Domain Randomization 축과 범위 (contract.md §10) — 우선순위 순서.

환경(r1·r2)은 이 표의 축·범위·분포를 그대로 쓰고, 샘플링 코드는 각자 둔다.
범위의 근거 강도는 contract.md §10 표에 적었다. 'nominal' 은 Gazebo 기본값이다.
"""

DR_AXES = (
    # 1. 맵 다양성 (근거 강함: arXiv 2210.04839 5→250 맵 43→74 %, arXiv 2605.02528)
    {'axis': 'map', 'priority': 1,
     'generators': ('rack_aisles', 'narrow_passage', 'cross_aisles', 'dead_end_dock', 'open_hall'),
     'n_maps_min': 1000, 'real_map_fraction_max': 0.2,
     'narrow_width_m': (0.60, 1.20), 'aisle_width_m': (1.2, 5.0),
     'resolution_m': 0.05, 'heldout': 'stratified_by_generator'},
    # 2. 가속도 동역학 (근거 강함: arXiv 2606.12042 0.99→0.52; 중간: 2605.15559 게인 ±40 %)
    {'axis': 'dynamics', 'priority': 2,
     'plant_time_constant_s': {'nominal': 0.08, 'range': (0.048, 0.112)},     # V:16 ×[0.6, 1.4]
     'plant_delay_s': {'nominal': 0.04, 'range': (0.02, 0.08)},              # V:17
     'accel_limit_scale': {'nominal': 1.0, 'range': (0.84, 1.0)},            # R:87 25 kg 0.843
     'payload_kg': {'values': (0.0, 2.0, 10.0, 25.0)}},                      # R:100-112
    # 3. 장애물 행동 (근거 중간: 사람 / 없음: 지게차 — 설계 선택)
    {'axis': 'obstacles', 'priority': 3,
     'worker': {'radius_m': 0.30, 'speed_mps': (0.3, 1.5), 'accel_mps2': (0.5, 1.0),
                'patterns': ('straight', 'curve', 'random', 'crossing')},
     'forklift': {'footprint_m': (-1.2, 2.0, -0.64, 0.64), 'speed_mps': (1.0, 1.5),
                  'accel_mps2': 1.0},
     'other_amr': {'footprint_m': (-0.30, 0.30, -0.20, 0.20), 'speed_mps': (0.5, 1.0),
                   'accel_mps2': 0.8},
     'reactive_fraction': 0.0, 'count': (0, 8)},
    # 4. 지연 (근거 중간: arXiv 2605.15559 U[0,0.1]/U[0,0.04] 를 하한으로)
    {'axis': 'latency', 'priority': 4,
     'obs_delay_s': (0.0, 0.15), 'action_delay_s': (0.0, 0.06),
     'eval_sweep_s': (0.0, 0.3)},
    # 5. 인지 누락·잡음 (근거 중간: 2605.15559, 2411.12150 / LiDAR 는 약함)
    {'axis': 'perception', 'priority': 5,
     'track_drop_p': (0.0, 0.3), 'track_pos_sigma_m': (0.0, 0.10),
     'track_vel_sigma_mps': (0.0, 0.20), 'lidar_sigma_m': (0.01, 0.06),
     'beam_dropout_p': (0.0, 0.02)},
)
