"""
계약 상수 — 모든 값의 출처는 저장소 파일:행 (develop d8ae80c 기준) 또는 contract.md 의 절.

이 파일이 단일 진실 공급원이다. r1(Isaac)·r2(2D)·r3(C++ 플러그인)은 값을 복사하지 말고
golden 벡터(golden/*.json 의 "constants" 블록)와 대조해 같은 값을 쓰는지 시험한다.
출처 표기: R = config/robot_params.yaml, S = config/sensors.yaml,
N = src/amr_navigation/config/nav2_params.yaml,
V = src/amr_navigation/config/velocity_profiler.yaml,
P = src/amr_perception/config/perception.yaml.
"""

import math

CONTRACT_VERSION = '1.2.0'   # contract.md 「변경 이력」

# ---------------------------------------------------------------- 로봇 기하·한계
FOOTPRINT_LENGTH = 0.60          # [m] R:15
FOOTPRINT_WIDTH = 0.40           # [m] R:16
HALF_L = FOOTPRINT_LENGTH / 2.0  # 풋프린트는 base_link(=회전 중심) 중심 사각형 (R:8, footprint.hpp rectangle)
HALF_W = FOOTPRINT_WIDTH / 2.0
ROBOT_RADIUS = math.hypot(HALF_L, HALF_W)  # 0.3606 외접원. N:356 (≤0 → 외접), P:216 (0.361)
WHEEL_SEPARATION = 0.36          # [m] R:27 (계약 계산에는 쓰지 않음 — 2D 동역학 참고값)
WHEEL_RADIUS = 0.0825            # [m] R:26

V_MAX_HW = 2.0                   # [m/s] R:57 limits.max_linear_velocity (하드웨어 한계)
V_MAX_OP = 1.0                   # [m/s] N:104 DWA max_vel_x (운용 상한 — 모든 비교군 공통 v_cap 기본값)
V_MIN = 0.0                      # [m/s] N:105 min_vel_x (후진 없음 — DWA 와 같게)
W_MAX = 1.5                      # [rad/s] R:60
A_MAX = 1.0                      # [m/s²] R:59 가속·감속 공통
ALPHA_MAX = 2.0                  # [rad/s²] R:61
JERK_MAX = 2.0                   # [m/s³] R:62
ANG_JERK_MAX = 6.0               # [rad/s³] V:11 (velocity_profiler max_angular_jerk)
E_STOP_DECEL = 3.0               # [m/s²] R:71 (DiffDrive 감속 한계 — 계약 계산에는 안 씀)

# ---------------------------------------------------------------- 주기
CONTROLLER_HZ = 20.0             # N:75 controller_frequency (플러그인 호출 주기)
LIDAR_HZ = 10.0                  # S:42
DT = 1.0 / LIDAR_HZ              # [s] 정책 결정 주기 0.1 s (contract.md §3.1)
PROFILER_HZ = 50.0               # V:6
PROFILER_DT = 1.0 / PROFILER_HZ

# ---------------------------------------------------------------- LiDAR
LIDAR_SAMPLES = 720              # S:28
LIDAR_ANGLE_MIN = -3.14159265    # [rad] S:38  (빔 0 = −180°, 빔 360 = 0° 전방)
LIDAR_ANGLE_MAX = 3.13286601     # [rad] S:39
LIDAR_ANGLE_INC = (LIDAR_ANGLE_MAX - LIDAR_ANGLE_MIN) / (LIDAR_SAMPLES - 1)  # 0.5° (S:29-35)
LIDAR_RANGE_MIN = 0.10           # [m] S:40
LIDAR_RANGE_MAX = 25.0           # [m] S:41
LIDAR_NOISE_STD = 0.03           # [m] S:43
LIDAR_X = 0.15                   # [m] S:44 extrinsic (base_link 기준)
LIDAR_Y = 0.0

# ---------------------------------------------------------------- 관측
N_SECTORS = 180                  # 2° 섹터, 섹터당 4 빔, 풋프린트 여유거리 최솟값 풀링 (contract.md §4.2)
BEAMS_PER_SECTOR = LIDAR_SAMPLES // N_SECTORS
SECTOR_CLIP = 10.0               # [m] 여유거리 상한 (정규화 분모)
N_PREV_ACTIONS = 3               # 최근 행동 수 (이론 보고서 「지연은 … 관측 증강」)
N_SUBGOALS = 5                   # A* 경로 위 0.5 m 간격 5 점 (실무 보고서 관측 절, arXiv 2605.02528)
SUBGOAL_SPACING = 0.5            # [m]
SUBGOAL_DIST_NORM = 2.5          # [m] = N_SUBGOALS × SUBGOAL_SPACING
GOAL_CLIP = 10.0                 # [m]
K_TRACKS = 8                     # 추적 객체 상위 k
TRACK_RANGE = 10.0               # [m] 관측에 넣는 트랙 거리 상한
TRACK_POS_NORM = 10.0
TRACK_VEL_NORM = 2.0             # [m/s] 지게차 1.5 m/s 를 덮는다
TRACK_FEATS = 8                  # px, py, vx, vy, dist, ttc, is_dynamic, valid
TTC_HORIZON = 5.0                # [s] P:210 추적기 TTC 지평
OBSTACLE_RADIUS = 0.30           # [m] N:341 DWA 트랙 덮개 반경 (TrackedObstacle 에 반경 필드 없음)
SCAN_AGE_NORM = 0.3              # [s] = LiDAR 고장 타임아웃 R:143
TRACK_AGE_NORM = 0.5             # [s] N:348 track_timeout
DEV_PEAK_NORM = 1.0              # [m] 명세 4.7 이탈 1 m (31_*.md:190)
DEV_TIMER_NORM = 5.0             # [s] 명세 4.7 복귀 5 s (31_*.md:191)

# ---------------------------------------------------------------- DWA (가행성·VO, 비교군 C 마스크)
SIM_DT = 0.1                     # [s] N:136
SIM_TIME_MIN = 1.5               # [s] N:137
SIM_TIME_MAX = 2.5               # [s] N:138  T_sim = clip(|v|/a + 0.5, min, max) dwa.cpp:60-64
GOAL_OVERSHOOT_MARGIN = 0.1      # [m] N:178
VO_TIME_HORIZON = 2.0            # [s] N:185
VO_MARGIN = 0.3                  # [m] N:186
VO_MAX_RANGE = 6.0               # [m] N:187
DYN_SPEED_THRESHOLD = 0.2        # [m/s] N:188 (dwa.cpp 동적 장애물 선별)
DYN_FAST_SPEED = 0.5             # [m/s] N:189 (dwa_controller.cpp:341 is_dynamic 아니면 이 속도 이상만)
VO_TIME_TIE = 0.02               # [s] N:191 VO 포화 시 동률 폭
FP_PAD = 0.0                     # [m] 풋프린트 충돌 판정 팽창 (DWA 치명 셀 외곽선 판정과 같게 0)

# ---------------------------------------------------------------- 비교군 C 격자 (contract.md §5.2)
LATTICE_AV = tuple(round(-1.0 + 0.2 * i, 10) for i in range(11))   # [m/s²] 간격 0.2 = JERK_MAX·DT
LATTICE_AW = tuple(round(-2.0 + 0.5 * i, 10) for i in range(9))    # [rad/s²] 간격 0.5 ≤ 0.6
N_CELLS = len(LATTICE_AV) * len(LATTICE_AW)   # 99
FALLBACK_INDEX = N_CELLS                      # 99 — 항상 가용
N_ACTIONS_C = N_CELLS + 1                     # 100
JERK_STEP_V = JERK_MAX * DT                   # 0.2 m/s² (한 결정 주기에 바꿀 수 있는 가속도)
JERK_STEP_W = ANG_JERK_MAX * DT               # 0.6 rad/s²
LATTICE_TOL = 1e-6

# ------------------------------------------- 안전 (safety_lite = safety_gate 의 속도 상한 부분)
E_STOP_DIST = 0.30               # [m] R:121
CRITICAL_DIST = 0.50             # [m] R:122
WARNING_DIST = 1.00              # [m] R:123
WARNING_MAX_SPEED = 0.5          # [m/s] R:133 (풋프린트 최고 점 속도)
CRITICAL_MAX_SPEED = 0.2         # [m/s] R:134
REACTION_LATENCY = 0.15          # [s] R:135
SAFETY_DECEL = 1.0               # [m/s²] = A_MAX (safety_gate.hpp max_deceleration)
TTC_CRITICAL = 2.15              # [s] safety_node.cpp:128 ttc.critical 기본값
ZONE_CLEAR, ZONE_WARNING, ZONE_CRITICAL, ZONE_STOP = 0, 1, 2, 3

# ---------------------------------------------------------------- 목표·종료
GOAL_XY_TOL = 0.10               # [m] N:98
GOAL_STOP_SPEED = 0.05           # [m/s] 도달 판정 속도 (설계 선택, contract.md §8)
STALL_RADIUS = 0.3               # [m] N:86
STALL_TIME = 20.0                # [s] N:87
TIME_LIMIT_BASE = 20.0           # [s] 시간 상한 = BASE + PER_M × 경로 길이 (설계 선택)
TIME_LIMIT_PER_M = 3.0           # [s/m] (1/3 m/s 평균 속도까지 허용)
TIME_LIMIT_MAX = 120.0           # [s] (COLLISION_PENALTY 와 짝 — contract.md §7.2)

# ------------------------------- 이탈 상태기계 (tests/integration/amr_itest/metrics.py:240)
DEV_OUT = 0.30                   # [m] 이탈 구간 시작 (metrics.py:240 out_thr)
DEV_BACK = 0.15                  # [m] 복귀 (metrics.py:241 back_thr)
DEV_LIMIT = 1.0                  # [m] 명세 31_*.md:190
RETURN_LIMIT = 5.0               # [s] 명세 31_*.md:191 (최대 이탈 시각 → 복귀, metrics.py:235)
CTE_TRACK_LIMIT = 0.10           # [m] 명세 31_*.md:554 곡선 CTE 10 cm (§9 점검표, 설계 선택으로 비용화)
CTE_REF = 0.05                   # [m] 명세 31_*.md:554 직선 CTE 5 cm

# ---------------------------------------------------------------- 보상·할인 (contract.md §6, §7)
GAMMA = 0.995                    # 물리 지평 Δt/(1−γ) = 20 s
GAE_LAMBDA = 0.97                # 신용할당 창 1/(1−γλ) = 28.7 step = 2.87 s
KAPPA = 1.0                      # [1/m] 퍼텐셜 Φ = −κ·d (PBRS)
PHI_DIST = 'eff'                 # Φ·진행 효용의 거리 d = max(d_arc, 목표 직선거리) (v1.2.0, contract.md §6)
PROGRESS_KAPPA = 0.5             # κ_b: 기저 진행 효용 κ_b·(d − d') (v1.2.0, PBRS 아님 — §6.1)
DEV_HINGE = 0.05                 # 1 m 초과 이탈 힌지 −q_d·1[e > DEV_LIMIT] /스텝 (v1.2.0)
TIME_COST = 0.02                 # 스텝당 시간 비용 (= 0.2 /s)
GOAL_REWARD = 10.0
COLLISION_PENALTY = 30.0      # > TIME_COST × 시간 상한 스텝(0.02×1200=24): 무할인 τ_crash ≺ τ_idle
SMOOTH_WEIGHT = 0.0125           # −w·((Δa_v/A_MAX)² + (Δa_w/ALPHA_MAX)²)
TRACK_WEIGHT = 0.004             # −q·min(e/CTE_REF, TRACK_CLIP)²  (추종 모드만)
TRACK_CLIP = 3.0
INTERVENTION_V_NORM = V_MAX_OP   # 개입 크기 정규화
INTERVENTION_W_NORM = W_MAX

COST_NAMES = ('collision', 'intervention', 'deviation_1m', 'return_5s', 'cte_track')
N_COSTS = len(COST_NAMES)
# 에피소드 비할인 비용 예산 기본값 (d > 0, Slater) — contract.md §7.3
COST_BUDGETS = (0.02, None, 5.0, 10.0, 50.0)
