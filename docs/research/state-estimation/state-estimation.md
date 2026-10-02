# 상태 추정 설계 브리프 — EKF 센서 퓨전 · AMCL 튜닝/납치 복구 · 2D SLAM 파라미터 · 맵 품질 지표 (명세 4.3)

- 작성일: 2026-09-21, **개정 2026-09-22 (리뷰 반영 + 감사 수정 §10.1 + 재개 감사 §10.2)** · 슬러그: `state-estimation` · 대상 스택: ROS2 Humble, Nav2 1.1.20 (nav2_amcl 1.1.20), robot_localization 3.5.4, slam_toolbox 2.6.10, Gazebo Fortress (gz-sim 6.18.0), ros_gz_bridge 0.244.26 — `amr-fleet-system:latest`에서 실측, 감사 시 `amr-fleet-system:wf-final`에서 동일 버전 재확인.
- 인터페이스 이름은 `docs/architecture/components.md` §3.2·§5.2 계약(상대 이름, 로봇 네임스페이스 아래)을 따른다. 계약을 바꾸는 항목(`amcl_pose_gated`, `kidnap_monitor_node` C++화, EKF `set_pose` 리맵)은 §7 "계약 변경 요청"에 모았다.
- 웹 도구: **사용 가능했음.** arXiv export API는 429가 잦아 `arxiv.org/abs/<id>` 직접 조회로 대체했고, Nav2/robot_localization/gz-sim은 GitHub 소스(humble / ign-gazebo6 브랜치)로 검증했다. 각 인용은 **VERIFIED**(초록/소스를 직접 조회) 또는 **RECALLED**(고전 문헌, 기억 기반; 서지 정보만 검색으로 확인한 경우 "서지 VERIFIED"로 병기)로 표기한다.
- 명세 목표(하드 요구): 위치 오차 **정지 3 cm / 직선 5 cm / 회전 8 cm**, Ground Truth 대비 **RMSE·Max error** 리포트(≥100 샘플, `[timestamp, gt_x, gt_y, est_x, est_y, error]`), AMCL 파티클/리샘플링 튜닝, **납치(Kidnapped) 후 제자리 회전 등으로 자가 복구**, slam_toolbox 파라미터 의미·튜닝 근거, 맵 품질 지표 정의·측정, 센서 공분산 근거 문서화, **IMU 바이어스 보정**(명세 2장), 오도메트리 드리프트 측정 실험(명세 2장), 통합 테스트 ≥10개 자동화·연속 4시간 운용(명세 10장).

---

## 0. 요약 (TL;DR)

| 항목 | 결론 |
| --- | --- |
| 기준선(baseline) | (1) 휠 오도메트리 → (2) IMU 전처리(LPF + 정지 바이어스 보정) → (3) robot_localization `ekf_node` 이중 필터(odom/map, REP-105) → (4) nav2_amcl(likelihood-field, KLD, augmented MCL) → (5) slam_toolbox online_async 매핑. 팀 자체 **6-상태 EKF(C++, 자이로 바이어스 상태 포함)**를 주 산출물로 구현하고 robot_localization을 비교 기준으로 둔다. |
| 독자 알고리즘 | **A. IAG-EKF** — 인코더·IMU **물리 잔차**(창 0.2 s)의 χ²₂ 슬립 검정으로 휠 오도메트리 갱신을 게이팅하고, 필터 무관 **표본별** 물리 잔차의 1 s 표본분산으로 R을 유계 적응(감사 수정). 위치: Reina 2006 잔차 검정 + Mehra/Mohamed-Schwarz IAE의 **변형**(FusionCore 2026이 χ² 게이팅+innovation 적응을 이미 ROS 2에서 결합) → 기여는 **폐형 임계·공분산 유도와 Nav2/A-B 통합**. **B. SMC-CUSUM** — 동적 빔 마스크 후 스캔-맵 인라이어 비율에 CUSUM + EKF NIS. 위치: Akai 2023 신뢰도 추정·Campbell 2015 지표 분류기의 **변형 + 순차검정 결합(new_combination, 좁은 의미)**. 대칭 통로 "조용한 납치"는 잡지 못함을 명시. **C. 능동 복구 FSM** — CAER형 캡 평균거리 점수로 coarse-to-fine(0.5 m × 5° → 0.1 m × 2°) 가설 시드 + 마스크 점수 판정·ICP 정련 → 다봉(주기적 랙 통로의 기본값)이면 모드를 odom으로 추적하며 **이동으로 판별** → 실패 시 σ_hit·max_particles 동적 확장 후 균일 재초기화 → 회전·검증·복귀(감사 시 재설계). 위치: CBGL/SFHS + Fox 1998 능동 정위의 **Nav2 엔지니어링 적응**. |
| 3/5/8 cm 달성 핵심 | σ_hit ≈ 0.05 m(튜닝 변수, α와 공동 스윕), max_beams 180, AMCL 공분산 하한, `smooth_lagged_data`(2 m/s·100 ms = 20 cm), ZUPT + nomotion update, 맵 ADNN ≤ 3 cm(목표 2 cm), 자이로 바이어스 보정. |
| GT 측정 | Gazebo `OdometryPublisher`(world pose) → `nav_msgs/Odometry` 브리지 → 시뮬 시간 보간 → SE(2) 정렬 후 RMSE/Max, 구간(정지/직선/회전) 분리. |

---

## 1. 명세·스택 제약 정리 (config/sensors.yaml · ekf.yaml · robot_params.yaml 실측)

- LiDAR 720빔 10 Hz σ=0.03 m, 최대 25 m, 장착 offset (0.15, 0, 0.20) m from base_link; **스캔 평면 = 지면 +0.38 m**(base_footprint→base_link z 0.18 + 0.20).
- IMU 100 Hz: 가속도 백색 σ_acc = 0.017 m/s², **정적 바이어스 |b_a| ~ N(0.10, 0.001) m/s²**(축별 무작위 부호, 시작 시 1회 추출, dynamic bias 0); 자이로 백색 σ_g = 2×10⁻⁴ rad/s, **정적 바이어스 |b_g| ~ N(0.01, 7.5×10⁻⁶) rad/s**. sensors.yaml 주석은 10 s 정지 평균 보정을 전제한다(잔차 σ: 자이로 2×10⁻⁴/√1000 = 6.3×10⁻⁶ rad/s, 가속도 0.017/√1000 = 5.4×10⁻⁴ m/s²). 계약(components.md §5.2, sensor_calibration.md §2.4)상 보정은 `imu_filter_node`(`imu/data_raw` → LPF·바이어스 제거 → `imu/data`)가 하며 `bias_estimation_time` 기본값은 **60 s**(잔차 자이로 2.6×10⁻⁶ rad/s, 가속도 2.2×10⁻⁴ m/s²). 이하 수치는 보수적으로 T_cal = 10 s 값을 쓰고 60 s 값을 병기한다. → §2.2에서 바이어스는 **백색잡음이 아니라 상태/보정량**으로 다룬다(개정).
- 휠 인코더 4096 tick/rev, 50 Hz, 슬립 잡음 Δŝ = Δs(1 + N(0, 0.01)) — **무차원 비율**(sensors.yaml 명시; 원본 브리프의 "단위 미정"은 해소). r = 0.0825 m, b = 0.36 m.
- 동역학: v ≤ 2.0 m/s, a ≤ 1.0 m/s², ω ≤ 1.5 rad/s, ω̇ ≤ 2.0 rad/s². 맵 60 × 40 m, 0.05 m → 1200 × 800 = 960 000 셀, 자유공간 ≈ 1000 m²(랙 제외 추정).
- **TF 규약(개정)**: URDF 루트 = `base_footprint`(robot_params.yaml), `base_footprint→base_link` z = 0.18 고정. 두 EKF 모두 `base_link_frame: base_footprint`(repo ekf.yaml, 명세 TF 예시와 동일), AMCL `base_frame_id: base_footprint`, map→odom은 ekf_map이 T_map→base_footprint·T_odom→base_footprint⁻¹로 합성.
- 설치 확인(컨테이너 실측 + humble 소스): nav2_amcl 서비스 `reinitialize_global_localization`, `request_nomotion_update`(std_srvs/Empty), `set_initial_pose`(nav2_msgs/srv/SetInitialPose); 토픽 `amcl_pose`, `particle_cloud`; **동적 파라미터**(amcl_node.cpp `dynamicParametersCallback`): `max_particles`/`min_particles`/`pf_err`/`pf_z`/`recovery_alpha_*` → `reinit_pf`(pf 재생성, `init_pose_/init_cov_`에서 재초기화 — **`init_pose_`는 `initOdometry()`(`on_configure` 시·α1–5/`robot_model_type` 변경 시; `on_activate`·`set_initial_pose`에서는 갱신 안 됨)에서만 `last_published_pose_`로 갱신되므로 복구 시점에는 낡은 값**, 감사 시 소스 재확인·재개 감사 시 호출 위치 정정). 여러 파라미터는 `set_parameters_atomically`로 한 번에 보내 콜백·`reinit_pf`를 1회로 만든다(비원자 `set_parameters`는 파라미터마다 콜백 호출), `sigma_hit`/`z_hit`/`z_rand`/`laser_likelihood_max_dist`/`laser_model_type`/`max_beams` → `reinit_laser`(레이저 모델 재생성; 다음 스캔에서 likelihood field `map_update_cspace` 재계산), α1–5 → `reinit_odom`, **`update_min_a`/`update_min_d`/`resample_interval`은 재초기화 없이 즉시 반영**. 갱신 조건은 `|Δ| > update_min_*`(엄격 부등호, amcl_node.cpp). robot_localization `smooth_lagged_data`, `history_length`, `predict_to_current_time`, `*_rejection_threshold`(n_σ, `n_σ²`과 비교), `set_pose` — **`set_pose`는 서비스·토픽 모두 노드 상대 이름이라 같은 네임스페이스의 두 EKF 인스턴스가 같은 이름을 광고**(ros_filter.cpp) → 런치에서 인스턴스별 리맵 필요(§7). gz-sim6 `UserCommands`: **`/world/<w>/set_pose`(ignition.msgs.Pose → Boolean)** 텔레포트 서비스 존재(VERIFIED). gz-sensors `GpuLidarSensor`는 **visual 지오메트리**에 대해 거리 측정(VERIFIED).

---

## 2. 기준선 알고리즘의 수학적 유도

### 2.1 차동 구동 오도메트리와 노이즈 전파

$$\Delta s_{L,R}=\frac{2\pi r}{N}\Delta n_{L,R},\quad \Delta s=\frac{\Delta s_R+\Delta s_L}{2},\quad \Delta\theta=\frac{\Delta s_R-\Delta s_L}{b},\quad v_k=\frac{\Delta s}{\Delta t},\ \omega_k=\frac{\Delta\theta}{\Delta t},\ \Delta t=0.02\ \mathrm s$$

- 1틱 = 1.27×10⁻⁴ m → 양자화 σ = 3.7×10⁻⁵ m, σ_v,q ≈ 1.3×10⁻³ m/s (무시 가능).
- 슬립 잡음 n_R ~ N(0, k_s²Δs_R²), n_L ~ N(0, k_s²Δs_L²), k_s = 0.01, 독립. 선형 사상 J = [[1/(2Δt), 1/(2Δt)], [1/(bΔt), −1/(bΔt)]]로
  $$\Sigma_{enc}=J\,\mathrm{diag}(k_s^2\Delta s_R^2,\,k_s^2\Delta s_L^2)\,J^\top:\quad
  \sigma_v^2=\frac{k_s^2(\Delta s_R^2+\Delta s_L^2)}{4\Delta t^2},\ \ \sigma_{\omega,enc}^2=\frac{k_s^2(\Delta s_R^2+\Delta s_L^2)}{b^2\Delta t^2},\ \ \mathrm{cov}(v,\omega)=\frac{k_s^2(\Delta s_R^2-\Delta s_L^2)}{2b\Delta t^2}$$
  직진(Δs_R = Δs_L): σ_v = k_s|v|/√2 = 0.014 m/s, σ_ω,enc = √2 k_s|v|/b = 0.079 rad/s (v = 2 m/s), cov = 0. **회전 중에는 cov ≠ 0**(§4.A에서 2×2 형태 사용, 개정).
- 자이로 백색 σ_g = 2×10⁻⁴ rad/s는 σ_ω,enc의 1/400. 그러나 **보정 전 자이로 바이어스 0.01 rad/s는 60 s에 0.6 rad(34°)** 헤딩 오차 → 자이로 우선 결론은 **바이어스 보정/추정을 전제**로만 성립(개정). 보정 후 60 s 헤딩 σ ≈ √((6.3×10⁻⁶·60)² + (σ_g√(0.01·60))²) = √((3.8×10⁻⁴)² + (1.5×10⁻⁴)²) ≈ 4×10⁻⁴ rad → 20 m(60 s) 주행 측면 오차 ≈ 0.4 cm(바이어스항 b·v·T²/2 = 0.38 cm ⊕ 랜덤워크 D·σ_θ/√3 = 0.18 cm; 상한 D·θ_end = 0.8 cm). 인코더 ω만 적분하면 2 m/s·10 s에 σ_θ ≈ 0.079·√(10·0.02) = 0.035 rad(2°) → 수십 cm.
- **공칭 바닥 드리프트 예측**(명세 2장 실험 대상, S9에서 측정): 100 m 직진, v = 1 m/s. 순수 인코더: 헤딩 랜덤워크 σ_θ = √2 k_s √(vΔtD)/b = 0.056 rad(3.2°) → 측면 ≈ D·σ_θ/√3 ≈ 3 m(3 %); 거리 스케일 σ_D = k_s√(Δs·D/2) = 0.010 m(0.010 %; 스텝당 σ = k_sΔs/√2, 감사 수정 — 원문 0.014는 √2 과대). 자이로 융합 후(T = 100 s): 측면 = 바이어스 잔차 b·v·T²/2 ⊕ 백색 D·σ_g√(Δt_imu T)/√3 = 3.2 cm ⊕ 1.2 cm, 스케일 1.0 cm → **≈ 3.5 cm/100 m(0.035 %, T_cal 10 s) / 2.0 cm(0.02 %, T_cal 60 s)** (checks/audit_numbers.py). 적재 25 kg는 슬립 크기(k_s 유효값)로 나타나므로 하중별로 재측정.

### 2.2 EKF (팀 직접 구현, 상태 x = [x, y, θ, v, ω, b_g]ᵀ — 6-상태, 개정)

**예측** (등속 모델, 중점 적분; 단위 x,y [m], θ [rad], v [m/s], ω [rad/s], b_g [rad/s]):

$$\theta_m=\theta+\tfrac12\omega\Delta t,\ \ x^-=x+v\Delta t\cos\theta_m,\ \ y^-=y+v\Delta t\sin\theta_m,\ \ \theta^-=\theta+\omega\Delta t,\ \ v^-=v,\ \ \omega^-=\omega,\ \ b_g^-=b_g$$

$$F=\begin{bmatrix}
1&0&-v\Delta t\sin\theta_m&\Delta t\cos\theta_m&-\tfrac12 v\Delta t^2\sin\theta_m&0\\
0&1&\ \ v\Delta t\cos\theta_m&\Delta t\sin\theta_m&\ \ \tfrac12 v\Delta t^2\cos\theta_m&0\\
0&0&1&0&\Delta t&0\\ 0&0&0&1&0&0\\ 0&0&0&0&1&0\\ 0&0&0&0&0&1\end{bmatrix},\qquad P^-=FPF^\top+Q_k$$

**공정 잡음 — 연속 백색가속도(CWNA) 모델(개정)**: 연속 잡음 강도 q_v [m²/s³], q_ω [rad²/s³], q_b [rad²/s³]. c = cos θ_m, s = sin θ_m:

$$Q_k=\begin{bmatrix} q_v\tfrac{\Delta t^3}{3}c^2 & q_v\tfrac{\Delta t^3}{3}cs & 0 & q_v\tfrac{\Delta t^2}{2}c & 0 & 0\\
q_v\tfrac{\Delta t^3}{3}cs & q_v\tfrac{\Delta t^3}{3}s^2 & 0 & q_v\tfrac{\Delta t^2}{2}s & 0 & 0\\
0&0& q_\omega\tfrac{\Delta t^3}{3} &0& q_\omega\tfrac{\Delta t^2}{2} &0\\
q_v\tfrac{\Delta t^2}{2}c & q_v\tfrac{\Delta t^2}{2}s &0& q_v\Delta t &0&0\\
0&0& q_\omega\tfrac{\Delta t^2}{2} &0& q_\omega\Delta t &0\\ 0&0&0&0&0& q_b\Delta t\end{bmatrix}$$

값: **q_v = 0.005 m²/s³, q_ω = 0.02 rad²/s³**(robot_localization의 `P += Δt·Q_param`과 v, ω 대각에서 정확히 같은 모델; 현재 ekf.yaml vyaw 0.02와 일치, vx 0.025는 5배 보수적), q_b = 10⁻¹⁰ rad²/s³(시뮬 바이어스는 정적이라 수치 안정용 최소값; 실기에서는 Allan variance의 bias-instability로 설정). **율 의존성 명시(개정)**: 원본의 이산 백색가속도(DWNA) σ_a = 0.5 m/s²는 σ_a²Δt = q_v가 **Δt = 0.02 s에서만** 성립한다. robot_localization은 측정 도착마다(IMU 100 Hz + odom 50 Hz, 가변 Δt) 예측하므로 팀 EKF도 **동일한 이벤트 스케줄로 예측**하고 CWNA를 쓴다 → **v, ω 대각은 모든 Δt에서 등가**. x, y, θ 블록은 등가가 **아니다**(감사 수정): r_l은 위치에 직접 랜덤워크 `P += Δt·Q_param`(ekf.cpp 확인)을 더하므로 Q_param,xx = 10⁻³ m²/s는 AMCL 갱신 간격 0.1 s마다 σ 1 cm, 1 s에 3.2 cm의 위치 확산을 추가한다 — AMCL 공분산 하한(2 cm)과 같은 차수라 무시할 수 없다(CWNA 자체 기여는 0.1 s에 1.3 mm). 따라서 팀 EKF에 **선택 항 Q_pos = diag(q_p, q_p, q_θ)·Δt**(기본 0, A/B 모드에서 r_l과 같은 q_p = 10⁻³, q_θ = 10⁻⁴)를 두어 "모델 차이"와 "IAG 효과"를 분리하고, ekf_map의 q_p는 {10⁻³, 10⁻², 0.05(repo 값)}를 S1/S2에서 스윕한다(repo ekf.yaml은 0.05로 AMCL 추종 3 s를 실측해 둠).

**관측 모델**:

| 센서 | z | H | R (근거) |
| --- | --- | --- | --- |
| 휠 오도메트리 50 Hz | [v, ω]ᵀ | 행 4,5 | Σ_enc(§2.1, 2×2, 교차항 포함) + 양자화 — **§4.A에서 적응**. 하한 σ_v,min 0.01 m/s, σ_ω,min 0.01 rad/s |
| IMU 자이로 100 Hz (LPF 후) | ω_g = ω + b_g + n_g | [0 0 0 0 1 1] | **R = σ_g² = 4×10⁻⁸ rad²/s²**(개정; 바이어스는 상태). 입력은 계약 토픽 `imu/data`(= `imu_filter_node`가 b̂_g를 뺀 값)이므로 팀 EKF의 b_g는 **잔차 바이어스**(초기 0, P_bb = SE²); robot_localization(바이어스 상태 없음)도 같은 `imu/data`를 R = σ_g² + SE² ≈ 4×10⁻⁸로 받는다. 절제 실험에서만 `imu/data_raw`를 받아 b_g를 정지 평균으로 초기화 |
| IMU 가속도 | a_x(`imu/data`, 보정됨) | 팀 EKF에는 **넣지 않음**(§4.A 슬립 잔차 전용; r_l 기준선은 repo대로 ax, ay 융합). 절제 실험으로 제어입력 u = a_x 버전 비교 | — |
| AMCL (map 인스턴스만) | [x, y, θ]ᵀ | [I₃ 0₃ₓ₃] | AMCL `set->cov`(파티클 집합 공분산, 소스 확인) **하한** σ_xy ≥ 0.02 m, σ_θ ≥ 0.01 rad(수렴 후 붕괴 보정) |
| ZUPT | [0, 0]ᵀ (v, ω) | 행 4,5 | diag(10⁻⁶, 10⁻⁶); Δn_L = Δn_R = 0(틱 변화 없음)이 0.2 s 지속 시. 정지 중 ω = 0이므로 **b_g가 자이로에서 직접 관측**(6번째 상태의 가관측성 근거) |

**바이어스 보정(명세 2장, 개정·감사 시 계약명 정렬)**: `imu_filter_node`가 기동 시 `bias_estimation_time`(계약 기본 60 s, 최소 10 s) 정지에서 b̂_g = mean(ω), b̂_a = mean(a) − g_ref(sensor_calibration.md §2.4 절차)를 구해 빼고 `imu/data`로 발행, 추정값은 `imu/bias`로도 발행(신규 토픽). 팀 EKF는 잔차 b_g(초기 0, P_bb = SE² = (6.3×10⁻⁶)² 또는 (2.6×10⁻⁶)²)를 ZUPT마다 재관측; 가속도 바이어스 잔차는 슬립 잔차 모델에만 들어간다(σ 5.4×10⁻⁴ / 2.2×10⁻⁴ m/s²). 스펙 예시 EKF(`imu0` yaw·ax·ay 융합, `pose0: /amcl_pose`) 및 repo ekf.yaml과의 **설계 차이 문서화**: (i) Gazebo IMU `orientation`은 GT 헤딩이라 절대 yaw 융합은 사실상 GT 사용 → 자이로 율만 융합(repo ekf.yaml과 동일; 실기에서도 자력계 없는 실내 AHRS yaw는 불안정), (ii) **a_x, a_y**: repo ekf.yaml은 융합한다(선회 구심항으로 이산화 오차 6.0 → 3.5 mm 실측, IMU가 회전축 위). `imu/data`가 이미 바이어스 보정되어 원문 반대 근거(0.10 m/s² 바이어스)는 소멸하므로 **robot_localization 기준선은 repo 설정(ax, ay 융합)을 유지**하고(감사 수정), 가속도 상태가 없는 팀 6-상태 EKF는 미사용 — 절제 실험에서 제어입력 u = a_x 버전과 비교, (iii) `pose0`는 게이팅·하한 적용 `amcl_pose_gated`(TRACKING 상태에서는 항등 통과; 계약 변경).

**업데이트** (Joseph form, 각도 innovation wrap): ν = z − Hx̂⁻, S = HP⁻Hᵀ + R, K = P⁻HᵀS⁻¹, P = (I−KH)P⁻(I−KH)ᵀ + KRKᵀ. **NIS 게이팅** ε = νᵀS⁻¹ν, p = 0.999 임계: m=1 10.8, m=2 13.8, m=3 16.3(r_l `*_rejection_threshold`는 n_σ이고 n_σ²과 비교 — 소스 확인 → repo `odom0`는 vx, vy, vyaw 3차원이므로 **twist 4.0**(감사 수정; 원문 3.7은 m=2 값), pose(x, y, yaw) 4.0).

**복잡도**: n = 6 → 스텝당 O(n³) ≈ 수백 flop, 150 Hz 이벤트 × 2 인스턴스 × 5 로봇 ≪ 1 % CPU.

### 2.3 REP-105 이중 필터 구조

이하 `ekf_odom`/`ekf_map`은 역할 약칭이며, 실제 노드는 계약대로 robot_localization `ekf_filter_node_odom`/`ekf_filter_node_map`(출력 `odometry/filtered`/`odometry/filtered_map`), 팀 EKF는 `amr_ekf_node` 두 인스턴스(출력 `odometry/amr_ekf`/`odometry/amr_ekf_map`)다.

- `ekf_odom`(world_frame = odom): 입력 `wheel_odom` [v, ω](+ ZUPT: 정지 시 같은 토픽으로 v = ω = 0, 분산 10⁻⁶), `imu/data` 자이로 ω_g. 출력 **odom→base_footprint** 50 Hz(개정), 연속.
- `ekf_map`(world_frame = map): 동일 입력 + `pose0 = amcl_pose_gated`. T_map→odom = T_map→bf · T_odom→bf⁻¹ 합성 발행(r_l 소스 확인). **AMCL `tf_broadcast: false`**.
- **TF 발행자 단일화(감사 추가)**: r_l 쌍과 팀 EKF 쌍을 동시에 띄우면 `odom→base_footprint`·`map→odom`이 이중 발행된다 → 런치 인자 `ekf_impl:={rl,amr}`로 **한 쌍만 `publish_tf: true`**, 다른 쌍은 `publish_tf: false`의 섀도 모드(토픽만 발행)로 같은 입력을 받아 A/B 로그를 남긴다(IT-01이 이중 부모 없음을 검사).
- `pose0_rejection_threshold`(4.0) 도입은 repo ekf.yaml의 "납치 후 큰 점프를 버리면 map 프레임이 복구되지 않는다"는 근거와 충돌하지 않는다: 복구 경로에서는 §4.C VERIFY가 `ekf_filter_node_map`에 `set_pose`로 필터를 재설정한 뒤 게이트를 연다(감사 추가).
- 지연: AMCL 포즈는 스캔 시각 스탬프 → `smooth_lagged_data: true`, `history_length: 1.0`, `predict_to_current_time: true`(repo ekf.yaml에 이미 있음). 팀 EKF는 1 s 상태 이력 버퍼에서 되감기-재적용.

### 2.4 AMCL (nav2_amcl 1.1.20) — 수식과 튜닝 근거

**운동 모델**(`DifferentialMotionModel`, 소스 확인; δ_rot,noise = min(|δ|, |π−δ|), 개정):

$$\hat\delta_{rot1}=\delta_{rot1}-\mathcal N\!\big(0,\ \alpha_1\delta_{rot1,noise}^2+\alpha_2\delta_{trans}^2\big),\quad
\hat\delta_{trans}=\delta_{trans}-\mathcal N\!\big(0,\ \alpha_3\delta_{trans}^2+\alpha_4(\delta_{rot1,noise}^2+\delta_{rot2,noise}^2)\big)$$

**α는 분산 계수**(개정): σ_trans = √α3·δ_trans, σ_rot = √(α1)·δ_rot 등. 단위 α1, α3 무차원, α2 [rad²/m²], α4 [m²/rad²]. 물리값(ekf_odom 입력): 인코더 스케일 σ ≈ 0.007·δ_trans → α3,phys = 5×10⁻⁵; 자이로 융합 헤딩 σ(0.1 m 갱신당) ≈ 6×10⁻⁶ rad → α2,phys ≈ 4×10⁻⁹(사실상 0). 파티클 다양성용 팽창 κ = 5–10배는 **α = κ²·α_phys ≈ 1.2×10⁻³–5×10⁻³**(원본의 0.05는 √0.05 = 22 % ≈ 32배로 과대 — 갱신마다 클라우드가 σ_hit보다 넓게 퍼져 가중치 붕괴). 일관성 검사: α2 = 2×10⁻³, 0.1 m 갱신 → σ_rot = 4.5×10⁻³ rad → 10 m 끝점 4.5 cm ≈ 0.9 σ_hit ✓. **초기값 α1–4 = 2×10⁻³**, `ekf_odom`의 갱신 간 증분 공분산(트위스트 공분산 적분)에서 α_phys를 실측한 뒤 **α ∈ {10⁻³, 2×10⁻³, 5×10⁻³, 2×10⁻², 0.2} × σ_hit ∈ {0.04, 0.05, 0.08, 0.2} 공동 스윕**(§6; 감사 시 초기점 α 2×10⁻³·σ_hit 0.05가 격자에 들어가도록 보강). Nav2 기본 0.2는 raw odom 전제.

**관측 모델**(likelihood_field, 소스 확인): d_k = 빔 끝점의 최근접 점유셀 거리(`laser_likelihood_max_dist`로 캡),
$$p_k=z_{hit}\exp\!\Big(-\frac{d_k^2}{2\sigma_{hit}^2}\Big)+\frac{z_{rand}}{z_{max\,range}},\qquad w\leftarrow 1+\sum_k p_k^3$$
(곱이 아닌 세제곱 합.) σ_hit **초기값 유도**(튜닝 변수, 개정): 맵 오차 σ_map은 ADNN 목표 0.03 m(반정규 평균)에서 σ_map = 0.03/√(2/π) = 0.038 m →
$$\sigma_{hit}\approx\sqrt{0.03^2+0.05^2/12+0.038^2}=\sqrt{2.55\times10^{-3}}\approx0.05\ \mathrm m$$
기본 0.2 대비 4배 예리. `laser_likelihood_max_dist` 1.0, `z_hit 0.8 / z_rand 0.2`, `max_beams 180`, `laser_max_range 25`. **단일 갱신 사후분포 폭(개정)**: p_k³ ∝ exp(−3d²/2σ_hit²)이므로 빔 법선 방향 폭 ≈ σ_hit/√3 ≈ 2.9 cm; 가산형 가중치라 빔 수로 √B 예리화되지 **않으며**, 시간 누적(리샘플링 + 작은 α)으로만 좁아진다.

**KLD 적응 샘플 수**(pf.c 확인): n = (k−1)/(2ε)·(1 − 2/(9(k−1)) + √(2/(9(k−1)))·z)³, `pf_z`는 z-분위값(0.99 ≈ 84 %, 2.33 = 99 %). 추적 시 k ≈ 20–50 → n ≈ 360–750, 발산 시 k ≈ 200 → 2480. 설정 **min 500 / max 5000, pf_err 0.05, pf_z 2.33**(복구 시 동적 확장, §4.C).

**Augmented MCL**(pf.c 확인): p_rand = max(0, 1 − w_fast/w_slow), **α_slow 0.001, α_fast 0.1**. 균일 재초기화의 기대 적중(개정, 3-D 포즈 공간): σ_hit = 0.05, 대표 거리 r = 5 m에서 분지(basin) ≈ 반경 3σ_hit = 0.15 m, 헤딩 ±3σ_hit/r = ±0.03 rad → 부피비 = (π·0.15²/1000)·(0.06/2π) = 6.8×10⁻⁷ → **N = 5000이면 기대 0.003개**(P ≈ 0.3 %). Nav2 기본(σ_hit 0.2, max_dist 2.0): 반경 0.6 m, ±0.12 rad → 4.3×10⁻⁵ → 5000개에 0.2개, 20 000개에 0.9개(P ≈ 58 %). augmented 주입은 갱신당 p_rand·N개를 추가하므로 회전 중 30회 갱신이면 누적 표본이 수십만 개가 되어 P가 오르지만, 대칭 랙 통로에서는 오수렴 후 w_fast가 회복되어 주입이 멈춘다 → **§4.C의 가설 시드가 1차, 균일 재초기화는 2차 경로**.

**리샘플링(개정 — 소스 재확인)**: nav2 humble `pf_update_resample`는 누적표를 **선형 탐색**하는 "Naive discrete event sampler"(O(N²), 평균 N²/2 비교), low-variance 샘플러는 "KLD와 결합 곤란" 주석과 함께 비활성. 비용: N = 500 → 1.25×10⁵ 비교(≈0.1 ms), N = 5000 → 1.25×10⁷(≈10–25 ms), N = 20 000 → 2×10⁸(≈0.2–0.4 s), N = 100 000 → 5×10⁹(≈ 5 s, **불가**). 따라서 복구 시 max_particles 상한을 **20 000**으로 잡고 `resample_interval 2`로 리샘플 빈도를 반감한다(P ≥ 0.9에 필요한 N ≈ 53 000은 리샘플당 ≈ 1.4–2.8 s라 채택 불가). `update_min_d 0.10 m`, `update_min_a 0.10 rad`(추적); 갱신 조건이 엄격 부등호라 1.0 rad/s 회전 × 10 Hz 스캔(스캔당 정확히 0.1 rad)에서는 갱신이 누락될 수 있으므로 **ROTATE 동안 `update_min_a`를 0.05로 동적 변경**(재초기화 없음, 감사 추가).

**정지 처리**: 움직임이 없으면 갱신 생략·마지막 포즈 재발행(소스 확인). 정지 직후 `request_nomotion_update` 1 Hz × 3회(연속 호출은 파티클 고갈 유발 — 4시간 운용 §6 S8에서 확인).

**복잡도(개정)**: 갱신당 O(N·B + N²/2). 추적(N ≈ 500): 9×10⁴ 룩업 + 1.25×10⁵ 비교 ≈ 0.5 ms → 5 로봇 × 10 Hz ≈ 2.5 % 코어. 복구(N = 20 000, 한 로봇): 3.6×10⁶ 룩업(≈ 0.07 s) + 2×10⁸ 비교(0.2–0.4 s) ≈ 0.3–0.5 s/리샘플 갱신. AMCL 콜백은 단일 스레드라 6.3 s 회전 동안 **1 코어가 포화되고 실제 처리 갱신은 ≈ 20–35회**(나머지 스캔은 드롭되며, 운동 모델은 마지막 갱신 이후 odom 증분을 쓰므로 정합성은 유지); KLD가 수렴에 따라 N을 줄이므로 이는 상한이다. 버스트 = **1 코어 × ≈ 6–10 s**(감사 시 §4.C·§7과 수치 통일; 원문의 "15회 ≈ 5 s"와 "30회 = 10 s"는 서로 모순).

### 2.5 slam_toolbox (2.6.10, online_async) — 파라미터 의미와 튜닝

Karto 스캔 매처는 odom 예측 포즈 주변 **상관 탐색 격자**(±`correlation_search_space_dimension`/2 = ±0.25 m, 0.01 m)와 각도(coarse ±0.349 rad/0.0349 rad → fine ±0.00349 rad)에서 응답(response = 스캔 점 중 smear된 점유셀에 떨어진 비율)을 최대화하고 `distance/angle_variance_penalty`로 odom에서 멀어질수록 곱셈 벌점을 준다. 매처는 응답 곡면의 2차 모멘트로 **공분산 Σ_ij**도 산출한다.

**루프 폐합 탐지·검증**(개정, 명세 "이해·설명" 요구): 현재 체인 밖 노드 중 `loop_search_maximum_distance` 이내, 체인 길이 ≥ `loop_match_minimum_chain_size`인 후보 체인에 대해 `loop_search_space_dimension`(8 m, 0.05 m) 조대 매칭 → **수락 조건**(Mapper.cpp `TryCloseLoop`, 감사 시 확인) response_coarse > `loop_match_minimum_response_coarse`(0.35) ∧ Σ_xx, Σ_yy < `loop_match_maximum_variance_coarse`(3.0) → 정밀 매칭 response_fine ≥ `loop_match_minimum_response_fine`(0.45) 이면 링크 추가; 정밀 단계 실패 시 기각(로그 "REJECTED!"). 사후 χ² 기각은 없으므로 오정합 방어는 임계 + 강인 손실함수뿐이다.

**그래프 최적화(기본 `solver_plugins::CeresSolver`; 목적함수는 Konolige 2010 SPA와 같은 꼴)**: 노드 포즈 (p_i, θ_i), 간선 측정 (p̂_ij, θ̂_ij), 잔차(slam_toolbox `PoseGraph2dErrorTerm`, 감사 시 소스 확인 — SE(2) log가 아니라 i-프레임 상대 위치 차 + 정규화 각도 차)
$$e_{ij}=\begin{bmatrix}R(\theta_i)^\top(p_j-p_i)-\hat p_{ij}\\ \mathrm{wrap}(\theta_j-\theta_i-\hat\theta_{ij})\end{bmatrix}\in\mathbb R^3,\qquad \Omega_{ij}=\Sigma_{ij}^{-1}\ (\text{Ceres에는 }\Omega^{1/2}e\text{로 전달})$$
$$\min_{\{p_i,\theta_i\}}\ \sum_{(i,j)\in\mathcal E}\rho_H\!\big(e_{ij}^\top\Omega_{ij}e_{ij}\big),\qquad
\rho_H(s)=\begin{cases}s,& s\le a^2\\ 2a\sqrt s-a^2,& s>a^2\end{cases}$$
`ceres_loss_function: HuberLoss`는 **a = 0.7로 하드코딩**(ceres_solver.cpp, 2.6.10 태그 확인) → 마할라노비스 0.7σ 밖은 선형 벌점으로 잘못 수락된 루프 링크의 영향을 유계화(파라미터로 a를 바꿀 수 없음). LM(`ceres_trust_strategy`)으로 희소 정규방정식(`SPARSE_NORMAL_CHOLESKY`, O(|E|) 비영 블록) 풀이. 명세 힌트("지도가 찌그러지면 chain_size, correlation_search_space 조정") 해석: 찌그러짐 = 오수락 루프(체인·응답 임계 ↑) 또는 매칭 실패(탐색 공간·smear ↑).

| 파라미터 | 기본 | 제안 | 근거 |
| --- | --- | --- | --- |
| resolution / max_laser_range | 0.05 / 20 | 0.05 / 25 | 명세 ≤ 0.05 m, LiDAR 25 m |
| minimum_travel_distance / heading | 0.5 m / 0.5 rad | 0.3 / 0.3 | 그래프 밀도·루프 후보 ↑ |
| correlation_search_space_dimension | 0.5 m | 0.5 | 0.3 m 이동 간 odom 오차 ≪ 0.25 m |
| correlation_search_space_smear_deviation | 0.1 | 0.05 | ≈ σ_lidar + res; 과대 시 벽 두꺼워짐 |
| loop_match_minimum_chain_size | 10 | 15 | 랙 통로 반복 구조 aliasing 억제 |
| loop_match_minimum_response_fine | 0.45 | 0.55 | 동일 |
| loop_search_maximum_distance | 3.0 m | 5.0 | 100 m 루프·0.05–1 % 드리프트 ≪ 5 m |
| ceres_loss_function | None | HuberLoss | 위 식 |
| mode | mapping | mapping → 저장 후 map_server + AMCL | 명세 |

### 2.6 맵 품질 지표 정의

**GT 점유격자 M_gt(개정)**: 월드 SDF의 정적 모델을 **visual 지오메트리**로(gpu_lidar가 visual에 대해 측정 — VERIFIED) **지면 +0.38 m 스캔 평면**에서 0.05 m 래스터화; 파이프라인은 각 정적 모델의 visual ≡ collision 여부를 검사해 불일치 목록을 리포트에 기록. M_est는 점유셀 2D ICP(SE(2))로 M_gt에 정렬 후:

- **허용오차 IoU(개정, 대칭 팽창)**: TP = |O_est ∩ dil_τ(O_gt)|, FP = |O_est \ dil_τ(O_gt)|, FN = |O_gt \ dil_τ(O_est)|, IoU_τ = TP/(TP+FP+FN), precision = TP/|O_est|, recall = 1 − FN/|O_gt| (τ = 1셀).
- **ADNN**(Santos 2013) = (1/|O_est|) Σ_{p∈O_est} min_{q∈O_gt}‖p−q‖ (GT 거리변환으로 O(N)); Chamfer(대칭)도 보고. 목표 **ADNN ≤ 0.03 m(하드) / 0.02 m(목표)** — 정지 3 cm 예산에서 맵 편향이 지배항(§5).
- **일관성(GT 불필요)**: (i) 포즈그래프 잔차 Σ e_ijᵀΩ_ij e_ij / (3(|E| − |V| + 1))(첫 노드 고정으로 자유 파라미터 3(|V|−1); 재개 감사 정정 — 원문 3|E| − 3|V|는 게이지 3을 빠뜨림), (ii) 벽 두께(경계 길이당 점유셀), (iii) Filatov 2017의 세 지표 — **점유셀 비율, 모서리 수, 폐영역 수**(PDF 확인, 개정) — 를 그대로 계산, (iv) **팀 자체 정의**: 랙 모서리 직각 편차(Harris 모서리 근방 두 벽 방향의 각도 − 90°의 RMS; Filatov의 지표가 아님), (v) 반복 매핑 IoU_τ(두 런 간).
- **궤적**: SLAM 포즈 ATE/RPE(Sturm 2012).

---

## 3. 문헌 조사 (2023-09 → 2026-09 + 가장 가까운 선행)

| # | 문헌 | 관련성 | 상태 |
| --- | --- | --- | --- |
| 1 | Zhang et al., "Tackling the Kidnapped Robot Problem via Sparse Feasible Hypothesis Sampling and Reliable Batched Multi-Stage Inference," IEEE T-IM (accepted), arXiv 2511.01219 | 단일 스캔 수동 재정위: RRT 희소 가설, SMAD 빔 오차 지표, TAM 정렬 지표. §4.B/C 지표 선행 | VERIFIED |
| 2 | Bilevich, Buber, Halperin, "Lifelong Localization in Dynamic Indoor Environments Combining Odometry with Sparse Distance Sampling," ICRA 2026, arXiv 2607.17852 | odom + 16개 거리 샘플로 납치 해결, 수렴 보장 | VERIFIED |
| 3 | Filotheou, "CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor," IROS 2024, arXiv 2307.14247 | **CAER**(ray별 누적 절대오차)로 대량 가설 평가 후 scan-to-map-scan 매칭 정련. §4.C 직접 선행 | VERIFIED |
| 4 | Xu et al., "Selective Kalman Filter," arXiv 2412.17235 | 퇴화 차원만 보조 센서 갱신 | VERIFIED |
| 5 | Zhao et al., "SuperLoc," ICRA 2025, arXiv 2412.02901 | 최적화 전 정합 위험 예측(54 %↑) | VERIFIED |
| 6 | Dolatabadi et al., arXiv 2501.02558 | 맵 매칭 공분산 학습 예측(2 cm↑) — 우리는 비학습 하한/게이팅 | VERIFIED |
| 7 | Dolatabadi et al., arXiv 2509.18954 | ICP 공분산 예측 | VERIFIED |
| 8 | Diker, Klein, "Neural Aided Adaptive Innovation-Based Invariant KF," arXiv 2603.26709 | innovation 기반 Q 적응(Lie 군) | VERIFIED |
| 9 | Khosravi et al., IAS-19, arXiv 2603.09783 | innovation/residual 통계로 Q,R 조정 | VERIFIED |
| 10 | Chen et al., "KF Auto-tuning ... Chi-Squared ... Bayesian Optimization," arXiv 2306.07225 | NIS/NEES χ² 정합 자동 튜닝 → §6 오프라인 절차 | VERIFIED |
| 11 | Mozzarelli et al., ECC 2024, arXiv 2403.13452 | 모듈형 오도메트리 개선(90 %↓) | VERIFIED |
| 12 | Yu et al., "Fully Proprioceptive Slip-Velocity-Aware State Estimation," IROS 2023, arXiv 2209.15140 | IMU+본체속도 슬립 추정(RIEKF + 외란관측기), **χ² 슬립 검정 포함** | VERIFIED |
| 13 | Chauchat, Bonnabel, Barrau, arXiv 2409.07050 | 휠 반경 미지 불변 필터 | VERIFIED |
| 14 | Alwala et al., arXiv 2603.14940 | 차동구동 EKF 강인성 | VERIFIED |
| 15 | Kuang et al., ICRA 2025, arXiv 2503.23480 | 신경 맵 MCL | VERIFIED |
| 16 | Gao et al., ERPoT, arXiv 2409.14723 | 폴리곤 맵 정합 | VERIFIED |
| 17 | Zhang et al., 2DLIW-SLAM, MST 2024, arXiv 2404.07644 | 2D LiDAR-IMU-휠 SLAM | VERIFIED |
| 18 | Davies et al., arXiv 2504.19654 | 점유격자 정제 | VERIFIED |
| 19 | Seghiri, Mansouri, Chemori, TIMC 2025 | AMCL용 결정적 ESR 리샘플링(대칭 환경) | VERIFIED(초록) |
| 20 | Ince, Yiltas-Kaplan, Keleş, "From Simulation to Reality: Comparative Performance Analysis of SLAM Toolbox and Cartographer in ROS 2," Electronics 14(24):4822, 2025 | Humble·Gazebo·실기: slam_toolbox ATE 0.13 m vs Cartographer 0.21 m, 동적 요소에 더 일관. **단, 저자들은 slam_toolbox의 CPU ≈ 70 %, 293 MB, 시작 5.2 s를 "더 높은 연산 요구와 설정 복잡도"로 평가**(Cartographer는 CPU 80 %·튜닝 민감). 선택 근거는 정확도·일관성이지 CPU가 아님(개정). 초록 자체가 CPU 70 % < 80 %인데 Toolbox를 "더 높은 연산 요구"로 기술하는 **내부 불일치**가 있어(재개 감사 시 원문 초록 재조회) CPU는 어느 쪽 근거로도 쓰지 않고 S7에서 직접 측정 | VERIFIED(초록; 감사 시 Semantic Scholar API로 재조회 — Cartographer 80 %·299 MB·튜닝 민감 문구 확인) |
| 21 | "Beyond Odometry Accuracy: ... Encoder Resolution and IMU Fusion ... Particle Filter Localization," Robotics 15(9):164, 2026 | IMU 융합이 odom은 개선하나 AMCL 안정성은 저하 가능 → α 재튜닝 필수 | VERIFIED(초록) |
| 22 | Zhu et al., TempLoc, arXiv 2602.03198 | 시간 일관성 재정위 | VERIFIED |
| 23 | GitHub rouf-rimon/adaptive-ekf-slip-detector | 인코더·IMU 잔차 → RF 분류기 → odom R 팽창(1+500λ). §4.A와 가장 가까운 공개 구현(학습 기반) | VERIFIED |
| 24 | Kharwar et al., "FusionCore: A 23-State UKF for IMU, Wheel Encoder, GPS, and Visual SLAM Fusion in ROS 2," arXiv 2605.25239 (2026-05) | **센서별 마할라노비스 χ² 게이팅 + innovation 기반 R 적응 + 자이로/가속도 바이어스 상태**, NCLT에서 robot_localization 대비 우위. §4.A의 "게이팅+적응" 조합은 이미 존재 → §4.A는 변형 | VERIFIED |
| 25 | Akai, "Reliable Monte Carlo Localization for Mobile Robots," arXiv 2205.04769; J. Field Robotics 40(3):595–613, 2023, doi 10.1002/rob.22149(감사 시 Crossref로 서지 확인) | MCL 안에서 **신뢰도(실패) 추정** + importance sampling으로 전역 정위 통합, 빠른 재정위. §4.B/C 선행 | VERIFIED(arXiv) |
| 26 | Bukhori, Ismail, "Detection of kidnapped robot problem in MCL based on the natural displacement of the robot," IJARS 14(4), 2017 | 센서 판독으로 "자연스러운 변위" 여부 판정, 파티클 가중치 기반 검출보다 우수. §4.B 선행(가중치 기반의 한계 근거) | VERIFIED(초록) |
| 27 | Campbell, Whitty, "Metric-based detection of robot kidnapping with an SVM classifier," RAS 69:40–51, 2015 (online 2014-08), doi 10.1016/j.robot.2014.08.004 | 독립적인 두 포즈 추정 간 불일치를 포함한 지표들을 선형/SVM 분류기로 결합해 실시간 납치 검출(3D 점군 설정). §4.B의 "정합 지표 + 필터 불일치" 병용의 선행(학습 기반) | 서지 VERIFIED(Crossref), 내용은 검색 초록 + ECMR 2013 선행판 초록(ANU 저장소) 기준 |
| 28 | Reina, Ojeda, Milella, Borenstein, "Wheel slippage and sinkage detection for planetary rovers," IEEE/ASME T-Mech 11(2), 2006 | 인코더 vs 자이로/가속도계/모터전류 불일치로 슬립 검출 — §4.A 물리 잔차의 원형 | 서지 VERIFIED, 내용 RECALLED |
| 29 | Fox, Burgard, Thrun, "Active Markov localization for mobile robots," RAS 25:195–207, 1998 | 능동 정위(정보 이득 기반 행동 선택) — §4.C 회전·탐색 이동의 원형 | 서지 VERIFIED, 내용 RECALLED |
| 30 | Zhang, Zapata, Lépinay, "Self-adaptive Monte Carlo localization for mobile robots using range finders," Robotica 30(2):229–244, 2012 (online 2011), doi 10.1017/S0263574711000567 | SAMCL: 사전계산한 유사 에너지 영역(SER)에서 표본을 뽑아 전역 정위·납치 복구 — §4.B/C의 "검출 + 표적 재시드" 선행(리뷰 판정에서 언급) | 서지 VERIFIED(Crossref), 내용 RECALLED |

고전(RECALLED): Thrun·Burgard·Fox 2005; Fox IJRR 2003(KLD); Bar-Shalom·Li·Kirubarajan 2001(NIS/NEES, CWNA); Mehra IEEE TAC 1970; Mohamed & Schwarz J. Geodesy 1999(IAE); Page Biometrika 1954(CUSUM); Lorden 1971(CUSUM 최적성); Moore & Stouch 2014(robot_localization); Macenski & Jambrecic JOSS 2021; Konolige et al. IROS 2010(SPA); Santos et al. SSRR 2013(ADNN); Filatov et al. 2017(arXiv 1708.02354, 지표 3종은 PDF 본문 확인 → VERIFIED); Sturm et al. IROS 2012; Lenser & Veloso ICRA 2000; REP-105.

**종합·위치 짓기**: (i) 납치/재정위는 단일 스캔 정합 지표(CAER, SMAD, TAM)로 대량 가설을 평가하고 정련하는 흐름, (ii) 융합은 innovation 적응 + χ² 게이팅이 ROS 2 패키지(FusionCore)로까지 정리됨, (iii) 실패 검출은 MCL 내부 신뢰도 추정(Akai)·지표 분류(Campbell)가 선행. 따라서 아래 제안 중 **개별 요소는 모두 선행이 있고**, 우리가 주장하는 것은 (a) 차동구동·시뮬 잡음 모델에 대한 폐형 임계/공분산 유도, (b) nav2_amcl 동적 파라미터·서비스만으로 구현되는 복구 절차, (c) robot_localization과 동일 입력을 쓰는 A/B 실험 설계다.

---

## 4. 독자 알고리즘 제안

### 4.A IAG-EKF: 물리 잔차 슬립 검정 + 잔차 기반 유계 적응 (개정)

**동기**: 슬립 순간 인코더 [v, ω]는 편향 관측이 되어 상수 R로는 상태에 흡수된다. 자이로·가속도계는 슬립에 무관하므로 **필터 밖의 물리 잔차**로 검정한다(Reina 2006 원리). 검출 가능한 것은 **슬립 과도(onset/offset)** 와 좌우 비대칭 슬립이며, 정속 대칭 슬립(v_enc ≠ v_body가 일정)은 고유수용 센서만으로는 관측 불가 — 이는 map 프레임에서 AMCL이 보정하고 IAG는 odom 프레임 손상을 과도 구간에서 제한한다(정직한 범위 한정).

**잔차(창 T_w = 0.2 s = 10 인코더 샘플 · 20 IMU 샘플)**:
$$r_\omega=\overline{\omega_{enc}}^{\,w}-\overline{(\omega_g-\hat b_g)}^{\,w},\qquad
r_v=\big[v_{enc}(t)-v_{enc}(t-T_w)\big]-\sum_{i\in w}(a_{x,i}-\hat b_a)\Delta t_{imu}$$
잡음(무슬립 가설): r = A n + m, n = 인코더 샘플 t−10…t의 22개 휠 잡음(r_ω는 t−9…t의 10샘플 평균, r_v는 v(t)와 v(t−10); 분산 k_s²Δs², §2.1), A는 평균/차분의 선형 사상, m = IMU 항(`imu/data`는 이미 b̂를 뺀 값이므로 식의 b̂ 항은 잔차 바이어스 모델링용).
$$\Sigma_r=A\,\mathrm{diag}(k_s^2\Delta s_{R,i}^2,k_s^2\Delta s_{L,i}^2)_{i\in w}A^\top+\mathrm{diag}\!\Big(\tfrac{\sigma_g^2}{20},\ 20\,\sigma_{acc}^2\Delta t_{imu}^2+(\sigma_{b_a}T_w)^2\Big),\qquad s_k=r^\top\Sigma_r^{-1}r\ \sim\ \chi^2_2$$
Σ_r은 인코더 v·ω 잡음의 **교차항**(회전 시 Δs_R ≠ Δs_L)을 A를 통해 자동으로 포함한다(개정). 수치: v = 2 m/s 직진에서 σ_rv = √(2·0.0141² + (0.017·0.01·√20)² + (5.4×10⁻⁴·0.2)²) = **0.020 m/s**(가속도 단위 0.10 m/s²; 원본의 1스텝 유한차분 1.0 m/s²에서 10배 개선), 저속 하한(σ_v,min 0.01)에서 0.014 m/s; σ_rω = √(0.079²/10 + (2×10⁻⁴)²/20) ≈ 0.025 rad/s. **검출 한계**(감사 수정 — 게이트 13.8은 단일 성분 기준 √13.8 = 3.7σ; 원문 "3σ … 0.025 rad/s"는 1σ 값): v = 2 m/s 직진에서 0.2 s 동안 휠-본체 속도 변화 차 |r_v| ≥ 0.074 m/s 또는 좌우 비대칭 |r_ω| ≥ 0.092 rad/s; v = 1, ω = 1.5에서 0.039 m/s / 0.048 rad/s(σ가 속도에 비례, 하한 σ_min이 적용되면 그만큼 보수적). Monte Carlo(checks/iag_residual_mc.py, 무슬립, 선회 포함): s_k 평균 2.00, P(s_k > 13.8) = 0.0009–0.0012로 χ²₂ 가정 성립; 교차상관은 ≤ 0.18로 작다. 바이어스는 §2.2 보정값을 사용(잔차 편향 제거, 개정). **시각 정렬 전제(재개 감사 재유도, checks/audit_resume.py)**: 인코더–IMU 시각 어긋남 δ의 편향은 창 차분 구조 때문에 r_v에서 δ·[a(t) − a(t−T_w)](**등가속에서는 0**; jerk 2 m/s³ 램프 Δa ≤ 0.4 m/s²면 δ = 5 ms에서 2 mm/s = 0.14σ — 원문 "1 m/s² 가속 중 5 ms → 0.005 m/s"는 계단 변화에만 해당), r_ω에서 δ·ω̇이며 **r_ω가 지배**한다: 제자리 회전 램프(ω̇ = 2 rad/s²)에서 σ_rω = 0.0032–0.0034 rad/s(하한 지배)이므로 편향 ≤ 1σ에는 |δ| ≤ 1.6 ms가 필요하다(δ = 2 ms → 1.2–1.3σ, 램프 중 게이트 오발 1.1–1.3 %; 5 ms → 3σ, 28–34 %). 특히 `imu_filter_node`의 LPF(2차 Butterworth 20 Hz @ 100 Hz, §7)는 저주파 군지연 **τ_LPF = 9.7 ms**를 만들며, 보상하지 않으면 r_ω 편향 5.8–6.2σ로 **제자리 회전 램프마다 오게이트(P ≈ 0.99)** 된다. 따라서 (i) `imu_filter_node`가 출력 스탬프를 τ_LPF(필터 계수에서 계산)만큼 앞당겨 발행하고(0–5 Hz에서 군지연 변동 ≤ 0.7 ms → 잔여 δ ≤ 0.7 ms; 두 EKF도 같은 이득), (ii) ω_enc,i·v_enc,i는 구간 [t_{i−1}, t_i]의 평균이므로 IMU도 **같은 구간에서 적분**해 짝지으며(끝 시각 스탬프끼리 점 표본으로 짝지으면 구간 중심 차이로 5–10 ms 어긋남), (iii) gtest/IT에 "무슬립 회전 램프 중 게이트 오발 ≤ 0.2 %" 검사를 둔다.

**적응 규칙**:
1. **게이팅**: s_k > χ²_{2,0.999} = 13.8 → 창의 인코더 갱신을 기각하고 **모델은 그대로 두되 CWNA 강도를 q_v,gated = a_max²·T_w = 1.0²·0.2 = 0.2 m²/s³로 팽창**(Q_vv = q_v,gated·Δt; 창 T_w 동안 σ_v² → (a_max T_w)² = (0.2 m/s)², v는 예측만, ω는 자이로로 계속 관측). a_x 제어입력 주입은 §2.2 모델과 상충하므로 기본 설계에서 제외하고 절제 실험(u = a_x − b̂_a, Q에 σ_acc² 추가)으로만 비교(개정). **판정 시점(감사 추가)**: 검정은 창 끝에서 내려지므로 최대 T_w = 0.2 s 늦다. 팀 EKF는 1 s 이력 버퍼로 창 내 인코더 갱신을 되감아 제거·재적용하고, robot_localization에는 되감기가 없으므로 `wheel_odometry_node`가 게이트 이후 샘플을 twist 분산 10³(사실상 무시)로 발행한다 — 두 필터는 **같은 메시지**를 받지만 게이트 반응은 다르며 이 차이는 A/B 결과 해석에 명시한다.
2. **연속 적응(잔차 기반, 필터 무관 — 개정, 감사 시 추정식 수정)**: 게이트용 0.2 s 창 잔차는 연속 표본이 10샘플 중 9개를 공유해 강하게 자기상관하므로 분산 추정에 쓰면 안 된다. 대신 **표본별(창 없는) 잔차** e_ω,i = ω_enc,i − ½(ω_g,2i + ω_g,2i+1), e_v,i = (v_enc,i − v_enc,i−1) − (a_x,2i + a_x,2i+1)Δt_imu의 최근 W = 50(1 s) 표본분산 Ĉ에서
   $$\hat R_{\omega\omega}=\mathrm{clip}\big(\hat C_{e_\omega}-\sigma_g^2/2,\ R_0,\ 100R_0\big),\qquad \hat R_{vv}=\mathrm{clip}\big(\tfrac12(\hat C_{e_v}-2\sigma_{acc}^2\Delta t_{imu}^2),\ R_0,\ 100R_0\big)$$
   (e_v는 인접 두 표본의 차라 분산이 2R_vv), R_0 = max(Σ_enc 모델 대각(§2.1), σ_min²). 원문 식 R̂_ωω = Ĉ_{r_ω} − σ_g²/20(창 잔차)은 창 평균의 1/10 축소를 되돌리지 않아 **R을 12배 과소추정**하고, ×10을 보정해도 중첩 창 때문에 편향(0.83배)·상대오차 43 %였다 — 표본별 잔차는 편향 없이 상대오차 ω 20 %, v 25 %(checks/iag_residual_mc.py; iid 이론 √(2/49) = 20 %). P⁻·innovation을 쓰지 않으므로 **휠 오도메트리 노드에서 계산해 `wheel_odom`의 `twist.covariance`로 발행 → robot_localization과 팀 EKF가 동일 입력**. 트레이드오프: W = 25면 상대오차 ≈ 29 %(지연 0.5 s), W = 50은 20 %(지연 1 s); 클립 [R_0, 100R_0]가 음수/폭주를 막는다.

**문헌 위치(개정)**: 잔차 검정 = Reina 2006·Yu 2023(χ²)의 **변형**, 유계 IAE = Mehra/Mohamed-Schwarz의 변형, 게이팅+적응 결합은 FusionCore(2026)에 이미 존재. 기여 = (a) 차동구동·시뮬 잡음 모델에서 Σ_r·임계의 **폐형 유도**(피어 리뷰에서 "왜 13.8인가"를 χ² 분위로 설명), (b) 필터 밖 잔차 기반 적응으로 두 필터에 동일 입력, (c) 6-상태 경량(FusionCore 23-상태 UKF 대비). **novelty_claim: variant_of_prior**.

**기대 효과(예측, S5에서 측정)**: 슬립 과도 구간에서 인코더 편향의 흡수를 차단하므로 odom 프레임 위치 오차 감소는 "슬립 과도가 전체 오차에서 차지하는 비율"에 비례 — μ = 0.1 패치 5 m 통과 시 가감속 슬립 Δv ≈ 0.1–0.3 m/s × 0.5 s ≈ 5–15 cm 오차를 대부분 제거할 것으로 예측(≥ 50 %는 **가설**, 측정으로 보고). map 프레임 RMSE는 AMCL 지배라 ≤ 1 cm 개선, 대신 슬립 중 AMCL 발산 횟수 감소. **리스크**: 저속에서 σ 하한 필수; 정속 대칭 슬립 미검출(위 한정).

### 4.B SMC-CUSUM: 동적 빔 마스크 + 스캔-맵 인라이어 비율 순차 검정 (개정)

**통계량**(스캔 t = AMCL과 같은 `scan_filtered`, x̂ = `odometry/filtered_map`(TF 활성 구현), D = AMCL과 동일 EDT, 캡 d_max = 1.0 m; 정적 맵 레이캐스팅으로 기대 거리 r̂_k 계산 — 180빔 Bresenham ≈ 9×10⁴ 셀 ≈ 0.3 ms):
$$\text{mask}_k=\mathbf 1[z_k<\hat r_k-3\sigma_{hit}]\ (\text{맵에 없는 근접물 = 동적 장애물/타 로봇}),\quad
\rho_t=\frac{\sum_k(1-\text{mask}_k)\mathbf 1[d_k<3\sigma_{hit}]}{\sum_k(1-\text{mask}_k)},\quad f_t=\frac1K\sum_k\text{mask}_k$$
- 동적 점유는 마스크로 **제외**되므로 ρ_t는 "맵보다 멀리 보이는 빔"(벽을 통과 = 납치 증거)과 끝점 불일치에만 반응한다. 납치로 벽이 예상보다 가까워지면 마스크가 과다해지므로 **f_t > f_th가 1 s 지속**을 2차 통계로 둔다(초기 f_th = 0.5, S6에서 5대·보행자 밀집 시 f_t 분포의 99.9 % 분위로 재보정; 비마스크 빔 K' < 30이면 ρ_t 무효).
- m_t = 캡 평균거리(CAER형)는 §4.C 스코어링과 공용.

**CUSUM**(Page 1954; Lorden 1971 — 주어진 평균 오경보 간격 ARL₀에서 크기 ≥ δ의 계단 변화를 최소 지연으로 검출하는 최적성):
$$g_t=\max\big(0,\ g_{t-1}+(\rho_0-\rho_t-\delta)\big),\quad \text{alarm if } g_t>h$$
**(δ, h)는 주장하지 않고 보정한다(개정)**: S6(동적 장애물 5개 + 타 로봇) 10 min 로그에서 ρ_t 분포를 얻어 δ = ρ_0 − q_{0.05}(ρ_t) 정도로 두고, h는 부트스트랩 ARL₀ ≥ 3600 s(≥ 36 000 스캔)를 만족하는 최소값으로 선택, 결과 **측정 오경보율**을 리포트. 마스크 없는 원본 파라미터(δ 0.15, h 1.0)의 결함(사람 0.5 m ≈ 빔 15 %, 두 명 또는 지게차 → 30 % 하락 2 s면 g = 3 > 1) 인정. 디바운스 임계(연속 3스캔) 대비 CUSUM의 이점: 약한 지속 하락(예: 0.25 하락 8스캔)을 누적 검출, 같은 ARL₀에서 지연 최소 — 두 방식을 절제 실험으로 비교.

**EKF 측 보강**: AMCL innovation NIS ε > 16.3 3회 연속 ∧ tr(P_amcl,xy) > 0.5² → 보조 경보. robot_localization은 NIS를 발행하지 않으므로 모니터가 ν = z_amcl − x̂_map, S = P_map(`odometry/filtered_map` 공분산) + R_amcl로 직접 계산한다(팀 EKF는 `amr_ekf/diagnostics`의 값 사용). 최종 경보 = CUSUM ∨ f-경보 ∨ 보조.

**범위 한정(개정)**: 텔레포트 후 **국소 기하가 바뀌는** 납치(다른 형태의 통로/개활지/벽 통과)는 ρ_t·f_t가 잡는다. **기하적으로 대칭인 통로로의 조용한 납치는 스캔이 옛 포즈와도 일치하므로 스캔 지표로는 원리적으로 검출 불가** — 이 경우 오도메트리 불연속(텔레포트 순간 v_enc = 0인데 위치가 점프하는 것은 시뮬 외부 사건이라 EKF에는 보이지 않음), 플릿 상호 관측, 마커 등 외부 단서가 필요하며 본 브리프 범위 밖.

**문헌 위치(개정)**: 지표는 CAER/SMAD/Akai 신뢰도 함수의 **변형**(마스크 + 인라이어 비율), Campbell 2015의 "정합 지표 + 독립 추정치 불일치" 병용을 비학습·순차검정으로 구성. 스캔-맵 인라이어 비율에 CUSUM을 적용한 문헌은 확인하지 못함 → **new_combination(좁은 의미)**. 연산: EDT 1회 10 ms(3.8 MB), 스캔당 레이캐스트 0.3 ms + 룩업 10 µs.

### 4.C 능동 복구 FSM + coarse-to-fine 가설 시드 + 이동 판별 (개정, 감사 시 재설계)

```
state TRACKING:  amcl_pose -> amcl_pose_gated (cov 하한); alarm -> SUSPECT
state SUSPECT (0.5 s):  게이트 차단(ekf_map dead-reckoning, map->odom 유지), localization/lost = true; 지속 -> SEED (해소 시 TRACKING)
state SEED:
  H0 = 자유공간(EDT > 0.3 m) 격자 0.5 m x 헤딩 5 deg     # 자유공간 1000 m² 기준 ≈ 4000 x 72 = 2.9e5 가설
  coarse m(h) = 캡 평균거리(90빔, d_max 1 m, 트리밍 없음)   # 26-48 M 룩업, 3.8 MB EDT는 L3 상주: 단일 스레드 0.5-1.5 s, OpenMP 8스레드 0.1-0.3 s
  top-50 클러스터(2 m / 30 deg 분리) -> fine 0.1 m x 2 deg (±0.5 m, ±8 deg, 11x11x9) x 180빔, 80 % 트리밍 평균 m~   # 9.8 M 룩업, 0.1-0.3 s
  모드 병합(0.5 m / 10 deg) -> point-to-EDT ICP(Gauss-Newton, ≤ 20 반복) -> 모드를 m~로 순위화, 상위 2개는 마스크 점수 m^도 계산   # m^: 기대거리 레이캐스트, "기대보다 짧은 빔"만 제외
  M = {h : m~(h) < m_ok}          # m_ok: S1-S3의 GT 포즈 m~ 분포 99.9 % 분위로 보정(초기 0.08 m); |M|이 50에 포화되면 top-100으로 재실행(+0.2-0.6 s)
  if |M| == 1 or (m~_2 - m~_1 > δ_m and m^_2 - m^_1 > δ_m) (δ_m 초기 0.02 m, S4 보정):   /set_initial_pose(h_1, cov diag(0.3², 0.3², 0.1²)) -> ROTATE
  elif |M| ≥ 2:  -> DISAMBIGUATE(M 전체, 최대 K = 50)
  else:          -> GLOBAL
state DISAMBIGUATE:   # 능동 정위(Fox 1998)의 단순화: 모드별 포즈를 odom 증분으로 전파하며 이동으로 별칭을 가른다
  drive_on_heading(국소 costmap 기준 최대 여유 방향, 0.3 m/s, ≤ 2 m/회; odom 프레임이라 전역 정위 불필요)
  매 스캔 모든 모드 재채점 m~, m^ (K x (180빔 레이캐스트 + 룩업) ≈ K x 0.3 ms ≤ 15 ms); m~ > m_ok 3스캔 연속인 모드 제거
  모드 1개 또는 margin > δ_m 5스캔 연속 -> /set_initial_pose(h_1) -> ROTATE
  누적 10 m 또는 40 s 초과 -> GLOBAL
state GLOBAL:   # 2차 경로
  set_parameters_atomically(sigma_hit 0.2, laser_likelihood_max_dist 2.0, max_particles 20000, min_particles 2000)  # reinit_laser + reinit_pf 각 1회
  /reinitialize_global_localization -> ROTATE
state ROTATE:
  set_parameters(update_min_a 0.05)            # 재초기화 없음; 1 rad/s x 10 Hz = 0.1 rad/스캔이 엄격 부등호 0.1에 걸리는 것 방지
  Nav2 Spin(2*pi, 1.0 rad/s, 6.3 s)
  매 스캔 rho(마스크 후), m^, tr(P_amcl)
  accept if rho >= rho_0 - delta for 5 scans AND tr(P_xy) < 0.3^2 -> VERIFY
  fail: GLOBAL(최대 2회) else FAIL
state VERIFY:
  GLOBAL 경유였다면 AMCL 평균을 SEED 모드 목록과 대조: 다른 모드와 margin ≤ δ_m -> DISAMBIGUATE
  set_parameters_atomically(추적값 복원)   # reinit_pf는 낡은 init_pose_로 재생성 -> 즉시 /set_initial_pose(AMCL 평균, cov); 그 사이 amcl_pose는 게이트가 차단
  ekf_filter_node_map/set_pose(AMCL 평균, cov); g <- 0; 게이트 복원, localization/lost = false -> TRACKING
state FAIL: localization/lost = true 유지, localization/health = LOST -> fleet 보고, 정지
```

**수치 근거(감사 시 재유도 — checks/seed_sim.py, seed_pipeline_sim.py, seed_modes_sim.py)**: 합성 60 × 40 m 창고(1.2 m 랙 × 12 m, 3 m 통로가 3블록 × 7열로 **완전 주기적**인 최악 조건), 180빔 σ 0.03 m, 빔 10 %를 0.5–2 m 동적 가림으로 치환, 무작위 참 포즈 25회(seed_pipeline_sim; seed_sim 40회, seed_modes_sim 15회).
(i) 원문 격자 1 m × 15°는 "캡 평균거리의 완만한 분지 안이라 적중이 결정적"이라 했으나 **틀렸다**: 7.5° 헤딩 오프셋은 10/20 m 빔 끝점을 1.3/2.6 m 옮겨 d_max = 1 m 캡에 포화시키므로 분지가 1 m보다 훨씬 좁다. 측정: 참 포즈가 top-20 후보의 fine 창(±0.5 m, ±8°) 안에 든 비율 **8/25**. 0.5 m × 5°(최악 0.35 m / 2.5° → 10 m 빔 0.44 m)는 **22/25(top-20), 23/25(top-50)**.
(ii) 점수 선택: coarse 단계의 트리밍은 판별 빔을 버려 순위를 악화(1 m × 15°, top-20 적중: 가림 없음 25/40 → 20/40, 가림 10 % 20/40 → 16/40)하므로 쓰지 않는다. fine 단계에서는 가림에 강건해야 한다: 참 포즈의 캡 평균은 가림 10 %에서 0.12 m(원문 m_ok 0.10을 넘어 **참 포즈를 기각**)이지만 80 % 트리밍은 0.014 m. "트리밍이 별칭을 가르는 빔까지 버린다"는 우려로 **마스크 점수 m̂**(§4.B 규칙: 기대거리보다 3σ_hit 넘게 짧은 빔만 제외, "벽을 뚫고 보이는" 빔은 유지)도 시험했으나 순위는 개선되지 않았다(15회 중 1위 정답: 트리밍 9, 마스크 6; seed_modes_sim.py). 다만 마스크 여유 > 0.02 m인 경우는 3/3 정답·오수락 0이어서, **순위는 m̃, 단봉 수락은 m̃·m̂ 두 여유를 모두 요구**하는 보수적 규칙으로 정했다(측정된 오수락: 마스크 여유 > 0.02 m 규칙 0/15, 트리밍 여유 > 0.05 m 규칙 0/25(0.5 m 격자) — 원문 1 m × 15° 격자는 같은 규칙에서 1/25 오수락; 표본이 작으므로 δ_m은 S4에서 재보정).
(iii) **주기적 랙에서는 다봉이 기본값**: fine 후 1·2위 모드의 점수 차 중앙값 3 mm, 1위와 0.02 m 이내인 모드 수 중앙값 11(최대 50 = 포화), 원문 단봉 조건(margin > 0.05 m)은 3–4/25만 통과(오수락 0), 이동 없이 1위를 고르면 정답 13/25. 360° LiDAR에서 제자리 회전은 새 기하를 주지 않으므로 원문의 "다봉이면 h_1, h_2, h_3을 회전으로 순차 검증"은 **별칭을 기각할 수 없다**(틀린 별칭에서도 ρ가 높게 유지 → 오수렴 수락). 그래서 이동으로 가르는 DISAMBIGUATE를 넣었다.
(iv) GLOBAL 경로: N = 20 000, σ_hit 0.2에서 분지 기대 적중 0.9개(P ≈ 58 %/초기화) + 회전 중 augmented 주입; 주기적 배치에서는 별칭 수렴이 가능하므로 VERIFY의 모드 대조가 필수. 리샘플은 §2.4대로 1 코어를 6–10 s 점유(처리 갱신 ≈ 20–35회).
(v) 시간 예산: 검출 0.5–1 s + SEED 0.3–0.8 s(8 스레드) + ROTATE 6.3 s → **단봉 ≈ 8 s**. 다봉은 DISAMBIGUATE가 통로 끝·교차로까지 가야 갈리므로 0.3 m/s로 2–10 m = 7–35 s가 더해져 **30 s를 넘길 수 있다**. GLOBAL 1회 ≈ 10–15 s. 따라서 "30 s 내 ≥ 90 %"는 주기적 통로에서 뒷받침되지 않는다 → S4 목표를 **60 s 내 ≥ 90 %, 오수렴 수락 0회**로 두고 30 s 내 비율은 비대칭 위치/주기적 통로를 나눠 측정·보고한다(명세는 시간 한도 없이 "스스로 복구"를 요구). 합성 최악 조건에서 정답 모드가 추적 집합에 들어오는 비율 23/25(top-50 창), 13/15(fine 모드 목록)로 ≈ 87–92 %가 SEED+DISAMBIGUATE 경로 성공률의 상한 추정이며, 나머지는 GLOBAL로 넘어간다.
(vi) 360° LiDAR에서 제자리 회전의 역할은 `update_min_a`를 넘겨 갱신·리샘플을 구동하는 것(`request_nomotion_update` 반복과 등가, 명세의 "제자리 회전 등" 충족)이며, 별칭 판별은 이동(DISAMBIGUATE)이 담당한다.
(vii) nav2는 이미 `free_space_indices`에서 균일 샘플링하므로 "자유공간 시드" 자체는 기여가 아니고 **스코어링·정련·마스크 판정·이동 판별**이 기여. 합성 맵은 실제 창고보다 주기성이 강한 최악 조건이며, 실제 맵(도크·충전소·기둥 등 비대칭 요소)에서의 값은 S4에서 측정한다.

**문헌 위치**: 가설 스코어링·정련 = CBGL(CAER + scan-to-map-scan)·SFHS의 **engineering_adaptation**(Nav2 서비스/동적 파라미터로 구현), 이동 판별 = Fox 1998 능동 정위·Lenser–Veloso 2000의 단순화, 검출-표적 재시드 = Akai 2023·Zhang 2012(SAMCL)와 같은 계열(AMCL 외부 구현). **novelty_claim: engineering_adaptation**.

---

## 5. 3 / 5 / 8 cm 달성 전략과 오차 예산 (개정)

| 오차원 | 크기 추정 | 대응 |
| --- | --- | --- |
| 관측 모델 단일 갱신 폭 + 파티클 잔여 분산 | σ_hit/√3 ≈ 2.9 cm(σ_hit 0.05) → 시간 누적·작은 α로 1–2 cm; 평균 추정치 오차 ≈ 폭/√N_eff + 편향 | α·σ_hit 공동 스윕, max_beams 180 |
| 맵 편향(체계적, 평균화 안 됨) | ADNN 2–3 cm — **정지 3 cm의 지배항** | ADNN ≤ 2 cm 목표, 오정합 시 재매핑 |
| 맵 양자화 | res/√12 = 1.4 cm | — |
| AMCL 지연(2 m/s × 0.1 s) | 최대 20 cm | `smooth_lagged_data`/이력 재적용 — **직선 5 cm의 지배항** |
| 정지 드리프트 | 0(ZUPT) + 마지막 AMCL 오차 | nomotion update 3회 |
| 자이로 바이어스(보정 후) | 4×10⁻⁴ rad/60 s → 20 m에 0.4 cm(상한 0.8 cm) | §2.2 보정(`imu_filter_node`) + 잔차 상태 |
| 회전: 헤딩 × 레버암 0.15 m | δθ = 2° → 0.5 cm | URDF–sensors.yaml extrinsic 일치 검증 |
| 회전: IMU LPF 지연(재개 감사 추가) | 보상 전 τ_LPF 9.7 ms → 선회 중 헤딩 지연 τω, odom 프레임 위치 오차 v·τ·Δθ = 1.5 cm(1 m/s, 90°), 3.1 cm(2 m/s, 90°) | `imu_filter_node` 스탬프 보상(잔여 ≤ 0.7 ms → ≈ 0.2 cm 이하, §7) |
| 회전: 스캔 왜곡 | 0(GPU LiDAR 동시 렌더; sim-to-real gap 명시) | 실기 시 deskew |
| tf 시간 보간 | transform_tolerance 0.1 s | ekf 50 Hz |

결론: 정지 오차는 맵 편향이 지배하므로 3 cm는 **ADNN ≤ 2 cm를 달성해야 여유가 생기는 타이트한 목표**이고, 직선 5 cm는 지연 보정이, 회전 8 cm는 헤딩·지연이 지배한다. 예상치: 정지 2–3 cm, 직선 3–4 cm, 회전 4–6 cm(S1–S3 측정으로 대체).

**RMSE/Max 측정 절차**: GT = Gazebo `OdometryPublisher`(`worldPose()`, `gaussian_noise 0`, 소스 확인) 50 Hz → `ground_truth/odom`(계약 §5.1, 평가 전용); canonical link 차이는 `xyz_offset`. map↔world: SLAM 시작 포즈를 world 원점·yaw 0에 두고 잔여 T_world←map을 보정 런의 SE(2) 최소제곱(Umeyama)으로 1회 추정·고정(정렬 전/후 보고). `/clock` 시뮬 시간, GT를 추정 스탬프에 선형 보간. e_i = ‖p_gt − p_est‖₂, RMSE, Max, 헤딩 별도. 구간: 정지(|v| < 0.02 m/s 1 s 이상), 직선(|ω| < 0.1 ∧ v > 0.2), 회전(|ω| ≥ 0.3). 명세 로그 포맷, 구간별 ≥ 100 샘플.

---

## 6. 평가 계획 (명세 지표 매핑, 개정)

| 시나리오 | 내용 | 지표 / 합격선 |
| --- | --- | --- |
| S1 정지 | 5개 지점 × 60 s, nomotion update | RMSE·Max ≤ 3 cm |
| S2 직선 | 메인 통로 40 m, v = 0.5/1.0/2.0 | ≤ 5 cm(지연 보정 유무) |
| S3 회전 | 제자리 ±1.5 rad/s, 코너 | ≤ 8 cm |
| S4 납치 | `/world/<w>/set_pose` 10회: 5–30 m, 대칭 통로 3회 포함, 정지/주행 | 검출 지연 ≤ 1 s(기하 변화 케이스), **60 s 내 복구(오차 < 0.2 m) ≥ 90 %, 오수렴 수락(오차 > 0.5 m로 TRACKING 복귀) 0회**, 30 s 내 비율은 비대칭 위치/주기적 통로로 나눠 측정 보고(§4.C (v)), 경로별(SEED 단봉/DISAMBIGUATE/GLOBAL) 집계 |
| S5 슬립 | μ = 0.1 패치 5 m + 25 kg + WheelSlip | odom 드리프트 [m/m], AMCL 발산 횟수, 상수 R vs IAG |
| S6 동적 | 이동 장애물 5개 0.3–1.5 m/s + 타 로봇 | ρ_t·f_t 분포, **(δ, h) 보정, 측정 오경보율** |
| S7 5대 동시 | 전체 플릿 | CPU ≤ 80 %, **노드별 예산**(§7), 지표 유지 |
| S8 4시간 연속(신규) | 플릿 반복 임무 4 h | 시간별 RMSE 추세, ρ_0 재보정(5 min 롤링 중앙값, 경보 중 동결, 하한 0.7), RSS/CPU 추세(누수 0), 파티클 수, 경보 수, EKF 이력 버퍼 유계 |
| S9 공칭 드리프트(신규, 명세 2장) | 직진 20/50/100 m, 폐사각 10 m, 하중 0/25 kg, v 0.5/1/2 | 병진 [%], 헤딩 [deg/m], 폐루프 복귀 오차; raw vs ekf_odom(§2.1 예측: 측면 3 % → 0.02–0.035 %) |
| M1 맵 | 3회 매핑 | IoU_1 ≥ 0.85, ADNN ≤ 3 cm(목표 2), 반복 IoU ≥ 0.9, 잔차 χ²/dof, 벽 두께 ≤ 2셀 |
| M2 루프 폐합(신규) | 100 m 강제 재방문 루프 2회 | 후보/수락/기각 수(로그), ATE 최적화 전/후, Huber 유무 |

비교군: **B0** Nav2 기본 AMCL + robot_localization 상수 공분산 · **B1** 튜닝 AMCL + 상수 공분산 · **Ours** B1 + IAG + SMC-CUSUM + 복구 · 절제: Σ_enc 모델만(게이팅·적응 없음, 재개 감사 추가)/게이팅만/적응만/CUSUM vs 디바운스 임계/시드 없이 GLOBAL만/이동 판별(DISAMBIGUATE) vs 원문식 회전 순차 검증(오수렴 수 비교)/바이어스 상태 vs 보정만/팀 EKF Q_pos 0 vs r_l 동일값. 각 10런, 시드 고정. **α × σ_hit 공동 스윕**(5 × 4 = 20셀, 셀당 3런 → 상위 3셀만 10런)은 S2에서, ekf_map q_p 스윕(§2.2)은 S1/S2에서. 필터 일관성: NEES(5-DOF: x, y, θ, v, ω — b_g는 GT 없음) **런별 95 % 구간 [0.83, 12.8], 10런 평균 [3.24, 7.14]**(χ²₅₀/10, 개정); NIS 히스토그램; Chen 2023 절차로 q, R 오프라인 미세 튜닝.

**자동화 통합 테스트(명세 ≥ 10개, launch_testing + pytest, 개정)**: IT-01 TF 트리(map→odom→base_footprint→base_link→센서, 이중 부모 없음, `view_frames`) · IT-02 S1 정지 ≤ 3 cm · IT-03 S2 1 m/s ≤ 5 cm · IT-04 S2 2 m/s ≤ 5 cm · IT-05 S3 ≤ 8 cm · IT-06 S4 10회 중 ≥ 9회 60 s 내 복구 ∧ 오수렴 0회 · IT-07 S6 10 min 오경보 ≤ 보정 목표 · IT-08 S5 슬립 중 max 오차 < 0.3 m · IT-09 S7 CPU ≤ 80 % · IT-10 M1 IoU/ADNN · IT-11 M2 루프 수락 ≥ 1, ATE 감소 · IT-12 S9 드리프트 상한 · IT-13 S8(야간 잡, CI 제외).

---

## 7. 구현 계획 (개정)

**패키지** `src/amr_localization`(C++17 노드 + Python 평가), 메시지 `amr_msgs`.

| 파일 | 역할 | 인터페이스 |
| --- | --- | --- |
| `src/imu_filter_node.cpp`(계약 노드명) | LPF(2차 Butterworth 20 Hz = `lpf_cutoff_hz`; 저주파 군지연 τ_LPF = 9.7 ms → **출력 스탬프를 τ_LPF만큼 앞당겨 발행**, §4.A 시각 정렬 전제·§5) + `bias_estimation_time`(기본 60 s) 정지 바이어스 추정 b̂_g, b̂_a, ZUPT 시 재추정 | sub `imu/data_raw`; pub `imu/data`(보정됨, cov σ_g² + SE² 등), `imu/bias`(신규, amr_msgs/ImuBias) |
| `src/wheel_odometry_node.cpp` | `joint_states` → 틱 양자화 → §2.1 → Σ_enc(2×2) → **슬립 잔차·χ² 게이트·표본별 잔차 기반 R̂**(§4.A) | pub `wheel_odom`(frame `<r>/odom`, child `<r>/base_footprint`, twist.cov 적응, pose는 드리프트 분석용), `wheel_odom/slip`(신규, SlipStatus: s_k, gated, R̂); sub `joint_states`, `imu/data` |
| `src/amr_ekf_node.cpp` + `include/amr_localization/ekf.hpp` | 6-상태 EKF(잔차 b_g), CWNA Q(+ 선택 Q_pos), Joseph, NIS 게이트, ZUPT, 1 s 이력 재적용, 이벤트 스케줄 예측, `world_frame`·`base_link_frame: base_footprint`, `publish_tf`(r_l과 배타) | sub `wheel_odom`, `imu/data`, `amcl_pose_gated`(map 인스턴스); pub `odometry/amr_ekf` / `odometry/amr_ekf_map`, tf(활성 구현일 때만); srv `amr_ekf_odom/set_pose`·`amr_ekf_map/set_pose`; pub `amr_ekf/diagnostics`(nis[], gated[], nees) |
| `src/kidnap_monitor_node.cpp`(**계약상 Python → C++ 변경 요청**: SEED 단계 2–50 M 룩업을 OpenMP로 0.1–1 s 안에 끝내야 함) | EDT + 레이캐스트 마스크, ρ_t/f_t/m_t, CUSUM, NIS 보조, FSM, 가설 스코어링(OpenMP)·마스크 판정·ICP, DISAMBIGUATE 모드 추적(odom 증분), AMCL 동적 파라미터 클라이언트 | sub `scan_filtered`, `/map`, `amcl_pose`, `odometry/filtered_map`, tf; pub **`localization/lost`(계약 유지: std_msgs/Bool, latched, BT `IsLocalized`)**, `amcl_pose_gated`(신규), `localization/health`(신규, 상세 상태); clients `reinitialize_global_localization`, `request_nomotion_update`, `set_initial_pose`, `amcl/set_parameters_atomically`, `ekf_filter_node_map/set_pose`(리맵 후 이름) ; action `spin`, `drive_on_heading`(Nav2 behavior, odom 프레임) |
| `config/ekf.yaml`(repo, 두 r_l 노드 공용 — 계약 §6) · `src/amr_localization/config/{amcl,slam_toolbox,kidnap_monitor,amr_ekf,imu_filter}.yaml` | §2/§4 값, 네임스페이스·프레임 접두(ekf.yaml 주석의 런치 규칙) | — |
| `launch/{slam,localization}.launch.py` | map_server(루트 1개)+amcl+imu_filter+ekf×2(`ekf_impl` 인자로 TF 발행 구현 선택)+monitor / slam_toolbox; **EKF 인스턴스별 `set_pose` 토픽·서비스 리맵**(`set_pose` → `ekf_filter_node_{odom,map}/set_pose`) | 로봇당 1세트 |
| `amr_localization/eval/{gt_recorder,localization_errors,odom_drift,map_quality,loop_closure_log}.py` | GT 동기 로깅, SE(2) 정렬·구간·RMSE/Max·NEES, S9 드리프트, GT 격자(visual, z 0.38)·IoU_τ·ADNN·Filatov 3종·직각 편차, M2 로그 파싱 | CSV → 표·그림 |
| `test/` | gtest: Jacobian 유한차분(< 10⁻⁶), 합성 NEES, Σ_r·χ² 게이트(무슬립 회전 램프 ω̇ 2 rad/s² 중 오발 ≤ 0.2 %, LPF 스탬프 보상 포함 — 재개 감사 추가), 바이어스 가관측성(ZUPT 후 P_bb 감소), CWNA Q 대칭·PSD, 오도메트리 폐형해, CUSUM 지연·ARL, 가설 스코어 합성맵 적중(checks/seed_pipeline_sim.py 이식: 주기 창고 맵 top-50 창 적중 ≥ 90 %, 마스크 판정 오수락 0), 표본별 잔차 R̂ 불편성(checks/iag_residual_mc.py 이식); pytest: IoU_τ/ADNN 토이 격자 | 커버리지 ≥ 70 % |
| `test/integration/` | IT-01…IT-12 launch_testing | CI, IT-13 야간 |

**robot_localization 기준선 변경**(repo ekf.yaml 대비, 감사 시 계약·repo 근거와 재정렬): `imu0: imu/data`(유지 — 계약상 이미 보정된 토픽), `imu0_config` yaw false·vyaw true·**ax/ay true 유지**(repo의 구심항 근거, §2.2), `odom0_twist_rejection_threshold 4.0`(vx, vy, vyaw 3차원), `pose0: amcl_pose_gated`, `pose0_rejection_threshold 4.0`(§2.3의 set_pose 복구 경로 전제), `smooth_lagged_data true`·`history_length 1.0`(repo map 노드에 이미 있음)·`predict_to_current_time true`, Q vx 0.005 / vyaw 0.02, ekf_map Q x,y ∈ {10⁻³, 10⁻², 0.05} 스윕(§2.2)·yaw 10⁻⁴, `initial_estimate_covariance`는 **ekf_map만** x, y, yaw 10⁻²(ekf_odom은 odom 원점 정의상 repo의 10⁻⁹ 유지), `base_link_frame: base_footprint`(유지). B0 비교군은 repo ekf.yaml 그대로. **R 출처 정합(재개 감사 추가)**: r_l의 측정 R은 ekf.yaml이 아니라 메시지 공분산에서 오며, repo ekf.yaml 주석은 `wheel_odom.twist.covariance`를 slip 0.01 → σ² = 10⁻⁴(vx, vy, vyaw)로 적는다. 이는 k_s를 절대 σ로 읽은 값이라 §2.1 모델 대비 vyaw를 1 m/s 직진에서 15배, 2 m/s에서 **62배 과소**(σ_ω,enc² = 6.2×10⁻³), vx는 2 m/s에서 2배 과소다(checks/audit_resume.py). 따라서 B0/B1의 "상수 공분산" = 이 주석값 diag(10⁻⁴, 10⁻⁴, 10⁻⁴)로 정의해 기준선 결함을 그대로 재현하고, Ours는 §2.1 Σ_enc(속도 의존, 하한 10⁻⁴) + IAG를 쓴다 — 공분산 모델 효과와 IAG 효과를 가르기 위해 절제군 "Σ_enc만(게이팅·적응 없음)"을 §6에 추가; vy(비홀로노믹 구속 vy ≡ 0, repo가 융합)는 모든 구현에서 10⁻⁴ 고정. repo 주석 갱신은 docs/algorithms/ekf.md에 근거와 함께 남긴다.

**계약 변경 요청(components.md §3.2·§5.2 갱신 대상, 감사 추가)**: (1) `ekf_filter_node_map`의 `pose0`를 `amcl_pose` → `amcl_pose_gated`, (2) `kidnap_monitor_node` 언어 Python → C++ 및 인터페이스 확장(위 표; `localization/lost`·`amcl_pose`·`reinitialize_global_localization`·`spin`은 유지, 구독 `odometry/filtered` → `odometry/filtered_map`(map 프레임 x̂ 필요; TF 활성 구현이 팀 EKF면 `odometry/amr_ekf_map`)으로 교체, `scan_filtered`·`/map`·AMCL 서비스/파라미터 클라이언트·`drive_on_heading` 추가), (3) 두 EKF의 `set_pose`(토픽·서비스)가 같은 네임스페이스에서 충돌하므로 인스턴스별 리맵을 계약에 명기, (4) 신규 토픽 `imu/bias`, `wheel_odom/slip`, `localization/health`, `odometry/amr_ekf[_map]`, (5) `imu_filter_node`의 `bias_estimation_time`은 계약 기본 60 s 유지(10 s도 sensors.yaml 기준 충분).

**노드별 연산 예산(5대, 개정)**: EKF 2 인스턴스(섀도 포함 4) ≪ 1 %; imu_filter ≪ 1 %; AMCL 추적 5 × 10 Hz × 0.5 ms ≈ 2.5 % 코어, **GLOBAL 복구 버스트 1 코어 × ≈ 6–10 s(한 로봇씩, §2.4)**; 모니터 5 × 10 Hz × 0.4 ms ≈ 2 %, SEED 버스트 0.3–0.8 s(OpenMP 8 스레드, 0.5 m × 5° 격자), DISAMBIGUATE 스캔당 ≤ 15 ms(모드 ≤ 50); EDT 3.8 MB × 5. 32 스레드 대비 여유이나 S7의 80 % 상한은 Gazebo 렌더링(GPU LiDAR × 5)이 지배 → 노드별 `top` 로그를 리포트에 분리.

**일정(주)**: 1 오도메트리·IMU 전처리·GT 파이프라인·S9 → 2 EKF(odom, 6-상태) + 단위테스트 → 3 AMCL 튜닝(α×σ_hit) + EKF(map) + S1–S3 → 4 SMC-CUSUM(마스크·보정) + 복구 FSM(SEED/GLOBAL) + S4/S6 → 5 IAG + 절제 + S5 → 6 맵 품질·M1/M2·통합 테스트·S8 야간·리포트.

---

## 8. 열린 질문 / 리스크 (개정)

1. ~~`slip_noise_stddev` 단위~~ → sensors.yaml에 무차원 비율로 명시(해소).
2. Gazebo IMU `orientation` ≈ GT 헤딩 → 자이로 율만 융합(설계 결정, §2.2에 문서화). 가속도 ax, ay는 r_l 기준선에서 repo대로 융합 유지(감사 정렬).
3. ~~동적 파라미터 가능 여부~~ → amcl_node.cpp `dynamicParametersCallback`에서 max/min_particles(reinit_pf), sigma_hit/laser_likelihood_max_dist(reinit_laser) 확인(해소). 잔여 리스크: reinit_pf가 `init_pose_/init_cov_`(활성화 시점의 낡은 값)로 클라우드를 재생성하므로 파라미터 변경 직후 반드시 `reinitialize_global_localization` 또는 `set_initial_pose`를 호출하고, 그 사이 발행된 `amcl_pose`는 게이트로 차단한다(FSM에 반영).
4. ~~텔레포트 서비스~~ → gz-sim6 UserCommands `/world/<w>/set_pose`(Pose→Boolean) 확인(해소). S4 하네스는 `ros_gz` 서비스 브리지 대신 `ign service` CLI 호출로 구현.
5. map↔world 정렬 규약·canonical link 합의(시뮬 담당).
6. 주기적 랙 통로에서는 SEED가 다봉을 내는 것이 기본(합성 최악 조건에서 단봉 판정 3–4/25) → DISAMBIGUATE 이동 2–10 m로 30 s를 넘길 수 있음(감사 재유도, §4.C). S4는 60 s 목표 + 30 s 비율 분리 보고. 원문의 "회전으로 3가설 순차 검증"은 360° LiDAR에서 별칭을 기각할 수 없어 폐기.
7. AMCL 입력을 융합 odom으로 바꾸면 α 재튜닝 없이는 고갈(§3-21) → 공동 스윕 필수.
8. O(N²) 리샘플러는 nav2 바이너리 제약 — 패치하지 않는다(명세: AMCL 적용). 복구 상한 20 000 파티클로 관리.
9. 정속 대칭 슬립은 IAG로 관측 불가(§4.A) — S5 결과 해석 시 명시.
10. **계약 변경 요청(§7)** — `amcl_pose_gated`, `kidnap_monitor_node` C++화, EKF `set_pose` 리맵, 신규 토픽 4종: components.md 담당자 합의 후 구현(감사 추가). 합의 전에는 모니터를 계약 이름(`localization/lost`, `amcl_pose` 구독)으로 먼저 구현하고 게이트는 선택 기능으로 둔다.
11. TF 이중 발행: r_l과 팀 EKF 중 한 쌍만 `publish_tf: true`(IT-01로 회귀 검사).
12. IMU LPF 군지연(20 Hz 2차 Butterworth, 9.7 ms): `imu_filter_node`의 스탬프 보상이 IAG 게이트의 전제 — 누락 시 제자리 회전 램프마다 오게이트(P ≈ 0.99)와 선회당 1.5–3 cm odom 오차(§4.A, §5, §7; 재개 감사 추가). `lpf_cutoff_hz`를 바꾸면 τ_LPF를 계수에서 다시 계산.

---

## 9. 참고문헌 (URL)

1. Zhang et al., arXiv 2511.01219 — https://arxiv.org/abs/2511.01219 (VERIFIED)
2. Bilevich, Buber, Halperin, ICRA 2026 — https://arxiv.org/abs/2607.17852 (VERIFIED)
3. Filotheou, CBGL, IROS 2024 — https://arxiv.org/abs/2307.14247 (VERIFIED)
4. Xu et al., Selective KF — https://arxiv.org/abs/2412.17235 (VERIFIED)
5. Zhao et al., SuperLoc, ICRA 2025 — https://arxiv.org/abs/2412.02901 (VERIFIED)
6. Dolatabadi et al. — https://arxiv.org/abs/2501.02558 (VERIFIED)
7. Dolatabadi et al. — https://arxiv.org/abs/2509.18954 (VERIFIED)
8. Diker, Klein — https://arxiv.org/abs/2603.26709 (VERIFIED)
9. Khosravi et al. — https://arxiv.org/abs/2603.09783 (VERIFIED)
10. Chen et al. — https://arxiv.org/abs/2306.07225 (VERIFIED)
11. Mozzarelli et al., ECC 2024 — https://arxiv.org/abs/2403.13452 (VERIFIED)
12. Yu et al., IROS 2023 — https://arxiv.org/abs/2209.15140 (VERIFIED)
13. Chauchat, Bonnabel, Barrau — https://arxiv.org/abs/2409.07050 (VERIFIED)
14. Alwala et al. — https://arxiv.org/abs/2603.14940 (VERIFIED)
15. Kuang et al., ICRA 2025 — https://arxiv.org/abs/2503.23480 (VERIFIED)
16. Gao et al., ERPoT — https://arxiv.org/abs/2409.14723 (VERIFIED)
17. Zhang et al., 2DLIW-SLAM — https://arxiv.org/abs/2404.07644 (VERIFIED)
18. Davies et al. — https://arxiv.org/abs/2504.19654 (VERIFIED)
19. Seghiri, Mansouri, Chemori, TIMC 2025 — https://journals.sagepub.com/doi/abs/10.1177/01423312241267042 (VERIFIED, 초록)
20. Ince, Yiltas-Kaplan, Keleş, Electronics 14(24):4822 — https://doi.org/10.3390/electronics14244822 (VERIFIED, 초록 via Semantic Scholar)
21. Beyond Odometry Accuracy, Robotics 15(9):164 — https://www.mdpi.com/2218-6581/15/9/164 (VERIFIED, 초록)
22. Zhu et al., TempLoc — https://arxiv.org/abs/2602.03198 (VERIFIED)
23. adaptive-ekf-slip-detector — https://github.com/rouf-rimon/adaptive-ekf-slip-detector (VERIFIED, README)
24. Kharwar et al., FusionCore — https://arxiv.org/abs/2605.25239 (VERIFIED)
25. Akai, Reliable MCL — https://arxiv.org/abs/2205.04769 (VERIFIED, arXiv); JFR 40(3):595–613 https://doi.org/10.1002/rob.22149 (서지 VERIFIED via Crossref)
26. Bukhori, Ismail, IJARS 2017 — https://journals.sagepub.com/doi/full/10.1177/1729881417717469 (VERIFIED, 초록 via Semantic Scholar)
27. Campbell, Whitty, RAS 69:40–51, 2015 — https://doi.org/10.1016/j.robot.2014.08.004 (서지 VERIFIED via Crossref; 내용: 검색 초록 + ECMR 2013 선행판 초록 https://openresearch-repository.anu.edu.au/items/7b0e2f6a-6325-46d2-ba46-79c65d8f157f)
28. Reina, Ojeda, Milella, Borenstein, T-Mech 2006 — https://ieeexplore.ieee.org/document/1618677/ , doi 10.1109/TMECH.2006.871095 (서지 VERIFIED, 내용 RECALLED)
29. Fox, Burgard, Thrun, RAS 25:195–207, 1998 (서지 VERIFIED, 내용 RECALLED)
30. Zhang, Zapata, Lépinay, Robotica 30(2):229–244, 2012 — https://doi.org/10.1017/S0263574711000567 (서지 VERIFIED via Crossref, 내용 RECALLED)
31. Nav2 AMCL 문서 — https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/ ; nav2_amcl humble 소스 amcl_node.cpp(`dynamicParametersCallback`, `initParticleFilter`, `uniformPoseGenerator`, `globalLocalizationCallback`) / pf.c(`pf_update_resample` 선형 샘플러, `pf_resample_limit`) / likelihood_field_model.cpp(`p += pz³`) / differential_motion_model.cpp — https://github.com/ros-navigation/navigation2/tree/humble/nav2_amcl (VERIFIED, 개정 시 재조회)
32. robot_localization humble-devel — https://github.com/cra-ros-pkg/robot_localization/tree/humble-devel — ekf.cpp(`P += Δt·Q`), filter_base.cpp(n_σ² 비교), ros_filter.cpp(`set_pose` 상대 이름) (VERIFIED, 감사 시 재조회)
33. slam_toolbox README(ros2) — https://github.com/SteveMacenski/slam_toolbox/blob/ros2/README.md ; solvers/ceres_solver.cpp(`HuberLoss(0.7)`, 2.6.10 태그)·solvers/ceres_utils.h(`PoseGraph2dErrorTerm`)·lib/karto_sdk/src/Mapper.cpp(`TryCloseLoop`)·config/mapper_params_online_async.yaml — https://github.com/SteveMacenski/slam_toolbox/tree/2.6.10 (VERIFIED, 감사 시 조회)
34. gz-sim6 OdometryPublisher — https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/odometry_publisher/OdometryPublisher.cc (VERIFIED); gz-sim6 UserCommands(`/world/<w>/set_pose`) — https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1UserCommands.html (VERIFIED); gz-sensors6 GpuLidarSensor(visual 지오메트리 측정) — https://gazebosim.org/api/sensors/6/classignition_1_1sensors_1_1GpuLidarSensor.html (VERIFIED); ros_gz_bridge README(humble) (VERIFIED)
35. Filatov et al., 2D SLAM quality evaluation methods — https://arxiv.org/abs/1708.02354 (VERIFIED, PDF 본문: 점유셀 비율·모서리 수·폐영역 수)
36. 고전(RECALLED): Thrun·Burgard·Fox 2005; Fox IJRR 2003; Bar-Shalom·Li·Kirubarajan 2001; Mehra 1970; Mohamed·Schwarz 1999; Page 1954; Lorden 1971; Moore·Stouch 2014; Macenski·Jambrecic JOSS 2021 (https://doi.org/10.21105/joss.02783); Konolige et al. 2010; Santos et al. 2013; Sturm et al. 2012; Lenser·Veloso 2000; REP-105 (https://www.ros.org/reps/rep-0105.html).

---

## 10. 리뷰 반영 이력 (2026-09-22)

**수학 오류 — 모두 수정**

| 지적 | 조치 |
| --- | --- |
| 자이로 R에 바이어스를 백색잡음으로 포함(1e-3은 가속도 바이어스 σ) | §1에 sensors.yaml 실제값(gyro bias 0.01 rad/s 정적, accel 0.10 m/s²) 반영; §2.2를 6-상태(b_g)로 확장, R_gyro = σ_g² = 4e-8, 10 s 정지 보정 + ZUPT 재관측, 보정 후 헤딩 드리프트 예산(4e-4 rad/60 s) 재계산; §4.A 잔차에 b̂_g, b̂_a 적용 |
| AMCL α를 σ 배수로 오기(0.05 = 32배) | §2.4를 분산 단위로 재서술, α_phys 유도(5e-5), 권장 2e-3, α×σ_hit 공동 스윕(§6) |
| 리샘플링 O(N log N) 오기 | pf.c 재조회로 선형 샘플러 O(N²) 확인; 비용표·5대 예산·복구 상한 20 000·resample_interval 근거 재작성(§2.4, §4.C, §7). **감사 시 보완**: §2.4/§4.C 버스트 수치 모순(5 s vs 10 s) 통일, `update_min_a` 경계 문제 |
| σ_map 0.02 vs ADNN 3 cm 불일치 | σ_map = 0.038 → σ_hit ≈ 0.05(튜닝 변수로 격하) |
| 균일 재초기화 5000개 기대 적중 | 3-D 분지 계산(σ_hit 0.05: 0.003개; 0.2/20 000: 0.9개) 명시, SEED를 1차 경로로, GLOBAL은 σ_hit·max_particles 동적 확장 후 2차 경로 |
| SEED 200×36 + 12 cm 허용 = 죽은 코드 | 캡 평균거리(CAER형) coarse → fine → ICP로 재설계. **감사 시 재수정**: 1 m × 15°의 "결정적 적중" 주장이 시뮬레이션으로 반증(8/25) → 0.5 m × 5°·top-50·마스크 판정·이동 판별(DISAMBIGUATE)로 교체 |
| r_a 유한차분 σ ≈ 1 m/s² | 0.2 s 창 속도 잔차로 교체(σ 0.02 m/s = 0.10 m/s²), 가속도 바이어스 보정 포함 |
| r_ω, r_a 독립 가정(회전 시 교차항) | r = A n + m 형태로 2×2 Σ_r(교차항 자동 포함), s = rᵀΣ_r⁻¹r |
| IAE가 P⁻·innovation 필요 → "동일 입력" 모순 | 잔차 기반(필터 무관) 공분산으로 교체, W = 50·클립 근거 명시. **감사 시 재수정**: 창 잔차 추정식이 R을 12배 과소추정 → 표본별 잔차로 교체 |
| 게이팅 시 a_x 전파가 5-상태 모델과 상충 | 기본 설계는 인코더 갱신 생략 + Q_vv 팽창; a_x 제어입력은 절제 실험으로만 |
| σ_hit/√B 0.4 cm | 가산형 가중치의 단일 갱신 폭 σ_hit/√3 ≈ 2.9 cm로 교체, 잔여 분산 행과 병합 |
| IoU_τ 비대칭 | TP/FP/FN 대칭 팽창 정의로 교체 |
| GT 격자 충돌 박스·0.20 m | visual 지오메트리(gpu_lidar 확인), 지면 +0.38 m 단면, visual≡collision 검사 기록 |
| 대칭 통로 조용한 납치 검출 주장 | 철회, 범위 한정 문단 추가(§4.B) |
| CUSUM δ = 0.15, h = 1 오경보 | 동적 빔 마스크 + f_t 통계 추가, (δ, h)는 S6 데이터 ARL 보정으로 결정·측정치 보고, 디바운스 대비 이점 설명·절제 비교 |
| Q_param ↔ ΓQ_cΓᵀ 율 의존 | CWNA 모델 채택·이벤트 스케줄 예측, Δt = 0.02 s에서만 등가임을 명시. **감사 시 보완**: x, y 블록 "무시 가능" 주장 철회, 팀 EKF에 선택 Q_pos 추가 |
| NEES 구간 | 런별 [0.83, 12.8], 10런 평균 [3.24, 7.14] |
| δ_rot,noise 표기 | 운동 모델 식 수정 |
| odom→base_link vs base_footprint | base_footprint로 통일(EKF 두 인스턴스·AMCL base_frame_id·TF 합성) |

**제안 판정 반영**: IAG-EKF(flawed) → 네 결함 모두 수정, 선행(Reina 2006, Yu 2023 χ², FusionCore 2026) 인용·variant_of_prior로 위치, 효과는 예측치로 격하. SMC-CUSUM(overclaimed) → 대칭 납치 주장 철회, 마스크·보정 절차 추가, Akai/Bukhori/Campbell 인용, new_combination은 "스캔-맵 인라이어 비율 CUSUM"의 좁은 의미로만. 능동 복구 FSM(flawed) → 수치 전면 재유도, 자유공간 시드 자체는 nav2 기존 기능임을 인정하고 기여를 스코어링·정련·순차 검증으로 한정, engineering_adaptation.

**스펙 공백 보완**: IMU 바이어스 보정 노드·상태(§2.2, §7) · 루프 폐합 SPA 목적함수·Huber·수락 규칙·M2 실험(§2.5, §6) · 4시간 S8 · 통합 테스트 IT-01…13 · TF base_footprint · 공칭 드리프트 S9와 예측치(§2.1) · 스펙 EKF 예시와의 차이 문서화(§2.2) · 열린 질문 3, 4를 설계 결정으로 전환(§1, §8) · 다중 로봇 상호 동적 장애물(마스크 대상)·노드별 CPU 예산(§7).

**인용 수정**: Ref 20 CPU 70 %를 slam_toolbox의 비용으로 정정(Semantic Scholar 초록 확인) · Filatov 2017 지표 3종을 PDF 본문에서 확인해 정정, 직각 편차는 팀 정의로 표기 · pf.c 리샘플링 "소스 확인" 오류 정정 · 신규 선행 6건 추가(24–29), 조회 수준을 각각 표기(Reina·Fox는 내용 RECALLED).

**리뷰어와 다른 입장(근거 포함)**: (1) 복구 시 max_particles ≥ 50k–100k 권고는 **채택하지 않음** — 같은 리뷰가 확인한 O(N²) 선형 리샘플러에서 N = 100k는 리샘플당 ≈ 5×10⁹ 비교(수 s)로 회전 중 갱신을 따라가지 못한다. 상한 20 000(≈ 0.3 s/리샘플) + σ_hit 확장 + 가설 시드 우선으로 대체했고, 기대 적중 계산을 그 값으로 제시했다. (2) Bukhori & Ismail 2017은 리뷰가 말한 "likelihood-field 기반"이 아니라 초록상 "자연스러운 변위" 판정 방식이므로 그렇게 인용했다. (3) Campbell & Whitty는 Semantic Scholar 서지상 RAS 2015(온라인 2014)로 표기했다.

### 10.1 감사(audit) — 리뷰 항목별 해소 검증과 추가 수정 (2026-09-22)

개정본을 critique.json의 **51개 항목**(수학 오류 19 · 제안 판정 3 · 스펙 공백 9 · 인용 문제 6 · 필수 개정 14) 전부와 대조했다. 수학 항목은 `checks/`의 스크립트로 수치를 다시 유도했고(audit_numbers.py, iag_residual_mc.py, seed_sim.py, seed_pipeline_sim.py, seed_modes_sim.py), 인용·소스 항목은 다시 조회했다(nav2_amcl 1.1.20 태그 pf.c·amcl_node.cpp·likelihood_field_model.cpp·differential_motion_model.cpp, slam_toolbox 2.6.10 태그 ceres_solver.cpp·Mapper.cpp, robot_localization humble-devel ekf.cpp·filter_base.cpp·ros_filter.cpp, gz-sim6 UserCommands.cc, Electronics 초록(Semantic Scholar API), Filatov PDF 본문, FusionCore arXiv 초록, Crossref 서지 5건). 개정자가 **맞게 해소한 항목 28개**는 수치·소스로 확인했고, **해소가 틀렸거나 불완전했던 23개**(M3, M6, M7, M9, M16, M19, P1–P3, G1, G2, G5, G6, G7, G9, C5, R1, R2, R4, R5, R9, R10, R11)는 아래처럼 고쳐 **51/51 해소**로 마감했다.

| 항목(critique) | 개정본의 문제 | 감사 수정 |
| --- | --- | --- |
| M6 SEED · 판정 P3 · 개정 R4 | 1 m × 15° 격자가 "캡 평균거리 분지 안이라 적중이 결정적"이라는 주장은 틀림(10–20 m 빔은 7.5°에 1.3–2.6 m 이동 → 캡 포화). 합성 주기 창고에서 참 포즈가 top-20 fine 창에 든 비율 **8/25**. 다봉을 "회전으로 3가설 순차 검증"하는 분기는 360° LiDAR에서 별칭을 기각할 수 없음. 가림 10 %에서 참 포즈의 캡 평균 0.12 m > m_ok 0.10(참 포즈 기각) | 0.5 m × 5°·top-50(적중 22–23/25), fine 단계 80 % 트리밍 m̃로 순위, 단봉 수락은 m̃·마스크 점수 m̂ 이중 여유, 다봉은 **DISAMBIGUATE**(모드 odom 추적 + `drive_on_heading` 이동), GLOBAL 결과도 모드 대조; 시간 예산·S4 목표(60 s 내 ≥ 90 %, 오수렴 0, 30 s 비율 분리 보고)·IT-06 재설정 (§4.C, §6, §8-6) |
| M9 IAE · 판정 P1 · 개정 R5(c) | R̂_ωω = Ĉ_{r_ω} − σ_g²/20(창 잔차)은 창 평균의 1/10 축소를 되돌리지 않아 **R 12배 과소추정**(MC 0.083배); ×10 보정해도 중첩 창 자기상관으로 0.83배·상대오차 43 %("20 %" 주장과 불일치) | 표본별 잔차 e_ω, e_v로 추정(MC: 편향 없음, 상대오차 20 %/25 %), R_0 정의, 게이트 판정 지연(≤ 0.2 s)과 r_l/팀 EKF 반응 차이 명시 (§4.A) |
| M7 가속도 잔차 · 개정 R5(a) | 창 잔차 σ(0.020 m/s, 0.025 rad/s)는 맞으나 "3σ 검출 한계 … 0.025 rad/s"는 1σ 값 | 게이트 √13.8 = 3.7σ 기준 0.074 m/s / 0.092 rad/s(2 m/s), 속도 의존 표기; χ²₂ 가정 MC 확인(P(s > 13.8) = 0.0009–0.0012); 스탬프 정렬 ≤ 2 ms 전제 (§4.A) |
| M16 Q 율 의존 · 개정 R9 | "x, y, θ 블록 차이는 Q_param 0.001로 작게 두어 무시" — r_l 위치 랜덤워크는 0.1 s에 1 cm, 1 s에 3.2 cm로 AMCL 하한(2 cm)과 같은 차수 | 비등가 명시, 팀 EKF 선택 항 Q_pos(= r_l 값)로 A/B 분리, ekf_map q_p 스윕 {10⁻³, 10⁻², 0.05} (§2.2, §6, §7) |
| M3 리샘플링 · 공백 G9 | §2.4 "15회 ≈ 5 s CPU" vs §4.C "30회 = 10 s" 모순; 1.0 rad/s × 10 Hz = 0.1 rad/스캔이 엄격 부등호 `update_min_a 0.1`에 걸려 갱신 누락 가능 | 단일 스레드 포화·처리 갱신 20–35회·버스트 6–10 s로 통일; ROTATE 중 `update_min_a 0.05`(재초기화 없는 동적 파라미터, 소스 확인); P ≥ 0.9에 필요한 N ≈ 53 000 불가 근거 추가 (§2.4, §4.C, §7) |
| 공백 G6 드리프트 | σ_D = k_s√(Δs·D)는 √2 과대(0.014 → 0.010 m); 100 m(100 s)에 60 s 헤딩 값을 써서 "5 cm" | 3.5 cm(T_cal 10 s) / 2.0 cm(60 s), 20 m 측면 0.4 cm(상한 0.8) (§2.1, §5, §6 S9) |
| 공백 G2 · 개정 R11 루프 폐합 | Huber "δ = 1"은 틀림 — slam_toolbox 2.6.10은 `HuberLoss(0.7)` 하드코딩; 잔차를 SE(2) log로 적었으나 구현은 i-프레임 상대 위치 차 + 각도 차; 수락 부등호 | `PoseGraph2dErrorTerm` 형태·a = 0.7·`TryCloseLoop` 조건으로 정정 (§2.5) |
| 공백 G1 · 개정 R1 IMU 바이어스 | 노드·토픽을 `imu_preprocess_node`, `/imu/data` → `/imu/data_calibrated`로 새로 만들어 계약(`imu_filter_node`, `imu/data_raw` → `imu/data`, `bias_estimation_time` 60 s)과 충돌 | 계약 이름으로 정렬, 팀 EKF b_g = 잔차 바이어스(초기 0), 60 s 값 병기 (§1, §2.2, §7) |
| 공백 G7 스펙 EKF 차이 | a_x, a_y 미융합 근거(0.10 m/s² 바이어스)는 보정된 `imu/data`에서 소멸, repo ekf.yaml의 구심항 실측 근거와 충돌; ekf_odom 초기 공분산 10⁻²는 odom 원점 정의와 모순; twist 게이트 3.7은 repo `odom0`(vx, vy, vyaw, m = 3)에 맞지 않음 | r_l 기준선은 ax, ay 융합 유지, 초기 공분산은 ekf_map만, twist 게이트 4.0 (§2.2, §7) |
| M19 · 공백 G5 · 개정 R10 TF | base_footprint 통일은 맞으나 r_l 쌍과 팀 EKF 쌍이 동시에 TF를 발행할 위험, 두 r_l 노드의 `set_pose`(토픽·서비스) 이름 충돌, `pose0_rejection_threshold`가 repo ekf.yaml의 "게이트 없음" 근거와 충돌 | `ekf_impl` 인자로 TF 발행 단일화(섀도 모드), 인스턴스별 `set_pose` 리맵, 복구 경로의 set_pose로 충돌 해소 설명 (§2.3, §7, §8) |
| 개정 R2 α·σ_hit | 공동 스윕 격자에 초기점(α 2×10⁻³, σ_hit 0.05)이 없음 | 5 × 4 격자 (§2.4, §6) |
| 인용 C5 · 판정 P2 | 판정문이 든 Zhang–Zapata–Lépinay(Robotica 2012) 누락, Campbell–Whitty DOI/쪽 미기재, Akai JFR판 미조회 | 표 30 추가(서지 Crossref, 내용 RECALLED), Campbell RAS 69:40–51·doi, Akai JFR 40(3):595–613 확인 (§3, §9) |
| 인터페이스 정합(과제 지시) | `kidnap_monitor_node`를 계약(Python, `localization/lost`)과 다르게 C++·`/localization/health`로 정의, `/scan` 구독(AMCL은 `scan_filtered`), 절대 이름 사용 | 상대 이름·계약 토픽으로 정렬, `localization/lost` 유지, 계약 변경 요청 목록(§7)과 §8-10 신설 |

**재확인만 한 항목(개정 정확)**: M1(바이어스 수치 0.6 rad/60 s, R = 4×10⁻⁸), M2(α 분산 단위: α3,phys 5×10⁻⁵, κ² 범위 1.25–5×10⁻³, 0.05 = 32배), M4(σ_map 0.0376, σ_hit 0.0502), M5(기대 적중 0.003 / 0.216 / 0.864, P 58 %), M8(교차항; MC 상관 ≤ 0.18), M10(q_v,gated 0.2 → (0.2 m/s)²), M11(σ_hit/√3 = 2.9 cm; `p += pz³` 소스 확인), M12(IoU_τ), M13(visual·0.38 m), M14, M15(가림 15 %, g = 3), M17(χ² 구간 [0.83, 12.83], [3.24, 7.14]), M18(`delta_rot*_noise` 소스 확인), KLD n(363/750/2484), 인용 C1(Electronics 초록: Cartographer 80 %·299 MB·튜닝 민감까지 확인), C2(Filatov PDF: Proportion·Corner Count·Enclosed Areas), C3(1.1.20 태그 pf.c 372행 "Naive discrete event sampler"), C4(amcl_node.cpp 1148–1320행, UserCommands.cc 681행), C6; 스펙 공백 G3(S8 4시간), G4(IT-01…13), G8(열린 질문 3·4 해소); 필수 개정 R3(O(N²) 서술), R6(§4.B 마스크·보정·대칭 철회), R7(선행 문헌 위치·novelty 격하), R8(IoU_τ·GT 격자·√3 폭), R12(S8·S9·통합 테스트), R13, R14(ref 20·Filatov).

**리뷰어와 다른 입장의 재검토**: (1) 복구 시 50k–100k 파티클 불채택 — 감사 계산으로 지지(P ≥ 0.9에 N ≈ 53 000 필요, 리샘플당 1.4–2.8 s). 다만 그 대신 둔 SEED 경로의 원래 근거는 틀렸고 위와 같이 교체했다. (2), (3)은 Crossref로 확인되어 유지.

### 10.2 재개 감사(사용량 한도로 중단된 감사의 마감 자체 점검, 2026-09-22)

§10.1 감사는 문서를 쓴 직후 자체 점검 전에 중단됐다. 51개 항목 전부를 다시 대조하되, 이번에는 **§10.1이 새로 들인 주장**(소스 해석·수치·인용 판정)을 독립 검증 대상으로 삼았다.

**독립 재검증(변경 없음)**: nav2_amcl 1.1.20(`amr-fleet-system:wf-final` 컨테이너에서 nav2_amcl 1.1.20 / robot_localization 3.5.4 / slam_toolbox 2.6.10 재확인) `pf.c` 372행 "Naive discrete event sampler"·low-variance 주석 비활성(M3·C3·R3), `dynamicParametersCallback`의 reinit 분류(sigma_hit·laser_likelihood_max_dist·z_*·max_beams → reinit_laser, max/min_particles·pf_err·pf_z·recovery_alpha_* → reinit_pf, update_min_a/d·resample_interval 즉시 반영; G8·C4·R13), `shouldUpdateFilter` 엄격 부등호, `LikelihoodFieldModel` 생성자의 `map_update_cspace`·`p += pz³`(M11), `SetInitialPose`·`request_nomotion_update` 서비스, Nav2 `DriveOnHeading` behavior 플러그인 존재(§4.C DISAMBIGUATE 전제); slam_toolbox `TryCloseLoop` 부등호·`HuberLoss(0.7)`·`PoseGraph2dErrorTerm`·기본 파라미터 표(G2·R11); robot_localization `P += Δt·Q`(ekf.cpp 427–430행), n_σ² 비교(filter_base.cpp 437행), `set_pose` 상대 이름(ros_filter.cpp 1057–1065행; M16·M19·G5·R10). 수치 재유도(checks/audit_numbers.out 대조): M1·M2·M4·M5·M17·G6·KLD, §4.A σ_r·임계·MC, §4.C 시뮬레이션 코드(fine 창 판정·클러스터 억제·트리밍) 검토. 인용 재조회: FusionCore arXiv 2605.25239 초록(센서별 χ² 게이팅·innovation 기반 R 적응·자이로/가속도 바이어스 상태·23-상태 UKF 문구 확인; C5·P1·R7), Akai arXiv 2205.04769 초록(신뢰도 추정 + importance sampling 재정위; P2), Filatov PDF 본문(Proportion / Corner Count / Enclosed Areas, 직각 지표 없음 — 참고로 WebFetch 요약기는 "Completeness/Correctness/Consistency·직각 지표"라는 **환각 요약**을 내 PDF 원문으로만 판정; C2·R14), Electronics 초록 원문(Semantic Scholar API; C1·R14).

**§10.1 해소에 남아 있던 오류와 수정(재개 감사)**

| 관련 항목 | 문제 | 수정 |
| --- | --- | --- |
| M7·M8·R5(a,b)·판정 P1 (§4.A 슬립 검정의 χ²₂ 보정) | χ²₂ 보정(P(s > 13.8) ≈ 0.001)은 인코더–IMU 시각 정렬을 전제하는데, 브리프 자신의 `imu_filter_node` LPF(2차 Butterworth 20 Hz @ 100 Hz)는 군지연 **9.7 ms**를 만든다. r_ω 편향 δ·ω̇가 제자리 회전 램프(σ_rω ≈ 0.0033, 하한 지배)에서 5.8–6.2σ → **램프마다 오게이트(P ≈ 0.99)**. 또 원문 "1 m/s² 가속 중 5 ms → r_v 0.005 m/s"는 틀림(창 차분이라 등가속에서 편향 0, 편향 = δ·Δa) | §4.A에 편향식·허용치(\|δ\| ≤ 1.6 ms)·수치(2 ms → 1.2σ/오발 1.1–1.3 %, 5 ms → 3σ/28–34 %) 명시, `imu_filter_node` 출력 스탬프 τ_LPF 보상(잔여 ≤ 0.7 ms)·구간 적분 정렬 규정, gtest "회전 램프 오발 ≤ 0.2 %" 추가(§4.A, §7, §8-12; checks/audit_resume.py) |
| (과제 지시: config 정합) §5 회전 예산 | LPF 지연이 EKF 헤딩을 τω만큼 늦춰 odom 프레임에 v·τ·Δθ = 1.5 cm(1 m/s, 90°) / 3.1 cm(2 m/s) 오차 — 예산표에 없음 | §5에 행 추가(보상 후 ≈ 0.2 cm 이하) |
| (과제 지시: ekf.yaml 정합) G7·§7 기준선 | repo ekf.yaml 주석의 `wheel_odom.twist.covariance` = 10⁻⁴(k_s를 절대 σ로 읽음)는 §2.1 모델 대비 vyaw 15배(1 m/s)·62배(2 m/s) 과소; 브리프는 B0/B1의 "상수 공분산"이 무엇인지, vy(repo가 융합) 분산을 정의하지 않음 | B0/B1 = 주석값 diag(10⁻⁴ ×3), Ours = Σ_enc + IAG, vy 10⁻⁴ 고정, 절제군 "Σ_enc만" 추가(§6, §7) |
| G8·C4·R13 (열린 질문 3 해소 근거) | "`init_pose_`는 `initOdometry()`(활성화·α 변경 시)" — `initOdometry()`는 `on_configure`와 α1–5/`robot_model_type` 변경 시 호출되며 `on_activate`에서는 호출되지 않음(amcl_node.cpp 249·1352행) | 호출 위치 정정; 파라미터 묶음은 `set_parameters_atomically`로 보내 `reinit_pf` 1회(비원자 호출은 파라미터마다 콜백) — FSM GLOBAL/VERIFY·§7 클라이언트 이름 반영 |
| G2·R11 (루프 폐합 설명) | 일관성 지표 χ²/dof의 분모 3\|E\| − 3\|V\|가 게이지(첫 노드 고정) 3을 빠뜨림 | 3(\|E\| − \|V\| + 1) (§2.6) |
| C1·R14 (ref 20) | 초록 원문이 CPU 70 %(Toolbox) < 80 %(Cartographer)인데 Toolbox를 "더 높은 연산 요구"로 기술 — 브리프 요약은 정확하나 이 내부 불일치를 밝히지 않음 | §3 표에 명시, CPU는 선택 근거로 쓰지 않고 S7에서 측정 |
| (과제 지시: components.md 정합) 계약 변경 요청 (2) | `kidnap_monitor_node`가 계약의 `odometry/filtered` 대신 `odometry/filtered_map`을 구독하는 **교체**를 "확장"으로만 적음 | 유지·교체·추가 항목을 나눠 명기(§7) |

**판정**: critique.json 51개 항목(M 19 · P 3 · G 9 · C 6 · R 14) 모두 문서에서 해소됨을 확인했다. §10.1이 잔여 오류를 남긴 5개 항목군(M7/M8/R5·P1, G7, G8/C4/R13, G2/R11, C1/R14)과 과제 지시상의 정합 2건(§5 LPF 지연 예산, 계약 변경 요청 (2))을 위와 같이 고쳤다. 리뷰어와 다른 입장 3건(§10 끝)은 유지한다.
