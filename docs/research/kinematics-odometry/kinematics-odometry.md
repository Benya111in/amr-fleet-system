# 설계 브리프 — 차동구동 기구학 · 휠 오도메트리 · 드리프트 분석 · 센서 전처리 (스펙 4.2)

- 작성일: 2026-09-21, 개정 v2: 2026-09-22 (적대적 리뷰 반영), **개정 v3: 2026-09-22 (감사: 리뷰 항목 전수 재검증 + 프로젝트 설정·인터페이스 계약 정합)**, **v3.1: 2026-09-22 (감사 재개 — 독립 재유도 `c9` 로 자체 점검, §8.3)** / 담당 영역: kinematics-odometry
- 대상 스택(컨테이너 `amr-fleet-system:wf-final` 에서 v3 재확인): ROS 2 Humble, Nav2 1.1.20, robot_localization 3.5.4, laser_filters 2.0.9, Gazebo Sim 6.18.0 (Fortress, `libignition-gazebo6-wheel-slip-system` 포함), ros_gz_bridge 0.244.26, PCL 1.12 헤더, Eigen3
- **정합 기준(v3)**: `config/robot_params.yaml`(r, b, 질량), `config/sensors.yaml`(엔코더·IMU·깊이 잡음, extrinsic), `config/ekf.yaml`(dual EKF), `docs/architecture/components.md` §3·§5(노드·토픽 이름, 주기), `docs/architecture/sensor_calibration.md`. 토픽·프레임은 components.md 표기대로 **로봇 네임스페이스 상대 이름**(`/amr_01/wheel_odom` → `wheel_odom`, 프레임 접두사 `amr_01/`).
- 웹 도구 사용. 인용 표기: **VERIFIED**(이 세션 또는 이전 세션에서 초록·서지·소스·전문을 직접 fetch) / **RECALLED**(고전 문헌, 기억 기반). 모든 수치는 `checks/c1–c9*.py` 로 재유도(§9; `c9` 는 c1–c8 코드와 독립인 폐형 재계산 33 항목).

---

## 0. 요약 (TL;DR)

1. **베이스라인**: 4096-tick 엔코더(50 Hz `joint_states`) → 차동구동 순기구학(중점 적분, 원호해 대비 상대 오차 ≤ Δθ²/24 = 3.75×10⁻⁵) → 자세·파라미터 **증강 공분산(5×5)** 전파 + `nav_msgs/Odometry` 공분산 계약(§2). 휠 잡음 = `sensors.yaml` 곱셈 슬립 잡음(σ_s = 0.01/주기) + 물리 슬립 `k_phys` + 양자화. 양자화는 **트위스트에만** 백색으로, 자세에는 **유계 항 1회**(MA(1) 텔레스코핑).
2. **시뮬레이터 사실**(VERIFIED): Gazebo `DiffDrive` 는 잡음·공분산 없는 오도메트리 + TF 를 자체 발행 → 운용 시 브리지하지 않는다(평가 GT 는 `OdometryPublisher` → `ground_truth/odom`). IMU `orientation` 은 참값 → `orientation_covariance[0] = −1`, EKF `imu0` yaw 미융합(ADR-KO-01; `config/ekf.yaml` 과 동일). `WheelSlip` 의 `wheel_normal_force` 는 **고정 SDF 파라미터**, 런타임에는 `/world/<w>/wheel_slip` 서비스로 compliance 만 바꾼다(§2.2). 깊이 카메라 네이티브 잡음은 **상수 σ 가우시안**뿐이라 거리 제곱 항은 `pointcloud_filter_node` 가 가산(§3.2, `sensors.yaml` 결정).
3. **독자 알고리즘(변형으로 정직하게 위치 지정)**: (a) **SACO** — 엔코더-자이로 각속도 잔차(비대칭 슬립) + **정렬된 창의 적분 속도 잔차** `Δv_enc − ∫a_imu`(대칭 슬립; 검출 후 정상 슬립은 누적 잔차로 유지)의 2채널 χ² 게이트로 `wheel_odom` **트위스트 공분산만** 팽창(메시지 자기일관). 선행: De Giorgi 2023 식 (8)/(10)(전문 VERIFIED), Reina 2006, Borenstein–Feng 1996, rouf-rimon 공개 구현. (b) **GRC** — `ψ=[r_R/b, r_L/b]`(Antonelli 2005/2007 의 c21/c22)를 자이로 기준 칼만형 RLS 로 온라인 추정. v3 에서 **v2 안전장치의 교착(갱신 제한 + 혁신 게이트)과 errors-in-variables 편향을 수치로 발견·수정** → 2-모드(커미셔닝/추적) + EIV 편향 보상 + CUSUM 동결(§4.3).
4. **평가**: 모든 정확도 실험은 스펙 로그 포맷으로 **map EKF 출력 `odometry/filtered_map` 의 RMSE + 최대오차**(GT `ground_truth/odom`)를 보고, odom EKF·원시 오도메트리 지표는 부가. 주지표는 EKF NEES 일관성·슬립 시 위치오차·AMCL 점프, 4 h 내구(T9), 통합 시나리오 8건(§6).

---

## 1. 스펙 요구사항 재정리 (4.1/4.2 및 연계 항목)

| 항목 | 스펙 | 처리 |
|---|---|---|
| 순기구학 직접 구현 / 엔코더 4096 tick + 슬립 잡음 | 필수 | §2.1, §2.2 |
| 누적 오차 분석·드리프트 리포트 | 필수 | §2.4, §5 T1–T3 |
| LiDAR/Depth/IMU 전처리 | 필수 | §3 |
| EKF 입력 공분산 근거 문서화 | 필수 | §2.3, §3.3, §4 |
| TF 트리 `map→odom→base_footprint→base_link→센서` (스펙 예시) | 필수 | §2.5 (dual EKF 발행자 확정) |
| 위치 추정 정지 3 / 직진 5 / 회전 8 cm, **RMSE + Maximum Error**, `[timestamp, gt_x, gt_y, est_x, est_y, error]` ≥ 100 샘플 | 정량 | §5 (T1·T2·T3·T6 모두 `odometry/filtered_map` 기준) |
| 센서 캘리브레이션 절차 문서 + extrinsic 설정 파일 | 필수 | extrinsic = `config/sensors.yaml`(유일 원본), 절차 = `sensor_calibration.md`; 본 영역은 내재 파라미터(§5 T3)·IMU(§3.3) |
| 4 h 연속 운용, 통합 시나리오 ≥ 10, 커버리지 ≥ 70 % | 필수 | §5 T9, §6 |

**파라미터(프로젝트 설정 값; v1/v2 의 가정값 r = 0.075, b = 0.35, 100 Hz 를 대체)**: `r = 0.0825 m`, `b = 0.36 m`(`robot_params.yaml`), `N = 4096 tick/rev`, `joint_states`·`wheel_odom` **50 Hz**(`Δt_e = 0.02 s`, components.md §5.1–5.2), 슬립 잡음 `σ_s = 0.01`(주기당 곱셈, `sensors.yaml wheel_encoder.slip_noise_stddev`), IMU 100 Hz `σ_g = 2×10⁻⁴ rad/s`, `σ_a = 0.017 m/s²`(`sensors.yaml imu`), 잔차·RLS 창 `T_w = 0.1 s`(엔코더 5 스텝, IMU 10 샘플, 10 Hz), EKF 50 Hz(dual). 공칭 `ψ_nom = r/b = 0.229`.

---

## 2. 베이스라인 수학 — 순기구학 · 양자화 · 공분산 전파

### 2.1 순기구학

휠 각변위 `Δφ_i = 2πΔn_i/N` [rad], `i ∈ {L, R}`:

$$\Delta s_i = r_i\,\Delta\phi_i,\qquad \Delta s = \tfrac12(\Delta s_R+\Delta s_L),\qquad \Delta\theta = \frac{\Delta s_R-\Delta s_L}{b}\quad[\mathrm{m}],[\mathrm{rad}]$$

중점(2차) 적분, `φ = θ_k + Δθ/2`:

$$x_{k+1}=x_k+\Delta s\cos\varphi,\qquad y_{k+1}=y_k+\Delta s\sin\varphi,\qquad \theta_{k+1}=\theta_k+\Delta\theta$$

정확한 원호(ICR)해는 `Δx = Δs·sinc(Δθ/2)·cosφ`, `Δy = Δs·sinc(Δθ/2)·sinφ` (`sinc u = sin u/u`) 이므로 중점식의 단계당 오차는 현(chord) 방향으로 정확히

$$e_{step} = \Delta s\,[1-\operatorname{sinc}(\Delta\theta/2)] \le \Delta s\,\frac{\Delta\theta^2}{24}$$

(`1 − sinc u ≤ u²/6`). 즉 **상대 스케일 오차 `Δθ²/24`**: 1.5 rad/s·50 Hz(Δθ = 0.03 rad)에서 `≤ 3.75×10⁻⁵`, 2 m/s 에서 단계당 `≤ 1.5×10⁻⁶ m`, 최악 가정(1 h 내내 최대 속도·최대 각속도 = 7.2 km)에도 `≤ 0.27 m`. 평균 반경 0.5 % 스케일 오차(7.2 km 에서 36 m)와 비교하면 무시 가능 → 중점식 채택(원호식 `integrate_arc()` 도 O(1) 로 제공). 단위 테스트: 원호해와 비교하여 `|e_step| ≤ Δs·Δθ²/24` 를 (Δs, Δθ) 격자에서 검증(c1: 최대 비율 1.000).

트위스트(`base_footprint` 기준, **발행 주기 = 단계 주기** `T = Δt_e`): `v_x = Δs/T`, `v_y = 0`, `ω = Δθ/T`. 2 m/s 에서 휠 각속도 24.2 rad/s → 15,800 tick/s(int64 카운터, 4 h 에 2.3×10⁸ tick). 복잡도 O(1)/step, < 1 µs.

### 2.2 엔코더 양자화 및 슬립 모델

- tick 분해능 `δ = 2πr/N = 1.27×10⁻⁴ m`(`sensors.yaml` 주석 0.127 mm 와 일치). 누적 카운트 양자화 오차 `q_k ~ U(−δ/2, δ/2)`, `σ_q² = δ²/12 = 1.3×10⁻⁹ m²` — **누적 카운트에서는 유계**.
- 단계 차분 `Δn_k` 의 오차 `e_k = q_k − q_{k−1}` 는 **MA(1)**: `Var(e_k) = δ²/6`, `Cov(e_k, e_{k+1}) = −δ²/12`(c1 시뮬레이션: 1.004, 0.997 배). 임의 구간의 합은 `q_end − q_start` 로 텔레스코핑 → **자세 전파에 δ²/6 을 매 단계 백색 잡음으로 더하면 안 된다**(그렇게 하면 1 h(50 Hz, 1.8×10⁵ 스텝) 후 휠당 `4.8×10⁻⁴ m²`, σ ≈ 2.2 cm 의 가짜 랜덤워크; 실제 헤딩 양자화 오차는 시작·끝 카운트 오차 합으로 `|Δθ_q| < 2δ/b = 7.0×10⁻⁴ rad` 유계). 자세 공분산에는 §2.3 의 유계 항을 **1회** 더한다.
- 차분을 실제로 취하는 **트위스트**에서는 백색 근사가 정당하다(음의 lag-1 상관을 무시하므로 보수적). 구간 `T` 1차 차분: `Var_q(v_i) = δ²/(6T²)`; 2차 차분(가속도)은 중간 카운트를 공유하므로 `Var_q(a_i) = 6σ_q²/T⁴ = δ²/(2T⁴)`(c1: 5.98σ_q²).
- 양자화 속도 잡음: 발행 트위스트(`T = 0.02 s`) `σ_{v,q} = δ/(√12 T) = 1.8×10⁻³ m/s`, `σ_{ω,q} = δ/(√3 b T) = 1.0×10⁻² rad/s`; 잔차 창(`T_w = 0.1 s`) `3.7×10⁻⁴ m/s`, `2.0×10⁻³ rad/s`.
- **비체계(non-systematic) 잡음 = 설정의 곱셈 슬립 잡음 + 물리 슬립.** `sensors.yaml`: 주기마다 `Δs_i ← Δs_i(1 + N(0, σ_s²))`, `σ_s = 0.01` → 단계 분산 `σ_s²Δs_i²`. 일정 속도에서 이는 Chong–Kleeman(1997, DOI VERIFIED) 의 `Var ∝ 이동거리` 모델과 같은 꼴이며 등가 계수는 `k_eff = σ_s²|Δs_{i,step}| = σ_s²|v_i|Δt_e`(1 m/s 에서 2×10⁻⁶ m, 2 m/s 에서 4×10⁻⁶ m). Gazebo 접촉 물리에서 오는 잔여 슬립은 `k_phys|Δs|` 로 더하고 T1 에서 식별(사전값 0):

$$\operatorname{Var}(\Delta s_{i,step})=\sigma_s^2\Delta s_{i,step}^2+k_{phys}|\Delta s_{i,step}|\ (\text{자세}),\qquad
\sigma_i^2(T)=\sigma_s^2|\Delta s_{i,step}|\,|\Delta s_{i,T}|+k_{phys}|\Delta s_{i,T}|+\tfrac{\delta^2}{6}\ (\text{구간 }T\text{ 차분}),\quad [k]=\mathrm{m}$$

  `Δs_{i,T} = v_i T`. 수치(2 m/s, `T_w = 0.1 s`, `k_phys = 0`): 휠 속도 슬립항 σ = 8.9×10⁻³ m/s ≫ 양자화 5.2×10⁻⁴ m/s → **주행 중 잡음은 슬립 항이 지배**.
- 체계(systematic) 오차: `ε = (r_R − r_L)/r̄` → 직진 곡률 `ε/b`, 거리 `L` 후 헤딩 `Lε/b`, 횡오차 `L²ε/(2b)`; `δb/b` → 회전각 스케일 `−Δθ·δb/b`. 예(b = 0.36 m): `ε = 0.5 %` → 20 m 에서 0.28 rad, 2.8 m; **잔여 `ε = 0.1 %` 라도 0.056 rad, 0.56 m**(결정론적, `L²` 성장). 시뮬레이션에서는 URDF = 공칭이므로 `ε = 0`(T3 에서 의도적으로 주입). → 캘리브레이션(§4 GRC, §5 T3)이 1순위이며, 오도메트리 단독으로 5 cm 는 불가능하고 AMCL 보정이 전제다.
- **Gazebo Fortress `WheelSlip`(gz-sim6 `WheelSlip.cc` VERIFIED, v3 에서 소스 재확인)**: SDF `slip_compliance_lateral/longitudinal`(무차원), `wheel_normal_force` [N], `wheel_radius`. `force = params.wheelNormalForce`(**고정 파라미터, 접촉력 측정값 아님**; L276), `slip1/2 = (r|ω|)/force × compliance`(L305–306)를 ODE force-dependent-slip 계수로 설정 → 실제 슬립 속도 = `slip × 실제 마찰력`. 따라서 **적재 증가 → 실제 접촉력 증가 → `wheel_normal_force` 를 갱신하지 않으면 슬립 속도가 커진다**(물리 직관과 반대). 런타임 갱신: `UserCommands` 의 서비스 `/world/<world>/wheel_slip`(`gz.msgs.WheelSlipParametersCmd`, L751)가 `WheelSlipCmd` 컴포넌트를 설정하고 `WheelSlip` 이 이를 소비(L251–272) — **compliance 만** 바꿀 수 있고 `wheel_normal_force` 는 바꿀 수 없다.
  **T4 슬립 에뮬레이션 프로토콜**: (i) 적재 0/2/10/25 kg(`robot_params.yaml payload`) 각각 SDF 변형의 `wheel_normal_force` 를 **구동륜 1 개의 정적 수직하중**으로 둔다. 상한은 `(47.6 + m_p)·g/2 = 233/243/282/356 N`(캐스터 분담 0 가정, g = 9.8)이며, 캐스터가 하중을 나누므로 실제 값은 정지 상태 접촉력으로 1회 측정해 넣는다. (ii) 저마찰 패치는 월드 지오메트리 대신 로봇이 패치 영역(GT 위치 기준 2×2 m)에 들어갈 때 `wheel_slip` 서비스로 compliance 를 ×10–×50 올리고 나갈 때 복원. (iii) "적재 반영 누락"(F_N 미갱신) 조건을 별도 ablation 으로 남겨 적응 공분산의 동기를 실험적으로 보인다.
- **tick 에뮬레이터**(스펙 "슬립 노이즈 적용", `wheel_odometry_node` 내부): `n_i = ⌊φ_i N/(2π)⌉`(int64, 언랩) + **`sensors.yaml` 곱셈 슬립 잡음 기본 활성**(`σ_s = 0.01`). 이는 측정 수준의 비체계 잡음이고 `WheelSlip` 은 로봇이 실제로 미끄러지는 물리 슬립(T4 패치·적재 조건)이라 서로 다른 현상이다(이중 계상 아님). v2 의 "기본 0" 은 설정과 모순되어 삭제.

### 2.3 공분산 전파 — 자세(증강) 와 트위스트(EKF 계약)

**자세 공분산.** 파라미터 오차는 단계 간 **완전 상관**이므로 매 단계 `F_ρΣ_ρF_ρᵀ` 를 백색으로 더하면 `Var(θ) ∝ L` 이 되어 실제(`∝ L²`)를 과소평가한다. 상태를 `s = [p; ψ]`, `ψ = [ψ_R, ψ_L] = [r_R/b, r_L/b]`(무차원) 로 증강하고 `b̄` 고정 하에 `Δs_i = ψ_i b̄ Δφ_i`, `Δθ = ψ_RΔφ_R − ψ_LΔφ_L` 로 쓴다. 입력 `u = [Δs_R, Δs_L]`, 잡음 `Σ_u = diag(σ_s²Δs_R² + k_phys|Δs_R|, σ_s²Δs_L² + k_phys|Δs_L|)`:

$$F_p=\begin{bmatrix}1&0&-\Delta s\sin\varphi\\0&1&\Delta s\cos\varphi\\0&0&1\end{bmatrix},\quad
F_u=\begin{bmatrix}\tfrac12\cos\varphi-\tfrac{\Delta s}{2b}\sin\varphi&\tfrac12\cos\varphi+\tfrac{\Delta s}{2b}\sin\varphi\\ \tfrac12\sin\varphi+\tfrac{\Delta s}{2b}\cos\varphi&\tfrac12\sin\varphi-\tfrac{\Delta s}{2b}\cos\varphi\\ \tfrac1b&-\tfrac1b\end{bmatrix},\quad
F_\psi=F_u\operatorname{diag}(\bar b\Delta\phi_R,\ \bar b\Delta\phi_L)$$

$$\Sigma_{pp}\leftarrow F_p\Sigma_{pp}F_p^{\top}+F_p\Sigma_{p\psi}F_\psi^{\top}+F_\psi\Sigma_{\psi p}F_p^{\top}+F_\psi\Sigma_{\psi\psi}F_\psi^{\top}+F_u\Sigma_uF_u^{\top},\qquad
\Sigma_{p\psi}\leftarrow F_p\Sigma_{p\psi}+F_\psi\Sigma_{\psi\psi}$$

GRC 가 `ψ̂` 를 갱신하면(§4.3, 이득 `K`, 회귀벡터 `h`) 오차가 `e_ψ⁺ = (I − Khᵀ)e_ψ⁻ − K e_z` 로 바뀌므로 **교차공분산도 함께 갱신**: `Σ_pψ ← Σ_pψ(I − Khᵀ)ᵀ`, `Σ_ψψ ← P_k`; 랜덤워크 예측 단계에서는 `Σ_ψψ ← Σ_ψψ + qI`(교차항 불변). 이 전파로 체계 잔여가 `Var(θ) ∝ L²`, `Var(y) ∝ L⁴` 로 성장한다(5×5, O(1)/step). **Monte-Carlo 검증**(c1: 직진 20 m, ψ 오차 0.1 %/휠 + 슬립 잡음, 4000 회): 증강 전파는 MC 의 `Var(θ)`·`Var(y)` 와 2 % 이내로 일치, 매 단계 백색화는 20 m 에서 `Var(θ)` 를 11 배 과소평가. 발행 시 양자화 유계 항을 **1회** 더한다: `Σ^p_pub = Σ_pp + F_u^{(0)}diag(δ²/6, δ²/6)F_u^{(0)⊤}`, `F_u^{(0)}` 는 `Δs→0` 인 `F_u`, `δ²/6` 은 휠당 누적 오차 `q_end − q_start` 의 분산. (시작 카운트 오차 `q_start` 가 남기는 상수 헤딩 오프셋 `σ = δ/(√6 b) = 1.4×10⁻⁴ rad` 은 위치로는 `L` 에 비례해 전파하지만 20 m 에서 2.9 mm 로 비체계 `σ_y`(0.29 m)의 1 % — 무시, c9.) 차원: `F_u` 의 [1]·[1/m] 열 × `Σ_u`[m²] → [m²],[rad²] ✓; `F_ψ` = [m]·[1]·[rad] → `Σ_ψψ`[1] 곱 → [m²] ✓.

**트위스트 공분산(`twist.covariance`, 6×6 row-major, x,y,z,rotX,rotY,rotZ — msg 정의 컨테이너 확인).** 발행 단계 `T = Δt_e` 차분, `σ_i² = σ_i²(T)`(§2.2):

$$\operatorname{Var}(v_x)=\frac{\sigma_R^2+\sigma_L^2}{4T^2}+h_v^{\top}\Sigma_{\psi\psi}h_v,\quad
\operatorname{Var}(\omega)=\frac{\sigma_R^2+\sigma_L^2}{b^2T^2}+h_\omega^{\top}\Sigma_{\psi\psi}h_\omega,\quad
\operatorname{Cov}(v_x,\omega)=\frac{\sigma_R^2-\sigma_L^2}{2bT^2}$$
$$h_v=\tfrac{\bar b}{2}[\dot\phi_R,\ \dot\phi_L]^{\top}\ [\mathrm{m/s}],\qquad h_\omega=[\dot\phi_R,\ -\dot\phi_L]^{\top}\ [\mathrm{rad/s}],\qquad
\operatorname{Var}(v_y)=\sigma_{vy,0}^2+(\kappa_y v_x\omega)^2\ ([\kappa_y]=\mathrm{s},\ \sigma_{vy,0}=0.02\ \mathrm{m/s})$$

차원: `m²/s²` ✓, `(rad/s)²` ✓, `[m/s]²·[1]` ✓. 수치(직진, `k_phys = 0`): 1 m/s 에서 `σ_vx = 7.3×10⁻³ m/s`, `σ_ω = 0.041 rad/s`; 2 m/s 에서 `1.4×10⁻² m/s`, `0.079 rad/s`. **`config/ekf.yaml` 주석과의 관계**: ekf.yaml 은 `wheel_odom.twist.covariance ← slip_noise_stddev 0.01 → σ² = 1e-4 (vx, vy, vyaw)` 를 상수 자리표시로 적고 값의 출처를 퍼블리셔로 위임한다. 위 식은 같은 `σ_s` 로부터 속도 의존 값을 낸다(`Var(v_x) = 1e-4` 는 v ≈ 1.4 m/s 에 해당, `Var(ω)` 는 1 m/s 에서 1.65×10⁻³ 로 상수값의 16 배) → ekf.yaml 주석 갱신을 상태추정 담당에 요청(§8 미해결). **정직한 한계**: robot_localization 은 매 메시지 공분산을 독립(백색)으로 취급하므로 `hᵀΣ_ψψh` 항은 **단계당 하한**일 뿐이며, 파라미터 체계 오차는 공분산이 아니라 **GRC 수렴으로 제거**하는 것이 설계 의도다(공분산 문서에 명시). 인덱스: `cov[0]=Var(v_x)`, `cov[7]=Var(v_y)`, `cov[35]=Var(ω)`, `cov[5]=cov[30]=Cov(v_x,ω)`; 미융합 성분(v_z, roll/pitch rate)은 `1e-3` 고정(robot_localization `preparing_sensor_data.rst` VERIFIED: 융합 변수의 0 분산에는 `1e-6` 을 더함, "Do not use large values to get the filter to ignore a given variable"). `pose.covariance` 에는 `Σ^p_pub`(인덱스 0,1,5,6,7,11,30,31,35). EKF 는 twist 만 융합(`odom0_config` vx, vy, vyaw) — 스펙 예시·`config/ekf.yaml` 과 일치.

### 2.4 드리프트 폐형 해석 (Kelly 2004; Chong–Kleeman 1997 — DOI VERIFIED)

`k_L = k_R = k`(= `k_eff + k_phys`), 직진 `L` 후(헤딩 Brownian → `Var(∫θ ds) = qL³/3`):

$$\operatorname{Var}(\theta)=\frac{2kL}{b^2},\qquad \operatorname{Var}(x)\approx\frac{kL}{2},\qquad \operatorname{Var}(y)\approx\frac{2kL^3}{3b^2};\qquad \text{제자리 회전 }\Theta:\ \operatorname{Var}(\theta)=\frac{k|\Theta|}{b}$$

차원 `[m][m]/[m²]` ✓, `[m][rad]/[m]` ✓. 수치(`sensors.yaml` 잡음만, `k_phys = 0`, L = 20 m): 1 m/s(`k = 2×10⁻⁶ m`) `σ_θ = 0.025 rad`, `σ_x = 4.5 mm`, `σ_y = 0.29 m`; 2 m/s(`k = 4×10⁻⁶ m`) `0.035 rad`, `6.3 mm`, `0.41 m`; 제자리 10 rev @1.5 rad/s(휠 0.27 m/s, `k = 5.4×10⁻⁷ m`) `σ_θ = 0.0097 rad`. MC(c1, 3000 회): `σ_θ`, `σ_y` 는 폐형과 2 % 이내, `σ_x` 는 5.7 mm(선형화에서 빠진 2차 항 `−∫θ²/2 ds` 때문; 선형 폐형은 하한). 체계 잔여(ε = 0.1 % → 0.056 rad) 와 비체계 항이 20 m 에서 같은 자릿수 → 리포트는 `Var(θ_err)` 대 `L` 의 **선형(비체계) + 2차(체계)** 동시 적합으로 `k` 와 `ε_res` 를 분리 식별하고, 속도별(1.0/2.0 m/s) 적합으로 `k_eff ∝ v` 를 확인해 `k_phys` 를 분리한다.

### 2.5 프레임 규약 · robot_localization 연동 (VERIFIED)

- **TF 확정(스펙 트리·`config/ekf.yaml`·components.md §4.2 준수, dual EKF)**: `robot_state_publisher` 가 `base_footprint→base_link`(고정, z = 0.18 m) 및 센서 프레임을, **`ekf_filter_node_odom`**(`world_frame: odom`, `base_link_frame: base_footprint`, `publish_tf: true`) 이 `odom→base_footprint` 를, **`ekf_filter_node_map`**(`world_frame: map`, `pose0 = amcl_pose`) 이 `map→odom` 을 발행한다(AMCL `tf_broadcast: false`). 출력 토픽은 각각 `odometry/filtered`, `odometry/filtered_map`. `wheel_odom` 은 `header.frame_id = <r>/odom`, `child_frame_id = <r>/base_footprint`, **TF 미발행**(components.md §5.2). 다중 로봇은 프레임 접두사 `amr_0i/` 를 런치에서 주입(ekf.yaml 주석). Nav2 의 `robot_base_frame: base_link` 는 TF 체인으로 해석되므로 충돌 없음. Gazebo `DiffDrive` 의 TF/odom 은 브리지하지 않는다(비교군 A 실험 시에만 별도 이름으로 브리지).
- 트위스트는 `child_frame_id`→`base_link_frame` 으로 자동 변환(`preparing_sensor_data.rst` VERIFIED; 여기서는 둘 다 `base_footprint` 라 항등); IMU 는 ENU 가정, 센서 자세는 TF(`imu_link`)로 자동 보정.
- **중력 처리**: Gazebo IMU 는 비력을 출력하므로 `imu/data.linear_acceleration` 에는 중력이 남는다(sensor_msgs/Imu 규약; `sensor_calibration.md` §2.4 "정지 시 z ≈ +g"). `config/ekf.yaml` 이 이미 `imu0_remove_gravitational_acceleration: true` 를 설정. `ros_filter.cpp`(ros2 브랜치 `prepareAcceleration`, L3028–3043 VERIFIED)는 `orientation_covariance[0] = −1` 이면 **필터 상태의 roll/pitch/yaw** 로 중력 벡터 `[0,0,g]` 를 회전시켜 뺀다 → `two_d_mode` 에서는 z 축에서 g 를 빼는 것과 같다 → 노드에서 중력을 제거하지 않는다(이중 제거 방지).
- **ADR-KO-01 — 스펙 EKF 예시(`imu0` yaw = true)로부터의 이탈**: gz-sensors6 `ImuSensor.cc`(VERIFIED) 는 각속도·가속도에만 잡음을 적용하고 `orientation` 은 참값이므로 yaw 융합은 GT 헤딩 융합이 된다 → `imu/data` 는 `orientation_covariance[0] = −1`, `imu0_config` yaw = **false**, vyaw = true. **프로젝트 `config/ekf.yaml` 이 이미 동일 결정**(주석: "Gazebo IMU 의 orientation 은 노이즈 없는 Ground Truth")이므로 본 ADR 은 스펙 예시와의 차이를 기록하는 문서다. 헤딩은 EKF 의 vyaw 적분 + (map EKF 에서) AMCL yaw 로 결정. **대안(yaw = true 가 요구될 경우)**: `imu_filter_node` 가 바이어스 제거 자이로를 적분한 **드리프트 yaw** 를 `orientation` 에 채우고 `imu0_differential: true`, vyaw = false(robot_localization 문서 VERIFIED: differential 은 자세를 속도로 변환) → 이중 계상 없이 예시 형식 유지.
- v1 의 "Nav2 오도메트리 가이드의 '미분 관계 필드 중복 융합 금지'" 문장은 원문을 확인하지 못해(404) 삭제 상태 유지.

---

## 3. 센서 전처리 설계

### 3.1 LiDAR (`scan_filter_node`, `scan` → `scan_filtered`; 720 빔, 0.5°, 25 m, 10 Hz, σ_r = 0.03 m)

Gazebo `gpu_lidar` 잡음은 빔별 독립 가우시안(sdformat `lidar.sdf` VERIFIED). `sensors.yaml`: `angle_min −180°`, `angle_max 179.5°`, 증분 정확히 0.5°, 미검출 빔 +inf, `lidar_link` 는 지면 +0.38 m(차체 상면 +0.33 m 위). 자체 C++ 필터 체인(O(n), ≈10 µs/scan; `laser_filters` 2.0.9 동종 필터와 합성 스캔 동치 검증). 파라미터는 components.md 계약 이름(`range_min/max`, `angle_mask`, `outlier_window`, `outlier_thresh`)을 쓰고 아래 3·4 단계는 추가 파라미터로 둔다.

1. **거리 필터** `[range_min, range_max] = [0.10, 25.0] m`; REP-117 규약(RECALLED) `< r_min → −Inf`, `> r_max → +Inf`, 오류 → `NaN`(배열 길이·각도 정렬 보존).
2. **각도 필터** `angle_mask`: 적재물/구조물 가림 섹터(YAML).
3. **풋프린트 필터**: 0.60×0.40 m + 0.05 m 내부 점 제거(스캔 평면이 차체 위라 차체 자체는 보이지 않으며, 상면 적재물·부착물 반사를 제거하는 안전장치).
4. **섀도우(veil) 필터**: 인접 빔 각 `β_i = atan2(r_{i+1} sinΔα, r_i − r_{i+1} cosΔα)`; `β_i < 10°` 또는 `> 170°` 이면 먼 점 제거.
5. **스페클(아웃라이어) 필터** = `outlier_window`/`outlier_thresh`: `min(|P_i−P_{i−1}|, |P_i−P_{i+1}|) > d_thr(r_i) = c₁ rΔα + c₂σ_r`(`c₁ = 3`, `c₂ = 3`) — Cao et al. 2025(VERIFIED)의 거리 적응 임계와 동일 구조.
6. (선택) 유리 반사 억제(SENTINEL 2026, GRAR 2026; 기본 비활성).

출력 `scan_filtered` → slam_toolbox, AMCL, costmap, `obstacle_tracker_node`, `safety_node`(components.md §4).

### 3.2 Depth 카메라 (`pointcloud_filter_node`, amr_perception; 640×480, FOV 87°, 10 m, 15 Hz)

노드 소유는 components.md 대로 amr_perception 이며, 본 절은 스펙 4.2 "Depth 점군 변환·다운샘플링" 의 알고리즘 설계를 제공한다. `f_x = 320/tan 43.5° = 337.2 px`. **잡음 모델(`sensors.yaml` 결정 + gz-sensors6 `DepthCameraSensor.cc` L405–424 VERIFIED)**: 시뮬레이터는 SDF `<camera><noise>` 의 **상수 σ 가우시안**만 지원하므로 `noise_base = 0.005 m` 만 SDF 에 넣고, 거리 제곱 항은 **본 노드가 픽셀별로 `N(0, (k_q d²)²)`, `k_q = noise_quadratic_coeff = 0.002 m⁻¹` 를 가산** → 합성 `σ(d) = √(0.005² + (0.002 d²)²)`(1 m: 5.4 mm, 3 m: 1.9 cm, 5 m: 5.0 cm). v2 의 "상수 σ = 0.01 m, 깊이 의존 항은 ablation 전용" 은 설정과 모순되어 삭제.

파이프라인(프레임당 < 1 ms, 입력 `camera/depth/image_raw` + `camera/depth/camera_info`, message_filters 동기):
1. 데시메이션 `decimation = 4` → 160×120(잡음 검증 시 1; `sensor_calibration.md` §2.6).
2. 유지 픽셀에만 거리 제곱 잡음 가산 → 역투영 `X = (u−c_x)Z/f_x`, `Y = (v−c_y)Z/f_y`, 유효 `Z ∈ [0.20, 5.0] m`(`sensors.yaml range_min`, 계약 `max_range` 5 m). 시뮬레이터 점군(`camera/depth/points`)은 좌표 규약 문제로 쓰지 않는다(`sensors.yaml` 주석).
3. 복셀 그리드 `leaf_size = 0.05 m`(0 이면 비활성), 복셀 대표점은 **최근접점**(안전 보수적) → 1–3 k 점, `camera/depth/points_filtered`(`frame_id = <r>/camera_depth_optical_frame`, 계약대로 광학 프레임). 구현: PCL 1.12 + `pcl_conversions`.
4. **높이 필터는 노드가 아니라 costmap voxel/obstacle layer** 의 `min_obstacle_height = 0.05`, `max_obstacle_height = 0.60` 으로 적용(레이어가 TF 로 전역 프레임에 변환 후 높이 판정 — 광학 프레임 발행 계약 유지). **임계 근거(설정 값으로 재계산)**: 광학축 깊이 오차 `δZ` 의 높이 성분은 `δz = δZ·tanβ`(β = 광선 고도각). 카메라 높이 = `base_link` 0.18 + extrinsic 0.25 = **0.43 m**(v2 의 0.25 m 는 오기), 수직 반화각 35.4° → 바닥은 Z ≥ 0.60 m 부터 보이며, 바닥점의 `3σ(Z)·0.43/Z` 최대값은 Z = 5 m 에서 **1.3 cm** ≪ 0.05 m → 바닥 오검출 없음. 반면 5 m 의 광선 방향 3σ = **15 cm**(복셀 3 개) 이므로 먼 장애물은 거리 방향으로 번지며, 최근접점 대표는 이를 로봇 쪽으로 치우치게 해 보수적이다(지역 costmap 팽창 반경 안에서 수용; 문서화).

### 3.3 IMU (`imu_filter_node`, `imu/data_raw` → `imu/data`; 100 Hz, 바이어스 + 잡음)

Gazebo 잡음(gz-sensors6 `GaussianNoiseModel.cc` VERIFIED): 백색 `N(0, σ)` + 상수 바이어스 `N(bias_mean, bias_stddev)`(부호 무작위, 실행마다 새로 추출) + (선택) 1차 Gauss–Markov 동적 바이어스 `b_{k+1} = e^{−Δt/τ}b_k + N(0, σ_{b,d}²)`, `σ_{b,d}² = σ_b²(τ/2)(1−e^{−2Δt/τ})`(L124–133, `expm1` 형) + `precision` 양자화. **SDF 값 = `sensors.yaml`**(v2 의 별도 "권장 SDF" 값은 설정과 달라 삭제): gyro `stddev 2×10⁻⁴ rad/s`, `bias_mean 0.01`, `bias_stddev 7.5×10⁻⁶`; accel `stddev 0.017 m/s²`, `bias_mean 0.10`, `bias_stddev 0.001`; `dynamic_bias_stddev = 0`(정적 캘리브레이션 검증 전까지), `correlation_time 300 s`.

1. **저역통과(`lpf_cutoff_hz`; 발행용 `imu/data` 전용, 잔차 계산에는 쓰지 않음 §4.2)**: 1차 IIR `y_k = αx_k + (1−α)y_{k−1}`, `H(z) = α/(1−βz⁻¹)`, `β = 1−α`. v1 의 `α = Δt/(τ_c+Δt)`(후진 오일러)는 −3 dB 점이 13.7 Hz(목표 20)·7.9 Hz(목표 10)로 어긋난다(c3 재확인). **정확 설계**: `|1−βe^{−jΩ_c}|² = 2α²` 를 풀면

$$\beta=(2-c)-\sqrt{(2-c)^2-1},\qquad c=\cos\Omega_c,\ \Omega_c=2\pi f_c/f_s$$

  → `f_c = 20 Hz`: `α = 0.673`(잡음분산비 `α/(2−α) = 0.51`, σ×0.71, DC 군지연 `β/α = 0.49` 샘플 ≈ 5 ms); `f_c = 10 Hz`: `α = 0.456`(0.30, σ×0.54, 1.19 샘플 ≈ 12 ms). 단위 테스트: 정현파 스윕으로 −3 dB 점이 `f_c` 의 ±2 % 안.
2. **바이어스 추정(`sensor_calibration.md` §2.4 와 동일 절차)**: (a) 기동 정지 `bias_estimation_time`(기본 **60 s**) 평균 → `b̂_g`, `b̂_a`(가속도는 `mean(a) − [0,0,g]`; 발행 데이터에서는 중력을 **제거하지 않음** — EKF 가 제거). 표준오차 `σ/√6000`: gyro 2.6×10⁻⁶ rad/s, accel 2.2×10⁻⁴ m/s². 보정 전 gyro 바이어스 0.01 rad/s 는 34°/min 드리프트이므로 **GRC·SACO 는 바이어스 추정 완료 후에만 활성**. (b) `dynamic_bias_stddev > 0` 으로 켤 때만 ZUPT(양 휠 `Δn = 0` 0.5 s & 자이로 분산 < 임계) 스칼라 칼만 갱신 `b⁻ = φb̂`, `P⁻ = φ²P + σ_{b,d}²`, `K = P⁻/(P⁻+σ_g²)`, `φ = e^{−Δt/τ}`.
3. **Allan 편차 검증**: `σ_A²(τ) = [2(M−1)]⁻¹Σ(ȳ_{i+1}−ȳ_i)²`(IEEE Std 952, RECALLED), 상대 신뢰구간 `≈ [2(M−1)]^{−1/2}`. 기본 설정(동적 바이어스 0)에서는 곡선이 기울기 −1/2 의 백색 잡음이어야 하며 `N = σ√Δt = 2×10⁻⁵ rad/√s`(gyro), `1.7×10⁻³ m/s/√s`(accel) 재현을 검증 — 1 h 로그로 `τ ≤ 10 s` 에서 ±3.7 %. **동적 바이어스를 켜는 경우** v1 의 30 분 로그는 `τ = 300 s` 에서 `M = 6`(±32 %)로 불충분 → **정지 로그 ≥ 4 h**(RTF 가속, 오프라인 배치): `τ ≤ 10 s` ±1.9 %, GM 피크(τ ≈ 1.9τ_GM ≈ 570 s) `M = 25`, ±14 %. 목적은 블라인드 식별이 아니라 **SDF 설정 재현 검증**과 EKF 공분산 근거.
4. 출력 `imu/data`: 바이어스 제거·LPF 된 `angular_velocity`, `linear_acceleration`(중력 포함), `orientation_covariance[0] = −1`. **공분산 대각 = LPF 전 원시 분산 + 바이어스 추정 분산**(`4×10⁻⁸ rad²/s²`, `2.9×10⁻⁴ m²/s⁴` — `config/ekf.yaml` 주석과 동일). LPF 후 분산(×0.51)을 넣지 않는 이유: LPF 출력은 자기상관이 있는데 EKF 는 100 Hz 메시지를 독립으로 취급하므로, 분산만 줄이면 정보량을 약 2 배 과장한다(v2 수정). 레버암 보정(일반형, TF 의 `imu_link` 오프셋 `l`): `a_centre = a_imu − ω̇×l − ω×(ω×l)` — **SACO 채널 2 의 내부 계산(`a_x^{centre}`)에만 쓰고, 발행 `imu/data` 에는 적용하지 않는다**. robot_localization 이 `imu_link→base_footprint` TF 원점 `p` 로 `R·a + p×α − (p×ω)×ω` 를 이미 계산하므로(`ros_filter.cpp` `prepareAcceleration` L3020–3024 VERIFIED, v3.1) 노드에서도 보정하면 이중 보정이 된다. `sensors.yaml` `l = [0, 0, 0.10]`(footprint 기준 `p = [0, 0, 0.28]`)이면 평면 운동(`ω ∥ α ∥ z`)에서 두 교차곱이 0 이라 x,y 보정은 0 — ekf.yaml 의 "IMU 가 회전 중심" 전제와 일치.

---

## 4. 독자 알고리즘 제안 — SACO + GRC (선행 연구의 변형)

### 4.1 동기

- §2.2: 체계 오차 ≫ 비체계 오차, 슬립은 영평균 잡음이 아닌 **편향 이벤트**. 고정 공분산은 EKF 를 과신시켜 AMCL 보정 점프를 유발(스펙 회전 8 cm 위험).
- 자이로는 접지와 무관한 각속도 기준(Reina et al. 2006, Ward–Iagnemma 2008); 가속도계는 자이로가 못 보는 **대칭 슬립**을 잡는다(De Giorgi et al. 2023 식 (8)/(10) 의 종방향 가속도 일관성 검사와 같은 원리).
- **EKF 관점의 실제 이득(재정의)**: 두 EKF 모두 이미 `imu0` vyaw 를 융합하므로 헤딩은 SACO 없이도 자이로가 지배한다. SACO 가 바꾸는 것은 (i) 슬립 중 `Var(v_x)` 팽창으로 odom EKF 는 `ax` 적분 쪽으로, map EKF 는 추가로 AMCL 쪽으로 가중을 옮겨 **위치 오차·AMCL 점프**를 줄이는 것, (ii) **공분산 일관성(NEES)**. 자이로 대체 헤딩의 ">50 % 감소" 는 **독립형 오도메트리 자세(`odom_gyro`)** 에만 해당한다.

### 4.2 SACO — 정렬된 창 연산자 기반 2채널 χ² 슬립 검출과 적응 공분산

**입력과 주기.** `wheel_odometry_node` 는 `joint_states`(50 Hz)마다 `wheel_odom` 을 **매 단계 50 Hz 로 발행**(components.md 계약, `safety_node` 엔코더 타임아웃 0.06 s — v2 의사코드의 10 Hz 발행은 타임아웃으로 안전 정지를 유발하므로 수정)하고, 잔차·판정은 비중첩 10 Hz 창에서 수행해 다음 창까지의 메시지에 적용한다. 잔차용 IMU 는 **원시 `imu/data_raw`** 를 추가 구독하며(계약 확장, §6), 바이어스는 `imu_filter_node` 와 같은 60 s 기동 정지 평균으로 추정한다.

**창 연산자 정렬**: 모든 잔차는 동일 구간 `W_t = [t−T_w, t]` 의 **평균 연산자** `⟨·⟩_t` 로 만든다. 엔코더 창 평균 각속도 `ω̄_enc = Δθ_w/T_w` 는 정의상 `⟨θ̇⟩_t` 이고, 자이로도 **LPF 를 거치지 않은 원시 샘플의 창 평균** `⟨ω_g⟩_t` 를 쓴다(군지연 불일치 0). **타임스탬프 요건**: 엔코더·IMU 스탬프 차 ≤ 2 ms(시뮬레이션은 둘 다 `/clock` 스탬프; c2: 5 ms 지연은 1 m/s² 가속 변화마다 5×10⁻³ m/s 의 가짜 `r_v` 를 만들어 0.3 m/s 에서 `d² ≈ 7`) — IMU 누적 적분을 엔코더 스탬프로 선형보간.

**채널 1(비대칭 슬립)**:
$$r_\omega=\bar\omega_{enc}-\big(\langle\omega_g\rangle_t-\hat b_g\big),\qquad
S_\omega=\underbrace{\frac{\sigma_R^2+\sigma_L^2}{b^2T_w^2}}_{\text{슬립+양자화}}+h_\omega^{\top}P h_\omega+\frac{\sigma_g^2}{M}+P_{b_g}$$

`σ_i² = σ_i²(T_w)`(§2.2), `M = 10`.

**채널 2(대칭 슬립) — v1 의 가속도 잔차를 적분 속도 잔차로 교체**: v1 의 `a_enc` 2차 차분은 슬립항 때문에 `σ_a ≈ 0.10–0.14 m/s²`(v1 가정, 리뷰 재현) — 설정 잡음으로도 2 m/s 에서 0.090 m/s²(c2) — 라 0.3 m/s² 이벤트를 `τ_on` 이상으로 분리하지 못하고(`(0.3/0.090)² = 11 < 13.8`), 84 ms 정렬 오차로 가속 램프마다 오경보를 낸다. 대신 창 `T_a = 0.5 s` 의 속도 변화를 비교한다:

$$r_v=\big[\bar v_{enc}(t)-\bar v_{enc}(t-T_a)\big]-\Big\langle \int_{s-T_a}^{s}\big(a_{x}^{centre}(\tau)-\hat b_{a}\big)\,d\tau\Big\rangle_{t}$$

`v̄_enc(t) = ⟨v⟩_t` 이므로 좌변 괄호는 `⟨v(s) − v(s−T_a)⟩_t = ⟨∫_{s−T_a}^{s} v̇⟩_t` 와 **항등적으로 같고**, 차동구동(`v_y = 0`)에서 `a_x = v̇` 이므로 무슬립·무잡음이면 가속 프로파일과 무관하게 잔차가 0 이다(1차 근사가 아니라 정확; 이산화 잔여는 c2 에서 저크 2·10 m/s³ 램프 1.5×10⁻⁴ m/s, 계단형 1000 m/s³ 에서도 3.7×10⁻³ m/s). 둘째 항은 원시 가속도의 누적 적분 `I(t)`(사다리꼴) 로 `⟨I(s) − I(s−T_a)⟩_t` 를 계산. 분산:

$$S_v=2\operatorname{Var}(\bar v_{enc})+\sigma_a^2\,\Delta t_i\,T_a+(P_{b_a}^{1/2}T_a)^2+h_{\Delta v}^{\top}Ph_{\Delta v},\qquad \operatorname{Var}(\bar v_{enc})=\frac{\sigma_R^2+\sigma_L^2}{4T_w^2},\quad h_{\Delta v}=\tfrac{\bar b}{2}\big[\Delta\langle\dot\phi_R\rangle,\ \Delta\langle\dot\phi_L\rangle\big]^{\top}$$

(마지막 항은 v3 추가: ψ 불확실성이 속도 변화에 곱해지는 항, `Δ` 는 `t` 와 `t−T_a` 창 사이 차; 수렴 후 1 m/s 변화당 7×10⁻⁴ m/s.) 차원: `[m/s]²`, `[m²/s⁴][s][s] = [m/s]²` ✓, `[m/s²·s]²` ✓. 검정통계량 `d² = r_ω²/S_ω + r_v²/S_v ~ χ²₂ (H₀)`.

**분해능(설정 잡음, `k_phys = 0`, GRC 수렴 `σ_ψ = 0.1 %`, `T_w = 0.1 s`, `T_a = 0.5 s`, 단일 채널이 `τ_on = 13.82` 도달 기준; c2)**:

| `v` [m/s] | `√S_ω` [rad/s] | 검출 가능 비대칭 슬립속도 `b·r_ω` [m/s] | `√S_v` [m/s] | 검출 가능 대칭 슬립속도 [m/s] |
|---|---|---|---|---|
| 0.3 | 0.0058 | 0.008 | 0.0019 | 0.007 |
| 1.0 | 0.018 | 0.024 | 0.0047 | 0.017 |
| 2.0 | 0.036 | 0.048 | 0.0090 | 0.034 |

→ 0.3 m/s² 의 대칭 슬립은 2 m/s 에서도 시작 후 약 0.11 s 에 분해능에 도달하고, 0.2 s 지속 시(Δv = 0.06 m/s) `r_v²/S_v ≈ 44`(창 평균으로 Δv ≈ 0.045 가 되어도 25). 그보다 약한 이벤트는 검출하지 않고 **트위스트 공분산의 슬립 항이 흡수**한다(설계상 한계로 문서화). 저속(0.3 m/s)에서는 IMU 적분 항(1.45×10⁻⁶)이 엔코더 항(2.1×10⁻⁶)과 같은 크기라 `σ_a` 가 분해능을 좌우한다. 잔차 잡음은 슬립 항이 지배하므로 `σ_s`·`k_phys` 식별(§5 T1)이 검출기 보정의 핵심이다.

**판정 규칙과 오경보 정의**: 비중첩 창(10 Hz)에서 `d² > τ_on = 13.82`(χ²₂ p = 0.001)가 **2회 연속**이면 슬립 ON, `d² < τ_off = 5.99` 가 `T_hold = 0.2 s` 지속되면 OFF. 명목 결정당 오경보: 단일 창 10⁻³(c2 H₀ MC 40,000 s @1 m/s: 1.00×10⁻³). 연속 창은 엔코더 창이 비중첩이라 독립이지만 IMU 적분 항은 80 % 중첩되므로, `r_v` 상관은 IMU 항의 `S_v` 점유율에 비례한다: ≥ 1 m/s 에서 ≤ 0.05(2-of-2 ≈ 1.2×10⁻⁶, 0.04 회/h), 0.3 m/s 에서 ≤ 0.33(4.8×10⁻⁶, 0.17 회/h), 정지 근처 ≤ 0.68(3.9×10⁻⁵, 1.4 회/h) (c8). 저속 오경보도 플래그 시간 비율로는 ~10⁻⁴ 이라 H2 에 여유가 있다. 잔차는 엄밀히 가우시안 백색이 아니므로(양자화 항은 삼각분포·MA(1), 물리 슬립 항은 이동거리 의존) **무슬립 주행 로그에서 `d²` 의 경험 99.9 % 분위수로 `τ_on` 을 재보정**하고 두 지표를 모두 보고: (i) 결정당 오경보율, (ii) **플래그 시간 비율**(가설 H2: ≤ 0.5 %, ≥ 1 h 무슬립 주행·가속 램프 ≥ 200 회·회전 ≥ 100 회 포함).

**정상(steady) 대칭 슬립 — 누적 잔차 유지(v3.1 추가)**: `r_v ≈ ⟨s(t) − s(t−T_a)⟩`(`s` = 대칭 과회전 속도)는 슬립 속도의 **`T_a` 차분**이므로 슬립의 *변화*만 본다. 일정한 과회전(예: 1 m/s 에서 5 % = 0.05 m/s 가 10 s 지속)은 시작 `T_a` 후 `r_v → 0` 이 되어, 위 히스테리시스만으로는 슬립 구간의 **5 %** 만 플래그된다(c9, 50 회). 그래서 ON 시점에 기준 창 `t_0 = t_1 − T_a`(`t_1` = 2-of-2 의 첫 초과 창; 그 `r_v` 의 기준 창이므로 슬립 이전)를 고정하고 누적 잔차를 유지한다:

$$\hat s(t)=\big[\bar v_{enc}(t)-\bar v_{enc}(t_0)\big]-\big\langle I(s)-I\big(s-(t-t_0)\big)\big\rangle_t,\qquad S_{\hat s}=2\operatorname{Var}(\bar v_{enc})+\sigma_a^2\,\Delta t_i\,(t-t_0)+P_{b_a}(t-t_0)^2$$

(같은 창 연산자라 무슬립이면 항등 0; 차원 `[m/s]²` ✓). OFF 조건을 `d² < τ_off` **그리고** `ŝ²/S_ŝ < χ²₁(0.95) = 3.84` 가 `T_hold` 지속으로 강화하고, `t − t_0 > T_max = 10 s` 이면 강제 OFF(IMU 적분 분산 증가 한계 — 이후는 트위스트 공분산 슬립 항과 map EKF 의 AMCL 담당). 1 m/s 에서 `√S_ŝ` = 4.8 / 6.0 / 7.3 mm/s(1 / 5 / 10 s) 라 0.05 m/s 정상 슬립은 10 s 까지 `ŝ²/S_ŝ ≥ 46`. c9: 슬립 구간 플래그 비율 0.05 → **0.97**, 무슬립 1 h 플래그 시간 비율 ≤ 6×10⁻⁵(0.3–2 m/s) 로 H2 불변. 분해능 미만으로 **서서히** 커지는 슬립은 여전히 미검출(한계 유지, 리스크 (iii)).

**적응 공분산(트위스트만; 자세는 손대지 않음)**:
$$\operatorname{Var}^{ad}(\omega)=\operatorname{Var}(\omega)+\mathbb 1_{slip}\,r_\omega^2,\qquad
\operatorname{Var}^{ad}(v_x)=\operatorname{Var}(v_x)+\mathbb 1_{slip}\Big[\max(r_v^2,\hat s^2)+\big(\tfrac b2 r_\omega\big)^2\Big],\qquad
\Sigma^{ad}_k\leftarrow\max(\Sigma^{ad}_k,\ \rho\Sigma^{ad}_{k-1}),\ \rho=e^{-\Delta t_e/\tau_d},\ \tau_d=0.3\ \mathrm s$$

차원 `(rad/s)²`, `(m/s)²`, `(m·rad/s)² = (m/s)²` ✓(한 휠 슬립 `Δv` 는 `ω` 오차 `Δv/b`, `v_x` 오차 `Δv/2 = (b/2)r_ω`). **메시지 자기일관**: `wheel_odom` 은 항상 `pose = Σ twist·Δt_e`(엔코더만, 중점식)이고 SACO 는 공분산만 바꾼다. 자이로 대체 헤딩(Gyrodometry 변형)은 **별도 토픽 `odom_gyro`**(자세만; EKF 미소비, 드리프트 리포트·EKF 부재 시 대체용)로 발행한다.

### 4.3 GRC — 자이로 기준 칼만형 RLS 파라미터 추정 (2-모드, 유계 P, EIV 편향 보상)

`ψ = [ψ_R, ψ_L] = [r_R/b, r_L/b]`(Antonelli 2005/2007 의 `c21, c22` 와 동일) 로 두면 자이로 모델이 선형: `z = ⟨ω_g⟩ − b̂_g = hᵀψ + e`, `h = [⟨φ̇_R⟩, −⟨φ̇_L⟩]` [rad/s], `R_k = S_ω,k − hᵀPh`(엔코더·자이로 항). **창 속도(10 Hz)** 로 갱신하며, 지수 망각 대신 **파라미터 랜덤워크 + 유계 P**(리뷰: `λ = 0.995` @100 Hz 는 2 s 기억, 비여기 방향에서 60 s 에 P ×1.2×10¹³):

$$P^-=P_{k-1}+qI,\quad \nu=z-h^{\top}\hat\psi_{k-1},\quad S=h^{\top}P^-h+R_k,\quad K=P^-h/S,$$
$$\hat\psi_k=\hat\psi_{k-1}+K\nu+\frac{P^-\Sigma_h\hat\psi_{k-1}}{S},\quad P_k=(I-Kh^{\top})P^-,\qquad \Sigma_h=\operatorname{diag}\!\Big(\frac{\sigma_R^2}{r^2T_w^2},\ \frac{\sigma_L^2}{r^2T_w^2}\Big)$$

- **EIV 편향 보상(v3 신설)**: 회귀벡터 `h` 자체가 엔코더 잡음(설정의 1 % 곱셈 잡음)을 포함하므로 `E[Kν] = P⁻(hhᵀ(ψ−ψ̂) − Σ_hψ̂)/S` 이고 둘째 항이 **P 가 큰(아직 여기되지 않은) 방향으로 ψ̂ 를 0 쪽으로 끌어당긴다**(`dbg_grc.py`: 비대칭 오설정에서 직진만 5 s 동안 ψ̂ 두 성분이 −1.9 %, −3.7 % 로 수축; 갱신당 크기 `P_maxψσ_φ̇²/S` 는 초기(`S ≈ hᵀP_max h`)에는 0.001 % 이지만 여기 방향이 수렴해 `S ≈ R` 이 되면 **0.125 %**(1 m/s 직진, c9) — 비여기 방향 P 가 클수록 커진다). 셋째 항이 이를 상쇄하는 표준 bias-compensated LS 보정이다(Söderström 2007, RECALLED).
- **2-모드 운용과 SACO 와의 순서**: 창마다 (1) 현재 `ψ̂` 로 잔차 계산 → (2) SACO 판정 → (3) 슬립 없음 & 양 휠 `|⟨φ̇_i⟩| > 1 rad/s` & ZUPT 아님 & 바이어스 추정 완료일 때만 GRC 갱신 → (4) 다음 창부터 `ψ̂` 적용.
  - **커미셔닝 모드**(기동 후 또는 재보정 요청 시, `P = P_max I` 에서 시작): 혁신 게이트·CUSUM 없이 위 식으로 갱신. 여기 누적 ≥ 60 s **그리고** `λ_max(P) ≤ (0.2 %·ψ_nom)²` 이면 추적 모드로 전환. 이 구간에서는 `S_ω` 의 `hᵀPh` 가 커서 채널 1 이 둔감하므로 커미셔닝은 무슬립 구역에서 수행(T3 절차; 채널 2 는 유효).
  - **추적 모드**: 혁신 게이트 `ν² ≤ 9S` 통과 시에만 갱신 + **양측 CUSUM**(게이트 전 모든 여기 창의 정규화 혁신 `ν/√S` 에 대해 `g± = max(0, g± ± ν/√S − κ)`, `κ = 0.5`, 경보 `h_c = 12`) 경보 시 **갱신 동결(래치)** + 진단 발행. 동결 해제는 재보정(커미셔닝) 요청으로만.
- **유계·안전장치**: `q = (10⁻³ψ_nom)²/6000 = 8.8×10⁻¹²`(10 분 동안 0.1 % 랜덤워크 상당, `ψ_nom = 0.229`); P 고유값을 `P_max = (0.05ψ_nom)² = 1.3×10⁻⁴` 로 클리핑(2×2 폐형) → 비여기 방향 P 는 선형·유계 성장(c3: 60 s 무여기 후 합 방향 분산 변화 < 3 %; 방향성 망각 Kulhavý–Kárný 1984 RECALLED·Gu et al. 2026 VERIFIED 의 단순 대안); 물리 범위 `ψ̂ ∈ [0.95, 1.05]ψ_nom`. **v2 의 "갱신당 변화 `|Kν| ≤ 10⁻³ψ_nom`" 은 삭제**: P 는 전체 이득으로 줄이면서 ψ̂ 는 잘린 만큼만 움직여 P 가 과신 → 이후 혁신이 게이트에 계속 걸려 **교착**(c4: 비대칭 5 % 오설정 [1.04, 0.97] 등 3 경우 모두 60 s 후에도 3.4–5 % 오차, 게이트 거부 449 회). 정상상태 속도 제한은 `q` 가 담당한다.
- **수치 검증(c7, 설정 잡음, 혼합 주행 60 s × 20 시드 × 4 경우 = 80 회)**: 5 % 오설정(대칭 [0.952, 0.952], 비대칭 [1.04, 0.97], [1.05, 0.95], [0.96, 1.03]) 모두 60 s 후 최대 오차 ≤ 0.85 %(≤ 1 % 도달 중앙값 경우별 10–32 s, 최악 55 s — [1.05, 0.95] 가 가장 느려 60 s 여유가 작다), 오동결 0/80. 추적 모드에서 한쪽 휠 지속 슬립 2 % → 0.4 s 에 동결, ψ̂ 흡수 0.012 %; 0.5 % → 9/10 동결(중앙 2.7 s), 흡수 ≤ 0.22 %. 무슬립 4 h 반복 주행 5 회: 오동결 0, ψ̂ 드리프트 ≤ 0.093 %.
- 가관측성: 직진은 `h ∝ [1/ψ_R, −1/ψ_L]`(좌우 비), 제자리 회전은 `h ∝ [1/ψ_R, 1/ψ_L]`(합 `ψ_R+ψ_L = 2r̄/b`)을 관측. 정보량(§2.2 잡음, 10 s): 1 m/s 직진으로 관측 방향 σ ≈ 0.045 %, 1.5 rad/s 회전으로 0.049 % → 혼합 주행 60 s 내 5 % → 1 % 는 잡음 모델상 여유 있음(위 MC 로 확인). 스케일 `r̄` 는 자이로만으로 비가관측(공칭 고정; 옵션으로 AMCL 저공분산 직진 구간의 `Δd_AMCL/Δd_odom` EMA — 위치추정 팀 인터페이스).
- 복원: `b̂ = 2r̄/(ψ̂_R+ψ̂_L)`, `r̂_i = ψ̂_i b̂`; `Σ_ψψ = P_k` 와 교차공분산 갱신(§2.3)을 증강 자세 공분산과 트위스트 하한 항에 반영.

### 4.4 의사코드 (50 Hz 콜백 + 10 Hz 창 처리)

```
on_joint_state(φ_L, φ_R, t):                          # 50 Hz (components.md)
  n_L, n_R = quantise(φ_L, φ_R, N); apply slip noise (σ_s = 0.01, sensors.yaml)
  Δs_i = ψ̂_i·b̄·2π·Δn_i/N ; (Δs, Δθ) = fk(Δs_R, Δs_L, b̄)
  Σ_u = diag(σ_s²Δs_R² + k_phys|Δs_R|, σ_s²Δs_L² + k_phys|Δs_L|)   # 자세용: 양자화 항 없음
  p = integrate_midpoint(p, Δs, Δθ)
  (Σ_pp, Σ_pψ) = propagate_aug(F_p, F_u, F_ψ, Σ_u, P)              # §2.3
  Σ_tw = twist_cov(σ_R²(Δt_e), σ_L²(Δt_e), b̄, Δt_e) + param_term(h_v, h_ω, P)
  Σ_tw = max(Σ_tw + slip_inflation_hold, ρ·Σ_tw_prev)               # 최근 10 Hz 판정 유지
  publish wheel_odom(pose=p, Σ_pp + Σ_q,bounded; twist=(Δs/Δt_e, 0, Δθ/Δt_e), Σ_tw)   # 매 스텝, pose = Σ twist·Δt_e
  win.push(Δs_R, Δs_L, Δθ, t)
on_imu_raw(ω_g, a_x, t): imu_win.push(ω_g, I_acc(t))                # 100 Hz, imu/data_raw
every 5th joint_state (10 Hz, 비중첩 창):
  r_ω, r_v, S_ω, S_v = residuals(win, imu_win, ψ̂, P, b̂_g, b̂_a)     # 스탬프 정렬 ≤ 2 ms, 보간
  slip = hysteresis_2of2(r_ω²/S_ω + r_v²/S_v, τ_on, τ_off, T_hold,
                        hold = ŝ²/S_ŝ ≥ 3.84 and t−t_0 ≤ 10 s)             # 누적 잔차 유지(v3.1)
  slip_inflation_hold = inflation(r_ω, max(|r_v|, |ŝ|)) if slip
  if bias_ready and not slip and excited(h) and not zupt: grc.step(h, z, R, Σ_h)   # 커미셔닝/추적 모드, EIV 보상, CUSUM
  publish odom_gyro(pose integrated with Δθ ← (⟨ω_g⟩−b̂_g)·T_w when slip)          # 독립형 평가용
  publish wheel_odom/diagnostics(d², slip, r_ω, r_v, ψ̂, P, mode, cusum)
```

### 4.5 문헌 대비 위치 (솔직한 novelty 평가)

| 블록 | 가장 가까운 선행 | 관계 |
|---|---|---|
| 자이로-엔코더 각속도 불일치로 슬립 판정 / 헤딩 대체 | Borenstein & Feng 1996 Gyrodometry(RECALLED); Reina, Ojeda, Milella, Borenstein 2006 T-Mech(DOI VERIFIED); Ward & Iagnemma 2008 T-RO(DOI VERIFIED) | **변형**: 고정 임계 → 슬립·양자화 잡음 모델로 정규화한 χ² + 2-of-2 + 히스테리시스 |
| 종방향(대칭) 슬립을 가속도계 일관성으로 검출 | **De Giorgi, De Palma, Parlangeli 2023, Robotics 13(1):7**(v3 에서 전문 VERIFIED: §3.1 식 (7)–(10) — 횡방향 식 (7)/(9), 종방향 `c21(ω_R,k−ω_R,k−1)+c22(ω_L,k−ω_L,k−1)−a_xΔt` 식 (8)/(10), IMU 데이터시트 기반 임계, "few steps" 반복 검사) | **변형**: 단계 차분 대신 정렬된 창 **적분 속도 잔차**(분산 `∝ T_a`, 무슬립 램프에서 항등 0) + 잡음 모델 정규화; Oeltjen 2025·Yu 2023 은 관련 있으나 가장 가까운 선행이 아님(v1 오귀속 정정) |
| 잔차 기반 오도메트리 공분산 팽창 | 공개 구현 rouf-rimon/adaptive-ekf-slip-detector(v3 README 재확인: 엔코더–IMU 요레이트, `R_odom(t) = R_nominal·(1 + 500λ)`, λ = 견인 손실 확률); Brossard & Bonnabel ICRA 2019(DOI VERIFIED); Okawara 2024(VERIFIED) | **변형**: 학습 없이 잔차 크기 자체로 팽창 + 감쇠, robot_localization twist 계약에 맞춘 O(1) 구현 |
| χ² 혁신 게이트 | 표준 기법(Bar-Shalom et al. 2001, RECALLED); Kuncara et al. 2024 Measurement(서지 VERIFIED; 리뷰어가 χ² 슬립 이벤트 검정을 지목했으나 초록이 ScienceDirect·SSRN 403, Semantic Scholar 초록 없음으로 v3 에서도 미확인) | 신규성 없음(명시) |
| `ψ = r/b` 파라미터화·선형 추정 | **Antonelli, Chiaverini, Fusco 2005 T-RO; Antonelli & Chiaverini 2007 Auton. Robots**(DOI VERIFIED; `c21, c22` 동일) | 파라미터화 동일 |
| 직진/회전 분리 가관측성(헤딩 오차 기준) | **Jung & Chung 2011 IJARS / 2012 ICRA**(DOI VERIFIED) | 동일 분해; 우리는 외부 헤딩 대신 자이로 |
| 온라인 증강 KF 로 `(r_L, r_R, b)` + 공분산 | **Martinelli, Tomatis, Siegwart 2006 Auton. Robots; Cantelli et al. 2016 Robotics**(DOI VERIFIED) | 우리는 자이로만 쓰는 2-파라미터 RLS 로 경량화; 스케일 비가관측을 명시 |
| 잡음 회귀벡터 편향 보상 | errors-in-variables / bias-compensated LS(Söderström 2007, RECALLED) | 표준 기법 적용(신규성 없음) |
| De Giorgi 2023 과의 관계(정정, 전문 확인) | 배치 LS(부록, Antonelli [12] 의 확장; 가정 A1–A4, A4 = 초기·최종 자세 기지), 슬립 구간 변위는 IMU 로 재구성(§3.2 식 (11)–(13)) 후 슬립 보상 배치 LS(§3.3 식 (14)–(23)) | GRC 는 **자이로 기준·재귀·자세 기지 불필요** → "De Giorgi 의 변형" 이 아니라 **Antonelli 2007 + Jung–Chung 2012 + Martinelli 2006 의 RLS 변형** |

결론: 논문급 신규성 주장 없음. **novelty_claim = "variant / new_combination"**: 새로운 것은 (i) 설정 잡음 모델(§2.2)로 정규화한 검정통계량, (ii) 정렬된 창 연산자(무슬립 항등 0), (iii) SACO↔GRC 상호 게이트와 2-모드 운용, (iv) robot_localization 공분산 계약 구현. 모든 식은 팀이 유도 가능(동료평가 요건).

### 4.6 측정 가능한 가설 · 리스크 · 예산

- **H1(주지표, EKF 일관성)**: 슬립 시나리오(T4)에서 map EKF 출력 NEES `e_kᵀΣ_k⁻¹e_k`(x, y, θ; χ²₃)가 `[0.216, 9.35]`(2.5–97.5 %) 안에 드는 샘플 비율이 고정 공분산(B) 대비 증가하여 **≥ 90 %**, 그리고 20 회 반복의 **시각별 평균 NEES** 가 `[χ²₆₀(0.025)/20, χ²₆₀(0.975)/20] = [2.02, 4.16]` 안에 드는 시각 비율 ≥ 90 %(v2 의 "3 ± 0.3" 은 20 회 평균의 표본 변동보다 좁아 통계적으로 정의되지 않아 교체). **H2**: 무슬립 주행에서 플래그 시간 비율 ≤ 0.5 %. **H3**(v3.1 정밀화): 표의 분해능(단일 채널 평균 `r²/S = τ_on`) 바로 위에서는 창당 검출확률이 비중심 χ²₂ 로 0.55 뿐이라 2-of-2 이벤트 검출이 0.68(5 창)·0.44(3 창)에 그친다(c9) → 검출률 가설은 **GT 잔차가 분해능의 1.5 배 이상**(`r²/S ≥ 2.25τ_on`; 창당 0.975, 2-of-2 는 0.3 s 이벤트 0.975·0.5 s 이벤트 0.999)이고 지속 ≥ 0.3 s 인 이벤트에 대해 **이벤트 단위 검출률 ≥ 95 %**, 지연(GT 잔차가 1.5 배 분해능을 넘은 시각부터) 중앙값 ≤ 0.3 s·95 % 분위 ≤ 0.4 s. 1.0–1.5 배 구간은 배수 `m` 대 검출률 곡선으로 보고하고 비중심 χ² 예측과 비교한다. 정상 대칭 슬립은 추가로 슬립 구간 플래그 비율 ≥ 0.9(누적 잔차 유지, §4.2). **H4**: T4 에서 map EKF 위치 RMSE·최대오차 및 AMCL 보정 점프(연속 `amcl_pose` 차분의 최대)가 B 대비 감소(효과 크기는 실험으로 보고; 사전 수치 약속 없음). **H5(GRC)**: 5 % 오설정에서 60 s 혼합 주행 커미셔닝 후 `|ψ̂−ψ_GT|/ψ_GT ≤ 1 %`(모의 최악 0.85 %), 4 h 동안 `ψ̂` 드리프트 ≤ 0.2 %(모의 0.093 %), 오동결 0. **H6(독립형)**: `odom_gyro` 의 슬립 시나리오 헤딩 오차가 `wheel_odom` 대비 ≥ 50 % 감소(EKF 지표 아님).
- **GT 슬립 정의**: `ground_truth/odom` 으로 휠 접지점 속도 `v_{c,i} = v_gt ± ω_gt b/2` 를 구해 `s_i = |r_iφ̇_i − v_{c,i}| / max(|v_{c,i}|, 0.1 m/s) > 5 %` 가 ≥ 0.1 s 지속하면 이벤트(`φ̇_i` 는 에뮬레이터 잡음 전 조인트 속도); 양 휠 동부호면 대칭(채널 2 담당), 아니면 비대칭(채널 1 담당)으로 라벨.
- 리스크: (i) 자이로 바이어스 오추정 → ψ 편향(60 s 기동 평균 SE 2.6×10⁻⁶ rad/s; 1×10⁻³ rad/s 오차도 1.5 rad/s 회전에서 ψ 에 ≤ 0.07 %); (ii) 잡음을 과소 설정하면 검출이 trivial → SDF·에뮬레이터 값은 `sensors.yaml` 고정, 슬립 서비스 필수; (iii) **채널 2 는 표 분해능 미만의 약한 대칭 슬립, 그리고 분해능 미만 속도로 서서히 커지는 슬립을 못 본다**(공분산 항으로 흡수, 문서화; 이미 검출된 정상 슬립은 누적 잔차로 최대 10 s 유지); (iv) 스케일 비가관측; (v) `k_phys` 오식별 시 `τ_on` 경험 재보정으로 보완; (vi) 엔코더-IMU 스탬프 불일치(≤ 2 ms 요건, I8 테스트); (vii) 커미셔닝 중 슬립(채널 1 둔감) → 무슬립 구역에서 수행.
- 실시간 예산: 50 Hz 경로 ~300 flop(5×5 전파 포함), 10 Hz 경로 2×2 RLS + 잔차 → 로봇당 < 0.1 % 코어, 5 대 합계 무시 가능.

---

## 5. 평가 계획 (스펙 지표 매핑)

공통 규칙: 모든 정확도 실험은 **map EKF 융합 출력 `odometry/filtered_map`** 을 GT `ground_truth/odom`(OdometryPublisher, 50 Hz, `frame_id: world`)과 비교하여 **RMSE 와 Maximum Error** 를 스펙 로그 `[timestamp, gt_x, gt_y, est_x, est_y, error]`(≥ 100 샘플, 20 Hz)로 기록하고, odom EKF `odometry/filtered`·원시 `wheel_odom`·`odom_gyro` 지표는 부가로 보고한다. **GT 정렬(v3.1 정정)**: `map`↔`world` 는 고정 SE(2) 변환 `T_wm` 이며, 맵 yaml 의 `origin`(이미지 좌하단 픽셀의 map 좌표)은 이 정렬 정보가 아니다. slam_toolbox 매핑은 첫 스캔에서 `map→odom` 을 항등으로 두고(RECALLED) odom 원점은 EKF 기동 시 자세이므로, 매핑 기동 시각의 `ground_truth/odom` 자세를 `T_wm` 으로 맵과 함께 저장해 모든 평가에 고정 사용한다(평가 궤적으로 재적합하면 추정 오차가 흡수되므로 금지). 비교군 (A) Gazebo `DiffDrive` 내장 오도메트리(실험 시에만 별도 이름으로 브리지) / (B) 우리 베이스라인(고정 공분산) / (C) +SACO / (D) +SACO+GRC.

| 실험 | 절차 | 지표 | 스펙 연계 |
|---|---|---|---|
| T1 직진 | 20 m ×10(양방향), 1.0/2.0 m/s | EKF RMSE·최대오차; 원시: 최종 오차/L [%], 헤딩 [deg/m], `Var θ` 대 L 의 선형+2차 적합 → `k`, `ε_res`; 속도별 적합으로 `k_eff ∝ v` 확인·`k_phys` 분리 | 누적 오차 분석, 직진 5 cm |
| T2 회전 | 제자리 ±10 rev ×5, 1.5 rad/s | EKF RMSE·최대오차; 원시 헤딩 오차, `k|Θ|/b` 적합, `b` 오차 | 회전 8 cm |
| T3 캘리브레이션 + GRC | `sensor_calibration.md` §2.5 의 2-파라미터 절차(직진 5 m·제자리 5 rev)를 기준으로, 추가로 4 m 정사각 CW×5/CCW×5(UMBmark)와 5 % 오설정 커미셔닝 60 s | `E_{max,syst}`, CW/CCW 중심 → `E_b, E_d`(Borenstein–Feng 1996, DOI VERIFIED; 부호 규약 원문 대조); GT 로 `(r_L, r_R, b)` 비선형 LS; GRC 수렴 시간·잔차(H5); EKF RMSE·최대오차 | 캘리브레이션 절차 문서 |
| T4 슬립 | 패치 2×2 m ×3(서비스 기반 compliance 전환), 적재 0/2/10/25 kg(F_N 갱신 / 미갱신 ablation) | H1–H4, H6; 검출률·오경보(결정당·시간비율), EKF RMSE·최대오차, AMCL 점프 | 회피·도킹 안정성 |
| T5 일관성 | T1–T4 전부 | EKF NEES(χ²₃) 대역 비율·시각별 평균, 원시 `wheel_odom` NEES(증강 공분산 검증) | 공분산 근거 |
| T6 정지 | 60 s 정지 ×5 (EKF 융합 상태) | EKF 위치 RMSE·최대오차, 바이어스 추정 오차, ZUPT 수렴(동적 바이어스 켠 경우) | 정지 3 cm |
| T7 전처리 | 합성/실 스캔·깊이 | 제거율, laser_filters 동치성, costmap 허위 장애물 수, 깊이 점 수·지연, 깊이 잡음 σ(d) 재현(`sensor_calibration.md` §2.6) | LiDAR/Depth |
| T8 자원 | 5 대 동시 | 노드별 CPU %, 토픽 hz | CPU 80 % |
| **T9 내구** | 5 대 순환 주행 **4 h**(시뮬 시간, RTF 가속 허용; 별도 배치 실행) | 토픽률(`wheel_odom` 50 / `imu/data` 100 / `scan_filtered` 10 / `camera/depth/points_filtered` 15 Hz) 및 지연 안정, RSS 증가 ≤ 5 %, `ψ̂` 드리프트 ≤ 0.2 %, GRC 오동결 0, `tr P ≤ 2P_max`, 슬립 플래그 최대 연속 ≤ 5 s, `wheel_odom` 자세 공분산 단조·유한, int64 tick, EKF RMSE 추세 없음, `map→odom→base_footprint→base_link` TF 연속 | 4 h 연속 운용 |

---

## 6. 구현 계획

- **패키지·노드(components.md §3 준수)**: `amr_localization`(C++17, ament_cmake, Eigen3; gtest + launch_pytest) 안에 `wheel_odometry_node`, `imu_filter_node`, `scan_filter_node`; 깊이 처리는 `amr_perception/pointcloud_filter_node`(설계는 §3.2). 5 대 네임스페이스 `/amr_01 … /amr_05`, 프레임 접두사 `amr_0i/`, 파라미터 YAML(재빌드 불필요; r, b 는 `robot_params.yaml`, 잡음은 `sensors.yaml` 에서 읽음).
  - `include/amr_localization/diff_drive_kinematics.hpp` — `fk()`, `integrate_midpoint()`, `integrate_arc()`, `jacobians(F_p, F_u, F_ψ)`, `propagate_aug()`(RLS 갱신 시 교차공분산 갱신 포함), `twist_covariance()`.
  - `encoder_emulator.hpp`(int64 tick·언랩·`slip_noise_stddev`), `window_stats.hpp`(정렬 창 연산자·스탬프 보간), `slip_detector.hpp`(SACO), `gyro_rls_calibrator.hpp`(GRC: 2-모드, EIV 보상, P 클리핑, CUSUM), `iir_filter.hpp`(정확 −3 dB 설계), `bias_kf.hpp`.
  - `src/wheel_odometry_node.cpp` — Sub `joint_states`(50 Hz), **`imu/data_raw`(100 Hz, 잔차용 — 계약 확장)**; Pub `wheel_odom`(50 Hz, reliable, `frame_id=<r>/odom`, `child_frame_id=<r>/base_footprint`, TF 미발행), **`odom_gyro`**(`nav_msgs/Odometry`, 10 Hz, EKF 미소비), **`wheel_odom/diagnostics`**(`diagnostic_msgs/DiagnosticArray`, 10 Hz). 굵은 3 항목은 components.md §5.2 에 없는 확장이므로 계약 갱신 요청(§8 미해결).
  - `src/imu_filter_node.cpp` — `imu/data_raw` → `imu/data`(파라미터 `lpf_cutoff_hz`, `bias_estimation_time` 60 s, `accel_bias`/`gyro_bias`; `orientation_covariance[0]=−1`, 중력 유지, 공분산 = 원시 분산 + 바이어스 분산).
  - `src/scan_filter_node.cpp` — `scan` → `scan_filtered`(파라미터 `range_min/max`, `angle_mask`, `outlier_window`, `outlier_thresh` + 풋프린트·섀도우 확장).
  - `amr_perception/src/pointcloud_filter_node.cpp` — `camera/depth/image_raw` + `camera/depth/camera_info` → `camera/depth/points_filtered`(`noise_quadratic_coeff`, `decimation`, `max_range`, `leaf_size`).
  - `tools/drift_experiment.py`, `umbmark.py`, `allan.py`, `nees.py`, `endurance_monitor.py` — rosbag2 → 지표/그림(스펙 로그 포맷 출력, `logs/` 규칙은 `sensor_calibration.md` 와 동일).
- **ROS/Nav2 인터페이스**: EKF 설정은 **`config/ekf.yaml` 그대로**(변경 없음) — `ekf_filter_node_odom`: `odom0 = wheel_odom`(vx, vy, vyaw), `imu0 = imu/data`(vyaw, ax, ay; yaw = false, `imu0_remove_gravitational_acceleration: true`), `two_d_mode: true`, 50 Hz, `base_link_frame: base_footprint`, `world_frame: odom`; `ekf_filter_node_map`: 같은 입력 + `pose0 = amcl_pose`, `world_frame: map`, `smooth_lagged_data: true`. costmap: 전역·지역 `scan_filtered`, 지역 voxel layer `camera/depth/points_filtered`(`min/max_obstacle_height` 0.05/0.60).
- **Extrinsic 소유권**: `lidar_link/camera_link/imu_link` 오프셋의 유일한 원본은 **`config/sensors.yaml` 의 `<센서>.extrinsic`**(`sensor_calibration.md` §1.1: URDF == sensors.yaml == TF), xacro 가 이를 사용. 본 영역 노드는 값을 하드코딩하지 않고 **TF 에서 읽는다**(IMU 레버암 포함). 검증 절차(`tf2_echo` 일치, LiDAR 벽/코너 검사)는 `sensor_calibration.md` §1.2·§2.1. 본 영역의 캘리브레이션 범위 = 내재 파라미터 `(r_L, r_R, b)`(T3: §2.5 절차 + UMBmark + GRC) 와 IMU 바이어스·Allan(§3.3). (v2 의 `config/extrinsics.yaml` 은 존재하지 않는 파일이라 정정.)
- Gazebo 측: 각 로봇 SDF 에 `DiffDrive`(cmd_vel 만, odom/TF 미브리지), `JointStatePublisher`(50 Hz), `OdometryPublisher`(GT), `WheelSlip`(적재별 `wheel_normal_force` 변형), IMU/LiDAR/Depth 잡음(`sensors.yaml`), 슬립 패치 스크립트(`/world/<w>/wheel_slip` 서비스 호출).
- **단위 테스트(커버리지 ≥ 70 %)**: FK 불변량; 중점 vs 원호 `≤ Δs·Δθ²/24`; 야코비안(F_p, F_u, F_ψ) vs 수치미분 1e-6; 증강 공분산 전파 vs Monte-Carlo 10⁴(파라미터 오차 포함, `Var(θ) ∝ L²`, 5 % 이내); 양자화 MA(1)(`δ²/6`, `−δ²/12`, 2차 차분 `6σ_q²`); GRC: 5 % 대칭·비대칭 오설정 4 경우 60 s 내 ≤ 1 %(**v2 교착 회귀 테스트**), EIV 보정 on/off 비교, 비여기 방향 P 유계, CUSUM 동결(지속 슬립 2 %)·오동결 0(무슬립 1 h); SACO: 무슬립 **가속 램프 데이터셋**(저크 2 m/s³ 프로파일 및 1 m/s² 도달 0.1 s, 각 200 회) 오경보 시간비율 ≤ 0.5 %, IMU 스탬프 오프셋 0/2/5 ms 민감도, 합성 슬립 이벤트(분해능 1.5 배) 검출 ≥ 95 % + `m = 1.0–2.0` 검출률 곡선이 비중심 χ²₂ 예측(c9)과 ±5 %p 일치, **정상 대칭 슬립(1 m/s, 5 %, 10 s) 플래그 비율 ≥ 0.9**(누적 잔차 유지, v3.1); 발행 주기 50 Hz·최대 간격 < 0.06 s; 섀도우/스페클/REP-117; 복셀 불변량; 깊이 σ(d) 가산; LPF −3 dB ±2 %; 바이어스 추정 SE; 중력 미제거 확인.
- **통합 시나리오(본 영역 기여 8건; 프로젝트 ≥ 10 중)**: I1 tick 에뮬레이터↔브리지(50 Hz, `Δn` 과 GT 휠 회전 1 tick 이내); I2 `wheel_odom`→`ekf_filter_node_odom`→`odom→base_footprint` TF 50 Hz, `view_frames` 로 프레임 충돌 0; I3 `imu_filter_node`→EKF 정지 시 `ax≈0`(중력 제거 확인), `−1` 규약 처리; I4 `scan_filtered`→costmap 기지 박스 셀 일치·자기 반사 허위 0; I5 `camera/depth/points_filtered`→voxel layer 1 m 장애물 표시·바닥 미표시; I6 슬립 패치 종단(SACO 플래그→EKF 공분산→AMCL 점프 감소); I7 5 대 네임스페이스·TF 무충돌·CPU; I8 `wheel_odom` 50 Hz 유지 시 `safety_node` 엔코더 타임아웃(0.06 s) 미발생 + `joint_states`/`imu/data_raw` 스탬프 차 ≤ 2 ms.
- 일정(주): 1 기구학·에뮬레이터·증강 공분산·테스트, 1 전처리 3 종 + IMU, 1 SACO+GRC, 1 실험(T1–T9)·리포트.

---

## 7. 참고문헌

**2023–2026 (VERIFIED)**
1. C. De Giorgi, D. De Palma, G. Parlangeli, "Online Odometry Calibration for Differential Drive Mobile Robots in Low Traction Conditions with Slippage," *Robotics* 13(1):7, 2023(Crossref 발행 2023-12-27, 권호 2024). https://doi.org/10.3390/robotics13010007 (**전문 VERIFIED v3**: mdpi-res.com PDF, §3 가정 A1–A4, §3.1 식 (7)–(10), §3.2 식 (11)–(13) 슬립 변위 IMU 재구성, §3.3 식 (14)–(23) 슬립 보상 LS(P 개 궤적 적층), 부록 배치 LS — v3.1 에서 절·식 번호 재대조)
2. T. Okawara et al., "Tightly-Coupled LiDAR-IMU-Wheel Odometry with Online Calibration of a Kinematic Model for Skid-Steering Robots," *IEEE Access*, 2024. https://arxiv.org/abs/2404.02515
3. T. Okawara et al., "…Online Neural Kinematic Model Learning via Factor Graph Optimization," *RAS* 187:104929, 2025. https://arxiv.org/abs/2407.08907
4. I. A. Kuncara, A. Widyotriatmo, A. Hasan, Y. Y. Nazaruddin, "Enhancing accuracy in field mobile robot state estimation with GNSS and encoders," *Measurement* 235:114903, 2024. https://doi.org/10.1016/j.measurement.2024.114903 (서지 VERIFIED; 초록·χ² 세부 미확인)
5. C. Oeltjen et al., "Online Slip Detection and Friction Coefficient Estimation for Autonomous Racing," arXiv 2025-09. https://arxiv.org/abs/2509.15423
6. X. Yu et al., "Fully Proprioceptive Slip-Velocity-Aware State Estimation … Invariant Kalman Filtering and Disturbance Observer," IROS 2023. https://arxiv.org/abs/2209.15140
7. W. Gu et al., "Covariance-Regulated Recursive Koopman Learning for Nonlinear Systems with Uncertain Time-Varying Dynamics," arXiv 2026-06. https://arxiv.org/abs/2606.15317
8. Y. Cao, Y. Huang, J. Ni, "A Two-Step Filtering Approach for Indoor LiDAR Point Clouds," *Sensors*, 2025. https://pmc.ncbi.nlm.nih.gov/articles/PMC12527108/
9. B. Liu, T.-Y. Lin, W. Zhang, M. Ghaffari, "Debiasing 6-DOF IMU via Hierarchical Learning of Continuous Bias Dynamics," RSS 2025. https://arxiv.org/abs/2504.09495
10. Abhishek S et al., "SENTINEL for Uncertainty-Aware SLAM," ICRA 2026 Workshop. https://arxiv.org/abs/2606.04853 / W. Shao et al., "GRAR: Glass-induced Reflection Artifact Removal in LiDAR Point Clouds," arXiv 2026-06. https://arxiv.org/abs/2606.10541
11. J. Choi et al., "KISS-IMU," ICRA 2026. https://arxiv.org/abs/2603.06205
12. N. Reginald et al., "Visual-Inertial-Wheel Odometry with Slip Compensation…," *Sensors*, 2025. https://pmc.ncbi.nlm.nih.gov/articles/PMC11902339/
13. C. Jiang et al., "WING: Wheel-Inertial Neural Odometry with Ground Manifold Constraints," arXiv 2024-07. https://arxiv.org/abs/2407.10101
14. B. Ćaran et al., "Odometry Calibration and Pose Estimation of a 4WIS4WID Mobile Wall Climbing Robot," ECMR 2025. https://arxiv.org/abs/2509.04016
15. M. Fazekas, P. Gáspár, "Wheel odometry model calibration with neural network-based weighting," *EAAI*, 2024 (서지). https://doi.org/10.1016/j.engappai.2024.108631
16. A. Navone et al., "Online Learning of Wheel Odometry Correction…," CASE 2023. https://arxiv.org/abs/2303.11725
17. 공개 구현: rouf-rimon, *adaptive-ekf-slip-detector*(엔코더–IMU 요레이트 잔차, 견인 손실 확률 λ, `R_odom(t) = R_nominal·(1.0 + 500.0·λ)` — README VERIFIED 2026-09-22). https://github.com/rouf-rimon/adaptive-ekf-slip-detector

**2019–2022 및 고전 (Crossref DOI VERIFIED 표시; 그 외 RECALLED)**
18. M. Brossard, S. Bonnabel, "Learning Wheel Odometry and IMU Errors for Localization," ICRA 2019. https://doi.org/10.1109/icra.2019.8794237 (VERIFIED)
19. M. K. Nutalapati et al., "A Generalized Framework for Autonomous Calibration of Wheeled Mobile Robots," arXiv:2001.01555, 2020(arXiv 주석 v3 재확인: "submitted to Elsevier Journal of Robotics and Autonomous Systems … under review"; v1 의 "RAS 2022" 정정). https://arxiv.org/abs/2001.01555 (VERIFIED)
20. G. Novotny et al., "Autonomous Vehicle Calibration via Linear Optimization," IEEE IV 2022. https://arxiv.org/abs/2204.12818 (VERIFIED)
21. L. Cantelli, S. Ligama, G. Muscato, D. Spina, "Auto-Calibration Methods of Kinematic Parameters and Magnetometer Offset for the Localization of a Tracked Mobile Robot," *Robotics* 5(4):23, 2016. https://doi.org/10.3390/robotics5040023 (VERIFIED)
22. C. Jung, W. Chung, "Accurate calibration of two wheel differential mobile robots by using experimental heading errors," ICRA 2012. https://doi.org/10.1109/icra.2012.6224660 (VERIFIED); "Calibration of Kinematic Parameters for Two Wheel Differential Mobile Robots by Using Experimental Heading Errors," *IJARS*, 2011. https://doi.org/10.5772/50906 (VERIFIED)
23. C. C. Ward, K. Iagnemma, "A Dynamic-Model-Based Wheel Slip Detector for Mobile Robots on Outdoor Terrain," *IEEE T-RO* 2008. https://doi.org/10.1109/tro.2008.924945 (VERIFIED)
24. G. Antonelli, S. Chiaverini, "Linear estimation of the physical odometric parameters for differential-drive mobile robots," *Autonomous Robots* 23, 2007. https://doi.org/10.1007/s10514-007-9030-2 (VERIFIED)
25. G. Reina, L. Ojeda, A. Milella, J. Borenstein, "Wheel slippage and sinkage detection for planetary rovers," *IEEE/ASME T-Mech* 11(2), 2006. https://doi.org/10.1109/tmech.2006.871095 (VERIFIED)
26. A. Martinelli, N. Tomatis, R. Siegwart, "Simultaneous localization and odometry self calibration for mobile robot," *Autonomous Robots* 22, 2006. https://doi.org/10.1007/s10514-006-9006-7 (VERIFIED)
27. G. Antonelli, S. Chiaverini, G. Fusco, "A calibration method for odometry of mobile robots based on the least-squares technique: theory and experimental validation," *IEEE T-RO* 21, 2005. https://doi.org/10.1109/TRO.2005.851382 (VERIFIED)
28. A. Kelly, "Linearized Error Propagation in Odometry," *IJRR* 23(2):179–218, 2004. https://doi.org/10.1177/0278364904041326 (VERIFIED v3 Crossref; v1 DOI 10.1177/0278364904039652 는 Dumitrescu et al. 논문으로 확인되어 정정)
29. K. S. Chong, L. Kleeman, "Accurate odometry and error modelling for a mobile robot," ICRA 1997. https://doi.org/10.1109/robot.1997.606708 (VERIFIED v3 Crossref; v1 의 …606954 는 Crossref 미해결)
30. J. Borenstein, L. Feng, "Measurement and correction of systematic odometry errors in mobile robots," *IEEE T-RA* 12(6):869–880, 1996. https://doi.org/10.1109/70.544770 (VERIFIED v3 Crossref); "Gyrodometry: A new method for combining data from gyros and odometry in mobile robots," ICRA 1996 (RECALLED)
31. R. Kulhavý, M. Kárný, "Tracking of slowly varying parameters by directional forgetting," IFAC 1984 (RECALLED). / Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, 2001 — χ² 게이팅·NEES 검정 (RECALLED). / T. Söderström, "Errors-in-variables methods in system identification," *Automatica* 43(6), 2007 (RECALLED). / E. S. Page, "Continuous inspection schemes," *Biometrika* 1954 — CUSUM (RECALLED). / IEEE Std 952-1997 Allan variance (RECALLED). / REP-117 (RECALLED). / A. Censi et al., *IEEE T-RO* 2013 (RECALLED).

**스택 문서·소스 (VERIFIED, v3 재fetch)**
32. robot_localization `doc/preparing_sensor_data.rst`(ros2): "Common errors" — 융합 변수의 0 분산에 `1e-6` 가산, "Do not use large values to get the filter to ignore a given variable", twist `child_frame_id`→`base_link_frame` 변환, ENU. https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/preparing_sensor_data.rst
33. robot_localization `doc/state_estimation_nodes.rst`: `imu0_remove_gravitational_acceleration`, `differential` 의 속도 변환. https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/state_estimation_nodes.rst ; `src/ros_filter.cpp` `prepareAcceleration` L3028–3043(`orientation_covariance[0] = −1` 이면 필터 상태 RPY 로 중력 제거). https://github.com/cra-ros-pkg/robot_localization/blob/ros2/src/ros_filter.cpp
34. gz-sim6 `WheelSlip.cc`(L163–165, 245–272, 276, 305–306), `UserCommands.cc`(L751 `/world/<w>/wheel_slip`), `DiffDrive.cc`; gz-sensors6 `ImuSensor.cc`, `GaussianNoiseModel.cc`(L124–133 동적 바이어스), `DepthCameraSensor.cc`(L405–424). https://github.com/gazebosim/gz-sim/tree/ign-gazebo6 , https://github.com/gazebosim/gz-sensors/tree/ign-sensors6
35. 컨테이너 `amr-fleet-system:wf-final` 확인: robot_localization 3.5.4, laser_filters 2.0.9, nav2_bringup 1.1.20, ros_gz_bridge 0.244.26, libignition-gazebo6 6.18.0(`WheelSlipCmd.hh`, `wheel-slip-system` 플러그인), `nav_msgs/Odometry`, `sensor_msgs/Imu`(−1 규약).
36. 프로젝트 문서: `config/robot_params.yaml`, `config/sensors.yaml`, `config/ekf.yaml`, `docs/architecture/components.md` §3–§5, `docs/architecture/sensor_calibration.md` §1–§2, `docs/architecture/README.md` TF 트리.

---

## 8. 리뷰 반영 이력

### 8.1 v1 → v2 (2026-09-22, 적대적 리뷰 반영)

**수학 오류(전부 수용·수정)**
1. §2.1 중점식 오차: `Δs·Δθ²` → 정확한 `Δs[1−sinc(Δθ/2)] ≤ Δs·Δθ²/24`; "mm 이하" 삭제; 단위 테스트 경계 갱신.
2. §2.2–2.3 양자화: MA(1)·텔레스코핑 구조 명시, `δ²/6` 을 자세 전파에서 제거하고 트위스트에만 유지, 자세에 유계 항 1회, 2차 차분 `6σ_q²` 명시.
3. §2.3 파라미터 체계 오차: 상태 `[p; ψ]` 증강·`Σ_pψ` 전파; 트위스트 `hᵀΣ_ψh` 는 하한이며 robot_localization 이 백색화함을 명시.
4. §4.2 가속도 채널 → **적분 속도 잔차**로 교체, 분해능 표, "여유 충분" 삭제.
5. §4.2 시간 정렬: 원시 샘플에 동일 창 연산자, 가속 램프 무슬립 데이터셋 추가.
6. §3.3 LPF: 정확 폐형 설계, 군지연 정정, 테스트 목표 ±2 %.
7. §3.3 Allan: ≥ 4 h, 신뢰구간 명시, 목적을 설정 검증으로 한정.
8. §4.3 망각: `λ` 삭제 → 창 속도 갱신, 랜덤워크 `q` + P 클리핑, 안전장치, SACO↔GRC 순서.
9. §4.2 오경보: 결정당(p = 0.001, 2-of-2)과 시간 비율(≤ 0.5 %) 이중 정의, 경험 분위수 재보정.
10. §2.2 WheelSlip: 고정 `wheel_normal_force`·force-dependent-slip·`WheelSlipCmd` 서비스로 정정, T4 프로토콜.
11. §4.2/4.4 메시지 일관성: `pose = ∫twist`, 자이로 대체 자세 `odom_gyro` 분리, 이득 재범위.

**제안 판정**: SACO(overclaimed) → 선행 명시 "변형/조합" 으로 재위치, 가속도 채널 재설계, 이득 재범위. GRC(sound) → Antonelli·Jung–Chung·Martinelli·Cantelli 대비 재위치, 안전장치 추가.
**스펙 갭**: TF·RMSE+최대오차·중력·T9·extrinsic·통합 시나리오·ADR-KO-01·깊이 잡음. **인용 정정**: Kelly, Chong–Kleeman, Borenstein–Feng DOI, Nutalapati, Nav2 페이지 주장 삭제, "2025 flock 논문" 삭제, 신규 선행 추가.

### 8.2 v2 → v3 (2026-09-22, 감사: critique.json 35 항목 전수 재검증 + 설정 정합)

감사 방법: math_errors 11 · proposal_verdicts 2 · spec_gaps 8 · required_revisions 14(critique.json 에 citation_problems 키는 없음; 인용 항목은 required_revisions #10–12 와 판정 근거에 포함) 를 항목별로 대조하고, 수학 항목은 `checks/c1–c7` 로 재유도, 인용 항목은 재fetch. v2 에서 **미해결·오해결**로 판정되어 v3 에서 고친 것:

1. **(M8/P2/R6, 오해결) GRC 교착**: v2 의 "갱신당 변화 제한 + 3σ 혁신 게이트" 조합은 P 를 전체 이득으로 줄이면서 ψ̂ 는 잘린 만큼만 움직여, 비대칭 5 % 오설정 3 경우 모두 60 s 후 3.4–5 % 오차에 고착(c4). → 제한 삭제, **EIV 편향 보상** 추가(잡음 회귀벡터가 미여기 방향으로 ψ̂ 를 수축시키는 현상 발견, c5), **커미셔닝/추적 2-모드**, CUSUM 을 구체 사양(κ = 0.5, h = 12, 게이트 전 모든 혁신)으로 확정. 80 회 모의에서 60 s 내 ≤ 0.85 %, 오동결 0, 4 h 드리프트 ≤ 0.093 %, 지속 슬립 2 % 는 0.4 s 에 동결(c7).
2. **(설정 정합, 전 절) 파라미터**: v1/v2 의 가정값 `r = 0.075`, `b = 0.35`, 100 Hz, `k = 10⁻⁵ m` → `robot_params.yaml`·`sensors.yaml`·components.md 값(`r = 0.0825`, `b = 0.36`, 50 Hz, `σ_s = 0.01` 곱셈 잡음 → `k_eff = σ_s²|v|Δt_e`)으로 §2.1–§4.6 의 모든 수치 재계산(δ, tick 률, 중점 오차 0.27 m/7.2 km, 가짜 랜덤워크 2.2 cm, 트위스트 σ, 드리프트 σ, SACO 분해능 표, GRC 정보량, `q`, `P_max`, ψ_nom).
3. **(M11/R5, 부분) 발행 주기**: v2 의사코드는 `wheel_odom` 을 10 Hz 창에서 발행 → components.md 50 Hz 계약 및 `safety_node` 엔코더 타임아웃 0.06 s 위반(안전 정지 유발). → 매 단계 50 Hz 발행, 트위스트는 단계 차분, 판정은 10 Hz 유지(§4.2, §4.4, 테스트·I8 추가).
4. **(M2/R2, 부분) 양자화 유계 항**: `δ²/12` → 시작·끝 카운트 오차를 모두 포함한 `δ²/6`, 헤딩 한계 `2δ/b`.
5. **(M3/R3, 부분) 교차공분산**: GRC 측정 갱신 시 `Σ_pψ ← Σ_pψ(I − Khᵀ)ᵀ` 누락 → 추가; 증강 전파를 MC 로 검증(2 % 이내, 백색화는 11 배 과소).
6. **(M4/R4, 부분) `S_v`**: ψ 불확실성 항 `h_ΔvᵀPh_Δv` 추가, 무슬립 잔차가 항등 0 임을 증명·수치 확인, v1 가속도 채널이 설정 잡음으로도 부족함 재확인.
7. **(M5, 부분) 타임스탬프**: 5 ms 스탬프 차가 가짜 잔차를 만든다는 수치 → ≤ 2 ms 요건·보간·테스트 추가.
8. **(M9/R14, 보강)**: H₀ MC 로 결정당 오경보 1.00×10⁻³ 확인, 2-of-2 명목 10⁻⁶ 은 ≥ 1 m/s 에서만 성립하고 저속에서는 IMU 적분 항 중첩 때문에 4.8×10⁻⁶(0.3 m/s)–3.9×10⁻⁵(정지 근처)임을 정량화(c8, 시간 비율 영향 ~10⁻⁴); H1 의 "평균 NEES 3 ± 0.3" 을 20 회 시각별 평균의 χ²₆₀ 대역 `[2.02, 4.16]` 으로 교체.
9. **(M10/R8, 보강) WheelSlip F_N**: `(m_robot + m_payload)·g/2` 는 캐스터 분담을 무시한 상한 → 설정 질량으로 상한(233–356 N) 제시 + 정지 접촉력 1회 측정; 적재 2 kg 추가.
10. **(S1/R13, 부분) TF·EKF**: 단일 EKF 서술 → `config/ekf.yaml` 의 dual EKF(`ekf_filter_node_odom` odom→base_footprint, `ekf_filter_node_map` map→odom, `pose0` 은 map 노드만).
11. **(S2/R13, 부분) 평가 대상**: `/odometry/filtered`(odom EKF, AMCL 미포함) → **`odometry/filtered_map`** 과 `ground_truth/odom` 비교.
12. **(S5, 오해결) extrinsic 파일**: 존재하지 않는 `config/extrinsics.yaml` → `config/sensors.yaml` 의 `extrinsic`(유일 원본) + `sensor_calibration.md` 절차.
13. **(S8, 오해결) 깊이 잡음**: v2 의 "상수 σ 0.01 m, 깊이 의존은 ablation 전용" 은 `sensors.yaml` 결정(0.005 m 네이티브 + 노드가 0.002·d² 가산)과 모순, 카메라 높이 0.25 m 도 오기(실제 0.43 m) → 모델·크롭 근거 재계산(바닥 3σ 높이 오차 ≤ 1.3 cm, 5 m 광선 방향 3σ = 15 cm 명시), 높이 필터를 costmap 층으로 이동(광학 프레임 발행 계약 유지).
14. **(인터페이스 정합)** 노드·토픽 이름을 components.md 로 통일: `imu_preprocess_node`→`imu_filter_node`, `/imu/raw`→`imu/data_raw`, `laser_preprocess_node`→`scan_filter_node`, `depth_cloud_node`/`/camera/points_ds`→`pointcloud_filter_node`/`camera/depth/points_filtered`, 패키지 `amr_kinematics`→`amr_localization`(+`amr_perception`), IMU 기동 바이어스 2 s → 계약 기본 60 s, 계약 확장 3 건 명시.
15. **(§3.3, 설정 정합)** IMU SDF 값을 `sensors.yaml` 로 교체(v2 의 별도 권장값 삭제), 동적 바이어스 0 에서는 Allan 이 백색 검증(1 h)이고 ≥ 4 h 는 동적 바이어스를 켤 때로 한정, **IMU 공분산은 LPF 전 분산**(LPF 후 분산은 자기상관 때문에 정보 과장) — ekf.yaml 과 일치.
16. **(인용, 재fetch)** De Giorgi 2023 전문을 v3 에서 직접 확인(식 (7)–(10), A1–A4, 배치 LS, IMU 운동 재구성) → "리뷰어 전문 확인 인용" 표기를 VERIFIED 로 승격. Kelly·Chong–Kleeman·Borenstein–Feng·Kuncara·Antonelli·Jung–Chung·Martinelli·Cantelli·Brossard·Reina·Ward DOI 를 Crossref 로 재확인, Nutalapati arXiv 주석 재확인, rouf-rimon 식·robot_localization 0-분산/중력 제거·WheelSlip/UserCommands 라인 재확인.

**리뷰어와 다른 입장(근거 포함, v2 에서 유지·갱신)**
- (a) **Kuncara et al. 2024 의 χ² 슬립 게이트 귀속**: v3·v3.1 에서도 초록 접근 불가(ScienceDirect·SSRN·ResearchGate 403, Semantic Scholar·OpenAlex 초록 없음 — OpenAlex 는 v3.1 에서 추가 조회, 검색 요약은 GNSS+엔코더·비선형 관측기+UKF·PSO 만 언급). χ² 게이팅 선행은 표준 교과서(Bar-Shalom)로 귀속하고 Kuncara 는 "서지 확인, 세부 미확인" 으로만 인용.
- (b) **깊이 크롭 0.05 m 이 "marginal"** 이라는 지적: 설정 잡음·실제 카메라 높이로 재계산해도 바닥점의 3σ 높이 오차는 ≤ 1.3 cm 이므로 잡음 관점 여유는 충분. 다만 먼 거리의 광선 방향 번짐(5 m 에서 3σ 15 cm)은 리뷰 취지대로 문서화.
- (c) **방향성 망각(Gu et al. 2026)** 을 그대로 채택하는 대신 칼만형 랜덤워크 + 고유값 클리핑 + 2-모드를 기본으로 택했다. 요구 성질(비여기 방향 P 유계, 첫 회전 시 점프 없음)은 같고, v3 모의로 수렴·무동결·유계를 확인.

**미해결(다른 소유자 조치 필요)**: (1) `config/ekf.yaml` 주석의 `wheel_odom.twist.covariance ← σ² = 1e-4` 상수 자리표시를 §2.3 속도 의존 식으로 갱신(상태추정 담당). (2) components.md §5.2 `wheel_odometry_node` 에 Sub `imu/data_raw`, Pub `odom_gyro`, `wheel_odom/diagnostics` 추가(아키텍처 담당). (3) Kuncara 2024 초록·χ² 세부 확인(기관 접근 필요). (4) 맵 저장 시 `map`↔`world` 고정 변환 `T_wm`(매핑 기동 시각의 `ground_truth/odom` 자세) 기록 — 매핑·평가 도구 담당(§5, v3.1).

### 8.3 v3 → v3.1 (2026-09-22, 감사 재개: 사용량 한도로 중단된 v3 감사의 자체 점검)

방법: critique.json 의 35 항목(math_errors 11 · proposal_verdicts 2 · spec_gaps 8 · required_revisions 14; `citation_problems` 키 없음 — 인용 항목은 R10–R12·P1/P2 근거)을 **v3 본문 기준으로 다시** 대조. 수학 항목은 c1–c8 과 독립인 `c9` 로 폐형 재계산(33/33 일치), 인용 항목은 재fetch(Crossref 17 DOI 재해결 — Kelly `…041326` ✓, 구 `…039652` = Dumitrescu et al. ✓, Chong–Kleeman `…606708` ✓ / `…606954` 미해결 ✓, Borenstein–Feng `10.1109/70.544770` ✓, Antonelli·Jung–Chung·Martinelli·Cantelli·Brossard·Reina·Ward·De Giorgi·Kuncara·Fazekas ✓; De Giorgi 전문 절·식 번호; gz-sim6 `WheelSlip.cc` L246/272/276/305–306·`UserCommands.cc` L751; `ros_filter.cpp` L3020–3043; `GaussianNoiseModel.cc` L124–133; `preparing_sensor_data.rst` 1e-6·"large values"·twist 변환; rouf-rimon README `1.0 + 500.0·λ`). 설정 정합은 `robot_params.yaml`·`sensors.yaml`·`ekf.yaml`·components.md §3–§5·`sensor_calibration.md`·`README.md` TF 트리와 재대조.

**v3.1 에서 추가로 고친 것**
1. **(M4/R4/P1, 오해결 보완) 정상 대칭 슬립**: 교체된 채널 2 의 `r_v` 는 슬립 속도의 `T_a` 차분(고역통과)이라 일정한 과회전은 시작 0.5 s 후 보이지 않는다 — v3 히스테리시스로는 10 s 정상 슬립의 5 % 만 플래그(c9). ON 시 기준 창 고정 누적 잔차 `ŝ`, 분산 `S_ŝ`(IMU 적분·바이어스 항 포함), OFF 조건 강화(`ŝ²/S_ŝ < 3.84`, `T_max = 10 s`), 팽창 항 `max(r_v², ŝ²)`, 의사코드·단위 테스트·리스크 (iii) 갱신 → 0.97, H₀ 플래그 시간 ≤ 6×10⁻⁵(§4.2).
2. **(M9/R14, 부분 → 해결) H3 의 정확 통계**: "분해능 이상이면 검출 ≥ 90 %" 는 분해능 경계에서 비중심 χ²₂ 창당 0.55, 2-of-2 이벤트 0.68 이라 성립하지 않음 → 1.5 배 분해능 이상(창당 0.975)·지속 ≥ 0.3 s 이벤트에 검출률 ≥ 95 %, 지연 분위 명시, 1.0–1.5 배는 곡선 보고(§4.6, §6).
3. **(S2/R13, 보완) GT 정렬**: "map↔world 오프셋을 맵 yaml origin 으로 정렬" 은 틀림(yaml `origin` 은 이미지 원점) → 매핑 기동 시각 GT 자세로 `T_wm` 1회 기록·고정 사용, 평가 궤적 재적합 금지(§5, 미해결 (4)).
4. **(S3, 보완) 레버암 이중 보정 방지**: robot_localization 이 TF 원점으로 `R·a + p×α − (p×ω)×ω` 를 이미 적용(`ros_filter.cpp` L3020–3024) → 발행 `imu/data` 에는 레버암 보정을 하지 않고 SACO 내부에서만 사용(§3.3).
5. **(R7, 인용 정밀화) De Giorgi 2023 절·식 번호**: "§3.2 식 (11)–(16)" → §3.2 식 (11)–(13)(슬립 변위 IMU 재구성), §3.3 식 (14)–(23)(슬립 보상 LS) (§4.5, §7).
6. **(M8/P2, 서술 정정) EIV 수축 크기**: "초기 갱신당 ≈ 0.1 %" → 초기 0.001 %, 여기 방향 수렴 후(`S ≈ R`) 0.125 % (c9; `dbg_grc.py` 재실행으로 −3.7 %/−1.9 % 재현). `q = 8.8×10⁻¹²`(8.75 의 반올림).
7. **(M2, 보완) 시작 카운트 양자화**: `q_start` 의 상수 헤딩 오프셋이 위치에 `L` 비례로 전파(20 m 에서 2.9 mm, 무시 가능)함을 명시(§2.3).
8. **(S6, 문서 불일치)** TL;DR 의 "통합 시나리오 7건" → §6 과 같은 8건.

**항목별 최종 상태(35/35 해결; ✓ = v2/v3 해결을 v3.1 에서 재확인, ✓⁺ = v3.1 에서 보완)**

| 항목 | 상태 | 위치 · 근거 |
|---|---|---|
| M1 중점 오차 | ✓ | §2.1 `Δθ²/24`; c1·c9 |
| M2 양자화 δ²/6 백색화 | ✓⁺ | §2.2–2.3 MA(1)·유계 항 1회·`q_start` 오프셋; c1·c9 |
| M3 파라미터 오차 백색화 | ✓ | §2.3 5×5 증강 + GRC 교차공분산 갱신, 트위스트 하한 명시; c1 MC |
| M4 `S_a` 과소 | ✓⁺ | §4.2 적분 속도 잔차 + 누적 잔차 유지; c2·c9 |
| M5 시간 정렬 | ✓ | §4.2 원시 샘플 동일 창 연산자, 스탬프 ≤ 2 ms, 램프 데이터셋; c2 |
| M6 LPF −3 dB | ✓ | §3.3 정확 설계 α = 0.673/0.456; c3·c9 |
| M7 Allan 길이 | ✓ | §3.3 1 h(백색)/≥ 4 h(동적 바이어스), CI; c3 |
| M8 지수 망각 | ✓⁺ | §4.3 창 속도 KF-RLS + 랜덤워크 + 고유값 클리핑 + 2-모드; c3·c7·c9 |
| M9 오경보 정의 | ✓⁺ | §4.2 p = 0.001·2-of-2·결정당/시간비율, §4.6 H2/H3; c2·c8·c9 |
| M10 WheelSlip F_N | ✓ | §2.2 고정 `wheel_normal_force`, 서비스, T4 프로토콜; 소스 L276/305–306/751 |
| M11 메시지 자기일관 | ✓ | §4.2/§4.4 `pose = Σ twist·Δt_e`, `odom_gyro` 분리, 50 Hz 발행 |
| P1 SACO(overclaimed) | ✓⁺ | §4.1/4.5/4.6 선행 재귀속·이득 재범위·NEES 주지표 + 정상 슬립·H3 |
| P2 GRC(sound) | ✓ | §4.3/4.5 Antonelli·Jung–Chung·Martinelli·Cantelli 대비, 교착 수정, CUSUM |
| S1 TF 규약 | ✓ | §2.5 dual EKF, `base_footprint`, `wheel_odom` TF 미발행 |
| S2 RMSE+최대오차 | ✓⁺ | §5 `odometry/filtered_map` vs GT, 스펙 로그, `T_wm` 정렬 |
| S3 중력 처리 | ✓⁺ | §2.5 `imu0_remove_gravitational_acceleration`, 필터 RPY 사용; §3.3 레버암 |
| S4 4 h 내구 | ✓ | §5 T9 |
| S5 extrinsic | ✓ | §6 `sensors.yaml` 유일 원본 + `sensor_calibration.md` |
| S6 통합 시나리오 | ✓⁺ | §6 I1–I8(TL;DR 개수 정정) |
| S7 ADR yaw | ✓ | §2.5 ADR-KO-01(`ekf.yaml` 동일 결정, 대안 포함) |
| S8 깊이 잡음 | ✓ | §3.2 `noise_base` + 노드 `k_q d²`, 크롭 근거(이견 (b)) |
| R1 | ✓ | = M1 |
| R2 | ✓⁺ | = M2 (2차 차분 `6σ_q²`, c1 5.98) |
| R3 | ✓ | = M3 (robot_localization 백색화 명시) |
| R4 | ✓⁺ | = M4/M5/M9 |
| R5 | ✓ | = M11 (NEES 주지표 H1) |
| R6 | ✓ | = M8 (+ 물리 범위, CUSUM 혁신 감시, SACO→GRC 순서) |
| R7 | ✓⁺ | §4.5 표(De Giorgi 식 번호 정밀화, Kuncara 는 이견 (a)) |
| R8 | ✓ | = M10 |
| R9 | ✓ | = M6/M7 + 중력 |
| R10 | ✓ | "2025 flock 논문" 삭제 확인 |
| R11 | ✓ | Nav2 문장 삭제, `preparing_sensor_data.rst` 검증 사실만 유지(v3.1 재확인) |
| R12 | ✓ | §7 DOI 정정(v3.1 Crossref 재해결), Nutalapati "under review" |
| R13 | ✓⁺ | = S1/S2/S4/S6/S7 |
| R14 | ✓⁺ | §4.6 H1(χ² 대역)·H2(시간비율)·H3(비중심 χ²)·GT 슬립 정의 |

리뷰어와 다른 입장 (a)–(c) 와 미해결 (1)–(4)(다른 소유자 조치)는 §8.2 끝의 목록이 최신(v3.1 갱신 포함)이다.

---

## 9. 수치 검증 스크립트 (`checks/`)

| 파일 | 내용 | 핵심 결과 |
|---|---|---|
| `c1_kinematics_quant_cov.py` | 중점 vs 원호, 양자화 MA(1), 증강 공분산 vs MC, 드리프트 폐형 vs MC, 트위스트 σ | 경계 비율 1.000; `Var(e)/(δ²/6) = 1.004`; 증강 2 % 이내, 백색화 11× 과소; 20 m σ_θ 0.025 rad |
| `c2_saco.py` | SACO 분해능 표, v1 가속도 채널 재현, 무슬립 램프 잔차, 스탬프 지연, H₀ 오경보·상관, NEES 대역 | 표 §4.2; 램프 1.5×10⁻⁴ m/s; 5 ms → 5×10⁻³ m/s; FA 1.00×10⁻³; `[2.02, 4.16]` |
| `c3_lpf_allan_grc_depth.py` | LPF −3 dB, Allan CI, λ 폭발, GRC 정보량, 깊이 크롭, WheelSlip F_N | 13.7/7.9 Hz → 정확 20/10 Hz; 1.15×10¹³; 1.3 cm; 233–356 N |
| `c4_grc_variants.py` | v2 GRC(클립) 교착 재현, Joseph/무클립 비교 | v2: 비대칭 3 경우 3.4–5 % 고착 |
| `c5_grc_cusum.py`, `c6_grc_bc_cusum.py`, `c7_grc_twomode.py`, `dbg_grc.py` | CUSUM, EIV 보상, 최종 2-모드, EIV 수축 진단 | 최종: 80/80 수렴 ≤ 0.85 %, 오동결 0, 4 h ≤ 0.093 % |
| `c8_fa_lowspeed.py` | 속도별 연속 창 상관과 2-of-2 오경보 | 1.2×10⁻⁶ (≥ 1 m/s) … 3.9×10⁻⁵ (정지 근처) |
| `c9_audit_final.py` (v3.1) | c1–c8 코드와 독립인 폐형 재계산 33 항목, EIV 수축 크기, `q_start` 오프셋, H3 비중심 χ² 검출확률, 정상 대칭 슬립 누적 유지 vs v3 히스테리시스, H₀ 플래그 시간 | 33/33 일치; `q = 8.75×10⁻¹²`; 0.001 → 0.125 %/갱신; 2.9 mm; `m = 1/1.5` → 0.68/0.999; 플래그 비율 0.05 → 0.97; H₀ ≤ 6×10⁻⁵ |
| `degiorgi2023.pdf/.txt`, `src/` | 재fetch 원문·소스(De Giorgi 전문, ros_filter.cpp, WheelSlip.cc, UserCommands.cc, GaussianNoiseModel.cc, DepthCameraSensor.cc, preparing_sensor_data.rst, rouf-rimon README) | §7 VERIFIED 근거 |
