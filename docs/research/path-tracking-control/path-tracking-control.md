# 경로 추종·속도 제어 설계 브리프 (spec 4.5) — Pure Pursuit / Stanley, 적응 Look-ahead, PID, S-curve/저크 제한 (리뷰 반영 개정판 v2.2)

- 작성일: 2026-09-21, 개정 2026-09-22 (v2 리뷰 반영 → v2.1 감사: 수치 재검증·계약 정합 → v2.2 최종 점검: 재개 항목 13건 수정, §9.3) / 영역: path-tracking-control / 대상 스택: ROS2 Humble, Nav2 1.1.20, Gazebo Fortress (ign-gazebo 6.18), C++17 / Python 3.10
- 웹 도구 상태: **사용 가능** (WebSearch, WebFetch, arXiv API, Crossref API, Semantic Scholar API). 인용 표기: **VERIFIED** = 초록·메타데이터를 직접 fetch, **RECALLED** = 고전 문헌을 기억에 의존.
- 스택 검증: 일회용 컨테이너 `docker run --rm amr-fleet-system:latest` / `:wf-final` 에서 `/opt/ros/humble` 헤더·plugin XML·dpkg 목록·ign-gazebo 6.18 시스템 `.so` 를 직접 확인 (`amr_dev` 컨테이너는 건드리지 않음). `/home/CAPYI` 는 읽기만 했다.
- **정합 기준**: 프로젝트 설정 `config/robot_params.yaml`·`config/sensors.yaml`·`config/ekf.yaml` 과 인터페이스 계약 `docs/architecture/components.md` §3.3·§4.1·§5, `docs/architecture/sequences.md` §1–2 (노드·토픽 이름과 값은 이 문서들을 따른다).
- 수치 검증: v2 는 `check/numcheck*.py`, v2.1 감사는 `checks/c01–c12*.py`, v2.2 최종 점검은 c01–c12 재실행(출력 일치, `checks/run_audit2_*.log`) + `checks/c13–c19*.py` (numpy/scipy, 폐루프 시뮬 포함)로 전부 재계산했다. 스크립트 이름을 본문에 `[c05]` 처럼 표기.

## 0. 요약 (TL;DR)

1. **기준선**: Pure Pursuit(PP)를 차동구동 축 중심 기준으로 유도하면 직선 근방 선형화가 $\zeta = 1/\sqrt2$, $\omega_n = \sqrt2\,v/L$ 인 2차계가 된다(교과서적 결과: Ollero & Heredia 1995, Snider 2009). 속도 비례 look-ahead $L = v\,t_L$ 는 극을 속도와 무관하게 고정한다(Nav2 RPP `use_velocity_scaled_lookahead_dist`). 원 경로 위에서는 정상상태 오차 0 이지만, **원 위의 선형화는 직선과 다르다**: 감쇠가 $\sqrt{1-L^2\kappa^2/4}$ 배로 준다(§2.2 명제 3, 자체 유도·폐루프 시뮬 검증; 정곡률 경로 안정성 해석은 Ollero & Heredia 1995 가 선행하므로 신규성은 주장하지 않음).
2. **독자 알고리즘 CCRP (Chord-Corrected Regulated Pursuit)** — 3개 모듈, 모두 정직하게 "선행연구의 변형/조합"으로 포지셔닝:
   - **CC-PP**: look-ahead 현(chord)의 횡좌표에서 완벽 추종 시에도 존재하는 기하 성분 $y_g^0$ 를 빼고 국소 곡률을 피드포워드. 선형화하면 Kanayama 형 FF+FB($k_e = 2Kv/L^2$, $k_\psi = 2Kv/L$)이며, 경로 불변성은 $O((L\kappa)^2)$ 근사에서만 성립. → **variant of prior** (Kanayama 1990 / Stanley FF 항 / Ahn et al. 2021·Yang et al. 2022 의 코너커팅 제거와 같은 목표, 다른 정식화).
   - **JRG (Jerk-bounded Regulated Governor)**: 호길이 매개 속도 상한 집합(Villagra et al. 2012 와 본질적으로 같은 상한 집합) + **결속점 제동 트리거**(현재 필터 상태 $(v,a)$ 에서의 폐형식 저크제한 제동거리 $D_b$ 로 감속 시작 시점을 정함 — 비영 상한 앞에서도 정확, 필터 지연 없음) + 50 Hz **폐형식 3차 필터**(한 스텝 스위칭면 착지; Zanasi 2000 / Haschke 2008 계열). → **new combination** (구성요소는 기존 문헌, Nav2 체인 결합과 ETA 부산물이 공학적 기여).
   - **PP-nominal DWA**: controller_server 안의 단일 nav2_core 플러그인(계약의 `amr_navigation::PurePursuitController`)이 `cmd_vel_nav` 소유. DWA 코어가 **곡률을 샘플 변수로** 후보 $(v_T,\kappa)$ 를 평가(미러 필터 상태에서 롤아웃, 명목 $(v^*,\kappa_{cmd})$ 항상 포함)하고 목표(계단 의미)로 내보내며, 하류가 $\omega = v\kappa$ 로 스케일해 가속 구간에서도 실행 곡률이 구조적으로 보존된다(DWPP 의 "직선 $\omega=\kappa v$ 위 점" 효과를 체인 전체로 확장). → **variant of DWPP (Ohnishi & Takahashi 2026)** + 장애물 비용.
3. **PID** (계약 준수 기본안): `velocity_profiler_node`(50 Hz) 안에서 피드포워드 + 몸체 $(v,\omega)$ **모델추종** 2-자유도 PI(PI 는 식별 모델 출력 대비 잔차만 보정 — v2.1 의 설정점 가중형은 자기 수락 기준 위반, §2.6.1), 피드백 `odometry/filtered`(EKF 가 4096 틱·슬립 노이즈의 `wheel_odom` 을 융합 → 스펙 4.1 잡음이 루프에 포함), 구동은 Gazebo `DiffDrive`. 페이로드(`payload/mass`)로 가속 한계·게인 스케줄. **대안 T**(계약 변경 필요): `ApplyJointForce` 토크 입력 바퀴 PI(극배치 + 질량 스케줄링, §2.6.2).
4. **평가**: GT(`ground_truth/odom`) 대비 CTE(직선 ≤ 5 cm, 곡선 ≤ 10 cm), 저크(정의·대역·잡음바닥 명시), ETA 오차, 응답시간(계약 정의: `assign_task` → 첫 `cmd_vel` ≠ 0; 스펙 문언 "로봇 첫 움직임" 으로 재면 ≈ 216–291 ms 로 초과 — §7 Q8 에 명시), CPU, 커버리지 ≥ 70 %, 4시간 연속 운전. 기준선: Nav2 RPP, DWB, **MPPI(주 비교)**, TEB(best-effort).

## 1. 스택 사실 확인 (컨테이너·공식 소스에서 직접 확인)

| 항목 | 확인 결과 | 근거 |
| --- | --- | --- |
| `nav2_core::Controller` | `configure(parent, name, tf, costmap_ros)`, `setPlan(nav_msgs::Path)`, `TwistStamped computeVelocityCommands(const PoseStamped&, const Twist&, GoalChecker*)`, `setSpeedLimit(double, bool)`, lifecycle 4종 | `/opt/ros/humble/include/nav2_core/controller.hpp` |
| controller_server 출력 | `geometry_msgs/Twist` 를 `cmd_vel` 로 publish(계약상 `cmd_vel_nav` 로 리맵), 기본 `controller_frequency: 20.0`; FollowPath 당 플러그인 **1개** 실행 | `controller_server.hpp/.cpp` (humble), components.md §5.3 |
| **인터페이스 계약** | `controller_server`(`controller_id`: `DWA` = `amr_navigation::DWAController`, `PurePursuit` = `amr_navigation::PurePursuitController`) → `cmd_vel_nav`(20 Hz) → `velocity_profiler_node`(프로파일·저크·PID, 50 Hz, Sub `odometry/filtered`, `payload/mass`) → `cmd_vel_smoothed` → `safety_node`(존·TTC·E-stop, **유일한 `cmd_vel` 발행자**, 50 Hz) → `ros_gz_bridge` → Gazebo `DiffDrive`. GT = `ground_truth/odom`(OdometryPublisher 50 Hz, 평가 전용) | components.md §3.3·§4.1·§5.1·§5.3·§5.4 |
| Nav2 RPP (1.1.20) | $\kappa = 2y_g/L^2$, $\omega = v\kappa$, look-ahead $=\mathrm{clamp}(\lvert v\rvert t_L, L_{\min}, L_{\max})$, 곡률 규제 $v\leftarrow vR/R_{\min}$, 접근 스케일링; **`allow_reversing_`, `use_rotate_to_heading_`, `shouldRotateToPath`, `shouldRotateToGoalHeading`, `rotateToHeading`, `goal_dist_tol_` 존재**; `use_fixed_curvature_lookahead` 는 없음(Iron 이후) | 설치 헤더 grep + humble 소스 |
| nav2_velocity_smoother | 가속도 제한만, **저크 제한 없음**; `scale_velocities_` 멤버 존재(한 성분 제한 시 다른 성분 비례 축소 = 곡률 보존) | `velocity_smoother.hpp` 147행 |
| nav2_collision_monitor 1.1.20 | 설치됨. 액션 `STOP / SLOWDOWN(slowdown_ratio, v·ω 동일 비율) / APPROACH`. **계약상 미사용** — 자체 `safety_node` 가 대체(components.md §1) | `types.hpp` 66–68행 |
| nav2_bringup 기본 로컬 costmap | `rolling_window: true, width: 3, height: 3` (반폭 1.5 m) → 우리 요구 **≥ 6×6 m**(§4.3). controller_server 의 로컬 costmap 은 DWA 와 공유되며 로컬플래너 브리프가 12×12 m 로 잡아 충족 | `nav2_params.yaml` 191–193행, local-planning 브리프 |
| DWPP (ref 1) | Nav2 상류에 통합되었다고 초록에 명시되나 **Humble 1.1.20 RPP 헤더에는 없음**(dynamic window 관련 멤버 0건) → 직접 구현 | 컨테이너 헤더 grep |
| 설치 패키지 | nav2-core/controller/dwb/mppi/regulated-pp/velocity-smoother/collision-monitor/behaviors/msgs(`SpeedLimit.msg`) 1.1.20; ros-gz-bridge 0.244.26; libignition-gazebo6 6.18.0; **teb_local_planner 미설치** | `dpkg -l` |
| Gazebo Fortress 시스템 | `DiffDrive`(자체 속도 제한기: `max_linear_acceleration`·`max_angular_acceleration`·`max_*_jerk` 등 — `.so` 문자열로 확인), `JointController`, `ApplyJointForce`(`/model/{m}/joint/{j}/cmd_force`, `ignition.msgs.Double`), `JointStatePublisher`, `OdometryPublisher`, `PosePublisher`, `DetachableJoint` — 모두 6.18.0 `.so` 존재 | 컨테이너 플러그인 목록, gazebosim API 문서 |
| 로봇 파라미터 | $d = 0.36$ m, $r = 0.0825$ m, 차체 45 kg + 바퀴 2×1.0 + 캐스터 2×0.3 = **공차 47.6 kg**, 바퀴 축관성 $J_w = 0.0034$ kg·m², $I_z$(차체) 1.95 kg·m², 구동륜 $\mu = 1.0$; payload 2/10/25 kg; $v_{\max}=2.0$, $v_{\min}=-0.5$, $a_{\max}=1.0$, $\omega_{\max}=1.5$, $\alpha_{\max}=2.0$ rad/s², $j_{\max}=2.0$ m/s³; safety: `distance_reference: footprint_edge`, 0.30/0.50/1.00 m, 존 속도 0.2/0.5 m/s, **`reaction_latency` 0.15 s**, 제동 모델은 $a_{\max}=1.0$ 사용, `clearance_speed_limit_enabled: true`(여유거리 연속 제한 $v_{\max}(D)$) | `config/robot_params.yaml` (프로젝트 워크트리) |
| 스펙 4.1 센서 | Wheel Encoder **틱 4096 이상 + 슬립 노이즈 필수**(`sensors.yaml`: 4096 틱, 50 Hz, 슬립 = 주기 변위에 $(1+\mathcal N(0,0.01))$ 곱 — **측정 변위에 거는 잡음**), IMU 100 Hz(가속 σ 0.017 m/s², 바이어스 0.10 m/s²) → 속도 루프 피드백·저크 지표 모두 이를 전제 | 스펙 §4.1, `config/sensors.yaml` |
| EKF | `ekf_filter_node_odom` 50 Hz, `wheel_odom`(vx, vy, vyaw) + `imu/data`(vyaw, ax, ay) → `odometry/filtered`; `two_d_mode` | `config/ekf.yaml` |

## 2. 기준 알고리즘의 수학적 유도

### 2.1 차동구동 모델과 속도 공간

단위: m, s, rad. 상태 $\mathbf x=(x,y,\theta)$, 입력 $(v,\omega)$.
$$\dot x = v\cos\theta,\quad \dot y = v\sin\theta,\quad \dot\theta = \omega,\qquad \omega_R = \frac{v+\omega d/2}{r},\ \ \omega_L = \frac{v-\omega d/2}{r}.$$
허용 속도집합은 마름모 $|v| + |\omega|\,d/2 \le r\,\omega_{w,\max}$. $v=2.0$, $\omega=1.5$ 동시 요구 시 $\omega_R = 27.5$ rad/s → URDF joint limit **≥ 28 rad/s**.

### 2.2 Pure Pursuit — 기하 유도, 원 경로 정확성, 직선·원 선형화

기준점 = 바퀴축 중점(base_link). 로봇 좌표계 look-ahead 점 $G=(x_g,y_g)$, $x_g^2+y_g^2 = L^2$:
$$R = \frac{L^2}{2y_g},\qquad \kappa = \frac{2y_g}{L^2} = \frac{2\sin\alpha}{L},\quad \alpha = \operatorname{atan2}(y_g,x_g),\qquad \omega = v\kappa\quad([\mathrm{m/s}][\mathrm m^{-1}] = \mathrm{rad/s}\ ✓).$$

**명제 1 (원 경로 정확성).** 반지름 $R$ 의 원 위에 접선 방향으로 놓인 로봇은 임의의 $L<2R$ 에 대해 정상상태 오차 없이 원을 따른다. *증명.* 동심원 $r_0$ 정상상태에서 $\kappa=1/r_0$, 현 조건 $2r_0(r_0 - R\cos\phi) = L^2$, $R^2 + r_0^2 - 2Rr_0\cos\phi = L^2$ → $r_0 = R$. ∎ 코너 커팅은 **곡률이 바뀌는 구간의 과도 현상**이다.

**명제 2 (직선 선형화, 교과서 결과).** 횡오차 $e$ (**좌측 +**, §2.5 규약), 방향오차 $\psi$. $y_g = -\sin\psi\sqrt{L^2-e^2} - e\cos\psi \approx -(e + L\psi)$:
$$\dot e = v\sin\psi \approx v\psi,\qquad \dot\psi = \omega = -\frac{2v}{L^2}(e + L\psi)\;\Rightarrow\; \ddot e + \frac{2v}{L}\dot e + \frac{2v^2}{L^2}e = 0,\qquad \omega_n = \frac{\sqrt2\,v}{L},\ \zeta = \frac1{\sqrt2}.$$
오버슈트 4.3 %, 2 % 정착 $\approx 4L/v$. $L = v\,t_L$ 이면 $\omega_n = \sqrt2/t_L$ 로 속도 불변 [Ollero & Heredia 1995 VERIFIED(직선·정곡률 경로 + 순수 지연 안정성 해석); Snider 2009 RECALLED]. 선형화는 $|\psi|\lesssim 45°$, $|e|\ll L$ 에서만 유효.

**명제 3 (원 위 정확한 1차 전개, 자체 유도·수치 검증 — 정곡률 경로 PP 안정성은 Ollero & Heredia 1995 가 다뤘으므로 같은 결과가 선행할 수 있어 신규성은 주장하지 않는다).** 곡률 $\kappa$ 의 원 위에서 $c_0 = \cos\phi_0 = 1 - L^2\kappa^2/2$, $c_1 = \sqrt{1 - L^2\kappa^2/4}$ 라 하면
$$y_g - y_g^0 \approx -c_0\,e - L\,c_1\,\psi,\qquad y_g^0 = \frac{L^2\kappa}{2},\qquad \dot\theta_p = v\kappa(1+\kappa e)+O(2).$$
(유한차분 검증: $R=1.5$, $L=1.6$ → $\partial y_g/\partial e = -0.431$, $\partial y_g/\partial\psi = -1.353$; 직선 극한 $-1$, $-L$.) 따라서 원 위에서 PP(및 §4.1 CC-PP, 이득 $K$)의 오차동역학은
$$\ddot e + \frac{2Kv}{L}c_1\,\dot e + \Big[\frac{2Kv^2}{L^2} + (1-K)v^2\kappa^2\Big]e = 0,\qquad \zeta_{circ} = \frac{K c_1}{\sqrt{2K + (1-K)L^2\kappa^2}}\ \xrightarrow{K=1}\ \frac{c_1}{\sqrt2}.$$
$R=1.5$: $L=1.6$ → $\zeta = 0.598$, $L=0.9$ → $0.675$, $L=0.876$(§4.2 거버너 작동점) → $0.676$ [c01, 유한차분으로 $\partial y_g/\partial e$, $\partial y_g/\partial\psi$ 까지 일치]. 오차동역학 자체(θ̇_p 항 포함)도 정확 기하 폐루프 시뮬로 확인: 초기오프셋 $e_0=\pm1$ cm 첫 오버슈트에서 역산한 $\zeta$ = 0.669/0.684(평균 0.676, $L=0.876$), 0.589/0.608(평균 0.598, $L=1.6$), 직선 극한 0.707 [c16] — $\pm e_0$ 비대칭은 2차 항. 거버너 작동점($v = \sqrt{a_{lat}R}$, $L = \max(vt_L, L_{\min})$)에서 $L\kappa$ 는 곡률과 함께 **커진다**: $R = 3/2/1.5/1/0.5$ m → $L\kappa = 0.41/0.51/0.58/0.72/1.18$, $\zeta_{circ}(K{=}1) = 0.69/0.68/0.68/0.66/0.57$. 즉 **극의 경로 불변성은 $O((L\kappa)^2)$ 까지만** 성립하며, $L\kappa\ll1$ 이 look-ahead 상한의 또 다른 근거다. $K=2$ 이면 $R=1.5$ 에서 $\zeta_{circ} = 1.0$ 이므로 급곡선 감쇠 저하는 $K$ 로 보상 가능(ablation).

### 2.3 Stanley 법칙과 차동구동 적용, 선택 근거

원형(Ackermann, 전륜 기준점) [Hoffmann et al. 2007 RECALLED]: $\delta = -\big(\psi + \arctan\frac{k\,e_f}{k_s + v}\big)$. 차동구동에는 가상 휠베이스 $\ell$ 을 두고 $p_f = p + \ell(\cos\theta,\sin\theta)$ 에서 $e_f = e+\ell\psi$, $\omega = (v/\ell)\tan\delta$. 선형화($k_s$ 유지):
$$\ddot e + \Big(\frac{v}{\ell} + \frac{vk}{k_s+v}\Big)\dot e + \frac{v^2 k}{\ell(k_s+v)}e = 0,\qquad \omega_n = \sqrt{\frac{v^2k}{\ell(k_s+v)}}\approx\sqrt{\frac{vk}{\ell}},\qquad \zeta = \frac{v/\ell + vk/(k_s+v)}{2\omega_n}.$$
$\zeta = \dfrac{1/\ell + k/(k_s+v)}{2\sqrt{k/(\ell(k_s+v))}} \ge 1$ 이 **임의의 $k_s\ge0$ 과 모든 $v$ 에서** 성립한다($1/\ell$ 과 $k/(k_s+v)$ 의 AM–GM; $k_s\to0$ 이면 $\zeta = (v/\ell + k)/(2\sqrt{vk/\ell})$, 등호 $v = k\ell$). **가상 휠베이스형 Stanley 는 모든 속도에서 과감쇠**($k=1,\ell=0.5$, $k_s=0$: $v=0.2/0.5/1/2$ → $\zeta = 1.11/1.00/1.06/1.25$; $k_s=0.5$: $1.01/1.06/1.16/1.34$ [c01])이고 $\omega_n\propto\sqrt v$ 로 **대역폭이 속도에 의존**한다. (저속 과민은 Ackermann 조향각형 $\arctan(ke/(k_s+v))$ 의 성질이며, $\omega=(v/\ell)\tan\delta$ 사상이 이를 제거한다.) 0–2 m/s 넓은 속도범위에서는 PP 의 **대역폭 불변**($\omega_n=\sqrt2/t_L$)이 튜닝을 단순하게 하므로 PP 계열을 기본으로 하고, Stanley 의 직접 횡오차 피드백은 §4.1 chord 보정으로 흡수한다 [Jung 2025, Lombard 2026 VERIFIED: Stanley 이점은 고속·차량동역학 영역]. Stanley 가상휠베이스형은 ablation 기준선.

### 2.4 적응 look-ahead 의 상·하한 (유도)

- 속도 스케일: $L(v) = \mathrm{clamp}(|v|\,t_L,\ L_{\min}(v),\ L_{\max})$, $t_L\in[0.6,1.0]$ s.
- **하한 (위치추정 잡음 증폭, RSS)**: $\omega$ 의 민감도 $\partial\omega/\partial e = 2Kv/L^2$, $\partial\omega/\partial\psi = 2Kv/L$ 이므로 독립 잡음 가정 하에
$$\sigma_\omega^2 = \Big(\frac{2Kv}{L^2}\Big)^2\sigma_e^2 + \Big(\frac{2Kv}{L}\Big)^2\sigma_\psi^2 \le \sigma_{\omega,\max}^2,\qquad u = 1/L:\ \ \sigma_e^2u^4 + \sigma_\psi^2u^2 - \Big(\frac{\sigma_{\omega,\max}}{2Kv}\Big)^2 = 0$$
$$\Rightarrow\ L_{\min}(v) = \Big[\frac{-\sigma_\psi^2 + \sqrt{\sigma_\psi^4 + 4\sigma_e^2(\sigma_{\omega,\max}/2Kv)^2}}{2\sigma_e^2}\Big]^{-1/2}\quad(\sigma_\psi\to0:\ \sqrt{2Kv\sigma_e/\sigma_{\omega,\max}},\ \text{단위 m ✓}).$$
$\sigma_e = 0.04$ m 는 **스펙의 오차 상한(대부분 저주파 바이어스)을 주기별 잡음 표준편차의 보수적 대리값**으로 쓴 것이다. $\sigma_\psi = 1°$, $\sigma_{\omega,\max}=0.15$ rad/s, $K=1$: $v = 0.2/0.5/1.0/2.0$ → $L_{\min} = 0.33/0.52/0.75/1.09$ m ($\sigma_\psi$ 항 기여 $v=2$ 에서 +5 %) [c08]. $t_L = 0.8$ s 면 $v\ge1$ 에서 $vt_L$ 이 자연 충족, 저속에서 하한 활성.
- **잡음이 정하는 저속 대역폭 한계 (따름정리)**: 직선 선형화에서 $k_e = 2Kv/L^2 = \omega_n^2/v$ 이므로 $\sigma_\psi\to0$ 일 때 예산 $k_e\sigma_e\le\sigma_{\omega,\max}$ 는 $\boxed{\omega_n \le \sqrt{v\,\sigma_{\omega,\max}/\sigma_e}}$ 와 동치다 — **$K$ 와 $L$ 을 어떻게 골라도** 저속 대역폭은 이 값을 넘을 수 없다($v=0.2$: 0.87 rad/s, $v=0.5$: 1.37 rad/s). 이 한계가 활성인 속도는 $v < v^* = 2\sigma_e/(\sigma_{\omega,\max}t_L^2) = 0.83$ m/s. 따라서 $K$ 는 대역폭이 아니라 **감쇠**만 바꾼다(§4.1 의 $K(v)$ 스케줄 해석).
- **상한**: $L_{\max}$ 는 (i) 로컬 costmap 반폭(§4.3: ≥ 6 m 창 → ≥ 3 m), (ii) 명제 3 의 $L\kappa\ll1$ ($R=1$ m 에서 $L\le0.9$ m 이면 $c_1\ge0.89$) 로 결정. 권장 $L_{\max}=1.8$ m; 곡률에 의한 $L$ 축소는 두지 않고 곡률은 속도 상한(§4.2)에 쓴다. 단 $L\kappa = t_L\sqrt{a_{lat,\max}\kappa}$ 는 곡률과 함께 커지므로($R=1.5$ m: 0.58, $R=1$ m: 0.72, $R=0.5$ m: 1.18 — $L_{\min}$ 활성) $R<1.5$ m 급 곡선에서는 원 위 감쇠 저하(명제 3)를 감수하거나 $K$ 로 보상한다.

### 2.5 Cross-Track Error 정의·측정 (스펙 4.10 표 준수, 부호 규약 통일)

계획 경로 $\{P_i\}$ (5 cm 간격, 평활 후), 단위접선 $t_i$, **좌측 법선 $n_i = (-t_{i,y}, t_{i,x})$**. GT 위치 $p$(`ground_truth/odom`, 50 Hz, 평가 전용)에 대해 최근접 선분 $i^*$ 를 창 탐색으로 찾고
$$\boxed{e = \big(t_{i^*}\times(p - P_{i^*})\big)\cdot\hat z = (p - P_{i^*})\cdot n_{i^*}}\qquad(\text{검산: } t=(1,0),\ p-P=(0,1)\Rightarrow e=+1,\ \text{좌측 +}).$$
이 규약을 §2.2, §4.1, `cte_logger`, 단위테스트 (9) 에 동일하게 쓴다. 로그 $[t, P_{i^*,x}, P_{i^*,y}, p_x, p_y, e]$. 구간 분류: $|\kappa_i| > 0.1\ \mathrm{m^{-1}}$ 이면 곡선, 전후 0.5 m 천이대도 곡선(보수적). 후진 구간은 별도 표기(부호는 경로 접선 기준이므로 불변). 보고: 평균 $|e|$(스펙), RMSE, 최대, P95. GT 기준 CTE = 추종오차 + 위치추정오차 → 추종기 자체 예산 평균 ≤ 2 cm(§7 Q1).

### 2.6 PID 속도 제어 — 계약 준수 기본안(몸체 PI) + 대안 T(토크 바퀴 PI)

**계약과의 관계.** components.md §3.3·§4.1 은 PID 를 `velocity_profiler_node`(50 Hz, Sub `odometry/filtered`·`payload/mass`, Pub `cmd_vel_smoothed`)에 두고 구동은 Gazebo `DiffDrive`(속도 지령)로 정했다. 따라서 **기본안은 §2.6.1**이고, v2 의 토크 수준 설계는 계약 변경(구동 = `ApplyJointForce`, `joint_states` ≥ 100 Hz)이 필요한 **대안 T(§2.6.2)**로 둔다(§7 Q3). 두 안 모두 스펙 4.5 의 "PID 속도 제어 + 게인 튜닝 문서화"를 만족한다.

#### 2.6.1 기본안 — 몸체 $(v,\omega)$ 2-자유도 PI (velocity_profiler_node, 50 Hz)

**플랜트.** DiffDrive 는 바퀴 조인트 속도 지령(서보)이므로 몸체 속도 플랜트는 서보 지연 + 측정 지연의 1차+지연 근사 $P(s) = e^{-T_d s}/(T_p s + 1)$ ($T_d$: 50 Hz 프로파일러 + EKF 50 Hz + 브리지, 추정 0.04–0.06 s; $T_p$: 서보·접촉, 0.05–0.10 s — ② 단계에서 식별). 질량은 바퀴 조인트 **effort 한계**(URDF `<limit effort>`, 시뮬 팀에 요청 — 없으면 이상 서보라 페이로드가 동역학에 거의 드러나지 않음, 스펙 4.1 "질량 변화 반영" 위배 위험)를 통해 가속 권한 $a_\tau(m) = 2\tau_{\max}/(rm)$ 로 나타난다.

**제어 법칙(피드포워드 + 모델추종 2-DOF PI, 축별; v2.2 정정)**:
$$u = u_{ff} + K_p\big(\hat y_m - y\big) + K_i\!\int (\hat y_m - y)\,dt,\qquad u_{ff} = r + \hat T_p\,a_r,\qquad \hat y_m = \hat P(s)\,u_{ff},\ \ \hat P(s) = \frac{e^{-\hat T_d s}}{\hat T_p s + 1},$$
$y = (v,\omega)$ of `odometry/filtered`, $(r, a_r)$ = §2.7 필터의 속도·가속 출력(ω 축은 $a_r = \kappa\,a$, 식별 스텝에서는 $a_r=0$), $\hat P$ 는 ② 단계에서 식별한 모델을 50 Hz 로 돌린 것. 피드포워드가 명목 추종(서보 1차 지연은 $\hat T_p a_r$ 로 선보상)을 맡고, PI 는 **모델 대비 잔차**(슬립·서보 포화·페이로드·모델 오차)만 본다. 초기 게인 $K_p = 0.3$–$0.5$, $K_i = 3$–$5\ \mathrm{s^{-1}}$: 피드백 루프 $C(s)P(s)$ 는 구조와 무관하므로 위 플랜트 범위에서 교차 3.0–5.5 rad/s, **위상여유 73–91°, 이득여유 13.5–19 dB** [c03] (외루프 PP $\omega_n = 1.77$ rad/s 와 1.7–3배 분리).
**정정 근거 [c13, c14, c15]**: v2.1 의 $u = r + K_p(b\,r - y) + K_i\!\int(r-y)$, $b=0.5$ 는 피드포워드 위에 설정점 가중을 겹친 형태라 자기 수락 기준을 못 넘는다 — 0.15 m/s 스텝 오버슈트 **12.6–25 %**(기준 ≤ 1 %), 램프에서 측정값 기준 정상오차 $a K_p(1-b)/K_i = 0.05$ m/s(기준 ≤ 0.03), S-curve 종단 오버슈트 1.3–4.5 %. 모델추종형은 식별 모델이 맞으면 스텝·S-curve 오버슈트 **0 %**, 순수지연을 뺀 추종오차 $\max|y - r(t-\hat T_d)| \le 0.021$ m/s; 모델 오차($\hat T_p$ ±30 %, $\hat T_d$ ±10 ms)에서도 스텝 ≤ 4.1 %, S-curve ≤ 1.3 %, 추종오차 ≤ 0.033 m/s, 서보 이득 −10 %(effort 포화·슬립 등가) 정상오차 0. **내루프가 PP 에 주는 영향(M11, 기본안)**: 직선 PP($t_L=0.8$) 초기오프셋 오버슈트가 운동학 4.3 % → 모델추종 내루프 4.4–5.1 %(등가 $\zeta\approx0.69$–0.70), v2.1 형 3.3–4.0 % [c15] — 명제 2 의 $\zeta = 0.707$ 예측은 통합 시험에서 ±0.02 이내로 유지된다.

**스펙 4.1 잡음이 루프에 들어오는 경로**: `wheel_odometry_node` 가 `joint_states` 를 4096 틱으로 양자화(50 Hz 차분 분해능 0.0063 m/s/바퀴)하고 슬립 노이즈(주기 변위 ×$(1+\mathcal N(0,0.01))$ → $v=2$ m/s 에서 몸체 $\sigma_v\approx0.014$ m/s)를 건 `wheel_odom` 을 EKF 가 융합한다. $K_p\le0.5$ 이면 지령 잡음 ≤ 0.007 m/s(원시 `wheel_odom` 기준; EKF 가 추가로 평활). 즉 노이즈 없는 `joint_states` 를 직접 쓰지 않는다.

**페이로드 스케줄링**: `payload/mass` 로 (i) 프로파일러·DWA 의 유효 가속 한계 $a_{\rm eff} = \min(a_{\max}, 0.8\,a_\tau(m))$ (계약: controller_server 도 `payload/mass` 로 가속 한계 보정), (ii) $K_i$ 를 $a_\tau$ 비례로 낮춰 포화 중 적분 누적을 줄인다. Anti-windup(back-calculation, $T_t = K_p/K_i$), E-stop 해제·모드 전환 시 적분기 리셋. **DiffDrive 자체 제한기**(`max_linear_acceleration`·`max_linear_jerk` 등)는 프로파일러 한계의 ≥ 1.2 배로 두어 구속되지 않게 한다(구속되면 프로파일러 필터 상태와 실제가 어긋나 §4.2 제동 트리거가 틀어진다).

**튜닝 절차(문서화 대상)**: ① 위 초기값(PI off, 피드포워드만) → ② 0.15 m/s 스텝(DiffDrive 제한기가 가속을 $\le 1.2a_{\max}$ 로 묶으므로 스텝 과도가 짧고, 대안 T 와 같은 진폭을 써 비교 가능)·S-curve 램프로 $\hat T_p, \hat T_d$ 최소자승 식별 → $\hat P$ 설정 → ③ 릴레이로 실효 지연 확인(ZN 참고값, 아래) → ④ 수락(PI on): 0.15 m/s 스텝 오버슈트 ≤ 1 %, S-curve 0→1 m/s 종단 오버슈트 ≤ 1 %, 순수지연 제외 추종오차 $\max|y - r(t-\hat T_d)| \le 0.03$ m/s, 정상오차 < 0.01 m/s; 강건성 점검으로 $\hat T_p$ 를 ±30 % 바꿔 스텝 오버슈트 ≤ 5 % 확인 → ⑤ 페이로드 0/2/10/25 kg 반복(적재 시 $\hat P$ 재식별 여부를 ④ 결과로 결정).

#### 2.6.2 대안 T — 토크 입력 바퀴 PI (`ApplyJointForce`, 계약 변경 필요)

**플랜트 (토크 → 속도)**. $I_z \approx m(a^2+b^2)/12 = 0.0433\,m$ (config `base_inertia` izz 1.95 kg·m² 와 일치):
$$\underbrace{\Big(m + \frac{2J_w}{r^2}\Big)}_{M_v}\dot v = \frac{\tau_L+\tau_R}{r} - b_v v,\qquad \underbrace{\Big(I_z + \frac{J_w d^2}{2r^2}\Big)}_{I_\omega}\dot\omega = \frac{(\tau_R-\tau_L)\,d}{2r} - b_\omega\omega\quad([\mathrm{kg}][\mathrm{m/s^2}] = [\mathrm{N\,m}]/[\mathrm m]\ ✓).$$
두 1차 플랜트로 분리되므로 $(v,\omega)$ 각각 PI 를 닫고 토크 합/차를 배분한다.

**극배치**. $g = 1/r$, 특성식 $M_v s^2 + (b_v + gK_p)s + gK_i = s^2 + 2\zeta\omega_c s + \omega_c^2$ 대응:
$$K_p = \frac{2\zeta\omega_c M_v - b_v}{g},\qquad K_i = \frac{\omega_c^2 M_v}{g}\quad([K_p] = \mathrm{N\,s},\ [K_i] = \mathrm N).$$
수치(프로젝트 config 질량: 공차 47.6 kg, $2J_w/r^2 = 1.0$ kg → $M_v = 48.6$ kg), $\zeta=1$, $\omega_c = 6$ rad/s: $K_p \approx 48.1$ N·s, $K_i\approx144$ N (25 kg 적재 $M_v = 73.6$: 72.9 / 219) [c03]. 외루프 대역은 $\omega_n = \sqrt2/t_L = 1.77$ rad/s (0.28 Hz) 이므로 $\omega_c/\omega_n = 3.4$ (초판의 "6배·1 Hz" 정정).

**2-자유도 PI (설정점 가중)**. 1-자유도 PI 는 폐루프 영점 $s = -K_i/K_p = -\omega_c/(2\zeta)$ 때문에 기준 스텝 응답이 $y = 1 - e^{-\omega_c t} + \omega_c t e^{-\omega_c t}$ → **오버슈트 13.5 % ($\zeta=1$), 20.8 % ($\zeta=0.707$)** — 초판의 "< 5 %" 기준은 달성 불가였다. 대신
$$u = K_p\big(b\,r - y\big) + K_i\!\int(r-y)\,dt,\qquad b_v = 0.5\ (\zeta=1:\ r\to y\ \text{가 정확히}\ \omega_c/(s+\omega_c),\ \text{오버슈트 }0\ \%),\quad b_\omega = 1.0.$$
$b_\omega=1$ 인 이유(정정): $b=0.5$ 이면 내루프가 **정확히 1차 지연 $6/(s+6)$** 이 되어, PP 직선 루프 특성식이 $(s/6+1)s^2 + (2v/L)s + 2v^2/L^2$ 로 바뀌고 지배극 감쇠가 0.707 → **0.59**, 초기 횡오차 복귀 오버슈트 4.3 → **6.4 %** [c03] 로 무시할 수 없다(v2 의 "Padé 162 ms → ζ 0.69" 는 한 주파수 등가지연 근사라 과소평가). $b=1$ 이면 외루프 $\omega_n$ 에서 위상지연 −2.3°(23 ms 등가)이고 PI 영점의 위상 앞섬 덕에 오버슈트는 오히려 **3.5 %** 로 준다 [c03, c09].
**운전 모드(v2.2 보완)**: $b_v=0.5$ 는 피드포워드 없는 기준 응답을 $\omega_c/(s+\omega_c)$ 로 만들므로 S-curve 램프에서 지연 $a/\omega_c = 0.167$ m/s($a=1$; 시뮬 0.163) — 아래 ④ 의 "램프 추종오차 ≤ 0.03 m/s" 와 양립하지 않는다 [c19]. 그래서 $b_v=0.5$ 는 **식별 스텝 전용**(가속 기준이 없는 스텝)이고, 운전 중에는 가속 피드포워드 $u_{ff} = (\hat M_v a_r + \hat b_v r)/g$ ($\hat M_v$ 는 `payload/mass` 로 갱신) + $b_v = 1$ PI 를 쓴다: 램프 추종오차 0.001 m/s(모델 일치), 25 kg 미반영 시 0.023 m/s [c19].

**피드백 = 엔코더 속도 (스펙 4.1)**. `joint_states`(계약 50 Hz → 대안 T 는 ≥ 100 Hz 필요) 에 4096 틱 양자화 + 슬립 노이즈를 건 속도를 30 ms 창 틱 차분으로 추정: 분해능 10 ms 0.0127, 30 ms 0.0042 m/s/바퀴 → 토크 리플 $K_p\cdot$(분해능)$/\sqrt{12}$ = 0.18 / 0.06 N·m. **슬립 노이즈는 `sensors.yaml` 에서 측정 변위에 거는 잡음이므로 이 속도 추정에 그대로 나타난다**(v2 의 "속도루프에 나타나지 않는다"는 틀림): $v=2$ m/s 에서 몸체 $\sigma_v = 0.014$ m/s → $K_p\sigma_v = 0.68$ N·m rms (포화 6 N·m 의 11 %) — 수락 가능하나 토크 리플을 지표로 보고(대안 T 는 슬립 모델을 100 Hz 주기에 맞게 재정의해야 함).

**포화·식별**. $\tau_{\max}$/바퀴 ≈ 6 N·m(**가정** — config 에 없음, URDF effort 로 확정 필요) → 가속/제동 권한 $a_\tau = 2\tau_{\max}/(rm)$ = 3.06 m/s² (47.6 kg), 2.00 m/s² (72.6 kg). 0.5 m/s 1-DOF 스텝은 $K_p\cdot0.5 = 24$ N·m 로 포화 지배 → 식별 스텝은 **0.15 m/s**($b=0.5$ 에서 초기 가속 $\omega_c\cdot0.15 = 0.9\le a_{\max}$, 토크 합 3.6 N·m; 0.2 m/s 는 1.2 m/s² 로 스펙 가속 한계를 넘는다) 또는 S-curve 램프. 정상 운전에서 속도 기준은 항상 §2.7 필터를 거치므로 스텝은 식별 전용이다.

**페이로드 게인 스케줄링**: `payload/mass` 로 $M_v, I_\omega$ 갱신, $\omega_c$ 고정으로 $K_p,K_i$ 재계산. 고정 게인으로 47.6→72.6 kg($M_v$ 48.6→73.6) 이면 $\omega_c\propto M_v^{-1/2}$ **−19 %**, $\zeta\propto M_v^{-1/2}$ −19 %, 실수부 $\zeta\omega_c = gK_p/2M_v$ **−34 %** [c03] (초판 "대역 35 %" 정정; v2 의 45→70 kg 기준 −20/−36 % 는 차체 질량만 쓴 값).

**Anti-windup (back-calculation)**: $\dot I = K_i e + \frac{1}{T_t}(u_{sat} - u)$, $T_t = K_p/K_i$. 100 Hz 전진오일러. 미분항 불사용(1차 플랜트). E-stop 해제·모드 전환 시 적분기 리셋.

**Ziegler–Nichols / 릴레이 (보조, 두 안 공통)**: $K_u = 4h/(\pi a)$, ZN-PI $K_p = 0.45K_u$, $T_i = T_u/1.2$ [Åström & Hägglund 1984 RECALLED]. 지연 없는 1차 플랜트는 진동하지 않으므로 $T_u$ 는 물리·제어주기 지연의 산물이며 ZN 게인은 참고값. **절차(대안 T)**: ① 극배치 초기값 → ② 0.15 m/s 스텝·램프로 $M_v, b_v$ 최소자승 식별 → ③ 릴레이로 실효 지연 측정 → ④ 수락: 0.15 m/s 스텝 오버슈트 ≤ 1 %($b_v=0.5$, 피드포워드 없음), S-curve 램프 추종오차 ≤ 0.03 m/s(가속 피드포워드 + $b_v=1$), 정상오차 < 0.01 m/s, 포화 없음 → ⑤ 페이로드 3종 반복.

### 2.7 사다리꼴 / S-curve 프로파일과 저크 제한

7-세그먼트 S-curve, $\Delta v$, 한계 $a, j$:
$\Delta v \ge a^2/j$: $t_j = a/j$, $t_c = \Delta v/a - a/j$, $T = \Delta v/a + a/j$; $\Delta v < a^2/j$: $t_j = \sqrt{\Delta v/j}$, $T = 2t_j$. 우리 값($a=1$, $j=2$): 0→2 m/s 2.5 s / 2.5 m (사다리꼴 2.0 s / 2.0 m).

**일반 감속 거리 (대칭 S-curve, 평균속도 $(v+v_t)/2$)** — 정지($v_t=0$)뿐 아니라 **비영 상한 앞 감속**에 필요:
$$d(v,v_t) = \begin{cases}\dfrac{v^2 - v_t^2}{2a} + \dfrac{(v+v_t)\,a}{2j}, & v - v_t \ge a^2/j\\[6pt] (v+v_t)\sqrt{\dfrac{v-v_t}{j}}, & v - v_t < a^2/j\end{cases}\qquad(\text{수치적분 검증: } 2.0\to1.10:\ 2.17\ \mathrm m,\ 2.0\to1.8:\ 1.20\ \mathrm m).$$
역함수 $d^{-1}(v_t, D)$: 장구간 폐형식 $v = -\frac{a^2}{2j} + \sqrt{\frac{a^4}{4j^2} + v_t^2 - \frac{v_t a^2}{j} + 2aD}$ (결과가 $v - v_t\ge a^2/j$ 일 때 유효), 아니면 단구간을 이분법(단조). **정지거리는 그 특수경우**:
$$D_{stop}(v) = \begin{cases} \dfrac{v^2}{2a} + \dfrac{va}{2j}, & v\ge a^2/j = 0.5\ \mathrm{m/s}\\[4pt] v^{3/2}/\sqrt j, & v < 0.5\end{cases},\qquad v_{stop}(D) = \begin{cases} -\dfrac{a^2}{2j} + \sqrt{\dfrac{a^4}{4j^2} + 2aD}, & D \ge a^3/j^2 = 0.25\ \mathrm m\\[4pt] (D\sqrt j)^{2/3}, & D < 0.25\end{cases}$$
($v=0.2$: 0.063 m, 초판 폐형식 0.070 m 는 11 % 과대 [c02] — 목표 접근·Critical 구역(≤ 0.2 m/s)이 바로 이 영역). 왕복 항등 $v_{stop}(D_{stop}(v)) = v$ 를 두 분기 모두에서 단위테스트.

**온라인 3차 필터 (계약: `velocity_profiler_node` 50 Hz; 100 Hz 에서도 동일, 상태 $(v,a)$, 목표 $v_{ref}$)** — Zanasi et al. 2000 / Haschke et al. 2008 계열의 이산시간 가변구조 필터. 스위칭면 $\sigma(v,a) = (v_{ref}-v) - a|a|/(2j_{\max})$ ("지금부터 최대 저크로 $a\to0$ 하면 $v_{ref}$ 에 닿는가")에 **다음 스텝에서 정확히 착지하는 저크를 폐형식으로 풀고** 한계로 자른다(v2 의 경계층 의사코드는 종단 스냅 `a = 0` 이 $|a|$ 를 무시해 **$|j|$ 가 5.8 m/s³(100 Hz)/8.9 m/s³(50 Hz) 로 한계의 3–4.5 배**, 속도 오버슈트 1.4–2.8 cm/s 였다 [c04] — 폐기):
```
if |v_ref - v| <= j_max*dt^2 and |a| <= j_max*dt:  a = 0; v = v_ref; return     # 종단 스냅 (|j| <= j_max 보장)
c   = v_ref - v - a*dt/2                        # a1|a1|/(2 j_max) + a1*dt/2 = c 를 만족하는 a1 이 스위칭면 착지
a1* = sign(c) * j_max * (-dt/2 + sqrt(dt^2/4 + 2|c|/j_max))
j   = clamp((a1* - a)/dt, -j_max, j_max)
a1  = clamp(a + j*dt, -a_eff, a_eff);  v = v + (a + a1)*dt/2;  a = a1               # a_eff: §2.6.1 페이로드 유효 가속
```
검증 [c05]: 0→2 m/s **2.500 s**(이론값), 50/100 Hz 모두 $\max|a| = 1.000$, $\max|j| = 2.000$, 오버슈트 0, 정상상태 저크 부호변화 0 회, 수렴 시 $a=0$ 정확; 무작위 스텝열 200 개에서도 한계 유지. 단위테스트 (4) 가 이 성질을 그대로 검사한다.
**기동 예외(선택, 기본 off)**: $v=0$, $v_{ref}>0$ 이면 $a \leftarrow a_0$ (`startup_accel_step`, 기본 **0.0**). v2 는 0.3 m/s² 를 기본으로 했으나 (i) 계약의 응답시간 정의(`assign_task` → 첫 `cmd_vel` ≠ 0, sequences.md §1)에서는 첫 프로파일러 출력이 이미 0 이 아니므로($v = j\,dt^2/2 > 0$) 필요 없고, (ii) 0.3 m/s² 스텝은 §5 저크 지표(가우시안 5 Hz)에서 피크 **5.2–5.4 m/s³**(한계 2; v2.1 의 Butterworth 지표로는 4.1–4.3) 를 만든다 [c11, c17]. GT 속도 기반 "물리적 첫 움직임"(§5 보조 지표)을 200 ms 안에 넣어야 할 때만 켠다(25 kg 박스 관성력 스텝 7.5 N ≪ 마찰 ≥ 74 N 이라 적재물 안정성과는 무관).

**횡저크 정의(차체 좌표, IMU 지표와 일치)**: $a_{lat} = v\omega$, $\boxed{j_{lat} \equiv \frac{d}{dt}(v\omega) = \dot v\omega + v\dot\omega}$. (Frenet 법선 성분은 $2\dot v\omega + v\dot\omega$, 접선 성분은 $\ddot v - v\omega^2$ — 우리는 제약·지표 모두 차체 정의를 쓴다.) 제약 적용 규칙:
$$|v|\ge0.1:\ \ |\dot\omega| \le \min\Big(\alpha_{\max},\ \frac{\max(0,\ j_{lat,\max} - |\dot v\omega|)}{|v|}\Big);\qquad |v|<0.1\ (\text{제자리 회전 영역}):\ |\dot\omega|\le\alpha_{\max}.$$
**종방향 우선 규칙**: $|\dot v\omega| > \rho\,j_{lat,\max}$ ($\rho = 0.5$) 이면 먼저 $|\dot v| \le \rho\,j_{lat,\max}/|\omega|$ 로 종방향을 줄여 $\dot\omega$ 몫을 최소 $(1-\rho)j_{lat,\max}/|v|$ 남긴다($\rho=1$ 이면 v2 처럼 $\dot\omega$ 예산이 0 이 되어 곡률 변경이 막힌다). 예: $\dot v = 1$, $\omega = 1.5$ → $|\dot v|$ 를 0.5 로 줄이고 $|\dot\omega|\le 0.75/|v|$.

**적재물 물리 한계 참고**: 25 kg 박스 미끄러짐 $a>\mu g\approx 3$–5 m/s², 전도 $\approx 12$ m/s² → $a_{\max}=1$ 에서는 비활성. 저크 제한의 실측 효과는 진동/피크 가속 억제이며 그렇게 보고한다.

**E-stop 은 저크·가속 제한을 우회**(스펙 "즉시 정지", config `emergency_stop_distance`: "속도 지령 0, 프로파일러 우회"). 정지거리 $D_E = v\tau + v^2/(2a_b)$ [c02]:
- **설계 기준(프로젝트 config 모델)**: $\tau$ = `reaction_latency` **0.15 s**, $a_b = a_{\max} = 1.0$ m/s²(config 주석: safety 제동 거리 계산에 이 값을 쓴다) → $v = 2.0/1.0/0.5/0.2$ m/s: **2.30 / 0.65 / 0.20 / 0.05 m**. 0.3 m 안에 서는 최대 속도 **0.64 m/s**. 계약 구동에서 E-stop 감속은 DiffDrive 자체 제한기가 정하며, §2.6.1 권고대로 $1.2a_{\max}$ 로 두면 1.97/0.57/0.18/0.05 m(0.3 m 내 0.69 m/s) [c02] — config 모델은 이보다 보수적인 설계 기준이다.
- 물리적 최선(토크 포화, 대안 T 또는 DiffDrive 제한 해제 시; $\tau_{\max}$ 6 N·m 가정): $a_\tau$ = 2.00 m/s²(72.6 kg)/3.06(47.6 kg), $\tau$ = 0.15 s → 1.30/0.40/0.14/0.04 m (적재), 0.3 m 안 최대 0.84 m/s. (v2 의 1.10/0.30/0.09/0.02 m 와 "$v\le1.0$" 결론은 지연 50 ms 가정이라 config 0.15 s 와 불일치 — 철회.) 구동륜 $\mu = 1.0$ 이므로 마찰은 비구속, 적재물 미끄러짐 한계(≈3 m/s²) 미만.
- **거리 기준점 = footprint 외곽**(config `safety.distance_reference: footprint_edge` 로 확정). 결론(설계 기준): 정지선(0.3 m)에서 Warning 상한 0.5 m/s 로 진입하면 여유 0.10 m, Critical 상한 0.2 m/s 이면 0.25 m. 0.64 m/s 를 넘는 속도에서의 진입은 거리 존만으로는 막지 못하므로 `safety_node` 의 두 연속 제한이 커버한다: (i) config `clearance_speed_limit_enabled: true` 의 **여유거리 제한** $v_{\max}(D) = -a\tau + \sqrt{(a\tau)^2 + 2a(D - 0.30)}$ ($D$ = footprint 외곽–장애물 거리; $D = 2.6$ m → 2.0 m/s, 1.0 m → 1.04 m/s — 위 $D_E$ 모델의 역함수라 정적 장애물 앞에서 0.3 m 전 정지를 보장), (ii) TTC 제한 $v\le a(\mathrm{TTC} - t_{react})$, $\tau_{crit} = 2.15$ s (sequences.md §2, 동적 장애물). 둘 다 $v,\omega$ 동일 비율 스케일로 적용하도록 요청(§4.3 κ 보존) — 수치는 안전 시스템 영역과 일치시켰다.

### 2.8 Nav2 RPP 기준선 (Humble 1.1.20, 코드 검증)

| 요소 | RPP 구현 | 우리 |
| --- | --- | --- |
| look-ahead | $\mathrm{clamp}(\lvert v\rvert t_L, 0.3, 0.9)$, 기본 고정 0.6 m; 기본 `desired_linear_vel 0.5` | $L(v)$ + $L_{\min}(v)$ RSS, $L_{\max}$ 1.8 |
| 조향 | $\kappa = 2y_g/L^2$, $\omega = v\kappa$ | CC-PP: $K(v)$, 곡률 FF, 직선·원에서 $K=1$ 이면 동일 |
| 곡률 규제 | $R<R_{\min}(0.9)$: $v\leftarrow vR/R_{\min}$, 하한 0.25 m/s | $\sqrt{a_{lat,\max}/\lvert\kappa\rvert}$ 등 물리량 기반 |
| 비용 규제 / 충돌 검사 | inflation 비용 역산 감속; 호 투영 최대 1.0 s | DWA 장애물 비용 + $D_{stop}$ 허용성 |
| 접근 감속 | 남은 경로 < 1.0 m 선형 | 결속점 제동 트리거(§4.2, 목표점 $v=0$ 포함) + $D_{stop}$ 구간별 폐형식 |
| 시작 회전 / 목표 회전 / 후진 | `shouldRotateToPath`(0.785 rad, 1.8 rad/s, $\alpha\le3.2$), `shouldRotateToGoalHeading`(goal_dist_tol), `allow_reversing_` | §4.1 모드 전환: 동일 의미, $\omega_{\max}=1.5$, $\alpha_{\max}=2.0$ |

## 3. 문헌 조사 (2023-09 → 2026-09 + 필수 선행연구)

| # | 문헌 | 상태 | 관련성 / 우리와의 차이 |
| --- | --- | --- | --- |
| 1 | Ohnishi & Takahashi, *DWPP: Dynamic Window Pure Pursuit Considering Velocity and Acceleration Constraints*, arXiv 2601.15006 (2026-01) | VERIFIED (arXiv API 초록 재확인) | 동적 창 내 직선 $\omega=\kappa v$ 에 가장 가까운 점 선택; 초록상 Nav2 상류 통합(Humble 1.1.20 에는 없음, §1). §4.3 은 같은 원리를 **곡률 운반**($(v_T,\kappa)$ 샘플 + 하류 $\omega = v\kappa$ 스케일)으로 체인 전체에 적용하고 장애물 비용을 더한다 |
| 2 | Elgouhary & El-Wakeel, *Dynamic Lookahead Distance via RL-Based Pure Pursuit*, arXiv 2603.28625 | VERIFIED | PPO 로 look-ahead. 우리는 해석적 상·하한 |
| 3 | Elgouhary & El-Wakeel, *Joint Lookahead and Steering-Gain Control with PPO*, arXiv 2602.18386 | VERIFIED | look-ahead·조향게인 동시 학습 — 우리 $K(v)$ 는 감쇠비로 해석되는 결정론 스케줄 |
| 4 | Gholampour & Beaver, *Reachability-Aware Time Scaling for Path Tracking*, arXiv 2604.00439 | VERIFIED | 가속도 여유 기반 속도 재조정(이중적분기) — JRG 상한과 동기 유사 |
| 5 | Promkaew et al., *Enhanced pure pursuit with dynamic steering control (PP-DSC)*, Sci. Rep. 16:8820, 2026, DOI 10.1038/s41598-026-38695-1 | VERIFIED (PMC 본문 재fetch, Crossref) | 저자 표현은 "four-wheeled steering-type AMR" 이나 본문상 **car-like 전륜조향(Ackermann)**, 휠베이스 613.5 mm. look-ahead 는 개념식 Eq. 20 ($1.5 + 3(v-1)$ m, 상한 11.5 m)과 구현 Eq. 29/표 3 두 가지가 모두 있다(표 3 값은 0.5/4.0 m·0.5–5.0 m/s, 본문 서술은 "0.5–5.0 m" 로 논문 안에서도 상한이 엇갈림). 평균 횡편차 0.05/0.07/0.08 m(직선/루프/8자) vs PP 0.19/0.40/0.27 m → 74 / 82 / 70 %(루프 지속선회 구간 87 %, 초록 요약 "68–82 %"), 1.0–5.0 m/s 야외. 곡률 반경 5–9 m 공장 시뮬에서는 **표준 PP 가 15.6 % 더 좋았다**. 차량·다른 속도대이므로 스펙 5/10 cm 의 "현실성" 근거로는 **약한 간접 증거** |
| 6 | Jung, *Model-Based Hybrid Control of Pure Pursuit and Stanley*, Sensors 25(20):6491, 2025 | VERIFIED (PMC) | IMM 확률 혼합, RMS 3–9 % 개선(차량). 우리는 혼합 대신 FF |
| 7 | Lombard et al., *Path Tracking with Dynamic Control Point Blending*, arXiv 2602.01892 | VERIFIED | 전/후축 제어점 혼합, 실차 |
| 8 | Fazekas et al., *Local Planner-Based Stanley Control in RC Car Racing*, IEEE IV 2024, arXiv 2408.15152 | VERIFIED | Stanley + 적응 look-ahead |
| 9 | Nantabut, *Unscented Transform-based Pure Pursuit*, ICINCO 2024, arXiv 2409.18585 | VERIFIED | 위치추정 불확실성을 UT 로 PP 에 전파 — §2.4 잡음 하한의 확률적 일반화(선행) |
| 10 | Gallina et al., *Sim-to-Real Vision-based Lane Keeping*, arXiv 2409.18097 | VERIFIED | PP + 미분항, 시간지연 안정성으로 튜닝 범위 |
| 11 | Kiemel & Kröger, *Jerk-limited Traversal of 1-D Paths*, ICRA 2024, arXiv 2407.13423 | VERIFIED | 저크제한 경로주행 이진탐색 — JRG 필터의 오프라인 대응 |
| 12 | Covic & Lacevic, *Online Generation of Collision-Free Trajectories in Dynamic Environments*, RA-L 2026, arXiv 2603.00759v3 | VERIFIED (arXiv API 초록·journal_ref) | 저크제한 quintic/quartic 스플라인 온라인 재생성, 유한 구간 조건부 정지-안전 보장. v3 초록에 "frequent target-state changes (up to 1 [kHz])" 가 **있다** — 리뷰의 "초록에 없음"과 v2 의 삭제는 틀렸으므로 "목표 상태 변경 최대 1 kHz 에서 비교"로 복원 |
| 13 | Jäger et al., *Towards Safe Path Tracking Using the Simplex Architecture*, arXiv 2503.10559 | VERIFIED (arXiv API 초록 재확인) | RL 제어기 + 고신뢰 제어기 1개의 Simplex 중재. RPP/DWA/MPPI 는 초록에서 "신뢰할 만하나 동적 조건에 적응하지 못하는 전통 제어기"로 **동기 부분에만** 언급되고, 결과는 "최신 기법과 비슷한 성능"으로만 요약된다(초판의 "3자 중재", v2.1 의 "테스트베드에서 평가된" 둘 다 초록으로 뒷받침되지 않아 정정) |
| 14 | Alwala et al., *Intelligent Control of Differential Drive Robots… EKF*, arXiv 2603.14940 | VERIFIED | RBF 적응 + 피드백선형화 — PID 대안(범위 밖) |
| 15 | Raghavan & Singh, *Do Better Imagined Rollouts Mean Better Robot Control?*, arXiv 2609.02811 | VERIFIED | 평가 설계 교훈(지평·측정주기 명시) |
| 16 | Mishra, Dhar, Majumdar, **Arulselvan**, *Online Joint Calibration of Steering Offset and Planar LiDAR Extrinsics*, arXiv 2608.26789 | VERIFIED | 창고 로봇 CTE 상승 원인 = 캘리브레이션 오프셋 |
| 17 | Macenski et al., *Regulated Pure Pursuit*, Auton. Robots 2023, arXiv 2305.20026 | VERIFIED | 기준선 자체 |
| 18 | Sukhil & Behl, *Adaptive Lookahead Pure-Pursuit for Autonomous Racing*, arXiv 2111.08873 | VERIFIED | 웨이포인트별 look-ahead 오프라인 탐욕 최적화 |
| 19 | Becker et al., *MAP controller*, ICRA 2023, arXiv 2209.04346 | VERIFIED | 타이어 동역학, 고속 영역 |
| 20 | Arslan, *Time Governors for Safe Path-Following Control*, arXiv 2212.01444 | VERIFIED | governor 개념의 출처 |
| 21 | Berscheid & Kröger, *Ruckig*, RSS 2021, arXiv 2105.04830 | VERIFIED | 다차원 저크제한 OTG(외부 라이브러리 미사용 원칙상 직접 구현) |
| 22 | **Ahn, Shin, Kim, Park**, *Accurate Path Tracking by Adjusting Look-Ahead Point in Pure Pursuit Method*, IJAT 22(1):119–129, 2021, DOI 10.1007/s12239-021-0013-7 | VERIFIED (메타데이터 + 초록 요지) | 차량–경로 관계로 look-ahead 점을 휴리스틱 선택해 **코너커팅 없이** 수렴, 실차. CC-PP 와 같은 목표(가장 가까운 경쟁 기법); 우리는 점 선택 대신 현 기하 성분 제거 + 곡률 FF |
| 23 | **Yang et al.**, *An optimal goal point determination algorithm… Pure Pursuit*, Comput. Electron. Agric. 194:106760, 2022 | VERIFIED (Crossref 메타데이터) + 초록 요지(검색 스니펫; 원문 403) | 운전자 전방주시를 모사해 look-ahead **영역 안에서 평가함수(트랙터 위치 예측 모델 기반)로 최적 목표점을 탐색**, 기존 PP 대비 추종오차 20 % 이상 감소(농기계). v2 의 "목표점을 경로 밖에 둔다"는 원문으로 확인되지 않은 리뷰 요약이라 정정. 22 와 함께 CC-PP 의 목표점 재선택형 비교 대상 |
| 24 | **Villagra, Milanés, Pérez, Godoy**, *Smooth path and speed planning for an automated public transport vehicle*, RAS 60(2):252–265, 2012 | VERIFIED (Crossref + 초록 요지) | 곡률·곡률변화율 유계 경로(클로소이드·원호·직선) + 횡가속·조향속도 임계 + **저크제한 속도 프로파일** — JRG 의 $v_{cap}$ 집합과 본질적으로 동일(선행) |
| 25 | **Zanasi, Guarino Lo Bianco, Tonielli**, *Nonlinear filters for the generation of smooth trajectories*, Automatica 36(3):439–448, 2000 | VERIFIED (Semantic Scholar 초록 전문, Crossref) | 도함수 유계 이산시간 가변구조 필터, 최소시간·무오버슈트, 한계 실시간 변경 가능 — §2.7 필터의 원형(우리 폐형식 한 스텝 착지 법칙은 같은 스위칭면의 단순화) |
| 26 | **Haschke, Weitnauer, Ritter**, *On-line planning of time-optimal, jerk-limited trajectories*, IROS 2008 | VERIFIED (메타데이터 + 초록 요지) | 온라인 시간최적 3차 궤적(속도·가속·저크 한계) — 동상 |
| 27 | **Ollero & Heredia**, *Stability analysis of mobile robot path tracking*, IROS 1995 | VERIFIED (메타데이터 + 초록 요지) | 직선·정곡률 경로에서 순수 지연을 포함한 추종기 안정성 해석 — §2.2 명제 2·3 및 내루프 지연 논의의 선행 |
| C1–C6 | Coulter 1992; Hoffmann et al. 2007; Snider 2009; Kanayama et al. 1990; Fox et al. 1997; Macfarlane & Croft 2003 / Åström & Hägglund 1984 / Ziegler & Nichols 1942 | RECALLED | 고전 원전 |

**갭 요약(정직 버전)**: (a) look-ahead 적응은 학습(2,3)·휴리스틱(5)·불확실성 전파(9)가 있고 look-ahead–지연 안정성(27)도 오래된 결과다. **우리가 찾지 못한 것**은 $(\sigma_e,\sigma_\psi)$ 로부터 닫힌 형태로 $L_{\min}(v)$ 를 주는 식과 그 따름정리 $\omega_n\le\sqrt{v\sigma_{\omega}/\sigma_e}$ 이며, 이는 작은 공학적 정리다. (b) 코너커팅 제거는 22·23 이 look-ahead 점 재선택으로, C2·C4 가 곡률 FF 로 이미 다룬다. CC-PP 의 "현 횡좌표에서 기하 성분 $y_g^0$ 를 빼는" 정식화 자체는 인쇄물에서 찾지 못했으나 결과 구조는 Kanayama FF+FB 이므로 **variant** 이다. (c) 저크제한 속도 프로파일(24)과 온라인 3차 필터(25,26)는 있고, Nav2 velocity_smoother 는 저크를 다루지 않는다 — AMR Nav2 체인에 **경로 매개 상한 + 결속점 제동 트리거(폐형식 $D_b$) + 횡저크**를 결합한 것은 공학적 조합.

## 4. 독자 알고리즘 제안: CCRP (Chord-Corrected Regulated Pursuit)

### 4.1 CC-PP: chord 보정 Pure Pursuit

**정의.** 경로 $P(s)$, 접선 $t$, 좌측 법선 $n$, 곡률 $\kappa(s)$, $\kappa'(s)$ (setPlan 시 3점 원적합 + 0.5 m 이동평균). 매 주기:
1. 투영 $s_r = \arg\min_s\|p - P(s)\|$ (직전 인덱스 ±2 m 창; setPlan 직후는 전역 탐색), $e = (p-P(s_r))\cdot n(s_r)$ (§2.5 규약), $\psi = \mathrm{wrap}(\theta - \theta_p(s_r))$.
2. $L = \mathrm{clamp}(|v|\,t_L, L_{\min}(v;K), L_{\max})$ (§2.4; $L_{\min}$ 은 **스케줄된 $K$ 로** 계산 — 아래 5).
   여기서 $v$ 는 프로파일러 미러 상태 $v_f$(§4.2; 측정값은 EKF 지연·잡음이 있어 look-ahead 흔들림 원인).
3. 실제 현 $G$: $s_r$ 이후 첫 경로점 중 $\|G-p\|\ge L$ (선분-원 교점 보간), 로봇좌표 $(x_g,y_g)$. 남은 경로 $< L$ 이면 $G$ = 목표점, $L\leftarrow\|G-p\|$.
4. 기준 현 $G^0$: $s_r$ 이후 첫 점 중 $\|G^0 - P(s_r)\|\ge L$, Frenet 좌표($P(s_r)$ 원점, $t(s_r)$ 축)에서 $(x_g^0, y_g^0)$.
5. 명령과 **속도 의존 게인 스케줄** (스펙 4.5 "게인 파라미터를 속도에 따라 적응"):
$$\boxed{\ \omega^* = v\Big[\kappa(s_r) + K(v)\,\frac{2\,(y_g - y_g^0)}{L^2}\Big],\qquad K(v) = K_0\Big(\frac{L(v)}{|v|\,t_L}\Big)^{\gamma},\ \gamma\in\{0,1,2\}\ },\qquad \kappa_{cmd} = \omega^*/v.$$
유효 피드백 게인 $k_e(v) = 2K(v)v/L(v)^2$, $k_\psi(v) = 2K(v)v/L(v)$ 이 곧 스펙 4.5 의 **속도 적응 게인 스케줄**이다: $v\ge v^*=0.83$ m/s 에서는 $L=vt_L$ 로 $k_\psi = 2K_0/t_L$ 일정·$k_e\propto1/v$, 극 $\omega_n = \sqrt{2K_0}/t_L$ 속도 불변; $v<v^*$ 에서는 $L=L_{\min}(v)$ 로 $\omega_n$ 이 잡음 한계 $\sqrt{v\sigma_\omega/\sigma_e}$ 를 따라 $\propto\sqrt v$ 로 준다. **기본 $\gamma=0$** ($K=K_0=1$, $\zeta=0.707$ 전 속도 일정; $v=0.2$: $L=0.33$, $\omega_n = 0.86$ rad/s, $\sigma_\omega = 0.150$ rad/s = 예산) [c08].
   **정정(v2 → v2.1)**: v2 는 $L = L_{\min}(v;K{=}1)$ 을 고정한 채 $\gamma=1$ 을 기본으로 해 "$v=0.2$: $K=2.05$, $\omega_n=1.23$" 을 제시했으나, 그 점의 명령 잡음은 $\sigma_\omega = 0.31$ rad/s 로 **스스로 정한 예산 0.15 의 2 배**($\gamma=2$: 0.60, 4 배)였다. §2.4 따름정리대로 저속 대역폭은 $K$ 로 올릴 수 없다. $L_{\min}$ 을 스케줄된 $K$ 와 함께 고정점으로 풀면 $\gamma=1$: $v=0.2$ 에서 $L=0.67$, $K=4.0$(상한), $\zeta=1.41$, $\omega_n=0.85$ — 대역폭은 같고 감쇠만 커진다. 그래서 $\gamma\in\{1,2\}$ 는 "저속 과감쇠 선호" **감쇠 스케줄 ablation** 으로만 둔다.

**성질.**
- (P1) 직선: $y_g^0 = 0$ → PP 와 동일. 원: $2y_g^0/L^2 = \kappa$ → $K=1$ 이면 PP 와 항등. CC-PP 는 곡률이 변하는 곳에서만 PP 와 다르다.
- (P2) 완벽 추종 시 $y_g = y_g^0$ → $\omega^* = v\kappa(s_r)$: $C^1$ 경로를 기하적으로 정확히 추종. 잔여 오차는 $\dot\omega$ 한계에서만 발생하며 §4.2 의 $\sqrt{\alpha_{\max}/|\kappa'|}$ 상한이 상쇄.
- (P3, 정정) 선형화: 직선에서는 $y_g - y_g^0 = -(e + L\psi) + O(2)$ 로 $\ddot e + \frac{2Kv}{L}\dot e + \frac{2Kv^2}{L^2}e = 0$, $\zeta = \sqrt{K/2}$, $\omega_n = \sqrt{2K}v/L$ — 이는 **Kanayama 형 FF+FB**($k_2 = 2K/L^2$, $k_3 = 2K/L$)이다. 원에서는 명제 3: 감쇠 $\times c_1$, 강성 $+(1-K)v^2\kappa^2$. **경로 불변성은 $O((L\kappa)^2)$ 근사**이며 거버너 작동점에서 $\zeta_{circ}$: $R\ge1.5$ m 0.68, $R=1$ m 0.66, $R=0.5$ m 0.57 ($K=1$; §2.2 명제 3).
- (P4) 잡음 하한 $L_{\min}(v;K)$ 는 §2.4 식(스케줄된 $K$ 포함, 고정점).

**모드 전환 (시작 회전 / 목표 정렬 / 후진)** — 선형화가 무효한 $|\psi|>45°$ 와 경로 끝을 명시적으로 다룬다:
- *Rotate-to-path*: $|\alpha| = |\operatorname{atan2}(y_g,x_g)| > 0.785$ rad 이고 $|v|<0.1$ 이면 명목 $(0,\ \omega_{rot})$, $\omega_{rot} = \mathrm{sign}(\alpha)\min(\omega_{\max}, \sqrt{2\alpha_{\max}|\alpha|})$; 이력 $|\alpha|<0.3$ 에서 해제. 주행 중 $|\alpha|>0.3$ 이면 $v_{ref}\leftarrow v_{ref}\max(0,\cos\alpha)$.
- *목표 정렬*: 남은 거리 $<$ `goal_dist_tol`(0.10 m) 이면 $v=0$, $\omega = \mathrm{sign}(\Delta\theta)\min(\omega_{\max},\sqrt{2\alpha_{\max}|\Delta\theta|})$ 로 $\theta_{goal}$ 정렬; `SimpleGoalChecker`(xy 0.10 m, yaw 0.05 rad)가 종료 판단. 2 cm/1° 정밀도는 도킹 모듈(마커) 소관. RPP `shouldRotateToGoalHeading` 과 같은 의미.
- *후진* ($v_{\min} = -0.5$ m/s → 후진 세그먼트에서 거버너 상한 $|v|\le0.5$): 경로를 첨점($t_i\cdot t_{i+1}<0$)에서 분할, 세그먼트 진입 시 $t\cdot(\cos\theta,\sin\theta)<0$ 이면 후진 모드: 가상 프레임 $\theta' = \theta+\pi$, $v' = -v > 0$ 에서 위 기하를 그대로 계산하고 $v = -v'$, $\omega$ 는 그대로(요레이트는 프레임 불변). 횡저크 경계는 $|v|$, CTE 부호는 경로 접선 기준으로 불변. RPP `allow_reversing_` 과 동일 의미.

**의사코드 (computeVelocityCommands, 20 Hz):**
```
s_r, e, psi, mode = project_and_mode(path, pose, prev_idx, v_f)     # O(window), 첨점·모드 이력 포함
(v_f, a_f, w_f)   = mirror.state()                                   # 프로파일러 필터 미러(§4.2), |v_f - v_meas| > 0.1 일 때만 재동기
L, K     = schedule(v_f, gamma)       # L = clamp(|v_f| t_L, L_min(v_f;K), L_max), gamma=0 이면 K=K0 (고정점, 상한 K_max=4)
(xg,yg)  = lookahead_robot_frame(path, pose, s_r, L, mode)          # 선분-원 교점, 후진이면 가상 프레임
(xg0,yg0)= lookahead_frenet_frame(path, s_r, L)
kappa_cmd= kappa[s_r] + K*2*(yg - yg0)/L^2                          # 곡률로 전달 (ω 가 아니라)
v_T      = governor.target(s_r, v_f, a_f, e, alpha, speed_limit)    # §4.2: 국소 상한 + 결속점 제동 트리거 (계단 목표)
(v_cmd, w_cmd) = dwa.select(v_f, a_f, v_T, kappa_cmd, mode, costmap) # §4.3: (v_T,κ) 샘플, 미러에서 필터 롤아웃, 출력=목표(v_T', κ' v_T')
mirror.push(v_cmd)                                                   # 프로파일러와 같은 필터로 50 Hz 부분스텝 전진
return TwistStamped{v_cmd, w_cmd}   # → cmd_vel_nav. velocity_profiler_node·safety_node 는 κ 보존 스케일링
```

### 4.2 JRG: 저크 제한 규제 속도 거버너

setPlan 시 격자 $\Delta s = 0.05$ m 마다 (Villagra et al. 2012 의 상한 집합과 동형; 우리는 곡률변화율·횡저크 항을 명시)
$$v_{cap}(s) = \min\Big\{v_{\max},\ \sqrt{\tfrac{a_{lat,\max}}{|\kappa|}},\ \tfrac{\omega_{\max}}{|\kappa|},\ \sqrt{\tfrac{\alpha_{\max}}{|\kappa'|}},\ \Big(\tfrac{j_{lat,\max}}{|\kappa'|}\Big)^{1/3},\ v_{ext}\Big\}$$
(단위 검산: $[\mathrm{m/s^2}\cdot\mathrm m]^{1/2}$, $[\mathrm{rad/s^2}/\mathrm m^{-2}]^{1/2}$, $[\mathrm{m/s^3}\cdot\mathrm m^2]^{1/3}$ 모두 m/s ✓). 권장 $a_{lat,\max}=0.8$, $j_{lat,\max}=1.5$. 예: $R=1.5$ → 1.10 m/s, $R=1$ → 0.89, $R=0.5$ → 0.63.

**결속점 제동 트리거 (v2.1 정정)**. 초판의 후방 패스 $\min(v_f, v_{stop}(D))$ 는 종점 정지만 다루어 비영 상한 앞에서 감속을 시작하지 않았다(2.0→1.10 m/s 에 $d = 2.17$ m 필요, 사다리꼴 1.40 m). v2 의 "앵커 포락선" $v_{prof}(s) = \min\{v_{cap},\ d^{-1}(\cdot)\ \text{원뿔}\}$ 은 두 가지 이유로 여전히 틀렸다 [c06]:
1. **앵커 불완전**: 국소 최소점만 앵커로 쓰면 "계단식으로 떨어진 뒤 완만히 더 내려가는" 상한(국소 최소가 아닌 하강점)에서 $v_{prof}$ 가 2.0→1.0 m/s 로 불연속 → 필터가 **+1.0 m/s 초과**로 진입.
2. **원뿔은 궤적이 아니다**: $d^{-1}(v_t, D)$ 는 "거리 $D$ 앞에서 $a=0$ 으로 출발해 $v_t$ 에 닿을 수 있는 최대 속도"들의 자취라서, 이를 $v_{ref}(s)$ 로 추종하면 원뿔 진입 순간 감속도가 계단($\approx0.9$ m/s²)으로 요구되고 인과적 저크 필터가 $a^2/2j\approx0.2$ m/s 뒤처진다. 모든 격자점을 앵커로 써도 곡선 진입에서 **+0.46 m/s**, 목표점에서 0.3 m/s 로 통과했다. 또 $\sum\Delta s/v_{prof}$ ETA 는 실제보다 **4–11 % 짧다**.

정정 설계 — 상한은 계획 시점에, 감속 시작은 온라인 상태로:
- **setPlan**: $v_{cap}(s)$ 격자와 **결속점 집합** $\mathcal B = \{k : v_{cap}(s_k) = \min_i d^{-1}(v_{cap}(s_i), |s_k - s_i|)\}$ (다른 점의 원뿔에 가려지지 않는 점; 원뿔은 이 가지치기에만 쓴다. $v_{cap}$ 오름차순 방문 + 가지치기로 $O(N|\mathcal B|)$, $N\approx2000$, $|\mathcal B|\lesssim 50$ → ≪ 1 ms). 목표점($v=0$)은 항상 $\mathcal B$ 에 속한다.
- **온라인(20 Hz, 계단 목표 의미)**: $v_T = \min\{v_{cap}(s_r),\ v_{cte}(e),\ v_{ttc},\ v_{mode},\ v_{ext}\}$, 그리고 앞쪽 결속점 $k\in\mathcal B$, $v_{cap,k} < v_f$ 에 대해
$$s_k - s_r \le D_b(v_f, a_f;\ v_{cap,k}) + v_f\,(T_c + T_{link})\ \Rightarrow\ v_T \leftarrow \min(v_T, v_{cap,k}),$$
  $(v_f, a_f)$ = 프로파일러 필터의 **미러 상태**(같은 폐형식 필터를 controller 안에서 50 Hz 부분스텝으로 전진; `odometry/filtered` 와 0.1 m/s 이상 벌어질 때 — `safety_node` 감속·E-stop 개입 등 — 만 재동기해 EKF 잡음이 $L$·트리거에 들어가지 않게 함), $T_c = 50$ ms, $T_{link}$ = `cmd_vel_nav` 전달 지연. 폐형식 제동거리(시간최적 저크제한, [c07]에서 필터 시뮬과 1 mm 이내 일치):
$$D_b = \begin{cases} v t_1 + \tfrac{a t_1^2}{2} - \tfrac{j t_1^3}{6} + d\big(v + \tfrac{a^2}{2j},\ v_t\big), & a\ge0,\ t_1 = a/j\\[4pt] d\big(v_v, v_t\big) - \big(v_v t_r - \tfrac{j t_r^3}{6}\big), & a<0,\ t_r = |a|/j,\ v_v = v + \tfrac{a^2}{2j}\ (\text{이미 감속 중})\end{cases}$$
  ($d$ 는 §2.7 일반 감속거리). 필터는 계단 목표를 받으면 현 상태에서 시간최적 S-curve 로 감속하므로 결속점에 정확히 $v_{cap,k}$ 로 도착한다. 2-rate 시뮬(controller 20 Hz → 프로파일러 50 Hz, 3 경로) [c07]: 전달 지연 ≤ 1 틱(20 ms)에서 상한 초과 ≤ 0.008 m/s(목표점 직전 0.1 m 제외), 목표점 정지 오차 ≤ 3 cm; 지연 60 ms 이면 곡선 진입 +0.05 m/s, 목표점 9–10 cm 초과($T_{link}$ 여유 포함 시 ≤ 7 cm) → 프로파일러는 입력 수신 즉시 처리(이벤트 구동)를 권장.
- $v_{cte}(e) = \mathrm{clip}\big(v_{\max}(1 - \tfrac{|e|-e_0}{e_1-e_0}),\ v_{floor},\ v_{\max}\big)$, $e_0=0.05$, $e_1=0.30$ m, $v_{floor} = 0.25$ m/s (v2 의 하한 $v_{\min}/v_{\max} = -0.25$ 는 $|e| > 0.30$ m 에서 **음의 속도(후진) 지령**, 0.36 m 에서 −0.5 m/s 를 내는 버그였다). `setSpeedLimit()` → $v_{ext}$ (`nav2_msgs/SpeedLimit`). 최종 저크 제한은 `velocity_profiler_node`(§2.7 필터, 계약 50 Hz)가 수행, E-stop 우회.

**ETA**: $T_{pred}$ = 계획 시점에 **같은 거버너 + 필터 코드를 경로 위에서 오프라인 재생**(결정적, 30 m 경로 1500 스텝 ≪ 5 ms)한 도착 시간. $\sum\Delta s/v_{prof}$ 식은 위와 같이 4–11 % 낙관적이라 폐기. 단위테스트: 오프라인 재생 = 온라인 폐루프(이상 플랜트) 1 % 이내. 스펙 4.4 "예측 시간 오차 15 %" 는 외란(재계획·장애물·잡음)만 남는다. 재계획 시 재산출 규약은 §7 Q6.

### 4.3 cmd_vel_nav 소유권: PP-nominal DWA (곡률 운반, 단일 플러그인)

FollowPath 당 Controller 플러그인은 **하나**(검증). 채택안 (A) 계약의 `amr_navigation::PurePursuitController`(`controller_id: PurePursuit`)를 합성 플러그인으로 구현: CC-PP 가 $(v^*, \kappa_{cmd})$ 를 내고, `amr_navigation::DWAController` 와 **공유하는 DWA 코어**(동적 장애물 예측·VO 샘플 제외 포함, components.md §3.3)가 **곡률을 샘플 변수로** 후보 $(v_T, \kappa)$ 를 평가한다:
$$J(v_T,\kappa) = w_o C_{obs} + w_t\Big[\Big(\tfrac{v_T-v^*}{v_{\max}}\Big)^2 + \Big(\tfrac{(\kappa-\kappa_{cmd})L^2}{2e_{ref}}\Big)^2\Big] + w_c\,C_{clear}\,\mathbb 1[d_{\min}<d_c],\qquad e_{ref} = 0.10\ \mathrm m$$
(두 번째 항 = 곡률 차이가 look-ahead 현에서 만드는 횡변위). 롤아웃은 미러 상태 $(v_f,a_f)$ 에서 **§2.7 필터로 $v_T$ 를 향해** 적분하고 $\omega(t) = v(t)\kappa$ ($\alpha_{\max}$ 제한)로 둔다 — 즉 동적 창은 명시적 상자가 아니라 필터의 도달 집합이다. 출력은 목표 $(v_T, \kappa v_T)$ (계단 의미).
**회전 모드(v2.2 보완)**: rotate-to-path·목표 정렬(§4.1, 명목 $v=0$)에서는 $\kappa$ 가 정의되지 않으므로 DWA 코어가 $(0,\omega)$ 후보($\omega$ 격자, $\alpha_{\max}$ 도달집합)로 전환해 회전 풋프린트 충돌만 검사하고 $(0,\omega)$ 를 그대로 내보낸다. 하류 `velocity_profiler_node` 는 $|v_{in}|\le0.05$ 분기($\omega$ 에 $\alpha_{\max}$ 제한만)로 통과시킨다 — 로컬플래너 브리프의 "회전 후보는 $|v|\le0.05$ 에서만" 규칙과 같다. 모드 해제 직후 첫 주기는 $v_f\approx0$ 이므로 $\kappa$ 샘플 격자의 $|\kappa|$ 를 $\omega_{\max}/\max(v_T,0.05)$ 로 자른다.
- **왜 $(v,\omega)$ 상자 창이 아닌가**: (i) 초판의 독립 편차 $(\omega-\omega^*)^2$ 는 상자 창에서 성분별 클리핑이라 가속 구간에서 곡률을 2배 이상 과명령했고, v2 의 가중합 $((v-v^*)/v_{\max})^2 + ((\omega-\kappa v)/\omega_{\max})^2$ 도 $\kappa\,a_{\max}\Delta t > \alpha_{\max}\Delta t$ 이면($|\kappa|>2$, $R<0.5$ m) 직선 $\omega=\kappa v$ 를 떠난다($R=0.4$: 실행 곡률 2.445 vs 2.5; DWPP 사전식 사영은 정확) [c10] — v2 의 "직선이 $V_d$ 와 만나는 한 곡률 정확" 은 이 경우 거짓. (ii) 창으로 자른 속도($v_c + a\Delta t$)를 하류 저크 필터에 넣으면 필터가 매 주기 0.05 m/s 앞의 목표만 보고 가속을 쌓지 못해 **0→2 m/s 에 5.5 s**(최적 2.5 s) 걸린다 [c07].
- **보조정리(전제 명시)**: 명목 후보 $(v^*,\kappa_{cmd})$ 가 허용($V_{adm}$)이고 $C_{obs}=0$, $d_{\min}\ge d_c$ 이면 $\arg\min J = (v^*,\kappa_{cmd})$ (비용 0 인 유일점, 명목 후보를 샘플 격자에 항상 포함). 곡률 보존은 구조적이다: 하류(`velocity_profiler_node`, `safety_node`)가 $\omega_{out} = v_{out}\kappa$ 로 스케일하므로 실행 곡률 $=\kappa$, 예외는 $\alpha_{\max}$ 가 걸릴 때뿐이며 이는 $v_{cap}$ 의 $\sqrt{\alpha_{\max}/|\kappa'|}$ 항으로 드물게 만들고 로그로 빈도를 보고한다. DWPP 의 "창 안에서 직선 $\omega=\kappa v$ 위 점 선택"과 같은 효과를 체인 전체에서 얻는 셈이다.
- **허용성(정정)**: 후보는 롤아웃 길이 $\ell_{roll}$ + **$D_b(v_f,a_f;0)$(저크 포함 정지거리, 2 m/s 에서 2.5 m)** 만큼의 호가 자유일 때만 허용(Fox et al. 1997 허용성의 저크판; `sim_time` 은 허용성과 무관). 비용은 고속에서 약 2배(2–4 ms).
- **로컬 costmap**: $D_{stop}(2.0)+0.3$ m 오버행 $= 2.8$ m, $L_{\max}=1.8$ m 를 덮으려면 **≥ 6 × 6 m rolling window** 가 필요. controller_server 의 로컬 costmap 은 DWA 와 하나를 공유하며 로컬플래너 브리프가 12 × 12 m(240×240 셀)로 잡으므로 충족 — 별도 값을 두지 않는다.
- (B, 계약 방식) BT 가 구간별로 `controller_id` `DWA`/`PurePursuit` 를 지정(components.md §3.3) — 순수 DWA 플러그인은 스펙 4.4 비교에 어차피 필요.

**토픽 체인(계약 이름, 로봇 네임스페이스 상대, 모두 `geometry_msgs/Twist`)**: `controller_server`(PurePursuit 플러그인) → `cmd_vel_nav`(20 Hz; $\omega/v = \kappa$ 운반) → `velocity_profiler_node`(50 Hz: §2.7 폐형식 3차 필터 + κ 보존 $\omega_{out} = v_{out}\kappa$, $\kappa = \omega_{in}/v_{in}$ ($|v_{in}|>0.05$), $|v_{in}|\le0.05$ 이면 $\omega$ 를 $\alpha_{\max}$ 제한만 해 통과 + §2.6.1 몸체 PI, Sub `odometry/filtered`·`payload/mass`) → `cmd_vel_smoothed` → `safety_node`(존 상한·TTC 제한·E-stop; **존 감속은 $v,\omega$ 동일 비율로 적용해 κ 보존**하도록 안전팀에 요청) → `cmd_vel`(50 Hz, 유일 발행자) → `ros_gz_bridge` → Gazebo `DiffDrive`. 대안 T 는 `velocity_profiler_node` 뒤에 바퀴 토크 PI 를 두고 `/model/<r>/joint/{left,right}_wheel_joint/cmd_force` 로 보내는 계약 변경이 필요(§2.6.2, §7 Q3).

### 4.4 선택 확장 (일정 외): 롤아웃 기반 $(L,K)$ 선택

후보 8개 × 15 스텝 전진 시뮬로 $\sum e^2 + \lambda\sum\Delta\omega^2$ 최소 후보 선택(1-파라미터 샘플링 MPC, Sukhil–Behl 의 온라인 대응). CC-PP 의 FF 가 작동하면 이득이 작다. **W1–W3 가 조기 완료될 때만** 착수(리뷰 반영).

### 4.5 신규성의 정직한 포지셔닝

| 모듈 | 가장 가까운 선행연구 | 주장 | 차이 |
| --- | --- | --- | --- |
| CC-PP | Kanayama FF+FB(C4), Stanley FF(C2), **Ahn 2021(22), Yang 2022(23)**, Ollero–Heredia(27) | **variant of prior** | 보정을 "현 횡좌표 기하 성분 제거"로 정식화 → 직선·원에서 PP 항등, 선형화가 Kanayama 게인 $2K/L^2, 2K/L$ 로 환원; 원 위 전개(명제 3 — 정곡률 해석은 27 선행, 설계 도구로만 사용); RSS $L_{\min}$; $K(v)$ 스케줄 |
| JRG | **Villagra 2012(24)**, Zanasi 2000(25), Haschke 2008(26), RPP 규제(17), Kiemel–Kröger(11) | **new combination** | 상한 집합(24)+결속점 제동 트리거(폐형식 $D_b(v,a;v_t)$, 미러 상태)+폐형식 한 스텝 착지 3차 필터(25,26 계열)+횡저크 규칙, 계약 체인(`cmd_vel_nav`→`velocity_profiler_node`) 삽입, 오프라인 재생 ETA |
| PP-nominal DWA | **DWPP(1)**, Nav2 DWB critics, Simplex(13) | **variant of DWPP** | $(v_T,\kappa)$ 곡률 샘플 + 하류 κ 보존으로 DWPP 효과를 체인 전체에 확장, 필터 롤아웃·저크 포함 허용성, "자유 통로 항등" 보조정리(전제 명시) |
| 페이로드 스케줄 모델추종 2-DOF PI | 표준 제어이론(피드포워드 + 모델추종/IMC 형 2-자유도) | engineering adaptation | 작업 이벤트로 알려진 질량(`payload/mass`), S-curve 가속으로 서보 지연 선보상, EKF 경유 엔코더 잡음 포함 피드백; 대안 T 는 토크 극배치 + 가속 피드포워드 |

**기대 효과(v2.1, 폐루프 시뮬로 정량화)**: 코너커팅은 **거버너 작동점**에서 평가해야 한다($R=1.5$ m: $v_{cap}=1.10$ m/s, $L=0.88$ m, 사지타 $L^2/8R = 0.064$ m 는 곡률 불연속 과도의 상계일 뿐 정상상태 오차가 아니다 — 명제 1). 운동학 폐루프 시뮬(직선 6 m → 90° 원호 → 직선 6 m, §4.2 거버너·필터, 20 Hz 제어, $\alpha_{\max}$ 제한, GT=이상 위치) [c12]:

| 곡선 구간(원호 ± 0.5 m) | PP ($L=vt_L$) | RPP 형 ($\mathrm{clamp}(1.5v, 0.3, 0.9)$) | CC-PP ($K=1$) |
| --- | --- | --- | --- |
| $R=1.5$ m 평균 / 최대 $\lvert e\rvert$ | 2.2 / 5.0 cm | 2.8 / 5.2 cm | **0.8 / 1.5 cm** |
| $R=1.0$ m 평균 / 최대 $\lvert e\rvert$ | 3.2 / 5.8 cm | 4.7 / 8.2 cm | **0.8 / 1.7 cm** |

따라서 곡선 평균 CTE 이득은 **1.4–3.9 cm**(최대오차 3.5–6.5 cm) 수준이다. 세 방식 모두 운동학만으로는 10 cm 를 만족하므로 이 이득의 의미는 §2.5 의 **추종기 자체 예산(≤ 2 cm)** 을 곡선에서도 지키는 데 있다(EKF 오차 몫을 남김). v2 의 "≈4–7 cm → 1–3 cm" 는 기준선을 과대평가했으므로 위 표로 대체하고, "> 10 cm → 3–6 cm" 는 철회 상태를 유지한다.

### 4.6 복잡도·실시간 예산 (로봇 1대, 20 Hz)

| 단계 | 복잡도 | 추정 시간 |
| --- | --- | --- |
| setPlan 전처리(곡률, $v_{cap}$, 결속점 가지치기, ETA 오프라인 재생) | $O(N\lvert \mathcal B\rvert )$ + $O(T/\Delta t)$, $N\approx2000$ | < 1 ms + < 5 ms (재계획 시) |
| 제동 트리거(폐형식 $D_b$) | $O(\lvert \mathcal B_{ahead}\rvert )$ | < 0.01 ms |
| 투영·현·모드 | $O(w)$, $w\approx40$ | < 0.02 ms |
| DWA (11 $v_T$ × 21 $\kappa$, 필터 롤아웃 + $D_b$ 호 검사) | ≈ 4.6k–21k 포즈 × ~40 셀 | 1–4 ms |
| 합계 | | ≈ 2–5 ms/주기 → 5대 × 20 Hz ≈ 0.5 코어 (32 스레드) |
| velocity_profiler_node: 3차 필터 + κ 보존 + 몸체 PI (50 Hz) | $O(1)$ | 무시 |

## 5. 평가 계획 (스펙 지표 매핑)

| 스펙 지표 | 정의/측정 | 목표 | 방법 |
| --- | --- | --- | --- |
| CTE 직선 / 곡선 | §2.5, GT(`ground_truth/odom`, OdometryPublisher 50 Hz) 기준 평균 $\lvert e\rvert$; 곡선 = $\lvert\kappa\rvert>0.1$ + 천이대 | ≤ 5 / 10 cm | `cte_logger` 20 Hz CSV, RMSE/최대/P95 병기, 후진 구간 별도 |
| 저크 | **정의: 차체 좌표** $j_{lon} = \dot a_x$, $j_{lat} = d(v\omega)/dt$. 1차 지표 = GT 속도(`ground_truth/odom` twist, 무잡음, 50 Hz) → **영위상 가우시안 커널 $\sigma_t = \sqrt{\ln2}/(2\pi\cdot5\,\mathrm{Hz}) = 26.5$ ms(−3 dB 5 Hz)** → 2회 차분; 2차 지표 = 스펙 IMU `imu/data` $a_x,a_y$(100 Hz) → 동일 커널 → 1회 차분(정적 바이어스는 차분으로 소거). **필터 선택 근거(v2.2)**: 커널이 음이 아니면 필터 출력은 원 저크의 볼록결합이라 $|j_{filt}|\le\max|j|$ 가 보장된다. v2.1 의 2차 Butterworth 는 음의 로브 때문에 한계에 정확히 붙은 S-curve(S1형 0→2→0 m/s)를 **P99 2.067 m/s³ 로 측정해 필터만으로 목표를 어긴다**(가우시안: 2.000) [c17]. **잡음 바닥**: config $\sigma_a = 0.017$ m/s² 백색(원시값; `imu/data` 는 `imu_filter_node` 저역통과 후라 더 낮음) → $\sigma_j = 0.146$ m/s³, P99 = 0.38 m/s³ [c17] (Butterworth 0.087/0.23 [c11]) — 정지 로봇 런으로 실측해 보고(접촉 물리 진동 포함) | P99 ≤ 2 m/s³ 및 최대값 병기 (필터 대역 5 Hz 에서; $t_j = 0.5$ s 저크 평탄부 보존). 프로파일이 한계에 정확히 붙어 PI 보정·접촉 물리 몫의 여유가 0 이므로, GT P99 가 2 를 넘으면 `jerk_limit_scale` 0.9(0→2 m/s 2.56 s)로 낮춘다 | 3차 필터 on/off, 기동 예외 on/off(on 이면 지표 피크 5.2–5.4 m/s³ [c17]) |
| 급가감속 방지 | $\lvert a\rvert$ P99 ≤ 1.0, $\lvert\alpha\rvert$ ≤ 2.0 | 스펙 한계 | GT 속도 미분(동일 필터) |
| 예측시간 오차 (4.4) | $\lvert T_{pred} - T_{real}\rvert/T_{real}$ | ≤ 15 % | JRG ETA vs 실측(재계획 시 재산출 규약 §7) |
| 응답시간 (4.10) | **계약 정의**(sequences.md §1): `assign_task` 요청 시각 → 첫 `cmd_vel` ≠ 0, 50 회 이상 평균/최대. 보조 지표: 같은 시작 → GT $v > 0.02$ m/s | 평균 ≤ 200 ms | 전체 예산은 sequences.md(플릿 20 + 통신 0–100(평균 50) + BT 10 + A* ≤ 50 + 컨트롤러 첫 주기 ≤ 50). **이 모듈 몫**: 컨트롤러 첫 계산(goal 수락 즉시, 2–5 ms) + `velocity_profiler_node`·`safety_node` 50 Hz 홉 각 ≤ 20 ms(평균 10) — sequences.md 예산에 빠져 있어 평균 여유를 0 으로 만든다 → **두 노드는 입력 수신 즉시 처리(이벤트 구동, 홉 < 1 ms)** 를 권장(§7 Q8). 필터 첫 출력은 $j\,dt^2/2 > 0$ 이므로 저크 상승은 계약 지표에 영향 없음. 보조 지표는 저크제한 상승 141 ms(기동 예외 시 56 ms)가 더해져 200 ms 를 넘을 수 있음을 그대로 보고. **스펙 정의와의 충돌(v2.2 명시)**: 스펙 10장 표는 응답시간을 "작업 명령 발행 → **로봇 첫 움직임**" 으로 적는다. 계약은 이를 첫 `cmd_vel` ≠ 0 으로 운용하지만, 물리적 첫 움직임(GT $v>0.005$ m/s)으로 평가하면 평균 예산 200 ms + 구동 지연 ≈ 20 ms + 저크 상승 71 ms ≈ **291 ms**, 홉 이벤트 구동 + 기동 예외를 모두 써도 ≈ 216 ms 로 **200 ms 초과** [c18] — 경로추종 모듈만으로는 해소 불가하므로 §7 Q8 로 아키텍처 결정에 올린다 |
| CPU (4.10) | 5대 동시 | ≤ 80 % | 플러그인 주기 시간 로깅 |
| 테스트 커버리지 (4.10) | `colcon build --cmake-args -DCMAKE_CXX_FLAGS=--coverage` + lcov (C++), `pytest --cov` (Python) | 주요 모듈 ≥ 70 %(코어 라이브러리 목표 80 %) | CI 리포트 |
| 4시간 연속 운전 (4.10) | 5대 작업 루프 4 h | 무충돌·무정지 | 감시: controller_server RSS 메모리·주기 P99, setPlan 시 투영 인덱스 전역 재탐색 횟수, E-stop 반복 후 적분기 리셋 상태, ETA 오차 드리프트, $\kappa=\omega/v$ 의 $v\to0$ 가드(NaN 0건) |
| 게인 튜닝 문서 (4.5) | §2.6.1 절차 ①–⑤ (대안 T 채택 시 §2.6.2) | — | 페이로드 0/2/10/25 kg |
| 안전 구역 (4.7) | 거리 기준 = **footprint 외곽**(config `distance_reference`); 진입 속도 vs $D_E$ (§2.7, $\tau$ 0.15 s, $a$ 1.0) | 접촉 0 | 안전팀(`safety_node`)과 공동 |

**시나리오** (각 20 회, 시드 고정): S1 직선 30 m @2.0; S2 U턴 $R\in\{1.0,1.5,2.0\}$; S3 슬라럼; S4 90° 교차로 3회; S5 페이로드 0/10/25 kg; S6 GT vs EKF 위치추정(추종기 예산 분리); S7 동적 장애물 1.0 m/s 횡단(복귀 ≤ 5 s, 이탈 ≤ 1 m); S8 후진 도킹 접근 3 m; S9 큰 초기 방향오차(120°) 시작.
**기준선**: Nav2 RPP(기본 + 재튜닝 2세트), DWB, **MPPI(주 비교, 설치됨)**, TEB(소스 빌드 best-effort), 팀 순수 PP($K=1$, 보정 없음), Stanley 가상휠베이스.
**Ablation**: chord 보정 on/off, $t_L$ on/off, $K_0\in\{1,1.5,2\}$, $\gamma\in\{0\ (\text{기본}),1,2\}$ (잡음 예산 고정점 포함), 3차 필터 on/off, $(v_T,\kappa)$ 곡률 샘플 vs $(v,\omega)$ 상자 창(가중합), 계단 목표 vs 창 클리핑 출력, 제동 트리거 vs 원뿔 포락선, $v_{cte}$ on/off, 몸체 PI on/off(피드포워드만).
**통계**: 평균±표준편차, 쌍대 Wilcoxon(같은 시드), 성공률·충돌 0건, 최소 여유거리. 결과는 `docs/reports/path_tracking_eval.md`.

## 6. 구현 계획

**언어/패키지**: C++17 (nav2_core 플러그인은 C++ 필수), 코어 라이브러리는 ROS 비의존(gtest 단독), Python 3.10 은 평가·로깅·numpy 레퍼런스 구현.

```
src/amr_navigation/
  include/amr_navigation/tracking/
    path_geometry.hpp        # 투영, Frenet, 곡률/곡률변화율, 선분-원 교점, 첨점 분할
    pure_pursuit.hpp         # PP, CC-PP(K(v), L(v), L_min(v;K) 고정점), 모드 전환, Stanley-virtual(ablation)
    speed_governor.hpp       # v_cap, 결속점 가지치기, d(v,v_t)/d^-1, D_b(v,a;v_t), v_stop 구간별, v_cte, ETA 오프라인 재생
    jerk_filter.hpp          # 폐형식 한 스텝 착지 3차 필터(+미러), 기동 예외(기본 off), 횡저크 규칙, κ 보존 스케일링
    dwa_core.hpp             # (로컬플래너 영역 공동) (v_T,κ) 샘플, 필터 롤아웃, D_b 허용성, 동적 장애물/VO
    pure_pursuit_controller.hpp  # 계약 플러그인 amr_navigation::PurePursuitController (CC-PP + JRG + DWA 중재)
  src/tracking/*.cpp, src/plugins/pure_pursuit_controller.cpp, src/plugins/dwa_controller.cpp
  src/nodes/velocity_profiler_node.cpp   # 계약 노드: cmd_vel_nav → cmd_vel_smoothed, 50 Hz, 3차 필터 + κ 보존 + 몸체 2-DOF PI,
                                         #   Sub odometry/filtered · payload/mass, 입력 수신 즉시 처리(이벤트) + 타이머
  (대안 T 채택 시) src/nodes/wheel_torque_pid_node.cpp  # ≥100 Hz, joint_states in, Float64 cmd_force ×2 out
  amr_navigation/eval/cte_logger.py, jerk_metrics.py, eta_metrics.py, response_time.py
  plugins/amr_controllers.xml ; config/nav2_params.yaml(PurePursuit 블록), velocity_profiler.yaml
  test/test_path_geometry.cpp test_pure_pursuit.cpp test_speed_governor.cpp test_jerk_filter.cpp
       test_velocity_profiler.cpp test_dwa_core.cpp test_cte_metrics.py
```
**CMake**: `amr_tracking_core`(rclcpp 비의존) + `amr_controllers`(플러그인 2개: `DWAController`, `PurePursuitController`) + 노드 1개(`velocity_profiler_node`) + `ament_add_gtest` 6개 + `pluginlib_export_plugin_description_file(nav2_core ...)`; 의존 `nav2_core nav2_costmap_2d nav2_util nav_2d_utils pluginlib tf2_geometry_msgs angles nav_msgs std_msgs amr_msgs`; `-Wall -Wextra -Wpedantic` 무경고(스펙 7장); 커버리지 플래그 CI 잡.

**Nav2 파라미터(초안, `src/amr_navigation/config/nav2_params.yaml` — components.md §6)**:
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["PurePursuit", "DWA"]          # controller_id (components.md §5.3)
    goal_checker_plugins: ["general_goal_checker"]      # Humble 문법(컨테이너 nav2_bringup 기본 params 로 확인)
    general_goal_checker: {plugin: "nav2_controller::SimpleGoalChecker", xy_goal_tolerance: 0.10, yaw_goal_tolerance: 0.05, stateful: true}
    PurePursuit:
      plugin: "amr_navigation::PurePursuitController"
      lookahead_time: 0.8            # s
      max_lookahead_dist: 1.8        # m (공유 로컬 costmap >= 6x6 전제; 현재 12x12)
      damping_gain_K0: 1.0           # zeta = sqrt(K0/2) (직선)
      gain_schedule_gamma: 0         # 기본 0. 1/2 는 감쇠 스케줄 ablation (L_min 은 스케줄된 K 로 고정점), K_max 4.0
      sigma_e: 0.04                  # m (스펙 상한을 보수적 대리값으로)
      sigma_psi_deg: 1.0
      sigma_omega_max: 0.15          # rad/s
      rotate_to_path_min_angle: 0.785
      goal_dist_tol: 0.10
      allow_reversing: true
      max_linear_vel: 2.0            # robot_params.yaml limits.* (아래 5개 동일 출처)
      min_linear_vel: -0.5
      max_angular_vel: 1.5
      max_linear_accel: 1.0
      max_angular_accel: 2.0
      max_linear_jerk: 2.0
      max_lateral_accel: 0.8
      max_lateral_jerk: 1.5
      cte_slowdown: [0.05, 0.30]     # m (e0, e1)
      cte_slowdown_floor: 0.25       # m/s
      brake_trigger_link_delay: 0.02 # s (T_link; 프로파일러 이벤트 구동 전제)
      dwa: {v_samples: 11, kappa_samples: 21, rollout_time: 1.0, admissibility: "jerk_stop_distance",
            w_obs: 1.0, w_track: 0.5, w_clear: 0.2, clear_dist: 0.4, kappa_err_ref: 0.10, output: "target"}
    DWA: {plugin: "amr_navigation::DWAController"}
velocity_profiler_node:
  ros__parameters:
    rate: 50.0
    event_driven: true
    startup_accel_step: 0.0          # 기동 예외 기본 off (§2.7)
    jerk_limit_scale: 1.0            # GT 저크 P99 > 2 이면 0.9 (§5)
    curvature_preserve_min_v: 0.05
    accel_margin_payload: 0.8
    pid: {kp: 0.4, ki: 4.0, structure: "model_following", lag_feedforward: true,
          model_tp: 0.075, model_td: 0.05, antiwindup_tt: 0.1}    # model_*: §2.6.1 ② 식별값으로 덮어씀
# local_costmap: 로컬플래너 브리프 값(12 x 12 m, 0.05 m) 공유 — 이 모듈 요구는 >= 6 x 6 m
```
**Gazebo 측 요구(시뮬 팀, 계약 준수)**: `DiffDrive`(`<topic>/<r>/cmd_vel`) 의 자체 제한기(`max_linear_acceleration`·`max_linear_jerk`·`max_angular_*`)를 프로파일러 한계의 ≥ 1.2 배로 설정; 바퀴 조인트 `<limit effort>`(≈ 6 N·m 후보)·`velocity ≥ 28 rad/s`(§2.1) 정의; `JointStatePublisher`(→ `joint_states` 50 Hz, `wheel_odometry_node` 가 4096 틱·슬립 모델 적용); GT `OdometryPublisher`(→ `ground_truth/odom`); 페이로드 `DetachableJoint` + `payload_manager_node` 의 `payload/mass`. 대안 T 채택 시에만 `ApplyJointForce` ×2 + `cmd_force` 브리지 + `joint_states` ≥ 100 Hz.

**단위 테스트(핵심 명세, v2.1)**: (1) 원 위 $\kappa_{pp} = 1/R$ 오차 < 1e-9; (2) 완벽 추종에서 $y_g - y_g^0 = 0$ (직선·원·클로소이드); (3) **직선, $\psi_0=0$, $L=1.2$ m, $e_0=0.2$ m** 복귀 오버슈트 4.3 ± 0.5 % ($K=1$, 운동학만; 비선형 시뮬 4.35 %) [c09]; $K=2$ 무오버슈트; 통합 시험 기대값(단위테스트 아님): 기본안 모델추종 내루프 4.4–5.1 % [c15], 대안 T $b_\omega=1$ 내루프 3.5 ± 0.5 %(시뮬 3.47 %) [c09]; (3b) 원 $R=1.5$, $L=0.876$, $e_0=\pm1$ cm 두 런의 첫 오버슈트에서 역산한 $\zeta$ 평균 $= 0.676\pm0.02$ (명제 3; 폐루프 시뮬 0.669/0.684 [c16]); (4) 3차 필터(50·100 Hz): 무작위 스텝열에 $|a|\le a_{\max}$, $|j|\le j_{\max}$(수치 여유 1e-9), 0→2 m/s 2.50 ± 0.02 s(기동 예외 off), 오버슈트 0, 정상상태 저크 부호변화 0, 수렴 시 $a=0$ 정확 [c05]; (5) $d^{-1}(d(v,v_t),v_t) = v$ 및 $v_{stop}(D_{stop}(v)) = v$ 를 **두 분기** 격자($v\in[0.05,2]$, $v_t\in[0,v)$)에서, $D_b$ = 필터 시뮬 제동거리 ± 5 mm; (6) 몸체 PI(기본안, 모델추종형): 식별 플랜트 모델에서 위상여유 ≥ 60°, 모델 일치 시 0.15 m/s 스텝·S-curve 종단 오버슈트 ≤ 1 %, 순수지연 제외 추종오차 ≤ 0.03 m/s, $\hat T_p$ ±30 % 오차에서 스텝 오버슈트 ≤ 5 % [c15], anti-windup 해제 오버슈트 < 5 % (대안 T: 0.15 m/s 스텝 ≤ 1 %($b_v=0.5$, 피드포워드 없음), 램프 추종오차 ≤ 0.03 m/s(가속 피드포워드 + $b_v=1$) + 무포화 [c19]); (7) 제동 트리거: 곡선 진입·계단형·완만 하강 상한 3종(§4.2 c06 사례)에서 폐루프 $v(s)\le v_{cap}(s) + 0.01$ m/s(목표점 직전 0.1 m 제외), 목표점 정지 오차 ≤ 0.03 m, ETA 오프라인 재생 = 폐루프 1 % 이내; (8) DWA 자유공간에서 명목 $(v^*,\kappa_{cmd})$ 반환, (8b) 정지→$R=1.5$ 및 $R=0.4$ 가속 진입에서 `cmd_vel_smoothed` 의 $|\omega/v - \kappa_{cmd}| < 10^{-6}$ ($v>0.05$, $\alpha$ 비포화 구간), (8c) 계단 목표 출력으로 0→2 m/s ≤ 2.6 s; (9) CTE 부호(좌측 +)·구간 분류; (10) 후진 모드 거울 대칭·$|v|\le0.5$; (11) rotate-to-path 이력; (12) $v_{cte}\ge v_{floor}>0$ (후진 지령 없음).

**일정(4주)**: W1 코어 기하·CC-PP·거버너(결속점·$D_b$·ETA 재생)·폐형식 필터 + gtest + numpy 교차검증(`checks/`) → W2 `PurePursuitController` 플러그인·`velocity_profiler_node`(필터·κ 보존·몸체 PI)·DiffDrive 한계 설정·단일 로봇 → W3 곡률 샘플 DWA 통합·모드 전환·시나리오 자동화·로깅·커버리지 CI → W4 기준선(RPP/DWB/**MPPI**; TEB best-effort) 비교·페이로드·4시간 런·리포트·피어리뷰 문서(`docs/algorithms/path_tracking.md`). §4.4 와 대안 T 는 일정 외.

## 7. 미해결 질문

1. **CTE 기준**: GT 기준 5 cm 가 EKF 바이어스로 미달하면 "추정 포즈 기준 CTE" 병기 여부.
2. **TEB 기준선**: Humble 바이너리 없음(components.md §8 과 동일) → 소스 빌드 브랜치 합의; MPPI 를 주 비교로 승격.
3. **바퀴 구동 방식**: 계약은 `DiffDrive`(기본안 §2.6.1). 토크 수준 PID·질량 효과를 더 직접 보이려면 대안 T(`ApplyJointForce`, `joint_states` ≥ 100 Hz)로 계약 변경 — 아키텍처 결정 필요. 어느 쪽이든 바퀴 조인트 **effort 한계(τ_max)** 가 URDF 에 정의되어야 페이로드가 동역학에 드러난다.
4. **곡률 소스**: 글로벌 평활 경로가 $G^2$ 인지(아니면 $\kappa'$ 상한이 속도를 과도히 깎음).
5. **κ 보존 요청**: `safety_node` 존 감속·TTC 제한을 $v,\omega$ 동일 비율로 적용할 것(안전팀). 거리 기준점은 config `footprint_edge` 로 확정됨.
6. **ETA 15 %** 정의(계획 시점 vs 재계획 반영) — 우리는 재계획마다 오프라인 재생으로 재산출.
7. **페이로드 질량 변경 방법**(DetachableJoint vs 관성 수정).
8. **응답시간 예산**: 운용 정의는 sequences.md §1(`assign_task` → 첫 `cmd_vel` ≠ 0). 남은 문제 두 가지 — (a) 그 예산표에 `velocity_profiler_node`·`safety_node` 의 50 Hz 홉(각 ≤ 20 ms)이 빠져 있다 → 이벤트 구동 처리로 합의하거나 예산표 갱신; (b) **스펙 10장 표의 문언은 "로봇 첫 움직임"** 이라, 심사가 물리적 움직임으로 해석하면 평균 ≈ 216–291 ms 로 요구(200 ms)를 넘는다 [c18]. 운용 정의를 스펙 해석으로 명시 합의하거나, 상류 예산(통신 지연 시뮬 평균 50 ms, A* 50 ms)을 줄여야 한다 — 경로추종 쪽 선택지는 기동 예외(저크 지표 피크 5.2–5.4 m/s³ 대가)뿐이다.
9. **DiffDrive 제한기**: 프로파일러보다 느슨하게(≥ 1.2 배) 설정하는 데 시뮬 팀 동의 필요(구속되면 미러 상태와 실제가 어긋남).

## 8. 참고문헌

1. [VERIFIED] F. Ohnishi, M. Takahashi, "DWPP: Dynamic Window Pure Pursuit Considering Velocity and Acceleration Constraints," arXiv:2601.15006, 2026. https://arxiv.org/abs/2601.15006
2. [VERIFIED] M. Elgouhary, A. S. El-Wakeel, "Dynamic Lookahead Distance via Reinforcement Learning-Based Pure Pursuit for Autonomous Racing," arXiv:2603.28625, 2026. https://arxiv.org/abs/2603.28625
3. [VERIFIED] M. Elgouhary, A. S. El-Wakeel, "Learning to Tune Pure Pursuit in Autonomous Racing: Joint Lookahead and Steering-Gain Control with PPO," arXiv:2602.18386, 2026. https://arxiv.org/abs/2602.18386
4. [VERIFIED] H. Gholampour, L. E. Beaver, "Reachability-Aware Time Scaling for Path Tracking," arXiv:2604.00439, 2026. https://arxiv.org/abs/2604.00439
5. [VERIFIED] N. Promkaew, N. Junhuathon, A. Phuphaphud, P. Kulvanit, S. Sukpancharoen, "Enhanced pure pursuit with dynamic steering control for autonomous mobile robots and application to safe navigation in chemical plants," Scientific Reports 16:8820, 2026, DOI 10.1038/s41598-026-38695-1. https://pmc.ncbi.nlm.nih.gov/articles/PMC12982613/
6. [VERIFIED] H. Jung, "Model-Based Hybrid Control of Pure Pursuit and Stanley Methods for Vehicle Path Tracking," Sensors 25(20):6491, 2025. https://pmc.ncbi.nlm.nih.gov/articles/PMC12567833/
7. [VERIFIED] A. Lombard, F. Perronnet, N. Gaud, A. Abbas-Turki, "Path Tracking with Dynamic Control Point Blending for Autonomous Vehicles: An Experimental Study," arXiv:2602.01892, 2026. https://arxiv.org/abs/2602.01892
8. [VERIFIED] M. Fazekas et al., "Evaluation of Local Planner-Based Stanley Control in Autonomous RC Car Racing Series," IEEE IV 2024, arXiv:2408.15152. https://arxiv.org/abs/2408.15152
9. [VERIFIED] C. Nantabut, "Unscented Transform-based Pure Pursuit Path-Tracking Algorithm under Uncertainty," ICINCO 2024, arXiv:2409.18585. https://arxiv.org/abs/2409.18585
10. [VERIFIED] A. Gallina, M. Grandin, A. Cenedese, M. Bruschetta, "A Sim-to-Real Vision-based Lane Keeping System for a 1:10-scale Autonomous Vehicle," arXiv:2409.18097, 2024. https://arxiv.org/abs/2409.18097
11. [VERIFIED] J. C. Kiemel, T. Kröger, "Jerk-limited Traversal of One-dimensional Paths and its Application to Multi-dimensional Path Tracking," ICRA 2024, arXiv:2407.13423. https://arxiv.org/abs/2407.13423
12. [VERIFIED] N. Covic, B. Lacevic, "Online Generation of Collision-Free Trajectories in Dynamic Environments," IEEE RA-L 2026 (arXiv journal_ref), arXiv:2603.00759v3. https://arxiv.org/abs/2603.00759
13. [VERIFIED] G. Jäger, N.-J. Friedrich, H. Petersen, B. Noack, "Towards Safe Path Tracking Using the Simplex Architecture," arXiv:2503.10559, 2025. https://arxiv.org/abs/2503.10559
14. [VERIFIED] A. Alwala, Y. Hu, G. da Silva Lima, W. M. Bessa, "Intelligent Control of Differential Drive Robots Subject to Unmodeled Dynamics with EKF-based State Estimation," arXiv:2603.14940, 2026. https://arxiv.org/abs/2603.14940
15. [VERIFIED] D. Raghavan, A. Singh, "Do Better Imagined Rollouts Mean Better Robot Control? A Controlled Study of World-Model Evaluation Under Feedback," arXiv:2609.02811, 2026. https://arxiv.org/abs/2609.02811
16. [VERIFIED] S. Mishra, A. Dhar, S. Majumdar, N. Arulselvan, "Online Joint Calibration of Steering Offset and Planar LiDAR Extrinsics for Wheeled Mobile Robots," arXiv:2608.26789, 2026. https://arxiv.org/abs/2608.26789
17. [VERIFIED] S. Macenski, S. Singh, F. Martín, J. Ginés, "Regulated Pure Pursuit for Robot Path Tracking," Autonomous Robots, 2023, arXiv:2305.20026. https://arxiv.org/abs/2305.20026
18. [VERIFIED] V. Sukhil, M. Behl, "Adaptive Lookahead Pure-Pursuit for Autonomous Racing," arXiv:2111.08873, 2021. https://arxiv.org/abs/2111.08873
19. [VERIFIED] J. Becker et al., "Model- and Acceleration-based Pursuit Controller for High-Performance Autonomous Racing," ICRA 2023, arXiv:2209.04346. https://arxiv.org/abs/2209.04346
20. [VERIFIED] Ö. Arslan, "Time Governors for Safe Path-Following Control," arXiv:2212.01444, 2022. https://arxiv.org/abs/2212.01444
21. [VERIFIED] L. Berscheid, T. Kröger, "Jerk-limited Real-time Trajectory Generation with Arbitrary Target States," RSS 2021, arXiv:2105.04830. https://arxiv.org/abs/2105.04830
22. [VERIFIED, Crossref·Semantic Scholar 메타데이터 + 검색 초록 요지] J. Ahn, S. Shin, M. Kim, J. Park, "Accurate Path Tracking by Adjusting Look-Ahead Point in Pure Pursuit Method," Int. J. Automotive Technology 22(1):119–129, 2021, DOI 10.1007/s12239-021-0013-7. https://link.springer.com/article/10.1007/s12239-021-0013-7
23. [VERIFIED, Crossref 메타데이터 + 검색 초록 요지(원문 403)] Y. Yang, Y. Li, X. Wen, G. Zhang, Q. Ma, S. Cheng, J. Qi, L. Xu, L. Chen, "An optimal goal point determination algorithm for automatic navigation of agricultural machinery: Improving the tracking accuracy of the Pure Pursuit algorithm," Computers and Electronics in Agriculture 194:106760, 2022, DOI 10.1016/j.compag.2022.106760. https://www.sciencedirect.com/science/article/abs/pii/S0168169922000771
24. [VERIFIED, Crossref 메타데이터 + 검색 초록 요지] J. Villagra, V. Milanés, J. Pérez, J. Godoy, "Smooth path and speed planning for an automated public transport vehicle," Robotics and Autonomous Systems 60(2):252–265, 2012, DOI 10.1016/j.robot.2011.11.001. https://www.sciencedirect.com/science/article/abs/pii/S092188901100203X
25. [VERIFIED, Semantic Scholar 초록 전문 + Crossref] R. Zanasi, C. Guarino Lo Bianco, A. Tonielli, "Nonlinear filters for the generation of smooth trajectories," Automatica 36(3):439–448, 2000, DOI 10.1016/S0005-1098(99)00164-8. https://www.sciencedirect.com/science/article/abs/pii/S0005109899001648
26. [VERIFIED, Crossref 메타데이터 + 검색 초록 요지] R. Haschke, E. Weitnauer, H. Ritter, "On-line planning of time-optimal, jerk-limited trajectories," IEEE/RSJ IROS 2008, pp. 3248–3253, DOI 10.1109/IROS.2008.4650924. https://ieeexplore.ieee.org/document/4650924
27. [VERIFIED, Crossref 메타데이터 + 검색 초록 요지] A. Ollero, G. Heredia, "Stability analysis of mobile robot path tracking," IEEE/RSJ IROS 1995, vol. 3, pp. 461–466, DOI 10.1109/IROS.1995.525925. https://ieeexplore.ieee.org/document/525925
28. [VERIFIED] Nav2 RPP README/소스·헤더 (humble). https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/README.md , https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_regulated_pure_pursuit_controller/include/nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp
29. [VERIFIED] Nav2 controller_server / velocity_smoother / collision_monitor 소스·설치 헤더 (humble 1.1.20). https://github.com/ros-navigation/navigation2/tree/humble
30. [VERIFIED] Gazebo Fortress `DiffDrive`, `JointController`, `ApplyJointForce` 문서/소스. https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1DiffDrive.html , https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/apply_joint_force/ApplyJointForce.cc
31. [VERIFIED] ros_gz_bridge 메시지 매핑 (humble). https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/README.md
32. [RECALLED] R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," CMU-RI-TR-92-01, 1992.
33. [RECALLED] G. M. Hoffmann, C. J. Tomlin, M. Montemerlo, S. Thrun, "Autonomous Automobile Trajectory Tracking for Off-Road Driving," ACC 2007.
34. [RECALLED] J. M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking," CMU-RI-TR-09-08, 2009.
35. [RECALLED] Y. Kanayama, Y. Kimura, F. Miyazaki, T. Noguchi, "A Stable Tracking Control Method for an Autonomous Mobile Robot," ICRA 1990.
36. [RECALLED] D. Fox, W. Burgard, S. Thrun, "The Dynamic Window Approach to Collision Avoidance," IEEE RA Magazine, 1997.
37. [RECALLED] S. Macfarlane, E. A. Croft, "Jerk-bounded manipulator trajectory planning," IEEE T-RA 2003; K. J. Åström, T. Hägglund, Automatica 1984; J. G. Ziegler, N. B. Nichols, Trans. ASME 1942.
38. [프로젝트 문서, 정합 기준] `config/robot_params.yaml`, `config/sensors.yaml`, `config/ekf.yaml`, `docs/architecture/components.md` §3.3·§4.1·§5·§6, `docs/architecture/sequences.md` §1–2, 로컬플래너 브리프(로컬 costmap 12×12 m).

## 9. 리뷰 반영 이력

### 9.1 v1 → v2 (개정자 기록, 원문 유지; **[→v2.1]** 표시는 §9.2 에서 정정된 항목)

**수학 오류 — 모두 수치 재검증 후 수정**
- CTE 부호: $e = (t\times(p-P))\cdot\hat z = (p-P)\cdot n$, $n=(-t_y,t_x)$ 로 통일(§2.2, §2.5, §4.1, 테스트 9). 초판 식은 부호가 반대였다.
- (P3) 경로 불변성 철회 → 명제 3(원 위 정확 전개, 유한차분 검증: $\partial y_g/\partial e = -0.431$, $\partial y_g/\partial\psi=-1.353$ at $R=1.5, L=1.6$)로 대체; $\zeta_{circ} = Kc_1/\sqrt{2K+(1-K)L^2\kappa^2}$; 테스트 (3) 을 직선·$\psi_0=0$·$L\ge1$ m 로 한정, (3b) 원 감쇠 테스트 추가; 선형화가 Kanayama FF+FB 임을 명기.
- PI: 폐루프 영점에 의한 13.5/20.8 % 오버슈트 확인(scipy) → 2-DOF PI $b_v=0.5$(0 % at $\zeta=1$), 식별 스텝 0.2 m/s(무포화), 수락 기준 재정의; "35 % 대역" → $\omega_c$ −20 %, $\zeta\omega_c$ −36 % **[→v2.1: config 질량으로 −19/−34 %, 식별 스텝 0.15 m/s, Padé 결론 정정]**; 외루프 1.77 rad/s(0.28 Hz), $\omega_c/\omega_n=3.4$; 내루프 위상지연 $b=1$: 2.3°, $b=0.5$: 16.4° 및 Padé 영향($\zeta$ 0.707→0.69) 정량화.
- 후방 패스: $v_{stop}$ 기반 → 앵커 기반 일반 S-curve 포락선 $d^{-1}(v_t, D)$; ETA 를 프로파일 + 오프라인 필터 통과로 검증. **[→v2.1: 포락선·ETA 식 폐기, 결속점 제동 트리거]**
- 곡률 보존: DWA 추종 비용을 DWPP 형($\omega-\kappa_{cmd}v$)으로 교체, 거버너·collision_monitor 도 κ 보존 스케일링, 보조정리 전제 명시, 가속 테스트 (8b) 추가. **[→v2.1: $(v_T,\kappa)$ 샘플·계단 목표, 계약 노드 이름]**
- 응답시간: 기동 상승 141 ms 확인 → 기동 예외 $a_0=0.3$ m/s²(56 ms, 적재물 안정성 무관)로 예산 ≈156 ms; 글로벌 계획 포함 시 불가함을 §7 Q8 로 명시. **[→v2.1: 계약 정의(첫 `cmd_vel`≠0)로 교체, 기동 예외 기본 off]**
- $D_{stop}/v_{stop}$ 구간별 폐형식($v<0.5$ m/s: $v^{3/2}/\sqrt j$, $D<0.25$ m: $(D\sqrt j)^{2/3}$), 테스트 (5) 두 분기.
- Stanley: 모든 속도에서 $\zeta\ge1$(AM–GM), $\omega_n\propto\sqrt v$ — 논거를 "대역폭 불변"으로 교체, $k_s$ 유지.
- 횡저크: 차체 정의 명시, 경계 0 클램프, 종방향 우선 규칙, $|v|<0.1$ 대안.
- E-stop: 우회 시 제동 권한(토크 2.1/3.2 m/s²) + 50 ms 지연으로 재계산(1.10/0.30/0.09/0.02 m), 기준점 footprint 외곽 명시. **[→v2.1: config τ 0.15 s·a 1.0 → 2.30/0.65/0.20/0.05 m]**
- $L_{\min}$: $\sigma_\psi$ 항 RSS 유지, $1/L$ 의 2차식 해, $\sigma_e$ 는 보수적 대리값임을 명시.
- 기대 효과: 거버너 작동점(1.10 m/s, $L=0.88$)에서 사지타 0.064 m vs RPP 0.068 m → 곡선 CTE 이득을 2–5 cm 로 하향. **[→v2.1: 폐루프 시뮬 표, 평균 이득 1.4–3.9 cm]**

**제안 재포지셔닝**: CC-PP = variant of prior(Kanayama/Ahn 2021/Yang 2022 명시); JRG = new combination(Villagra 2012·Zanasi 2000·Haschke 2008 인용, 두 오류 수정); PP-nominal DWA = variant of DWPP(비용 채택); §4.4 는 일정 외; MPPI 를 주 비교로. 갭 (a) 는 "닫힌 형태 $L_{\min}$ 식을 찾지 못함"으로 축소하고 Ollero & Heredia·Nantabut 인용.

**스펙 갭 보완**: $K(v)$ 스케줄($\gamma$) **[→v2.1: 기본 γ=1 은 잡음 예산 2배 위반, γ=0 기본]**; rotate-to-path·목표 정렬·후진 모드; 엔코더 피드백(4.1)과 슬립 처리 근거 **[→v2.1: 슬립은 측정 잡음으로 루프에 들어옴, 계약 경로는 EKF]**; 커버리지 70 %·4시간 런과 장기 운전 점검 항목; 저크 지표 정의·대역(5 Hz)·잡음 바닥; DWA 허용성 = $D_{stop}(v)$; 로컬 costmap 6×6 m; 안전거리 기준점.

**인용 수정**: ref 16 저자 추가(Arulselvan); ref 13 표현 완화; ref 5 플랫폼(Ackermann)·수치(74/82–87/68–70 %) 정정, "현실성 근거" 격하; ref 12 "1 kHz" 삭제 **[→v2.1: v3 초록에 있음, 복원]**; Ahn 2021, Yang 2022, Villagra 2012, Zanasi 2000, Haschke 2008, Ollero & Heredia 1995 추가(모두 fetch 로 존재·요지 확인; Springer/ScienceDirect 본문은 403/IdP 리다이렉트로 메타데이터·검색 초록으로 검증).

**리뷰어와 다른 점(근거 포함)**
1. 리뷰어의 일반 감속거리 $d(v,v_t) = (v^2-v_t^2)/2a + (v-v_t)a/2j$ 는 **부호 오류**다. 대칭 S-curve 의 평균속도가 $(v+v_t)/2$ 이므로 $(v+v_t)a/2j$ 가 맞다(수치적분: 2.0→1.10 m/s 에서 2.17 m; 리뷰어 식은 1.62 m). $v_t=0$ 에서 두 식이 일치해 눈에 띄지 않았다.
2. $v_{stop}$ 분기 임계는 $a^3/(2j^2)$ 가 아니라 $D_{stop}(a^2/j) = a^3/j^2 = 0.25$ m 이다(리뷰어도 0.25 m 로 썼으나 식이 달랐다).
3. 내루프 위상지연 16° 는 $b=0.5$ 일 때이며, PP 극에 미치는 영향은 Padé-1 로 $\zeta$ 0.707→0.69 로 작다. 그래도 $\omega$ 루프는 $b_\omega=1$(2.3°)로 두어 문제를 회피했다. **[→v2.1: $b=0.5$ 내루프는 정확히 $6/(s+6)$ 이고 지배극 ζ 0.59·오버슈트 6.4 % 로 영향이 크다 — 리뷰어 우려가 옳았다. 결론($b_\omega=1$)은 유지]**
4. PP-DSC(ref 5)의 look-ahead 상수는 리뷰어(1.5 m + 3(v−1), 상한 11.5 m)와 본인 재fetch(0.5–4.0 m, 0.5–5 m/s)가 다르다 — 어느 쪽도 본문 표에 확정 인용하지 않고 "속도 선형(하·상한)"으로만 기술했다. **[→v2.1: 본문 재확인 결과 둘 다 있다(Eq. 20 개념식, Eq. 29/표 3 구현값)]**
5. Comput. Electron. Agric. 2022(ref 23)는 리뷰어가 403 으로 못 열었지만 Semantic Scholar 메타데이터와 검색 요지("look-ahead 점을 경로 밖에 두어 코너커팅 해결")로 확인해 추가했다. **[→v2.1: 확인 가능한 초록 요지는 "look-ahead 영역에서 평가함수로 최적 목표점 탐색, 오차 20 % 이상 감소"이며 "경로 밖"은 확인되지 않아 정정]**

### 9.2 v2 → v2.1 감사 (2026-09-22) — critique.json 49 개 항목 전수 대조

**[→v2.2]** 이 표의 판정 중 M3·M6·M11·PV4·PV6·SG2·SG6·SG7·C3·C4·RR5·RR8·RR13 은 §9.3 최종 점검에서 재개되어 추가 수정되었다.

개정자가 자체 점검 전에 중단되어, critique.json 의 모든 항목(수학 13, 제안 판정 6, 스펙 갭 10, 인용 7, 필수 수정 13)을 문서와 대조하고 수치 항목은 `checks/c01–c12` 로 재계산, 인용 항목은 재fetch 했다. 판정: **A** = v2 에서 올바르게 해결(표기 수준 수정만), **B** = v2 해결이나 수치·정합 오류를 v2.1 에서 수정, **C** = v2 가 틀리게/부분적으로 해결 → v2.1 에서 실질 수정. **최종 49/49 해결** (A 20, B 11, C 18).

| 항목 | 판정 | v2.1 조치 (근거) |
| --- | --- | --- |
| M1 CTE 부호 | A | 확인만 [c01] |
| M2 원 위 선형화 (P3) | B | 식·유한차분 일치 확인; $\zeta(L{=}0.876)$ 0.677→0.676; "거버너 작동점 $L\kappa\le0.59$" 는 $R\ge1.5$ m 에서만 참 → $R$=1/0.5 m 값(0.72/1.18, ζ 0.66/0.57) 명시 [c01] |
| M3 PI 영점 오버슈트 | B | 13.5/20.8/0 % 확인; config 질량($M_v$ 48.6 kg)으로 게인 재계산; 식별 스텝 0.2→0.15 m/s(0.2 는 초기가속 1.2 m/s² > $a_{\max}$) [c03] |
| M4 후방 패스 | C | $d(v,v_t)$ 식은 옳음(리뷰어 식이 틀림, 수치적분 일치). 그러나 v2 앵커 포락선은 앵커 불완전(+1.0 m/s 초과)·원뿔≠궤적(+0.46 m/s 초과, 목표점 0.3 m/s 통과)·ETA 4–11 % 낙관 → **결속점 + 폐형식 $D_b$ 제동 트리거 + 오프라인 재생 ETA** 로 교체 [c06, c07] |
| M5 곡률 과명령 | C | v2 가중합 비용은 $R<0.5$ m 에서 직선을 벗어남(κ 2.445 vs 2.5), 창 클리핑 출력은 가속을 5.5 s 로 늦춤 → $(v_T,\kappa)$ 곡률 샘플·계단 목표·하류 κ 보존, 보조정리 재기술 [c10, c07] |
| M6 응답시간 예산 | C | 계약 정의(sequences.md §1: 첫 `cmd_vel`≠0) 채택, 프로파일러·safety 50 Hz 홉을 예산에 추가·이벤트 구동 권고, 기동 예외 기본 off(on 이면 저크 지표 피크 4.1–4.3 m/s³) [c02, c11] |
| M7 $D_{stop}$ 구간별 | A | 확인; "12 % 과대" → 11 % [c02] |
| M8 Stanley 감쇠 | A | 확인; $k_s\ge0$ 전 범위로 일반화 [c01] |
| M9 횡저크 정의 | A | $\lvert v\rvert <0.1$ 분기 모순 제거, 종방향 우선 규칙 $\rho=0.5$ (v2 규칙은 $\dot\omega$ 예산 0) |
| M10 E-stop 거리 | C | v2 는 지연 50 ms(config `reaction_latency` 0.15 s 와 불일치), 1.10 m 재현 불가(1.06) → config 모델 2.30/0.65/0.20/0.05 m, 0.3 m 내 정지 한계 0.64 m/s, 토크 최선 0.84 m/s; 기준점은 config 로 확정 [c02] |
| M11 대역 문장·위상지연 | B | −19/−34 %(config 질량); v2 의 "Padé → ζ 0.69" 는 오류(정확히 1차 지연 → ζ 0.59, 6.4 %) [c03] |
| M12 $L_{\min}$ RSS | A | 0.33/0.52/0.75/1.09 m, σψ +5 % 확인; 따름정리 $\omega_n\le\sqrt{v\sigma_\omega/\sigma_e}$ 추가 [c08] |
| M13 기대 효과 | B | "4–7 → 1–3 cm" 근거 없음 → 폐루프 시뮬: 평균 PP 2.2/RPP형 2.8 → CC-PP 0.8 cm ($R$=1.5), 3.2/4.7 → 0.8 cm ($R$=1) [c12] |
| PV1 CC-PP overclaimed | A | variant of prior·Kanayama·Ahn/Yang 명시 확인; 이득 정량화(M13) |
| PV2 JRG flawed | C | v2 필터 의사코드가 $\lvert j\rvert $ 5.8/8.9 m/s³ 로 한계 위반·오버슈트 → 폐형식 한 스텝 착지 법칙($\lvert j\rvert \le2$ 정확, 0→2 m/s 2.500 s); 포락선 교체(M4) [c04, c05] |
| PV3 DWA flawed | C | M5 와 동일 조치 |
| PV4 페이로드 PI | B | 계약(`velocity_profiler_node`, DiffDrive, `odometry/filtered`)에 맞춘 기본안 추가(PM 73–91°), 토크 설계는 대안 T 로 [c03] |
| PV5 롤아웃 선택 | A | 일정 외 유지 |
| PV6 교과서 결과 | A | Ollero & Heredia 초록 재확인(직선·정곡률 + 순수 지연) |
| SG1 $K(v)$ 스케줄 | C | v2 기본 γ=1 은 $v=0.2$ 에서 $\sigma_\omega$ 0.31 rad/s(예산 0.15 의 2배) → γ=0 기본, 유효 게인 $k_e(v),k_\psi(v)$ 를 스펙 4.5 스케줄로 명시, γ=1/2 는 고정점 $L_{\min}$ 과 함께 감쇠 ablation [c08] |
| SG2 시작·목표 회전 | A | 확인 |
| SG3 후진 | A | 후진 $\lvert v\rvert \le0.5$ 상한 명시 |
| SG4 엔코더 피드백 | C | v2 의 "슬립이 속도루프에 안 나타남"은 sensors.yaml 모델과 모순(측정 변위 잡음, 2 m/s 에서 σ 0.014 m/s) → 계약 경로(`wheel_odom`→EKF→`odometry/filtered`)로 루프에 포함, 대안 T 수치 정정 [c03] |
| SG5 커버리지·4시간 | A | 확인 |
| SG6 200 ms 응답 | C | M6 와 동일 |
| SG7 저크 지표 | B | GT 소스 `ground_truth/odom` 50 Hz(계약), 잡음 바닥 config σ 0.017 → σj 0.087, P99 0.23 [c11] |
| SG8 DWA 허용성 | A | $D_b(v,a;0)$(현 상태 기준)으로 일반화 |
| SG9 로컬 costmap | B | ≥ 6×6 m 요구 유지, 공유 로컬 costmap(로컬플래너 브리프 12×12 m)과 모순되던 별도 6×6 설정 제거 |
| SG10 안전거리 기준점 | A | config `distance_reference: footprint_edge` 로 확정 |
| C1 스팟체크 | A | ref 1/13/16 초록 arXiv API 로 재확인 |
| C2 ref 16 저자 | A | 4 저자 재확인 |
| C3 ref 13 표현 | A | 초록과 일치 확인 |
| C4 ref 5 | C | PMC 본문 재확인: car-like 전륜조향, look-ahead 두 식 모두 존재, 74/82(87)/70 %, 공장 시뮬에서 PP 가 15.6 % 우세 추가, Sci. Rep. 16:8820 |
| C5 ref 12 "1 kHz" | C | **리뷰어 오류**: v3 초록에 "up to 1 [kHz]" 존재 → 복원 |
| C6 누락 선행연구 | C | 6 편 DOI·저자·권호 Crossref 로 전부 확인; ref 23 설명("경로 밖 목표점")이 확인되지 않아 초록 요지로 정정 |
| C7 RECALLED | A | 변경 없음 |
| RR1 | A | = M1 |
| RR2 | B | = M2, 테스트 (3) 내루프 기대값 5.0 → 3.5 ± 0.5 % [c09], (3b) 0.676 |
| RR3 | C | = M5 |
| RR4 | C | = M4 + 테스트 (5)(7) 재정의 |
| RR5 | B | = M3/M11/SG4 + 계약 기본안 |
| RR6 | C | = PV2 (필터 교체, 부호변화 테스트 0 회) + M9 |
| RR7 | C | = M10 |
| RR8 | C | = M6 |
| RR9 | A | = M12 + 갭 (a) 확인 |
| RR10 | A | = M8 |
| RR11 | C | = SG1 (+ SG2/SG3 확인) |
| RR12 | B | 관련연구 확인, ref 5/12/23 정정, 기대 효과 시뮬 |
| RR13 | B | 저크 지표 소스·잡음, 허용성, costmap 공유, 커버리지·4 h, §4.4 일정 외, MPPI 주 비교 — 확인 및 소폭 정정 |

**계약·설정 정합 수정(critique 밖)**: 노드·토픽 이름을 components.md 로 통일(`cmd_vel_nav`, `velocity_profiler_node`, `cmd_vel_smoothed`, `safety_node`, `cmd_vel`, `payload/mass`, `ground_truth/odom`, 플러그인 `amr_navigation::PurePursuitController`/`DWAController`, `controller_id` `PurePursuit`/`DWA`); 존재하지 않는 `amr_velocity_governor`·`cmd_vel_safe`·`wheel_velocity_pid`·`/amr_i/payload_mass`·collision_monitor 경로 제거; 프로파일러 50 Hz; 설정 출처를 `/home/CAPYI/config`(구버전) → 프로젝트 `config/robot_params.yaml`(공차 47.6 kg, $J_w$, $\mu$, safety 블록)로; $v_{cte}$ 하한 버그(음의 속도) 수정; DiffDrive 제한기·effort 한계 요구를 시뮬 팀 요구사항에 추가.

### 9.3 v2.1 → v2.2 최종 점검 (2026-09-22, 감사 재개) — 49 개 항목 재대조

v2.1 감사는 §9.2 를 쓴 직후 중단되어 마지막 자체 점검이 없었다. 이번 점검은 (i) `checks/c01–c12` 를 모두 재실행해 본문 수치와 대조(전부 재현, `run_audit2_fast.log`/`run_audit2_slow.log`), (ii) v2.1 에서 새로 쓴 설계(계약 기본안 PI, 저크 지표, 곡률 샘플 DWA)를 독립 시뮬 `c13–c19` 로 검증, (iii) 인용 재fetch(arXiv API 원문 XML, PMC 본문, Crossref 6건, SNU Pure 초록, 검색 초록), (iv) config·components.md·sequences.md·컨테이너(Humble 1.1.20) 정합 확인으로 진행했다. **최종 49/49 해결** — 아래 13 개 항목은 v2.1 에서 틀리거나 불완전하게 해결되어 v2.2 에서 다시 고쳤고, 나머지 36 개는 v2.1 해결을 재확인했다.

| 항목 | v2.1 상태 | v2.2 조치 (근거) |
| --- | --- | --- |
| M3 / PV4 / RR5 PI 오버슈트·수락 기준 | 대안 T 는 옳았으나 **새 기본안**(피드포워드 + $b=0.5$ 설정점 가중)이 자기 수락 기준 위반: 0.15 m/s 스텝 12.6–25 %(≤ 1 %), 램프 측정 정상오차 $aK_p(1-b)/K_i = 0.05$ m/s(≤ 0.03). 대안 T 도 $b_v=0.5$·무피드포워드로 램프 지연 0.167 m/s | 기본안 → **모델추종 2-DOF PI**($u_{ff} = r + \hat T_p a_r$, PI 입력 $\hat P u_{ff} - y$): 모델 일치 0 %/0 %, 추종오차 ≤ 0.021 m/s, $\hat T_p$ ±30 % 에서 ≤ 4.1 %; 대안 T 운전 = 가속 피드포워드 + $b_v=1$(0.001 m/s, 25 kg 미반영 0.023), $b_v=0.5$ 는 식별 스텝 전용; 절차 ④·테스트 (6)·파라미터 갱신 [c13, c14, c15, c19] |
| M11 내루프 위상지연 | 대안 T 에 대해서만 정량화 | 기본안 내루프에서 PP 오버슈트 4.4–5.1 %(등가 ζ 0.69–0.70), 테스트 (3) 에 통합 기대값 분리 [c15] |
| M6 / SG6 / RR8 응답시간 | 계약 정의(첫 `cmd_vel`≠0)로 바꾸고 충돌을 명시하지 않음 | 스펙 10장 문언은 "로봇 첫 움직임" — 물리 해석 시 평균 ≈ 291 ms(이벤트 구동 + 기동 예외로도 ≈ 216 ms) > 200 ms 를 §5·§7 Q8·TL;DR 에 명시, 아키텍처 결정으로 이관 [c18] |
| SG7 / RR13 저크 지표 | 2차 Butterworth 5 Hz: 음의 로브 때문에 한계에 붙은 S-curve 를 P99 2.067 m/s³ 로 측정(필터만으로 목표 위반) | 비음수 가우시안 커널(σ 26.5 ms, −3 dB 5 Hz) → $|j_{filt}|\le\max|j|$ 보장(P99 2.000), 잡음 바닥 σ 0.146 / P99 0.38, 기동 예외 피크 5.2–5.4, `jerk_limit_scale` 여유 노브 [c17] |
| SG2 / RR3·RR11 회전 모드 | 모드는 정의했으나 $(v_T,\kappa)$ 곡률 샘플 DWA 는 $v=0$ 회전에서 κ 미정의 | 회전 모드에서 $(0,\omega)$ 후보로 전환, 프로파일러 $|v|\le0.05$ 분기와 연결, 해제 직후 κ 격자 제한(§4.3) |
| PV6 명제 3 신규성 | "신규 유도" 표기 | Ollero & Heredia 1995 초록(정곡률 경로 + 순수지연 안정성, PP 적용) 재확인 → 신규성 주장 철회, 설계 도구로만 사용; 폐루프 시뮬로 ζ 0.676/0.598 재검증 [c16], 테스트 (3b) 를 $e_0=\pm1$ cm 평균으로 명세 |
| C3 ref 13 | "RPP/DWA/MPPI 는 테스트베드에서 평가된 전통 제어기" | arXiv 원문 초록: 세 제어기는 동기 부분에서만 언급 → 표현 정정 |
| C4 ref 5 | look-ahead 구현값 "0.5–4.0 m" | PMC 본문 재fetch: 표 3(0.5/4.0 m)과 본문("0.5–5.0 m")이 논문 안에서 엇갈림을 명시; 74/82/70 %, 초록 68–82 %, 87 %, 15.6 %, Sci. Rep. 16:8820 재확인 |

**재확인(변경 없음) 요지**: M1(부호식·검산), M2(유한차분 c01 + 폐루프 c16), M4(일반 감속거리 $(v+v_t)a/2j$ 가 옳고 리뷰어 식이 틀림 — 수치적분 일치, 결속점 트리거 c06/c07 재현), M5(c10: v2 가중합 κ 2.445 vs 2.5, 창 클리핑 5.52 s vs 계단 목표 2.46 s), M7(0.063 m, 11 % 과대), M8(Stanley ζ ≥ 1 AM–GM, 손 유도 일치), M9(차체 정의·ρ 규칙 예시), M10(config 모델 2.30/0.65/0.20/0.05 m·0.64 m/s, DiffDrive 1.2a 1.97/0.57/0.18/0.05·0.69, 토크 0.84), M12(RSS $L_{\min}$ 1.086 m 손계산 일치, 따름정리 0.866 rad/s·$v^*$ 0.833), M13(c12 표 재현); PV1–PV3·PV5, SG1·SG3–SG5·SG8–SG10(로컬플래너 브리프 12×12 m 확인), C1(arXiv 4건 원문 재확인), C2(Arulselvan), C5(원문 XML 에 "up to 1 [kHz]" — 리뷰어 오류), C6(Crossref: Ahn IJAT 22(1):119–129; Yang CEA 194:106760; Villagra RAS 60(2):252–265; Zanasi Automatica 36(3):439–448; Haschke IROS 2008 3248–3253; Ollero–Heredia IROS 1995 vol. 3 461–466), C7.

**계약·설정 정합 수정(critique 밖)**: Nav2 YAML 을 Humble 문법으로(`goal_checker` → `goal_checker_plugins: ["general_goal_checker"]` + 이름 블록 — 컨테이너 nav2_bringup 기본 params 로 확인), 한 줄에 `;` 로 이은 키(유효하지 않은 YAML) 분리, `velocity_profiler_node` PID 파라미터를 모델추종형으로 교체(파싱 검증); config `clearance_speed_limit_enabled`(여유거리 연속 제한 $v_{\max}(D)$, $D=2.6$ m → 2.0 m/s)를 §1·§2.7 에 반영; RPP/velocity_smoother/collision_monitor 헤더 사실(§1) 컨테이너 재확인.

**남은 외부 의존(문서로 해소 불가, §7)**: Q8 응답시간 정의(스펙 문언 vs 계약), Q3·Q9 바퀴 effort 한계와 DiffDrive 제한기 설정(시뮬 팀), Q5 `safety_node` 의 κ 보존 스케일링(안전팀), $\tau_{\max}$ 6 N·m 가정(URDF 확정 전까지 가정 표기 유지).
