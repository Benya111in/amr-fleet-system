# 인지·추적 설계 브리프 — YOLOv8 검출, Pinhole 2D→3D, LiDAR 동적장애물 분류, KF/IMM 추적, TTC (스펙 4.6, 4.7)

작성일 2026-09-21 · **개정 2026-09-22 (rev2 적대적 리뷰 반영 + rev2.1 재개 후 자체 점검 — 이력은 §10)** · 영역: perception-tracking · 대상 스택: ROS2 Humble / Nav2 1.1.20 / Gazebo Fortress / C++17 + Python 3.10 / RTX 5090 + 32 스레드

> 정합 기준: `config/sensors.yaml`, `config/robot_params.yaml`, `config/ekf.yaml`, `docs/architecture/components.md` §3.4·§5.4(노드·토픽 이름), `sequences.md` §2(TTC 임계), `multi_robot.md` §7(CPU 예산), `src/amr_msgs/msg/*.msg`. 수치는 모두 `checks/rev4_*.py`·`checks/rev5_fixes.py`(실행 로그 `checks/rev4_*.out`, `checks/rev5_fixes.out`)와 `checks/bench/`(컨테이너 실측)로 재유도했다. rev2.1(같은 날, 재개 후 자체 점검)의 추가 정정은 §10.7.

## 0. 요약 (TL;DR)

- **필수 베이스라인(B1)**: (a) `yolo_node`(YOLOv8, 5클래스) → `object_localizer_node`(Pinhole 역투영 + 깊이 노이즈 모델 + **클래스별 표면→중심 보정** + 공분산) → `perception/detected_objects`(map) → `detection_marker_node`; (b) `obstacle_tracker_node`(C++): `scan_filtered` → **odom 프레임**에서 정적 지도 배경 분리 → 적응형 브레이크포인트 분할 → 편향 보정 클러스터 측정 → **CV 칼만 필터 + 확장 행렬 GNN(Hungarian) 연관** → 창(window) 최소제곱 속도 검정 → 속도·방향·신뢰도·동적 여부·**TTC** → `perception/tracked_obstacles`(map, 10 Hz). 전 과정 설명 가능한 고전 알고리즘.
- **rev2의 핵심 수정**: 추적 프레임을 map→**odom**으로 이동(AMCL 보정 점프가 가짜 속도가 되는 문제 제거, 8 cm 점프 시 정지 물체 오탐 20 %→0 %), 클러스터 공분산 재유도(가로 양자화 $s^2/24$, 클래스별 시선 편향 **보정 후** 잔차만 노이즈로), 속도 검정을 KF χ² 3연속 → **1 s 창 LS 기울기 검정**으로 교체(0.3 m/s 물체 확정: 기존 0 %/3 s → 중앙값 0.5–0.6 s), 연관 비용을 행/열 상수(무효)에서 **미연관·탄생 비용을 가진 확장 행렬 NLL**로, GPU 예산 산술 수정(이미지당 6.7 ms / 배치당 33.3 ms), 카메라 3D는 **15 Hz**(깊이 주기)로 명시, 카메라 검출은 **클래스 전용 갱신**(OOSM 제거).
- **발견된 설계 제약(신규)**: LiDAR 스캔 평면이 지면 +0.38 m라 **다른 AMR 차체(상면 0.33 m)와 소·중형 박스(0.15/0.30 m)는 LiDAR에 보이지 않는다**(`checks/rev4_geom_meas.out` B). → 피어 AMR은 `/amr_XX/odometry/filtered_map`으로 트랙에 주입(§3.8), 저상 화물은 depth voxel layer(`camera/depth/points_filtered`)와 카메라 검출이 담당.
- **제안의 정직한 재포지셔닝**: P1(클래스 조건부 IMM)은 **Joint Tracking and Classification(JTC)의 공학적 적용**(engineering_adaptation), P3(동적 점수)는 **2D DATMO 일관성 검출(Wang et al. 2007)의 공학적 적용**. 구 P2(UPA)는 독립 제안에서 제외하고 올바른 부분만 베이스라인 측정 모델에 흡수. 신규성 주장은 없고, 가치는 **B1 대비 측정된 개선**(FDE@1.5 s, 동적 판정 지연 p90, 오탐률)으로만 주장한다.
- **문헌**: arXiv/문서 페이지를 WebFetch로 재확인(2026‑09‑22). 출판사 페이지가 403인 고전(JTC, DATMO)은 WebSearch로 서지만 확인한 **RECALLED**로 표기.

## 1. 스펙 정량 목표 → 설계 답 → 측정 방법

| 스펙 | 정량/필수 목표 | 설계 답 (절) | 측정 방법·합격 기준 (§6) |
|---|---|---|---|
| 4.6 객체 인식 | YOLOv8, ≥3종(화물/사람/표지판) | 5클래스 {box, person, sign, forklift, amr}, Gazebo 합성 데이터 미세조정 (§3.2) | 합성 val 2k장 클래스별 mAP50 ≥ 0.9, P/R |
| 4.6 커스텀 메시지·마커 | 커스텀 msg + RViz 마커 | `amr_msgs/DetectedObjectArray`(+공분산 필드 제안) → `perception/markers` CUBE+TEXT "Class/Conf/Dist" (§7) | `ros2 topic echo`, RViz 스크린샷, launch_testing |
| 4.6 추론 속도 | GPU ≥ 30 FPS (또는 CPU ≥ 10 FPS) | 단일 배치 `yolo_node`(5 스트림, 배치 5, FP16; TensorRT 도입 시 `quantize=16`), 이미지당 6.7 ms·배치당 33.3 ms 예산; CPU 폴백 imgsz 320, 2 스레드(실측 125 core‑ms/프레임 → 코어당 8.0 FPS), 함대 저하 운용 로봇당 5 Hz (§3.2) | 스트림별 처리 FPS(5대 동시, 60 s) ≥ 30, E2E 지연 p95; CPU 폴백 단일 스트림 벽시계 FPS ≥ 10(유휴 호스트, W3) + core‑ms/프레임 |
| 4.6 3D 변환 | 픽셀+깊이 → map 3D, Pinhole | `object_localizer_node`: $X=(u-c_x)Z/f_x$ …, `camera_info`의 K 사용($f_x$=337.2 px), 표면→중심 보정, $\Sigma$ 전파, TF (§3.1) | base 프레임 RMSE(box·person): 1–3 m ≤ 0.07, 3–6 m ≤ 0.08, 6–10 m ≤ 0.10 m; map 프레임 ≤ 0.09/0.11/0.15 m; 평균 NEES가 χ² 95 % 대역 내 (N=300: [1.78, 2.23]) |
| 4.1 동적 장애물 | 0.3–1.5 m/s, ≥5개, 직선/곡선/무작위 | 속도별 판정 경로 3개(자유공간·LS 속도·클래스)와 속도별 지연 예산 (§3.6) | 0.3/1.0/1.5 m/s × 접근/횡단/이탈/정지‑출발 시나리오의 "최초 가시→`is_dynamic`" 지연 p50/p90 |
| 4.7 동적 분류 | LiDAR로 동적 분류 | 베이스라인: 지도 배경 분리 + LS 속도 검정; P3: DATMO 로그오즈 (§3.6, §5.3) | 동적 P/R(트랙‑프레임), 정지 트랙당 시간당 오탐 ≤ 1, 선반 0.2 m 옆 보행자 재현율 |
| 4.7 KF 추적 | KF/EKF, 속도·방향·신뢰도 | CV‑KF(B1) / JTC‑IMM(P1, CT는 EKF), 방향 표준편차 출력, 로지스틱 보정 신뢰도 (§3.5, §5.1) | MOTA·IDSW(CLEAR MOT, 0.5 m), 속도/방향 RMSE, FDE@1.0/1.5 s, 신뢰도 ECE ≤ 0.05 |
| 4.7 충돌 예측·TTC | 미래 위치 예측, TTC, 임계 이하 회피 | 트랙 예측 + 공분산 성장 + 경로(`plan`) 교차로 TTC, τ_warn 3.0 s / τ_crit 2.15 s(sequences.md) (§3.7) | TTC 오차(GT 궤적으로 계산한 TTC 대비), 확정 동적 트랙의 거리 ≥ 요구 거리(§3.7 표) |
| 4.7 1.0 m/s 인식·재계획 | 1.0 m/s 장애물 인식 후 재계획 | 1.0 m/s: 확정+동적 지연 p90 ≤ 0.5 s, 정면 접근 시 요구 확정 거리 10.5 m(로봇 2.0 m/s) (§3.6–3.7) | 공동 시험(플래너): 재계획 트리거 시점의 트랙 거리·TTC 로그 |
| 4.7 VO/ORCA | 반응형 회피(DWA 플러그인) | 계약: 위치·속도·반경·4×4 공분산·동적 확률 제공 (§7.3) | DWA 팀 통합 시험 |
| 4.7 센서 고장 | 고장 감지 → 정지/저속 | 원시 센서 타임아웃은 `safety_node`(robot_params); 인지 파이프라인 건강은 `perception/health` + 트랙 스탬프 나이 (§7.4) | 결함 주입(스캔 정지, GPU 끔, TF 지연) 시 상태 전이·지연 로그 |
| 4.4 costmap | 동적 장애물 반영 주기·유지 시간 설정 | obstacle layer `observation_persistence: 0.0`, `expected_update_rate: 0.3`, `inf_is_valid: true`, local costmap 10 Hz 권고 (§7.5) | 이동 장애물 잔상 셀 수, costmap 갱신 주기 실측 |
| 4.9 다중 로봇 | 5대, 서로가 동적 장애물 | 피어 AMR 트랙 주입(지연 보상) + 적재 AMR은 LiDAR 클러스터와 연관 (§3.8) | 피어 교차 시나리오 MOTA, TTC |
| 4.10 성능 | CPU ≤ 80 %, 커버리지 ≥ 70 %, 지표 측정 표준화 | 인지 CPU 목표 ≤ 2.5 코어(5대 합) (§7.7), gtest/pytest, 로그 포맷 고정(§6.2) | `/proc/stat` 60 s 샘플(multi_robot.md §7 스크립트), `colcon test --coverage` |
| 9 동료평가 | "칼만 필터 예측 정확도" | FDE@1.0/1.5 s와 NEES/NIS 일관성을 정의·로그화 (§6.2) | 표준 로그 → 스크립트 집계 |

## 2. 설치 스택·설정 정합성 검증 (`docker run --rm amr-fleet-system:wf-final`, `amr_dev` 미접촉)

- **파이썬 스택(wf-final)**: `ultralytics 8.4.155`, `torch 2.11.0+cu128`, `torchvision 0.26.0`, `numpy 1.26.4`, `scipy 1.15.3`, `opencv(-contrib)-python 4.11.0.86`. rev1이 보고한 numpy 2.2.6 ↔ scipy 1.8.0 충돌(`latest` 이미지)은 wf-final에서 해소되어 `scipy.optimize.linear_sum_assignment` import 성공. 추적기는 components.md대로 C++이며 Hungarian은 직접 구현하고, pytest에서 scipy를 **기준 구현(oracle)** 으로 쓴다. `onnxruntime`/`tensorrt`/`openvino` 파이썬 런타임은 없음.
- **ROS 패키지**: `vision_msgs 4.1.1`, `image_geometry 3.2.1`, `laser_geometry 2.4.1`, `nav2_costmap_2d 1.1.20`(dpkg 확인). `ObstacleArray`류 동적 장애물 메시지는 없으므로 `amr_msgs/TrackedObstacleArray`(기존)를 확장한다(§7.2).
- **GPU 실측**(`--gpus all`, RTX 5090, `checks/bench/gpu_result*.txt`): **호스트가 다른 사용자의 작업으로 과부하 상태(loadavg 47–52 / 32 스레드)** 여서 수치는 상한이 아닌 "혼잡 조건" 값이다. YOLOv8n fused FP16 forward 중앙값 **배치 1: 14.2 ms, 배치 5: 13.6 ms**(p95 55/45 ms) → 커널 런치(CPU) 병목이며 배치 5가 배치 1과 같은 시간이므로 **배치 추론이 처리량 5배**. `predict()` 전체(전/후처리 포함) 배치 1 중앙값 4.4 ms(1회차)/26 ms(2회차, 부하 변동), VRAM 할당 68 MB. 깨끗한 호스트에서의 재측정과 TensorRT 엔진 측정은 W3 게이트(§7.8). **CPU 실측**(rev2.1): 호출 스레드 CPU 시간 기준 imgsz 320 파이프라인 125.2 core‑ms/프레임 → 코어당 8.0 FPS(§3.2, `checks/bench/cpu_pipeline.txt`).
- **Ultralytics 인자**: export 문서(2026‑09‑22 재확인)상 `half=True`/`int8=True`는 `quantize=16/8`로 전달되는 **deprecated 별칭** → `model.export(format="engine", quantize=16, batch=5)`. 8.4.155의 `predict(half=True)`도 같은 deprecation 경고를 출력함(실측).
- **깊이 노이즈(설정 정합)**: rev1의 "Fortress 깊이 셰이더에 가우시안 경로 없음 → 자체 `depth_noise_node`" 판단은 프로젝트 설정과 충돌하므로 폐기한다. `sensors.yaml depth_camera`는 시뮬레이터 네이티브 거리 무관 σ `noise_base`=0.005 m + 프로젝트 노드가 가산하는 거리 제곱 항 `noise_quadratic_coeff`=0.002 m⁻¹, 합성 $\sigma_d(Z)=\sqrt{0.005^2+(0.002Z^2)^2}$(1 m 0.5 cm, 3 m 1.9 cm, 5 m 5.0 cm, 10 m 20 cm)로 확정되어 있다. components.md §3.4는 이 항을 `pointcloud_filter_node`에서만 가산하므로, `object_localizer_node`도 **같은 파라미터로 ROI 픽셀에 가산**해야 명세 7장(노이즈 적용)과 공분산 모델이 일치한다(인터페이스 변경 제안, §7.2).
- **카메라 내부 파라미터**: `sensor_calibration.md` §2.2의 Gazebo 6.18 실측 $f_x=f_y=337.2098$ px, $c_x=320$, $c_y=240$(87° HFOV @ 640 px). rev1이 암묵적으로 쓴 ~500 px가 아니다. K는 반드시 `camera_info`에서 읽는다.
- **LiDAR**: 720빔, 0.5000°(angle_max 179.5°, Fortress 실측), 0.10–25 m, σ 0.03 m, 미검출 빔은 **+inf**. `gpu_lidar`는 렌더링 센서로 한 업데이트의 전 빔을 한 시각에 렌더링하므로(RECALLED, W1 첫날 `time_increment` 확인) 시뮬레이션에는 스캔 내 자기운동 왜곡이 없다.
- `ros_gz_bridge`: CameraInfo/Image/PointCloud2/LaserScan/Imu 변환 지원(rev1 확인 유지).

## 3. 베이스라인 알고리즘의 수학적 유도

### 3.0 프레임과 센서 기하 (rev2 신설)

**추적 프레임 = `<r>/odom`.** `ekf_filter_node_odom`의 odom→base_footprint는 연속이고 점프하지 않는다(REP‑105, `ekf.yaml`). map 프레임에서 추적하면 AMCL 보정(스펙상 3–8 cm, 납치 복구 시 m 단위)이 **모든 클러스터를 동시에** 이동시켜 가짜 속도가 된다. LS 속도 검정(§3.6)에 map 프레임 계단을 넣은 몬테카를로: 5 cm → 정지 물체 오탐 1.4 %, 8 cm → 20.4 %, 0.30 m → 100 %; odom 프레임에서는 계단이 없다(`rev4_dyn.out` V). 따라서

- 측정·예측·연관·속도 검정은 모두 odom 프레임,
- 정적 지도(`/map`) 중첩 판정과 자유공간 격자는 스캔 스탬프의 최신 map→odom으로 지도를 odom에 옮겨 계산,
- 출력(`perception/tracked_obstacles`, components.md 계약상 frame `map`)과 TTC(경로 `plan`이 map)만 map으로 변환한다. 이때 로봇 포즈 공분산은 출력 공분산에 **한 번만** 더한다(§3.1의 식).

**LiDAR 평면의 가시성.** 스캔 평면 높이 $h_L = 0.18+0.20 = 0.38$ m(`robot_params.yaml base_link_height` + `sensors.yaml lidar.z`).

| 물체(명세 8장 물품·로봇) | 상단 높이 | 2D LiDAR |
|---|---|---|
| 소형 박스 30×20×15 | 0.15 m | **보이지 않음** |
| 중형 박스 50×40×30 | 0.30 m | **보이지 않음** |
| 대형 박스 60×50×40 | 0.40 m | 보임(여유 2 cm) |
| 다른 AMR 차체 | 0.33 m | **보이지 않음** |
| 중형 화물 적재 AMR | 0.63 m | 보임 |
| 사람 | 다리(정강이) 단면 | 보임(2개 원, 반경 ≈ 0.06 m) |

따라서 (i) 피어 AMR은 플릿 포즈로 주입해야 하며(§3.8, rev1의 오픈 질문은 임계 경로였다), (ii) 저상 화물은 카메라 검출(`detected_objects`)과 지역 costmap voxel layer(`camera/depth/points_filtered`, ≤ 5 m)로만 보인다.

**카메라 시야.** 높이 $h_C = 0.18+0.25 = 0.43$ m, HFOV 87°, VFOV $2\arctan(240/337.2)=70.9°$, 수평 장착 → 전방 0.60 m부터 바닥이 보인다.

### 3.1 Pinhole 역투영, 깊이 노이즈, 표면→중심 보정 (스펙 4.6)

광학 프레임(x 우, y 하, z 전방)에서

$$u = f_x \frac{X}{Z} + c_x,\quad v = f_y \frac{Y}{Z} + c_y;\qquad X = \frac{(u-c_x)Z}{f_x},\quad Y = \frac{(v-c_y)Z}{f_y}.$$

Gazebo 깊이(`32FC1`)는 Z‑depth이다. 광학↔`camera_link` 고정 TF는 rpy $=(-\pi/2, 0, -\pi/2)$(`sensors.yaml camera_link.optical_rpy`).

**대표 깊이.** bbox 중앙 50 %×50 % ROI의 유효 픽셀(0.20–10 m, `depth_camera.range_min/max`) $m$개에 대해, 먼저 명세 노이즈 항 $\mathcal N(0,(kZ^2)^2)$, $k$=`noise_quadratic_coeff`를 픽셀별로 가산하고(§2), 중앙값 $\tilde Z$를 취한다. 유효 비율 < 30 %면 폐기. 픽셀 노이즈가 독립이면 $\operatorname{Var}(\tilde Z)\approx \frac{\pi}{2}\sigma_d(\tilde Z)^2/m$(MC 확인: 0.00442 vs 0.00443, `rev4_geom_meas.out` C). 실제 스테레오/ToF 깊이는 공간 상관이 있으므로 파라미터 `depth_noise_iid`(sim: true → $m_\text{eff}=m$, 실기: false → $m_\text{eff}=1$)로 보수 처리한다.

**표면→중심 보정(편향).** 중앙값은 **보이는 표면**의 깊이이고 GT와 트랙 상태는 물체 **중심**이다. bbox 중심을 지나는 광선을 따라 중심까지의 거리 $\delta$는 물체 형상·자세에 따른 확률변수이다. 바닥 위 직사각형(반치수 $a,b$)이 임의 요각일 때 $\delta(\phi)=\min(a/|\cos\phi|,\,b/|\sin\phi|)$로부터(`rev4_geom_meas.out` D):

| 클래스(카메라) | $\mu_\delta$ [m] | $\sigma_\delta$ [m] | 근거 |
|---|---|---|---|
| box(중형 50×40) | 0.250 | 0.034 | 직사각형, 요각 균일 |
| box(대형 60×50) / (소형 30×20) | 0.306 / 0.136 | 0.039 / 0.026 | 〃 |
| amr(60×40) | 0.271 | 0.052 | 〃 |
| person(몸통 ≈ 0.40×0.25) | 0.12 | 0.04 | 원형 근사, 보정 대상 |
| forklift | 0.5 | 0.15 | 초기값, 보정 대상 |
| sign(얇은 판) | 0.02 | 0.02 | 〃 |

클래스 사후 $p(c)$(§5.1)가 있으면 $\bar\mu=\sum_c p(c)\mu_c$, $\sigma_\delta^2=\sum_c p(c)[\sigma_{\delta,c}^2+(\mu_c-\bar\mu)^2]$. 박스 크기는 YOLO 클래스만으로 구분되지 않으므로 box는 bbox 폭에서 추정한 가로폭 $\hat L = \tilde Z w/f_x$로 소/중/대를 고른다. 보정 없이 중심 GT와 비교하면 중형 박스 25 cm, 사람 12 cm의 **체계 오차**가 남아 rev1의 "≤ 0.10 m"은 달성 불가였다. 보정된 3D 점: $\mathbf P_c = \mathbf P_s + \bar\mu\,\mathbf P_s/\|\mathbf P_s\|$.

**공분산(광학 프레임).** $\boldsymbol\xi=(u,v,Z)$, $\Sigma_\xi=\operatorname{diag}(\sigma_u^2,\sigma_v^2,\sigma_Z^2)$, $\sigma_Z^2=\frac{\pi}{2}\sigma_d(\tilde Z)^2/m_\text{eff}+\sigma_\delta^2$,

$$J=\begin{bmatrix}Z/f_x & 0 & (u-c_x)/f_x\\ 0 & Z/f_y & (v-c_y)/f_y\\ 0&0&1\end{bmatrix},\qquad \Sigma_\text{opt}=J\Sigma_\xi J^\top .$$

$\sigma_u=\sigma_v=\sqrt{(\kappa w)^2+\sigma_\text{skew}^2}$: $\kappa$는 bbox 중심 지터의 폭 비율(초기 0.05, 합성 val에서 GT 투영 중심으로 추정), $\sigma_\text{skew}=f_x|\omega_\text{ego}|\Delta t_\text{skew,max}/\sqrt3$(RGB–깊이 시각차 $\Delta t_\text{skew}\sim U[-16.7, 16.7]$ ms 가정; 최대 요레이트 1.5 rad/s에서 최대 변위 8.4 px, $\sigma_\text{skew}$ 4.9 px — `rev5_fixes.out` S; 동기 시뮬레이션에서는 0). 가로 오차는 $\sigma_X=(Z/f_x)\kappa w=\kappa L$로 **거리와 무관**하다($w=f_xL/Z$).

**base → map.** 광학 → base_link는 고정 외부 파라미터로 $\Sigma_\text{base}=R_{bo}\Sigma_\text{opt}R_{bo}^\top$(정확도 평가는 이 프레임, §6). map 출력은 로봇 포즈 $(x_b,y_b,\theta)$ 오차를 **base 원점을 피벗**으로, 교차공분산을 포함해 한 번 더한다:

$$\Sigma_\text{map}=R_{mb}\Sigma_\text{base}R_{mb}^\top+\big[I_2\ \ \mathbf g\big]\,\Sigma_\text{pose}\,\big[I_2\ \ \mathbf g\big]^\top,\qquad \mathbf g=\begin{bmatrix}-(p_y-y_b)\\ p_x-x_b\end{bmatrix},$$

$\Sigma_\text{pose}$는 `odometry/filtered_map`의 (x, y, yaw) 3×3 공분산(교차항 포함). 선형화 결과가 몬테카를로와 일치($[[1.348,-0.867],[-0.867,2.208]]\times10^{-3}$ vs $[[1.343,-0.864],[-0.864,2.203]]\times10^{-3}$, `rev4_geom_meas.out` F). rev1의 카메라 원점 피벗은 4.12 m 앞 물체에서 레버암을 4.12 → 3.95 m로 과소평가했고 교차항을 버렸다.

**예측 정확도와 목표(범위 대역별).** $\kappa=0.05$와 표의 $\sigma_\delta$, ROI 중앙값 항 $\frac{\pi}{2}\sigma_d^2/m$까지 넣은 지면 RMS(`rev5_fixes.out` E2; rev4 E는 중앙값 항을 빠뜨려 원거리를 과소평가): 시뮬레이션(독립 픽셀)에서 중형 박스 4.2 / 4.2 / 4.8 cm(1–3 / 3–6 / 6–10 m, 6–10 m 대역 안 최댓값 5.7 cm @10 m), 사람 4.6 / 4.6 / 4.7 cm — 근·중거리는 $\kappa L$과 $\sigma_\delta$가 지배하고 원거리에서만 깊이 항이 보인다. 상관 깊이 노이즈(실기 보수, $m_\text{eff}=1$)에서는 박스 4.4 / 6.2 / 14.2 cm, 사람 4.7 / 6.4 / 14.4 cm. **합격 목표(시뮬레이션, base 프레임, box·person)**: 1–3 m ≤ 0.07 m, 3–6 m ≤ 0.08 m, 6–10 m ≤ 0.10 m(예측 최댓값 대비 여유 ≥ 1.7배). map 프레임은 위치 추정 오차(스펙 직선 5 cm)와 요 오차 레버암($\sigma_\theta$ 0.01 rad 가정)을 대역 원단에서 제곱합한 예측 7.2 / 8.9 / 12.6 cm(박스), 7.4 / 9.1 / 12.2 cm(사람)에 맞춰 ≤ 0.09 / 0.11 / 0.15 m. $\kappa,\sigma_\delta$가 보정으로 바뀌면 같은 식으로 목표를 재계산한다(식이 목표의 근거).

복잡도: 검출당 $O(m\log m)$(중앙값), $m\le 0.25wh$.

### 3.2 YOLOv8 클래스, 합성 데이터 미세조정, 추론 예산

- COCO 80에는 `person`만 직접 해당(`stop sign`, `suitcase` 근접) → 클래스 $\mathcal C=\{\text{box},\text{person},\text{sign},\text{forklift},\text{amr}\}$로 **Gazebo 합성 데이터 미세조정**. 자동 라벨: GT 모델 포즈 + 3D 박스 8꼭짓점을 §3.1 순방향식으로 투영 → 2D 박스, GT 깊이로 가림률 > 60 % 제외. 도메인 랜덤화(조명, 바닥/선반 텍스처, 카메라 피치, 거리 1–10 m, 노이즈·블러)는 [21]의 결론(재질·렌더링·후처리·distractor)을 따름. 4–6k장, YOLOv8n/s, 60 epoch. 사람(1.7 m)은 10 m에서 bbox 높이 57 px로 검출 가능 범위.
- **추론 예산(수정)**: 5 스트림 × 30 FPS = 150 img/s → **이미지당 6.67 ms**, 배치 5를 30 Hz로 돌리면 **배치당 33.3 ms**(rev1의 "배치당 6.7 ms"는 이미지당 값을 배치에 쓴 오류). 혼잡 호스트 실측 fused forward 배치 5 중앙값 13.6 ms(이미지당 2.7 ms)로 중앙값 기준 충족, p95 45 ms는 미충족 → TensorRT(`quantize=16`, CUDA 런치 오버헤드 감소)와 유휴 호스트 재측정이 W3 게이트.
- **배포 형태(계약 유지)**: components.md는 로봇 네임스페이스별 `yolo_node`를 전제한다. 처리량 실측(배치 1 ≈ 배치 5 시간)에 따라 **루트에 `yolo_node` 1개**를 띄워 `/amr_XX/camera/image_raw` 5개를 구독하고 `/amr_XX/perception/detections_2d`를 그대로 발행한다(토픽 계약 불변, 파라미터 `robots: [amr_01..amr_05]`; `robots`가 비면 네임스페이스별 단일 스트림 모드). 배치 정책: 스트림별 최신 1장, 5장이 모이거나 첫 장 후 10 ms 경과 시 실행 → 배치 대기 ≤ 10 ms, 큐 길이 1(오래된 프레임 폐기). 출력 헤더 스탬프 = 입력 이미지 스탬프.
- **지연 예산(목표)**: 이미지 스탬프 → `detections_2d` p95 ≤ 50 ms(브리지 + 배치 대기 ≤ 10 + 추론 ≤ 20(TensorRT, 유휴 호스트) + 후처리 ≤ 3); → `detected_objects` p95 ≤ 70 ms(깊이 짝짓기 + 역투영 ≤ 5 ms). 배치 대기는 명시적으로 E2E에 포함한다.
- **3D 출력률 = 15 Hz**: RGB 30 Hz, 깊이 15 Hz(`sensors.yaml`). `object_localizer_node`는 **깊이 프레임을 트리거**로 가장 가까운 검출 메시지와 짝짓는다(`ApproximateTimeSynchronizer` slop 17 ms = RGB 반주기, 최대 시각차 16.7 ms; 동기 시뮬레이션에서는 0). rev1의 slop 30 ms는 사실상 15 Hz였으나 30 Hz로 기술되어 있었다. 2D 검출(`detections_2d`)은 30 Hz, 3D(`detected_objects`)·마커는 15 Hz로 명시한다.
- **CPU 폴백(스펙 4.6 대안 10 FPS)**: GPU 불가(`torch.cuda.is_available()==False`) 시 `yolo_node`가 자동으로 CPU 백엔드로 전환하고 `perception/health`에 WARN. CPU 경로는 Ultralytics `predict()` 래퍼를 쓰지 않고 **융합 모델 직접 호출 + cv2 레터박스 + torchvision `batched_nms`** 로 구성한다(아래 실측에서 래퍼가 같은 입력에 1242 core‑ms/프레임으로 직접 경로의 약 10배였고 원인은 미규명 — `bench/cpu_cputime.txt`).
  - **실측(혼잡 무관 지표)**: 호스트 포화(loadavg 90–119 / 32 스레드)로 벽시계 FPS는 무의미하므로 **호출 스레드 CPU 시간**(`time.thread_time`, intra‑op 1 스레드)을 측정했다. imgsz 320, FP32: 전처리 0.7 + 순전파 117.0 + 후처리 1.1 → 파이프라인 **125.2 core‑ms/프레임**(p90 142.5; `bench/cpu_pipeline.txt`, `rev5_cpu_budget.out`). 전용 코어 1개당 **8.0 FPS**(p90 7.0). SMT 형제 스레드가 바쁜 조건의 값이라 비용의 **상한**이다.
  - **설계 답**: 단일 스트림 10 FPS = 1.25 코어 → `cpu_threads: 2`(intra‑op 병렬 효율 ≥ 0.63이면 충족). 유휴 호스트 벽시계 FPS ≥ 10 확인이 W3 게이트이고, 미달이면 imgsz 256 또는 ONNX/OpenVINO 런타임 추가(현재 이미지에 없음, §2)를 인프라와 결정한다.
  - **함대 예산**: 5대 동시 CPU 추론 10 Hz는 6.3 코어 → 호스트 합계 26.8 코어로 80 %(25.6) 초과, **5 Hz면 3.1 코어 → 23.6 코어(74 %)로 충족**. 따라서 폴백 모드는 "스펙 대안 시연 = 단일 스트림 10 FPS"와 "GPU 고장 시 저하 운용 = 로봇당 5 Hz"로 나눈다(추적은 LiDAR 중심이라 클래스 갱신률 저하만 발생).

### 3.3 LiDAR 전처리, 정적 배경 분리, 분할

입력은 components.md대로 `scan_filtered`(`scan_filter_node`가 거리/각도/아웃라이어 필터 수행). **+inf 빔은 유지**해야 한다(자유공간 광선과 costmap clearing에 필요 — `scan_filter_node` 요구사항으로 전달). 시뮬레이션에서는 스캔 내 왜곡이 없으므로(§2) 스캔 스탬프의 TF(odom←lidar_link)로 한 번 변환한다. 실기 회전형 LiDAR에서는 `laser_geometry::LaserProjection::transformLaserScanToPointCloud`(빔별 `time_increment` + TF 보간)를 쓰고, 잔차는 "TF 시각 지터 × (자차 속도 + 각속도×거리)"로 유도해 NIS로 검증한다(rev1의 임의 $\beta\|\mathbf v_\text{ego}\|^2\Delta t^2$ 항은 삭제).

**점 단위 배경 라벨.** 각 점을 정적 지도(odom으로 옮긴 `/map`, 0.05 m)의 점유 셀과의 거리로 라벨: $d_\text{map}\le r_\text{bg}=0.10$ m(2셀, 위치 추정 오차 3–8 cm 흡수)이면 배경. components.md의 "`scan_filtered` − `/map` 배경"에 해당한다.

**적응형 브레이크포인트 검출(ABD, [C4])**: 연속 빔 $i-1,i$가

$$\|\mathbf p_i-\mathbf p_{i-1}\| > D_\text{th}=r_{i-1}\frac{\sin\Delta\phi}{\sin(\lambda-\Delta\phi)}+3\sigma_r\quad(\Delta\phi=0.5°,\ \lambda=10°,\ \sigma_r=0.03\ \text{m})$$

이거나 **배경 라벨이 바뀌면** 세그먼트를 끊는다. $D_\text{th}$ = 0.25 m(3 m), 0.35 m(5 m), 0.51 m(8 m), 1.15 m(20 m). 선반에서 0.2 m 떨어진 보행자는 3 m 거리에서 ABD만으로는 분리되지 않으므로($0.2<0.25$) 배경 라벨 경계가 분리를 담당한다(비지도화 정적물 옆은 아래 재분할).

**병합·재분할(수정)**: 세그먼트 병합은 **최소 점간 간격** < 0.10 m이고 **배경 라벨이 같을 때만**(rev1의 "중심 거리 < 0.3 m"는 선반 옆 보행자를 흡수했다). PCA 장축이 클래스 크기 사전의 최대(사람 1.0 m, 미지 1.5 m)를 넘으면 가장 큰 내부 간격에서 재분할. 최소 점 수: $r\le 6$ m에서 3, $r>6$ m에서 2(다리 모델: 10 m에서 평균 2.2점, $n\ge3$ 31 % / $n\ge2$ 86 %; 3/5 확정 확률 18 % → 98 %, `rev4_extra.out` a). 클러스터 특징: 보정 중심, PCA 폭·길이, 점 수 $n$, 평균 거리 $\bar r$, 배경 중첩률 $\rho_\text{map}$(반경 $r_\text{ov}=0.25$ m, 지도 오정렬 구조물 포착). 복잡도 $O(N)$, $N=720$.

### 3.4 클러스터 측정 모델 (rev2 재유도)

**편향 보정.** 2D 광선 추적 시뮬레이션(0.5°, σ_r 0.03 m, 요각·보폭 무작위; `rev4_geom_meas.out` G)에서 점 중심은 물체 중심보다 시선 방향으로 가깝다:

| 클래스(LiDAR; 편향·점 수는 5 m) | 시선 편향 $\mu_{\delta}$ | $\sigma_\delta$ | 가로 형상 $\sigma_\text{lat}$ (3/5/8 m 최대) | 평균 점 수 |
|---|---|---|---|---|
| person(다리 2개) | 0.061 | 0.027 | 0.014 | 4.5 |
| box(대형 60×50) | 0.225 | 0.029 | 0.015 | 16 |
| amr(적재 60×40) | 0.200 | 0.037 | 0.012 | 15 |
| forklift(차체 ≈1.2×1.0) | 0.465 | 0.057 | 0.051 | 32 |
| unknown(사전) | 0.15 | 0.10 | 0.03 | — |

$\sigma_\text{lat}$는 광선 추적의 **총** 가로 표준편차에서 양자화분 $s^2/24$를 뺀 **형상 기여분** $\sqrt{\sigma_\text{tot}^2-s^2/24}$이다(`rev5_fixes.out` G2, 12k 시행; 예: 사람 5 m 총 0.0109 = √(0.0089² + 0.0064²)). rev2 초안은 총값(5 m)을 넣어 아래 식에서 양자화를 두 번 셌다.

측정 $\mathbf z=\bar{\mathbf p}+\bar\mu_\delta\,\mathbf u_\text{LOS}$ ($\bar\mu_\delta=\sum_c p(c)\mu_{\delta,c}$, 센서에서 멀어지는 방향). 이것이 rev1의 "지속 편향을 백색 노이즈로 흡수"를 대체한다.

**공분산.** 시선 좌표계에서

$$R_\text{cl}=R_\psi\operatorname{diag}(\sigma_\parallel^2,\sigma_\perp^2)R_\psi^\top,\qquad \sigma_\parallel^2=\frac{\sigma_r^2}{n}+\sigma_\delta^2,\qquad \sigma_\perp^2=\frac{(r\Delta\phi)^2}{24}+\sigma_\text{lat}^2,$$

각 성분 하한 0.02 m. $\sigma_\delta^2$에는 클래스 혼합 분산 $\sum_c p(c)(\mu_{\delta,c}-\bar\mu_\delta)^2$을 더한다.

- 시선 방향: $n$개 거리 노이즈의 평균이므로 $\sigma_r^2/n$(rev1은 $1/n$ 누락).
- 가로 방향: 빔 격자 오프셋 $u\sim U[0,s)$, $s=r\Delta\phi$일 때 점 중심의 가로 오차는 $n$과 무관하게 분산 $s^2/24$이다. 유도: 폭 $W=ms+f$에서 오차는 $u-f/2$ ($u<f$) 또는 $u-(s+f)/2$ ($u\ge f$)로 조각 균일, 분산 $\frac{1}{12s}[f^3+(s-f)^3]$를 $f\sim U[0,s)$로 평균하면 $s^2/24$. 평판 몬테카를로: 0.0054/0.0089/0.0141 m vs 0.0053/0.0089/0.0143 m(3/5/8 m). 리뷰가 제안한 $(r\Delta\phi)^2/(12n)$은 $n$배 과소(§10 M2).
- 포즈 공분산은 측정마다 더하지 않는다(odom 프레임, §3.0).

**속도 검정용 지터 공분산.** 보정 후 잔차 $\sigma_\delta$는 물체별로 **지속적**이어서 위치 갱신에는 노이즈지만 시간 기울기 검정에는 영향이 없다(상수의 기울기는 0). 기울기 검정은 프레임 간 지터 $R_\text{jit}=\operatorname{diag}(\sigma_r^2/n+\sigma_\text{seg}^2,\ s^2/24+\sigma_\text{seg}^2)$, $\sigma_\text{seg}=0.03$ m(정지 물체 로그의 중심 차분 표준편차로 보정)를 쓴다. 지속 편향을 백색으로 넣으면 미지 클래스 시선 방향 0.3 m/s의 4 s 내 발화율이 62 %(중앙값 2.1 s)로 떨어지고, 지터를 쓰면 100 %(중앙값 0.6 s)이다(`rev4_dyn.out` V vs `rev4_extra.out` b).

### 3.5 CV 칼만 필터(B1), 연관, 생명주기, 출력

**CV 모델(단일 파라미터화).** $\mathbf x=[p_x,p_y,v_x,v_y]^\top$(odom), $\Delta t$ = 스캔 스탬프 차(≈ 0.1 s),

$$F=\begin{bmatrix}I_2&\Delta t I_2\\0&I_2\end{bmatrix},\quad Q=q\begin{bmatrix}\tfrac{\Delta t^3}{3}I_2&\tfrac{\Delta t^2}{2}I_2\\\tfrac{\Delta t^2}{2}I_2&\Delta t I_2\end{bmatrix},\quad H=[I_2\ 0],\ R=R_\text{cl}.$$

연속 백색 가속도(CWNA) 스펙트럼 밀도 $q$ [m²/s³] **하나만** 쓴다: 1 s 동안 속도 변화 표준편차 $\sqrt{q\cdot 1\,\text{s}}$. B1은 클래스 무관 $q=0.25$(0.5 m/s/√s); P1의 CV 모드도 **동일한 CWNA 모델**(§5.1). rev1의 "사람 $q=0.5$" / YAML `sigma_a: 0.7` / "IMM CV = CTRV(ω=0)"의 3중 정의는 폐기. 정상상태(σ 0.031/0.020 m): $q$=0.1/0.25/0.5에서 $\sigma_v$=0.14/0.20/0.25 m/s(`rev4_dyn.out` L). 갱신은 Joseph 형식 $P=(I-KH)P^-(I-KH)^\top+KRK^\top$.

**게이트와 연관(확장 행렬 GNN, [C20]).** 정규화 혁신 제곱 $d_{ij}^2=\boldsymbol\nu_{ij}^\top S_{ij}^{-1}\boldsymbol\nu_{ij}$, 실현 가능 조건 $d^2\le\gamma=\chi^2_{2,0.99}=9.21$. 트랙 $n$·검출 $k$에 대해 $(n+k)\times(k+n)$ 행렬:

$$C=\begin{bmatrix}C^{\text{as}}_{n\times k} & \operatorname{diag}(c^\text{miss}_i)\ (\text{비대각}=B)\\ \operatorname{diag}(c^\text{birth}_j)\ (\text{비대각}=B) & 0_{k\times n}\end{bmatrix},$$

$$C^\text{as}_{ij}=d_{ij}^2+\ln\det(2\pi S_{ij})-2\ln P_{D,i}\ (\text{게이트 밖}=B),\quad c^\text{miss}_i=-2\ln(1-P_{D,i}),\quad c^\text{birth}_j=-2\ln\lambda_B,$$

$B=10^6$(유한 대수; ∞ 금지). 음의 로그우도비이므로 무차원이고, "할당 vs 미탐+탄생" 결정이 베이즈 비교가 된다. $P_{D,i}$=0.9(가시), 0.5(다른 트랙의 그림자 속 또는 $r>8$ m), $\lambda_B=0.01$ m⁻²(탄생 비용 9.21). $\ln\det S$ 항 덕분에 공분산이 부푼 coasting 트랙이 확정 트랙의 검출을 빼앗지 않는다(예: 확정 σ 0.10 m·0.15 m 거리 $d^2$=2.25 vs coasting σ 0.50 m·0.30 m 거리 $d^2$=0.36 → $d^2$만 쓰면 coasting이 차지, NLL은 확정 트랙에 할당). 무작위 150문제에서 Hungarian(scipy oracle) = 전수탐색(`rev4_dyn.out` K). **신뢰도는 비용에 넣지 않는다**: rev1의 $-\lambda\log c_i-\lambda\log s_j$는 행/열 상수라 완전 할당 문제에서 argmin을 바꾸지 못한다(§10 M9). 트랙 품질은 $P_{D,i}$와 생명주기에서만 작용한다.

**생명주기.** tentative → confirmed: 최근 5스캔 중 3회 연관(첫 검출 후 최소 0.2 s); confirmed → coasting: 미연관 시 예측만, $m\le5$(0.5 s); 삭제: coasting 5회 초과 또는 tentative 5스캔 중 3회 미연관. 물리 상한 게이트(클래스 무관): $\|\mathbf z-\hat{\mathbf p}_\text{last}\|\le v_\text{phys}\,\Delta t_\text{since}+3\sqrt{\lambda_\max(S)}$, $v_\text{phys}=3.0$ m/s.

**출력(속도·방향·신뢰도).** $v=\|\hat{\mathbf v}\|$, $\theta=\operatorname{atan2}(\hat v_y,\hat v_x)$, $\sigma_\theta=\sqrt{\mathbf n^\top P_{vv}\mathbf n}/v$ ($\mathbf n$: 속도에 수직 단위벡터); $v<0.1$ m/s이면 방향은 정의되지 않으므로 직전 유효값 유지 + `heading_std`=π(메시지 확장 제안, §7.2). 헤딩은 상태에 없으므로 wrap 문제가 없다. **신뢰도**는 확률 의미를 갖도록 로지스틱 모델로 둔다:

$$c=\sigma\!\big(\beta_0+\beta_1\tfrac{h}{h+m}+\beta_2\ln\tfrac{\operatorname{tr}P_{pp}}{\sigma_\text{ref}^2}+\beta_3\max_c p(c)\big),\quad \sigma_\text{ref}=0.3\ \text{m},$$

초기 $\beta=(-2,4,-0.5,1)$, 로그에서 "GT와 0.5 m 이내 매칭" 레이블로 IRLS(numpy) 적합 → 보정도(ECE, 신뢰도 도표)가 의미를 가진다. rev1의 가중합은 확률 의미가 없어 "보정" 평가가 성립하지 않았다.

### 3.6 동적 판정 베이스라인과 속도별 지연 예산

**단일 속도 증거 정의(LS 창 기울기 검정).** 최근 $W=10$ 스캔(1.0 s)의 odom 보정 중심 $\mathbf z_k$에 대한 최소제곱 기울기 $\hat{\mathbf v}=\sum_k(t_k-\bar t)(\mathbf z_k-\bar{\mathbf z})/S_{tt}$, $S_{tt}=\sum(t_k-\bar t)^2$, $\operatorname{Cov}(\hat{\mathbf v})=R_\text{jit}/S_{tt}$(시선축 기준). 통계량 $T=\hat{\mathbf v}^\top\operatorname{Cov}(\hat{\mathbf v})^{-1}\hat{\mathbf v}\sim\chi^2_2$ ($H_0$: 정지). **발화 조건**: $n\ge4$, $T>\chi^2_{2,0.999}=13.82$ **그리고** $\|\hat{\mathbf v}\|>v_{\min,\text{eff}}$가 **2스캔 연속**,

$$v_{\min,\text{eff}}=0.15\ \text{m/s}+2\,\sigma_{\delta,c}\,|\omega_\text{LOS}|,$$

$\omega_\text{LOS}$는 자차 운동으로 인한 시선 회전율(자차 속도·자세로 계산). 로봇이 정지 물체 옆을 지날 때 잔차 편향 벡터가 시선과 함께 회전해 $\sigma_\delta|\omega_\text{LOS}|$의 겉보기 속도를 만들기 때문이다(미지 클래스 σ_δ 0.10 m: 고정 $v_\min$이면 ω_LOS 1 rad/s에서 3 s 통과당 오탐 29.9 % → 적응형 0.9 %; 0.5 m/s 이동체 검출 100 % 유지, 0.3 m/s는 74 %로 저하 — 이 경우는 자차 회전에 무관한 자유공간 경로(§5.3)가 보완, `rev4_extra.out` c). `robot_params.yaml`의 존 상한(경고 0.5 / 위험 0.2 m/s)과 여유거리 속도 제한 $v_\max(D)$로부터 $|\omega_\text{LOS}|\le v(D)/D$의 최댓값은 경고 존 경계 바로 밖 **D = 1.0 m에서 1.04 rad/s**(v = 1.04 m/s), D = 2 m 0.85, D = 3 m 0.67, 경고 존 안 ≤ 1.0 rad/s(D = 0.5 m)이다(`rev5_fixes.out` W; rev2 초안은 존 경계 값 0.50만 적어 최악값을 놓쳤다). 따라서 위 평가의 ω_LOS = 1.0 rad/s 사례가 사실상 최악이며 2.0 rad/s는 도달 불가하다. **한계(정직한 기술, `rev5_passing_latency.out`)**: 로봇이 지나가는 동안의 B1 지연 p90(시선·횡단 중 나쁜 쪽)은 사람 클래스 트랙이면 ω_LOS 0 / 0.5 / 1.04 rad/s 모두 0.7 s(0.3 m/s)·0.4 s(1.0 m/s)로 불변이다($v_{\min,\text{eff}}\le0.21$ m/s). **미지 클래스의 0.3 m/s 이동체는 ω_LOS 0.5에서 1.1 s, 1.04에서 3.4 s(4 s 내 79 %)로 예산 밖**이고 1.0·1.5 m/s는 0.4 s로 영향이 없다. 따라서 아래 0.3 m/s 예산(≤ 0.8 s)은 B1에서 "로봇 정지 또는 사람 클래스 트랙" 조건으로 한정하고, 통과 중 미지 클래스 저속체는 P3 자유공간 경로(접근·횡단, 자차 회전과 무관)와 카메라 클래스에 의존한다. 남는 조합(통과 중 + 미지 + 0.3 m/s로 멀어짐)은 TTC가 무한대인 비위협 운동이다. 통과 시나리오(§6.1)는 별도 행으로 보고한다. rev1의 §3.4(3연속 KF χ²)와 §5.3(단일 지시자)의 이중 정의는 이것 하나로 통일한다. 겹치는 창의 검정은 상관되므로 오탐률은 $0.001^2$ 같은 독립 가정이 아니라 **시뮬레이션**으로 제시한다.

| 지표(몬테카를로, `rev4_dyn.out` V / `rev4_extra.out` b) | 0.3 m/s | 0.5 | 1.0 | 1.5 |
|---|---|---|---|---|
| 사람 5 m, 횡단: 최초 검출→발화 중앙값 / p90 | 0.5 / 0.6 s | 0.4 / 0.4 | 0.4 / 0.4 | 0.4 / 0.4 |
| 사람 5 m, 시선 방향 | 0.6 / 0.7 s | 0.4 / 0.5 | 0.4 / 0.4 | 0.4 / 0.4 |
| 미지 5 m(지터 모델), 시선 방향 | 0.6 / 0.7 s | 0.5 / 0.5 | 0.4 / 0.4 | 0.4 / 0.4 |
| 비교: rev1 베이스라인 KF χ² 3연속($q$=0.26/0.5, `rev3_checks.py` C) | **3 s 내 0 %** | — | 0.5–0.6 s(중앙값) | 0.4 s |

정지 물체 오탐(로봇 정지, 가우시안 지터): 사람 R 0 / 6.7 트랙·h, 미지 0.45 회/트랙·h(목표 ≤ 1).

**자유공간 위반 증거(광선 기반).** 점 $\mathbf p$에 대해 직전 $K=5$ 스캔 중 $\ge 2$개에서 $\mathbf p$를 지나는 빔의 끝점이 $\mathbf p$보다 **0.10 m 이상 멀었으면**(= 관측된 자유 공간) 위반. 미관측(가림) 셀은 증거 0(unknown ≠ free). 정지 점의 우연 위반: 스캔 쌍당 $\Phi(-0.10/(0.03\sqrt2))=0.0092$, 5중 2 요구 시 $\approx8\times10^{-4}$. $\rho_\text{free}$ = 위반 점 비율. 운동 방향별(`rev4_dyn.out` FS):

- 접근(센서 쪽): 전면이 자유였던 공간으로 들어가므로 변위 0.10 m 후 $\rho_\text{free}\to1$: 0.43 / 0.20 / 0.17 s (0.3 / 1.0 / 1.5 m/s).
- 횡단: 이동 중 첫 관측이면 선단 띠 폭 $\approx v\Delta t(K-K_f+1)=0.4v$ → $\rho_\text{free}=\min(1,0.4v/W)$ = 0.40 / 1.0 / 1.0(사람 가시폭 W 0.30 m); 정지→출발이면 $\rho_\text{free}\ge0.5$까지 0.60 / 0.25 / 0.20 s.
- 이탈(센서에서 멀어짐): 뒤쪽은 물체 자신에 가려 unknown → 증거 없음 → 속도 경로만.

**속도별 지연 예산(최초 검출 스캔 또는 운동 시작 → `is_dynamic=true` 발행, p90, 처리 ≤ 0.05 s 포함).** 생명주기 하한 0.2 s(3번째 히트). 속도 경로는 몬테카를로(`rev4_dyn.out` V, 정지→출발은 `rev4_stopgo.out`), 자유공간 경로는 위 기하 모델(P3 판정 임계 $\rho^*=0.12$: 미지 클래스에서 $4\rho_\text{free}>-\operatorname{logit}0.382$).

| 속도 | B1 속도 경로: 횡단 / 시선(접근·이탈) / 정지→출발 | P3 자유공간 경로: 접근 / 횡단(이동 중 등장) / 정지→출발(횡단) | 예산(p90) |
|---|---|---|---|
| 0.3 m/s | 0.65 / 0.75 / 0.75 s | 0.50 / 0.25 / 0.30 s | **≤ 0.8 s** |
| 1.0 m/s | 0.45 / 0.45 / 0.45 s | 0.25 / 0.25 / 0.20 s | **≤ 0.5 s** |
| 1.5 m/s | 0.45 / 0.45 / 0.35 s | 0.25 / 0.25 / 0.20 s | **≤ 0.5 s** |

이탈(센서에서 멀어짐)은 자유공간 증거가 없어 속도 경로 값이 그대로 예산이다. B1만으로도 예산을 만족하며(0.3 m/s 행은 로봇 정지 또는 사람 클래스 트랙 조건 — 위 한계 참조), P3는 접근·횡단에서 지연을 줄이는 것이 가설이다(§5.3). rev1의 일괄 "≤ 0.5 s"는 0.3 m/s에서 근거가 없었으므로 속도별 예산으로 대체하고 0.3 m/s 시나리오를 평가에 추가했다(§6). **베이스라인(B1)의 동적 판정**: 배경 분리 후 남은 클러스터에서 LS 검정 발화 → `is_dynamic`(해제: 5스캔 연속 미발화). P3(§5.3)는 여기에 자유공간·클래스·정지 증거를 더한다.

### 3.7 예측과 TTC (obstacle_tracker_node 계약)

components.md/sequences.md §2는 TTC를 추적 노드가 `plan` + `odometry/filtered_map`으로 계산하도록 정했다.

- **예측**: 트랙 평균은 CV(B1) 또는 IMM 혼합(P1)으로 $\tau\in[0,T_h]$, $T_h=5.0$ s(> τ_warn 3.0 s), 0.1 s 간격. 공분산 성장 $P(\tau)=F(\tau)PF(\tau)^\top+Q(\tau)$, 위치 성분 $P_{pp}(\tau)=P_{pp}+2\tau P_{pv}+\tau^2P_{vv}+q\tau^3/3$. $q$=0.25에서 정상상태 위치 σ: 1.0 s 0.36 m, 1.5 s 0.62 m, 3.0 s 1.62 m($q$=0.1: 0.24/0.41/1.05 m) — 불확실성 확대는 정직하게 전달하고, 정지 모드가 우세한 트랙은 IMM이 성장을 줄인다.
- **로봇 궤적**: `plan`(map)을 현재 속도 $\max(|v_r|,0.2)$ m/s로 따라가는 위치 $\mathbf p_r(\tau)$.
- **충돌 판정**: $\|\mathbf p_r(\tau)-\hat{\mathbf p}_o(\tau)\|\le r_\text{robot}+r_o+\min(k\sigma_o(\tau),0.5\ \text{m})$, $r_\text{robot}=\sqrt{0.3^2+0.2^2}=0.361$ m, $r_o$ = 트랙 반경(클래스 사전: 사람 0.30, 적재 AMR 0.361, 지게차 0.8, 미지 = 보정 클러스터 외접원), $k=1$, $\sigma_o$ = 연결 방향 위치 σ. TTC = 최초 충돌 $\tau$, 없으면 `inf`(메시지 규약). 정적 트랙도 계산(경로 위 비지도화 정적물).
- **요구 확정 거리**(정면 접근, 확정·동적 지연 $\ell$): $d_\text{req}=(v_r+v_o)(\tau_\text{warn}+\ell)$ → 로봇 2.0 + 사람 1.0 m/s, $\ell$=0.5 s: **10.5 m**; 1.0+1.0: 7.0 m; 2.0+1.5: 12.3 m; 2.0+0.3($\ell$=0.8 s, 0.3 m/s 예산): 8.7 m(`rev5_fixes.out` R; $\ell$=0.3/0.8 s 민감도는 `rev4_dyn.out` L). 사람 다리의 10 m 확정 확률은 최소 점 수 2에서 98 %(모델). 12 m 이상이 필요한 조합(2.0+1.5)은 인지 한계로 플래너 팀에 **순항 속도 제한 또는 카메라 선행 트랙**을 요구사항으로 전달한다.
- **마감**: 매 스캔 발행(빈 배열 포함), `header.stamp` = 스캔 스탬프, 스캔 도착→발행 p99 ≤ 30 ms.

### 3.8 다중 로봇: 피어 AMR 주입

§3.0에 따라 무적재 AMR은 LiDAR에 보이지 않는다. `obstacle_tracker_node`는 파라미터 `peers: [amr_02, …]`의 `/amr_XX/odometry/filtered_map`(50 Hz)을 구독해 클래스 `amr`(p=1), 반경 0.361 m, `source=PEER` 트랙으로 주입한다. 수신 나이 $a$(= now − stamp)만큼 CV 외삽하고 공분산에 $Q(a)$를 더한다. 평가에서는 multi_robot.md §6과 같은 0–100 ms 균등 지연을 주입해 견고성을 확인한다. 적재 AMR의 LiDAR 클러스터와 카메라 `amr` 검출은 같은 게이트로 피어 트랙에 연관해 중복 트랙을 만들지 않는다. 자기 자신은 제외. 이 결정은 30회 회피 시험(다른 로봇이 동적 장애물)의 임계 경로이므로 오픈 질문에서 설계 결정으로 승격했다.

### 3.9 복잡도(로봇 1대, 스캔 1회)

배경 라벨 $O(N)$(거리변환 LUT, 지도 변경 시 1회 계산), ABD $O(N)$, 클러스터 $k\le40$, 트랙 $n\le30$, 게이트 $O(nk)$, 확장 행렬 Hungarian $O((n+k)^3)\approx 3.4\times10^5$, 자유공간 광선 720빔 × 최대 200셀 ≈ $1.4\times10^5$, TTC 50스텝 × $n$ → C++로 목표 < 2 ms.

## 4. 문헌 조사 (2023‑09 → 2026‑09; VERIFIED = arxiv.org/abs·html 또는 공식 페이지 WebFetch)

| # | 논문 (venue, year) | 관련성 / 우리가 취하는 것 (rev2 수정 반영) |
|---|---|---|
| 1 | Poly‑MOT, IROS 2023 (2307.16675) | 클래스별 운동모델·유사도 — P1의 선행(3D 검출기의 하드 클래스) |
| 2 | Fast‑Poly, RA‑L 2024 (2403.13443) | 필터 기반 3D MOT의 실시간 SOTA — 고전 필터 설계의 타당성 |
| 3 | IMM‑MOT, 2025 (2502.09672) | IMM을 **5개 차량 클래스(bicycle, bus, car, motorcycle, truck)에만 선택 적용**, 모델 뱅크 {CV, CA, CTRV, CTRA}, Damping Window 생명주기 — rev1의 "클래스 무관" 기술은 오류(클래스 선택적). P1의 차별성을 약화시킨다 |
| 4 | MCTrack, IROS 2025 (2409.16149; IROS 채택은 공식 GitHub README "accepted to IROS 2025") | 속도·가속도 등 **운동 정보 출력 품질 지표** 제안 → v/θ RMSE 채택. (rev1의 "BEV 평면 매칭" 주장은 확인한 초록/README에 없어 삭제) |
| 5 | UCMCTrack, AAAI 2024 (2312.08952) | 지면 평면 KF + 사영 불확실성 반영 Mapped Mahalanobis Distance — 측정 공분산 전파의 선행 |
| 6 | UncertaintyTrack, ICRA 2024 (2402.12303) | 검출 위치 불확실성 활용, ID switch 약 19 % 감소 |
| 7 | UTrack, ECCV 2024 UnCV WS (2408.17098) | 검출 예측분포를 연관에 사용 |
| 8 | SG‑LKF, 2025 (2508.00358) | 자차 속도에 따른 불확실성 적응을 **학습된 MotionScaleNet(MLP)** 으로 예측(KITTI/nuScenes). rev1의 수작업 $\beta$ 항의 근거가 될 수 없어 항 자체를 삭제 |
| 9 | OptiPMB, T‑ITS 2025 (2503.12968) | PMB: 측정 주도 탄생·적응 검출확률 — 확장 행렬의 $P_D$/탄생 비용 설계 참고 |
| 10 | Wei, Liang, Meyer 2025 (2506.18124) | 신경망 강화 베이즈 MOT — 맥락 |
| 11 | Conf‑SLAMMOT, 2024 (2412.01041) | 신뢰도를 **인자 그래프** 연관에 사용 — 헝가리안 행 상수와 무관(rev1의 근거 주장 철회) |
| 12 | LV‑DOT, 2025 (2502.20607) | 실내 로봇 LiDAR‑비전 동적장애물 검출·추적 — 시스템 수준 최근접 선행 |
| 13 | UniMT/RTMCT, 2025→2026 v2 (2504.13647) | 다중 클래스 LiDAR‑카메라 + 클래스별 궤적 예측 |
| 14 | Plozza et al., IEEE SAS 2024 (2412.15000) | 2D LiDAR 사람 추적: 검출 **DR‑SPAAM** + 추적 **Norfair**, MOTA 85.45 %, 20 Hz, Jetson Xavier NX — 사람 트랙 참고 수준 |
| 15 | Semantic2D, 2024→2026 v2 (2409.09899) | 2D LiDAR 단독 의미 분할 — 데이터셋 부담으로 미채택 |
| 16 | Rached, Jia, Kondo, How, 2026 (2603.15826) | "순수 기하 LiDAR 파이프라인은 정적 구조물 근처·부분 관측 동적물을 놓친다" — 배경 라벨 분할·P3 동기 |
| 17 | Dynablox, RA‑L 2023 (2304.10049) | 보수적 자유공간 신뢰도 기반 3D 이동체 검출(86 % IoU, 17 FPS). **원형이 아니라** 2D DATMO([C17][C18])의 현대 3D 계승으로 위치 조정 |
| 18 | Le Gentil et al., IROS 2024 (2410.05152) | 스캔 왜곡 보정 + 시공간 동적 분류 — 실기 전이 시 de‑skew 근거 |
| 19 | MF‑MOS, ICRA 2024 (2401.17023) | 잔차 영상의 운동 단서 |
| 20 | MambaMOS, ACM MM 2024 (2404.12794) | 학습형 MOS(3D·라벨 필요, 미채택) |
| 21 | Zhu et al., ICRA 2025 (2506.07539) | 합성 데이터만으로 YOLOv8 mAP50 96.4 %; 재질·렌더링·후처리·distractor |
| 22 | Mueller et al., ACRA 2024 (2503.22965) | Unity 도메인 랜덤화, 팔레트 0.995 mAP50; 위치 < 4.2 cm는 **정면·5 m 이내 조건** |
| 23 | Rustler et al., IEEE Access 2025 (2501.07421) | 스테레오 깊이 오차의 **경험적** 비교(D435 1 m 이내 < 1 cm, ZED 2 4 m에서 < 3 cm); 함수형 노이즈 모델은 제시하지 않음 |
| 24 | Cai et al., COINS 2024 (2412.15040) | **ToF**(PMD Flexx2) 축방향 노이즈를 거리·**입사각**의 가우시안 함수로 모델링 — 이차식 $a+bZ^2$의 근거가 아님. 우리 이차 항은 프로젝트 결정(`sensors.yaml`, D435급 약 $0.0025d^2$ 주석)과 고전 [C6](Kinect) |
| 25 | FutrTrack, VISAPP 2026 (2510.19981) | 카메라‑LiDAR 트랜스포머 추적 — 예산·설명가능성상 미채택 |
| 26 | Meng et al., 2019 (1912.00603, ICRA 2020 투고) | **도로 맥락으로 IMM의 시변 전이확률행렬 조정** — P1의 "맥락으로 Π 조절"의 직접 선행 |
| 27 | Lim, Paek, Kong, "IMM‑based Multiple Object Tracking using a State Prediction Neural Network" (PR‑IMM), 2026‑09‑10 (2609.13307) | 레이더 MOT에서 트랜스포머 상태예측기를 CV/CA/CT 뱅크에 **추가 모델**로 넣고 Doppler 활용 — 최신 IMM‑MOT 계열(학습형, 미채택) |

고전(RECALLED; [C13]–[C18]은 WebSearch로 서지 확인, 출판사 페이지 403으로 본문 미페치): [C1] Kuhn 1955 / Munkres 1957; [C2] Bar‑Shalom & Tse 1975 PDA, Fortmann et al. 1983 JPDA; [C3] Blom & Bar‑Shalom 1988 IMM; [C4] Borges & Aldon 2004 ABD; [C5] Bewley et al. 2016 SORT; [C6] Nguyen, Izadi, Lovell 2012 Kinect 노이즈; [C7] Ahn et al. 2019 D435 노이즈; [C8] Jia et al. 2020 DR‑SPAAM; [C9] Przybyła 2017; [C10] Hartley & Zisserman 2003; [C11] Thrun et al. 2005; [C12] Mersch et al. 2022; **[C13] Challa & Pulford 2001 JTC(IEEE TAES 37(3)); [C14] Ristic, Gordon, Bessell 2004(Inf. Fusion 5) — 클래스 정합 필터들이 단일 IMM으로 집약됨; [C15] Bar‑Shalom, Kirubarajan, Gokberk 2005 분류 보조 연관(IEEE TAES 41(3)); [C16] Chavez‑Garcia & Aycard 2016(IEEE T‑ITS 17(2)) 다중 센서 분류 증거 융합; [C17] Wang, Thorpe, Thrun, Hebert, Durrant‑Whyte 2007 SLAMMOT/DATMO(IJRR 26(9)); [C18] Vu, Burlet, Aycard 2011(Inf. Fusion 12(1)) 격자 기반 이동체 검출 + MHT·적응 IMM**; [C19] Bar‑Shalom, Li, Kirubarajan 2001 교재(CWNA, 미지 선회율 CT 모델); [C20] Blackman & Popoli 1999(GNN 할당 비용·미연관 비용); [C21] Bernardin & Stiefelhagen 2008 CLEAR MOT 지표.

## 5. 제안 — 정직한 재포지셔닝

리뷰 판정: P1 already_published, P2 flawed, P3 flawed. rev2는 P1·P3를 **engineering_adaptation**으로 낮추고 결함을 고쳐 유지하며, P2는 독립 제안에서 제외한다. 각 제안의 주장은 "선행 기법의 이 스택·이 센서 구성 적용이 B1 대비 측정 가능한 개선을 준다"로 한정한다.

### 5.1 P1 — JTC‑IMM: 카메라 클래스 사후를 쓰는 클래스 의존 IMM (engineering_adaptation)

- **선행**: 클래스별 운동모델과 클래스 사후의 결합은 JTC [C13][C14](클래스 정합 필터 → 단일 IMM 집약), 분류 보조 연관 [C15], 맥락 기반 시변 Π [26], 카메라 분류 증거의 LiDAR 추적 융합 [C16], 클래스별 모델 [1][2], 클래스 선택적 IMM [3]에 이미 있다. 우리 몫은 **2D LiDAR 클러스터 + YOLO 후기 융합 + Gazebo 창고**라는 적용과 그 측정뿐이다.
- **상태·모드(수정)**: 공통 상태 $\mathbf x=[p_x,p_y,v_x,v_y,\omega]^\top$(odom, **데카르트 속도** — 헤딩 wrap과 $v\approx0$에서의 $\theta$ 비가관측 문제를 상태에서 제거). 모드 $\{\text{ST},\text{CV},\text{CT}\}$:
  - ST: $F=\operatorname{diag}(1,1,0,0,0)$(속도·선회율 리셋), $Q=\operatorname{diag}(\sigma_\text{st}^2,\sigma_\text{st}^2,\sigma_{v0}^2,\sigma_{v0}^2,\sigma_{\omega0}^2)$, $\sigma_\text{st}$=0.02 m, $\sigma_{v0}$=0.05 m/s, $\sigma_{\omega0}$=0.05 rad/s.
  - CV: §3.5와 **동일한 CWNA**($q_c$), $\omega\leftarrow0$($\sigma_{\omega0}$). B1과 IMM‑CV는 같은 모델이다.
  - CT(미지 선회율, [C19]): $s=\sin\omega\Delta t$, $c=\cos\omega\Delta t$,
  $$f(\mathbf x)=\begin{bmatrix}p_x+\frac{s}{\omega}v_x-\frac{1-c}{\omega}v_y\\ p_y+\frac{1-c}{\omega}v_x+\frac{s}{\omega}v_y\\ c\,v_x-s\,v_y\\ s\,v_x+c\,v_y\\ \omega\end{bmatrix},$$
  EKF 자코비안의 $\omega$ 열: $\partial p_x/\partial\omega=v_x\frac{\Delta t c\omega-s}{\omega^2}-v_y\frac{\Delta t s\omega-(1-c)}{\omega^2}$, $\partial p_y/\partial\omega=v_x\frac{\Delta t s\omega-(1-c)}{\omega^2}+v_y\frac{\Delta t c\omega-s}{\omega^2}$, $\partial v_x/\partial\omega=-\Delta t(s v_x+c v_y)$, $\partial v_y/\partial\omega=\Delta t(c v_x-s v_y)$; $|\omega|<10^{-4}$에서는 2차 테일러 분기($\partial p_x/\partial\omega=-\tfrac12\Delta t^2v_y$ 등). 중앙차분 대비 최대 오차 $2\times10^{-6}$(v=0, 소ω 분기 포함; `rev4_dyn.out` J). $Q_\text{CT}$ = CWNA($q_c$) ⊕ $q_\omega\Delta t$.
- **클래스 조건부 파라미터**: $\Pi(p)=\sum_c p(c)\Pi_c$, $q_j(p)=\sum_c p(c)q_{j,c}$. 다섯 클래스 모두 정의(행 합 1 확인):
  $\Pi_\text{person}=\begin{bmatrix}.90&.08&.02\\.05&.85&.10\\.05&.15&.80\end{bmatrix}$, $\Pi_\text{forklift}=\begin{bmatrix}.95&.05&0\\.02&.90&.08\\.02&.08&.90\end{bmatrix}$, $\Pi_\text{amr}=\begin{bmatrix}.95&.05&0\\.02&.93&.05\\.02&.08&.90\end{bmatrix}$, $\Pi_\text{box}=\Pi_\text{sign}=\begin{bmatrix}.99&.01&0\\.30&.70&0\\.30&.20&.50\end{bmatrix}$.
  사후로 Π를 혼합하는 것은 클래스 가설을 매 스텝 병합하는 **근사**다(정확한 JTC는 클래스별 필터 뱅크를 유지). 이 근사의 손실은 B2(아래)와의 비교로 드러난다.
- **IMM 순환**([C3]): 혼합 $\mu_{i|j}=\Pi_{ij}\mu_i/\bar c_j$, $\hat{\mathbf x}^{0j}=\sum_i\mu_{i|j}\hat{\mathbf x}^i$, $P^{0j}=\sum_i\mu_{i|j}[P^i+(\hat{\mathbf x}^i-\hat{\mathbf x}^{0j})(\cdot)^\top]$(각도 성분이 없어 산술 평균이 타당); 모드별 예측·갱신, $\mu_j\propto\Lambda_j\bar c_j$; 출력 혼합.
- **연관용 공분산(수정)**: $\bar{\mathbf z}=\sum_j\bar c_j\hat{\mathbf z}_j$, $S_\text{assoc}=\sum_j\bar c_j\big[S_j+(\hat{\mathbf z}_j-\bar{\mathbf z})(\hat{\mathbf z}_j-\bar{\mathbf z})^\top\big]$(평균 간 퍼짐 항; 예: 모드 예측이 0.3 m 벌어지면 대각 0.050 → 0.0725). 모드가 엇갈리는 기동 중 게이트가 좁아지는 rev1의 결함 제거.
- **클래스 속도 게이트 삭제(리뷰 수정안과 다른 결론)**: 평균형 $v_\max(p)$는 경계가 아니고(0.8 m/s 예), 리뷰가 제안한 상한 포락 $\max\{v_{\max,c}:p(c)\ge\tau\}$도 $p\le0.95$ 캡 아래에서는 나머지 4클래스가 각 0.0125를 가지므로 $\tau\le0.0125$면 전역값 3.0 m/s(무효), $\tau\ge0.02$면 0.5 m/s(오분류된 보행자 절단)다(`rev4_dyn.out` J). 따라서 클래스 무관 물리 상한 $v_\text{phys}$=3.0 m/s(§3.5)만 둔다.
- **클래스 사후 갱신(수정)**: 한 LiDAR 주기에 연관된 카메라 검출 중 최고 점수 1개만 사용(주기당 1회), 점수 구간별 혼동행렬 우도 $L(\hat c\mid c)=M^{(b)}_{c,\hat c}$(합성 val에서 추정, 행=참 클래스), 프레임 간 상관을 위한 템퍼링 $\beta=0.5$, 캡과 망각을 식에 명시:
  $$p_k(c)\propto\big[(1-\eta)p_{k-1}(c)+\eta\pi_0(c)\big]\,\big(M^{(b)}_{c,\hat c}\big)^{\beta},\qquad p_k\leftarrow\operatorname{cap}_{0.95}(p_k),\ \eta=0.02/\text{주기}.$$
  rev1의 $\varepsilon+(1-\varepsilon)M s$는 점수가 모든 클래스에 같게 곱해져 우도가 아니었다. 수치(`rev4_extra.out` d): 고점수 'person' 연속 시 0.74 → 0.95(캡), 저점수 시 0.57 → 0.80 → 0.92 → 0.95; 반대 증거 5회로 person 0.95 → 0.04; 카메라 증거 없으면 6 s 후 0.49로 감쇠. $\beta$는 검증 시퀀스의 사후 NLL로 격자 탐색.
- **카메라 검출은 클래스 전용(OOSM 제거)**: 카메라 3D 측정은 최대 ~70 ms 늦게 도착하므로 위치 갱신에 쓰지 않는다. 연관만 카메라 스탬프 시각으로 **역예측**한 트랙 위치(최근 1 s 사후 상태 링버퍼에서 직전 LiDAR 사후를 CV로 전진)에 대해 수행한다. rev1의 "도착 즉시 비동기 위치 갱신"은 1.5 m/s 물체에 최대 15 cm의 과거 위치를 좁은 공분산으로 주입했다.
- **비교 기준(baseline to beat)**: **B1**(CV‑KF, 같은 측정 모델·GNN)과 **B2**(같은 3모드 IMM, 클래스 무관 $\Pi=\frac15\sum_c\Pi_c$, $q$ 고정). P1 효과 = B2 대비, IMM 효과 = B1 대비로 분리한다.
- **가설(검증 대상, 주장 아님)**: 사람 정지‑출발·선회 시나리오에서 FDE@1.5 s가 B1 대비 ≥ 15 % 감소, 정지 박스의 가짜 속도(|v̂|) 감소; B2 대비 증분은 작을 수 있으며(≥ 5 %가 가설) 그대로 보고한다. NEES/NIS가 χ² 대역에 드는지로 튜닝 적합성을 판정.
- **비용**: 트랙당 필터 3개(5차원), ≤ 30 트랙 × 10 Hz → < 0.3 ms.

### 5.2 (구) P2 UPA — 독립 제안에서 제외

리뷰의 네 결함(신뢰도 항 무효, $R_\text{cl}$ 축 오배정·지속 편향의 백색화, 측정마다의 공통 모드 포즈 공분산, 유도되지 않은 자차 속도 항)을 인정한다. 올바른 부분 — 측정 공분산의 해석적 전파(§3.1, §3.4), 확장 행렬 NLL 연관(§3.5) — 은 고전 공학 관행([5][6][C20])이므로 **베이스라인에 흡수**했고, 신뢰도 비용 항과 $\beta\|\mathbf v_\text{ego}\|^2\Delta t^2$ 항은 삭제했다. NEES 보정 시험(§6)은 이제 편향 보정과 odom 프레임 덕분에 통과 가능성이 있는 형태다.

### 5.3 P3 — 2D DATMO 일관성 로그오즈 동적 점수 (engineering_adaptation)

- **선행**: 이전에 관측된 자유 공간을 침범한 측정을 이동체로 보는 일관성 기반 검출은 2D 레이저 DATMO [C17][C18]가 원형이며, Dynablox [17]는 그 3D 현대판이다. 정적 구조물 근처 누락 문제 [16]를 배경 라벨 분할(§3.3)과 함께 다룬다.
- **점수(수정)**: 나이브 베이즈 로그우도비의 합으로

$$\ell=\underbrace{\operatorname{logit}\pi_\text{mv}}_{\text{기저율}}+\underbrace{\big[\operatorname{logit}\bar p_\text{mv}(p)-\operatorname{logit}\pi_\text{mv}\big]}_{\text{클래스}}-w_\text{map}\max\!\Big(0,\tfrac{\rho_\text{map}-\rho_0}{1-\rho_0}\Big)+w_\text{free}\,\rho_\text{free}+w_\text{vel}\,\mathbb 1[\text{LS 발화}]-w_\text{st}\,\mathbb 1[\text{확신 정지}],$$

$P(\text{dyn})=\sigma(\ell)$. $\bar p_\text{mv}(p)=\sum_c p(c)p_{\text{mv},c}$, $p_\text{mv}$=(box .02, sign .01, person .60, forklift .50, amr .60)는 "그 클래스가 **지금 움직일** 확률"(rev1의 "동적 클래스 사전" 0.90과 구분), $\pi_0$=(.30, .05, .30, .15, .20)은 LiDAR 가시 클러스터의 클래스 기저율(로그로 재추정), $\pi_\text{mv}=\sum\pi_0p_\text{mv}=0.382$ → 카메라 증거가 없으면 클래스 항은 정확히 0. 지도 항은 **0 이하**(rev1은 $\rho_\text{map}=0$에 +2.0을 주어 비지도화 정적물이 0.894로 동적 판정), $\rho_0=0.5$. $\rho_\text{free}$는 관측된 자유 공간만(§3.6), LS 발화는 §3.6의 단일 정의(유지: 5스캔 연속 미발화까지), "확신 정지" = 창이 가득 찬 상태에서 $\|\hat{\mathbf v}\|<0.10$ m/s가 10스캔 연속(정지 물체 검정당 통과 확률 0.986). 초기 가중치 $w=(w_\text{map},w_\text{free},w_\text{vel},w_\text{st})=(4,4,4,2)$, `is_dynamic`: $P>0.5$ 설정 / $<0.35$ 해제.
- **진리표**(`rev4_dyn.out` D):

| 상황 | P(dyn) |
|---|---|
| 지도에 있는 선반 다리(오정렬로 배경 분리 실패, $\rho_\text{map}$ .95) | 0.017 |
| 비지도화 정적 팔레트, 초기(증거 없음) / 1 s 정지 확인 후 | 0.381 / 0.077 |
| 비지도화 정적물, 노이즈 $\rho_\text{free}$ 0.05 | 0.430 |
| 대형 박스 p(box)=.8 | 0.101 |
| 서 있는 사람 p=.8, 초기 / 1 s 정지 후 | 0.536 / 0.135 |
| 주차 지게차 p=.8, 1 s 정지 후 | 0.104 |
| 선반 0.2 m 옆 보행자($\rho_\text{map}$ .6, free .8, 속도 발화) | 0.997 |
| 0.3 m/s 횡단 보행자 초기(free .4, 속도 미발화, 클래스 없음) | 0.753 |
| 접근 보행자(free 1.0, 클래스 없음) / 이탈 보행자(속도만) | 0.971 / 0.971 |
| 1 m/s로 밀리는 박스 p(box)=.8 | 0.996 |

서 있는 사람의 초기 0.536(>0.5)은 1 s 이내 과도적 보수 판정으로 의도한 것이며 P/R에 그대로 집계한다. 가중치는 GT 레이블(0.5 s 평균 $|v_\text{GT}|>0.1$ m/s) 로그에 대한 **로지스틱 회귀**(특징 = 위 5항)로 재적합한다 — 식이 로지스틱 형태이므로 적합 결과가 곧 보정된 LLR이다.
- **비교 기준**: sequences.md §2의 자리표시 규칙 "KF 속도 $|v|>0.2$ m/s"(B0‑dyn)와 B1(배경 분리 + LS 검정만). **지표**: 속도별 판정 지연 p90(0.3/1.0/1.5 m/s × 방향), 동적 P/R, 정지 트랙당 시간당 오탐, 선반 0.2 m 옆 보행자 재현율. **가설**: 0.3 m/s 접근·횡단에서 지연 p90을 B1 대비 ≥ 30 % 단축(자유공간 경로), 오탐 ≤ 1 /트랙·h.
- **비용**: 720빔 광선 × 최대 200셀 + 링버퍼 K=5 → < 0.5 ms.

### 5.4 의사코드 (obstacle_tracker_node, 10 Hz, odom 프레임)

```
on_scan(scan):                                       # scan_filtered, +inf kept
  T_ol = tf(odom<-lidar_link, scan.stamp); T_mo = tf(map<-odom, scan.stamp)
  pts  = project(scan, T_ol)                         # sim: no intra-scan de-skew needed
  bg   = map_distance_lut(T_mo^-1 * /map) <= 0.10    # per-point background label
  segs = abd_segment(pts, bg, dphi=0.5deg, lambda=10deg, sigma_r=0.03)  # split on label change
  cls  = merge_min_gap(segs, 0.10, same_label=True); resplit_by_size_prior(cls)
  for c in cls: c.z, c.R, c.Rjit = cluster_model(c, class_prior(c))      # bias shift + R_cl
  for t in tracks: t.predict(dt)                     # B1: CV-KF | P1: IMM(Pi(p), q(p))
  inject_peers(tracks, peer_odom, now)               # /amr_XX/odometry/filtered_map, age-extrapolated
  C = augmented_cost(tracks, cls, gate=9.21, PD, lambdaB=0.01, BIG=1e6)
  A = hungarian(C)                                   # own C++ impl; pytest oracle = scipy
  update/miss/birth per A; lifecycle(3 of 5, max_miss 5)
  for t in tracks:    t.vel_test = ls_slope_test(t.buffer, W=10, thr=13.82, vmin_eff(t, ego))  # tentative too; only confirmed published
                      t.rho_free = free_space_violation(t, ray_ring[K=5], margin=0.10, kf=2)
                      t.p_dyn = datmo_logodds(t); t.is_dynamic = hysteresis(t.p_dyn, .5, .35)
  out = to_map(tracks, T_mo, Sigma_pose)             # pose covariance added once, here
  for t in out: t.ttc = ttc(t, plan, v_robot, T_h=5.0)
  publish(tracked_obstacles(stamp=scan.stamp), tracked_markers); health.tick()

on_detected_objects(msg):                            # 15 Hz, class-only (no position update)
  for d in msg.objects:
    t* = gate(retrodict(tracks, d.stamp), d.position, d.cov)
    if t*: t*.pending_class = best_score(t*.pending_class, d)
  # applied once in the next on_scan: p <- cap(((1-eta)p + eta pi0) * M_b[:,c]^beta)
```

### 5.5 리스크와 완화

| 리스크 | 완화 |
|---|---|
| 클래스 사후 오류 → 부적절한 Π·q | 캡 0.95, 템퍼링 β, 망각 η, 모든 클래스에 CV 모드 포함, B2 비교로 손실 측정 |
| Gazebo 액터가 gpu_lidar에 보이지 않을 가능성 | W1 1–2일차 차단 시험(§7.8), 실패 시 시뮬 팀이 액터 visual/collision 추가 — 전 4.7 평가의 선행 조건 |
| 무적재 AMR·저상 화물이 LiDAR에 안 보임 | 피어 주입(§3.8), voxel layer·카메라 검출 |
| 원거리(> 8 m) 사람 다리 점 부족 | 최소 점 2(> 6 m), 카메라 선행 클래스, 속도 제한 요구를 플래너에 전달(§3.7) |
| 로봇 통과 시 잔차 편향 회전 → 가짜 속도 | 클래스 편향 보정 + $v_{\min,\text{eff}}$ 적응 + 자유공간 증거(회전에 무관) |
| 혼잡 호스트로 GPU/CPU 실측 불확실 | W3 유휴 호스트 재측정 게이트, 실패 시 2D 30 Hz 유지·운용률 15 Hz |
| sim‑to‑real(깊이 상관, 스캔 왜곡) | `depth_noise_iid:false`, de‑skew 경로, 모든 계수 YAML 외부화 |

## 6. 평가 계획

### 6.1 시나리오 (30 시나리오 × 5 시드, 평균 ± 표준편차; 대응 t‑검정 + Wilcoxon)

속도 {0.3, 1.0, 1.5} m/s × 운동 {접근, 횡단, 이탈, 정지‑출발, 곡선, 무작위} 보행자/지게차; **선반 0.2 m 옆 보행자**; 비지도화 정적 팔레트·주차 지게차; 적재/무적재 피어 AMR 교차(0–100 ms 지연 주입); 로봇이 0.5–2.0 m/s로 정적 물체 옆 통과(정지 물체 오탐) 및 **통과 중 0.3 m/s 미지 클래스 이동체**(B1 vs P3 지연, §3.6 한계); AMCL 점프/납치 복구 중 오탐; 결함 주입(스캔 정지, GPU 끔, TF 지연 0.3 s).

### 6.2 지표 정의와 로그 포맷(스펙 4.10 표준화)

- **GT**: 로봇 `ground_truth/odom`(평가 전용), 장애물은 Gazebo 동적 포즈(Pose_V → TFMessage 평가 전용 브리지, W1 확인) 100 Hz. **평가 대상**: LiDAR 가시(상단 > 0.38 m 또는 피어), 거리 ≤ 10 m, GT 기하로 계산한 시선 비가림.
- **MOTA/IDSW(CLEAR MOT [C21])**: 프레임별 GT↔트랙 Hungarian 매칭, 지면 평면 중심 거리 임계 0.5 m. 로그 `[t, gt_id, gt_x, gt_y, gt_vx, gt_vy, class, trk_id, trk_x, trk_y, trk_vx, trk_vy, matched]` → `logs/perception/mot_*.csv`.
- **예측 정확도(동료평가 "KF 예측 정확도")**: 확정 트랙마다 시각 $t$에 $\hat{\mathbf p}(t+1.0)$, $\hat{\mathbf p}(t+1.5)$를 기록하고 GT와 비교(FDE@1.0/1.5 s). 로그 `[t, trk_id, horizon, pred_x, pred_y, gt_x, gt_y, err]`. 클래스·속도·모드별 집계, B1/B2/P1 비교.
- **일관성**: NIS(갱신마다)와 합성 궤적 NEES; 3D 투영 NEES는 base 프레임에서 **대역당 N=300 독립 표본**(같은 물체 0.5 s 이상 간격)의 평균이 $[\chi^2_{2N}(0.025)/N,\ \chi^2_{2N}(0.975)/N]=[1.78, 2.23]$(N=1000이면 [1.878, 2.126]) 안이면 합격. rev1의 "2 ± 0.3"은 N이 없었다.
- **3D 위치(4.6)**: 대역별 RMSE/최대(§3.1 목표), base·map 두 프레임, 보정 분할/홀드아웃 분할 분리(κ, σ_δ를 보정 분할에서 추정).
- **동적 판정(4.7)**: 트랙‑프레임 P/R(GT 이동 = 0.5 s 평균 $|v|>0.1$ m/s), **최초 가시 → `is_dynamic`** 지연 p50/p90을 속도·방향별로(§3.6 예산과 비교), 정지 트랙‑시간당 오탐 수.
- **TTC**: GT 궤적과 같은 알고리즘으로 계산한 TTC 대비 절대오차; 재계획 트리거 순간의 트랙 거리 vs $d_\text{req}$.
- **신뢰도**: 10구간 신뢰도 도표, ECE ≤ 0.05(로지스틱 보정 후).
- **검출(4.6)**: 합성 val 2k(별도 조명/텍스처 시드) mAP50/mAP50‑95, 클래스별 P/R; 5 스트림 동시 스트림별 FPS(60 s) ≥ 30, 지연 p50/p95(§3.2 예산), CPU 폴백 단일 스트림 FPS ≥ 10.
- **자원**: 스캔당 처리시간 p50/p99, 5대 CPU(`/proc/stat`), GPU 사용률/VRAM.

### 6.3 비교군

B0: CV‑KF + 최근접 연관(SORT‑lite), 동적 = $|v|>0.2$ m/s(sequences.md 자리표시) · **B1**: 본 베이스라인 · **B2**: 클래스 무관 IMM · **P1**: JTC‑IMM · **P3**: B1 + DATMO 로그오즈 · 전체 = P1+P3. 절제: −배경 라벨 분할, −편향 보정, −odom 프레임(map 추적), −$\ln\det S$ 항.

### 6.4 스펙 하드 요구

30회 회피 시나리오 충돌 0(플래너 팀과 공동; 우리 기여는 확정 동적 거리·TTC 로그), ≥ 3클래스 마커 시각화, GPU 30 FPS(또는 CPU 10 FPS 대안), 5대 CPU ≤ 80 %, 주요 모듈 커버리지 ≥ 70 %.

## 7. 구현 계획

### 7.1 패키지·노드 (components.md §3.4 이름 준수, 모두 `amr_perception`)

- `yolo_node`(Python): §3.2. 파라미터 `weights`, `imgsz`, `quantize`, `robots`, `batch_timeout_ms: 10`, `conf_thresh: 0.35`, `iou: 0.5`, `backend: auto|cuda|cpu`, `cpu_imgsz: 320`, `cpu_threads: 2`, `cpu_fleet_rate_hz: 5.0`(GPU 고장 저하 운용), CPU 경로는 `predict()` 대신 융합 모델 + 자체 레터박스/NMS.
- `object_localizer_node`(Python, numpy): §3.1. `image_geometry.PinholeCameraModel`로 `camera_info` K 사용, 깊이 트리거 동기(slop 0.017 s), ROI 노이즈 가산(`sensors.yaml depth_camera.noise_quadratic_coeff`), 표면→중심 보정, $\Sigma$ 전파, TF(map).
- `detection_marker_node`(Python): CUBE + TEXT_VIEW_FACING "Class: Box, Conf: 0.92, Dist: 1.5m"(스펙 예시 형식), 크기는 클래스 사전.
- `obstacle_tracker_node`(C++17/Eigen): 라이브러리 `tracking/{scan_geometry, background_lut, abd_segmenter, cluster_model, cv_kf, ct_model, imm, gnn_assignment(hungarian), lifecycle, vel_test, free_space, datmo_scorer, class_fusion, peers, ttc, health}`.
- 도구: `tools/synth_labeler.py`, `tools/train.py`, `tools/export_trt.py`(`quantize=16, batch=5`), `tools/eval_{mot,latency,nees,prediction}.py`(numpy/scipy; 로지스틱 IRLS 포함).
- 테스트: gtest — Hungarian vs 전수탐색(n+k ≤ 8), 확장 행렬 동치, CV/IMM NEES·NIS(합성 1000궤적), CT 자코비안 수치 비교, IMM 정지→출발 μ 추종, ABD·배경 라벨 분할(선반 0.2 m 옆 합성 스캔), 가로 양자화 분산 $s^2/24$, LS 검정 오탐/지연, DATMO 진리표 단조성, TTC 해석해 비교. pytest — 투영 왕복, 자코비안 vs 수치미분, 포즈 공분산 MC, 깊이 중앙값 분산, 라벨러 가림 판정. 커버리지 ≥ 70 %(`colcon test --coverage`, `pytest --cov`).

### 7.2 인터페이스 (components.md §5.4 기준 + 변경 제안)

| 노드 | 방향 | 이름 | 타입 | 비고 |
|---|---|---|---|---|
| `yolo_node` | Sub/Pub | `camera/image_raw` → `perception/detections_2d` | `sensor_msgs/Image` → `vision_msgs/Detection2DArray` | ≤ 30 Hz, 스탬프 = 이미지 |
| `object_localizer_node` | Sub | `perception/detections_2d`, `camera/depth/image_raw`, `camera/camera_info` | | 깊이 트리거 15 Hz |
| | Pub | `perception/detected_objects` | `amr_msgs/DetectedObjectArray` | map, ≤ 15 Hz(rev1의 ≤ 30 Hz 정정) |
| `detection_marker_node` | Pub | `perception/markers` | `visualization_msgs/MarkerArray` | |
| `obstacle_tracker_node` | Sub | `scan_filtered`, `/map`, `odometry/filtered_map`, `plan`, TF | | components.md 그대로 |
| | Sub(**추가 제안**) | `perception/detected_objects` | `amr_msgs/DetectedObjectArray` | 클래스 전용 갱신 |
| | Sub(**추가 제안**) | `/amr_XX/odometry/filtered_map`(피어) | `nav_msgs/Odometry` | §3.8 |
| | Pub | `perception/tracked_obstacles` | `amr_msgs/TrackedObstacleArray` | map, 10 Hz, reliable, 스탬프 = 스캔 |
| | Pub | `perception/tracked_markers` | `visualization_msgs/MarkerArray` | 속도 화살표 + id |
| 인지 공통 | Pub(**추가 제안**) | `perception/health` | `diagnostic_msgs/DiagnosticArray` | 1 Hz + 변화 시, §7.4 |

**메시지 확장 제안(`amr_msgs`, 추가 필드만 — 기존 필드 의미 불변)**: `DetectedObject` += `float64[9] position_covariance`(map, 3×3), `float32[] class_probs`. `TrackedObstacle` += `string class_name`, `float32 class_confidence`, `float32[16] state_covariance`([p_x,p_y,v_x,v_y], map, 행 우선), `float32 radius`, `float32 heading_std`, `float32 dynamic_probability`, `float32[3] mode_probabilities`, `uint8 source`(0 LIDAR, 1 PEER). 기존 `is_dynamic`, `confidence`, `heading`, `velocity`, `time_to_collision`(inf = 비충돌)은 §3.5–3.7 정의로 채운다. `object_localizer_node`의 노이즈 가산은 components.md §3.4 표에 추가되어야 한다.

### 7.3 플래너/TTC/VO 팀과의 계약

- 좌표·시각: `tracked_obstacles`는 frame `map`, `header.stamp` = 스캔 스탬프(소비자는 나이를 계산해 외삽), 매 스캔 발행(빈 배열 포함), 스캔→발행 p99 ≤ 30 ms.
- 예측 모델: 소비자는 CV 외삽 $\mathbf p+\tau\mathbf v$와 공분산 성장 $P_{pp}(\tau)=P_{pp}+2\tau P_{pv}+\tau^2P_{vv}+q\tau^3/3$($q$=0.25, `state_covariance`에서 $P$ 사용), 지평 ≤ 5 s.
- VO/DWA 입력: 위치, 속도, `radius`(+ $k\sigma$ 팽창은 소비자 선택), `dynamic_probability`(> 0.5 = `is_dynamic`).
- TTC: §3.7 알고리즘으로 추적 노드가 계산, BT `IsTTCBelowThreshold`(τ_warn 3.0 s)와 `safety_node`(τ_crit 2.15 s)가 소비.
- 요구 확정 거리 표(§3.7)를 순항 속도 결정의 입력으로 제공.

### 7.4 인지 파이프라인 건강 감시(스펙 4.7 센서 고장)

원시 센서 타임아웃은 `safety_node`가 `robot_params.yaml safety.sensor_timeouts`(LiDAR 0.3 / depth 0.2 / RGB 0.1 s)로 처리한다(중복 구현 안 함). 인지는 **처리 체인**을 감시해 `perception/health`로 보고:

| 항목 | WARN | ERROR | 대응 |
|---|---|---|---|
| 추적 입력 스캔 나이 / 처리 p99 | > 0.15 s / > 50 ms | > 0.3 s / 마감 3회 연속 초과 | ERROR 시 `safety_node`가 `tracked_obstacles` 나이 > 0.3 s를 LiDAR 체인 고장으로 보고 **정지**(safety 팀에 제안) |
| TF odom←lidar / map←odom 조회 나이 | > 0.1 s | > 0.3 s | map←odom만 실패: odom 추적 지속, 출력은 마지막 변환 + WARN |
| YOLO 추론 지연 / GPU 가용 | p95 > 50 ms | GPU 없음 | CPU 백엔드 자동 전환(§3.2), 클래스 없는 추적 지속 |
| 깊이 짝짓기율 | < 12 Hz | < 5 Hz | 3D 검출 중단 표시 |

`fleet_adapter_node`가 ERROR를 `/fleet/alerts`로 중계(대시보드 알림).

### 7.5 costmap 설정(스펙 4.4; 소유: navigation — `nav2_params.yaml`에 제안값)

- 어떤 데이터가 어디로: **obstacle layer = `scan_filtered` 전체 현재 반환**(정적+동적, 추적기와 독립된 안전망), **voxel layer = `camera/depth/points_filtered`**(저상 화물·무적재 AMR), **추적 목록 = 배경 분리 후 클러스터 + 피어** → DWA 플러그인(예측·VO), BT/safety(TTC). rev1의 선택적 `perception/dynamic_cloud` 관측 소스는 **삭제**(동일 물체 이중 표시·잔상 원인).
- local costmap obstacle layer(`scan_filtered`): `marking: true`, `clearing: true`, `observation_persistence: 0.0`(최신 스캔만 → 이동체 잔상 없음), `expected_update_rate: 0.3`(= 3 주기, safety LiDAR 타임아웃과 일치), `inf_is_valid: true`(+inf 빔으로 clearing), `obstacle_max_range: 6.0`, `raytrace_max_range: 8.0`.
- voxel layer(`camera/depth/points_filtered`): `observation_persistence: 0.0`, `expected_update_rate: 0.2`(15 Hz × 3), `obstacle_max_range: 5.0`(`pointcloud_filter_node` max_range 5 m).
- local costmap `update_frequency: 10.0`(LiDAR와 같음; 1.5 m/s 물체의 갱신 간 변위 0.15 m = 3셀), `publish_frequency: 5.0`. CPU 초과 시 multi_robot.md §7대로 5/2 Hz로 낮추고 이 경우 동적 물체는 추적 목록 경로가 주도한다.

### 7.6 파라미터 YAML (`src/amr_perception/config/perception.yaml`, 재빌드 없이 변경)

```yaml
/**/obstacle_tracker_node:
  ros__parameters:
    frames:    {tracking: odom, output: map}          # launch가 amr_XX/ 접두사 주입
    scan:      {sigma_r: 0.03, keep_inf: true}
    background: {r_bg: 0.10, r_overlap: 0.25}
    abd:       {lambda_deg: 10.0, merge_min_gap: 0.10, min_points_near: 3, min_points_far: 2, far_range: 6.0}
    size_prior_max: {person: 1.0, unknown: 1.5}
    cluster_model:                                    # LiDAR bias mu, sigma_delta, sigma_lat  [m]
      person:   [0.061, 0.027, 0.014]                 # sigma_lat = shape part only (quantisation s^2/24 added in code)
      box:      [0.225, 0.029, 0.015]
      amr:      [0.200, 0.037, 0.012]
      forklift: [0.465, 0.057, 0.051]
      unknown:  [0.15, 0.10, 0.03]
      sigma_floor: 0.02
      sigma_seg: 0.03                                 # frame-to-frame jitter for the slope test
    kf:        {q_cwna: 0.25}                         # m^2/s^3, B1 and IMM-CV share it
    gnn:       {gate_chi2: 9.21, pd_visible: 0.9, pd_occluded: 0.5, lambda_birth: 0.01, big: 1.0e6, v_phys: 3.0}
    lifecycle: {confirm_hits: 3, confirm_window: 5, max_misses: 5}
    vel_test:  {window: 10, chi2: 13.82, v_min: 0.15, consecutive: 2, release: 5, static_v: 0.10, static_scans: 10}
    free_space: {window: 5, min_free: 2, margin: 0.10}
    imm:
      enabled: false                                  # true = P1
      st: {sigma_pos: 0.02, sigma_v0: 0.05, sigma_w0: 0.05}
      q_c:      {person: 0.25, forklift: 0.25, amr: 0.5, box: 0.1, sign: 0.1}     # m^2/s^3
      q_omega:  {person: 1.0, forklift: 0.25, amr: 0.25, box: 0.1, sign: 0.1}     # rad^2/s^3
      pi_person:   [0.90, 0.08, 0.02,  0.05, 0.85, 0.10,  0.05, 0.15, 0.80]
      pi_forklift: [0.95, 0.05, 0.00,  0.02, 0.90, 0.08,  0.02, 0.08, 0.90]
      pi_amr:      [0.95, 0.05, 0.00,  0.02, 0.93, 0.05,  0.02, 0.08, 0.90]
      pi_box:      [0.99, 0.01, 0.00,  0.30, 0.70, 0.00,  0.30, 0.20, 0.50]
      pi_sign:     [0.99, 0.01, 0.00,  0.30, 0.70, 0.00,  0.30, 0.20, 0.50]
    classes:     [box, sign, person, forklift, amr]
    class_prior: [0.30, 0.05, 0.30, 0.15, 0.20]      # pi_0, LiDAR-visible clusters (re-estimate)
    class_fusion: {beta: 0.5, cap: 0.95, eta: 0.02, score_bins: [0.35, 0.6, 1.0]}
    datmo:     {enabled: false, p_move: [0.02, 0.01, 0.60, 0.50, 0.60], rho0: 0.5,
                w_map: 4.0, w_free: 4.0, w_vel: 4.0, w_static: 2.0, set: 0.5, clear: 0.35}
    ttc:       {horizon: 5.0, step: 0.1, k_sigma: 1.0, sigma_cap: 0.5, robot_radius: 0.361,
                radius: {person: 0.30, amr: 0.361, forklift: 0.8}}
    peers:     [amr_01, amr_02, amr_03, amr_04, amr_05]   # self excluded at runtime
/**/object_localizer_node:
  ros__parameters:
    sync_slop: 0.017
    roi_frac: 0.5
    min_valid_frac: 0.3
    depth_noise: {apply_quadratic: true, iid_pixels: true}   # k from config/sensors.yaml
    kappa: 0.05
    camera_offset:                                    # mu, sigma_delta [m] along the ray
      box_small: [0.136, 0.026]
      box_medium: [0.250, 0.034]
      box_large: [0.306, 0.039]
      amr: [0.271, 0.052]
      person: [0.12, 0.04]
      forklift: [0.5, 0.15]
      sign: [0.02, 0.02]
/yolo_node:
  ros__parameters:
    weights: models/yolov8n_warehouse.engine          # PyTorch .pt until TensorRT policy decided
    imgsz: 640
    quantize: 16
    robots: [amr_01, amr_02, amr_03, amr_04, amr_05]
    batch_timeout_ms: 10
    conf_thresh: 0.35
    iou: 0.5
    backend: auto
    cpu_imgsz: 320
    cpu_threads: 2                                    # 125 core-ms/frame @320 -> 10 FPS needs >= 1.25 cores
    cpu_fleet_rate_hz: 5.0                            # 5 robots x 5 Hz = 3.1 cores (fits 80 % budget); 10 Hz would not
```

### 7.7 연산 예산(5대 동시, multi_robot.md §7의 "로봇당 YOLO 전/후처리 0.6 코어 @ ≤ 10 FPS" 항목과의 정합)

- GPU: YOLOv8n 배치 5 @ 30 Hz — 배치당 예산 33.3 ms, 혼잡 실측 forward 중앙값 13.6 ms, VRAM ≈ 70 MB(모델) + 작업 공간.
- CPU 폴백(측정, §3.2): 125 core‑ms/프레임 @320 → 단일 스트림 10 FPS 1.25 코어, 5대 × 5 Hz 3.1 코어(합계 23.6 코어, 74 %), 5대 × 10 Hz는 26.8 코어로 초과(`rev5_cpu_budget.out`).
- CPU 목표(측정 전): 추적기 5 × < 2 ms/스캔 @ 10 Hz ≈ 0.1 코어, object_localizer 5 × ≈ 2 ms @ 15 Hz ≈ 0.15 코어, yolo_node(단일 프로세스, GPU 레터박스) 150 img/s × ≈ 5 ms ≈ 0.75 코어, 마커·기타 ≈ 0.2 코어 → **≈ 1.2 코어**, 목표 상한 2.5 코어.
- **정합 이슈**: multi_robot.md §7은 YOLO를 로봇당 ≤ 10 FPS로 가정해 0.6 코어 × 5 = 3.0 코어를 배정했다. 30 FPS를 로봇별 프로세스로 돌리면 같은 단가로 9 코어가 되어 합계 ≈ 29.5 코어로 80 %(25.6 코어)를 넘는다. 따라서 (i) 단일 배치 프로세스 + GPU 전처리로 30 FPS를 달성하고, (ii) W3 실측이 2.5 코어를 넘으면 **운용률 15 Hz**(3D 출력률과 동일)로 낮추되 스펙 4.6의 30 FPS는 처리량 벤치마크로 별도 입증한다. 이 결정은 multi_robot.md §7 표에 반영을 요청한다.

### 7.8 일정(주 단위, 담당, 기한)

| 주 | 기간 | 내용 | 담당 |
|---|---|---|---|
| W1 | 2026‑09‑28 ~ 10‑02 | **차단 요소 1–2일차**: 액터가 gpu_lidar·깊이에 보이는지(평면 벽 앞 액터 스캔), `scan.time_increment`, compose `--gpus all`, Pose_V 평가 브리지 → 결과를 시뮬/인프라 팀과 10‑01까지 확정. 이후 `amr_msgs` 추가 필드 PR, COCO 가중치(person)로 `yolo_node`·`object_localizer_node`·마커 배관 완성(4.6 배관 증명), 합성 라벨러 착수 | 인지(시뮬·인프라 협조) |
| W2 | 10‑05 ~ 10‑09 | 추적 B1: 배경 라벨·ABD·클러스터 모델·CV‑KF·확장 GNN·생명주기·LS 검정·TTC·health, gtest | 인지 |
| W3 | 10‑12 ~ 10‑16 | 5클래스 미세조정, TensorRT 정책 결정·엔진, 유휴 호스트 GPU/CPU 재측정 게이트, P1(IMM·클래스 융합), P3(DATMO), 피어 주입 | 인지 |
| W4 | 10‑19 ~ 10‑23 | 평가 하네스·GT 로깅·시나리오 스윕, κ·σ_δ·σ_seg·로지스틱 가중치 보정 | 인지 |
| W5 | 10‑26 ~ 10‑30 | 절제·B0/B1/B2 비교, 문서화, 동료평가 리허설 | 인지 |

## 8. 오픈 질문

1. Gazebo 액터가 렌더링 기반 gpu_lidar에 보이는지(visual 기준이면 보여야 함)와 충돌 계수용 collision 유무 — W1 1–2일차 시험으로 확정(시뮬 팀, 10‑01).
2. TensorRT 파이썬 런타임 설치 정책(오프라인 재현 빌드) — W3 게이트 전 결정(인프라).
3. `safety_node`가 `tracked_obstacles` 나이 > 0.3 s를 정지 사유로 채택할지(safety 팀).
4. multi_robot.md §7 YOLO CPU 배정 갱신(§7.7), components.md §3.4/§5.4의 추가 구독·`perception/health`·메시지 필드·`object_localizer_node` 노이즈 가산 반영, sequences.md §2의 `is_dynamic (|v| > 0.2 m/s)` 자리표시 규칙을 §3.6/§5.3 정의로 교체(아키텍처 문서 소유자).
5. JPDA를 게이트 중첩 트랙에 국소 적용할지 — 사람 군집 시나리오가 30회 시험에 포함될 때만.

## 9. 참고문헌

VERIFIED(WebFetch; [3][4][8][11][14][16][17][22][23][24][26][27][D1]은 2026‑09‑22 재확인, 나머지는 2026‑09‑21 페치 + 리뷰어 2026‑09‑22 재확인): [1] https://arxiv.org/abs/2307.16675 · [2] https://arxiv.org/abs/2403.13443 · [3] https://arxiv.org/abs/2502.09672 (html 본문: 5개 클래스, {CV, CA, CTRV, CTRA}) · [4] https://arxiv.org/abs/2409.16149 + https://github.com/megvii-research/MCTrack (IROS 2025) · [5] https://arxiv.org/abs/2312.08952 · [6] https://arxiv.org/abs/2402.12303 · [7] https://arxiv.org/abs/2408.17098 · [8] https://arxiv.org/abs/2508.00358 · [9] https://arxiv.org/abs/2503.12968 · [10] https://arxiv.org/abs/2506.18124 · [11] https://arxiv.org/abs/2412.01041 · [12] https://arxiv.org/abs/2502.20607 · [13] https://arxiv.org/abs/2504.13647 · [14] https://arxiv.org/abs/2412.15000 (html 본문: DR‑SPAAM + Norfair) · [15] https://arxiv.org/abs/2409.09899 · [16] https://arxiv.org/abs/2603.15826 · [17] https://arxiv.org/abs/2304.10049 · [18] https://arxiv.org/abs/2410.05152 · [19] https://arxiv.org/abs/2401.17023 · [20] https://arxiv.org/abs/2404.12794 · [21] https://arxiv.org/abs/2506.07539 · [22] https://arxiv.org/abs/2503.22965 · [23] https://arxiv.org/abs/2501.07421 · [24] https://arxiv.org/abs/2412.15040 · [25] https://arxiv.org/abs/2510.19981 · [26] https://arxiv.org/abs/1912.00603 · [27] https://arxiv.org/abs/2609.13307 · [D1] Ultralytics Export 문서 https://docs.ultralytics.com/modes/export/ (`quantize`, `half`/`int8` deprecated 별칭, `batch`) · [D2] Nav2 obstacle layer 파라미터 이름은 설치된 `nav2_costmap_2d 1.1.20` 라이브러리 문자열로 확인.

RECALLED(미페치): [C1] H. Kuhn, "The Hungarian method for the assignment problem," Naval Res. Logist. Q. 1955; J. Munkres 1957 · [C2] Y. Bar‑Shalom, E. Tse, Automatica 1975; T. Fortmann, Y. Bar‑Shalom, M. Scheffe, IEEE J. Oceanic Eng. 1983 · [C3] H. Blom, Y. Bar‑Shalom, IEEE TAC 1988 · [C4] G. Borges, M. Aldon, J. Intell. Robot. Syst. 2004 · [C5] A. Bewley et al., ICIP 2016 · [C6] C. Nguyen, S. Izadi, D. Lovell, 3DIMPVT 2012 · [C7] M. Ahn et al., ICCAS 2019 · [C8] D. Jia, A. Hermans, B. Leibe, IROS 2020 · [C9] M. Przybyła, RoMoCo 2017 · [C10] R. Hartley, A. Zisserman, 2003 · [C11] S. Thrun, W. Burgard, D. Fox, 2005 · [C12] B. Mersch et al., RA‑L 2022 · [C13] S. Challa, G. Pulford, "Joint target tracking and classification using radar and ESM sensors," IEEE TAES 37(3):1039–1055, 2001 · [C14] B. Ristic, N. Gordon, A. Bessell, "On target classification using kinematic data," Information Fusion 5:15–21, 2004 · [C15] Y. Bar‑Shalom, T. Kirubarajan, C. Gokberk, "Tracking with classification‑aided multiframe data association," IEEE TAES 41(3):868–878, 2005 · [C16] R. O. Chavez‑Garcia, O. Aycard, "Multiple sensor fusion and classification for moving object detection and tracking," IEEE T‑ITS 17(2):525–534, 2016 · [C17] C.‑C. Wang, C. Thorpe, S. Thrun, M. Hebert, H. Durrant‑Whyte, "Simultaneous localization, mapping and moving object tracking," IJRR 26(9):889–916, 2007 · [C18] T.‑D. Vu, J. Burlet, O. Aycard, "Grid‑based localization and local mapping with moving object detection and tracking," Information Fusion 12(1):58–69, 2011 · [C19] Y. Bar‑Shalom, X.‑R. Li, T. Kirubarajan, Estimation with Applications to Tracking and Navigation, Wiley 2001 · [C20] S. Blackman, R. Popoli, Design and Analysis of Modern Tracking Systems, Artech House 1999 · [C21] K. Bernardin, R. Stiefelhagen, "Evaluating multiple object tracking performance: the CLEAR MOT metrics," EURASIP J. Image Video Process. 2008.

## 10. 리뷰 반영 이력 (rev1 → rev2 → rev2.1, 2026‑09‑22)

판정: **반영** = 수정 완료, **반대(근거)** = 리뷰의 수정안과 다른 결론을 증거와 함께 채택(문제 자체는 해소), **부분** = 설계 답은 있으나 실측 게이트가 남음. 검증 스크립트: `checks/rev4_geom_meas.py`, `checks/rev4_dyn.py`, `checks/rev4_extra.py`, `checks/rev4_stopgo.py`, `checks/rev5_fixes.py`, `checks/rev5_passing_latency.py`, `checks/rev5_cpu_budget.py`, `checks/bench/`(이전 시도의 `rev3_*.py`, `rev_checks*.py`는 참고용으로 보존).

### 10.1 수학 오류 (math_errors)

| ID | 리뷰 지적 | 처리 | 위치 |
|---|---|---|---|
| M1 | GPU 예산 "배치당 6.7 ms"는 이미지당 값 | 반영: 이미지당 6.67 ms / 배치당 33.3 ms, 배치 대기(≤ 10 ms 타임아웃)를 E2E에 포함, 혼잡 호스트 실측 병기 | §0, §3.2, §7.7 |
| M2 | $R_\text{cl}$ 축 배정 오류, 지속 편향의 백색화, 전폭 관측에도 $L_\perp^2/12$ | 반영 + **부분 반대**: 시선 $\sigma_r^2/n+\sigma_\delta^2$, 편향은 클래스별로 명시 보정 후 잔차만 노이즈. 가로는 리뷰의 $(r\Delta\phi)^2/(12n)$이 아니라 $(r\Delta\phi)^2/24$($n$ 무관) — 조각 균일 유도 + 평판 MC 일치(0.0089 vs 0.0089 m @5 m) | §3.4 |
| M3 | 레버암 피벗이 카메라 원점, 포즈 교차공분산 누락 | 반영: base 피벗 $[I_2\ \mathbf g]\Sigma_\text{pose}[I_2\ \mathbf g]^\top$, MC 일치 | §3.1 |
| M4 | 공통 모드 포즈 공분산을 측정마다 가산, map 추적이 AMCL 점프를 가짜 속도로 | 반영: odom 프레임 추적, 포즈 공분산은 출력에서 1회. 8 cm 점프 오탐 20.4 % → 0 | §3.0 |
| M5 | CV 프로세스 노이즈 3중 정의(q, σ_a, CTRV ω=0) | 반영: CWNA $q$ 단일 정의, B1 = IMM‑CV 동일 모델, YAML `q_cwna` | §3.5, §5.1, §7.6 |
| M6 | $v_\max(p)$ 평균은 경계가 아님 | **반대(근거)**: 리뷰의 상한 포락도 0.95 캡 아래에서 무효(τ ≤ 0.0125 → 3.0 m/s) 또는 유해(τ ≥ 0.02 → 0.5 m/s) → 클래스 속도 게이트 삭제, 클래스 무관 $v_\text{phys}$=3.0 m/s | §5.1, `rev4_dyn.out` J |
| M7 | 혼합 예측 공분산에 평균 간 퍼짐 항 누락 | 반영: $S_\text{assoc}=\sum\bar c_j[S_j+(\hat z_j-\bar z)(\cdot)^\top]$ (0.050 → 0.0725 예) | §5.1 |
| M8 | θ wrap 미처리, ST에서 v≈0 시 θ 비가관측 | 반영: 데카르트 속도 상태 [p, v_x, v_y, ω] + 미지 선회율 CT, 헤딩은 출력 전용·`heading_std`, 자코비안 수치 검증 | §5.1, §3.5 |
| M9 | $-\lambda\log c_i-\lambda\log s_j$는 행/열 상수라 무효 | 반영: 미연관·탄생 비용을 가진 확장 행렬 NLL, 신뢰도는 $P_D$·생명주기로만; 유한 BIG; 150/150 전수탐색 일치 | §3.5 |
| M10 | MPFS 부호·사전 결함(비지도화 정적물 0.894), unknown ≠ free 미구분 | 반영: 지도 항 ≤ 0, 기저율 일관 클래스 항, 관측 자유만; 진리표 재유도(정적 팔레트 0.381 → 1 s 후 0.077) | §5.3 |
| M11 | 속도 증거 이중 정의, 연속 검정 상관 무시 | 반영: LS 창 검정 단일 정의, 오탐률은 시뮬레이션(0–0.45 /트랙·h) | §3.6 |
| M12 | "확정 지연 ≤ 0.5 s" 근거 없음, 0.3 m/s 불가 | 반영: 경로별 유도 속도별 예산(0.3: ≤ 0.8 s, 1.0/1.5: ≤ 0.5 s), 0.3 m/s 시나리오 추가 | §3.6, §6 |
| M13 | 표면 깊이 vs 중심 GT 편향, $f_x$≈337 px | 반영: 클래스별 표면→중심 보정(박스 0.25, 사람 0.12 m), $f_x$=337.2(camera_info), 대역별 목표를 전파 모델에서 재도출 | §3.1 |
| M14 | 클래스 베이즈 갱신: s가 우도 아님, 과신, 캡 누락, Π_sign/Π_amr 미정의 | 반영: 점수 구간별 혼동행렬 우도, 주기당 1회 + β=0.5, 캡·망각 명시, 5클래스 Π 정의(행 합 검증) | §5.1 |
| M15 | 자차 속도 항 유도 없음·이중 계산 | 반영: 삭제(sim은 스캔 내 왜곡 없음), 실기 잔차는 TF 지터로 유도 후 NIS 검증 | §3.3, §5.2 |
| M16 | 카메라 OOSM | 반영: 카메라 = 클래스 전용 갱신, 연관은 스탬프 역예측 | §5.1, §5.4 |
| M17 | 중심 거리 < 0.3 m 병합이 선반 옆 보행자 흡수 | 반영: 최소 점간 간격 0.10 m + 동일 배경 라벨만 병합, 라벨 경계 분할(ABD만으로는 3 m에서 0.25 m까지 병합됨을 추가 지적), 크기 사전 재분할, 시나리오 추가 | §3.3, §6.1 |
| M18 | slop 0.03 s → 실효 15 Hz 미기재 | 반영: 깊이 트리거, slop 17 ms, 3D 15 Hz 명시 | §3.2, §7.2 |
| M19 | NEES "2 ± 0.3"에 N 없음 | 반영: N=300 → [1.78, 2.23], N=1000 → [1.878, 2.126] (χ²_{2N}/N) | §6.2 |

### 10.2 제안 판정 (proposal_verdicts)

| ID | 판정 | 처리 | 위치 |
|---|---|---|---|
| V1 | P1 already_published (JTC) | 반영: engineering_adaptation으로 재포지셔닝, JTC·Meng·Chavez‑Garcia·Bar‑Shalom 인용, IMM‑MOT 기술 정정, 결함 수정, B2(클래스 무관 IMM) 대비 증분만 주장 | §5.1 |
| V2 | P2 flawed | 반영: 독립 제안 제외, 올바른 부분만 베이스라인에 흡수, 무효 항 삭제 | §5.2 |
| V3 | P3 flawed (DATMO 원형 누락, 식 결함) | 반영: 2D DATMO의 engineering_adaptation, Dynablox는 3D 계승으로, 식·진리표·로지스틱 재적합 | §5.3 |

### 10.3 스펙 공백 (spec_gaps)

| ID | 공백 | 처리 | 위치 |
|---|---|---|---|
| S1 | 0.3 m/s 탐지 불가·미평가 | 반영: LS 검정 + 자유공간으로 0.3 m/s 중앙값 0.5–0.6 s, 속도별 평가 | §3.6, §6 |
| S2 | TTC/VO 계약·센서 고장 감시 없음 | 반영: TTC 알고리즘(추적 노드, components.md 역할), 계약(§7.3), `perception/health` | §3.7, §7.3, §7.4 |
| S3 | CPU 10 FPS 대안 미계획 | 반영(rev2.1): 자동 CPU 폴백 설계 + **혼잡 무관 실측** 125.2 core‑ms/프레임(imgsz 320) → 코어당 8.0 FPS, 단일 스트림 10 FPS는 2 스레드(효율 ≥ 0.63), 함대 저하 운용 5 Hz가 80 % 예산 충족(10 Hz는 초과)을 수치로 제시. 남은 것은 유휴 호스트 벽시계 확인(W3 게이트)뿐 | §3.2, §7.7 |
| S4 | costmap 주기·유지 시간 값 없음 | 반영: `observation_persistence 0.0`, `expected_update_rate 0.3/0.2`, `inf_is_valid`, 10 Hz, 데이터 경로 명시, dynamic_cloud 삭제 | §7.5 |
| S5 | 깊이 15 Hz vs 30 Hz 주장 | 반영: 3D·마커 15 Hz, 지연 목표 15 Hz 기준 | §3.2 |
| S6 | 지표 표준화(MOTA 매칭, 로그, 예측 정확도), 휴리스틱 신뢰도 보정 | 반영: CLEAR MOT 0.5 m, 로그 포맷, FDE 로그, 로지스틱 신뢰도 + ECE | §3.5, §6.2 |
| S7 | 카메라–LiDAR 외부 파라미터 문서 | 반영: `sensors.yaml`의 lidar_link(0.15, 0, 0.20)·camera_link(0.18, 0, 0.25)·optical_rpy 사용, 절차는 `sensor_calibration.md` §2.1–2.3 준수; 교차 검증(대형 박스 LiDAR 모서리 vs 깊이 에지 ≤ 2 cm)을 W4 평가에 추가 | §3.0–3.1, §6 |
| S8 | 다중 로봇 피어 융합이 임계 경로 | 반영: 무적재 AMR 비가시(0.33 < 0.38 m)를 수치로 보이고 피어 주입을 설계 결정으로 | §3.0, §3.8 |
| S9 | W1 과부하, 액터 가시성 소유자·기한 없음 | 반영: W1 1–2일차 차단 시험(담당·10‑01 기한), 미세조정·TRT를 W3로 이동 | §7.8, §8 |

### 10.4 필수 수정 (required_revisions)

| ID | 요구 | 처리 | 위치 |
|---|---|---|---|
| R1 | odom 프레임 추적, 포즈 공분산 측정별 제거, NEES 재실행 | 반영(NEES는 base 프레임 정의로 재설계, 실측은 W4) | §3.0, §6.2 |
| R2 | $R_\text{cl}$·편향 보정·대역별 목표·$f_x$ | 반영(가로 항은 M2의 근거로 $s^2/24$) | §3.1, §3.4 |
| R3 | MPFS 재작성 + 진리표 | 반영 | §5.3 |
| R4 | 속도별 지연 예산 + 0.3 m/s | 반영 | §3.6 |
| R5 | 속도 게이트 포락, 퍼짐 항, 헤딩 wrap, θ 비가관측, Π 5클래스, 캡·템퍼링 | 반영(속도 게이트는 M6 근거로 삭제) | §5.1 |
| R6 | 확장 할당 행렬 또는 신뢰도 제거, 유한 대수 | 반영(둘 다: 확장 NLL + 신뢰도 비용 제거) | §3.5 |
| R7 | 카메라 OOSM, 15 Hz·slop 문서화 | 반영 | §3.2, §5.1 |
| R8 | CV 노이즈 정합, 자차 항 삭제/유도 | 반영 | §3.5, §3.3 |
| R9 | 최소 간격 병합·라벨 경계, 0.2 m 시나리오 | 반영 | §3.3, §6.1 |
| R10 | GPU 예산 정정 + CPU/ONNX 폴백 실측 | 반영(rev2.1): 예산 정정, GPU 혼잡 실측(배치 5 forward 13.6 ms), CPU 폴백은 onnxruntime 미설치로 torch 융합 모델 경로를 택해 CPU 시간으로 실측(125.2 core‑ms/프레임; `predict()` 래퍼는 10배 느려 배제). ONNX/OpenVINO는 유휴 호스트에서 10 FPS 미달일 때의 대안으로 명시 | §3.2 |
| R11 | 신규성 정직화 + 인용 정정(IMM‑MOT, [24], [22], `half`) | 반영: JTC [C13][C14][C15], Meng [26], Chavez‑Garcia [C16], Wang [C17], Vu [C18], PR‑IMM [27] 추가; Gordon–Maskell–Kirubarajan 2002(SPIE)는 직접 확인하지 못해 인용하지 않음(동일 내용은 [C13][C14]가 대표) | §4, §5, §9 |
| R12 | 플래너 계약 + 건강 토픽 | 반영 | §7.3, §7.4 |
| R13 | MOTA 정의·로그, 신뢰도 보정 재정의, NEES N | 반영 | §6.2, §3.5 |
| R14 | costmap 값·데이터 경로 | 반영 | §7.5 |

### 10.5 인용 문제 (citation_problems)

| ID | 문제 | 처리 |
|---|---|---|
| C1 | IMM‑MOT를 "클래스 무관"으로 기술 | 반영: html 본문 재페치 — 5개 차량 클래스, {CV, CA, CTRV, CTRA} ([3]) |
| C2 | SG‑LKF를 수작업 β의 근거로 과독 | 반영: 학습형 MotionScaleNet 명시, β 항 삭제 ([8]) |
| C3 | [24]는 ToF·거리+입사각 모델, [23]은 경험적 비교 | 반영: 두 문헌의 기술 정정, 이차 항의 출처를 `sensors.yaml` 결정과 [C6]으로 ([23][24]) |
| C4 | Dynablox를 원형으로 오귀속, Wang/Vu 누락 | 반영: [17]을 3D 계승으로, [C17][C18] 추가 |
| C5 | [22] < 4.2 cm의 조건 누락 | 반영: 정면·5 m 이내 조건 명기 |
| C6 | `half`는 deprecated 별칭 | 반영: `quantize=16`, 문서 재페치 ([D1]) |
| C7 | Plozza 추적기(Norfair) 누락 | 반영: html 본문 재페치로 DR‑SPAAM + Norfair 명기 ([14]) |
| C8 | MCTrack IROS 2025 출처, BEV 매칭 주장 | 반영: 공식 GitHub README로 채택 확인, BEV 매칭 주장 삭제 ([4]) |
| C9 | P1/P3 핵심 선행 누락, PR‑IMM | 반영: [26][27] VERIFIED, [C13]–[C18] RECALLED(서지 확인) 추가 |

### 10.6 리뷰 외 정합 수정 (설정·계약 기준)

- 깊이 노이즈 모델을 `sensors.yaml`(네이티브 0.005 m + 이차 0.002 m⁻¹, 제곱합)로 교체, rev1의 `depth_noise_node`·"가우시안 경로 없음" 주장 폐기.
- 패키지·노드·토픽을 components.md 이름으로 통일: `amr_vision/amr_tracking/amr_perception_msgs` → `amr_perception`/`amr_msgs`; `yolo_server_node` → `yolo_node`, `projector_node` → `object_localizer_node`, `tracker_node` → `obstacle_tracker_node`; `perception/detections_3d` → `perception/detected_objects`, `perception/tracks` → `perception/tracked_obstacles`; 입력 `scan` → `scan_filtered`; 네임스페이스 `/amr_XX`.
- 스택 사실 갱신(wf-final: numpy 1.26.4 / scipy 1.15.3로 충돌 해소), LiDAR +inf 처리, `gpu_lidar` 스캔 동시성.
- 신규 발견: LiDAR 평면(0.38 m) 아래의 무적재 AMR·소/중형 박스 비가시성, 원거리 다리 점 수(최소 점 수 거리 의존화), 요구 확정 거리 표.

### 10.7 rev2.1 — 재개 후 자체 점검 정정 (2026‑09‑22, `checks/rev5_*.py` → `checks/rev5_*.out`, `checks/bench/cpu_*.txt`)

중단 후 재개하면서 rev2 본문 수치를 검증 로그와 설정에 다시 대조해 찾은 잔여 불일치다. 리뷰 항목이 아니므로 집계(§10.8)에는 넣지 않는다.

| ID | 발견 | 정정 | 위치 |
|---|---|---|---|
| A1 | §3.1 대역별 카메라 RMS가 ROI 중앙값 깊이 항을 빼고 계산(`rev4` E) → "대역 무관 4.2 cm"는 원거리 과소평가 | 중앙값 항 포함 재계산: 박스 4.2 / 4.2 / 4.8 cm(6–10 m 최댓값 5.7), 사람 4.6 / 4.6 / 4.7 cm; map 프레임 예측 7.2 / 8.9 / 12.6 cm를 명시해 목표 ≤ 0.09 / 0.11 / 0.15 m의 근거로 연결(목표값 불변) | §3.1 |
| A2 | §3.4 $\sigma_\perp^2=s^2/24+\sigma_\text{lat}^2$에 광선 추적의 **총** 가로 σ(양자화 포함)를 $\sigma_\text{lat}$로 넣어 양자화를 이중 계산 | $\sigma_\text{lat}$ = 형상 기여분 $\sqrt{\sigma_\text{tot}^2-s^2/24}$의 3/5/8 m 최대: 사람 0.014, 박스 0.015, amr 0.012, 지게차 0.051 m(YAML 동기화). 사람·박스·amr은 5 m 이내에서 하한 0.02 m가 지배해 실효 $\sigma_\perp$ 불변, 8–10 m에서 최대 +1.8 mm; 지게차는 3 m 형상 기여(0.051)를 택해 5 m에서 36 → 52 mm로 보수화 | §3.4, §7.6 |
| A3 | $\sigma_\text{skew}$ 식은 $/\sqrt3$인데 수치 8.4 px는 최대 변위 | 최대 8.4 px, σ 4.9 px로 구분 | §3.1 |
| A4 | $\lvert\omega_\text{LOS}\rvert$ 상한을 존 경계 값(0.50 rad/s)으로 적어 최악값 누락, 로봇 통과 중 지연 미평가 | `robot_params.yaml`로 최댓값 1.04 rad/s(D = 1.0 m) 도출. 통과 중 지연을 새로 모사(`rev5_passing_latency.py`): 사람 클래스는 불변(0.7 s), 미지 클래스 0.3 m/s는 1.1 s(ω 0.5)·3.4 s(ω 1.04)로 예산 밖 → 0.3 m/s 예산의 적용 조건(로봇 정지 또는 사람 클래스)과 P3·카메라 보완 경로를 본문에 명시 | §3.6 |
| A5 | §3.7 $d_\text{req}$(ℓ = 0.5 s)가 인용한 `rev4_dyn.out` L에 없음(ℓ = 0.3/0.8만 출력) | `rev5_fixes.out` R로 재계산, 0.3 m/s(ℓ = 0.8 s) 행 8.7 m 추가 | §3.7 |
| A6 | [4][26][27] VERIFIED 표기의 근거를 이 세션에서 재확인 | WebFetch 재페치: MCTrack README "2025‑06‑16. MCTrack is accepted to IROS 2025."·운동 지표; Meng et al. 시변 TPM 문장; PR‑IMM 제목·저자·2026‑09‑10·레이더/CV·CA·CT + 트랜스포머 예측기 → [27] 기술 구체화 | §4, §9 |
| A7 | S3·R10이 "부분"(CPU FPS 미측정)으로 남아 있었음 | 혼잡 호스트에서도 유효한 CPU 시간 지표로 실측(125.2 core‑ms/프레임 @320, 코어당 8.0 FPS) → 스레드 수·함대 저하 운용률(5 Hz)·80 % 예산 적합성을 수치로 확정. `predict()` 래퍼 이상(10배)을 발견해 CPU 경로에서 배제 | §3.2, §7.1, §7.6, §7.7 |

### 10.8 집계

| 범주 | 항목 수 | 반영 | 반대(근거, 문제는 해소) | 부분 |
|---|---|---|---|---|
| math_errors | 19 | 18 (M2는 가로 항만 근거 있는 부분 반대) | 1 (M6: 클래스 속도 게이트 삭제) | 0 |
| proposal_verdicts | 3 | 3 | 0 | 0 |
| spec_gaps | 9 | 9 (S3는 rev2.1에서 부분 → 반영) | 0 | 0 |
| required_revisions | 14 | 14 (R10은 rev2.1에서 부분 → 반영; R5의 속도 게이트는 M6 근거로 삭제) | 0 | 0 |
| citation_problems | 9 | 9 | 0 | 0 |
| **합계** | **54** | **53** | **1** | **0** |

54건 모두 해소되었다(53 반영, 1건은 리뷰 수정안 대신 근거와 함께 다른 해법 채택). 실측 확인이 남은 항목(유휴 호스트 벽시계 FPS, NEES·지연·오탐의 Gazebo 실측)은 설계 결함이 아니라 W3–W4 평가 게이트(§7.8)이며, 각 합격 기준은 §1·§6에 수치로 정의되어 있다.
