# 2D SLAM · 맵 품질 · AMCL · 납치 복구 (명세 4.3)

> 구현/설정: `src/amr_localization/config/{slam_toolbox,amcl,kidnap_monitor}.yaml`,
> `amr_localization/{kidnap_detector,scan_map_match,global_seed,kidnap_monitor_node,amcl_map_adapter,world_geometry,map_quality}.py`,
> 런치 `localization.launch.py mode:=slam|localization`. 설계 기준: research brief `state-estimation` §2.4~2.6, §4.C.
> 측정 조건은 §6 머리 (load average 6~25, 외부 작업 종료 후 — 잠정치 아님).

## 1. 구성과 모드

| 모드 | 노드 | map→odom 발행 |
| --- | --- | --- |
| `mode:=slam` (매핑) | 전처리 3 + `ekf_filter_node_odom` + `slam_toolbox` (online async) | slam_toolbox |
| `mode:=localization` (주행) | 전처리 3 + EKF 2 + 루트 `map_server` + `amcl` + `lifecycle_manager_*` + `kidnap_monitor_node` | `ekf_filter_node_map` (AMCL 은 `tf_broadcast: false`) |

두 모드는 map→odom 발행자가 겹치므로 동시에 켜지 않는다. 입력 스캔은 모두 `scan_filtered`
(거리·각도·아웃라이어·섀도우 필터, kinematics.md §6), odom 은 `ekf_filter_node_odom` 의 TF.
맵 저장: `ros2 run nav2_map_server map_saver_cli -f maps/warehouse` (해상도 0.05 m, trinary PGM + YAML).

## 2. slam_toolbox — 원리와 파라미터 근거

**스캔 매칭 (프런트엔드).** 새 스캔을 odom 예측 자세 주변 상관 탐색 창(±`correlation_search_space_dimension`/2,
분해능 0.01 m, 각도 coarse ±0.349 rad → fine 0.00349 rad)에서 스미어된 점유 격자와의 응답
(끝점이 점유 셀에 떨어지는 비율)을 최대화하는 자세로 정한다. odom 에서 멀어질수록
`distance/angle_variance_penalty` 로 응답에 벌점을 곱하고, 응답 곡면의 2차 모멘트로 매칭 공분산 Σ 를 낸다.
이동이 `minimum_travel_distance/heading` 을 넘을 때마다 포즈 그래프에 노드(스캔)와 간선(상대 자세, Σ)을 추가한다.

**루프 폐합 탐지·검증.** 현재 체인 밖 노드 중 `loop_search_maximum_distance` 이내이고 체인 길이가
`loop_match_minimum_chain_size` 이상인 후보에 대해 `loop_search_space_dimension`(8 m) 조대 매칭 →
`response_coarse > loop_match_minimum_response_coarse` ∧ `Σ_xx, Σ_yy < loop_match_maximum_variance_coarse` →
정밀 매칭 `response_fine ≥ loop_match_minimum_response_fine` 이면 루프 간선 추가 (아니면 기각).

**그래프 최적화 (백엔드, Ceres).** 노드 자세 (p_i, θ_i), 간선 측정 (p̂_ij, θ̂_ij, Ω_ij = Σ_ij⁻¹):

```
e_ij = [ R(θ_i)ᵀ (p_j − p_i) − p̂_ij ;  wrap(θ_j − θ_i − θ̂_ij) ]
min Σ ρ_H( e_ijᵀ Ω_ij e_ij ),   ρ_H = Huber (a = 0.7) — 잘못 수락된 루프 간선의 영향을 선형 벌점으로 제한
```

Levenberg–Marquardt + 희소 정규방정식(`SPARSE_NORMAL_CHOLESKY`)으로 푼다. 루프가 닫히면 체인 전체가
재배치되어 누적 드리프트가 루프 둘레에 분산된다.

| 파라미터 | 기본 | 설정 | 근거 |
| --- | --- | --- | --- |
| `resolution` | 0.05 | **0.05** | 명세 ≤ 0.05 m |
| `max_laser_range` | 20 | 25 | LiDAR 사양, 60 m 창고의 긴 통로 끝 벽까지 래스터화 |
| `minimum_travel_distance / heading` | 0.5 / 0.5 | 0.3 / 0.3 | 노드 밀도 ↑ → 좁은 랙 통로에서 국소 정합·루프 후보 ↑ |
| `correlation_search_space_dimension` | 0.5 | 0.5 | ±0.25 m 창: 0.3 m 이동 간 odom(EKF) 오차 ≪ 0.25 m |
| `correlation_search_space_smear_deviation` | 0.1 | 0.05 | ≈ σ_lidar(0.03) + res. 크면 벽이 두꺼워짐 |
| `loop_search_maximum_distance` | 3.0 | 5.0 | 100 m 루프의 0.05~1 % 드리프트(≤ 1 m)를 넉넉히 포함 |
| `loop_match_minimum_chain_size` | 10 | 15 | 반복 랙 통로(겉보기 같은 곳) aliasing 오수락 억제 |
| `loop_match_minimum_response_fine` | 0.45 | 0.55 | 같은 이유 |
| `ceres_loss_function` | None | HuberLoss | 오수락 루프의 영향 제한 |
| `map_update_interval` | 5 | 2 | 매핑 중 커버리지 확인 편의 |

명세 힌트("지도가 찌그러지면 chain_size, correlation_search_space 조정") 해석: 찌그러짐 = 오수락 루프
(체인·응답 임계 ↑) 또는 국소 매칭 실패(탐색 창·스미어 ↑).

## 3. 맵 품질 지표 (명세 "맵 일관성, 구조물 정확도")

지면 진실 M_gt: 월드 SDF 의 정적 모델 visual 을 **스캔 평면 높이 0.38 m** 에서 자른 단면을 0.05 m 로
래스터화 (`world_to_map`; gpu_lidar 는 visual 을 측정하므로 visual 을 쓴다; 지게차·작업자는 제외).
SLAM 맵 M_est 는 map 프레임 = 매핑 시작 자세이므로 스폰 자세로 초기 정렬 후 점유 셀을 GT 거리장에
맞추는 SE(2) 최소제곱(soft-L1)으로 잔여 정렬을 구한다 (정렬량도 보고). `map_quality` CLI 가 계산한다.

| 분류 | 지표 | 정의 |
| --- | --- | --- |
| 구조물 정확도 | **ADNN** | M_est 점유 셀 → 최근접 M_gt 점유 셀 평균 거리 (+ 중앙값, p95) |
| | Chamfer | ½(ADNN + 관측 영역 안 GT 셀 → 최근접 M_est 셀 평균) |
| | IoU_τ / precision / recall | τ = 1 셀 허용 팽창 |
| | **기지 거리 오차** | 마주 보는 외벽 사이 간격 (GT 59.6 m / 39.6 m) — 벽마다 LiDAR 가 본 표면 셀로 직선 적합 |
| | 랜드마크 중심 오차 | 기둥·랙(면적 ≤ 4 m²) 의 관측된 부분 중심 vs 근처 M_est 점 중심 |
| 일관성 | **외벽 직선성** | 벽 표면 셀 직선 적합 RMS 잔차, 각도 오차 (루프 오정합이면 휘거나 꺾인다) |
| | 벽 두께 | 벽 단위 길이당 점유 셀 수 (이중 벽·번짐 검출) |
| | 점유/자유/미지 비율 | 커버리지 |

## 4. AMCL — 모델과 튜닝 근거 (`config/amcl.yaml`)

- **운동 모델** (DifferentialMotionModel): `δ̂ = δ − N(0, α₁δ_rot² + α₂δ_trans²)` 등. α 는 **분산 계수**.
  입력 odom 이 EKF(휠+자이로) 출력이라 raw odom 전제의 기본 0.2 (σ 45 %) 는 과대 → 갱신마다 구름이 σ_hit 보다
  넓게 퍼져 가중치가 붕괴한다. 물리값(병진 σ ≈ 0.7 %)에 파티클 다양성 팽창 κ ≈ 6 → α ≈ 2e-3~5e-3.
- **관측 모델** (likelihood field): `p = z_hit exp(−d²/2σ_hit²) + z_rand/z_max`, 가중치 `1 + Σ p³`.
  `σ_hit ≈ sqrt(σ_lidar² + res²/12 + σ_map²) ≈ 0.05` 에 맵 오차 여유를 두어 0.08. z_hit 0.8 / z_rand 0.2
  (동적 장애물 흡수), `max_beams` 180 (60 은 좁은 통로 벽 정보 부족), `laser_likelihood_max_dist` 2.0.
- **파티클**: min 500 / max 8000, KLD (`pf_err` 0.05, `pf_z` 0.99). nav2 humble 리샘플러는 O(N²) 선형 탐색 →
  8000 개가 상한(≈ 30~60 ms/갱신). 추적 중 KLD 가 수백 개로 줄인다.
- **리샘플링**: `resample_interval` 1, Augmented MCL (`recovery_alpha_slow` 0.001, `fast` 0.1) — 스캔 불일치 시
  무작위 파티클 주입.
- **갱신 조건**: `update_min_d` 0.10 m, `update_min_a` 0.08 rad (1 rad/s × 10 Hz 회전에서 스캔당 0.1 rad 가
  엄격 부등호 0.1 에 걸려 갱신이 누락되지 않도록).
- **정지 직후 무이동 갱신** (`kidnap_monitor_node` 의 `StopUpdateScheduler`): AMCL 은 움직일 때만 갱신하므로
  정지하면 마지막 주행 중 갱신의 오차(2~6 cm, 이전 실행 실측)가 그대로 굳어 정지 3 cm 를 넘는다. 움직이다
  `|v| < 0.01 m/s`, `|ω| < 0.01 rad/s` 가 0.5 s 지속되면 `request_nomotion_update` 를 1 s 간격 3 회 호출해
  같은 자세의 스캔으로 파티클을 다시 모은다. 무이동 갱신은 운동 잡음 없이 리샘플하므로 반복하면 파티클이
  고갈된다 → 정지당 3 회로 제한 (brief §2.4, `nomotion_updates_on_stop`).

### 4.1 nav2_amcl 반 셀 좌표 편향과 보정 (`amcl_map_adapter`)

Gazebo 측정에서 AMCL 오차가 경로·지도와 무관하게 **(−x, −y) 쪽으로 쏠리는** 것이 보였다 (보정 전 GT 맵 정지 오차
(−0.5, −1.3), (−1.7, −0.8), (−0.6, −1.7), (−1.1, −1.1) cm …; §6.3). 원인은 nav2_amcl (humble 1.1.20,
`map/map.hpp`) 의 좌표 변환이다:

```
convertMap:  o' = o + (N/2)·s                       (o = OccupancyGrid origin, N = 폭 [셀], s = 해상도)
MAP_GXWX(x) = floor((x − o')/s + ½) + N/2 = floor((x − o)/s + ½)      ← 반올림
MAP_WXGX(i) = o' + (i − N/2)·s               = o + i·s                ← 셀 모서리
```

OccupancyGrid 규약에서 셀 i 는 [o + i·s, o + (i+1)·s) 인데 AMCL 은 [o + (i−½)s, o + (i+½)s) 로 본다. 즉 AMCL 내부
지도(우도장 포함)는 모든 구조물이 (−s/2, −s/2) 옮겨진 것과 같고, 스캔을 그 지도에 맞춘 추정 자세도 같은 만큼
치우친다 — s = 0.05 m 에서 축마다 2.5 cm, 대각 3.5 cm 로 **정지 3 cm 목표와 같은 크기**다.

보정: `amcl_map_adapter` (Python, 로봇 네임스페이스) 가 `/map` 을 받아 원점만 (+s/2, +s/2) 옮긴 사본을
`map_amcl` (transient_local) 로 재발행하고 AMCL 은 그것을 구독한다 (`amcl_half_cell_fix:=true` 기본). 그러면
`floor((x − o − s/2)/s + ½) = floor((x − o)/s)` 이고 `MAP_WXGX(i)` 는 셀 중심을 주어 두 변환이 모두 규약과 일치한다.
AMCL 은 원점 yaw 를 쓰지 않으므로 이동량도 회전하지 않는다(원점 yaw ≠ 0 이면 경고). 다른 `/map` 구독자(costmap,
추적기, kidnap_monitor, 교통 관리)는 원본을 그대로 쓴다. 검증: `test_amcl_map_adapter_shifts_origin_half_cell`
(AMCL 반올림 셀 = 규약 셀), Gazebo 전후 비교는 §6.3.

### 4.2 튜닝 과정 (Gazebo 스윕, 같은 스택에서 `ros2 param set` 으로 설정만 바꿔 같은 경로 반복)

경로: 정지 30 s → 직진 14 m (0.5 m/s) → 제자리 2π → 원호 (r 1 m) → 제자리 −π → 직진 14 m (0.8 m/s) → 정지 20 s.
구간 RMSE [cm] (정지 = 경로 전체 정지 구간, "주행 후" = 첫 30 s 초기화 직후 정지를 뺀 값). 보정(§4.1) 전 측정.

| 설정 | SLAM 맵 정지 / 주행 후 / 직선 / 회전 | GT 맵 정지 / 주행 후 / 직선 / 회전 | 판단 |
| --- | --- | --- | --- |
| 기준 (σ_hit 0.08, d/a 0.10/0.08) | 4.63 / 6.38 / 4.17 / 4.01 | 4.08 / 5.65 / 3.71 / 2.10 | 채택 (아래 둘보다 좋음) |
| σ_hit 0.05 | 4.69 / 6.46 / 4.93 / 3.20 | 5.83 / 8.08 / 8.00 / 3.82 | 날카로운 우도 → 파티클 고갈, 직선 악화 |
| σ_hit 0.05 + update_min_d/a 0.05 | 4.28 / 5.93 / 4.74 / 3.37 | 5.85 / 8.11 / 7.45 / 2.49 | 갱신 잦음 → 리샘플 과다 |

세 설정 모두 정지 구간이 기준(3 cm)을 넘었고 오차가 일관되게 (−x, −y) 로 쏠려 §4.1 의 체계 편향을 찾았다.
σ_hit 를 줄이는 방향은 효과가 없거나 나빠 기준값(0.08)을 유지했다. 보정 후 결과는 §6.3.

## 5. 납치(Kidnapped Robot) 감지와 복구

### 5.1 감지 (`kidnap_detector.py`, `scan_map_match.py`)

| 신호 | 식 | 역할 |
| --- | --- | --- |
| 공분산 급증 | AMCL `var_x + var_y > 0.5 m²` 또는 `var_yaw > 0.5 rad²` | 발산 (지속 경보) |
| 점프 | 연속 AMCL 갱신 사이 `‖Δp_amcl − Δp_odom‖ > 1.0 m` 또는 헤딩 0.5 rad | 오수렴·재수렴 (순간 사건 → SUSPECT 계기) |
| **스캔-맵 불일치** | 인라이어 비율 `ρ = #{d(끝점) ≤ 0.2 m}/#유효 빔 < 0.5` 가 5 스캔 연속 | **주 신호** (지속 경보) |

스캔-맵 일치도는 맵의 유클리드 거리 변환(AMCL likelihood field 와 같은 구조)을 맵당 한 번 만들고, 스캔 시각의
map 자세 = 마지막 AMCL 자세 ∘ (그 뒤 odom 상대 이동) 로 끝점을 옮겨 조회한다 (O(빔 수)).
**정지 중 납치**는 odom 이 움직임을 못 보므로 AMCL 이 갱신조차 하지 않아 공분산·점프로는 잡히지 않는다 —
그래서 일치도가 주 신호다. 상태: TRACKING → (경보) SUSPECT → `suspect_time` 1 s 지속 → LOST.

### 5.2 복구 (`global_seed.py`, `kidnap_monitor_node.py`)

LOST 선언 시 `localization/lost = true` (latched, BT `IsLocalized` 가 구독) 후 시도 k = 1, 2, … :

1. **전역 가설 탐색 (시드)** — 첫 시도에 현재 스캔 1 개로 자유 공간 전체(점유까지 ≥ 0.3 m 인 알려진 자유 셀)를
   0.5 m × 5° 격자로 훑어 캡 평균 거리 `m(h) = mean(min(d, 1 m))` (90 빔, 절사 없음)로 점수화 → 헤딩마다 상위
   50 개 → NMS(1 m / 20°) → 각 후보 주변 ±0.5 m / ±5° 를 0.1 m × 1° 로 정밀화(180 빔, 하위 80 % 절사 평균 —
   동적 장애물 가림 빔 제거). **최종 순위는 절사하지 않은 인라이어 비율** `ρ(h)`(끝점이 점유 셀 0.2 m 이내인 빔
   비율)로 매긴다: 절사 평균은 가림에 강하지만 별칭(창고의 점대칭 배치)을 가르는 소수 빔까지 버려, 이전 Gazebo
   실행에서 (15, −15, π) 납치가 점대칭 별칭 (−15, 15, 0) 으로 오수렴했다(절사 점수 0.0041 vs 0.0056 로 거의 같음;
   ρ 는 1.000 vs 0.931 로 갈린다).
   시도 k ≤ `seed_attempts`(3) 는 k 번째 가설을 AMCL `initialpose` (σ 0.25 m, 5.7°) 로 넣는다.
   (제자리 회전으로 헤딩이 바뀌었으면 odom 헤딩 변화만큼 가설 헤딩을 옮긴다.)

   **구현 (`global_seed.GridScorer`)**: 가설 위치를 셀 중심에 두면 격자 좌표 `g = (c + ½, r + ½)` 에서 빔 끝점 셀은
   `(c, r) + floor(½ + R(ψ − ψ₀) b / res)` 로, 오프셋이 위치와 무관한 정수다. 헤딩마다 오프셋을 한 번 계산하고
   `flat[base_i + off_j]` 한 번의 gather 로 전체 가설 × 빔 점수를 낸다(테두리를 최대 빔 길이만큼 두른 거리장
   사본을 써서 맵 밖 끝점도 분기 없이 `max_distance`). 60 × 40 m 맵에서 자유 셀 격자 8 994 개 × 72 헤딩 × 90 빔
   = 5.8×10⁷ 조회 + 정밀화 1.2×10⁷ 조회가 **0.2 s** (부동소수 좌표 조회 구현 8.2 s → 40 배, 결과 동일;
   `test_grid_scorer_matches_float_scorer` 가 두 구현을 대조). 단일 스레드 rclpy 노드의 실행기를 막는 시간이
   짧아져 LOST 선언 직후 곧바로 회전을 시작한다.
2. 시드가 소진되면 AMCL `reinitialize_global_localization` (자유 공간 균일 분포; Augmented MCL 이 보조).
3. 각 시도마다 **제자리 회전 2π** (Nav2 `spin` 액션; 단독 시험은 `fallback_cmd_vel_topic`) — 360° LiDAR 라도
   회전은 `update_min_a` 를 넘겨 AMCL 갱신·리샘플을 구동해 파티클을 수렴시키고, 수렴 여부를 여러 방향에서 확인한다.
4. **수렴 판정**: AMCL `var_x + var_y < 0.1 m²` 이고 `ρ ≥ 0.7` 이 5 회 연속 → `ekf_filter_node_map/set_pose` 로
   map EKF 를 AMCL 평균으로 즉시 옮기고 `localization/lost = false`, 3 s 재감지 유예.
5. 회전이 끝나도 수렴하지 않으면 다음 시도, 모두 실패하거나 120 s 초과면 FAILED (lost 유지, 수렴하면 언제든 복귀).

AMCL 균일 재초기화만으로는 60 × 40 m 창고에서 σ_hit 분지에 파티클이 떨어질 확률이 낮다 (research brief
§2.4: N = 5000, σ_hit 0.05 → 기대 적중 0.003 개). 시드 단계가 이를 결정적 탐색으로 바꾼다.
한계: 주기적인 랙 통로는 스캔 하나로 구분되지 않는 가설(별칭)이 여러 개일 수 있다 — 상위 3 개를 차례로
시도하고 일치도로 검증하지만, 제자리 회전은 별칭을 가르지 못한다(360° LiDAR). 이동으로 가르는 DISAMBIGUATE
단계(brief §4.C)는 확장점으로 남긴다 (`KidnapDetector` 의 시도 루프에 행동을 추가).

## 6. 측정 결과

측정 조건: 2026-09-22 10:50~12:40 KST, 호스트 32 스레드, 세션 시작·끝 `uptime` load average 6~25 (외부
`gsim` 작업 종료 후 — **잠정치 아님**; 동시에 도는 다른 에이전트 시뮬레이션 포함), Gazebo Fortress 6 headless +
GPU 렌더링, 창 RTF 평균 0.88~0.99. 월드 `warehouse.sdf` (60 × 40 m, 동적 작업자·지게차 포함), 로봇 스폰
(−16, −2, 0). 카메라 갱신율만 1 Hz 로 낮춘 설정 사본을 썼다 (위치 추정은 카메라 미사용). GT 는
`ground_truth/odom` (OdometryPublisher, 월드 자세) 을 map 프레임으로 옮겨 쓰고, 오차는 feature/evaluation-tools
의 `pose_error_logger` → `analyze` 판정(RMSE, 구간은 GT 속도로 정지/직선/회전 분류)으로 낸다.

### 6.1 매핑 (slam_toolbox, `mode:=slam`) → `maps/warehouse.{pgm,yaml}`

- 경로: 통로 전부를 도는 GT 되먹임 주행 ≈ 324 m (1 m/s, 재방문 루프 포함), 시뮬레이션 480 s, 창 RTF 평균 0.88
  (최소 0.49), load 23.7 → 10.6. 루프 폐합 시 전역 최적화(Ceres) 실행 **22 회** (로그 기준).
- 결과: 1203 × 802 셀 **@ 0.05 m** (60.15 × 40.10 m), origin (−14.1, −18.1) — map 프레임 = 매핑 시작 자세.
  `map_saver_cli` trinary. 저장소 `maps/warehouse.pgm`, `maps/warehouse.yaml` (루트 map_server 기본 맵).

맵 품질 (`ros2 run amr_localization map_quality`, §3 지표, GT = 월드 visual 단면 0.38 m):

| 분류 | 지표 | 값 |
| --- | --- | --- |
| 정렬 | map→world (스폰 초기값 후 SE(2) soft-L1) | (−16.060, −2.040) m, 0.056°; 정렬 후 잔차 RMS 8.3 cm |
| 구조물 정확도 | **ADNN** 평균 / 중앙 / p95 | **2.83 / 1.42 / 8.95 cm** |
| | Chamfer (관측 영역) | 6.65 cm |
| | IoU_τ / precision / recall (τ = 1 셀) | 0.392 / 0.751 / 0.498 |
| | **기지 거리 오차** (외벽 간격, GT 59.60 / 39.60 m) | 59.504 m (**−9.6 cm, −0.16 %**) / 39.456 m (**−14.4 cm, −0.36 %**) |
| | 랜드마크(기둥·랙 36 개) 중심 오차 평균 / 최대 | 5.9 / 10.3 cm |
| 일관성 | **외벽 직선성** RMS (서/동/남/북) | 2.10 / 2.24 / 1.67 / 1.54 cm |
| | 외벽 각도 오차 | 0.064 / 0.023 / 0.042 / 0.019° |
| | 벽 두께 | 2.8~3.2 셀 (이중 벽·번짐 없음) |
| | 점유 / 자유 / 미지 | 0.021 / 0.979 / 0.000 (창고 전체 관측) |

해석: 외벽이 곧고(RMS ≤ 2.2 cm) 각도 오차가 0.07° 이하라 루프 폐합 후 전역 일관성은 좋다(찌그러짐 없음).
네 외벽 면이 모두 안쪽으로 5~8 cm 들어와 맵이 0.16~0.36 % 작게 그려졌다 — 아래 6.2 에서 LiDAR 거리·시각
편향이 없음을 확인했으므로 스캔 매칭 단계의 축척 편향(odom 예측 쪽 벌점 `distance_variance_penalty`)으로
본다. ADNN 중앙값 1.4 cm 는 맵 해상도 수준이다.

### 6.2 센서 검증 (위치 추정 오차 원인 분리용)

- **LiDAR 거리 편향**: 정지 로봇 6 자세 × 30 스캔 평균을 GT 자세에서 1 cm 래스터로 광선 투사한 기대 거리와 비교
  — 정적 구조물 빔 3 412 개 잔차 **중앙값 −0.19 cm** (0~2 / 2~5 / 5~10 / 10~25 m 구간 −0.10 / −0.14 / −0.29 /
  −0.23 cm). 편향 없음.
- **스캔 시각 오프셋**: 제자리 0.5 / 1.0 rad/s 회전, 0.5 / 0.8 m/s 직진 중 스캔마다 GT(t_stamp + δt) 로 맞춘
  δt* 중앙값 **0 ms** (회전), −5~0 ms (직진). 스탬프 오차 없음.

### 6.3 위치 추정 정확도 (AMCL + 이중 EKF, `odometry/filtered_map` vs GT, 명세 3 / 5 / 8 cm)

맵: §6.1 SLAM 맵 (`maps/warehouse.yaml`), map↔world 는 §6.1 의 벽 정합 결과 (−16.060, −2.040, 0.056°) 로 **고정**
(위치 추정 실행과 독립인 1 회 보정). 경로 (한 바퀴 ≈ 150 s): 정지 30 s → 직진 14 m @0.5 m/s → 정지 5 s →
제자리 2π → 정지 → 원호 (r 1 m) → 정지 → 제자리 −π → 정지 → 직진 14 m @0.8 m/s → 정지 20 s. 첫 바퀴만 AMCL 을
GT 로 초기화하고 둘째·셋째 바퀴는 앞 바퀴가 끝난 상태에서 그대로 잇는다 (그 바퀴의 첫 정지 30 s 는 초기화 이점 없음).
최종 설정(반 셀 보정 + 주입 끔 + `wheel_separation_multiplier`), load average 7.5 → 17.3, 창 RTF 0.97~0.99.
`amr_evaluation analyze` 판정 (RMSE, max):

| 실행 | 정지 (3 cm) | 직선 (5 cm) | 회전 (8 cm) | 헤딩 RMSE | 판정 |
| --- | --- | --- | --- | --- | --- |
| 납치 세션의 정확도 경로 (1 바퀴, 초기화) | 2.78 (max 4.88) | 1.69 (4.46) | 1.33 (2.31) | 0.10° | PASS |
| 반복 1 바퀴 (초기화) | 2.82 (5.28) | 1.59 (5.11) | 1.01 (2.21) | 0.08° | PASS |
| 반복 2 바퀴 (이어서) | **3.79** (4.88) | 3.04 (4.93) | 1.92 (5.10) | 0.10° | 정지 FAIL |
| 반복 3 바퀴 (이어서) | 2.38 (3.01) | 2.58 (5.12) | 1.58 (3.90) | 0.09° | PASS |
| **4 바퀴 합산** (정지 12 468 / 직선 9 918 / 회전 8 178 샘플) | **2.99** | **2.30** | **1.52** | ≤ 0.10° | 경계 PASS |

정지 오차를 직전 동작별로 나누면 (같은 4 바퀴, 정지 명령 구간의 RMSE):

| 직전 동작 | 정지 수 | RMSE | 정지별 |
| --- | --- | --- | --- |
| 직진 0.5 m/s (동쪽) | 4 | 0.88 cm | 0.2 / 0.8 / 1.3 / 0.9 |
| 제자리 회전 | 8 | 0.92 cm | 0.2 ~ 1.4 |
| 원호 | 4 | 1.04 cm | 0.4 ~ 1.5 |
| **직진 0.8 m/s (서쪽, 긴 통로)** | 4 | **3.87 cm** | 4.9 / 3.0 / 1.8 / 4.9 |

직선·회전은 목표의 1/2~1/5 로 여유 있게 만족한다. 정지는 대부분 1 cm 수준이지만, 0.8 m/s 로 긴 통로(y = −2)를 따라
14 m 달린 뒤의 정지에서 통로 방향 오차 2~5 cm 가 남는다 (주행 중 쌓인 통로 방향 오차가 정지 후 무이동 갱신으로도
풀리지 않음). 이 정지가 경로의 정지 시간 대부분(끝 정지 20 s + 다음 바퀴 첫 정지 30 s)을 차지해 합산 RMSE 가
2.99 cm 로 경계에 걸린다. 같은 자리의 스캔-맵 정합 비용 지형을 떠 보면 SLAM 맵에서 통로 방향으로 ±5 cm 가 거의
평평하다 (맵의 통로 끝 구조물 번짐, §6.1 축척 −0.16 %) — 즉 **이 자리의 통로 방향 위치는 이 맵으로 정할 수 있는
정밀도 자체가 수 cm** 다. 개선 후보(열린 항목): 맵 축척 편향 제거(slam_toolbox `distance_variance_penalty` 조정 후
재매핑), 정지 순간 부호 거리장 스캔 정합으로 AMCL 재중심화(GT 맵 오프라인 시험 0.3 cm 수렴, SLAM 맵에서는 위
평평함 때문에 불확실), 고속 구간 `update_min_d` 조정.

설정별 비교 (반복 1 바퀴, 같은 경로·같은 SLAM 맵):

| 설정 | 정지 | 직선 | 회전 | 거짓 LOST |
| --- | --- | --- | --- | --- |
| 보정 전 (AMCL 이 `/map` 직접, Augmented MCL 주입 켬) | 4.63 | 4.17 | 4.01 | 스윕 4 바퀴 중 2 |
| + 반 셀 보정 (§4.1) | 1.75 | 2.02 | 1.18 | 1 바퀴 중 0 (같은 설정 납치 세션 1 회) |
| + 주입 끔 (**최종**) | 2.82 | 1.59 | 1.01 | 4 바퀴 중 0 |

반 셀 보정이 세 구간 모두 2~3 cm 를 줄였다 (정지 4.63 → 1.75~2.82 cm). 주입을 끈 뒤 정지 값의 차이는 위 "0.8 m/s 뒤
정지" 의 바퀴별 변동(1.8~4.9 cm) 범위 안이다. 주입을 끈 이유는 정확도가 아니라 **거짓 LOST** 다: 주입이 켜진
15 바퀴 중 5 바퀴에서 주행 도중 AMCL 공분산이 5~40 m² 로 부풀어 `localization/lost` 가 잘못 올라갔다 (BT 가 작업을
멈춘다). 끈 뒤 5 바퀴(납치 세션 포함)에서 거짓 LOST 0 회.

GT 맵(월드 visual 단면 래스터)으로 같은 경로를 돌리면 오히려 나쁘다 (반 셀 보정 후 정지 3.49 / 4.31 cm, 직선 2.70 /
3.78 cm, 회전 1.79 / 2.91 cm). GT 래스터는 "셀 중심이 물체 안" 인 셀을 점유로 두므로 표면에서 평균 s/2 안쪽부터
점유이고, AMCL 우도장(점유 셀 중심까지 거리)으로는 벽이 2.5 cm 멀리 보인다 — 광선 끝점이 떨어진 셀을 점유로 두는
SLAM 맵과 표현 규약이 다르다 (추정; 광선 끝점 규약 GT 래스터로 재확인은 열린 항목). 그래서 GT 맵은 맵 품질 평가에만
쓰고, 위치 추정 성능은 실제 운용 맵(SLAM)으로 보고한다.

### 6.4 납치 복구 (명세 "스스로 위치 복구")

시험 절차 (검증 하네스): 정지 로봇을 `ign service /world/warehouse/set_pose` 로 순간 이동 → `localization/lost` 전이와
map EKF 오차로 감지 지연·복구 시간·복구 10 s 뒤 오차를 잰다. 목표 8 곳 (월드 좌표; 점대칭 별칭이 있는 (15, −15, π)
포함), 최종 설정, load 7.5 → 16.1.

| 목표 (x, y, yaw) | 감지 [s] | 복구 (납치부터) [s] | 복구 10 s 뒤 오차 | 성공 |
| --- | --- | --- | --- | --- |
| (12, 0, 1.57) | 3.34 | 4.46 | 3.0 cm | ✔ |
| (−21.5, 8, 0) | 3.48 | 4.52 | 4.7 cm | ✔ |
| (15, −15, 3.14) | 3.36 | 4.48 | 1.8 cm | ✔ |
| (5, −6, 0.4) | 3.48 | 4.80 | 1.4 cm | ✔ |
| (−8, 11, −1.57) | 3.68 | 4.90 | 4.3 cm | ✔ |
| (20, 6, 2.5) | 3.18 | 4.30 | 2.8 cm | ✔ |
| (−2, −15, 0.8) | 3.50 | 4.70 | 2.4 cm | ✔ |
| (8, 11, 3.0) | 3.18 | 4.30 | 3.0 cm | ✔ |
| **8/8** | 평균 3.40 | 평균 4.56 (감지 후 1.04~1.32) | 평균 2.9 cm, 헤딩 ≤ 0.2° | 성공 기준: 10 s 뒤 < 20 cm, < 5° |

- 모든 경우 스캔-맵 인라이어 비율(0.01~0.07 < 0.5)이 LOST 를 확정했고 (정지 중 납치라 공분산·점프는 반응 없음),
  전역 가설 탐색 1 순위가 정답이었다 (탐색 0.21~0.33 s; 1 순위 인라이어 0.92~0.99, 2 순위 0.79~0.93). 시드 후
  회전 시작 직후(≈ 1 s) 수렴 판정이 나 `ekf_filter_node_map/set_pose` 로 map EKF 를 옮겼다.
- 같은 세션의 정확도 경로(§6.3 1 행)와 합쳐 LOST 는 정확히 8 회 (거짓 0).
- 이전 실행(고부하, 절사 점수 순위): 3 곳 중 (15, −15, π) 가 점대칭 별칭 (−15, 15, 0) 으로 오수렴해 42.6 m 오차로
  실패 → 최종 순위를 인라이어 비율로 바꾼 뒤 (§5.2) 이 목표를 포함해 24/24 (반 셀 보정 전 8, 보정 후 주입 켬 8, 최종 8) 성공.
- 한계: 창고가 완전 주기적인 구역에서는 스캔 하나로 별칭을 가를 수 없어 시드 3 개 + AMCL 전역 재초기화로도 틀릴 수
  있다 — 이동으로 가르는 DISAMBIGUATE (brief §4.C) 는 확장점.
