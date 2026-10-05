# 3D LiDAR 센서 계약 — 토픽·프레임·QoS·필드 (보너스 2, s1-sensor)

이 문서는 3D LiDAR SLAM(RTAB-Map·LIO-SAM, s3)이 **그대로 쓰는** 센서 쪽 계약이다. 바꿀 필요가 생기면 이 문서와
`src/amr_description/config/lidar3d.yaml` 을 함께 고친다. 근거 보고서: 「3D LiDAR SLAM 도입 방안」 31–80행(센서),
191–200행(오라클), 255–272행(1–2단계). 표기: **[측정]** 명령·로그 있음, **[코드]** 이 저장소 코드로 정해진 값,
**[문서주장]** 보고서·상류 소스 주장(우리 실행에서 미확인).

## 1. 켜는 법

| 방법 | 효과 |
|---|---|
| `multi_robot.launch.py robots_file:=…/fleet_spawn_lidar3d.yaml` | amr_01 만 3D (나머지 4대는 기본과 같다) |
| 스폰 목록 로봇 항목에 `lidar3d: true` | 그 로봇만 3D (`amr_bringup/fleet_spawn.py` 가 형식 검증) |
| `description.launch.py lidar3d:=true` + `spawn.launch.py lidar3d:=true` | 단독 기동 (두 런치에 같은 값을 줘야 한다) |

- 두 런치의 `lidar3d` 기본값은 `auto` 다: 런치 범위에 `robots_file` 이 있으면 그 파일의 이 로봇 `lidar3d` 키, 없으면
  false (`amr_description/lidar3d.py resolve`). `multi_robot`·`system.launch.py` 는 `robots_file` 을 최상위 인자로
  선언하므로 그 아래 description·spawn 런치가 같은 값을 본다 [코드].
- 기본(키 없음·false)의 URDF 는 3D 이전(d8ae80c)과 **바이트 단위로 같고**, 브리지 항목도 같다 —
  `src/amr_description/test/test_urdf_golden.py` [코드·시험].

## 2. 토픽 (로봇 네임스페이스 `/<robot>/` 아래 상대 이름)

| ROS 토픽 | 형 | 발행자 | QoS | 내용 |
|---|---|---|---|---|
| `lidar3d/points_raw` | sensor_msgs/PointCloud2 | ros_gz_bridge (gz `/<robot>/lidar3d/points`) | RELIABLE·VOLATILE [측정], depth 10 [코드 주석] | Gazebo 원본, 10 Hz [측정]. 브리지 항목은 lazy 지만 전처리가 스폰 때부터 구독하므로 사실상 늘 켜져 있다 (아래 "따라잡기") |
| `lidar3d/points` | sensor_msgs/PointCloud2 | `lidar3d_preprocessor` (amr_lidar3d) | RELIABLE·VOLATILE [측정], keep_last 5 [코드] | **소비자는 이것을 쓴다.** 비유한 점 제거, `height 1`, `is_dense true`, 필드 배치 그대로, 10 Hz [측정] |
| `imu/data_raw` | sensor_msgs/Imu | ros_gz_bridge | reliable, keep_last 10 | 3D 로봇만 **200 Hz** (다른 로봇 100 Hz). 방위는 정답 — 6절 |
| `ground_truth/odom` | nav_msgs/Odometry | ros_gz_bridge | reliable | 평가 전용 정답 (frame `world`, child `<prefix>base_footprint`, 50 Hz) |

- 다층 LaserScan(gz `/<robot>/lidar3d`)은 **브리지하지 않는다**: ros_gz_bridge(humble)는 다층 스캔을 "가운데 행"만
  넘긴다 [문서주장 — 보고서, ros_gz convert/sensor_msgs.cpp].
- **따라잡기 함정 — 센서를 늦게 켜지 마라.** Fortress 센서의 다음 갱신 시각은 한 주기씩만 늘어난다
  (`nextUpdateTime += period`). 그래서 구독자가 없어 꺼져 있던 센서를 시뮬레이션 시각 T 에 켜면 놓친 T × 10 장을
  물리 스텝마다(1 kHz) 몰아서 렌더하며 따라잡는다 (스폰을 늦추면 생기는 현상과 같은 원인 — spawn.launch.py 머리말).
  [측정] `s1_check_a`(전처리 lazy): 켠 뒤 3.7 s(sim) 동안 98 장, 간격 중앙값 0.001 s, 26 Hz(sim).
  `s1_check_b`(lazy 끔): 8.9 s(sim) 동안 90 장, 간격 0.100 s 일정, 짧은 간격 0 개.
  그래서 `spawn.launch.py` 가 띄우는 전처리는 **lazy 를 끄고** 스폰(월드 일시 정지) 때부터 입력을 구독한다 →
  3D 로봇의 센서는 sim 0 부터 10 Hz 로 렌더한다 (그 로봇의 렌더 비용은 늘 든다 — 7절).
  전처리 노드의 `lazy:=true` 는 선택으로 남겨 두었지만, 기동 직후(sim ≈ 0)에 소비자가 붙는 것이 보장될 때만 쓴다.
- QoS 를 reliable 로 둔 이유: RTAB-Map 기본 구독이 reliable 이고, reliable 발행자에는 best-effort 구독자도 붙는다.
  반대(best-effort 발행자 + reliable 구독자)는 연결되지 않는다. 전처리의 입력 구독은 best-effort(SensorDataQoS)
  — `ros2 topic info -v` 실측 (s1_check_c/topic_info.txt). 발행 깊이(depth)는 info 가 UNKNOWN 으로 내 확인 못 했다.
- 컴포넌트: `amr_lidar3d::CloudPreprocessor` — SLAM 노드와 같은 컨테이너에 올리면 intra-process (`launch/preprocessor.launch.py container:=…`).
  spawn.launch.py 가 띄우는 기본 인스턴스와 **중복으로 띄우지 않는다** (`lidar3d_preprocessor:=false` 로 끈 뒤 컨테이너에 싣는다).

## 3. 프레임

```
<prefix>base_footprint ─ <prefix>base_link (z 0.18) ─ <prefix>lidar3d_link   xyz (0.27, 0, 0.72), rpy 0
```

- `lidar3d_link`: 본체 규약 (x 전방, y 좌, z 상), **광학 중심 지면 +0.90 m**, 데크 앞끝(차체 전면 3 cm 안쪽) 마스트 위 [코드].
  base_footprint 기준 (0.27, 0, 0.90). robot_state_publisher 가 `/tf_static` 으로 낸다.
- 점군 `header.frame_id` = `<prefix>lidar3d_link` (Gazebo `ignition_frame_id`; 전처리는 기본으로 바꾸지 않는다).
  `multi_robot` 의 접두어는 `amr_01/`, `system.launch.py` 단일 로봇은 접두어 없음.
- `header.stamp` = 스캔 한 장의 시뮬레이션 시각 (`/clock`). LiDAR·IMU·정답이 같은 시계라 시간 오프셋 0.

## 4. 점군 형식

| 항목 | 값 | 근거 |
|---|---|---|
| 필드 | `x` FLOAT32 @0, `y` FLOAT32 @4, `z` FLOAT32 @8, `intensity` FLOAT32 @16, `ring` UINT16 @24 | [측정] raw·전처리 같음 |
| point_step | 32 B (정렬 패딩) | [측정] |
| **점별 time** | **없다.** 스캔 전체가 한 시각·한 자세의 순간 렌더 (모션 왜곡 없음) | [문서주장] GpuLidarSensor.cc |
| ring | 0 = 가장 아래 빔(−15.0°) … 15 = 위(+15.0°), 2° 간격 — 링별 고도각 중앙값이 정확히 −15, −13, …, +15° | [측정] s1_check_c |
| 메모리 순서 | 링 우선(행 = 링 하나, 1800 열). 각 링은 방위 −π(= ±180°)에서 시작해 0.2° 씩 증가, 첫·끝 열 겹침 없음 | [측정] 링 바뀜 15 회 · [코드] SDF |
| raw 조직 | `height 16`, `width 1800` (조직 점군), `is_dense false` | [측정] |
| raw 비유한 점 | 범위 밖 빔은 **+inf**(x·y·z 모두), NaN 0 개. 스폰 자리(18, −16)에서 28 800 중 3 946 (13.7 %) — 주로 위쪽 빔(천장 8 m 가 30 m 밖) | [측정] 전처리 유지 24 854 |
| 전처리 출력 | `height 1`, `width` = 유한 점 수, `row_step = width·32`, `is_dense true`, 순서 유지 (링 우선 그대로) | [측정·시험] |
| intensity | **모든 점 0.0** — 시각체 `laser_retro` 를 정하지 않아서다. 반사율 물리 모델이 아니므로 SLAM 특징으로 쓰지 마라 | [측정] 고유값 1 개 |
| 거리 | 0.10–30 m, 빔별 가우시안 σ 0.03 m (Fortress 는 이것만 지원) | [코드] |
| 크기 | 스캔당 28 800 빔 × 32 B = 0.92 MB (raw), 10 Hz | [계산] |

### 소비자가 할 것 (s3)

- **RTAB-Map**: `deskewing:=false` (time 이 없다), 최소거리는 SLAM 쪽에서 0.2 m (`Icp/RangeMin` 등 — 통로 랙 면이 횡 0.30 m).
- **LIO-SAM**: `ring` 이 있어 "ring channel not available" 로 죽지 않는다. `time` 이 없어
  "Point cloud timestamp not available, deskew function disabled" 경고가 나온다 — 정상이다.
  **time 을 합성하지 않는다** (`col/width·0.1 s` 를 넣으면 LIO-SAM 이 없는 왜곡을 IMU 회전 ω·t 만큼 "보정"해
  1.5 rad/s 에서 10 m 점이 약 1.5 m 어긋난다 — 보고서 계산). `N_SCAN 16`, `Horizon_SCAN 1800`, `lidarMinRange 0.2`.
- PCL 로 받을 때 `time` 필드가 없다는 경고(pcl::fromROSMsg "Failed to find match for field 'time'")는 무시해도 된다.

## 5. 센서 정의 (Gazebo Fortress `gpu_lidar`, `urdf/amr_lidar3d.xacro`)

16 채널, 수직 −15°…+15° (samples 16, resolution 1), 수평 1800 열 −π…π−2π/1800 (resolution 1 — Fortress 는
resolution≠1 보간을 하지 않는다), 10 Hz, range 0.10–30 m, 가우시안 σ 0.03, `visibility_mask` 로 자기 차체·마스트·화물
제외(다른 로봇은 보인다), 시각체만 있는 마스트(충돌체·관성 없음 — 동역학·적재 동작 무변화).

## 6. IMU 방위 오라클 — 쓰지 마라

Fortress IMU 는 가속도·자이로에만 잡음을 붙이고 **방위(orientation)는 잡음 없는 정답**이다 (기준 = 스폰 시점 자세).
`enable_orientation` 을 끄면 w=0 쿼터니언이 나가 소비 노드가 깨질 수 있어 켜 둔다 (기존 결정). 그러므로
3D SLAM 의 IMU 입력은 방위를 버리고 다시 만든다: `imu/data_raw` → 프로젝트 imu_filter_node(바이어스 보정) →
imu_filter_madgwick(`use_mag:=false`) (보고서 65–69행). 정답 방위를 쓴 실행은 "오라클 행"으로만 따로 보고한다.
3D 로봇 IMU: 200 Hz, 이산 σ = 기본 σ × √2 (자이로 2.83e-4 rad/s, 가속도 0.024 m/s²), 바이어스 그대로 [코드].

우리 실행에서의 확인 [측정] (`s1_check_a`·`b`·`c`, 같은 결과 세 번):

| 항목 | 값 | 뜻 |
|---|---|---|
| 정지 중 방위 쿼터니언 성분 표준편차 최대 | 1.1–1.3 × 10⁻¹⁵ (부동소수 바닥) | 방위에 잡음이 **없다** |
| 정지 중 자이로 σ (축별) | 2.77–2.91 × 10⁻⁴ rad/s | 설정 2.83 × 10⁻⁴ 과 맞다 → 잡음은 각속도에만 |
| 정지 중 가속도 σ (축별) | 0.023–0.025 m/s² | 설정 0.024 와 맞다 |
| 자이로 평균 (축별) | ±0.0100 rad/s | 정적 바이어스 0.01 (부호 무작위) |
| 제자리 회전 1.4–3.0 rad 동안 IMU 방위 yaw 변화 − 정답 yaw 변화 | 최대 1.0 × 10⁻⁴ rad, 중앙값 0 | 정답(50 Hz)을 IMU 시각(200 Hz)에 선형 보간한 오차 수준 — 방위 = 정답 |
| 같은 구간 자이로 적분 yaw − 정답 | 끝에서 0.15–0.22 rad | 바이어스 0.01 rad/s × 15–22 s. 방위는 자이로 적분이 아니다 |
| IMU 주기 (헤더 스탬프) | 200.0 Hz, 간격 0.005 s 일정 | |

→ 보고서 주장("Gazebo IMU 방위에 잡음이 없다")이 우리 실행에서 그대로 확인됐다. **3D SLAM 의 IMU 경로에
`imu/data_raw` 의 방위를 넣으면 3D 쪽만 정답 자세를 받는다** — 반드시 madgwick 로 다시 만든다.

## 7. 측정 (보고서 1–2단계)

모든 실행: 이미지 `amr-ai/rl:0.3`, ROS_DOMAIN_ID 206, IGN_PARTITION `amr_ai_s1`, `gz_slot.sh` 경유, 로그
`/home/CAPYI_AI/wt/s1-sensor/logs/<태그>/` (HEAD.txt·diff_stat.txt·env.txt 포함). 구동: `amr_lidar3d/scripts/run_lidar3d_bench.sh`.

### 7.1 형식·주기 (1단계 통과 기준) [측정]

| 기준 (보고서 1단계) | 결과 | 태그 |
|---|---|---|
| `/points` 10 Hz | `lidar3d/points` 10.000 Hz (헤더 스탬프), 간격 0.100 s 일정, 짧은 간격 0 — 5대 동시에도 10.00 Hz | s1_check_b·c, s1_load_L1_a·L5_a |
| 필드 xyz/intensity/ring | x·y·z·intensity FLOAT32 @0/4/8/16, ring UINT16 @24, point_step 32 | s1_check_a·b·c |
| 전처리 후 `is_dense` | raw `is_dense false`(16×1800) → 전처리 `is_dense true`, height 1, 24 854 점 | s1_check_a·b·c |
| 3D 로봇 IMU 200 Hz | 200.0 Hz | s1_check_a·b·c, s1_load_L1_a |
| 전처리 비용 | 스캔당 0.6–0.9 ms (28 800 → 24 854 점), 0.014 코어 | s1_check_b launch.log, s1_load_L1_a |

### 7.2 센서 오차 — gpu_lidar 점 vs 정답 시각체 면 [측정]

조건: 동적 행위자(작업자 6·지게차·셔틀)를 뺀 월드(`make_static_world.py`)에 amr_01 만, 스폰 자리(18, −16, yaw π/2)에
정지, 스캔 20 장 = 497 080 점. 정답 = 월드 SDF 시각체(상자 446 + 평면 2, 건너뛴 형상 0). 점을 정답 자세
(`ground_truth/odom`, 정지 중 위치 표준편차 < 1e-20 m)와 장착 외부 파라미터로 월드 좌표로 옮겼다.
세 실행(a·b·c)의 값이 소수 넷째 자리까지 같아 c 만 적는다.

| 지표 | 값 |
|---|---|
| 가장 가까운 정답 면까지 거리 \|d\| 중앙값 / p95 | **0.010 m / 0.044 m** |
| \|d\| ≥ 0.5 m 비율 (정답에 없는 곳에 찍힌 점) | 0.05 % — 원인 미확인 (가설: 모서리 혼합 화소) |
| 빔 방향 거리 잔차(측정 − 정답 면까지 광선 거리) 부호 중앙값 / 평균 | +0.5 mm / +1.8 mm — **계통 편차 없음** |
| 잔차 표준편차 (\|잔차\| < 0.5 m) | 0.062 m 전체 |

거리대별 잔차 표준편차 (설정 σ = 0.03 m):

| 거리 | 바닥 점 (월드 z < 0.1) | 바닥 외 점 |
|---|---|---|
| 0–5 m | 0.035 | 0.030 |
| 5–10 m | 0.054 | 0.031 |
| 10–20 m | **0.164** | 0.045 |
| 20–31 m | — | 0.096 |

해석: 5 m 안과 벽·랙 면은 σ 0.03 과 맞는다. 먼 바닥은 σ 의 5 배다 — 0.90 m 높이에서 10–20 m 바닥은 입사각
2.6–5° 로 스친다. 렌더 기반 광선의 각도 양자화가 스치는 면에서 큰 거리 차가 된다는 가설과 맞지만 각도 오차 자체는
재지 않았다(가설). 바닥 점의 면 거리(수직 거리)는 p95 1.5–2.1 cm 로 작다 — 거리 방향 오차가 면에 거의 평행하기
때문이다. 보고서 2단계 기준("잡음 σ보다 큰 계통 편차는 원인 확인"): 편차(중앙값)는 0 이고, σ 보다 큰 것은
**퍼짐**이며 위치(먼 바닥)를 특정했다. gz-sim #2743 의 "Classic 보다 부정확" 과 같은 현상인지는 Classic 과 비교하지
않아 모른다. 한 자세·정지 상태만 쟀다 (움직이는 중·다른 자리 미측정).

### 7.3 부하 (보고서 2단계 최소 집합) — **공유 부하 하 예비값**

조건: `multi_robot.launch.py` 5대, 스택·전역 노드 없음(월드 + 로봇 + 센서 브리지 + 전처리), 조건당 측정 300 s
(구독 후 20 s 뒤 시작). "켬" = 프로브가 `/amr_0N/lidar3d/points` 를 구독(전처리는 어차피 기동 즉시 구독).
"끔" = `lidar3d_preprocessor:=false` — 3D 센서가 URDF 에 있지만 아무도 구독하지 않는다(lazy 브리지). 같은 시각에
다른 에이전트의 Gazebo·TLA+·학습이 돌았다 (loadavg 31–117, 32 코어).

| 조건 | 태그 | RTF | RTF 10 s 창 최소/중앙/최대 | gz 서버 코어 | gz CPU s / sim s | 브리지 코어 | 전처리 코어 | 3D 점군 hz(sim) | 3D 간격 최대 s | 2D scan hz | IMU hz(amr_01) | GPU 사용률 중앙/최대·메모리(장치 전체) | loadavg 중앙 (최소–최대) | 동시 Gazebo |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| N0 | s1_load_L0_a | 0.205 | 0.106/0.181/0.353 | 1.15 | 5.62 | 0.25 | 0.000 | — | — | 10.00 | 100.0 | 5.0/15.0 %, 4386.0 MiB | 71.31 (31.55–93.96) | amr-ai-f0-holdfix-gz-f0_s12_a |
| N1_on | s1_load_L1_a | 0.193 | 0.072/0.193/0.291 | 1.22 | 6.30 | 0.28 | 0.014 | 10.00–10.00 (1대) | 0.100 | 10.00 | 200.0 | 4.0/18.0 %, 4408.0 MiB | 61.3 (41.68–116.78) | amr-ai-f0-holdfix-gz-f0_s12_b |
| N5_on | s1_load_L5_a | 0.139 | 0.084/0.144/0.182 | 1.18 | 8.51 | 0.26 | 0.061 | 10.00–10.00 (5대) | 0.100 | 10.00 | 200.0 | 10.0/25.0 %, 4491.0 MiB | 42.88 (33.02–55.83) | amr-ai-f0-holdfix-gz-f0_s12_b |
| N1_off | s1_load_L1off_a | 0.207 | 0.111/0.206/0.268 | 1.30 | 6.28 | 0.30 | 0.000 | — | — | 10.00 | 200.0 | 6.0/23.0 %, 1559.0 MiB | 42.48 (31.15–55.59) | amr-ai-f0-holdfix-gz-f0_s12_b, amr-ai-f0-holdfix-gz-f0_s10_a |
| N0 | s1_load_L0_b | 0.174 | 0.143/0.173/0.219 | 1.04 | 6.00 | 0.24 | 0.000 | — | — | 10.00 | 100.0 | 4.0/8.0 %, 1537.0 MiB | 49.95 (39.55–67.46) | amr-ai-f0-holdfix-gz-f0_s10_a |

읽는 법과 한계:
- **N=5(5대 모두 3D)는 분명히 무겁다**: RTF 0.139 로 N0 두 번(0.174·0.205)보다 낮고, Gazebo 서버 CPU 초 / 시뮬레이션 초가
  8.51 로 N0 평균 5.81 보다 46 % 많다 — loadavg 는 오히려 N5 쪽이 낮았다(43 vs 50·71).
- **N=1 은 이 측정으로 가를 수 없다**: N1 켬 6.30 · N1 끔 6.28 · N0 5.62/6.00. 켬과 끔이 같고, N0 두 번의 차(7 %)가
  N1 과 N0 의 차와 비슷하다. RTF 0.193 도 N0 범위(0.174–0.205) 안이다. 보고서 2단계 통과 기준 "1대 장착 시 RTF 하락 폭
  기록" → **하락 폭이 공유 부하 변동보다 작아 분리 안 됨** 이 기록이다. 가르려면 조용한 시간대에 ABAB 반복이 필요하다.
- 시뮬레이션 시각 기준 주기는 모든 조건에서 10 Hz 그대로다 (Fortress Sensors 시스템은 렌더 중이면 물리 PostUpdate 가 기다린다 [문서주장 — 보고서, Sensors.cc] —
  비용은 RTF 로만 나타난다). 벽시계 주기 = 10 × RTF (N5 에서 1.39 Hz).
- GPU 는 장치 전체(공유) 값이라 3D 몫을 가를 수 없다. 사용률 중앙값 4–10 %, 메모리는 같은 시간대 N0→N1→N5 가
  4386 → 4408 → 4491 MiB 였지만 다른 사용자 몫이 섞여 [참고] 다. 병목은 GPU 가 아니라 CPU(렌더 스레드 대기 포함)로 보인다.
- 미측정: 5 Hz 조건, 브리지 켬/끔 × N5, 스택(Nav2·측위) 포함 부하, 같은 조건 반복(N1·N5 는 한 번씩).

