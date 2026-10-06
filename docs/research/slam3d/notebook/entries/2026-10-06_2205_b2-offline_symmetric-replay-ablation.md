# 파일럿 bag 재생 — 지표 대칭화, LIO-SAM 평면·절제 행, RTAB-Map IMU 제거, 재생 3 회 산포 (사전 등록)

- 시각: 2026-10-06 22:05 KST · 에이전트: b2-offline · 단계: B2-5r (보너스 2 5단계 재생 보강, Gazebo 없음)
- 코드: `eaca848` (브랜치 `feature/b2-offline`) — 실행 때 커밋은 실행 목록 `runs/b2-offline.jsonl` 의 `commit` 에 남긴다
- 관련 실행 id: `B2off-<행>-r<k>` (재생), `B2off-madg-derive` (IMU 재계산), `B2off-sym-old` (기존 산출물 재계산),
  `B2off-test-*` (시험·변이)
- 근거: `docs/research/slam3d/rtabmap.md` §3–§7 (s3 파일럿 결과), 리서치 노트
  `/home/CAPYI_AI/research/research_notes/실측 기반 후속 기술 조사/planar_lio_z_drift.md` 권고 R1–R15,
  `docs/research/slam3d/evaluation.md` §4.1·§7·§8 (지표·표본·재생 통제)

## 질문

1. 모든 행을 **같은** 정렬 없는 SE(2) 지표(xy ATE·yaw ATE·RPE, 같은 평가 시간창)로 다시 재면 s3 표의 순위·수치가 바뀌는가?
2. LIO-SAM 의 z 표류(최대 0.26 m)는 IMU roll/pitch 누설(imuRPYWeight) 탓인가, 퇴화 방향에서 IMU 예측을 따라간 탓인가?
   평면 행(z/rotation_tollerance 0, imuRPYWeight 0)은 z 를 없애면서 xy 를 해치지 않는가?
3. RTAB-Map Force3DoF 행에서 IMU 를 빼면 처리 스캔 누락이 0 이 되고 xy 정확도는 그대로인가?
4. 같은 bag·같은 구성을 3 회 재생하면 지표가 얼마나 흔들리는가 (행 간 차이를 읽을 문턱)?

## 가설과 예측 (결과 보기 전에 적는다)

- **H0 (재계산 정합)**: 기존 s3 산출물(`traj_online.tum`)을 새 도구로 같은 시간창·정렬 없음으로 재면 s3 표 xy ATE 와
  0.01 cm 안에서 같다 (s3 표는 이미 `tum_to_se2` 로 xy 만 썼다). 다르면 도구 결함으로 보고 먼저 고친다.
  공통 시간창으로 자르면 값이 0.1 cm 넘게 바뀌지 않는다고 예측한다 (창 차이는 끝 몇 장).
- **H1 (R2, imuRPYWeight 0)**: LIO-SAM z 최대가 기준(같은 재계산 IMU, loop on) 대비 ≤ 0.5 배면 "원인 2(roll/pitch 누설)가 지배",
  ≥ 0.8 배면 "원인 1(퇴화 + IMU 예측)이 지배", 그 사이면 "섞임". 예측: ≤ 0.5 배 (확신 낮음).
- **H2 (R4, madgwick gain 0.01)**: madgwick 출력 roll/pitch 오차(GT 방위 대비, 재계산 bag 전체) RMSE 가 gain 0.1 대비 줄어든다.
  예측: 0.1 에서 RMSE 0.5–3°, 0.01 에서 그 절반 이하. LIO-SAM z 최대 판정은 H1 과 같은 규칙.
- **H3 (R3, 정답 방위 상한 — 오라클)**: z 최대가 기준의 ≤ 0.5 배면 방위 경로가 원인, ≥ 0.8 배면 원인 1.
  **이 행은 정답 방위를 쓰므로 순위·비교 주장에 쓰지 않고 상한 참고로만 표기한다.**
- **H4 (R6, 평면 행)**: z 최대 < 1 cm, 지도 바닥 아래(z < −0.05 m) 점 < 0.5 %, xy ATE RMSE 가 6DoF 기준보다
  "재생 잡음 문턱"(아래) 넘게 나빠지지 않는다. 예측: 셋 다 통과. 루프 on 이므로 키프레임 그래프 최적화로 z 가 다시 생기는지
  (`traj_online` z) 도 본다.
- **H5 (R11, RTAB-Map IMU 제거)**: IMU 를 뺀 rtabmap3d·rtabmap2d 는 처리 스캔 누락 0/1 788. IMU 있는(재계산 IMU, 공백 없음)
  rtabmap3d 와의 xy ATE RMSE 차이가 재생 잡음 문턱 안 → "IMU 제거 확정". 예측: 둘 다 통과 (Force3DoF + 휠 guess 라 IMU 영향 없음).
- **H6 (재현 잡음)**: RTAB-Map·slam_toolbox 의 xy ATE RMSE 재생 간 범위(max − min) < 0.2 cm, LIO-SAM < 0.5 cm.

판정 규칙 (사전 등록):
- **재생 잡음 문턱** = 그 지표의 행별 재생 간 표준편차 중 큰 쪽의 2 배. 두 행은 (i) 재생 범위가 겹치지 않고 (ii) 평균 차가 문턱보다
  클 때만 "다르다" 고 쓴다. 아니면 "차이를 가를 수 없다". bag 이 1 개라 **장면 간 일반화는 주장하지 않는다** (n = 1 bag).
- 순위는 xy·yaw 지표로만 매긴다. z·roll·pitch·바닥 평탄도는 3D 전용 별도 열이고 순위에 넣지 않는다 (리서치 노트 Q4).
- 처리 스캔 누락: RTAB-Map 은 icp_odometry 로그 "Odom: ratio=" 줄 수(1 차)와 C++ 기록기 대응(2 차). LIO-SAM 은 imageProjection 출력
  대응이고 **bag 의 마지막 2 장은 구조적 캐시(imageProjection.cpp 228–230행)라 분모에서 뺀 값과 뺀 장 수를 둘 다** 적는다.
  slam_toolbox 는 처리 수를 셀 근거가 없어 미측정으로 둔다 (s3 와 같음).

## 설정

- bag: `/home/CAPYI_AI/runs/s3-rtabmap/s3_pilot_a/bag` (55 m, sim 178.7 s, 점군 1 788 장, 동적 행위자 없음). GT `gt.tum`
  (z 표준편차 3.5e-7 m, roll·pitch 0 — 평면 가정이 시뮬에서 정확히 맞는다 [계산, 이 항목 작성 때 numpy]).
- **재계산 IMU bag** (`B2off-madg-derive`): 원 bag 의 `imu/data`(바이어스 보정, 방위 = 정답) 35 766 개를 madgwick 노드
  (imu_filter_madgwick 2.1.5, amr-ai/slam:0.1, 우리 `madgwick.yaml`) **두 개(gain 0.1 / 0.01)** 에 한 개씩 넣고 출력이 나온 뒤
  다음을 넣는 잠금 단계(lockstep)로 다시 계산한다. 출력 header 는 입력 스탬프 그대로라 실시간 재계산과 같은 값이어야 한다
  (madgwick 은 입력 스탬프 차로 dt 를 낸다). 확인: 출력 수 = 입력 수, 기록된 madgwick 과 공백(9.48 s) 이전 구간이 같은지.
  파생 bag 은 재생에 쓰는 토픽만 담는다 (`/clock /tf /tf_static`, 점군, `scan_filtered`, `imu/data`, 재계산 madgwick 둘).
  "재생 중 재계산" 대신 이렇게 하는 이유: liosam 이미지에 imu_filter_madgwick 이 없고 [측정: `ros2 pkg list`], 재생 중
  best-effort 입력(깊이 5)을 잃으면 재생마다 IMU 가 달라져 재현 잡음 측정을 오염시킨다.
- 행 (모두 재계산 IMU bag 에서, `--rate 0.5`, 기록된 /clock, 컨테이너마다 `ROS_LOCALHOST_ONLY=1`, DOMAIN 45, 3 회씩):

| 행 | 이미지 | 바꾼 것 (s3 행 대비) | 근거 |
|---|---|---|---|
| `rtabmap3d_noimu` | slam | `imu_topic:=''` (IMU 안 씀 → `wait_imu_to_init false`) | R11 |
| `rtabmap3d_imu` | slam | 재계산 madgwick 0.1 (공백 없음), `wait_imu_to_init true` — IMU 유무 비교용 | R11 |
| `rtabmap2d_noimu` | slam | `imu_topic:=''` | R11 |
| `slam_toolbox` | slam | 없음 (입력 토픽이 원 bag 과 같은 바이트) | 재현 잡음 |
| `liosam_loop` | liosam | IMU = 재계산 madgwick 0.1 (6DoF 기준, 발표 그대로) | R1 |
| `liosam_noloop` | liosam | 위 + 루프 끔 | 재현 잡음 |
| `liosam_rpy0` | liosam | `imuRPYWeight 0` | R2 |
| `liosam_g001` | liosam | IMU = 재계산 madgwick **gain 0.01** | R4 |
| `liosam_planar` | liosam | `z_tollerance 0`, `rotation_tollerance 0`, `imuRPYWeight 0` | R6 |
| `liosam_oracle` | liosam | IMU = `imu/data` (**정답 방위 — 오라클, 상한 참고용**) | R3 |

- 재생 구동기 고침: 행 런치가 SIGINT 를 못 받아 20 s 뒤 SIGKILL 되던 것(s3 §4 "Killed")을 SIGINT 기본 처분 복구 래퍼로
  띄워 고친다 — RTAB-Map DB 에 최적화 자세가 남는지(`db_info.txt` "Optimized graph") 로 확인한다.
- 지표 (새 모듈 `amr_evaluation.slam3d.symmetric`, rclpy 비의존): 공통 시간창 = 비교하는 모든 실행의 `traj_online` 구간의 교집합.
  주 열: xy ATE RMSE·max (정렬 없음), xy ATE RMSE (SE(2) 정렬), yaw ATE RMSE·max (정렬 없음), RPE 1 m·10 m (%), RTE 10 s.
  3D 전용 열(3D 추정 행만, 2D 행은 해당 없음): z |오차| 최대·RMSE, roll·pitch RMSE, 지도 바닥 아래 점 비율(z < −0.05 m),
  바닥 대역 점의 z 1 백분위. 기존 6 행(s3 r0)도 같은 도구로 다시 재어 s3 표와의 차이를 적는다.
- 실행 묶음: `/home/CAPYI_AI/env/cpu_slot.sh b2-offline` 한 번에 재생 컨테이너 4 개 이하 (재생 하나 ≈ 7 분),
  묶음마다 loadavg 를 남긴다 — 공유 부하 하 예비값. 원자료 `/home/CAPYI_AI/runs/b24/b2-offline/`.

### 사전 등록 뒤 바꾼 것 (재생 결과를 보기 전, 22:20 기록)

1. **오라클 IMU 토픽을 `imu/data` 에서 합성 토픽 `imu/data_gtorient` 로 바꿨다.** 파생 bag 을 만들며 재 보니 `imu/data` 의
   방위는 정답이 아니라 **항등 쿼터니언**이다 — 55 m·4 회전 동안 yaw 가 끝까지 0, 항등과의 각 최대 0.0° [측정:
   `/home/CAPYI_AI/runs/b24/b2-offline/derive/derive.json` `imu_data_orientation_vs_identity_max_deg`, `tmp/yawcheck.py`],
   코드도 그렇다 (`src/amr_localization/src/imu_filter_node.cpp` 147–149행 `orientation.w = 1`, 공분산[0] = −1).
   정답 방위는 Gazebo 원시 `imu/data_raw` 에만 있다. 그래서 오라클 행 입력은 `imu/data`(바이어스 보정 각속도·가속도)에 같은
   스탬프 `imu/data_raw` 의 방위만 옮긴 토픽으로 만들었다 (각속도·가속도는 다른 행과 같다). 과제 지시·s3 문서(rtabmap.md §1.1
   "imu/data 의 방위 필드는 Gazebo 정답")·리서치 노트의 "정답 방위 토픽 = /amr_01/imu/data" 는 **틀렸다** — 문서에서 고친다.
   이 평평한 월드에서는 GT roll·pitch 가 정확히 0 이라(위 설정) 항등 방위도 roll·pitch 는 정답과 같다 — 그래서 `imu/data` 도
   SLAM 입력에서 계속 막는다.
2. 이름: 사전 등록 표의 `liosam_loop`·`liosam_noloop` (재계산 IMU) 은 코드에서 `liosam_loop_re`·`liosam_noloop_re` 다 —
   원 bag·기록된 madgwick 을 쓰는 s3 행 이름과 섞이지 않게.
3. **원 bag 재생 3 회를 더한다**: s3 의 `rtabmap3d`·`rtabmap2d`·`liosam_loop`·`liosam_noloop` (기록된 madgwick, 공백 포함) 을
   그대로 3 회씩. s3 표 값 자체의 재현 잡음과, 기록 IMU ↔ 재계산 IMU 차이(같은 행 설정)를 직접 본다. 판정 규칙은 같다.
4. 재생 동시 수: 첫 묶음(4 개)에서 컨테이너당 CPU 0.15–0.36 코어 [측정 `docker stats`] 라 나머지는 동시 13 + 6 개로 돈다.
   동시 수가 처리 누락을 늘리는지는 묶음별 loadavg 와 누락 표로 본다.

파생 bag 확인 [측정]: 입력 35 766 개 → 이득마다 출력 35 766 개, 잠금 단계 벽시계 2.5 s, 예상 밖 출력 0, 노드 파라미터
`gain` 0.1 / 0.01 확인 (`ros2 param get`). 같은 명령을 두 번 돌린 파생 bag(v1 `_v1_derive_no_oracle`, v2 `derive`)의 재계산
madgwick·점군·imu/data 가 바이트와 bag 시각까지 같다 (`derive/determinism_v1_v2.log`) — 잠금 단계 재계산은 결정적이다.

## 결과

전체 표·진단은 `docs/research/slam3d/replay.md` (§3 표, §4 판정, §5 산포, §6 누락, §7 원인). 원자료 `/home/CAPYI_AI/runs/b24/b2-offline/`
(`replay/<행>/r<k>/`, `replay_orig/<행>/r<k>/`, `summary_final/{sym_final,judgements}.json`, `derive/`, `sym_old/`, `mut/`, `tmp/*.log`).
재생 43 회 (파생 bag 31, 원 bag 12), 모두 rc 0. 재생 중 loadavg 3–36 (같은 시간 보너스 1 Gazebo 측정) — **공유 부하 하 예비값**.

**불리한 것 먼저**
- RTAB-Map 처리 스캔 누락 0 **미달**: IMU 를 빼도(`rtabmap3d_noimu` 4 회, `rtabmap2d_noimu` 3 회), 공백 없는 IMU 를 줘도(`rtabmap3d_imu` 3 회)
  모든 재생에서 9.6 s 한 장(1/1 788)이 빠진다. 원 bag(기록 IMU) 행은 9.5 s 한 장 (6 회 모두). 원인: 휠-EKF TF guess 조회 실패 —
  기록 때 sim 9.48–9.64 s 에 `/clock`·EKF TF·madgwick 이 함께 멈췄다. 첫 15 s 재생 시험에서 guess 끔 → 9.6 처리, `wait_for_transform`
  0.2·1.0 s → 둘 다 누락 [측정 `tmp/waittest*.log`, `tmp/tfgap.log`, `tmp/order.log`].
- H5 미달 (누락·xy 둘 다): xy RMSE `rtabmap3d_noimu` 0.7710 vs `rtabmap3d_imu` 0.7697 cm, 차 0.0014 cm 가 재생 sd(0.0002) 문턱을 넘어
  규칙상 "다르다" — 실용상 무의미. **사전 등록 규칙에 실용 동등 한계가 없었던 결함**.
- H6 미달 2 행: slam_toolbox xy 재생 범위 0.39 cm (> 0.2), `liosam_loop@orig` 0.86 cm (> 0.5). 둘 다 한 재생의 사건(메시지 버림 1 회,
  기동 경합)에서 나왔다.
- H1·H2·H3 예측 틀림: LIO-SAM z 최대(기준 `liosam_loop_re` 29.34 ± 0.82 cm)는 imuRPYWeight 0 → 31.67 (1.08 배), gain 0.01 → 28.66
  (0.98 배), 정답 방위(오라클) → 26.64 (0.91 배). 사전 규칙상 모두 "원인 1(퇴화 + IMU 예측) 지배". 원인 1 자체(R5)는 재지 않았다.
- 기동 경합: LIO-SAM 24 재생 중 4 회가 첫 2–3 장을 놓쳤다 (15 재생 동시 기동, loadavg 12–14).
- 첫 SIGINT 고침 실패(34 재생 SIGKILL 종료), 첫 변이 실행 무효(PYTHONPATH 덮어씀) — 둘 다 진단·재실행.
- 사전 등록 뒤 오라클 토픽을 바꿨다 (`imu/data` 방위가 항등 — 위 "사전 등록 뒤 바꾼 것").

**표 (공통 창 [4.6, 182.5] s, 평균 ± sd, n = 재생 수) [측정·계산]**

| 행 | n | xy ATE RMSE cm | yaw RMSE ° | z 최대 cm | 바닥 아래 % | 판정 누락 / 재생 |
|---|---|---|---|---|---|---|
| rtabmap3d_noimu | 4 | 0.771 ± 0.000 | 0.011 | 0.00 | 0.00 | 1 (9.6) ×4 |
| rtabmap3d_imu | 3 | 0.770 ± 0.000 | 0.011 | 0.00 | 0.00 | 1 (9.6) ×3 |
| rtabmap2d_noimu | 3 | 2.15 ± 0.02 | 0.072 | — | — | 1 (9.6) ×3 |
| slam_toolbox | 3 | 1.87 ± 0.23 | 0.156 | — | — | 미측정 |
| liosam_planar | 3 | 1.71 ± 0.10 | 0.086 | 0.25 ± 0.03 | 0.00 | 0 ×2, 2 (기동) ×1 |
| liosam_loop_re | 3 | 5.69 ± 0.23 | 0.077 | 29.34 ± 0.82 | 4.00 | 0 ×3 |
| liosam_noloop_re | 3 | 5.50 ± 0.08 | 0.075 | 28.90 ± 0.35 | 4.05 | 0 ×3 |
| liosam_rpy0 | 3 | 6.04 ± 0.20 | 0.072 | 31.67 ± 1.21 | 4.54 | 0 ×3 |
| liosam_g001 | 3 | 5.58 ± 0.22 | 0.077 | 28.66 ± 0.43 | 3.94 | 0 ×2, 2 (기동) ×1 |
| liosam_oracle (상한 참고) | 3 | 5.58 ± 0.20 | 0.075 | 26.64 ± 0.41 | 3.99 | 0 ×2, 3 (기동) ×1 |
| rtabmap3d@orig | 3 | 0.770 ± 0.000 | 0.011 | 0.00 | 0.00 | 1 (9.5) ×3 |
| rtabmap2d@orig | 3 | 2.19 ± 0.03 | 0.073 | — | — | 1 (9.5) ×3 |
| liosam_loop@orig | 3 | 5.62 ± 0.48 | 0.071 | 26.56 ± 0.36 | 3.85 | 1 (9.6) ×3 (+ 기동 2, r0) |
| liosam_noloop@orig | 3 | 5.29 ± 0.06 | 0.069 | 26.72 ± 0.41 | 4.22 | 1 (9.6) ×3 |

재생별 값은 `runs/b2-offline.jsonl` (run_id `B2off-<행>[-orig]-r<k>`).

**사전 등록 판정**: H0 통과 (s3 값 0.01 cm 안 재현, 공통 창 변화 ≤ 0.0015 cm) · H1 원인 1 지배 (예측 틀림) · H2 방위 통과(pitch RMSE
0.49° → 0.14°, roll 0.061° → 0.022°)·z 원인 1 지배 · H3 원인 1 지배 · H4 통과 (z 0.25 cm, 바닥 아래 0 %, xy 5.69 → 1.71 cm) ·
H5 미달 · H6 미달 2 행.

**madgwick 재계산** (`B2off-madg-derive`): 출력 35 766 = 입력, 두 번 만든 파생 bag 바이트 동일. 기록된 출력과 공백 앞 7 × 10⁻⁵°,
공백 직후 1.47° → 기록 때 madgwick 이 입력을 놓쳤다 (s3 의 "원인 미확인" 해소, 추론 포함).

**시험·변이**: amr_slam3d 70 + amr_evaluation slam3d 47(대칭 8 포함) 통과, flake8·pep257 0. 변이 14 개 모두 잡힘 (`mut/mutation.json`).

## 해석

- 지표를 대칭화해도 xy 순위는 s3 와 같다 — s3 표는 이미 xy 였다. 새로 보인 것은 (1) yaw 는 모든 행이 0.2° 아래라 xy 차이는 위치
  표류에서 나온다, (2) LIO-SAM 6DoF 의 z 26–32 cm·roll/pitch 0.9–1.2° 가 같은 표의 별도 열로 드러난다.
- LIO-SAM z 표류는 IMU 방위 품질 문제가 아니다 (정답 방위로도 9 % 만 준다). 평면 clamp 는 z 를 없애고 xy 를 3.3 배 줄였다 —
  평면 로봇에서 LIO-SAM 을 쓴다면 평면 행이 기본이어야 한다. 이 결론은 이 bag 하나에서다.
- RTAB-Map 의 IMU 는 Force3DoF + 휠 guess 에서 정확도에 영향이 없다 (0.0014 cm). IMU 를 빼도 손해가 없고 IMU 대기 버림 위험이
  사라진다 — 그러나 이 bag 에서는 같은 기록 정체가 휠 guess 쪽으로 한 장을 다시 빼앗는다.
- 재현 잡음은 대부분 사건(기동 경합·메시지 버림)이다. 같은 입력을 받은 재생은 거의 같은 답을 낸다. 재생 잡음 문턱(2 sd)은 그 부하에서만
  유효하다.
- 말하지 않는 것: 장면 일반화, 동적 장애물, 3D 의 통로 끝 이득(S6), 휠 오라클 없는 조건에서의 순위.

## 다음

replay.md §8 의 녹화 계획 (Gazebo 필요, 보너스 1 측정과 겹치지 않는 시간대): 기록 madgwick 제거 + 잠금 단계 파생, 기록 직후 공백 검사,
출발 전 정지 구간, 구독 확인 뒤 재생 시작, S1·S3·S6 bag 3 개씩, 행 6 개 × 재생 3. LIO-SAM 원인 1 의 직접 확인(R5), 휠 열화·guess 없음 행.
사전 등록에는 실용 동등 한계를 넣는다.
