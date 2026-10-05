# 보너스 2 평가 설계 — 정답·지표·요인·시나리오·표본·재생 통제

작성 2026-10-06, 브랜치 `feature/slam3d-eval` (기준 develop `d8ae80c`). 근거 보고서는 같은 폴더의
[`3d-lidar-slam-report.md`](3d-lidar-slam-report.md) (원본 사본) 77–80행(정답), 191–239행(공정성·요인·지표·표본),
240–254행(절제 실험), 255–272행 3·6 단계다 (행 번호는 **원본** 기준 — 사본은 머리 4 행만큼 밀린다). 센서 토픽·프레임은 [`sensor.md`](sensor.md)(s1 소유)를 따른다.

명세 원문 (`31_지능형_물류_AMR_시스템_구축.md` 5장, 329–332행, §5 보너스 — 선택 과제):

> 2. 3D LiDAR 기반 SLAM
>     - 3D LiDAR 센서를 추가하고 3D SLAM(LIO-SAM 등)을 적용한다.
>     - 2D SLAM 대비 맵 품질 및 위치 추정 정확도를 비교 분석한다.
>     - 3D 포인트 클라우드 처리 및 최적화 기법을 학습한다.

로그·산출물 경로(`/home/CAPYI_AI/...`)는 이 서버의 저장소 밖 디렉터리다 (공유 서버, 커밋하지 않음). 일회성 계산 스크립트는
`/home/CAPYI_AI/logs/agents/s2-eval/scripts/` 에 남겼다 (perfect map·대역 차이·원본 대조·규모 시험·변이 검사).
표기: **[측정]** 명령·로그 있음 · **[계산]** 오프라인 스크립트(경로 있음) · **[문서주장]** 근거 미확인 · **미측정**.
공유 서버라 시간 수치는 "공유 부하 하 예비값" 이다.

## 0. 먼저 알아야 할 불리한 사실

1. **3D 쪽은 아직 아무것도 재지 않았다.** 이 문서와 도구는 센서(s1)·SLAM(s3)이 준비되면 바로 재기 위한 것이다.
   아래 [계산] 값은 전부 현행 **2D** 지도(`maps/warehouse.yaml`, 2026-09-22 slam_toolbox)와 정답 기하에서 나왔다.
2. **0.05 m GT 로는 2D 와 3D 의 cm 단위 차이를 말할 수 없다.** 정답 면을 그대로 0.05 m 셀에 칠한 "완전한 지도"도
   0.05 m GT 대비 ADNN₀ 가 **2.74 cm** 다 [계산] — 현행 SLAM 지도의 2.59 cm 보다 **나쁘다**. 셀 양자화가 재려는
   값과 같은 크기라는 보고서의 경고(77–80행)가 수치로 확인됐다. 주 결과는 0.01 m GT 로 낸다 (§2).
3. **ADNN₀(map_quality 규칙)는 GT 해상도에 따라 값이 바뀐다.** 같은 지도가 0.05 GT 에서 2.59 cm, 0.01 GT 에서
   **1.43 cm** 다 [계산]. 고체 단면을 채운 GT 에서는 면 **뒤**(벽 속)에 떨어진 추정 셀이 거리 0 을 받기 때문이다
   (중앙값 0.0). GT 경계면까지 재는 `surface` 기준은 3.80 → 3.75 cm 로 해상도에 거의 무관하다. 주 지표는 `surface`.
4. **보고서의 "운용 대역(0.05–2.0 m)에는 랙 선반 돌출이 들어간다"는 이 월드에 맞지 않는다.** 선반(2.0 x 1.0)이 데크와
   발자국이 같아 돌출이 없다. 0.01 m 에서 비교 대역(0.05–0.40) = 0.20 m 단면 (차이 **0 셀**), 운용 대역이 더하는
   것은 두께 2 cm 표지판 **2 356 셀**(0.25 %)뿐이다 [계산]. 대역 재현율 차이로 "2D 가 못 보는 구조물"을 낼 수
   없으므로 S7 에는 돌출물을 월드에 따로 넣어야 한다 (§6).
5. **현실적인 S6(대기 AMR)은 2D 통로 축 구속을 없애지 못한다.** 토큰 요청 지점(입구 2.5 m 앞)에 선 AMR 한 대는 2D
   축 구속 점을 70 → 59 개로 줄일 뿐이다 (실제 정지점은 그보다 입구 쪽 — §6.1). 완전 퇴화(0 개)는 입구 자체를 막을 때만 생긴다 [계산, §6.2].
   보고서의 "대기 로봇이 통로 입구에 서서 2D 의 끝 시야를 가린다 → 가장 현실적인 실험"은 과장이다.
6. 보고서 1장 표의 원본 스크립트는 기본 인자(복셀 0.2 m)로 돌리면 **표와 다른 수**를 낸다. 표는 복셀 0
   으로 낸 값이다 (14 칸 전부 일치 확인, §6.2). 보고서 S5 의 "기존 8 개 목표"는 저장소 납치 시험
   (`tests/integration/test_05_kidnapped_robot.py` `TARGETS`)의 **5 개**와 다르다.

## 1. 정답(GT) 생성기 — 3 단계

`src/amr_simulation/worlds/gen_warehouse_world.py` 에 `--gt-dir` 플래그로만 켜지는 내보내기를 더했다. **기본 실행은
그대로**다: 인자 없이 또는 `out.sdf` 만 주면 전과 같은 `warehouse.sdf` 를 바이트 단위로 같게 쓴다
(`test_slam3d_gt.py::test_default_run_is_byte_identical`, 기존 `amr_simulation/test/test_world.py` 도 통과).
`--gt-dir` 만 주면 SDF 를 쓰지 않는다.

```bash
python3 src/amr_simulation/worlds/gen_warehouse_world.py --gt-dir /home/CAPYI_AI/runs/s2-eval/gt_full
#   --gt-spacing 0.01  --gt-res 0.01  --gt-slice-z 0.20  --gt-band 0.05 0.40 --gt-band 0.05 2.0  (기본값)
#   --gt-roi XMIN XMAX YMIN YMAX (점만 자름)  --gt-no-points  --gt-exclude PREFIX  --gt-legacy-exclude
```

| 파일 | 내용 |
| --- | --- |
| `visual_mesh.obj` | 정적 시각체 448 개의 상자·평면 메시 (g = 부류, o = 시각체 이름) |
| `primitives.json` | 같은 도형 목록: 부류, 종류, 크기, 월드 3x4 변환 — 해석적 거리·광선 추적용 |
| `surface_points.ply` | 시각체 **합집합** 표면 점, 간격 ≤ 1 cm, float32 x y z + uchar label |
| `slice_z0.200_r0.010.{pgm,yaml}` | 0.20 m 단면 래스터 (map_server 형식, 범위 x ±31, y ±21) |
| `band_z0.050-0.400_r0.010.*`, `band_z0.050-2.000_r0.010.*` | 비교용·운용용 높이 대역 래스터 |
| `dynamic_swept_r0.050.*` | 동적 장애물 경로 영역 (actor 꺾은선 ± 0.5 m, 차량 차선) — 유령 점 판정용 |
| `gt_meta.json` | 생성 조건, SDF sha256 과 커밋된 SDF 일치 여부, 부류별 도형·점 수 |

규약 (구현이 기대는 사실은 코드에서 다시 확인했다)

- **시각체**를 쓴다: gpu_lidar 는 렌더링 기반이다. 랙 충돌체는 통짜 2.0 x 1.0 x 2.5 이고 시각체는 데크 0.5 m·기둥 4 ·
  선반 2 (`models/rack/model.sdf`). 포즈 합성은 기존 `amr_localization/world_geometry.py` 와 같다
  (world ← include/model ← link ← visual). 상자 jitter 는 build_sdf 가 seed 를 매번 다시 걸어 커밋된 월드와 같다.
- **부류** 12 개: floor, ceiling, wall, rack, cargo(랙 위 정적 화물), pillar, plate(표지판·마커·문), fixture(충전기·
  조명), floor_paint(바닥 도색 패드), movable_cargo(도크 화물 — 밀릴 수 있다, 시작 자세 기준), vehicle, other.
- **제외**: 움직이는 모델 `forklift`, `shuttle_amr`, `worker`, `amr_` 접두어. 기존 world_geometry 는 shuttle_amr 를
  빼지 않아 **스폰 자세 그대로 GT 에 넣는다** (0.05 m 에서 96 셀). `--gt-legacy-exclude` 가 그 집합을 재현한다
  (단면 검증 전용 — 셔틀의 원기둥·구가 들어와 대역·점·메시에서는 빠진다).
- **합집합 표면**: 면마다 간격 ≤ spacing 셀 중심 격자로 표본하고, 점을 바깥 법선으로 0.1 mm 민 위치가 다른 고체
  안이면 버린다 (데크 위 상자 바닥, 상자 아래 데크 윗면, 데크에 묻힌 기둥 등). 바닥 평면은 아래 반공간, 천장(법선
  −z)은 위 반공간으로 가린다. 실내 센서가 볼 수 없는 외벽 바깥면·방 밖 점은 뺀다. 같은 평면에 겹친 두 면(랙 기둥
  바깥면과 데크 옆면)은 둘 다 남아 그 자리 밀도가 두 배다 (면적 0.08 x 0.5 m 조각들 — 영향 작음).
- **GT 관측 마스크는 만들지 않는다.** "주행 궤적에서 보이는 면만 남기기"는 GT 쪽을 깎는 별도 조작이고(재현율·COM
  분모가 바뀐다), Oxford Spires 식 "추정 점 중 GT 범위 밖 지우기"와도 다르다. 쓰면 결과 CSV `param` 에 이름을 남긴다.

생성 비용 [측정, 공유 부하 하 예비값]: 1 cm 전체 GT 벽시계는 부하에 따라 **8–43 s** 다. 이 문서의 이전 판은
"4.9 s, load ≈ 3" 이라고 적었는데 그 수를 뒷받침하는 로그가 없었다 (리뷰 B1) — 지웠다. 로그가 있는 실행만 적는다:

| 실행 | 벽시계 | loadavg (1 분값) | 로그 |
| --- | --- | --- | --- |
| 처음 생성 (HEAD 66ce091, `gt_full/PROVENANCE.txt`) | 28.98 s (`time` real) | **기록 없음** | `/home/CAPYI_AI/logs/agents/s2-eval/gt_full.log` |
| 리뷰어 재실행 (HEAD c91e8eb) | 42.7 s | 38.75 → 30.32 | 리뷰 `REVIEW.md` B1 (리뷰어 scratch `gt_review.log`) |
| 재측정 3 회 연속 (HEAD c6f1efc) | 23.4 / 9.1 / 8.1 s (python 프로세스) | 23.8 → 26.9 / 26.9 → 28.8 / 28.8 → 26.7 | `/home/CAPYI_AI/logs/agents/s2-eval/gt_full_timing_r2.log` |

첫 실행이 늦은 원인은 재지 않았다 (디스크 쓰기 900 MB 의 캐시 상태로 짐작만 한다). 재측정 출력 12 파일의 sha256 은
`gt_full/` 과 전부 같다 (같은 로그) — 결정적이다. 점 **69 340 691** 개 (바닥 22.0 M, 천장 24.0 M, 벽 15.9 M, 랙 3.9 M, …),
PLY 901 MB, 래스터 26 MB x 3. 산출물 `/home/CAPYI_AI/runs/s2-eval/gt_full/` (저장소에는 넣지 않는다 — 언제든 다시 만든다).

보고서와 다르게 한 것 (3 단계): ① 표면 표본과 3D 지표를 Open3D 대신 numpy·scipy(`cKDTree`)로 구현했다 — 작업 이미지
`amr-ai/rl:0.3` 에 Open3D 가 없고(`import open3d` → ModuleNotFoundError) 면별 셀 중심 표본은 numpy 로 충분하다.
② 3 단계 산출물의 "거울 자세 정합 점수 지도(선택)" 는 **만들지 않았다** (다음 물결, §10).

## 2. 생성기 검증 — 현행 지도의 ADNN₀ 2.59 cm 재현

**기준값의 출처**: `docs/algorithms/slam.md` §6.1 표 "ADNN_0 (정렬 전) 2.59 cm, Chamfer 3.67 cm, IoU/P/R 0.575 /
0.759 / 0.741" (지도 `maps/warehouse.yaml`, 2026-09-22). 과제 지시는 `docs/reports/` 에서 찾으라고 했지만 거기에는
ADNN 이 없다 — `grep 2.59 docs/reports` 가 잡는 `performance.md:261` 의 `2.594` 는 동적 회피의 `max_dev_first_m` 이라
**무관한 수**다. `performance.md:65` 의 지도 품질 99.55 % / 0.26 % 는 통합 시험 03 의 다른 지표다.

| 단계 | 결과 | 근거 |
| --- | --- | --- |
| 기존 CLI 를 지금 트리에서 다시 돌림 | ADNN₀ **0.025914** m, Chamfer 0.0367, IoU/P/R 0.575/0.759/0.741 — §6.1 그대로 | [계산] `/home/CAPYI_AI/logs/agents/s2-eval/baseline_map_quality.log` |
| 지도를 만든 시점(336abfd)의 월드로 | 위와 바이트 단위로 같은 JSON (그 뒤 더한 위치 표지 판은 0.05 m 셀 중심에 안 걸린다) | [계산] 같은 로그, `/home/CAPYI_AI/runs/s2-eval/mq_336abfd/` |
| 새 생성기 0.05 m 단면 (`--gt-legacy-exclude`) | world_geometry 래스터와 **셀 단위로 같다** (차이 0 / 1 041 600 셀) | 시험 `test_slice_matches_existing_evaluator_gt` |
| 그 GT 로 map_quality.evaluate | ADNN₀ 0.025914127536902674 (12 자리 일치), Chamfer 0.0367, IoU 0.575, R 0.741 | 시험 `test_adnn0_reproduces_existing_map_value` |

생성기는 기존 GT 를 재현한다. 다음은 같은 지도를 새 지표·새 GT 로 잰 값이다 [계산] (`slam3d_eval grid`, τ = 0.05 m,
관측 여유 0.10 m, 결과 `/home/CAPYI_AI/runs/s2-eval/validation/results.csv`, 표 `/home/CAPYI_AI/logs/agents/s2-eval/validation_table.txt`):

| GT | ADNN₀ filled (중앙) | ADNN₀ surface (중앙) | 정밀도 filled / surface | 재현율 filled / surface | F1 surface | IoU (공통 0.05) |
| --- | --- | --- | --- | --- | --- | --- |
| 0.05 단면, 기존 제외 | 2.59 (0.62) cm | 3.80 (4.38) cm | 0.759 / 0.717 | 0.741 / 0.990 | 0.832 | 0.697 |
| 0.05 단면, 새 제외 | 2.59 (0.62) | 3.80 (4.38) | 0.759 / 0.717 | 0.744 / 0.996 | 0.834 | 0.700 |
| **0.01 단면** | **1.43 (0.00)** | **3.75 (3.62)** | 0.880 / **0.705** | 0.603 / **0.994** | **0.825** | 0.710 |
| 0.01 대역 0.05–0.40 | 1.43 | 3.75 | 0.880 / 0.705 | 0.603 / 0.994 | 0.825 | 0.710 |
| 0.01 대역 0.05–2.0 | 1.40 | 3.77 | 0.886 / 0.705 | 0.604 / 0.994 | 0.825 | 0.713 |

양자화 바닥값 — 정답 면을 지나는 0.05 m 셀만 점유로 칠한 "완전한 지도"(원점은 실제 지도와 같게) [계산,
`/home/CAPYI_AI/logs/agents/s2-eval/perfect_map_floor.log`]:

| GT | ADNN₀ filled | ADNN₀ surface | 정밀도 | 재현율 surface / filled | F1 |
| --- | --- | --- | --- | --- | --- |
| 0.05 | 2.74 cm | 2.74 cm | 0.941 | 1.000 / 0.608 | 0.970 |
| 0.01 | 0.88 cm | **1.28 cm** | 1.000 | 1.000 / **0.393** | 1.000 |

읽는 법

- 0.01 GT 에서 완전한 지도의 surface ADNN₀ 바닥은 1.28 cm, 현행 2D 지도는 3.75 cm → **지도 오차의 몫 ≈ 2.5 cm**.
  0.05 GT 에서는 완전한 지도(2.74)가 실제 지도(2.59)보다 나빠 이 구분이 불가능하다.
- `filled` 재현율은 0.01 GT 에서 쓰지 않는다: 관측 여유 0.10 m 가 벽·데크 **내부** 셀을 분모에 넣어 완전한 지도도
  0.39 다. `surface` 재현율은 추정 **자유** 셀에서 0.10 m 안의 GT 경계 셀만 센다 (벽 뒷면은 안 들어간다).
- 셀 중심 규약: 거리는 GT **셀 중심**까지라 0.01 GT 에서 ±0.5 cm, 0.05 추정 지도에서 셀 중심이 면에서 최대 2.5 cm
  떨어진다. slam.md §3 의 띠 중앙값 편향(면 뒤 +0.7~+3.4 cm)은 이 지표들이 그대로 받는다.

## 3. 높이 대역 — 보고서 해석의 수정

보고서 167–171행: 비교용 0.05–0.40 m, 운용용 0.05–2.0 m, "두 대역의 재현율 차이를 내면 2D 가 못 보는 구조물의 비율이
수치로 나온다". 이 월드에서는 [계산] (`/home/CAPYI_AI/logs/agents/s2-eval/band_diff.log`):

- 0.01 m 래스터 셀 수: 단면 0.20 m **956 252** = 대역 0.05–0.40 **956 252** (차이 0), 대역 0.05–2.0 **958 608**.
- 운용 대역이 더하는 2 356 셀은 **전부 표지판 판**(두께 0.02 m, 벽·랙 끝·기둥 면 앞 2 mm)이다.
- 이유: 랙 선반 2.0 x 1.0 = 데크 발자국, 기둥 안전띠(0.52, z < 1)가 기둥(0.50)보다 넓어 0.20 단면이 이미 최대 발자국,
  충전기 몸체는 높이 내내 같은 단면.

결론: 대역 재현율 차이는 이 월드에서 ≈ 0 이 **설계상 정해져 있다**. 3D 의 "스캔 평면 위 구조" 이점을 보이려면 S7 의
돌출물을 월드 변형으로 넣어야 한다. 이 사실은 `test_band_contains_slice_and_world_has_no_overhang` 이 고정한다 —
월드에 돌출물이 생기면 시험이 깨지고 이 절을 고친다.

## 4. 지표 정의 — 6 단계 도구

코드: `src/amr_evaluation/amr_evaluation/slam3d/` (rclpy 비의존), CLI `ros2 run amr_evaluation slam3d_eval {traj,grid,cloud}`
(또는 `python3 -m amr_evaluation.slam3d.cli`). 시험: `test/test_slam3d_{metrics,gt,localizability,evo_crosscheck}.py`.

### 4.1 궤적 (`se2.py`) — 주 결과는 정렬 없는 SE(2)

| 지표 | 정의 |
| --- | --- |
| 연관 | 추정 시각마다 GT 를 선형 보간 (x, y 선형, yaw 는 감아서 선형). GT 범위 밖·둘러싼 GT 표본 간격 > `max_gap`(0.1 s) 은 버린다 (외삽 없음). GT 는 `/<robot>/ground_truth/odom` (월드 절대 자세, `amr_gazebo.xacro:184`) |
| ATE | e_i = ‖p̂_i − p_i‖, Δψ_i = wrap(ψ̂_i − ψ_i). 정렬 **none(주)** / origin(첫 자세 맞춤 = evo `--align_origin`) / se2(2D Umeyama, 축척 없음). 3D 추정기도 (x, y, yaw) 로 내려 같은 정렬 클래스로 잰다 |
| RPE | 거리 Δ ∈ {1, 2, 5, 10, 20} m. 쌍 (i, j): **GT 경로** 길이 s_j − s_i ≥ Δ 인 첫 j. E = (Q_i⁻¹Q_j)⁻¹(P_i⁻¹P_j), 병진 ‖t(E)‖ (+ Δ 대비 %), 회전 |∠E|. 기본은 모든 시작점, `--consecutive` 는 겹치지 않는 연속 구간 |
| RTE | 시간 창 10 s (Potokar & Kaess) — 쌍 정의만 시간으로 |
| 명세 구간 | GT 속도로 정지/직선/회전 (`segments.classify_motion` 임계 그대로) 나눠 RMSE·max·p95, 3 / 5 / 8 cm 판정을 RMSE·max **둘 다** 기록 |
| 3D 진단 | z·roll·pitch 오차는 주 지표에 넣지 않고 따로 (`z_rp_errors`) |

evo 교차 확인 [계산] (`test_slam3d_evo_crosscheck.py`, amr-ai/slam:0.1 의 evo 1.37.1, 8 통과,
`/home/CAPYI_AI/logs/agents/s2-eval/evo_crosscheck.log`): ATE none / origin / SE(2) Umeyama 와 연속 쌍 RPE(Δ = 1, 5, 5.013, 10.007 m)가
**1e-12 안에서 같다**. 두 가지 함정을 확인했다:
1. `evo_rpe` 는 기본으로 **추정** 궤적 경로로 쌍을 고른다. `--pairs_from_reference` 를 주지 않으면 값이 다르다
   (예: Δ = 1 m 병진 RMSE 0.0203 vs 0.0221 m, Δ = 10 m 회전 RMSE 0.117 vs 0.171°).
2. `evo.main_rpe.rpe()` 는 넘긴 궤적을 `delta_ids` 로 제자리에서 줄인다 — 같은 객체로 두 번 부르면 두 번째 값이 틀린다.
evo 의 `--all_pairs` 는 "Δ 에 가장 가까운 j (허용오차 안)" 라 우리 "첫 ≥ Δ" 와 정의가 다르다 (비교하지 않았다).

### 4.2 2D 지도 (`gridmap.py`) — 같은 격자 공간

2D SLAM 지도와 3D 지도의 높이 대역 투영(RTAB-Map `/map` → `map_saver_cli`)을 같은 GT 래스터에 올린다. 정렬 없음.

| 지표 | 정의 |
| --- | --- |
| ADNN₀ filled | 추정 점유 셀 중심 → GT 점유 **전체** 거리장 (map_quality ADNN₀ 와 같은 값 — 호환용) |
| **ADNN₀ surface** | 추정 점유 셀 중심 → GT **경계 셀**(8 이웃에 비점유) 거리장 — 면 뒤로 밀린 셀도 거리를 받는다 (주 지표) |
| 정밀도@τ | 위 거리 ≤ τ 비율 (filled / surface 각각). τ 기본 = 추정 해상도 |
| **재현율@τ** | 관측된 GT 경계 셀(추정 **자유** 셀에서 0.10 m 안) 중 최근접 추정 점유 셀 중심이 τ 안인 비율. filled 판(map_quality 규칙: 알려진 셀에서 0.10 m 안의 GT 점유 전체)도 기록 |
| F1@τ | 정밀도 surface 와 재현율 surface 의 조화평균 |
| Chamfer₀ | ½(ADNN₀ + GT→추정 평균) — filled / surface 각각 |
| IoU_τ | 공통 격자(기본 0.05 m, GT 범위)로 셀 중심 풀링 뒤 map_quality 규칙(τ = 1 셀) |

정렬 후 지표(map_quality 의 SE(2) soft-L1)는 보조로만 — 우리 계약은 map = world 이고 정렬은 결과를 숨긴다.

### 4.3 3D 점군 (`cloud.py`)

| 지표 | 정의 (출처) |
| --- | --- |
| 정밀도·재현율·**F-score@5·10 cm** | d_e→g ≤ τ 비율, d_g→e ≤ τ 비율, 조화평균 (Oxford Spires, 주 지표) |
| AC@τ | τ 안 추정 점의 d_e→g 평균 (MapEval "임계 안 대응점의 오차") |
| COM@τ | = 재현율@τ. MapEval 의 \|C_τ\|/N_g 를 GT→추정 방향으로 읽었다 **[해석]** |
| Chamfer | sum = mean(d_e→g) + mean(d_g→e) (MapEval CD), half = ½ sum (map_quality 규약) — 둘 다 이름을 붙여 낸다 |
| 유령 점 비율 | 정적 GT 에서 τ 보다 먼 추정 점 중 `dynamic_swept` 영역 안의 비율 (분모 = 전체 추정 점), 영역 무관 "설명 안 되는 점" 비율도 |
| SA / DA / AA | DynamicMap: 라벨 붙은 원시 누적 점 중 정적 점 보존율 / 동적 점 제거율 / √(SA·DA). "남았다" = match_dist 안에 출력 점 |

GT 부류는 `--gt-labels` 로 고른다 (바닥·천장 포함 여부를 결과 `param` 에 남긴다). 밀도가 다른 지도는 `--voxel` 로 맞춘다.
**유령 점 비율은 바닥을 포함한 GT 로 재거나, 추정 점을 바닥 위(예: z > 0.05 m)로 자른 뒤 잰다** — GT 에서 바닥을 빼고 추정
바닥 점을 남기면 그 점이 "설명 안 됨" 이 되고, 동적 영역 안이면 유령으로 세어진다 (리뷰 N5). S6 에서는 블로커 점도 정적 GT 에
없으므로 블로커 발자국을 지표에서 가린다 (§6.1).

규모 확인 [측정, 공유 부하 하 예비값]: 바닥·천장·도색을 뺀 1 cm GT **21 854 623** 점 대 합성 추정
(GT 를 5 cm 복셀로 솎고 σ 2 cm 등방 잡음, 855 164 점). 처음 실행은 지표 계산 22.8 s, 최대 RSS 3.75 GB 인데 loadavg 를
남기지 않았다 (`cloud_scale.log` — 이전 판의 "load 43–55" 는 로그에 없는 수라 지웠다, 리뷰 B1). 재측정(HEAD c6f1efc)은
지표 계산 **16.8 s**, 전체 36.7 s, 최대 RSS **3.74 GB**, loadavg 26.73 → 27.14 (`/home/CAPYI_AI/logs/agents/s2-eval/
cloud_scale_r2.log`; 그 사이 같은 에이전트의 짧은 pytest 가 겹쳤다 — 로그 끝 메모). 지표 값 16 개는 두 실행이 같다.
정밀도@5 cm 0.988 은 평면 잡음의 닫힌 식 1 − 2Φ(−2.5) = 0.9876 과, AC 1.61 cm·정확도 평균 1.66 cm 는 σ√(2/π) = 1.60 cm
와 맞는다 [계산].
MME 는 같은 도구·반경·센서 밀도의 3D 후보끼리만 (보고서 230행) — 이 도구에는 넣지 않았다.

### 4.4 결과 CSV (`results.py`)

긴 형식: 한 행 = 실행 하나 · 지표 하나 · 통계 하나. 열 26 개 (`results.COLUMNS`):
`schema, run_id, scenario(S1–S7|MAP|SYN), bag_id, seed, replay, row, stage(odometry|slam|localization|map),
factor_sensor(2d|3d), factor_wheel_prior(on|off), factor_dof(se2|se3), factor_wheel_quality(nominal|degraded),
factor_aisle_end(open|blocked), metric, param, alignment(none|origin|se2|n/a), segment, stat, value, unit, n, failed,
git_head, gt_sha256, tool, notes`. 실패 실행도 행을 남기고(`failed=1`, value NaN 가능) 평균에서 빼지 않는다.
표본이 없는 통계는 n = 0, NaN. 짝지은 검정의 짝 = 같은 `bag_id`·`replay`.

## 5. 요인 설계 (보고서 201–215행)

| 요인 | 수준 | 구현 |
| --- | --- | --- |
| A 센서 | 2D (+0.20 m, 720 빔, 25 m) / 3D (+0.90 m, 16 x 1800, 30 m) | 같은 bag, RTAB-Map `icp_odometry` 에 scan 또는 cloud |
| B 휠 사전값 | 휠-EKF guess 있음 / 없음 | `guess_frame_id` 유무 |
| C 자유도 | SE(2) / SE(3) | `Reg/Force3DoF` |
| D 휠 품질 | 현행(사실상 오라클) / 열화(RPE 1–3 %) | `wheel_odometry.yaml` 의 `left/right_wheel_radius`(1 % 어긋남), `slip_distance_coeff` > 0, `wheel_param_rel_stddev` > 0 (현행 0.0 / 0.0 — 코드에서 확인) |
| E 통로 끝 | 열림 / 막힘 | S6 변형 (§6) |

묶음: ① A×B (D 현행, E 열림) — "휠을 같이 주면 2D 와 3D 가 거의 같다" 확인, ② A×B (D 열화), ③ A×B (E 막힘).
고정 행: 현행 2D 스택(slam_toolbox → AMCL + 이중 EKF + 정합), LIO-SAM(휠 없음, madgwick 방위, 루프 on/off),
정답 IMU 방위 **오라클 행**(따로 보고), 3D 사거리를 25 m 로 자른 행. 오도메트리 단계(루프 off)와 SLAM 단계(루프 on)를
둘 다 보고 (`stage`).

## 6. 시나리오 S1–S7

매핑에는 slam.md §6.1 의 322 m GT 되먹임 경로를 쓴다 (`MAP`). "씨드" 는 월드를 바꾸지 않는 **출발 지연**으로 정의한다:
actor·차량은 sim 시간으로 결정적으로 돌므로, 기록 시작(경로 출발) 시각을 씨드 난수로 0–120 s 에서 뽑으면 동적 장애물과의
만남이 달라진다. 생성기 `SEED` 를 바꾸는 방식은 **정적 화물 자세까지 바뀌어** 씨드마다 GT 를 따로 만들어야 하므로 쓰지 않는다.

변형 월드 적재 확인 [측정]: 리뷰 B2 반영 뒤(HEAD c6f1efc) 기본·s1·s6-open·s6-queue·s6-queue2·s6-wall 여섯 SDF 를
Fortress 서버(헤드리스 렌더링, `--iterations 2000`)로 띄워 모두 rc 0, `[Err]`·`[Wrn]` 0 줄, 각 6.7–7.6 s, SDF 의 actor 6 / 0 /
4 / 4 / 4 / 4, 블로커 0 / 0 / 0 / 1 / 2 / 2 (`/home/CAPYI_AI/logs/agents/s2-eval/variant_gz_load_r2.log`,
`/home/CAPYI_AI/runs/s2-eval/variants_c6f1efc/`, `gz_slot.sh` 슬롯 1 07:30:02–07:30:46, loadavg 26.3–30.4). 이전 판
(f1e7b3f, 다섯 SDF) 기록은 `variants/` 에 남겼다 — 그 s6-* 월드는 사람이 블로커를 지나는 판이다. 런타임에 블로커 엔티티
목록을 조회하지는 않았고, actor 가 실제로 블로커를 피하는지는 Gazebo 에서 보지 않았다 (생성 시점 기하 검사만).

| 시나리오 | 정의 | 지금 가능한가 |
| --- | --- | --- |
| S1 정상 | 동적 장애물 정지 — actor·차량 없는 월드 변형 | 된다: `gen_warehouse_world.py --variant s1 <out.sdf>` (지게차·셔틀 odom 브리지 토픽은 비게 된다) |
| S2 동적 | 현행 월드, 씨드(출발 지연)가 다른 bag ≥ 3 | 된다 (센서·기록 준비 후) |
| S3 좁은 통로 왕복 | (0, −5.5) → 통로 → (0, −14.5) → 되돌아옴, 통로 중앙 (0, −10) 에서 15 s 정지. 오차를 통로 축(y)·횡(x)으로 분해 (`segment=aisle_axis/aisle_lateral`) | 된다 |
| S4 점대칭 별칭 | 스폰 5 자세(`fleet_spawn.yaml`: (18…26, −16, π/2))와 그 거울 자세 (−x, −y, yaw+π) 근처에서 전역 위치추정 시작 | 된다 |
| S5 납치 | `test_05_kidnapped_robot.py` 의 목표 **5 개** + 각 목표의 거울 자세 | 된다 |
| S6 통로 끝 막힘 | 아래 변형 (요인 E) | 국소화 계산은 됐다 [계산]. 월드: `--variant s6-open / s6-queue / s6-queue2 / s6-wall` (정적 블로커, actor 4 명 — §6.1). 비교 기준은 **s6-open** (기본 월드 아님). follow 는 움직이는 AMR 이라 정적 변형이 없다 — 두 번째 로봇으로 |
| S7 돌출 장애물 | 0.3–1.0 m 높이로 통로에 튀어나온 물체 (Nav2 폐루프) | **아니다**: 월드에 돌출물이 없다 (§3) — 변형 필요 |

### 6.1 S6 변형 정의 (`localizability.s6_blockers`)

좁은 통로: 중심 (0, −10), y −12 ~ −8, 순폭 0.60 m. 교통 관리자는 1 차선 통로 토큰을 입구 `corridor_approach_m` 2.5 m
앞에서 요청하고 못 받으면 멈춰 기다린다 (`amr_fleet/config/traffic.yaml:35`). (0, −5.5) 는 이 **요청** 지점이다 — 실제
정지점은 확인 주기·지연·제동만큼 입구 쪽으로 더 들어온다 (같은 파일 34행 주석의 1 m/s 계산으로 최대 1.1 m). 정지점 쪽
감도는 계산하지 않았다 (§10). 같은 방향 추종은 허용 (`same_direction: true`).

| 변형 | 블로커 | 현실성 |
| --- | --- | --- |
| open | 없음 | 기준 (월드 `s6-open`) |
| queue | AMR 1 대 (0.6 x 0.4 x 0.30) 북쪽 요청 지점 (0, −5.5), 통로를 향함 | 대기 위치의 근사 (위 문단) |
| queue2 | 양끝 대기 (0, −5.5) + (0, −14.5) | 실제로 생긴다 (양방향 수요) |
| follow | 같은 방향 추종 AMR 이 대상 1.0 m 뒤 (0, −11.0) | 실제로 생긴다 |
| wall | 양끝 입구 바로 앞을 0.8 x 0.3 x 0.5 m 화물로 완전 차단 | 최악 (보고서의 "4 m 사거리 상한" 대용과 같은 성격) |

**S6 월드의 사람 actor (리뷰 B2 로 고침).** Fortress 의 actor 는 정적 모델과 충돌하지 않는다. 이전 판에서는 기본 월드의
actor 6 명을 그대로 두어 `worker_slow_south` 가 남쪽 대기 블로커 (0, −14.5) 를 **그대로 지났고** (가장자리까지 0.00 m, 루프
t ≈ 19.5 s), `worker_random` 은 북쪽 블로커 가장자리에서 0.20 m 를 지났다 [계산, `/home/CAPYI_AI/logs/agents/s2-eval/
s6_actor_clearance.log`]. 그 월드로 녹화하면 사람과 상자가 겹친 점이 스캔에 들어간다. 지금은:

- `check_blockers` 가 블로커를 그 월드에 남는 actor 의 웨이포인트 꺾은선(선분–상자 거리)과 차량 왕복 차선에서
  `BLOCKER_CLEARANCE` = 0.5 m(= `GT_DYN_MARGIN`, 사람 반폭 + 팔 흔들림) 이상 떨어지게 강제한다. 어기면 생성이 멈춘다.
- s6-* 네 변형은 모두 `S6_DROP_ACTORS` (`worker_random`, `worker_slow_south`) 를 뺀 **같은 actor 4 명**을 쓴다. 변형마다 다르게
  빼면 막힘(요인 E)과 사람 수가 함께 바뀌기 때문이다. 그래서 요인 E 의 "열림" 수준은 기본 월드가 아니라 **`s6-open`**
  (블로커 없음, actor 4 명)이다. 남은 4 명과 블로커의 최소 거리는 0.60 m (`worker_crossing` 대 s6-wall 북쪽 화물).
- 시험: 출력 SDF 에서 읽은 actor 경로·블로커로 독립 거리 확인, 기본 actor 전부를 두면 거절되는 회귀, 여유 경계 ±1 cm,
  선분–상자 거리 해석값 (`test_slam3d_gt.py`). 변이 8 종 모두 잡힘 (`/home/CAPYI_AI/logs/agents/s2-eval/mutation_b2.log`).
- 지표: GT(`--gt-dir`)는 언제나 기본 월드라 `dynamic_swept` 래스터에 빠진 두 actor 의 경로가 남는다 — 유령 점 판정에는
  더 넓은 쪽이라 해가 없지만, 블로커 발자국이 그 경로와 겹친다(남쪽 대기). 규칙: S6 결과에서는 블로커 발자국(여유
  0.10 m)을 지표에서 가리고 `param` 에 적는다. **`slam3d_eval` 에는 아직 그 옵션이 없다** (§10). 변형 actor 만의 래스터는
  `gt_dynamic_raster(build_sdf(variant)[2])` 로 만들 수 있다.

### 6.2 S6 단일 스캔 localizability [계산]

도구 `python3 -m amr_evaluation.slam3d.localizability --gt-dir <GT>` (원본 연구 스크립트의 이식). 원본의 단순 기하를 넣으면
보고서 1장 표 14 칸(축 구속 점·λmin/λmax, 사거리 무제한·4 m)을 **전부 그대로** 낸다 (`/home/CAPYI_AI/runs/s2-eval/localizability/
port_equivalence.txt`, 원본 출력 `original_localizability2_voxel0.txt`). 원본 기본 인자(복셀 0.2)는 표와 다른 수를 낸다
(`original_localizability2.txt`). 아래는 **정답 생성기 기하**(상자 jitter·표지판·마커·충전기 포함), 위치 (0, −10),
통로 방향, 복셀 0, σ 0.03 m, 블로커 점은 지도에 없으므로 정보에서 뺀 값 (`gt_s6_outlier.csv`):

| 변형 | 2D 720 @0.20: 축 구속 점 / λmin/λmax | 16ch ±15° @0.85 | 16ch @0.90 1800 열 (권장안) |
| --- | --- | --- | --- |
| open | 70 / 0.108 | 4 540 / 0.677 | 7 927 / 0.725 |
| queue | 59 / 0.091 | 4 527 / 0.675 | 7 902 / 0.722 |
| queue2 | 48 / 0.074 | 4 527 / 0.675 | 7 902 / 0.722 |
| follow | 35 / 0.056 | 4 540 / 0.677 | 7 927 / 0.725 |
| wall | **0 / ≈ 0 (퇴화)** | 4 540 / 0.677 | 7 927 / 0.725 |
| (사거리 4 m 상한, open) | 0 / ≈ 0 | 1 304 / 0.220 | 2 647 / 0.252 |

- 정답 기하에서도 보고서의 순위는 유지된다 (2D ≪ 3D, 3D 장착 높이 0.85–1.2 m 가 0.45 m 보다 낫다, 15° 숙임은 나쁘다).
  절대값은 바뀐다 (예: 16ch @0.85 열림 4 839 / 0.75 → 4 540 / 0.68).
- **현실적인 대기(queue)는 2D 를 퇴화시키지 않는다** (−16 %, 양끝 −31 %). 추종(follow) −50 %. 완전 퇴화는 입구 자체를
  막을 때(wall)만. 그래서 S6 은 **queue2·follow(현실)와 wall(최악)을 둘 다** 돌리고, 현실 변형에서 차이가 안 보이면
  "현실 조건에서는 차이 없음"으로 정직하게 보고한다.
- 블로커를 정지한 동안의 스캔 대 스캔 정합처럼 구속으로 세면(`--blockers-as landmark`) 2D 도 줄지 않는다
  (블로커 앞면이 축을 구속한다) — 사전 지도 위치추정과 오도메트리에서 S6 의 의미가 다르다.
- 완전 대응을 가정한 하한이라 **순위 비교용**이다. 실제 gpu_lidar 출력과 정합기 동작으로 확인하기 전이다.

## 7. 표본 수와 검정 (보고서 232–239행)

- 시나리오마다 씨드가 다른 bag 3–5 개 x bag 당 재생 3–5 회 = 구성당 10–25 개. 재생 간 분산(알고리즘·ROS 비결정성)과
  bag 간 분산(장면)을 따로 보고한다.
- 표본 수: 짝지은 차이 표준편차 σ_d = 0.5 cm 에서 δ = 0.5 cm 를 α = 0.05, 검정력 0.8 로 잡으려면
  n ≈ ((1.96 + 0.84) σ_d / δ)² = 7.8, t 보정으로 약 10. **예비 5 회로 σ_d 를 재고 다시 계산한다.** 위 표에서 2D 지도의
  양자화 바닥이 1.28 cm 이므로 지도 지표의 δ 는 그보다 작게 잡으면 의미가 없다.
- 먼저 같은 bag·같은 구성을 3 회 돌려 결과 산포("재현 잡음")를 재고, 그것을 차이 판정 문턱으로 쓴다.
- 검정: 같은 bag 끼리 짝지은 Wilcoxon 부호순위 (정규성 확인 시 짝 t), 부트스트랩 95 % CI, 효과크기. 지표·시나리오가
  여럿이므로 Holm 보정. 실패율·별칭률은 Wilson 95 % CI. **사전 등록한 비교만 검정한다** (이 문서의 묶음 ①②③ + S6 변형).

## 8. 재생 통제 규칙 (보고서 232–234행, 일부 [문서주장])

1. 한 번 기록하고 여러 번 재생한다. bag: 2D 스캔, 3D 점군(전처리 전), 원시 IMU, 휠 odom, EKF odom,
   `ground_truth/odom`, TF. Nav2 폐루프 결과(S7)는 재생할 수 없으므로 따로.
2. 시간원은 하나: 녹화 bag 의 `/clock` 은 재생에서 빼고 `ros2 bag play --clock 200` 으로 낸다. Humble 의 `--clock` 은
   기본으로 발행하지 않고, 값 없이 주면 40 Hz 다 [문서주장 — 보고서·검증 노트, 이 저장소에서 미확인].
3. 정확도 실행은 `--rate 0.5` 이하. 노드별 수신·필터 통과·처리 스캔 수를 세어 **누락 0** 을 확인한다. slam_toolbox 는
   `scan_queue_size` 를 키운다 (Humble 기본 1 [문서주장]).
4. 자원 실행은 `--rate 1.0`, `--cpuset-cpus` 로 코어 고정, 구성 순서를 ABAB 로 섞는다. 공유 서버라 DVFS 를 끌 수 없다는
   한계를 적는다. RTF·시간은 loadavg 와 함께 "공유 부하 하 예비값".
5. 실행마다 로그 디렉터리에 `git rev-parse HEAD`, `git diff --stat`, GT 의 `gt_meta.json` sdf_sha256, 이미지 태그, 씨드를
   직접 쓴다 (`run_integration.sh` 는 HEAD.txt 를 쓰지 않는다 — handover §4). 태그는 새로 만든다.
6. evo 를 쓸 때는 `--pairs_from_reference` 를 반드시 준다 (§4.1). 정렬을 주면 결과 CSV `alignment` 에 남긴다.

## 9. 주장 가능 범위 (보고서 291–303행 + 이번 계산)

| 주장 | 가능 여부 | 조건 |
| --- | --- | --- |
| 3D LiDAR 를 추가하고 3D SLAM 을 적용했다 | 측정 후 가능 | 판·설정·센서 사양 공개 |
| 같은 입력에서 2D 대비 지도 품질·위치 정확도를 비교했다 | 측정 후 가능 | **0.01 GT**, 정렬 없음, surface 기준, 휠 사전값·IMU 방위 처리를 조건표에 |
| 0.05 m GT 로 2D 와 3D 지도의 cm 차이를 판정했다 | **불가** | 완전한 지도도 2.74 cm (§2) |
| 3D 는 스캔 평면 위 구조를 지도에 담는다 | 이 월드에서는 **표지판뿐** | S7 돌출물 변형 없이는 대역 차이 ≈ 0 (§3) |
| 막힌 통로 끝에서 3D 가 통로 축을 더 잘 구속한다 | 측정 후 가능 | S6 wall 대 **s6-open** 결과 + §6.2 계산. 현실 대기(queue)에서는 2D 도 구속이 남는다 |
| 정상 조건에서 2D 와 3D 정확도가 비슷하다 | 측정 후 가능 | 짝지은 검정과 CI — "차이가 x 이하로 검출되지 않음" |
| 3D 가 일반적으로 더 정확하다 / 별칭을 푼다 / deskew 효과 / 5 대 동시 운용 | 불가 | 보고서 표 그대로 |

## 10. 남은 일 (다음 물결)

- S7 돌출물 월드 변형: 높이를 정해야 한다 — 2D 스캔 평면 0.20 m 위, 빈 차체 상단 0.33 m(화물 실으면 0.73 m) 아래에
  걸쳐야 2D 는 못 보고 로봇에는 부딪힌다. 0.90 m 의 16 채널 아래 빔(−15°)은 높이 0.25 m 를 수평 2.4 m 밖에서만 본다.
  S1·S6 변형(`--variant`)은 넣었다 — 기본 출력은 바이트 동일, GT(`--gt-dir`)는 변형과 무관하게 언제나 기본 월드.
- S6 블로커가 Gazebo 에서 실제로 gpu_lidar 에 어떻게 보이는지 (2 단계 센서 오차 측정과 함께). actor 가 블로커와 겹치지
  않는다는 것도 지금은 생성 시점 기하 검사뿐이다 — 녹화 전에 한 번 화면이나 스캔으로 본다.
- S6 queue 의 실제 정지점 감도: 요청 지점 (0, −5.5) 대신 (0, −5.0)·(0, −4.4) 로 §6.2 를 다시 계산 (리뷰 N4).
- 거울 자세 정합 점수 지도 (보고서 3 단계 선택 산출물) — 만들지 않았다.
- `slam3d_eval grid`·`cloud` 에 영역 가림 옵션(S6 블로커 발자국)을 더할 것 (§6.1).
- `cli traj --align se2` 에서 연관 표본이 2 개보다 적으면 `umeyama_se2` 가 ValueError 로 멈춘다 — `failed=1` 행을 남기게
  고칠 것 (리뷰 N6). `observed_mask` 의 0.01 GT 전역 좌표 배열(호출당 수백 MB)도 줄일 것 (리뷰 N11).
- 3D 지도 → 2D 대역 투영 경로(RTAB-Map `Grid/MaxObstacleHeight`)의 결과를 이 도구로 처음 재는 것 (s3 결과 대기).
- `docs/research/README.md` 색인에 이 폴더를 넣는 것은 그 파일 소유자 몫이다.
