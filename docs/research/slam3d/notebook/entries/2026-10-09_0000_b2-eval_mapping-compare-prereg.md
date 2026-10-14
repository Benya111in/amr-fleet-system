# 보너스 2 P 단계 E2·E3(a)·E5(매핑) — 8 bag × 6 행 × 재생 3 매핑 비교 (사전 등록)

- 시각: 2026-10-09 00:00 KST · 에이전트: b2-eval · 단계: P-E2 (`bonus2_completion_plan.md` §4 P 재생·평가)
- 코드: 사전 등록 커밋 = 이 항목을 넣은 커밋 (브랜치 `feature/b2-eval`, 분기 `feature/bonus2-slam3d` `3d0a17d`).
  분석·구동 스크립트 sha256 (이 커밋의 파일):
  - `src/amr_slam3d/scripts/peval.py` `6de70f13fa0ff4f33e9b4bfcd3ced05882a712a6f8f15bf563ed4dd141c6d3fa`
  - `src/amr_slam3d/scripts/eval_driver.py` `b207778c7816a5b13b4de12c58dcc3f8e006d2e6d5f5c3536542380f2d4683a4` (686d902 판 `83378c18…` 에서 실행 전에 바꿈 — 재생을 git archive
    스냅샷(`SNAPSHOT_HEAD`)에서 돌리는 기능과 작업별 사본(`wt`)만 더했다. 결과를 보기 전, 재생 0 건일 때)
  - `src/amr_evaluation/amr_evaluation/slam3d/pstats.py` `58c37a2e1846735eaac447b774443a4df6ee34deb56064bb3df310c52249d048`
  - `src/amr_evaluation/amr_evaluation/slam3d/pmetrics.py` `916666f0a4223e1f121418adb24c111629cc5a30fbe59bcba05831a90ac0d6f0`
- 관련 실행 id: `B2eval-e2-<bag>-<행>-r<k>` (재생 144), `B2eval-validate-pilot` (분석 도구 검증), `B2eval-test-*`
- 근거: `bonus2_completion_plan.md` §3 E2·E3·E5, §6 2026-10-08 17시 결정 (S1 = 3 bag 전체와 사전 등록 2 bag 만을 둘 다),
  `recordings.md` §0·§3·§4, `replay.md` §3–§6, `evaluation.md` §4·§6·§7, `rtabmap.md` §4, 보고서 「지표」 절.

## 질문

1. (E2) 같은 녹화를 여러 번 재생할 때, 3D 행(RTAB-Map 3D·LIO-SAM 평면)의 지도가 2D 행(slam_toolbox·RTAB-Map 2D)보다
   같은 2D 격자 공간에서 정답(0.01 m)에 더 가까운가 — bag 8 개(장면 다양성)에 걸쳐 차이와 그 불확실성은?
2. (E3(a)) 같은 재생의 매핑 궤적에서, 정렬 없는 SE(2) ATE·RPE 가 3D 와 2D 사이에 다른가?
3. (E5 매핑 쪽) 통로 끝 막힘(s6_wall·s6_queue)이 열림(s6_open) 대비 통로 축 오차를 2D 행에서 3D 행보다 더 키우는가?
4. (E1 일부) RTAB-Map 3D·LIO-SAM(평면·6DoF)이 8 bag 모두에서 끝까지 도는가, 처리 스캔 누락은?

## 가설과 예측 (결과 보기 전에 적는다)

근거는 파일럿 bag 하나(`replay.md` §3: xy RMSE RTAB-Map 3D 0.77 · LIO-SAM 평면 1.71 · slam_toolbox 1.87 · RTAB-Map 2D 2.15 cm,
지도 ADNN₀ surface RTAB-Map 3D 4.28 · RTAB-Map 2D 4.02 · slam_toolbox 4.62 cm — 이 항목의 도구 검증 `B2eval-validate-pilot`)뿐이다.

- **P-C1** (RTAB-Map 3D − RTAB-Map 2D, xy ATE RMSE): 8 bag 모두 음수 → 판정 `A<B`. 확신 중간 — S6(5.6 m, 통로 안) 은 두 행
  모두 1 cm 안팎이라 bag 차가 δ(0.2 cm)보다 작을 수 있다.
- **P-C2** (RTAB-Map 3D − slam_toolbox, ATE): `A<B`. 확신 중간 (slam_toolbox 는 재생 사건 산포가 크다).
- **P-C3** (LIO-SAM 평면 − slam_toolbox, ATE): `undecided` (파일럿에서 범위가 겹쳤다).
- **P-C4** (RTAB-Map 3D − RTAB-Map 2D, 지도 ADNN₀ surface): `undecided` (파일럿 3D 가 0.26 cm 나빴다 — 방향 예측 없음).
- **P-C5** (RTAB-Map 3D − slam_toolbox, ADNN₀ surface): `undecided`.
- **P-E5**: 매핑에서는 블로커가 정지 중 스캔 대 스캔 정합의 랜드마크가 되므로(evaluation.md §6.2 `--blockers-as landmark`)
  2D 도 축 구속을 잃지 않는다 → 모든 행에서 wall − open 의 통로 축 RMSE 차가 판정 문턱(δ 0.2 cm, 2 × 재생 sd) 아래
  (`undecided`) 이고, 2D 와 3D 의 차의 차도 δ 아래. **가설("3D 가 막힌 통로 끝에서 축을 더 잘 구속")은 매핑 쪽에서는
  지지되지 않을 것으로 예측한다.** 위치추정(사전 지도) 쪽 E5 는 E3(b) 항목에서 따로 사전 등록한다.
- **P-E1**: RTAB-Map 3D·LIO-SAM 세 행이 8 bag × 3 재생 모두 rc 0·궤적 덮음 ≥ 90 %. 처리 스캔 누락 > 0 재생은 행마다
  ≤ 3/24 (스모크에서 2 회 중 1 회 4 장 — 되풀이되지 않는 사건).
- 실패율: slam_toolbox "Message Filter dropping" > 0 재생은 ≤ 6/24 (파일럿 3 회 중 1 회).
- 이 예측들이 틀리면 그대로 적는다. 판정은 아래 규칙만 쓴다.

## 설정

### 행·bag·재생

| 묶음 | 행 (`rows.py`) | 이미지 | bag 종류 |
|---|---|---|---|
| 주 | `slam_toolbox` (2D, 휠 odom TF) · `rtabmap2d_noimu` (2D, 휠 guess) · `rtabmap3d_noimu` (3D, 휠 guess) · `liosam_planar` (3D, 휠 없음, SE(2) clamp) | slam·slam·slam·liosam | 원·원·원·파생 |
| 참고 | `liosam_loop_re` (LIO-SAM 6DoF, 루프 on) | liosam | 파생 |
| 보조 (순위·검정 밖) | `liosam_oracle` (정답 방위 — 오라클 상한) | liosam | 파생 |

- bag 8 개 = `recordings.md` §3.1 의 공백 검사 통과 bag 전부: `s3_aisle_a1`, `s1_seed1_a1`, `s1_seed3_a2`, `s1_seed2x_a1`
  (**사전 등록 밖 — 결과 뒤 변경 프로토콜**), `s6_open_a3`, `s6_queue_a1`, `s6_wall_a1`, `map_long_a1`.
  RTF 0.25 로 녹화한 bag 5 개와 0.5 로 녹화한 2 개(s3_aisle·s1_seed1)도 표에 표시한다.
- 재생 3 회 (r0·r1·r2) × 행 6 × bag 8 = **144 재생** (`eval_driver.py plan --stage e2`). 순서: map_long 의 slam_toolbox·
  rtabmap3d_noimu 6 재생 먼저 (E3(b) 가 쓸 지도), 그다음 주 행 r0 → r1 → r2, 참고, 보조.
- **재생 배속** (recordings.md §4): 목표 유효 sim 속도 E = **0.25** (스모크·파일럿 0.25–0.29 와 같은 수준) →
  배속 = 0.25 / 녹화 RTF: s3_aisle·s1_seed1 0.507, s1_seed3 1.082, s1_seed2x 1.232, s6_open·queue 1.004, s6_wall·map_long 1.008.
- 재생 구동: `replay_row.sh` 그대로 (멈춘 재생 → 구독 확인 → resume, 기록된 `/clock`, 스폰 = bag 의 `b2rec_spawn.yaml`).
  컨테이너 `amr-ai-b2-eval-e2-<bag>-<행>-r<k>`, `--rm`, `ROS_LOCALHOST_ONLY=1`, DOMAIN 45, 기본 브리지 네트워크.
- 구동기: `eval_driver.py run` (nohup setsid, 재개 가능 — 끝난 작업(DONE) 건너뜀, 낡은 claim 은 `.aborted.<시각>` 로 남기고 다시),
  `cpu_slot.sh b2-eval` 슬롯 둘. 한 슬롯 안 최대 k 재생, 한 번 잡은 슬롯은 1 시간 뒤 새 작업을 받지 않고 놓는다(30 s 쉼).
  원자료 `/home/CAPYI_AI/runs/b24/b2-eval/e2/<bag>/<행>/r<k>/` (job.json·HEAD.txt·status.txt·DONE·host.log + replay_row 산출물).

### 동시 수·부하 규칙 (실측으로 정한다 — 규칙은 지금 정한다)

- 시작 k = **4 / 슬롯 (동시 8)**. 처음 끝난 E2 재생 16 개 중 처리 스캔 판정 누락 > 0 (RTAB-Map·LIO-SAM) 이 **2 개 이상**이면
  나머지는 k = 2 로 줄이고(`control.json`) 이 항목에 시각과 함께 적는다. 구독 확인 시간 초과(rc 4)가 연달아 두 번이면 같은 조치.
  누락은 동시 수와 무관할 수도 있다 (스모크의 4 장은 단독 재생에서 났다) — 그래도 규칙대로 한다.
- 새 재생은 (loadavg 1 분값 − 우리 컨테이너 CPU 코어 합) ≤ **48** 일 때만 띄운다. 넘으면 30 s 마다 다시 본다
  (`load_wait.log`). 우리 몫 = `docker stats` 의 `amr-ai-b2-eval-*` CPU%. 60 s 마다 loadavg·우리 CPU 를 `cpu_samples.jsonl` 에.
- 기동 실패(rc 4 구독 확인 시간 초과, docker 오류 125–127)만 새 시도로 다시 돈다 (상한 3, 이전 시도 `.attempt<n>`).
  재생이 돈 뒤의 결과는 다시 돌리지 않는다.

### 지표 (`peval.py replays` — 재생별)

- 궤적 = `traj_online.tum` (입력 스탬프마다 map → base_footprint, map = 월드) 대 bag `gt.tum`. **평가 창** = 그 bag 의 E2 재생 중
  GT 구간의 90 % 이상을 덮는 것들의 공통 구간 (`<out>_windows.json`).
  - **주**: 정렬 없는 SE(2) xy ATE RMSE [cm] (`ate_rmse_cm`), 함께 max·p95.
  - 보조: SE(2) Umeyama 정렬 RMSE (`ate_se2_rmse_cm` — **보조로만**, map = world 계약), yaw RMSE, 거리 RPE 1·5·10 m (모든 시작점,
    병진 RMSE cm·% 와 회전), 명세 구간 정지·직선·회전 RMSE·max·p95 (3 / 5 / 8 cm 통과 여부 RMSE·max 둘 다), 3D 행 z·roll·pitch.
  - 통로 축·횡 분해 (S3·S6 bag): GT 위치가 통로 상자 x ∈ [−0.30, 0.30], y ∈ [−12, −8] 안인 표본의 오차 y 성분(축)·x 성분(횡)
    RMSE·max·p95·부호 평균.
- 2D 지도 = probe 의 `map_world.yaml` (월드 정렬 0.05 m) 대 GT 0.01 m 단면 z 0.20 (`/home/CAPYI_AI/runs/s2-eval/gt_full`,
  sdf sha256 2efa665e… = 커밋 SDF), τ = 0.05 m, 관측 여유 0.10 m: **주 = ADNN₀ surface [cm]**, 정밀도·재현율 surface, F1, ADNN₀
  filled, IoU (공통 0.05 m 격자). S6 queue·wall 은 블로커 외접 사각형 + 0.10 m 를 추정·GT 양쪽에서 가린다 (`pmetrics.mask_boxes`,
  evaluation.md §6.1 의 규칙을 처음 구현). LIO-SAM 행은 격자를 내지 않는다 → 2D 지도 지표 **해당 없음**.
- 3D 점 지도 (3D 행: RTAB-Map 3D·LIO-SAM 셋): `peval.py cache` 의 **GT 움직임 키프레임**(직전 키프레임에서 0.10 m 또는 0.10 rad,
  bag 마다 행과 무관하게 같은 장들)을 행의 궤적(같은 스탬프의 6DoF 자세)으로 월드에 붙인다. 거리 0.2–20 m, 장마다·전체
  0.05 m 복셀. GT = `surface_points.ply` 0.05 m 복셀. **영역 `roi=traj10m_z3`** = GT 궤적 xy 에서 10 m 안 × z ∈ [−0.10, 3.0]
  (GT 를 깎는 조작 — 이름 붙여 보고, 추정 점도 같은 영역으로). F-score·정밀도·재현율 @5·10 cm, AC@5 cm. 같은 장을 같은 규칙으로
  붙이므로 이 지표는 행의 **자세 품질**만 가른다 (LIO-SAM 특징점 GlobalMap 과 RTAB-Map 지도의 밀도 차를 없앤다 — rtabmap.md §4.3
  의 불공정 해소). RTAB-Map 자체 `cloud_world.ply`·LIO-SAM GlobalMap 은 원자료에 남기고 쓰지 않는다.
- 스캔당 지연 (RTAB-Map 행): icp_odometry "update time" p50·p95·max, rtabmap "RTAB-Map=" — **공유 부하 하 예비값**. LIO-SAM 은 미측정.
- **실패** (재생 단위): rc ≠ 0 · 처리 스캔 판정 누락 > 0 (RTAB-Map = 로그·기록기 대응 중 큰 값, LIO-SAM = 끝 2 장 뺀 대응 —
  replaystat) · 누락 미측정(대응 파일 없음) · slam_toolbox 는 "Message Filter dropping" > 0 (처리 수 근거가 없어 **대용**) ·
  치명 패턴 > 0 · 궤적 덮음 < 90 %. 실패 재생은 따로 세고(행별 실패율 Wilson 95 %) **평균에서 빼지 않는다** (궤적이 아예 없으면 그
  지표만 NaN).

### 판정 (`peval.py compare`) — 사전 등록 비교만 검정한다

- 짝 = 같은 bag·같은 재생 번호의 두 행. bag 차 d_b = 그 bag 의 짝 차(A − B) 평균. 검정 = d_b 의 **Wilcoxon 부호순위 정확**(양측),
  구간 = **두 단계 bag 묶음 부트스트랩** (bag 복원 추출 → 그 bag 안 짝 복원 추출, 10 000 회, seed 20261008, 백분위 95 %).
- 가족 (Holm 보정은 가족 안에서):
  - **T (위치 추정 정확도 — 매핑 궤적)**, 지표 `ate_rmse_cm`, δ = 0.2 cm: C1 `rtabmap3d_noimu − rtabmap2d_noimu` (요인 A 만),
    C2 `rtabmap3d_noimu − slam_toolbox` (3D 대 현행 2D), C3 `liosam_planar − slam_toolbox` (명세 3D SLAM 대 현행 2D).
  - **M (맵 품질)**, 지표 `map_adnn_surf_cm`, δ = 0.2 cm: C4 `rtabmap3d_noimu − rtabmap2d_noimu`, C5 `rtabmap3d_noimu − slam_toolbox`.
- 판정: Holm p < 0.05 **이고** 구간이 0 을 포함하지 않고 |평균 차| ≥ δ → `A<B` (A 가 낫다) / `A>B`. 구간이 (−δ, δ) 안 →
  `equiv` ("차이가 δ 이하로 검출되지 않음"). 그 밖 → `undecided` ("가르지 못함"). **"같다"·"효과 없음" 이라고 쓰지 않는다.**
- **기각 가능한 설계인지 먼저** (`pstats.min_p_wilcoxon`): bag 8 → 가능 최소 p 2/256 = 0.0078 (T: × 3 = 0.023, M: × 2 = 0.016 → 기각
  가능), bag 7 (사전 등록만) → 2/128 = 0.0156 (T: 0.047, M: 0.031 → 기각 가능, 단 7 bag 이 모두 같은 부호일 때만).
  S1 만 (bag 3 → 0.25, bag 2 → 0.5) 은 **기각 불가능한 설계** → 기술 통계(bag 별 차·부호 수)만 낸다.
- **S1 규칙 (완성 계획 §6, 2026-10-08 17시 결정)**: 모든 결과를 (i) 8 bag(= S1 3 bag 포함) 과 (ii) 사전 등록 7 bag(= S1 2 bag)
  로 둘 다 낸다. 가족 판정이 둘에서 다르면 **(ii) 를 주 결과로** 쓴다. S1 만의 표도 3 bag·2 bag 둘 다.
- 분산 성분: 행·지표마다 bag 안 재생 간 합동 sd 와 bag 평균들의 sd 를 따로 (`pstats.variance_components`).
- 기술 비교 (검정 안 함): 그 밖의 모든 행 짝·지표(RPE, max, SE(2) 정렬, F-score — 3D 행끼리, 6DoF 참고, 오라클은 순위 밖).

### E5 (매핑 쪽) — S6 세 bag

- 행 4 개 (주 행). 지표 `aisle_axis_rmse_cm` (보조: lateral, max, 전체 ATE). bag 하나씩이라 bag 간 일반화 불가 — 재생 3 회 짝만.
- 행마다 wall − open, queue − open: 판정 `pstats.replay_rule` (평균 차 > max(2 × 합동 재생 sd, δ 0.2 cm) → `differs`, 아니면
  `undecided`). 2D 대 3D 의 차의 차 (`rtabmap2d − rtabmap3d`, `slam_toolbox − rtabmap3d`, `slam_toolbox − liosam_planar`) 를 같은
  δ 로 기술한다. **evaluation.md §6.2 의 기하 계산(2D 축 구속 점 open 70 · queue 59 · wall 0, 16ch 7 927 · 7 902 · 7 927)과 나란히** 쓴다.
  가설 지지 = 2D 행 wall − open 이 `differs`(+) 이고 3D 행은 `undecided`, 차의 차 > δ. 그 밖은 지지 안 됨 (불리해도 그대로).

### E3(b) 가 쓸 SLAM 지도 선택 규칙 (지금 정한다 — E3(b) 의 나머지는 다음 항목에서 사전 등록)

- 2D 지도 = `slam_toolbox` 의 map_long 재생, 3D 지도 = `rtabmap3d_noimu` 의 map_long 재생 (RTAB-Map DB). 후보 = rc 0·누락 0
  (slam_toolbox 는 메시지 필터 버림 0)·지도 산출물 있음. 후보 3 개 → 자기 궤적 전 구간 xy ATE RMSE 가 **중앙값**인 재생,
  2 개 → 재생 번호가 작은 것, 1 개 → 그것, 0 개 → 누락이 가장 적은 재생(같으면 번호가 작은 것)을 쓰고 "실패 재생의 지도" 로 표시.

### 시간 어림

E2 144 재생: 재생 벽시계 = sim 구간 / 0.25 + 기동·마무리 약 75 s → 행·재생 하나(8 bag)에 약 2.1 h, 전체 직렬 약 38 h.
동시 8 이면 약 5 h + 부하 대기 (지금 loadavg 1 분값 115 — 대기가 길 수 있다). 분석(캐시·지표·비교) 약 1 h.
오래 걸리면 E2 → E3(b) → E5 → E4 순으로 끝낸다 (과제 지시).

### 도구 검증 (사전 등록 전, 이미 공개된 파일럿 결과로만) — `B2eval-validate-pilot` [계산]

b2-offline 의 파일럿 재생 12 개(`/home/CAPYI_AI/runs/b24/b2-offline/replay/<행>/r0–2`, 링크만)를 이 도구로 다시 쟀다
(`/home/CAPYI_AI/runs/b24/b2-eval/validate/out/replays.jsonl`, 창 [4.5, 182.5] s). xy ATE RMSE 가 replay.md §3 표의 재생별 값과
0.001 cm 안에서 같다 (rtabmap3d_noimu 0.7705/0.7709/0.7709 vs 0.7707/0.7711/0.7711, slam_toolbox 1.7400/1.7402/2.1303 vs
1.7405/1.7407/2.1309, liosam_planar 1.7667/1.5866/1.7662 vs 1.7672/1.587/1.7666 — 창 차이 [4.6 → 4.5 s]). 실패 판정도 같다 (RTAB-Map
누락 1, liosam_planar r1 누락 2, slam_toolbox r2 메시지 필터 1). slam_toolbox 지도 ADNN₀ surface 4.618 / F1 0.746 / IoU 0.548 은
rtabmap.md §4.3 의 4.62 / 0.746 / 0.548 과 같다. 이 검증은 공개된 결과의 재계산이라 새 정보를 보지 않았다.

## 실행 중 기록 (지표를 보기 전 — 누락·rc·구동기 상태만)

- **재생 사본**: E2 는 `ws_e2` = `ebf0cea` (git archive) 에서 돈다 — 사전 등록 커밋(686d902) 뒤 E3(b)·E4 행·launch 인자를 더했지만
  E2 행의 동작은 같다 (기본 인자 그대로, 시험 `test_config.py` 전체 통과). 구동기는 그 뒤 두 번 바꿨다 (아래).
- **00:04–00:30 부하 대기**: 부하 규칙(다른 부하 ≤ 48)에 막혀 재생 0. 다른 사용자 openroad·ruby 등으로 loadavg 66–132.
  00:11 구동기 수정(부하에 막히면 빈 슬롯을 쥐지 않는다, `ebf0cea`) 뒤 다시 띄웠다. 00:30 첫 재생 8 개 (map_long 6 + 정답 자세 DB + LIO-SAM 1).
- **01:17 구동기 결함 (결과 무관, 복구함)**: 컨테이너가 끝나고 그 pool 이 30 s 주기로 거두기 전에 다른 pool 이 claim 을 "낡은 것"
  으로 보고, **끝난 출력**을 `.aborted` 로 옮긴 뒤 같은 재생을 다시 띄웠다 — `map_long` slam_toolbox r0, rtabmap3d_noimu r2,
  정답 자세 DB r0 3 건. 원래 재생의 pool 은 rc 0 으로 끝났다 (driver.log END rc=0). 복구: 중복 재생 3 개를 `docker stop`
  (rc 143) 하고 `.dup.<시각>` 으로 남김, 원래 출력(`loadavg_end` 까지 감)을 제자리로 되돌리고 DONE 에 복구 메모. 고친 구동기
  (`2648ad7` — claim 에 주인 pool pid, 주인이 살아 있으면 건드리지 않고 주인 없이 끝난 출력은 DONE(rc 0 추정))를 01:21 에 다시 띄웠다.
  옛 pool 을 죽인 01:20–01:3x 사이에는 그 pool 이 띄운 재생 5 개(map_long rtabmap2d r0, s1_seed1 네 행 r0)가 슬롯 밖에서
  끝까지 돌았다 (슬롯 규칙 위반 — 그 동안 동시 재생 최대 10).
- **동시 수 규칙 발동 (01:25)**: 처음 끝난 재생 중 처리 스캔 판정 누락 > 0 이 3 개 — map_long `rtabmap3d_noimu` r0·r1·r2
  (6 956 장 중 10·7·12 장, 모두 sim 70–200 s, 그 구간의 loadavg 85 → 50). 사전 등록 규칙대로 k 4 → **2 / 슬롯** (동시 4).
  map_long 의 slam_toolbox 3 재생은 메시지 필터 버림 0, liosam_planar r0 은 판정 누락 0.
- E3(b) 지도 선택 (`peval.py select-maps`, 01:21, 사전 등록 규칙 — ATE 값은 보지 않고 고른 번호만 확인): slam_toolbox **r1**
  (깨끗한 후보 3 → ATE 중앙값), rtabmap3d_noimu **r1** (깨끗한 후보 0 → 누락이 가장 적은 재생 = 7 장, "실패 재생의 지도").
  `maps/selection.json`, 지도 sha256 은 `maps/` 아래.

### 이어받기 (2026-10-13 22시, 새 b2-eval 인스턴스) — 지표를 보기 전 (rc·누락·필터 버림만)

- **끊긴 경위** [측정]: 구동기·pool·재생 컨테이너 4 개가 **2026-10-09 07:07:46 KST 에 함께 끊겼다** — 네 재생의 `env.txt`
  `launch_exit` 가 07:07:46, `driver.log` 에 END·REL 없음, `cpu_slots.log` 에 b2-eval REL 없음, 그 직전 `cpu_samples.jsonl` loadavg 218.
  서버 재부팅(2026-10-10 17:43)보다 **하루 반 앞**이다. 앞선 인스턴스가 사용량 한도로 끝난 시각과 같아, 세션 종료가 nohup setsid
  자식까지 거둔 것으로 본다 **[추정]** (journal 에 OOM 기록 없음). 교훈: setsid 구동기도 감독 세션과 함께 죽을 수 있다 → 재개
  가능 설계(DONE 건너뜀)가 이번에 그대로 쓰였다.
- **재고** (`runs/b24/b2-eval/resume/inventory_before.jsonl`, `tmp/inventory.py` — 작업 목록 301 × 원자료, `peval.drops_of`):
  E2 완결 66 / 144 (주 행 r0·r1 64 + map_long 의 slam_toolbox·rtabmap3d_noimu r2), 부분 4, 미시작 74. 참고·보조 행(liosam_loop_re·
  liosam_oracle) 0. E3(b) 정답 자세 DB 1 / 85 완결, E4 0 / 72. 완결 66 개 모두 rc 0.
  주 행 (r0 / r1 / r2 — "완" = DONE rc 0, **실패** = 사전 등록 실패 규칙의 누락·메시지 필터 버림 > 0, "부분" = 완결 표지 없음):

| bag | slam_toolbox | rtabmap2d_noimu | rtabmap3d_noimu | liosam_planar |
|---|---|---|---|---|
| s3_aisle_a1 | 완 **실패**(필터 1) / 완 **실패**(필터 1) / — | 완 **실패**(누락 2) / 완 **실패**(누락 1) / — | 완 **실패**(누락 1) / 완 **실패**(누락 1) / — | 완 / 완 / — |
| s1_seed1_a1 | 완 / 완 / — | 완 / 완 / 부분 | 완 / 완 / — | 완 / 완 / 부분 |
| s1_seed3_a2 | 완 / 완 / — | 완 / 완 / — | 완 / 완 / — | 완 / 완 / — |
| s1_seed2x_a1 | 완 / 완 / — | 완 / 완 / — | 완 / 완 / — | 완 / 완 / — |
| s6_open_a3 | 완 / 완 / — | 완 / 완 / — | 완 / 완 / — | 완 / 완 / — |
| s6_queue_a1 | 완 / 완 / — | 완 / 완 / — | 완 / 완 / — | 완 / 완 / — |
| s6_wall_a1 | 완 **실패**(필터 1) / 완 **실패**(필터 1) / — | 완 **실패**(누락 3) / 완 **실패**(누락 3) / — | 완 **실패**(누락 3) / 완 **실패**(누락 3) / — | 완 **실패**(누락 1) / 완 **실패**(누락 1) / — |
| map_long_a1 | 완 / 완 / 완 | 완 **실패**(누락 1) / 완 **실패**(누락 1) / 부분 | 완 **실패**(누락 10) / 완 **실패**(누락 7) / 완 **실패**(누락 12) | 완 / 완 / 부분 |

- 눈에 띄는 것 (해석은 결과 절에서): s3_aisle·s6_wall 두 bag 의 실패가 **r0·r1 에서 같은 수로 되풀이**되고 행을 가로지른다
  (s6_wall: RTAB-Map 2D·3D 누락 3·3, LIO-SAM 1, slam_toolbox 필터 1). 부하에 따른 우연이 아니라 bag 에 묶인 사건일 수 있다 —
  사전 등록 규칙대로 실패로 세고 평균에서 빼지 않는다. s3_aisle rtabmap2d 의 "치명" 1 은 `aborting odometry update` (같은 장의
  `Odom: ratio=0` 1 건). 원인 조사는 E2 가 끝난 뒤 **보충(사전 등록 외)** 으로 한다.
- **조치**: 부분 재생 4 개(map_long rtabmap2d_noimu·liosam_planar r2, s1_seed1 rtabmap2d_noimu·liosam_planar r2)를 지우지 않고
  `<출력>.aborted.20261013T220358` 로 옮겼다 (안에 `ABORTED.txt` 사유, 낡은 claim 의 owner 를 `claim_owner.txt` 로). claim 디렉터리는
  손으로 지웠다 — 재부팅 뒤 pid 가 재사용되면 구동기의 "주인 살아 있음" 판정이 틀릴 수 있어서다. 끝난 66 재생은 다시 돌리지 않는다:
  구동기는 `DONE` 이 있는 작업을 집지 않는다 (`is_done`; 다시 띄우기 전 `eval_driver.py status` = e2 done 66 / todo 78, 첫 후보 6 개가
  모두 r2 미완결 작업임을 확인). `plans/*.json` 은 지금 코드의 `plan_e2/e3/e4()` 출력과 같다 (301 작업 비교).
- **다시 띄움**: 2026-10-13 22:04 KST, `bin/eval_driver_888a5c4.py` (= 커밋 888a5c4 의 `eval_driver.py`, sha256 `de421b71…`), k = 2 / 슬롯
  (control.json 그대로), 부하 규칙 그대로 (시작 loadavg 22.9). 남은 순서 = E2 주 행 r2 → 참고 행 → E3(b) → E2 보조(오라클) → E4.
  남은 재생 234 개, 직렬 어림 약 52 h → 동시 4 로 약 13 h (+ 부하 대기).
- **보충(사전 등록 외) 누락 분해 — 정의를 지금 적는다**: 재고에서 누락 스탬프를 보니 (`tmp/miss_stamps.py`, 정확도 지표는 보지 않음)
  s3_aisle·s6_wall·map_long(2D) 의 누락은 모두 **bag 의 첫 장들**(sim 10.7–11.7 s — 첫 점군 스탬프에서 0.2 s 안, 출발 전 정지 구간)이고
  r0·r1 에서 같은 스탬프다 (s6_wall: RTAB-Map 2D·3D 10.7·10.8·10.9 s, LIO-SAM 10.7 s; s3_aisle: 11.5 (·11.7) s; map_long 2D 11.0 s).
  map_long rtabmap3d 의 7–12 장은 주행 중(sim 72–202 s)이다. 그 bag 들은 첫 점군 스탬프가 첫 EKF TF 스탬프와 같거나 0.1 s 안이다
  (s6_wall 점군 10.700 / TF 10.715) — guess TF 가 아직 버퍼에 없어 첫 장들을 버린 것으로 본다 **[추정]**. 실패 판정은 사전 등록 규칙
  그대로 둔다(주 결과). 결과 문서에 **보충**으로 누락 재생을 "시작 장만"(모든 누락 스탬프 ≤ 그 bag 첫 점군 스탬프 + 1.0 s) / "주행 중
  포함" 으로 나눠 함께 적는다 (`presults.py failures`). 이 정의는 누락 위치를 본 뒤 정했으므로 사전 등록 외다.

### EKF 결함 bag — E2 중간 결과를 본 뒤 찾은 녹화 결함 (2026-10-14 11:30, 결과 뒤 — 보충)

- **경위**: E2 주 행·참고 행(120 재생)이 끝난 10:46 에 오라클 보조 행 전 **중간 분석**(`analysis/e2_interim/`, 커밋된 peval.py 그대로,
  창은 오라클 행 없이 — 최종은 오라클 뒤 다시 계산)을 돌렸다. `s1_seed3_a2`·`s1_seed2x_a1` 에서 휠-EKF 를 쓰는 세 행(slam_toolbox·
  RTAB-Map 2D·3D)의 ATE 가 96–221 cm, `s6_wall_a1` 에서 10–19 cm 였고 휠을 쓰지 않는 LIO-SAM 두 행은 같은 bag 에서 2–8 cm 였다.
- **원인** [측정]: bag 의 EKF `odometry/filtered` 를 정답과 첫 자세에서 맞춰 비교했다 (`tmp/odom_drift.py`, `supp/odom/odom_drift.json`):
  EKF yaw 오차가 **처음부터 일정한 0.573 °/s (= 0.0100 rad/s)** 로 자란다 — s1_seed3 −0.0100, s1_seed2x −0.0100, s6_wall +0.0100 rad/s
  (정지 중에도). 최대 xy 오차 12.0 m · 37.9 m · 1.63 m, yaw 128° · 149° · 48°. 같은 bag 의 `wheel_odom` 은 21 · 98 · 0.5 cm 로 멀쩡하다.
  녹화 `launch.log` 의 `imu_filter_node` 가 이 세 bag 에서만 기동 0.4 s 안에 **"motion detected during startup bias estimation; falling
  back to parameter bias"** (파라미터 바이어스 = 0) 를 냈다. 시뮬 IMU 의 자이로 바이어스 평균은 0.01 rad/s (`amr_description` sensors.yaml
  `gyro_bias_mean`) 이고 다른 다섯 bag 은 바이어스를 ±0.0100 (s3_aisle 만 0.01048) 으로 추정했다. 그래서 세 bag 의 EKF odom 은 자이로
  바이어스가 그대로 적분된 것이다. 녹화 공백 검사(`recgate`)는 간격만 보고 odom 일관성은 보지 않아 놓쳤다.
- **해석의 범위**: 휠-EKF guess 를 쓰는 RTAB-Map 2D·3D 와 odom TF 를 쓰는 slam_toolbox 가 같은 결함을 받는다. LIO-SAM 은 휠·EKF 를 쓰지
  않고 IMU 바이어스를 스스로 추정한다. 이 bag 들의 차이는 2D 대 3D 가 아니라 **휠-EKF 사전값 유무(요인 B)** 의 차이다. 배포 스택의
  실제 결함 후보이기도 하다 (기동 때 흔들리면 바이어스 0 으로 떨어짐) — `amr_localization` 소유자에게 요청할 일 (결과 문서).
- **분석 처리**: 사전 등록 주 결과(8·7 bag)는 그대로 낸다. **보충(사전 등록 외, 결과 뒤)** 으로 결함 bag 셋을 뺀 5 bag (`s3_aisle_a1`·
  `s1_seed1_a1`·`s6_open_a3`·`s6_queue_a1`·`map_long_a1`) 의 같은 표를 기술로 낸다 (5 bag 가능 최소 p 0.0625 → 기각 불가능한 설계).
  E3(b)·E4 의 보충 규칙은 그 결과를 보기 전에 그 항목에 적었다.

## 결과

2026-10-15 02:20 최종 분석 (144 재생 모두 rc 0, 재생 기간 2026-10-09 00:30 – 2026-10-15 01:46, 재생 시작 loadavg 5–80) [측정·계산, 공유 부하 하].
원자료 `runs/b24/b2-eval/e2/`, 지표 `analysis/e2/replays.jsonl`·`compare.json`, 표 `analysis/tables/e2.md` (presults.py), 보충
`analysis/supp/e2_clean.md`. 분석 HEAD `db8cea2` (peval.py 사전 등록 판 그대로). 평가 창은 오라클 행 전 중간 분석과 같았다
(`replays_windows.json` 동일) — 중간 분석의 수치가 최종과 같다.

**불리한 것 먼저**
- **사전 등록 가족 T·M 의 다섯 비교가 모두 "가르지 못함"** 이다 (8 bag·사전 등록 7 bag 둘 다, Holm p ≥ 0.23). 기각 가능한 설계였지만
  bag 사이 부호가 갈렸다.
- **녹화 결함 3 bag** (위 「EKF 결함 bag」): 휠-EKF 를 쓰는 세 행이 `s1_seed3_a2` 96–102 cm, `s1_seed2x_a1` 208–222 cm, `s6_wall_a1`
  10–19 cm 로 틀렸다. 이 bag 들에서 RTAB-Map 두 행의 SE(2) 정렬 오차는 0.2–1.8 cm (slam_toolbox 2.9–12.4 cm) 라 궤적 모양은 거의 맞고 **틀어진 것은 주로 월드 기준 방위**다
  (yaw RMSE 5.9–20°). 사전 등록 주 결과는 이 bag 을 포함한다.
- **실패 재생** (Wilson 95 %): slam_toolbox 6/24 [0.12, 0.45] (메시지 필터 버림 — s3_aisle·s6_wall 결정적), RTAB-Map 2D 11/24 [0.28, 0.65],
  RTAB-Map 3D 11/24 [0.28, 0.65], LIO-SAM 평면·6DoF·오라클 각 3/24 (s6_wall 첫 장). 누락 재생 중 "시작 장만"(첫 점군 + 1 s 안,
  보충 분류) — RTAB-Map 2D 9/11, 3D 6/11, LIO-SAM 3/3. 주행 중 누락은 RTAB-Map 3D 5 재생 (map_long 7·10·12 장 — 부하 85→50 때,
  s1_seed1 r2 6 장·s1_seed2x r2 11 장 — 부하 130–190 때), 2D 2 재생.
- 예측 P-E1 ("3D 세 행 누락 > 0 재생 ≤ 3/24") 은 RTAB-Map 3D 에서 **틀렸다** (11/24). LIO-SAM 은 3/24 로 맞았다.
- 예측 P-C1·P-C2 (`A<B`) 는 **틀렸다** (가르지 못함). P-C3·C4·C5 (`undecided`) 는 맞았다.
- 2D 격자 지도에서 RTAB-Map 3D 의 IoU 가 2D 두 행보다 낮다 (결함 없는 5 bag 평균 0.40 대 slam_toolbox 0.59·RTAB-Map 2D 0.54,
  표면 재현율 3D 0.70 대 slam_toolbox 0.93) — 3D 가 2D 격자 지표에서 앞서지 않는다.

**사전 등록 가족 판정** (차 = A − B, 작을수록 A 가 낫다; 두 단계 bag 부트스트랩 95 %):

| 비교 | 8 bag | 사전 등록 7 bag |
|---|---|---|
| C1 RTAB-Map 3D − 2D, ATE | −2.37 [−5.62, −0.07] cm, 음 6/양 2, Holm p 0.23 → 가르지 못함 | −2.27 [−6.10, 0.21], 5/2, 0.47 → 가르지 못함 |
| C2 RTAB-Map 3D − slam_toolbox, ATE | 0.00 [−3.71, 4.09], 6/2, 0.64 → 가르지 못함 | −1.52 [−4.67, 1.72], 6/1, 0.47 → 가르지 못함 |
| C3 LIO-SAM 평면 − slam_toolbox, ATE | −38.6 [−89.5, −0.85], 6/2, 0.23 → 가르지 못함 | −15.4 [−42.1, 0.68], 5/2, 0.47 → 가르지 못함 |
| C4 RTAB-Map 3D − 2D, 지도 ADNN₀ surface | +3.32 [−3.23, 14.06], 5/3, 1.0 → 가르지 못함 | +4.20 [−3.40, 16.31], 4/3, 1.0 → 가르지 못함 |
| C5 RTAB-Map 3D − slam_toolbox, ADNN₀ | +4.14 [−1.75, 13.60], 3/5, 1.0 → 가르지 못함 | +4.16 [−2.33, 15.17], 3/4, 1.0 → 가르지 못함 |

S1 만 (3 bag·2 bag) 은 기각 불가능한 설계 — 기술만 (`analysis/tables/e2.md`).

**보충(사전 등록 외, 결과 뒤) — EKF 결함 bag 제외 5 bag** (기각 불가능한 설계, 기술): C1 −3.45 [−8.28, −0.49] cm (4/1), C2 −1.60
[−2.56, −0.66] cm (**5/0**), C3 +0.27 [−1.18, 2.19] (3/2), C4 −1.41 [−4.86, 0.53] (3/2), C5 −0.28 [−1.07, 0.42] (2/3). 행별 xy ATE RMSE
(bag 평균의 평균): RTAB-Map 3D 1.46 · LIO-SAM 6DoF 2.98 · slam_toolbox 3.06 · LIO-SAM 오라클 3.08 · LIO-SAM 평면 3.33 · RTAB-Map 2D 4.91 cm.
지도 ADNN₀ surface: RTAB-Map 3D 4.15 · slam_toolbox 4.43 · RTAB-Map 2D 5.56 cm, IoU 0.40 · 0.59 · 0.54.

**행별 (8 bag, bag 평균의 평균 — 재생 간 sd / bag 간 sd)** — xy ATE RMSE: slam_toolbox 42.2 (0.00/74.1), RTAB-Map 2D 44.6 (0.19/78.8),
RTAB-Map 3D 42.2 (0.04/79.2), LIO-SAM 평면 3.60 (0.34/2.17), 6DoF 3.60 (0.16/2.46), 오라클 3.13 (0.24/1.68) cm. SE(2) 정렬(보조):
4.76 · 2.93 · **1.00** · 2.19 · 2.30 · 2.26 cm. RPE 10 m: 1.29 · 0.24 · **0.16** · 0.29 · 0.34 · 0.33 %. 3D 점 지도 F@5 cm: RTAB-Map 3D 0.674,
LIO-SAM 평면 0.739, 6DoF 0.335 (z 최대 57 cm 표류), 오라클 0.375. **재생 간 sd 는 bag 간 sd 보다 두 자릿수 이상 작다** — 같은 bag 을
다시 돌려도 거의 같은 값이 나오고 (slam_toolbox 는 재생 간 sd 0.00), 차이는 bag(장면·녹화)에서 온다.

**E5 (매핑) — 통로 축 RMSE [cm], 재생 3** (`pstats.replay_rule`, δ 0.2):

| 행 | open | queue | wall | queue − open | wall − open |
|---|---|---|---|---|---|
| slam_toolbox | 1.53 ×3 | 1.00 ×3 | 3.51 ×3 | −0.53 differs | +1.99 differs |
| RTAB-Map 2D | 12.93 / 12.95 / 12.75 | 1.82 / 1.86 / 1.89 | 2.04 ×3 | −11.02 differs | −10.84 differs |
| RTAB-Map 3D | 0.18 ×3 | 0.18 ×3 | 0.41 ×3 | −0.00 undecided | +0.23 differs |
| LIO-SAM 평면 | 0.57 / 0.58 / 0.57 | 0.61 / 0.60 / 0.61 | 1.11 / 1.13 / 1.11 | +0.03 undecided | +0.55 differs |

- 사전 등록 규칙(2D wall − open `differs`(+) **이고 3D `undecided`**, 차의 차 > δ): 3D 두 행도 wall − open 이 `differs`(+0.23·+0.55)
  라 **지지 안 됨**. 차의 차 (2D − 3D, wall): slam_toolbox − RTAB-Map 3D +1.75, slam_toolbox − LIO-SAM +1.44, RTAB-Map 2D − 3D −11.08 cm.
- 혼입: **wall bag 은 EKF 결함 bag** 이다 (휠-EKF 행의 wall 값은 결함을 함께 받는다; 횡 오차 slam_toolbox 18.8·RTAB-Map 10.2 cm).
  RTAB-Map 2D 는 **열림(open)** 에서 통로 축 12.9 cm 로 가장 나빴다 — 막힘과 무관한 사건이다 (원인 미조사).
- 기하 계산(evaluation.md §6.2: 2D 축 구속 점 open 70 · queue 59 · wall 0)이 예측하는 "wall 에서 2D 축 오차 급증" 은 매핑에서 나타나지
  않았다 (slam_toolbox +2.0 cm, RTAB-Map 2D 는 오히려 open 이 최악). P-E5 의 예측("매핑 쪽 지지되지 않음") 은 결론이 맞았고, 근거로 든
  "모든 행 `undecided`" 는 틀렸다.

**E1 (끝까지 도는가)**: RTAB-Map 3D·LIO-SAM 평면·6DoF·오라클 네 3D 행이 8 bag × 3 재생 모두 rc 0, 궤적 덮음 ≥ 0.993, 런치 깨끗한 종료.
처리 스캔 누락 > 0 재생은 위 실패와 같다.

**스캔당 지연 (공유 부하 하 예비값, 재생 p50·p95 의 평균)**: RTAB-Map 3D icp_odometry 62.5 · 117.6 ms (최대 852), rtabmap 149 · 274 ms;
RTAB-Map 2D icp 7.3 · 47.0 ms, rtabmap 40.5 · 94.2 ms. LIO-SAM·slam_toolbox 미측정.

## 해석

- E2 결과의 해석은 결과 문서 `bonus2_results.md` §4–§5 에 모은다 (E3(b)·E4 와 함께 읽어야 해서).

## 다음
