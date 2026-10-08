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

## 결과

(E2 끝난 뒤 채운다)

## 해석

## 다음
