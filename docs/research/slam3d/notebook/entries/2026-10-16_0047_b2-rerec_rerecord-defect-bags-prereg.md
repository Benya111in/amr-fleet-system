# 보너스 2 결함 bag 재녹화 — 기동 바이어스 결함을 고친 녹화와 E2·E3(b)·E4 재생, 결함 bag 교체 8 bag 재분석 (사전 등록)

- 시각: 2026-10-16 00:47 KST (작성 시작) · 에이전트: b2-rerec · 단계: P-RR (`bonus2_completion_plan.md` §3 E1–E6 의 '부분' 메우기,
  COORDINATION §13)
- 코드: 사전 등록 커밋 = 이 항목을 넣은 커밋 (브랜치 `feature/b2-rerec`, 분기 `feature/bonus2-slam3d` `c3976b5`). 녹화 도구 고침
  `3e33b46`, 정착 대기 진단 `14898ca`. 녹화는 이 커밋의 git archive 사본(`runs/b24/b2-rerec/ws_rec`, `SNAPSHOT_HEAD`)에서, 재생은 원래 재생 사본
  (`runs/b24/b2-eval/ws_e2` = `ebf0cea`, `ws_e3b` = `a0850e1`, 읽기 전용)에서 돈다.
- **분석 스크립트 sha256** (원래 사전 등록 판 그대로 — 고치지 않는다):
  - `src/amr_slam3d/scripts/peval.py` `dfe200753dcf8d43ce17ada9387ed322f37fdf1f113115b6feafa015f548b91c` (= E3(b)·E4 항목의 판, E2 판에 더하기만)
  - `src/amr_slam3d/scripts/presults.py` `8e50b004b65c6b27629308094e9a7ad3501ece43c46bdd469a15351d4abcc86e`
  - `src/amr_slam3d/scripts/loc_lost.py` `c4859fa8b2788034757de7c6dc81f9e865dd616ccc0eea8a507ee490e240270a`
  - `src/amr_evaluation/amr_evaluation/slam3d/pstats.py` `58c37a2e1846735eaac447b774443a4df6ee34deb56064bb3df310c52249d048`
  - `src/amr_evaluation/amr_evaluation/slam3d/pmetrics.py` `916666f0a4223e1f121418adb24c111629cc5a30fbe59bcba05831a90ac0d6f0`
- **녹화·재생 도구 sha256** (`3e33b46`, settle_wait.py 는 `14898ca`):
  - `src/amr_slam3d/amr_slam3d/recgate.py` `6d46a1fd66a0056df2d48462d5884a34372f68d6e3aefeb4c80e00d9a486b6be`
  - `src/amr_slam3d/scripts/bag_tools.py` `07f6dd624fe91fc5ec4c7f552eb9ce551547927197f3beaa2ebbc380205b6f76`
  - `src/amr_slam3d/scripts/record_bag.sh` `cc899b28f377f36517b7be0ae9d11453d17320ccdcf43e4e0c70119a50b2a249`
  - `src/amr_slam3d/scripts/settle_wait.py` `dc46189b5c610b27f2907cb75c7becaf06d3a796ffb0495675fce1b710f2da79` (판정 밖 진단 first20·first100 —
    받은 첫 IMU 표본 20·100 개의 정지 판정 — 을 더한 판, 시운전은 그 전 판 `2bd2a006…`)
  - `src/amr_slam3d/scripts/loc_stack_args.py` `dbec09c8b4fea12c12bc0a515fc92a0946fed0f92ce16134cffd95dc835b9c8b`
  - `src/amr_slam3d/scripts/rec_attempt.sh` `294c02973a2a4562049ace6d8b419ec5e4ea5c9b5be60e7ffd910859bbac21ab`
  - `src/amr_slam3d/scripts/rec_driver.sh` `bfe9d9477825dd9b5b4482c20d8266901b4a6a0bb4db6b42540b10e090cb573c`
  - `src/amr_slam3d/scripts/rec_report.py` `1993480af175f4bc5f388dd42075e0de946e462af70a50745b86509ff6010866`
  - `src/amr_slam3d/scripts/eval_driver.py` `63332e97b8dc46e8e77ba5189b38d20797095d44716ef745c1c2b67db8966c85`
  - `src/amr_slam3d/scripts/rerec_plan.py` `46824fde9e1c188fba950704d4a8d1bb7f11dc4f8bc329b8a8d36a12339bbf15`
  - `src/amr_slam3d/config/rerec_plan.txt` `3387e3c442205cbc06b7b0cac61cec11b82c3fe554e765acb709d44958765b5b`
- 관련 실행 id: `B2rerec-rr_<시나리오>_a<k>` (녹화 시도), `B2rerec-dry_s6wall_a1` (시운전 — P 집합 밖), `B2rerec-<단계>-<태그>-<행>-r<k>`
  (재생), `B2rerec-test-*`·`B2rerec-validate-gate` (시험·도구 검증), `B2rerec-ana-*` (분석)
- 근거: `bonus2_results.md` §0-2·§12-1·2·3 (결함과 요청), E2 항목 「EKF 결함 bag」, E3(b)·E4 항목 「결과 전에 더하는 보충 분석 규칙」,
  `recordings.md` §0·§3, 녹화 항목 `2026-10-08_1030_b2-rec_record-gate-scenarios.md` (프로토콜·변경 이력), `amr_localization`
  `src/imu_filter.cpp` `ImuFilter::finishStartup` (결과 문구), `config/imu_filter.yaml` (정지 판정 문턱).

## 질문

1. 녹화 결함(기동 바이어스 추정이 "motion detected … falling back to parameter bias" 로 0 에 떨어짐)은 **로봇이 스폰 뒤 자리 잡는 동안
   imu_filter 가 기동해서** 생기는가, 그리고 로봇이 정지한 뒤 위치추정 스택을 띄우면 사라지는가? — 결함 bag 셋(`s6_wall`, `s1_seed3`,
   `s1_seed2`)을 같은 경로·같은 월드 변형으로 다시 녹화해, 기존 공백 검사(규칙 1–4)와 새 검사(규칙 5·6)를 몇 번 시도에 통과하는가.
2. 결함 bag 셋을 재녹화 bag 으로 **교체한 결함 없는 8 bag** 에서, 원래 사전 등록 분석(가족 T·M·L1·L2, E5 규칙, E4 기술 판정, E1 실패
   셈)의 결론은 무엇이고 기존 결과(결함 포함)와 어떻게 다른가?

## 가설과 예측 (결과 보기 전에 적는다)

**정직성 메모**: 아래 예측의 근거는 이미 공개된 결과(결함 없는 5 bag·4 bag 의 보충 표 — 결과 문서 §4·§5·§6) 다. 새 bag 의 결과는 아직 없다.
분석 규칙(가족·δ·검정·실패 규칙)은 원래 사전 등록 그대로이고 **이 항목에서 바꾸는 것은 bag 집합(결함 bag 교체)뿐**이다.

- **H-RR0 (원인 — 진단 근거)**: 녹화 시도 14 회의 imu_filter 기동 결과 [측정 `runs/b24/b2-rec/rec/*/launch.log`]: 파라미터 바이어스로
  떨어진 4 시도(`s1_seed3_a2`·`s1_seed2x_a1`·`s6_wall_a1`·`s6_open_a1`)는 **모두 RTF 목표 0.25** 이고, RTF 0.5 시도 6 회(`s3_aisle_a1`·
  `s1_seed1_a1`·`s1_seed2_a1`–`a3`·`s1_seed3_a1`)는 0 이다 (RTF 0.25 시도 8 회 중 4). 네 번 모두 imu_filter 시작 줄 0.38 s(벽시계) 뒤, 곧
  첫 판정(표본 20 개 = sim 0.1 s)에서 떨어졌다. bringup 은 스폰 확인(스폰 z 0.02 m) 뒤 월드를 재개하자마자 위치추정 스택을 띄운다
  (`launch_utils.spawn_sequence(after_spawn=stack_actions)`). RTF 0.25 에서는 같은 벽시계 기동 지연 동안 sim 이 반만 흘러, imu_filter 의
  첫 표본이 로봇이 바닥에 닿아 자리 잡는 구간에 걸릴 수 있다 **[추정]** — 첫 표본의 IMU 값은 기록되지 않아 직접 확인하지 못했다.
- **H-RR1 (녹화)**: 정착 뒤 기동(아래 프로토콜)이면 세 시나리오 모두 **1 회차에** 규칙 1–6 을 통과한다. 파라미터 바이어스 줄 0 / 시도.
  확신 중간 — 원래 실패의 주원인이던 EKF 타이머 지연(규칙 1)은 부하에 달렸다 (지금 서버는 한가하다, 1 분 loadavg 1–5).
- **H-RR2 (표류)**: 재녹화 3 bag 의 EKF yaw 표류율 |rate| ≤ 0.0005 rad/s (결함 없는 5 bag 최대 0.00041 — `s3_aisle_a1`).
- **H-RR3 (E2·E3(a), 교체 8 bag, 가족 T·M — Holm)**: C2 (RTAB-Map 3D − slam_toolbox, ATE) `A<B` (결함 없는 5 bag 이 5/0, −1.60 cm);
  C1 (3D − RTAB-Map 2D) 가르지 못함 (5 bag 4/1 — `s1_seed1` +0.37; 8 bag 에서 반대 부호 하나면 가장 작은 순위일 때만 Holm p 0.047),
  C3·C4·C5 가르지 못함 (5 bag 3/2·3/2·2/3). 확신: C2 중간, 나머지 중간.
- **H-RR4 (E3(b), 교체 7 bag)**: L2 (SLAM 지도) `A<B` (4 bag 4/0, −0.93…−2.45 cm) — 확신 높음. L1 (정답 지도) `A<B` 확신 낮음 —
  `s3_aisle` 의 +0.11 cm 가 7 bag 중 가장 작은 |차| 이면 p 0.031 로 기각, 아니면 가르지 못함. 재녹화 bag 에서 배포 2D 사슬의 발산
  (창 안 xy 최대 > 50 cm) 은 `s6_wall` 을 빼면 0 으로 예측한다.
- **H-RR5 (E5, 결함 없는 wall)**: 위치추정(정답 지도) 에서 배포 2D 사슬은 결함 없는 wall bag 에서도 **출발 전에 위치를 잃는다** (wall − open
  `differs`(+)) — 원래 bag 의 시간 분해에서 이탈 시각의 EKF yaw 표류는 약 1.2° 뿐이었고 방아쇠는 사전 지도에 없는 양끝 화물로 보았다
  **[추정, 결과 문서 §7.2]**. 이 예측이 틀리면 (2D 가 wall 에서 잃지 않으면) 원래 bag 의 2D 이탈은 EKF 결함이 만든 것이다. 3D 는
  `undecided`. 매핑(E5) 은 원래처럼 지지 안 됨 (3D 두 행도 wall 에서 `differs`).
- **H-RR6 (E4, S1 재녹화 둘)**: 점-대-점 ATE 나빠짐 (4/4), 복셀 0.05 max 나빠짐, 0.2 RPE 좋아짐 — 원래와 같은 방향. 결함이 빠지면 0.2 의
  ATE 좋아짐과 비유한 점 제거 끔의 ATE 나빠짐이 4 bag 중 ≥ 3 으로 "바꾼다" 가 될 수 있다 (결함 없는 두 bag 에서 그랬다) — 확신 낮음.
- **H-RR7 (E1)**: 3D 행 네 개 재녹화 bag × 3 재생 모두 rc 0, 궤적 덮음 ≥ 0.99. 처리 스캔 누락은 원래처럼 bag 시작 첫 장(첫 점군 스탬프 ≈
  첫 EKF TF 스탬프인 bag)에서 결정적으로 생길 수 있다 — `s6_wall` 재녹화가 그 꼴이면 다시 3/3.

예측이 틀리면 그대로 적는다. 판정은 아래 규칙만 쓴다.

## 설정

### 1. 녹화 결함 고침 (결과 전, `3e33b46`)

**(a) 정착 뒤 위치추정 기동** — `record_bag.sh` `LOC_AFTER_SETTLE=1` (기본):
1. `multi_robot.launch.py` 를 `with_localization:=false` 로 띄운다 (월드·로봇·브리지·점군 전처리 — 나머지 인자는 원래 그대로).
2. `settle_wait.py`: `imu/data_raw` (원시, 브리지 lazy 아님)의 header 스탬프로 sim ≥ 3.0 s 이고, 마지막 2.0 s(sim) 표본이 기대 수(400)의
   80 % 이상이며 `recgate.imu_still` — 축별 표본 σ 최대 자이로 ≤ 0.005 rad/s · 가속도 ≤ 0.1 m/s², |‖ā‖ − 9.8| ≤ 0.25 m/s² — 를 만족하면
   끝난다 (imu_filter 정지 판정 0.01 · 0.2 · 0.5 의 절반 문턱; 정지 로봇의 가속도 x σ 는 0.043–0.056 m/s² 였다 [계산: `s6_open_a3`·
   `s6_queue_a1` launch.log 의 SE × √12001]). 벽시계 900 s 안에 못 만나면 종료 코드 9 (시도 실패).
3. `localization.launch.py` 를 bringup 이 주는 인자 그대로 따로 띄운다 — `loc_stack_args.py` 가 `amr_bringup.launch_utils` 의
   `stack_arguments`·`stack_extra_arguments`·`frame_prefix` 를 같은 값(목록 앞 1 대, prefixed, use_sim_time, mode odom)으로 부른다 (시험
   `test_loc_stack_args_match_bringup`). DDS 프로파일은 bringup 이 자기 자식에게 준 `FASTRTPS_DEFAULT_PROFILES_FILE` (launch.log 줄)을
   이 런치에만 준다 — 기록기·주행기는 원래처럼 주지 않는다. 로그 `launch_loc.log`.
4. 그 뒤는 원래 프로토콜 그대로 (odom EKF 확인 → 주행기 기록 전 기동 → sim 10 s 정지 → ready → 기록기 → 구독 확인 → go → 출발 전 정지
   → 경로 → 끝 정지 → 기록기 멈춤 → 정리). 바이어스 추정 창은 배포 코드 그대로(60 s 또는 움직임 감지까지) — 결함 없는 5 bag 과 같다.
5. **기록 전 확인**: 주행기 ready 뒤 `launch_loc.log` 에 imu_filter 시작 줄이 없거나 "falling back to parameter bias" 가 있으면 **녹화하지
   않고** 종료 코드 8 (시도 실패로 센다).

**(b)(c) 공백 검사에 규칙 5·6** (`recgate.py`, `bag_tools.py gate --launch-log … --odom-check`):
- 규칙 5 (기동 바이어스): imu_filter_node 의 첫 기동 결과 줄이 성공 문구("startup bias estimated from N samples" 또는 "…; using N samples
  collected before motion")이고 N ≥ 1000, "falling back to parameter bias" 줄 0.
- 규칙 6 (EKF yaw 표류율): bag 의 `odometry/filtered` yaw 와 `ground_truth/odom` yaw 를 각자 첫 표본에서 0 으로 맞춘 오차의 최소제곱 기울기
  |rate| < 0.002 rad/s (`scripts/odom_drift.py` 와 같은 정의).
- 규칙 1–4 (간격·덮음·정지 구간·S6 출발 시각)는 그대로. 실패하면 그 bag 은 쓰지 않고 다시 녹화한다 (시나리오마다 상한 3).
- **도구 검증 (결과 전 — 이미 공개된 bag)** [측정 `B2rerec-validate-gate`, `runs/b24/b2-rerec/validate/`]: 새 gate 를 기존 녹화 8 bag 에 돌렸다 —
  결함 bag 셋(`s1_seed3_a2`·`s1_seed2x_a1`·`s6_wall_a1`)은 규칙 5 (첫 결과 fallback)·규칙 6 (|rate| 0.00998·0.00995·0.00999) 둘 다 실패,
  나머지 다섯은 통과 (|rate| ≤ 0.00041, 바이어스 표본 4 845–12 001). `odom_drift.json` 의 표류율과 6 자리까지 같다.
- 시험 [측정 `B2rerec-test-*`]: `colcon test` amr_slam3d·amr_evaluation 625 개 (ament 린트 포함) 실패 0, 건너뜀 8 (evo — 원래와 같음).
  변이 검사: 새 가드 24 개를 하나씩 되돌림 → **24/24 잡힘** (1 차 22/24 — "첫 결과 fallback" 사유 문구·뒤섞인 odom 정렬을 시험이 못 가렸다 →
  시험을 고쳐 다시, `runs/b24/b2-rerec/mut/`). `settle_wait.py` 의 창 표본 수 가드와 `record_bag.sh` 의 기록 전 grep 은 단위 시험이 없다
  (시운전·본 녹화의 `env.txt`·`settle.json` 으로 확인).
- `amr_localization` 코드는 고치지 않는다 (배포 결함은 결과 문서 §12-1 의 요청으로 남긴다).

### 2. 재녹화 (Gazebo)

| 계획 id | 경로 (`config/routes/`) | 월드 변형 | 스폰 | 씨드 | 바꾸는 슬롯 (분석 이름) |
|---|---|---|---|---|---|
| `rr_s6_wall` | `s6_inside` | `s6-wall` | `config/spawn/s6_inside.yaml` | — | `s6_wall_a1` |
| `rr_s1_seed3` | `s1_seed3` | `s1` | bringup `fleet_spawn_lidar3d.yaml` | 3 | `s1_seed3_a2` |
| `rr_s1_seed2` | `s1_seed2` | `s1` | 같음 | 2 | `s1_seed2x_a1` (셋째 S1 슬롯 — **원래 사전 등록 경로 s1_seed2**) |

- 계획 파일 `config/rerec_plan.txt` (시험 `test_rerec_plan_file_matches_original_scenarios` 가 원래 `rec_plan.txt` 의 같은 줄과 대조). 짧은 S6 벽부터.
- **원래 프로토콜 변경 이력 그대로**: RTF 목표 0.25 (`RTF_TARGET` 기본), 시작 조건 1 분 loadavg < 8 (최대 6 h 대기), 기록기 QoS 깊이,
  출발 전 ≥ 10 s 정지·S6 출발 sim 60 s, 끝 3 s 정지, 파생 bag (`madg_lockstep.py` gain 0.1·0.01 + 오라클 토픽). 다른 것은 위 1 의 (a)–(c)뿐.
- 실행: `rec_driver.sh` (nohup setsid, 재개 가능, `driver.lock`) → `gz_slot.sh b2-rerec` → `rec_attempt.sh` → 컨테이너 `amr-ai-b2-rerec-gz`
  (`amr-ai/slam:0.1`, `--gpus all`, `--shm-size=8g`, `--memory 28g`), DOMAIN 47, `IGN_PARTITION=b2rec_<태그>`, `ROS_LOCALHOST_ONLY=1`.
  사본 `ws_rec` = 이 커밋의 git archive + colcon (amr_bringup·amr_slam3d·amr_evaluation·amr_lidar3d 까지, 13 패키지). 원자료
  `runs/b24/b2-rerec/rec/<태그>/` (원래 시도 디렉터리와 같은 파일 + `launch_loc.log`·`settle.json`·`settle.log`).
- 시도 상한 3 (끊긴 시도도 1 회). 3 회 모두 실패하면 그 시나리오는 원인을 진단해 이 항목에 적고, 분석에서 그 슬롯은 **원래 결함 bag 을
  그대로 두고 "결함 bag" 표시** — 주 결과는 "교체된 bag 만 바꾼 집합" 이고 결함 없는 집합이 아님을 적는다 (아래 4).
- 녹화가 끝나면(통과든 실패든) `runs/b24/b2-rerec/RECORDING_DONE` 을 만든다 (b4-p5b 가 기다린다 — COORDINATION §13).
- 시운전 1 회 (`dry_s6wall`, P 집합 밖, `3e33b46` 사본 `ws_dry`, 시도 상한 1) — 아래 「시운전」.

### 3. 재생 (cpu_slot, 원래 사전 등록 조건)

- 작업 목록 `rerec_plan.py plan` (`eval_driver.job()`·행 목록 그대로): **E2** 재녹화 bag 3 × 행 6 (주 4 + 6DoF 참고 + 오라클 보조) × 재생 3
  = 54, **E3(b)** 3 × 위치추정 행 4 × 재생 3 = 36, **E4** S1 재녹화 둘 × 절제 행 9 × 재생 2 = 36 → **126 재생**. E4 의 다른 두 bag
  (`s3_aisle_a1`·`s1_seed1_a1`) 은 원래 재생을 쓴다.
- 같은 조건 (시험 `test_plan_same_conditions_as_original_replays`·`test_plan_e3_env_matches_original_plan`·`test_rate_rule_matches_original`):
  재생 사본 E2 `ws_e2` (ebf0cea) · E3(b)·E4 `ws_e3b` (a0850e1), E3(b) 지도 = b2-eval `maps/` (map_long 은 다시 녹화하지 않는다 — 같은 지도·
  같은 정답 자세 DB), 배속 = 0.25 / 녹화 RTF (RTF 소수 셋째 자리), DOMAIN 45·46·48, 우선순위 (E2 주 20+k · 참고 30+k · E3(b) 45+k ·
  오라클 55+k · E4 60+k), `replay_row.sh` 그대로.
- 구동기 `eval_driver.py run` (`B2_AGENT=b2-rerec`, 이 커밋 판의 사본 `runs/b24/b2-rerec/bin/`), `cpu_slot.sh b2-rerec` 슬롯 둘, **k = 2 / 슬롯**
  (원래 E2 는 4 로 시작해 규칙으로 01:25 에 2 로 줄였고 E3(b)·E4 는 처음부터 2 — 여기서는 처음부터 2), 부하 규칙 (1 분 loadavg − 우리 몫)
  ≤ 48, 기동 실패(rc 4·125–127)만 다시 (상한 3). 신호 파일 `go/e3`·`go/e4` 는 이 커밋 뒤에 만든다.
- 메모리: COORDINATION §12 — 재생 컨테이너 명령은 원래와 같고(`--memory` 없음) `mem_guard.sh` 가 10 g 상한을 건다. 상한에 걸려 죽은 재생은
  무효로 보고 다시 돈다 (기록).
- 재생마다 실행 목록 한 줄 (`B2rerec-<단계>-<태그>-<행>-r<k>`, 실패 재생 포함).

### 4. 분석 (원래 사전 등록 스크립트 그대로, 결함 bag 교체)

- **분석 보기** (`rerec_plan.py view`, `runs/b24/b2-rerec/view/`): 슬롯 이름 = 원래 bag 이름의 심볼릭 링크. 결함 없는 기존 5 bag
  (`s3_aisle_a1`·`s1_seed1_a1`·`s6_open_a3`·`s6_queue_a1`·`map_long_a1`)은 원래 원자료(녹화·재생·캐시)를, 슬롯 셋은 재녹화 bag 과 그 재생을
  가리킨다. peval·presults 는 bag 이름을 디렉터리 이름으로 읽고 이름별 규칙(S6 변형·블로커 가림·통로 상자)을 쓰므로 고치지 않고 그대로 돈다.
- 명령 (b2-eval `tmp/analyze.sh` 와 같다): 캐시 `peval.py cache` (재녹화 슬롯 셋), E2 `peval.py replays --stage e2 --grid-band` →
  `compare`, E3(b) `replays --stage e3 --no-grid --warmup 2.0 --bags <7>` → `compare-loc` + `loc_lost.py`, E4 `replays --stage e4
  --window-stage e2 --grid-band` → `compare-abl` (기준 = E2 의 `rtabmap3d_noimu`), 표 `presults.py --rec view/rec`.
- **판정 집합 (결과 전 결정)**:
  - **주 결과 = 교체 8 bag** (스크립트 라벨 `ALL8` · E3(b) `ALL7` · E4 4 bag 규칙 "≥ 3/4"). 8 bag 이 모두 사전 등록 시나리오다 — 원래 S1
    규칙(완성 계획 §6)의 이유였던 사전 등록 밖 bag `s1_seed2x` 가 빠지고, 그 자리의 `s1_seed2` 재녹화는 원래 사전 등록 경로를 이 항목에서
    녹화 전에 사전 등록한 것이다.
  - 스크립트가 함께 내는 `PRE7`·`PRE6`·E4 "3 bag" 은 이제 "**s1_seed2 재녹화 제외**" 집합이다 (슬롯 이름 `s1_seed2x_a1` 을 빼므로). 보조로
    나란히 적고, 주 결과와 판정이 다르면 그 사실을 불리한 것으로 적는다 (주 결과는 바꾸지 않는다).
  - S1 만 (3·2 bag) 은 원래처럼 기각 불가능한 설계 — 기술만.
  - 가족·지표·δ·검정(Wilcoxon 정확 + Holm, 두 단계 부트스트랩 10 000 회 seed 20261008)·실패 규칙·창 규칙·E5 `replay_rule`·E4 기술 판정·
    명세 구간 표 — 모두 원래 그대로.
  - **기존 결과(결함 포함)** 는 지우지 않고 결과 문서에 나란히 둔다 (원래 판정 = "처음 결과", 교체 8 bag = "재녹화 결과").
  - 재녹화가 3 회 모두 실패한 슬롯이 있으면: 주 결과 = 그 슬롯만 원래 결함 bag 인 8 bag (표시), 보조 = 결함 bag 을 뺀 집합 (기술).
- **보기 일치 검사 (결과를 보기 전에 정한 정지 규칙)**: 보기 위에서 다시 낸 기존 5 bag 의 재생별 지표(`ate_rmse_cm`·`map_adnn_surf_cm`·
  `fail`)가 원래 `analysis/{e2,e3,e4}/replays.jsonl` 의 같은 재생과 같아야 한다 (같은 코드·같은 원자료·같은 창). 다르면 분석을 멈추고 원인을
  찾는다.
- E4 지연: 원래 판정(기준 = E2 재생)은 측정 시각 혼입이 있었다. 재녹화 두 bag 은 E2 기준 재생과 E4 절제 재생이 같은 구동기 실행 안에서
  돈다 — 그래도 판정 규칙은 원래 그대로이고, 원래의 보충 기준("같은 기간 무관 손잡이 네 행 평균", 결과 뒤 정한 것)도 같은 셈으로 함께 낸다
  (보충 표시 유지).

### 5. 판정표 갱신 규칙 (E1–E6)

완료 정의(완성 계획 §3)는 바꾸지 않는다. 재녹화 결과로 '부분' 의 이유가 없어지면 판정을 올리고, 남으면 그대로 둔다:
- E1: 3D 행이 끝까지 돌고 판·설정 표가 있다 + **처리 스캔 누락 0 (또는 원인과 함께 수)** — 누락이 남으면 원인(시작 장·부하)과 수를 적는다.
- E2·E3·E5: 결함 bag 이 주 결과에 남지 않으면 '부분' 의 결함 이유가 없어진다. 다른 이유(예: ADNN₀ 도구 편향 — 사전 등록 도구 값을 주로 둔 것)는
  따로 적는다.
- E4: 절제 bag 4 개가 결함 없는 bag 이 되면 결함 이유가 없어진다. 지연 기준 시각 혼입은 원래 두 bag 에 그대로 남는다.
- E6: 과목 형식 요약은 사용자 양식 확인 전이라 '부분' 그대로.

### 시간 어림

녹화: S6 벽 약 6 분 + S1 씨드 3 약 16 분 + S1 씨드 2 약 21 분 (+ 기동·정착·사후 각 3–5 분). 재생 126: 직렬 약 31 h (sim 구간 / 0.25 + 75 s)
→ 동시 4 로 약 8 h (+ 부하 대기). 분석 약 1.5 h (E2 의 기존 5 bag 재계산 포함).

## 시운전 (P 집합 밖 — 바뀐 기동 절차 확인) [측정, `B2rerec-dry_s6wall_a1`]

원자료 `runs/b24/b2-rerec/dryrun/rec/dry_s6wall_a1/` (사본 `ws_dry` = `3e33b46`, 00:47–00:55 KST, 1 분 loadavg 4.1 → 4.2 — 서버 한가).
`s6_inside` 경로 + `s6-wall` 월드 (원래 시운전과 같은 조합 — 통로 안 스폰·후진·출발 시각·블로커가 모두 걸린다).

- 바뀐 기동 절차가 끝까지 돌았다: 월드·로봇 기동 7 s 뒤 `imu/data_raw` → 정착 판정 첫 검사(sim 3.0 s)에서 통과 (벽시계 15 s; 마지막 2 s 창
  자이로 σ 2.9e-4 rad/s, 가속도 σ 0.024 m/s², ‖ā‖ 9.70 m/s²) → 위치추정 런치 (`[localization] mode=odom ns=/amr_01 prefix="amr_01/"
  config=/ros2_ws/config` — 원래 launch.log 의 줄과 같다, 같은 네 노드) → 기록 전 바이어스 확인 `ok` → 기록기 14/14 구독 → 출발 sim 60.14 s.
- **공백 검사 규칙 1–6 모두 통과**: 규칙 5 = "startup bias estimated from 12001 samples … over 60.0 s" (자이로 z −0.009989 rad/s), 규칙 6 = EKF yaw
  표류율 1.2e-6 rad/s (yaw 오차 최대 0.013°). RTF 0.248. 처음 정지 45.8 s (원래 S6 bag 49.2–49.5 s — 위치추정을 sim 약 3 s 늦게 띄워 주행기
  ready 가 그만큼 늦다), 끝 정지 5.0 s.
- 관찰: 바이어스 추정 60 s 가 출발(sim 60.14 s) 뒤에 끝났다 (imu_filter 첫 표본이 sim 약 3–4 s) — 움직임 감지가 직진 출발을 곧바로 잡지 못해
  출발 뒤 약 3 s 표본이 평균에 들어갔을 수 있다 (배포 코드의 감지 문턱 — 결함 없는 원래 bag 들도 출발 뒤 약 2–22 s 를 넣었다: s1_seed1 출발 23.9 s · 추정 25.8 s, map_long 22.1 · 24.2, s3_aisle 23.1 · 45.1 [측정 launch.log·gate.json]).
  규칙 6 의 표류율은 1.2e-6 rad/s 로 영향이 보이지 않았다. 프로토콜은 바꾸지 않는다.
- 시운전 bag 은 P 집합에 넣지 않는다.

## 실행 중 기록 (재생 지표를 보기 전 — 녹화·파생·구동기 상태만)

### 녹화 (00:57–01:41 KST, 사본 `ws_rec` = `af63345`) [측정, 서버 1 분 loadavg 2.0–8.9 — 다른 사용자 openroad·ruby 동시]

| 태그 (슬롯) | 시도 | 규칙 1–6 | RTF | sim 구간 s | 처음 / 끝 정지 s | 출발 sim s | 기동 바이어스 (규칙 5) | EKF yaw 표류율 (규칙 6) |
|---|---|---|---|---|---|---|---|---|
| `rr_s6_wall_a1` (`s6_wall_a1`) | 1 | 통과 | 0.249 | 80.5 | 45.8 / 5.0 | 60.14 | 전체 60 s · 12 001 표본, 자이로 z −0.010005 | 2.1e-6 rad/s (yaw 오차 최대 0.023°) |
| `rr_s1_seed3_a1` (`s1_seed3_a2`) | 1 | 통과 | 0.249 | 224.3 | 10.8 / 4.0 | 25.18 | 움직임 전 4 859 표본 (24.3 s), z 0.010005 | −7.3e-6 (0.31°) |
| `rr_s1_seed2_a1` (`s1_seed2x_a1`) | 1 | 통과 | 0.249 | 260.8 | 11.0 / 4.0 | 25.18 | 움직임 전 4 859 표본 (24.3 s), z −0.010001 | −4.7e-7 (0.26°) |

- **H-RR1 맞음**: 세 시나리오 모두 1 회차 통과, 파라미터 바이어스 줄 0. **H-RR2 맞음**: |표류율| ≤ 7.3e-6 rad/s (결함 bag 0.0100 의 1/1 000 아래).
- **H-RR0 의 직접 증거** [측정 `rec/*/settle.json` `first20`·`first100`]: 위치추정을 띄우기 전 받은 첫 IMU 표본 20 개(sim 0.002–0.095 s)는
  세 녹화 모두 정지가 아니었다 — 평균 가속도 크기 2.85·3.05·2.86 m/s² (g 9.8), 가속도 σ 4.6 m/s² (스폰 z 0.02 m 에서 떨어지는 자유 낙하·착지).
  첫 100 개(sim 0.5 s)도 ‖ā‖ 8.33–8.53, σ 3.4 m/s². 원래 프로토콜에서 imu_filter 가 sim 0.5 s 안에 첫 표본을 받으면 첫 판정(20 개)이 이
  구간에 걸려 "motion detected … falling back to parameter bias" 가 된다. 정착 판정은 세 녹화 모두 첫 검사(sim 3.0 s)에서 통과했다 (벽시계 15–17 s).
- 원래 S6 bag 과 다른 점: 처음 정지 45.8 s (원래 49.2–49.5 s — 위치추정·주행기를 sim 약 3 s 늦게 띄움), 원자료 점군 805 장 (원래 840–843).
  S1 은 출발 25.18 s (원래 21.5–23.9 s). 둘 다 규칙 3·4 안이다.
- `rr_s1_seed3_a1` 에서 기록기가 `imu/data_raw`·`points_raw`·`scan` 을 0.245–0.26 s 늦게 구독했다 (덮음 규칙 2 의 0.5 s 안 — 통과).

### 파생 bag 실패와 도구 고침 (결과 뒤 변경 — 파생 도구, 재생·SLAM 결과 전)

- `rr_s1_seed3_a1` 의 녹화 직후 파생(`madg_lockstep.py`)이 "bag 끝의 짝 없는 imu/data 가 너무 많다 (앞 49, 뒤 0)" 로 멈췄다 (위의 0.245 s 늦은
  `imu/data_raw` 구독 — 짝 없는 imu/data 49 개 = 0.245 s). `record_bag.sh` 는 파생 실패를 녹화 판정에 넣지 않는다 (원래 설계 — "따로 다시 만든다").
- 고침 `fbc4ad0`: `EDGE_UNPAIRED_MAX` 10 → 100 — 공백 검사 규칙 2 가 토픽마다 bag 처음·끝 0.5 s 까지의 덮음 부족을 통과시키므로 파생도 같은
  0.5 s (200 Hz 의 100 개)까지 받는다. 짝 없는 끝 표본에서는 오라클 방위 토픽만 빠진다 (재계산 madgwick 은 imu/data 전부). 원래 8 bag 의
  끝 짝 없음은 0–1 개라 그 파생은 바뀌지 않는다. 10 은 원래 b2-rec 가 s3_aisle 1 개를 보고 정한 값(역시 결과 뒤 변경)이었다.
- 다시 파생 [측정 `B2rerec-derive-rr_s1_seed3_a1`]: 출력 44 853 = imu/data 수, 오라클 짝 없음 앞 49·뒤 0, 예상 밖 출력 0. 실패한 첫 파생 로그는
  `rec/rr_s1_seed3_a1/derive_failed_3e33b46.log`. 다른 두 bag 의 파생은 녹화 때 그대로 됐다 (짝 없음 0·0).
- 영향: `liosam_oracle` (보조, 순위·검정 밖) 행만 이 bag 에서 오라클 토픽이 0.245 s 늦게 시작한다 (bag 처음 ≥ 10 s 정지 구간 안).

### 재생 구동기

- 01:43 `eval_driver.py run` (`B2_AGENT=b2-rerec`, `bin/eval_driver.py` = `af63345` 판), 작업 126. 녹화 구동기의 `driver.log` 는 재생 구동기와
  이름이 겹쳐 `rec_driver.log` 로 옮겼다 (내용 그대로).
- **작업 목록 고침 (재생 0 건일 때)**: 파생 실패를 보고, 모든 작업의 `requires` 에 그 bag 디렉터리의 `metadata.yaml` 을 더했다 — bag 이 없는
  작업을 집으면 docker 가 빈 디렉터리를 만들어 재생이 실패로 끝나고 다시 돌지 않는다 (기동 실패 rc 만 다시). 조건(사본·배속·행·우선순위)은
  그대로다. 그 9 작업(`rr_s1_seed3_a1` 의 LIO-SAM 세 행 × 재생 3)은 다시 파생한 뒤 풀렸다.
- 01:43 시작 때 1 분 loadavg 102 (b4-p5b 의 5 대 Gazebo 기동 — 우리 몫 0) → 사전 등록 부하 규칙(다른 부하 ≤ 48)으로 대기.

## 결과

(재생 뒤 채운다)

## 해석

## 다음
