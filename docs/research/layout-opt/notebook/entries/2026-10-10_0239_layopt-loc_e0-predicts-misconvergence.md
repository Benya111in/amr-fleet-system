# 실험 1 — E0 마진이 재초기화 오수렴을 예측하는가 (H1, 게이트 A)

- 시각: 2026-10-10 02:39 KST · 에이전트: layopt-loc · 단계: exp1
- 코드: `76eb27c` (브랜치 `feature/layopt-loc`)
- 관련 실행 id: (runs/<에이전트>.jsonl 의 run_id 목록)

## 질문
지도만으로 구한 E0 위험 점수(체류 자세에서 이웃 k대를 넣은 가상 스캔의 앨리어싱 마진)가 Gazebo 단일 로봇 E2 시행에서
전역 재초기화 오수렴을 판별하는가 (prereg H1, 게이트 A).

## 가설과 예측
- H1 (prereg §2, 숫자 고정): −margin 의 오수렴 AUROC ≥ 0.8 이면 지지, 0.7–0.8 이면 약한 지지, < 0.7 이면 기각.
  기각이거나 판정이 불가하면(오수렴 0 건 또는 전부 오수렴) 게이트 A 는 미통과다. 그때 R_loc 는 E2 측정표(자세 종류 × k)로 바꾼다.
- 결과를 보기 전의 예측:
  - 마진 < 0.05(converge_margin)인 자세는 시드 #1(참)이 여백 게이트에 막힌다.
    그 뒤 select_seed 가 인라이어 차 < 0.15 인 2–3 순위(거울·주기 별칭)를 심고, 끝내 AMCL 전역으로 간다.
    그래서 오수렴이 잦을 것이다. 마진이 큰 자세는 첫 회전에서 정수렴할 것이다.
  - 그러므로 AUROC 는 높을 것으로 예상한다(≥ 0.8).
  - 다만 운용 inlier 는 참 자세에서 이웃 가림에 무감하다(loc.md §1.2). 그래서 자연 단계 LOST 는 드물 것이다.
    P(LOST | inlier) 는 추정되지 않을 수 있다.
- 파일럿 근거 (SLAM 지도, 실험에 넣지 않음): 대기 슬롯 4/4 오수렴, 자유 공간 2/2 정수렴
  (e2/20261009T173623Z_pilot-slam-dbg3). 표본이 작아 예측의 근거로만 쓴다.

## 설정
- 배치: 기준 묶음 `v10-v20-v30-v40-v50`(래스터 지도 sha256 f266190e…, world.sdf = warehouse.sdf 바이트 동일, 생성기 2b71c30).
  E0 와 E2 가 같은 지도를 쓴다.
- 시행 목록: `layopt/loc/exp1_trials.json` (`make_trials.py exp1`, 시드 20261010) — **이 항목과 함께 커밋한다(사전 등록)**.
  - 자세 79개: 대기 5, 도크 4, 충전 3, queue 12(46 중 시드로), 자유 공간 55.
    자유 공간은 후보 368개의 E0 k=0 마진 5분위에서 11개씩 뽑는다. 분위 경계는 0.0697/0.0969/0.1258/0.1663 이다.
  - 자세 × k ∈ {0, 2, 4} = 237 시행, 순서는 시드로 섞는다.
  - 이웃: 체류 지점은 layout 의 neighbors 앞 k 개(모자라면 있는 만큼, k_used), 자유 공간은 e0.free_space_neighbors 다.
- E2 절차: `docs/research/layout-opt/loc.md` §2.
  - 운용 스택 system.launch.py: localization + navigation + perception, behavior 끔.
  - 자연 20 s → 정합 거부 주입 강제 LOST → 120 s 복구 창.
- 판정 (prereg §3):
  - 판정 시각은 TRACKING 선언 또는 LOST + 120 s 이다. 그 시각 odometry/filtered_map 대 ground_truth/odom 위치 오차를 쓴다.
  - < 0.5 m 정수렴, > 1.0 m 오수렴, 그 사이 모호.
- 분석 (`layopt/loc/analyze_exp1.py`, 같은 커밋):
  - y_mis = 오수렴. 모호는 0 이고, 민감도 분석에서는 뺀다.
  - 위험 점수 = −margin(같은 자세·k·이웃의 E0).
  - AUROC 의 95% CI 는 자세 단위 군집 부트스트랩 2,000 회(시드 20261010)로 구한다.
  - 로지스틱: P(오수렴 | margin), P(자연 LOST | inlier).
  - 표: 자연 LOST 율과 오수렴 율을 k별·자세 종류별로 낸다(Wilson 구간).
- 실행:
  - 명령: `python3 layopt/loc/e2_campaign.py --trials layopt/loc/exp1_trials.json --tag exp1 --per-session 20`
    (gz_slot.sh layopt-loc, DOMAIN 57, amr_ai_layopt_e2, 이미지 amr-ai/rl:0.3).
  - 세션당 20 시행이고, 이상이 나면 새 세션으로 시도 2 회까지 간다.
- 멈춤 규칙: 237 시행을 섞인 순서로 끝까지 돈다. 시간이 모자라면 done ≥ 150 에서 멈추고 done 전부로 분석한다
  (순서가 무작위라 부분 집합도 무작위다). 멈추게 되면 그 시각과 이유를 일지에 적는다.
- 공유 부하 아래의 RTF 와 시간 수치는 예비값이다. 시행마다 loadavg 를 남긴다.

## 결과
(표: 시드별 값, 실패 시드 수. [측정]/[계산]/미측정 표기. 원자료 경로)

## 해석
(결과가 말하는 것과 말하지 않는 것. 공유 부하 하 예비값 여부)

## 다음
(이 결과로 바뀌는 결정, 남은 질문)
