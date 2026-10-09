# 배치 묶음 생성기 · 기준 배치 회귀 · 하네스 연결

- 시각: 2026-10-10 02:56 KST · 에이전트: layopt-gen · 단계: W1-W3 (+W6 하네스)
- 코드: 생성기 `d3a3d22` (월드 --spec) · `2b71c30` (layopt/gen) · `10a3103` (수준 수정) · `96a8883` (하네스 --bundle) · `83202d2` (전수 생성 시점), 브랜치 `feature/layopt-gen`
- 관련 실행 id: `LOGEN_bundle_baseline_20261009T174428Z`, `LOGEN_mapq_baseline_20261009T174449Z`, `LOGEN_bundles_20261009T175800Z`, `LOGEN_20261009T175916Z_v10-v20-v30-v40-v50_LOsmoke`, (재시도 — 아래 결과)

## 질문

배치 사양 하나에서 Gazebo 월드·점유 지도·운용 설정을 한 가지 규칙으로 만들 수 있는가?

이 질문은 두 조건으로 나눠 확인한다.
1. 기준 배치에서 저장소 원본을 그대로 재현한다 (design W1·W3 완료 판정).
2. 그 묶음으로 5 대 Gazebo 스택이 실제로 뜬다.

## 가설과 예측

결과를 보기 전에 적었다.

- 예측 1: 생성기 리팩터가 출력 순서와 난수 소비를 바꾸지 않으면 `warehouse.sdf` 가 바이트 단위로 같다.
- 예측 2: 손으로 쓴 `traffic_zones.yaml` 머리말의 규칙을 일반화하면 구역 17 개와 포켓이 원본과 해석값 차이 0 이다.
  - 규칙: 틈 가운데 2 m, 랙 앞 0.6 m, 도크 ±0.75 m.
  - 차이가 나면 그 차이를 문서에 적고 무해한 이유를 보인다.
- 예측 3: 래스터 지도는 SLAM 지도와 표면 위치(ADNN)가 몇 cm 안에서 맞는다.
  - 래스터는 물체 속을 채우고 SLAM 은 면만 남기므로 IoU 는 낮게 나올 것이다.
- 판정 1·2 가 실패하면 기준 배치 묶음을 Gazebo 에 올리지 않는다 (design §5 "W3 회귀 시험 통과 전에는 새 배치를 Gazebo 에 올리지 않는다").

## 설정

- 생성기와 검사: `layopt/gen/`. 컨테이너 `amr-ai/rl:0.3`, `--network none`.
- 비교: `layopt/gen/compare_baseline.py`.
- 지도 비교: `amr_localization.map_quality` 를 SLAM 지도와 묶음 지도에 각각 돌렸다. `--world` 는 묶음 월드다.
- 전수 생성: `cpu_slot.sh layopt-gen layopt/gen/run_build.sh all --workers 6`.
- 하네스: `p5_driver.py --bundle … --run-one … --order LOsmoke --domain 59 --partition amr_ai_layopt_e3`. gz_slot `layopt-gen`, 작업공간 `runs/layopt/tmp/gen/ws/96a8883`.

## 결과

- [계산] 기준 묶음 비교:
  - `world.sdf` 가 `warehouse.sdf` 와 바이트 단위로 같다.
  - `behavior.yaml` · `fleet_spawn.yaml` · `traffic_zones.yaml` 해석값 차이 0.
  - 구역 17, 포켓 37. 과제 지시문의 "36" 이 아니라 원본 파일도 37 이다.
- [계산] 지도 (`runs/layopt/tmp/gen/map_quality/20261009T174449Z_baseline/`):

| 비교 | ADNN | p95 | IoU_τ | precision / recall | 점유 비율 |
|---|---|---|---|---|---|
| SLAM 지도 vs 월드 GT | 0.0258 m | 0.077 | 0.575 | 0.759 / 0.741 | SLAM 0.0213 |
| 래스터 지도 vs 월드 GT (자기 일치) | 0 | — | 0.998 | — | 래스터 0.0379 |

- [계산] 전수 540: 유효 500, 무효 40.
  - lane 36: 북쪽 대기 × 그 옆 충전소. staging 이 지게차 차선에서 0.64 m 다.
  - resolve 4: 구조물 자리가 없다.
  - 30 s 걸렸다. 부하 36/33/28.
- 시험 생성(묶음 아님)이 결함 두 개를 찾았고, 수준을 사전 등록하기 전에 고쳤다 (`10a3103`).
  1. 북쪽 벽 split_docks 충전소 180 개가 전부 무효였다.
  2. B 열을 옮긴 배치에서 동측 보행자 경로가 기둥 열 밖으로 옮겨져 동쪽 대기 슬롯과 겹쳤다.
- [측정] 스모크 1 실패 (CLOCK_STALL).
  - 원자료: `runs/layopt/tmp/gen/e3_smoke/20261009T175916Z_v10-v20-v30-v40-v50_LOsmoke`.
  - 묶음 겹침 sha256 5/5 일치. 스택·서버가 기동했다.
  - Gazebo 는 계속 돌았다 (ign clock sim 55 s, RTF 0.31). 그런데 ROS `/clock` 이 sim 15.4 s(벽시계 18:02:50 UTC) 이후 DDS 구독자에게 오지 않았다.
  - 기록기 180 s 감시로 끝났다 (rc 4). 벽시계 332 s.
  - 같은 로그에 SHM `open_and_lock_file failed` 9 건과 amr_02 Nav2 bringup 중단이 있다. 유효했던 P5 실행 3 개에는 이 SHM 오류가 0 건이다.
  - 부하: 시작 27/32/29, 중간 1 분 값 95, 끝 42/61/46.
    - 다른 사용자 openroad 가 ~11 코어를 썼다.
    - layopt-sim 의 E1 과 layopt-loc 의 E2 Gazebo 가 함께 돌았다.
- 스모크 재시도: 아래 "다음" 에 결과를 덧붙인다.

## 해석

- W1·W3 의 완료 판정은 통과다. 구역·포켓 규칙을 일반화해도 기준 배치에서 손 작성본이 그대로 나왔다. 같은 규칙을 540 개 배치에 썼다.
- 지도: SLAM 지도와 래스터 지도의 표면 위치 차는 2.6 cm (ADNN) 다. IoU 0.575 는 대부분 "속 채움 대 면" 차이다. 측위 결과에 주는 영향은 **미측정** 이다 (layopt-loc E2 의 두 지도 반복 몫).
- 스모크 1 의 실패는 묶음 내용 때문이라는 증거가 없다.
  - 겹침은 확인됐고, 월드가 기준과 같다.
  - /clock 정지는 P5 의 알려진 실패 형태다.
  - 다만 이번에는 SHM 오류가 함께 나왔다. 원인은 부하와 도메인 59 포트 번호 중 어느 쪽인지 가르지 못했다.

## 다음

- 부하 조건을 지키며 스모크를 다시 돈다 (드라이버 기본 시작 조건).
- 다시 /clock 정지가 나면 원인을 가르기 위해 다음 둘을 남긴다.
  - 같은 조건에서 묶음 없이 저장소 기본 배치로 한 번 돈다 (도메인 59).
  - 도메인만 바꿔 한 번 돈다.
