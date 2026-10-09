# layopt 배치 묶음 생성기와 5 대 Gazebo 실행 (layopt-gen)

작성 2026-10-10 (layopt-gen). 계약은 `contract.md`, 수준은 `levels.md` (사전 등록), 설계는 `design.md` §5 W1–W3·W6 이다.
코드는 `layopt/gen/` (생성기·검사) 와 `tests/integration/fleet_p5/` (Gazebo 하네스) 에 있다.

## 1. 한눈에

| 무엇 | 어디 |
|---|---|
| 묶음 540 개 (유효한 것만 월드·지도·설정이 있다) | `/home/CAPYI_AI/runs/layopt/bundles/<spec_id>/` — 봉인 (SHA256SUMS, SEALED) |
| 기준 배치 묶음 | `/home/CAPYI_AI/runs/layopt/bundles/v10-v20-v30-v40-v50/` (생성기 2b71c30) |
| 5 대 실행용 빌드 작업공간 (git archive 고정본) | `/home/CAPYI_AI/runs/layopt/tmp/gen/ws/96a8883/` |
| 묶음 하나 만들기 | `layopt/gen/run_build.sh one <spec_id>` |
| 전부 만들기 | `/home/CAPYI_AI/env/cpu_slot.sh layopt-gen layopt/gen/run_build.sh all --workers 6` |
| 5 대 실행 한 번 | `layopt/gen/run_e3.sh <spec_id> [LO\|LOd\|LOsmoke]` (§5) |

## 2. 생성 경로

```
spec_id ─ levels.resolve ─▶ 배치 사양(LayoutSpec, 생성기 default_layout() 과 같은 키)
            │                    └─ gen_warehouse_world.build_sdf ─▶ world.sdf ─ raster ─▶ map.pgm/map.yaml
            ├─ zones.derive ─▶ config/traffic_zones.yaml
            ├─ dwell.dwell_points ─▶ layout.json dwell_points
            ├─ configs ─▶ config/behavior.yaml, config/fleet_spawn.yaml
            └─ validate.check ─▶ valid.json
```

- **월드 생성기** (`src/amr_simulation/worlds/gen_warehouse_world.py`):
  - `--spec <json>` 으로 배치 사양을 받는다. 사양이 없거나 현행 값이면 출력은 커밋된 `warehouse.sdf` 와 바이트 단위로 같다 (`test_world.py`, `layopt/gen/tests/test_generator_spec.py`).
  - `build_sdf()` 는 난수 상태를 처음부터 다시 시작한다. 그래서 한 프로세스에서 여러 번 불러도 같은 사양이면 같은 바이트가 나온다.
- **지도** (`raster.py`):
  - `amr_localization/world_geometry.py` 로 월드 시각체를 스캔 높이 0.20 m 에서 자른다. 해상도 0.05 m, 범위 [−30.5, 30.5] × [−20.5, 20.5] (world_to_map 기본).
  - 움직이는 모델은 뺀다: `DEFAULT_EXCLUDE` + `shuttle_amr`. 셔틀이 DEFAULT_EXCLUDE 에 없어 출발 자세에 상자로 찍히기 때문이다.
  - 외벽 바깥 면 밖은 미지(205)로 쓴다. 기준 배치도 같은 방식으로 만든다.
- **설정**:
  - `behavior.yaml` 은 저장소 원문에서 배치가 정하는 줄만 바꾼다. 바꾸는 줄: 충전소 3 개의 staging·marker_pose, `waiting_pose`, 위치 표지 74 개.
  - 도크 4 개와 다른 튜닝값·주석은 원문 그대로 둔다.
  - `waiting_pose` 는 대기 패드 중심 [x, y, 패드 yaw] 다. 원문의 [22, −16, 0] 과 같은 규칙이다. 다중 로봇은 런치가 로봇마다 자기 슬롯으로 덮어쓴다 (`launch_utils.stack_arguments`).
- **교통 구역·포켓·경로·체류 지점의 규칙**은 `levels.md` §4 에 있다.
- **월드 이름 주의**: `world.sdf` 의 `<world name="warehouse">` 는 그대로다. 이 파일을 `warehouse.launch.py world:=<경로>` 로 직접 주면 런치가 월드 이름을 파일 이름(`world`)으로 잡는다. 그러면 스폰·world_gate 가 `/world/world/...` 를 찾다가 실패한다.
  - 해결 1: 파일을 `warehouse.sdf` 라는 이름으로 복사하거나 링크해서 준다.
  - 해결 2: P5 하네스처럼 `src/amr_simulation/worlds/warehouse.sdf` 위에 겹쳐 마운트한다.

### `layout.json` 필드 (contract §2 + 정보 필드)

- contract 필드: `spec_id, bounds, racks, pillars, structures, docks, chargers, waiting_slots, narrow_aisle, markers, dwell_points`.
- 정보 필드:
  - `racks[].kind` (main | narrow), `racks[].row`
  - `chargers[].{yaw, marker_pose, marker_id, docked}`
  - `dwell_points[].docked` (dock·charger 의 도킹 완료 자세)
  - `levels, level_keys, waiting_zone, traffic, dynamic_lanes, notes`
  - `gen_spec` — 생성기에 준 배치 사양 그대로. 이 사양으로 `world.sdf` 를 다시 만들 수 있다.
- `markers` 는 81 개다: 도크 4, 충전 3, 기둥 32, 랙 42.

### `valid.json`

`{spec_id, valid, reasons: ['<종류>:<내용>', …], reason_kinds, info}`. 종류 정의는 `levels.md` §5 에 있다.

`info` 의 내용:
- `pose_clearance_m`: 슬롯·staging 의 지도 여유
- `pose_lane_clearance_m`: 동적 경로까지 거리
- `n_zones, n_pockets, spacing_conflicts, n_dwell, reach_components, occupied_cells`

무효 묶음은 `spec.json`, `layout.json`(해석됐을 때), `valid.json` 만 두고 봉인한다. 월드·지도·설정이 없으므로 실행 대상이 아니다.

## 3. 기준 배치 회귀 (design W1·W3 완료 판정, W2 보고)

| 대조 | 결과 |
|---|---|
| `world.sdf` ↔ `warehouse.sdf` | 바이트 단위로 같다 |
| `behavior.yaml`, `fleet_spawn.yaml` ↔ 원본 | yaml 해석값 차이 0 |
| `traffic_zones.yaml` ↔ 원본 (손 작성) | 해석값 차이 0. 구역 17, 포켓 37 |

시험은 `layopt/gen/compare_baseline.py` 와 `tests/test_bundle.py` 다.

### 래스터 지도 ↔ SLAM 지도 (W2) — [계산]

`map_quality.py` 를 두 번 돌렸다. 원자료는 `runs/layopt/tmp/gen/map_quality/20261009T174449Z_baseline/` 이다.

| 지표 | SLAM 지도 (maps/warehouse.yaml, 2026-09-22) vs 월드 GT | 묶음 래스터 지도 vs 월드 GT |
|---|---|---|
| ADNN 정렬 전 / 후 [m] | 0.0259 / 0.0258 | 0.0000 / 0.0000 |
| ADNN 중앙 / p95 [m] | 0.0155 / 0.0770 | 0 / 0 |
| Chamfer [m] | 0.0367 | 0.0035 |
| IoU_τ / precision / recall (τ = 1 셀) | 0.575 / 0.759 / 0.741 | 0.998 / 1.000 / 0.998 |
| 점유 / 자유 / 미지 비율 | 0.0213 / 0.9544 / 0.0243 | 0.0379 / 0.9378 / 0.0243 |
| 랜드마크 중심 오차 평균 / 최대 [m] (n=40) | 0.045 / 0.128 | 0 / 0 |

읽는 법:
- map_quality 의 GT 는 래스터와 같은 방식(world_geometry 0.20 m 단면)이다. 그래서 왼쪽 열이 곧 "SLAM 지도 ↔ 래스터 지도" 차이다.
- 오른쪽 열은 자기 일치 검사다. recall 0.998 이 1 이 아닌 이유는 하나다. map_quality GT 는 `shuttle_amr` 를 빼지 않고, 묶음 지도는 뺀다.
- 래스터는 물체 단면을 속까지 채운다 (점유 비율 0.038). SLAM 지도는 LiDAR 가 본 면만 점유로 둔다 (0.021).
  - IoU_τ 0.575 의 대부분이 이 차이다. 표면 위치는 ADNN 2.6 cm 로 맞는다.
  - 가능도장(AMCL)과 전역 시드 정합은 가장 가까운 점유 셀까지 거리를 쓰므로 속 채움의 영향이 작다고 본다 — **미측정**.
  - layopt-loc 의 E2 가 두 지도로 같은 시행을 반복해 차이를 잴 계획이다 (design §9).

## 4. 하네스 연결 (W6)

`tests/integration/fleet_p5/p5_driver.py` 에 옵션을 더했다. 옵션이 없으면 예전과 같다 — `layopt/gen/tests/test_harness.py` 가 기존 실험 계획 전부와 드라이버 기본값을 확인한다.

- `--bundle <묶음>`: 다섯 파일을 묶음 것으로 바꾼다.
  - 스택·서버 컨테이너의 소스 트리 파일 위에 묶음 파일을 읽기 전용으로 겹쳐 마운트한다. 대상은 `world.sdf` → `src/amr_simulation/worlds/warehouse.sdf`, `config/behavior.yaml`, `config/fleet_spawn.yaml`, `config/traffic_zones.yaml` 이다.
  - 작업공간이 `--symlink-install` 빌드라 다음 노드가 모두 묶음 파일을 연다.
    - `install/.../share/.../*.yaml` 을 여는 노드: behavior.launch.py 의 params_file, 분산 에이전트 zones_file
    - 소스 트리 기본값을 여는 traffic_manager (`zones_file: ""`)
    - 월드를 여는 warehouse.launch.py, obstacle_truth_node
  - 지도는 `map_yaml:=/bundle/map.yaml`, 스폰 목록은 `robots_file:=/bundle/config/fleet_spawn.yaml` 런치 인자로 준다. 대기 자세는 로봇마다 슬롯이 된다.
  - 기록기는 묶음 구역·behavior 파일을 `meta.json` 에 넣는다. 분석기는 그 원문을 쓴다.
  - 확인: `p5_container.sh` 가 노드가 실제로 여는 경로(symlink 를 따라간 곳)의 sha256 을 묶음 파일과 비교해 `bundle_check.txt` 에 남긴다. 하나라도 다르면 기동하지 않는다 (종료 5). 서버 컨테이너도 구역 파일을 같은 방식으로 확인한다.
  - 묶음이 봉인되지 않았거나 `valid=false` 면 드라이버가 거부한다.
- `--run-one <태그>`: `--order` 의 실험 하나, 갈래 0 을 한 번 돈다. 짝도 재시도도 없고, 태그 디렉토리가 이미 있으면 거부한다.
  - 시작 전 부하 조건은 P5 와 같다: load1 < 30, load5 < 50, load15 < 80 이 두 번 연속. 대기 상한은 `--load-wait-max-s`.
  - 갈래 중 부하 중단·`/clock` 정지·준비 시간 초과 규칙도 P5 그대로다.
- 이름·기록 옵션:
  - `--agent-id`: 컨테이너 이름 `amr-ai-<id>-gz-<태그>`
  - `--slot-id`: gz_slot.sh id
  - `--partition`: IGN_PARTITION
  - `--notebook`, `--stage`, `--run-id-prefix`, `--record-agent`: 실행 목록 위치·형식
  - `--seal`: 끝나면 갈래 디렉토리를 봉인한다
- 실험 (p5_plan.py, 기존 `ORDER` 에는 넣지 않았다):

| id | 내용 |
|---|---|
| `LO` | 중앙 모드. 1 차 5 작업을 t_base 에, 2 차 같은 5 쌍(`PLAN_PAIRS[0:5]`)을 t_base + 300 s 에 낸다. 지평 900 s 고정 (`end_when_done: false` — 2 차 뒤에도 대기 슬롯 체류를 같은 길이로 남긴다) |
| `LOd` | 같은 계획, 분산 모드 |
| `LOsmoke` | 1 차 5 작업, 지평 60 s. 기동 확인용 (셈 밖) |

## 5. 5 대 실행 한 번 — 조정자 명령

호스트에서 포그라운드로 실행한다. 이 브랜치가 체크아웃된 작업 트리의 드라이버를 쓴다. 드라이버가 스택 컨테이너를 `gz_slot.sh <slot-id>` 로 감싸 포그라운드에서 돌린다.

```bash
# 간단히 (아래 명령과 같다. 환경 변수 E3_WS · E3_RAW · E3_AGENT · E3_SLOT 로 바꿀 수 있다)
E3_AGENT=<자기 id> /home/CAPYI_AI/wt/layopt-gen/layopt/gen/run_e3.sh v10-v20-v30-v40-v50 LO
```

풀어 쓴 명령:

```bash
SPEC=v10-v20-v30-v40-v50                          # 돌릴 묶음 (valid.json valid=true 인 것)
TAG=$(date -u +%Y%m%dT%H%M%SZ)_${SPEC}_LO         # 새 run_id (= 원자료 디렉토리 이름)
WS=/home/CAPYI_AI/runs/layopt/tmp/gen/ws/96a8883  # 빌드한 고정 작업공간 (git archive 96a8883)
REPO=/home/CAPYI_AI/wt/layopt-gen                 # 실행 목록을 남길 작업 트리 (자기 것으로)
python3 $REPO/tests/integration/fleet_p5/p5_driver.py \
    --raw /home/CAPYI_AI/runs/layopt/e3 --ws-new $WS --ws-old $WS --repo $REPO \
    --bundle /home/CAPYI_AI/runs/layopt/bundles/$SPEC \
    --order LO --run-one $TAG --domain 59 --t-start 60 \
    --agent-id layopt-e3 --slot-id layopt-e3 --partition amr_ai_layopt_e3 \
    --notebook docs/research/layout-opt/notebook --stage E3 --run-id-prefix '' \
    --record-agent <자기 id> --seal
```

- 원자료는 `/home/CAPYI_AI/runs/layopt/e3/$TAG/` 에 모인다.
  - P5 와 같은 것: `gt.csv, est.csv, events.jsonl, metrics.json, summary.json, launch.log, …`
  - layopt 확장이 더하는 것: `bundle_check.txt, bundle_SHA256SUMS.txt`
  - 드라이버 일지는 `/home/CAPYI_AI/runs/layopt/e3/driver.log`, 시작 조건은 `e3/pairs/$TAG.start_condition` 이다.
- 반환 코드: 0 유효, 4 무효 (metrics.json `valid=false`), 1 STOP 파일, 2 인자·태그 오류.
- 실행 목록 등록: `--repo` 작업 트리의 `docs/research/layout-opt/notebook/runs/<record-agent>.jsonl` (run_id = 태그).
- 분산 모드는 `--order LOd` 다. 기동만 확인하려면 `--order LOsmoke` 를 쓴다.
- 두 실행을 동시에 돌리지 않는다. P5 기록에 동시 실행이 무너진 사례가 있다 (design §4). 두 번째 실행은 앞 실행이 끝난 뒤 새 태그로 돌린다.
- 이 고정 작업공간은 소스를 git archive 로 떼어 왔다. 브랜치 작업 트리를 고쳐도 실행에 쓰는 코드는 바뀌지 않는다.
  - 코드를 바꿔야 하면 새 커밋으로 작업공간을 다시 만든다:
    1. `git archive <커밋> | tar -x -C …/ws/<커밋>`
    2. `cpu_slot.sh` 안에서 `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic -Werror" --packages-up-to amr_bringup`
  - `ARCHIVE_COMMIT.txt` 가 실행 기록 `HEAD.txt` 로 들어간다.

## 6. 스모크 실행 (기준 배치 묶음) — 결과

`LOsmoke` 를 기준 묶음, 도메인 59, 파티션 `amr_ai_layopt_e3`, gz_slot `layopt-gen` 으로 돌렸다. 원자료는 `runs/layopt/tmp/gen/e3_smoke/<태그>/` 이고, 실행 목록 run_id 는 `LOGEN_<태그>` 다.

| 시도 | 태그 | 결과 | 벽시계 | 부하 (시작 → 중간 최대 → 끝, 1/5/15 분) |
|---|---|---|---|---|
| 1 | 20261009T175916Z_…_LOsmoke | **무효 CLOCK_STALL** (sim 15.4 s, 준비 전) | 332 s (+ 시작 대기 60 s) | 27.1/32.0/28.5 → 1 분 95 → 42.4/60.6/45.6 |
| 2 | 20261009T180713Z_…_LOsmoke | **무효 CLOCK_STALL** (sim 2.1 s, 준비 전) | 356 s (+ 시작 대기 330 s) | 26.7/44.1/44.0 → 1 분 101 → 42.1/52.8/50.3 |
| 3 | 20261009T182454Z_…_LOsmoke | (엄격한 시작 조건 load < 20/25/35 로 대기 중) | | |

[측정] 두 번 모두 확인된 것과 확인 안 된 것을 나눈다.

- 묶음 겹침은 맞았다. `bundle_check.txt` 의 sha256 이 5/5 일치했다. 확인한 경로:
  - install share 의 월드·behavior·fleet_spawn·구역 파일
  - traffic_manager 가 읽는 소스 트리 구역 파일 (서버 컨테이너도 따로 확인)
- 기동은 정상 순서로 진행됐다.
  - 런치 인자: `map_yaml:=/bundle/map.yaml robots_file:=/bundle/config/fleet_spawn.yaml`
  - 스폰 위치: 묶음 슬롯 (18…26, −16), 5 대 스폰 확인
  - 월드 재개, 스택 기동
- 끊긴 것: 기록기 그래프에서 `/clock` 발행자가 0 이 됐다 (`graph_at_end.json`, 노드 ~100 개만 보임). 그래서 기록기 180 s 감시가 rc 4 로 끝냈다.
- 시도 1 의 Gazebo 자체는 계속 돌았다. 같은 시각 `ign topic` 의 sim 은 55 s, RTF 0.31 이었다. 시도 2 는 2.1 s sim 에 벽시계 약 140 s 가 걸렸다.
- 두 시도 모두 스택 기동 중 부하 1 분 값이 95–101 이었다.
  - 다른 사용자의 openroad 가 ~4–11 코어를 썼다.
  - layopt-sim E1 (cpu_slot, `--cpus 12`), layopt-loc E0·E2 Gazebo 가 함께 돌았다.
  - P5 가 `/clock` 정지로 무효가 된 경우와 같은 형태다 (P5 원자료 10 건 중 3 건 CLOCK_STALL).
- 시도 1 에는 SHM `open_and_lock_file failed` 가 9 건 있었다. 유효했던 P5 실행 3 건에는 0 건이다. 시도 2 에는 1 건이었다.
- **원인은 가르지 못했다** (미측정). 과부하 기동 실패로 보지만, 도메인 59 와 저장소 기본 배치를 같은 조건에서 대조하지 않았다.
- 판단: E3 실행은 P5 처럼 저부하 시간대에 돌아야 한다. 방법은 둘이다.
  - `exclusive.sh`
  - 드라이버의 `--load-max/--load5-max/--load15-max` 를 낮춘다.

## 7. 540 묶음 생성 — 결과 [계산]

- 명령: `cpu_slot.sh layopt-gen layopt/gen/run_build.sh all --workers 6`. 생성기 83202d2, 2026-10-10 02:58:00–02:58:30.
  - 벽시계 30 s, 6 프로세스. 부하 시작 36.1/33.2/28.2, 끝 38.3/34.1/28.6.
  - 요약: `runs/layopt/tmp/gen/batches/20261009T175800Z_bundles/summary.json`.
- 결과: **유효 500 / 540**, 무효 40, 오류 0. 540 개 모두 봉인했다.
- 무효 이유:

| 종류 | 수 | 내용 |
|---|---|---|
| lane | 36 | n_center(v14) × beside_waiting(v31): 북쪽 벽 충전소 staging 3 개가 지게차 차선 상자에서 0.64 m (< 0.66) |
| resolve | 4 | se_current(v10) × line3(v21) × beside_waiting(v31) × structures(v51): 대기 슬롯 줄 양 끝에 구조물 S1 자리가 없음 (동쪽은 벽, 서쪽은 충전소) |

- 기준 묶음만 2b71c30 으로 먼저 만들었다 (§3). 83202d2 로 다시 만들어 보니 `spec.json` 밖의 파일이 모두 같았다.
- 수준별 범위 (유효 묶음):
  - 구역 수: uniform6 17, b_shift3 29, pitch_mix 17, end_moved 16. end_moved 는 C 열 서쪽 끝이 비어 x_bc 가 5 개다.
  - 포켓 수: b_shift3 11–13, 나머지 34–37.
  - queue 체류 지점: 38–46 개.
  - 슬롯·staging 의 지도 여유 최솟값: 0.97 m (stagger 뒤 열, 북쪽 벽).
  - 동적 경로 거리 최솟값: 1.50 m.

## 8. 알려진 한계와 정오표

- **queue 체류 지점 정오표 (2026-10-10 03:12, 47b04f1)**: 봉인 묶음(생성기 83202d2·2b71c30) 의 `layout.json` queue 점 일부는 구역에 들어갈 수 없는 쪽에 있다.
  - 모든 유효 묶음의 4 개: `queue:narrow_aisle:w`, `:e`, `queue:dock_2_approach:s`, `queue:dock_b_approach:s`.
  - b_shift3 의 12 개: `x_ab_{2,4,…,12}:n`, `x_bc_{2,4,…,12}:s`.
  - E0·E1 은 묶음별 목록에 있는 id 를 빼고 쓴다. 목록: `runs/layopt/tmp/gen/errata/20261009T181209Z_queue_blocked.json` (`{"drop_queue_ids": {spec_id: [id…]}}`, `layopt/gen/errata_queue.py`).
  - 생성기는 고쳤다 (`dwell.approach_blocked`). 묶음은 봉인 규칙 때문에 다시 만들지 않았다. 다시 만들지는 조정자가 정한다.

- **대기 패드 도색**: 바닥 도색(10 × 4 m, 시각체만)은 슬롯 배열과 무관하게 중심에 놓인다. line3 의 양 끝 슬롯(±6 m)은 도색 밖이다. LiDAR 와 지도에는 영향이 없고, 카메라 영상만 다르다.
- **b_shift3 의 통로**:
  - 통로 AB·BC 에 1 m 간격 교차로가 12 개씩 생겨 사슬로 묶인다. 통로를 따라가는 로봇은 지나는 구역을 한꺼번에 요청한다.
  - 통로 포켓은 대부분 구역 안이라 빠진다 (자동 포켓이 대신한다).
  - 교통 비용 차이의 일부는 이 구역 규칙에서 나온다. 해석할 때 함께 적는다.
- **b_shift3 의 경로**: 셔틀 차선이 y 6..11 로 짧아지고, 동측 보행자가 x 23.375 로 옮겨진다. V4 효과에 동적 장애물 차이가 섞인다 (levels.md §4).
- **지도 비교 범위**: SLAM 지도와의 비교는 기준 배치만 했다. 변형 배치에는 SLAM 지도가 없다 (모두 래스터).
