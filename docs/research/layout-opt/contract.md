# layopt 작업 간 계약 — 배치 묶음(bundle)과 E0/E1 출력 형식

여러 작업(생성기, E0, E1, E2/E3 러너)이 같은 형식을 쓰도록 여기서 고정한다.
형식을 바꿔야 하면 이 문서를 먼저 고치고 커밋한 뒤 일지에 남긴다.

## 1. 배치 묶음 `/home/CAPYI_AI/runs/layopt/bundles/<spec_id>/`

| 파일 | 내용 |
|---|---|
| `spec.json` | `{spec_id, levels: {V1..V5}, generator_commit, created}` |
| `layout.json` | 아래 §2의 해석된 기하 |
| `world.sdf` | Gazebo 월드 (생성기 출력) |
| `map.pgm`, `map.yaml` | 점유 지도. 0.05 m, 스캔 높이 0.20 m에서 SDF를 잘라 래스터화. 기준 배치도 같은 방식으로 만든다 |
| `config/behavior.yaml` | 도크·충전기(스테이징, 마커), `waiting_pose`, `charger_dock_ids`, `landmark_ids`/`landmark_poses` 반영본 |
| `config/fleet_spawn.yaml` | 로봇 5대 대기 슬롯 |
| `config/traffic_zones.yaml` | 구역(교차로, 협폭 통로, 도크 접근)과 대피 포켓. 모든 배치에 같은 알고리즘을 적용한다 |
| `valid.json` | `{valid: bool, reasons: [..]}` (도달성, 겹침, 동적 장애물 경로, 슬롯 여유) |
| `SHA256SUMS`, `SEALED` | 봉인 |

spec_id 형식: `v1<i>-v2<j>-v3<k>-v4<l>-v5<m>`(각 수준 번호, 0 = 현행). 기준 배치는 `v10-v20-v30-v40-v50`이다.

## 2. `layout.json`

```json
{
  "spec_id": "v10-v20-v30-v40-v50",
  "bounds": [-30.0, 30.0, -20.0, 20.0],
  "racks":      [{"id": "A1", "x": -18.0, "y": 9.0, "yaw": 0.0, "l": 2.0, "d": 1.0}],
  "pillars":    [{"id": "P1", "x": -10.0, "y": 12.0, "half": 0.25}],
  "structures": [{"id": "S1", "kind": "pallet_stack", "x": 0.0, "y": 0.0, "yaw": 0.0, "l": 1.2, "d": 1.0}],
  "docks":      [{"id": "dock_1", "kind": "inbound", "pad": [-27.5, 17.0], "staging": [x, y, yaw]}],
  "chargers":   [{"id": "c1", "pad": [x, y], "staging": [x, y, yaw]}],
  "waiting_slots": [{"robot": "amr_01", "x": 18.0, "y": -16.0, "yaw": 1.5708}],
  "narrow_aisle": {"cx": 0.0, "cy": -10.0, "clear": 0.6},
  "markers":    [{"id": 0, "x": -29.99, "y": 17.0, "yaw": 0.0, "kind": "dock"}],
  "dwell_points": [
    {"id": "wait:amr_01", "kind": "waiting", "x": 18.0, "y": -16.0, "yaw": 1.5708,
     "neighbors": [[20.0, -16.0, 1.5708], [22.0, -16.0, 1.5708]]}
  ]
}
```

- `dwell_points`는 로봇이 오래 서는 자세다.
  - `waiting`: 대기 슬롯. 이웃은 다른 슬롯이다.
  - `dock`, `charger`: 스테이징 자세. 이웃은 그 앞 대기열 자세다.
  - `queue`: 교차로·도크 접근 구역 앞에서 게이트를 기다리는 자세.
- `neighbors`는 동시에 서 있을 수 있는 다른 로봇의 자세이며, 가까운 순서로 적는다. E0은 앞에서 k개를 쓴다.

## 3. E0 출력 `/home/CAPYI_AI/runs/layopt/e0/<run_id>/<spec_id>.json`

```json
{"spec_id": "...", "map_sha256": "...", "points": [
  {"id": "wait:amr_01", "kind": "waiting", "k": 0, "margin": 0.031, "inlier": 0.99,
   "second": [x, y, yaw], "second_dist": 48.2}
], "route_samples": {"n": 0, "margin_q05": 0.0, "margin_q50": 0.0}}
```

- `k` ∈ {0, 2, 4}: 이웃 로봇 수.
- `inlier`: 이웃을 넣은 가상 스캔의 scan-map inlier 비율(운용 판정 0.50과 같은 정의).
- `margin`: 운용 `global_seed`와 같은 정의.

## 4. E1 출력 `/home/CAPYI_AI/runs/layopt/e1/<run_id>/results.jsonl` (한 줄 = 배치 × 시드 × 부하)

```json
{"spec_id": "...", "seed": 0, "load": 1.0, "horizon_s": 3600,
 "completed": 0, "throughput_per_h": 0.0, "task_p50_s": 0.0, "task_p95_s": 0.0,
 "deadlocks": 0, "unresolved": 0, "gate_wait_s": 0.0, "distance_m": 0.0,
 "dwell": [{"id": "wait:amr_01", "k_hist_s": {"0": 0.0, "1": 0.0, "2": 0.0}}],
 "loc_events": {"lost": 0, "misconverge": 0, "recovery_s": 0.0}}
```

- `dwell[].k_hist_s`: 그 체류 지점에 서 있던 시간을, 동시에 근처(이웃 목록 자세)에 있던 로봇 수별로 나눈 값.

## 5. 도메인·파티션·컨테이너 이름

| 용도 | ROS_DOMAIN_ID | IGN_PARTITION | 컨테이너 이름 |
|---|---|---|---|
| E2 단일 로봇 측위 시행 | 57 | `amr_ai_layopt_e2` | `amr-ai-layopt-e2-*` |
| E3 5대 운용 | 59 | `amr_ai_layopt_e3` | `amr-ai-layopt-e3-*` |

모든 Gazebo 실행은 `/home/CAPYI_AI/env/gz_slot.sh layopt-<작업> …`로 포그라운드에서 돈다.
CPU를 많이 쓰는 스윕(E0/E1 전수 평가)은 `/home/CAPYI_AI/env/cpu_slot.sh layopt-<작업> …`로 돈다.
