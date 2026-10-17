# held-out B · O 장면 경로 집합 (sha256)

만든 때 2026-10-17 22:27 KST. **held-out B 실행 전에 커밋한다** (prereg §3.2, §7-2).
위치 `/home/CAPYI_AI/paper_research/runs/paths/` (컨테이너 `/runs/paths`). 만든 명령:

```bash
python3 docs/research/paper-research/scripts/orchestrate_x.py paths --commit 15a1749 --dirty 0 \
    --ws /home/CAPYI_AI/wt/dwa-abl
# = make_paths_x.py --out /runs/paths --seed 20261018
#   → plan_astar_x.py (배포 planner_server AStar, 배포 nav2_params.yaml, 도메인 70)
#   → make_paths_x.py --astar-scenes → scenes_obstacles.py --seed 20261018
```

- 생성기 확인: 사본 `make_paths_x.py` 를 원본 seed 20261014 로 돌리면 held-out A 의 11 개 파일 sha256 이 원본
  `runs/dwa-abl/paths/manifest.json` 과 모두 같고, 사본 `plan_astar_x.py` 의 A* 20 경로 점열도 원본과 같다
  (원본과 다른 것은 meta 의 planning_time_s 뿐 — 계획 시간 측정값이라 실행마다 다르다).
  확인 산출물 `/home/CAPYI_AI/paper_research/tmp/verify_A/`.
- A* 계획에 쓴 빌드: `paper_research/ws` 에는 아직 빌드가 없어 원 worktree `/home/CAPYI_AI/wt/dwa-abl` 의 빌드를
  읽기 전용으로 붙였다. 그 빌드의 소스는 커밋 6e12aea 이고 `git diff 6e12aea 474264e -- src` 가 비어 있어
  이 worktree 의 `src/amr_navigation` 과 같다.
- A* 쌍 24 개 중 앞 20 개가 모두 계획됐다 (실패·5 m 미만 0). 매끈 경로는 34 번 뽑아 32 개.

## 주의: held-out A 와 겹치는 파일

생성기는 seed 만 바꿨고 **좁은 통로 8 개는 무작위 요소가 없어 held-out A 와 비트 단위로 같다**
(`heldout_narrow.json` sha256 e4bb33fb… 동일). 튜닝 집합·합성 지도·탐침도 같다 (의도대로).
즉 held-out B 60 경로 중 새 경로는 매끈 32 + A* 20 = 52 개이고, 좁은 통로 8 개는 DWA-R 설계 때 본
held-out A 결과에 들어 있던 경로다. 결과에서 이 점을 밝히고 통로를 뺀 민감도도 낸다.

## 파일

| 파일 | held-out A 와 | sha256 |
| --- | --- | --- |
| astar_pairs.json | 새로 만듦 | 1a358b122531df31dd7672b1be6196c4c788d1fd0c593fa1b4cce3d18fcadd10 |
| composite.pgm | 같음 | 4b5c315ba88f80f1471919d909fa7f525375da0ae8e6a25673d497ce52a3c4a8 |
| composite.yaml | 같음 | c65d6116eb05c9152c4bb053584f4d59508f76e0e39317254ff76751ed6c1716 |
| heldout_astar.json | 새로 만듦 | e1c160fe7c35998862cf339ccf5bc150da662a0fa5364d970a2d07c9c0ef20e2 |
| heldout_narrow.json | 같음 | e4bb33fb98513a99e08b974606f8809b5c6b21da5b9f042b164812f781fed6e5 |
| heldout_smooth.json | 새로 만듦 | 444ee3d604741736285aa374d4c578aeca75bef3948dacfbc762e49786ecc2d8 |
| manifest.json | 생성 기록 | fa9497eb23225fde0770d95f6ccd8b2e83da23e0e68cc39148563254160b9b6c |
| probe.json | 같음 | fdbe05c6d7f764e2b7975a41c2ad259f1abdd5dac23fea6b98669aaa2c6f86de |
| scenes_heldout_astar.txt | 새로 만듦 | 65dc4b72dab59ef5dc0f300fd8c37cad8d467d5d1449806a97d5189da039b2ce |
| scenes_heldout_narrow.txt | 같음 | 3fb95ffb6e7e5087914131d392d65d83c0593d772c0fcbcde3d7d0cc9023515d |
| scenes_heldout_smooth.txt | 새로 만듦 | b84786fe2890d0872e97ce9fd8a858bef722ff83962f82d8f86ac4cc39e13dd2 |
| scenes_obstacles.json | 새로 만듦 | bcdc3ab1099bf2e076d6cdc210d854db89877a74a6cc9141984e04942a82d9dd |
| scenes_probe.txt | 같음 | 0467c23282286138fa6425b4b6d8e0ee05f537a83cb223e3f39616c50f805fe8 |
| scenes_tuning.txt | 같음 | 44ba5da40e471486b12ce3495e348912d33cf803791e87c16167d1371086c5fc |
| tuning.json | 같음 | 1ffdf0e581f4c05b607fdd1eaf4255b47062e7eb5eed8b409d7ed7b4a6e3e179 |

## C3 잡음 부분집합 (prereg §5, 12 경로)

매끈 앞 6: S01_pwc–S06_pwl · 좁은 통로 N1·N4·N6 (원본 규칙, `dwa-ablation/scripts/orchestrate.py` NOISE_IDS) ·
길이 20 m 미만 A* 앞 3: A03 (18.4 m) · A05 (14.4 m) · A08 (17.3 m).

prereg 문장은 "통로 앞 3" 이지만 같은 문장이 "기존 규칙을 따라" 라고 하고, 기존 규칙은 N1·N4·N6 이었다
(0.60 직선 · 0.70 직선 · 90° 회전 진입). 결과를 보기 전(이 커밋)에 기존 규칙 쪽으로 정했고 results.md
「사전 등록과 다르게 한 것」에 적는다.

## O 장면 (prereg §3.2, `scenes_obstacles.json`)

held-out B 매끈 앞 20 경로, 원판 하나씩 (kinematic_sim `moving_obstacles` 에 속도 0 으로). 20 장면 모두 첫 뽑기에서
통과 여유 조건(원판 좌·우 어느 한쪽에 0.8 m 빈 띠)을 만족했다.

| 장면 | 기준 경로 | 원판 x, y [m] | 반지름 [m] | 호길이 비율 | 법선 오프셋 [m] | 경로 길이 [m] | 다시 뽑음 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| O01 | S01_pwc | 81.055, -1.681 | 0.308 | 0.650 | -0.093 | 22.00 | 0 |
| O02 | S02_pwl | 69.470, 5.195 | 0.379 | 0.594 | +0.054 | 8.85 | 0 |
| O03 | S03_pwc | 64.663, 8.846 | 0.253 | 0.567 | -0.100 | 19.85 | 0 |
| O04 | S04_pwl | 74.076, 5.458 | 0.380 | 0.688 | +0.045 | 15.75 | 0 |
| O05 | S05_pwc | 74.553, 2.026 | 0.287 | 0.362 | -0.076 | 19.85 | 0 |
| O06 | S06_pwl | 72.049, -4.778 | 0.364 | 0.612 | -0.065 | 10.60 | 0 |
| O07 | S07_pwc | 73.603, 4.660 | 0.373 | 0.311 | -0.073 | 19.95 | 0 |
| O08 | S08_pwl | 71.595, -2.598 | 0.268 | 0.328 | -0.071 | 9.30 | 0 |
| O09 | S09_pwc | 77.481, 3.318 | 0.377 | 0.464 | -0.003 | 18.65 | 0 |
| O10 | S10_pwl | 69.040, -11.851 | 0.287 | 0.636 | -0.096 | 20.95 | 0 |
| O11 | S11_pwc | 68.265, -7.198 | 0.258 | 0.583 | -0.002 | 14.05 | 0 |
| O12 | S12_pwl | 69.765, -4.707 | 0.342 | 0.520 | +0.031 | 10.00 | 0 |
| O13 | S13_pwc | 72.665, -3.483 | 0.380 | 0.542 | +0.002 | 8.10 | 0 |
| O14 | S14_pwl | 76.030, 7.246 | 0.266 | 0.605 | -0.088 | 16.95 | 0 |
| O15 | S15_pwc | 62.464, -1.426 | 0.303 | 0.668 | +0.028 | 11.70 | 0 |
| O16 | S16_pwl | 68.795, 4.048 | 0.300 | 0.318 | +0.041 | 14.85 | 0 |
| O17 | S17_pwc | 65.381, -8.953 | 0.376 | 0.597 | +0.002 | 20.75 | 0 |
| O18 | S18_pwl | 66.528, 7.290 | 0.320 | 0.616 | +0.098 | 17.35 | 0 |
| O19 | S19_pwc | 75.083, 2.329 | 0.378 | 0.525 | +0.008 | 11.90 | 0 |
| O20 | S20_pwl | 57.694, 2.794 | 0.259 | 0.620 | +0.012 | 21.25 | 0 |
