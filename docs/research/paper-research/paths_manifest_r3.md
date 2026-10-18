# 3 라운드 held-out D 경로 sha256 (prereg_r3.md §4)

만든 때 2026-10-19 KST. **3 라운드 튜닝·held-out D 실행 전에 커밋한다.**
- 위치: `/home/CAPYI_AI/paper_research/runs/r3/paths_d` (컨테이너 `/runs/r3/paths_d`).
- 만든 명령: `python3 docs/research/paper-research/scripts/orchestrate_r3.py paths --commit 9e75b56 --dirty 0 --ws /home/CAPYI_AI/wt/dwa-abl`
  - 생성기는 2 라운드와 같은 make_paths_r2.py --narrow-random (seed 20261021, 매끈 32 · 통로 8 · A* 20) 이다.
  - 그 뒤 plan_astar_x.py (배포 planner_server, 이 집합의 지도) → --astar-scenes → scenes_obstacles.py --n 20.
- A* 쌍 24 개 중 앞 20 개가 모두 계획됐다. 매끈 경로는 32 번 뽑아 32 개다.
- 튜닝은 2 라운드 튜닝-20 (`runs/r2/paths_tune`, `paths_manifest_r2.md`) 을 다시 쓴다.

| 통로 | 폭 [m] | 길이 [m] | 진입 | 편향 [m] |
| --- | --- | --- | --- | --- |
| N1 | 0.60 | 4 | straight | +0.0181 |
| N2 | 0.70 | 6 | s_bend | -0.0254 |
| N3 | 0.60 | 6 | turn_right | +0.0150 |
| N4 | 0.60 | 6 | s_bend | -0.0046 |
| N5 | 0.60 | 4 | s_bend | -0.0083 |
| N6 | 0.70 | 6 | turn_right | +0.0148 |
| N7 | 0.60 | 4 | straight | +0.0182 |
| N8 | 0.70 | 6 | turn_right | +0.0287 |

A* 길이 [m]: A01 10.0, A02 42.8, A03 13.4, A04 29.0, A05 42.7, A06 25.7, A07 55.4, A08 23.9, A09 50.7, A10 17.4, A11 32.0, A12 39.5, A13 12.8, A14 49.3, A15 26.0, A16 47.6, A17 36.2, A18 20.8, A19 15.2, A20 33.7.

O 장면 (모두 첫 뽑기 통과):

| 장면 | 기준 경로 | x [m] | y [m] | r [m] |
| --- | --- | --- | --- | --- |
| O01 | S01_pwc | 72.83 | 3.01 | 0.254 |
| O02 | S02_pwl | 68.81 | 3.70 | 0.258 |
| O03 | S03_pwc | 77.15 | -4.01 | 0.386 |
| O04 | S04_pwl | 70.20 | 5.60 | 0.292 |
| O05 | S05_pwc | 73.66 | 1.91 | 0.363 |
| O06 | S06_pwl | 61.51 | -3.31 | 0.381 |
| O07 | S07_pwc | 70.19 | 4.11 | 0.306 |
| O08 | S08_pwl | 77.42 | -2.07 | 0.335 |
| O09 | S09_pwc | 62.77 | -0.42 | 0.279 |
| O10 | S10_pwl | 78.22 | 0.29 | 0.251 |
| O11 | S11_pwc | 74.34 | 2.02 | 0.379 |
| O12 | S12_pwl | 65.72 | -2.43 | 0.328 |
| O13 | S13_pwc | 66.96 | 0.90 | 0.286 |
| O14 | S14_pwl | 57.05 | 1.03 | 0.271 |
| O15 | S15_pwc | 71.46 | 6.59 | 0.267 |
| O16 | S16_pwl | 78.30 | -5.24 | 0.304 |
| O17 | S17_pwc | 70.54 | -2.66 | 0.265 |
| O18 | S18_pwl | 63.44 | -7.48 | 0.375 |
| O19 | S19_pwc | 70.72 | -2.91 | 0.283 |
| O20 | S20_pwl | 59.52 | -2.02 | 0.328 |

| 파일 | sha256 |
| --- | --- |
| astar_pairs.json | 0042f1c32e7addd4df8e3aab2fcd2c99c191e9915eece031e4bcb7c6cb333777 |
| composite.pgm | fed61e8ccff61eed4d42bbe8f632130cda214a2740a09baacb23ddc8f7aad68a |
| composite.yaml | c65d6116eb05c9152c4bb053584f4d59508f76e0e39317254ff76751ed6c1716 |
| heldout_astar.json | 870b6587bb3d9ef6f4e907c06013ddea3bb8bcdfdba7b8ffb9461700c7d9f9ec |
| heldout_narrow.json | 319879d154da7baa13224eb1acc2efc1f9beb7292af788c4bd4050bcf8408fa4 |
| heldout_smooth.json | c9288e29a66f66980add99ca0116ddcbd86679413ee2fc9ae31336a413123acc |
| manifest.json | 8d92eb20e6432ee9465be38800613d888cc78c284c2d592ec09da579a493866f |
| probe.json | fdbe05c6d7f764e2b7975a41c2ad259f1abdd5dac23fea6b98669aaa2c6f86de |
| scenes_heldout_astar.txt | c5c0930bda0931214fc721d41376f9212c65c4ab42c4e87d6f6c6e30a63e11aa |
| scenes_heldout_narrow.txt | 85c62aaddbe3e46f98dcef6a54824a255e07fa16e3fc20fbb5d04d27ed69348c |
| scenes_heldout_smooth.txt | aa4ae29dc434252a019071d5051f9b04baa3b8f6f80da0a58addd190187943a8 |
| scenes_obstacles.json | b8d7c1d4b32121b8ba8408d2959633fea00a397829c3e82f40993893be7da57f |
| scenes_probe.txt | 0467c23282286138fa6425b4b6d8e0ee05f537a83cb223e3f39616c50f805fe8 |
| scenes_tuning.txt | 44ba5da40e471486b12ce3495e348912d33cf803791e87c16167d1371086c5fc |
| tuning.json | 1ffdf0e581f4c05b607fdd1eaf4255b47062e7eb5eed8b409d7ed7b4a6e3e179 |

## C3 부분집합 (prereg_r3 §4)

매끈 S01–S06 · 통로 N1·N4·N6 · 길이 20 m 미만 A* 앞 셋 A01·A03·A10.
