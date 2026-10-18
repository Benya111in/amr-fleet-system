# 2 라운드 경로 집합 sha256 (prereg_r2.md §1)

만든 때 2026-10-18 KST. **튜닝·held-out C 실행 전에 커밋한다.**
- 위치: `/home/CAPYI_AI/paper_research/runs/r2/paths_tune`·`paths_c` (컨테이너 `/runs/r2/…`).
- 만든 명령: `python3 docs/research/paper-research/scripts/orchestrate_r2.py paths --commit ce69b8f --dirty 0 --ws /home/CAPYI_AI/wt/dwa-abl`
  - = make_paths_r2.py --narrow-random → plan_astar_x.py (배포 planner_server, 그 집합의 지도) → make_paths_r2.py --astar-scenes → scenes_obstacles.py --n 6|20.

- 생성기 확인: make_paths_r2.py 를 시드 의존 통로 옵션 없이 1 라운드 seed 20261018 로 돌렸다.
  - held-out B 11 개 파일 sha256 이 모두 같다.
  - scenes_obstacles.py (기본 20) 출력도 같다 (`tmp/r2check`).
- 통로는 default_rng([seed, 1]) 로 뽑았다.
  - 폭 {0.60, 0.70}, 길이 {4, 6} m, 진입 {직선, 90° 우·좌, S 자}, 편향 ±U(0, 0.04) m.
  - 자리(x 108, y −24 … 18)와 기하 식은 1 라운드와 같다.
  - 지도(composite)는 집합마다 그 집합의 랙만 담는다.
- A* 계획 빌드는 1 라운드와 같은 원 worktree 빌드(`/home/CAPYI_AI/wt/dwa-abl`, src 6e12aea = 474264e) 다.
  - 쌍 n+4 개 중 앞 n 개가 모두 계획됐다.

## 튜닝-20 (매끈 10 · 통로 4 · A* 6) + 튜닝 O 6 — `paths_tune`, seed 20261020

| 통로 | 폭 [m] | 길이 [m] | 진입 | 편향 [m] |
| --- | --- | --- | --- | --- |
| N1 | 0.70 | 4 | turn_left | -0.0013 |
| N2 | 0.60 | 4 | s_bend | -0.0233 |
| N3 | 0.60 | 6 | turn_right | -0.0329 |
| N4 | 0.60 | 6 | turn_right | +0.0078 |

A* 길이 [m]: A01 16.4, A02 26.4, A03 30.6, A04 30.1, A05 47.2, A06 32.4. 매끈 경로 10 번 뽑아 10 개.

O 장면 (모두 첫 뽑기 통과):

| 장면 | 기준 경로 | x [m] | y [m] | r [m] |
| --- | --- | --- | --- | --- |
| O01 | S01_pwc | 78.28 | 8.73 | 0.270 |
| O02 | S02_pwl | 64.84 | 5.02 | 0.357 |
| O03 | S03_pwc | 73.76 | 5.09 | 0.387 |
| O04 | S04_pwl | 77.07 | -9.14 | 0.399 |
| O05 | S05_pwc | 66.25 | -3.61 | 0.327 |
| O06 | S06_pwl | 75.22 | 5.86 | 0.365 |

| 파일 | sha256 |
| --- | --- |
| astar_pairs.json | 1929d46071295b420defaba4b8c4dbbbee3b0404372527f9a54ee4eb0a275326 |
| composite.pgm | 7f68539ae6553f03239c0134effba31cc1d2c615d65dd908a6fca943a43102fd |
| composite.yaml | c65d6116eb05c9152c4bb053584f4d59508f76e0e39317254ff76751ed6c1716 |
| heldout_astar.json | fdde52c085373ae702de04d4cf2b0f898c0092da7eccc3e53766da0f4018c7dc |
| heldout_narrow.json | e9d0873cd3cc657d49d861919abde3d49f24474e31ef9b5fdebf8e846210c952 |
| heldout_smooth.json | 09ba78cb24b7d13a3d814b7962275236a7902e139bcb4699570677e038e7158b |
| manifest.json | bbc24a2ac89d5b1e63098396ec60a0849f0b20b780753944aed9d5dd4a3e8d4b |
| probe.json | fdbe05c6d7f764e2b7975a41c2ad259f1abdd5dac23fea6b98669aaa2c6f86de |
| scenes_heldout_astar.txt | 7a074061453c0026fc1305f31623785132022d23e89972fa767a6e3b79ed812a |
| scenes_heldout_narrow.txt | 76010fa794e1fdf1f63cf3af2446b7851c619983d67cb1494f33a83758ed6240 |
| scenes_heldout_smooth.txt | 0f23bd39a335a319712c85ca19558c559f2fb2d09b1e1fafbbaa402fae269da0 |
| scenes_obstacles.json | 70c362f6ccff1b41fda8d9f5966c6069f7182d8a4b19f93955422f6d7d7d64f9 |
| scenes_probe.txt | 0467c23282286138fa6425b4b6d8e0ee05f537a83cb223e3f39616c50f805fe8 |
| scenes_tuning.txt | 44ba5da40e471486b12ce3495e348912d33cf803791e87c16167d1371086c5fc |
| tuning.json | 1ffdf0e581f4c05b607fdd1eaf4255b47062e7eb5eed8b409d7ed7b4a6e3e179 |

## held-out C (매끈 32 · 통로 8 · A* 20) + O 20 — `paths_c`, seed 20261019

| 통로 | 폭 [m] | 길이 [m] | 진입 | 편향 [m] |
| --- | --- | --- | --- | --- |
| N1 | 0.70 | 6 | straight | -0.0217 |
| N2 | 0.60 | 4 | s_bend | +0.0165 |
| N3 | 0.70 | 6 | s_bend | -0.0379 |
| N4 | 0.60 | 6 | turn_left | -0.0341 |
| N5 | 0.60 | 4 | turn_right | +0.0200 |
| N6 | 0.70 | 4 | turn_right | -0.0049 |
| N7 | 0.70 | 4 | turn_left | +0.0263 |
| N8 | 0.70 | 6 | turn_left | +0.0361 |

A* 길이 [m]: A01 35.0, A02 56.2, A03 12.5, A04 29.4, A05 28.5, A06 34.0, A07 40.1, A08 34.1, A09 30.8, A10 20.3, A11 16.4, A12 35.9, A13 13.0, A14 38.8, A15 16.7, A16 23.6, A17 23.4, A18 10.6, A19 19.8, A20 5.3. 매끈 경로 32 번 뽑아 32 개.

O 장면 (모두 첫 뽑기 통과):

| 장면 | 기준 경로 | x [m] | y [m] | r [m] |
| --- | --- | --- | --- | --- |
| O01 | S01_pwc | 70.54 | -4.67 | 0.361 |
| O02 | S02_pwl | 65.86 | -6.48 | 0.311 |
| O03 | S03_pwc | 65.67 | 8.64 | 0.293 |
| O04 | S04_pwl | 66.10 | 3.59 | 0.324 |
| O05 | S05_pwc | 72.70 | 7.72 | 0.303 |
| O06 | S06_pwl | 72.82 | 7.01 | 0.389 |
| O07 | S07_pwc | 74.58 | 2.36 | 0.293 |
| O08 | S08_pwl | 61.69 | -0.56 | 0.334 |
| O09 | S09_pwc | 65.22 | 3.14 | 0.399 |
| O10 | S10_pwl | 76.84 | 5.62 | 0.307 |
| O11 | S11_pwc | 71.34 | 5.77 | 0.262 |
| O12 | S12_pwl | 72.65 | -4.47 | 0.259 |
| O13 | S13_pwc | 75.61 | -0.27 | 0.254 |
| O14 | S14_pwl | 78.48 | -3.18 | 0.294 |
| O15 | S15_pwc | 66.09 | -3.44 | 0.360 |
| O16 | S16_pwl | 62.30 | 0.77 | 0.285 |
| O17 | S17_pwc | 64.29 | 0.63 | 0.302 |
| O18 | S18_pwl | 70.35 | 7.33 | 0.399 |
| O19 | S19_pwc | 65.12 | 6.24 | 0.316 |
| O20 | S20_pwl | 74.13 | 0.31 | 0.354 |

| 파일 | sha256 |
| --- | --- |
| astar_pairs.json | 81920553f6640f3d535a2ede2c008de1f60d231919047c07b39daf13aae32783 |
| composite.pgm | 923ae3236f0c7d2ff32aaaa7e545f641eb7b56cd09d454426216d02addf56a97 |
| composite.yaml | c65d6116eb05c9152c4bb053584f4d59508f76e0e39317254ff76751ed6c1716 |
| heldout_astar.json | 1e9465b0f23c7d7bebdc953ad2e4ebe890259b6eb9ce06562d3a5a64ac477557 |
| heldout_narrow.json | d754ba8e376544fe486e389adf8af4abc58a8f96901e412de6f65d97e66c6c2f |
| heldout_smooth.json | cd484b92e3de9a81a7ce2e7e3e859b59c5f30b4df4fbda6d50378d1aa4c7b1d7 |
| manifest.json | d5f100df4bd4569ed471a463c08f603133299b5de8a978975a5f50532b14cf0d |
| probe.json | fdbe05c6d7f764e2b7975a41c2ad259f1abdd5dac23fea6b98669aaa2c6f86de |
| scenes_heldout_astar.txt | 5d678286a4f9a6b07250a6ed26debe7f02072228891ed6b2d7f9e69ac9e08264 |
| scenes_heldout_narrow.txt | c13654a052ed0b823bb1985c65e246e348b89ba97d74e13e8ebfac7f63774961 |
| scenes_heldout_smooth.txt | 2286ba8e2df1613419c79fdffcaaac98569f7e06ffed0178726cf8c42575c6fc |
| scenes_obstacles.json | e48e51e143e85164c9546c3ab6ad0aede8de5a07a6ed79f5ab51af862b3e8da1 |
| scenes_probe.txt | 0467c23282286138fa6425b4b6d8e0ee05f537a83cb223e3f39616c50f805fe8 |
| scenes_tuning.txt | 44ba5da40e471486b12ce3495e348912d33cf803791e87c16167d1371086c5fc |
| tuning.json | 1ffdf0e581f4c05b607fdd1eaf4255b47062e7eb5eed8b409d7ed7b4a6e3e179 |

## held-out C 의 C3 부분집합 (prereg_r2 §3)

매끈 S01–S06 · 통로 N1·N4·N6 · 길이 20 m 미만 A* 앞 셋 A03·A11·A13.
