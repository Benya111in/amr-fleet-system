# DWA 논문 보강 실험 2 라운드 결과 (held-out C)

측정 조건:
- 사전 등록: `prereg_r2.md` (커밋 317b55a).
- 코드: 코어 사본 `47feeaa` (pr-core, CORE R2 READY).
- 하네스: `6195663` (분석 규칙 고정, 결과 전) – ⟪끝 커밋⟫.
- 이미지: `amr-ai/rl:0.3` (ROS 2 Humble, Nav2 1.1.20).
- 시험대: ROS 운동학 시험대(실시간)와 chain_sim_x(결정적). 1 라운드와 같다.
- 경로 (`paths_manifest_r2.md`):
  - 튜닝-20: seed 20261020, 매끈 10 · 통로 4 · A* 6 + 튜닝 O 6.
  - held-out C: seed 20261019, 매끈 32 · 통로 8 · A* 20 + O 20.
  - 좁은 통로는 이번부터 시드 의존이라, held-out C 의 60 경로 모두 1 라운드 경로와 다르다.
- 원자료: `/home/CAPYI_AI/paper_research/runs/r2/<단계>/` (봉인).
- 표 원본: `analysis/tables_r2.md` (`scripts/analyze_r2.py tables`).
- 전수 분석: `analysis/r1r2_full_analysis.md` (`scripts/full_r1r2.py`).
- 구간: 모두 경로(또는 장면) 묶음 부트스트랩 95 % (B 10 000, seed 12345).

⟪본문⟫
