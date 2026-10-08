# F5 도크 배치 변경 — 운동학 모의로 결정점 미달 침범이 사라지는지

- 시각: 2026-10-08 10:21 KST · 에이전트: b4-wire · 단계: W
- 코드: `3f33338` (브랜치 `feature/b4-wire`)
- 관련 실행 id: W-f5-old, W-f5-new (runs/b4-wire.jsonl)

## 질문
도크 구역 높이를 staging ±1 m → ±0.75 m 로 줄여 요청 거리를 1.5 → 2.0 m (결정점 거리 1.85 m 이상)로 올린 배치 변경
(`3f33338`)이, P4 가 운동학 모의에서 본 "현행 hold 의미가 무장애 · 손실 0 에서도 도크 구역을 침범" (A1h 6/30, A5h 14/30 실행,
p4.md §4.5, 기제 = p0.md §5 결정점 미달)을 없애는가? 다른 구역 사건 · 처리량은 어떻게 되나?

## 가설과 예측
- H1 (주): 새 배치에서 hold 비교군 A1h · A5h 의 F0 · 손실 0 도크 구역 동시 점유 실행 = 0/30 (두 비교군 모두).
  P4 의 침범은 6/6 이 "거절 hold 를 받았지만 못 섰다" 였고 깊이 ≤ 0.18 m — 요청 거리 0.5 m 증가가 그 부족분(1.85 − 1.5 =
  0.35 m)을 넘는다. **기각**: 어느 한 비교군이라도 > 0 이면 H1 기각 → "미달 — 남은 위험" 으로 적고 사건을 추적한다.
- H2 (보조, 판정 없음): F6(피해 로봇이 도크 안에서 멈춤) 의 도크 사건도 줄어든다 (P4: 13/13 hold true — 같은 기제).
- H3 (보조, 판정 없음): 허가 목록 비교군 A1 · A5 의 도크 사건은 옛 · 새 모두 0 (P4 와 같음). 교차로 · 통로 사건 수는 변하지 않는다.
- 처리량(보조): 완료 작업 수 짝 차 — 예측 없음 (구역이 좁아져 대기 자리가 바뀐다). CI 가 0 을 포함하면 "가르지 못함".
- 표본 설계 확인: 0/30 의 95 % 상한 9.5 % — "침범률 < 9.5 %" 까지만 말할 수 있다 (P4 옛 값 20 % · 47 % 와는 가를 수 있다).

## 설정
- 시뮬레이터: amr_fleet.fleetsim (P4 운동학 플릿 모의, Gazebo 아님). 이미지 amr-ai/rl:0.3, PYTHONHASHSEED=0, cpu_slot.
- 옛 배치 = `fd17e49` git archive (P4 코드 + 옛 traffic_zones.yaml), 새 배치 = `3f33338` git archive. 두 묶음의 fleetsim
  코드 차이는 plan.py 의 분류 상수 · report.py 의 분류 함수뿐 (모의 동역학 · 정책 코드는 같다) → 차이는 배치(yaml)에서 온다.
- 셀: 비교군 A1h_nearest_hold · A5h_cbaa_hold · A1_nearest_central · A5_cbaa_central, 고장 F0 · F6, 손실 0, 시드 1–30,
  rate 1/45 (P4 와 같은 명령: `python3 -m amr_fleet.fleetsim.sweep --workers 6 --arms … --losses 0 --faults F0,F6
  --seeds 1-30 --rate 0.022222222222222223`). 비교군 · 고장 · 시드마다 n = 30, 옛 · 새 각 240 실행.
- 분석: `docs/research/fleet-distributed/tools/f5_dock_check.py` (이 항목과 같은 커밋 — 결과 전에 고정). 도크 사건 =
  cooccupancy_zones 의 dock_* 키 합. 옛 배치 재실행 digest 를 P4-hold-01 (`/home/CAPYI_AI/runs/b24/b4-p4/P4-hold-01/out/runs.jsonl`)
  과 대조한다 (재현 확인 — 같지 않으면 그 사실을 적고 옛 재실행 값을 기준으로 쓴다).
- 원자료: `/home/CAPYI_AI/runs/b24/b4-wire/W-f5/{old,new}/`.

## 결과
(표: 시드별 값, 실패 시드 수. [측정]/[계산]/미측정 표기. 원자료 경로)

## 해석
(결과가 말하는 것과 말하지 않는 것. 공유 부하 하 예비값 여부)

## 다음
(이 결과로 바뀌는 결정, 남은 질문)
