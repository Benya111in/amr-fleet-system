"""
보너스 1 평가 도구 — DWA(A) 대 학습 정책(B·C) 정량 비교 (rl/eval/PROTOCOL.md).

순수 Python(numpy·scipy) 이다. rclpy·torch 에 의존하지 않으므로 어느 컨테이너에서나 pytest 로 돈다.
rliable 은 선택 의존성이다 (없으면 같은 정의의 numpy 구현으로 대체 — aggregate.py).

모듈:
  binomial        단일 비율 구간 (Clopper–Pearson·Wilson·Jeffreys·rule of three)·무사고 실증 표본수
  noninferiority  두 비율 비열등성 (Farrington–Manning·Newcombe·Tango)·우월성 (Fisher·mid-p·Boschloo)
  sequential      0 충돌 순차 감시 (Ville 경계 SPRT, success-run e-process)
  aggregate       IQM·층화 부트스트랩 CI·개선 확률 (rliable 정의)
  paired          McNemar mid-p·Wilcoxon 부호순위·Hodges–Lehmann
  multiplicity    Holm·고정 순서 검정
  srcc            Spearman 순위 상관·부트스트랩·Bonett–Wright 구간
  cvar            최악 α% 레벨 CVaR (plug-in·cross-fit)·Rockafellar–Uryasev
  planning        표본수·시드 수·Gazebo 시간 계획
  schema          결과 CSV 계약 (r1·r2·r3 가 쓰는 열)
  report          결과 CSV → 사전 등록 판정표
"""

__version__ = '0.1.0'
