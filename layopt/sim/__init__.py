"""
E1 — ROS 없는 빠른 함대 시뮬레이터 (layopt 배치 최적화 연구, design.md §4 · §5 W5).

배치 묶음(bundle, contract.md §1–2)을 받아 5 대 함대를 3600 s 교대로 돌리고 교통 지표와 체류(dwell) 기록을
contract.md §4 형식으로 낸다. 설계·근사·보정 기록은 docs/research/layout-opt/sim.md.

- layout   : 묶음(layout.json · map · traffic_zones · world.sdf) 또는 지금 저장소 파일 → Layout
- navmap   : 점유 지도 → Nav2 식 팽창 코스트맵 · 팀 A*(amr_core_py) 경로 · 여유 거리 지도
- actors   : 작업자 6 · 지게차 · 셔틀 궤적 (world.sdf 에서 읽음), 시드별 위상
- traffic  : 실제 TrafficManager(traffic_manager.py) 연결 — 관측 만들기 · 명령(hold · 포켓 · keepout) 적용
- robot    : 로봇 1 대 (운동학, 작업 단계, 도킹 시간, 배터리, 충전, 물리 막힘)
- fleet    : 중앙 할당 (fleet_manager 의미: 0.5 s 라운드, nearest, 스케줄러 순서)
- workload : 포아송 작업 열 · P5 작업 · 시작 잔량 (공통 난수)
- dwell    : 체류 지점별 k_hist_s
- loc      : 측위 사건 훅 (기본 0)
- sim      : 실행 1 건 → contract §4 행 + 부가 진단
- sweep    : 묶음 × 시드 × 부하 스윕 (JSONL, 이어 하기)
- calib    : 실험 0 오프라인 보정 (P5 Gazebo 원자료 ↔ E1)
"""

from __future__ import annotations

import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
_PATHS = [REPO / 'src' / 'amr_fleet', REPO / 'src' / 'amr_behavior',
          REPO / 'layopt' / 'sim' / 'native' / 'build']


def ensure_paths() -> None:
    """저장소의 순수 코어(amr_fleet · amr_behavior)와 amr_core_py 빌드를 import 경로에 둔다."""
    for p in _PATHS:
        s = str(p)
        if p.is_dir() and s not in sys.path:
            sys.path.insert(0, s)


ensure_paths()
