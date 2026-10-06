"""
재생 한 건의 산출물 → 처리 스캔 누락 표 한 줄과 대칭 비교표 manifest 항목 (rclpy 비의존).

replay_row.sh 가 남기는 파일만 읽는다: probe.json (입력 수·궤적), log_scan.json (로그 패턴·처리 줄 수),
match_*.json (입력 ↔ 처리 출력 스탬프 대응, LIO-SAM 끝 2 장 판정 포함), env.txt (런치 종료·loadavg),
db_info.txt (RTAB-Map DB 의 최적화 그래프 자세 수).

처리 수 근거 (rtabmap.md §4.1 과 같은 규칙):
- RTAB-Map: 1 차 = icp_odometry 로그 "Odom: ratio=" 줄 수 (그중 ratio=0 은 정합 실패 null odom 으로 따로 센다),
  2 차 = C++ 기록기의 icp_odom 스탬프 대응 (match). 누락 = 입력 − 1 차 처리 줄 수.
- LIO-SAM: imageProjection 출력(deskew/cloud_deskewed) 스탬프 대응. 끝 2 장(구조적 캐시)은 판정 분모에서 빼고 따로.
- slam_toolbox: 처리 수 근거 없음 → None (미측정).
"""

from __future__ import annotations

import json
import os
import re
from typing import Dict, Optional

from amr_slam3d import rows as rows_mod

MATCH_FILE = {'rtabmap': 'match_amr_01_icp_odom.json',
              'liosam': 'match_lio_sam_deskew_cloud_deskewed.json'}


def family(row: str) -> str:
    """행 이름 → 계열 (rtabmap | liosam | slam_toolbox)."""
    for f in ('rtabmap', 'liosam', 'slam_toolbox'):
        if row.startswith(f):
            return f
    raise ValueError(f'모르는 계열: {row}')


def _json(path: str) -> Optional[dict]:
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def parse_env(text: str) -> Dict[str, object]:
    """env.txt → 런치 종료 방식, 재생 시작·끝 loadavg 1 분 값."""
    out: Dict[str, object] = {'launch_exit': None, 'load_play_start': None, 'load_play_end': None}
    m = re.search(r'^launch_exit: (.*)$', text, re.M)
    if m:
        out['launch_exit'] = m.group(1).strip()
    for key, tag in (('load_play_start', 'play_start_wall'), ('load_play_end', 'play_end_wall')):
        m = re.search(rf'^{tag}: \S+ loadavg: ([0-9.]+)', text, re.M)
        if m:
            out[key] = float(m.group(1))
    return out


def optimized_poses(db_info: str) -> Optional[int]:
    """rtabmap-info 출력의 "Optimized graph: N poses" (없으면 None)."""
    m = re.search(r'Optimized graph:\s+(\d+) poses', db_info)
    return int(m.group(1)) if m else None


def summarize(out_dir: str, row: str) -> Dict[str, object]:
    """재생 출력 디렉터리 하나 → 누락 표 한 줄 (값이 없으면 None — 미측정)."""
    fam = family(row)
    r = rows_mod.row(row)
    probe = _json(os.path.join(out_dir, 'probe.json')) or {}
    scan = _json(os.path.join(out_dir, 'log_scan.json')) or {}
    s: Dict[str, object] = {'row': row, 'family': fam, 'dir': out_dir,
                            'input': (probe.get('input') or {}).get('count'),
                            'traj_n': (probe.get('traj_online') or {}).get('n'),
                            'tf_valid': (probe.get('tf') or {}).get('valid'),
                            'fatal': sum((scan.get('fatal') or {}).values()) if scan else None,
                            'notes': scan.get('note'), 'common': scan.get('common'),
                            'processed': None, 'missing': None, 'null_odom': None,
                            'tail_exempt': r.tail_exempt, 'tail_missing': None,
                            'match_missing': None, 'missing_stamps': None,
                            'imu_wait_drops': None, 'db_optimized_poses': None}
    env_path = os.path.join(out_dir, 'env.txt')
    if os.path.isfile(env_path):
        with open(env_path, encoding='utf-8') as f:
            s.update(parse_env(f.read()))
    m = _json(os.path.join(out_dir, MATCH_FILE[fam])) if fam in MATCH_FILE else None
    if fam == 'rtabmap':
        lines = scan.get('processed_log_lines')
        notes = scan.get('note') or {}
        s['null_odom'] = notes.get('Odom: ratio=0.000000')
        s['imu_wait_drops'] = notes.get("didn't receive IMU newer")
        if lines is not None and s['input'] is not None:
            s['processed'] = int(lines)
            s['missing'] = int(s['input']) - int(lines)
        if m:
            s['match_missing'] = m.get('judged_missing', m.get('missing'))
            s['missing_stamps'] = m.get('judged_missing_stamps', m.get('first_missing'))
        db = os.path.join(out_dir, 'db_info.txt')
        if os.path.isfile(db):
            with open(db, encoding='utf-8', errors='replace') as f:
                s['db_optimized_poses'] = optimized_poses(f.read())
    elif fam == 'liosam':
        notes = scan.get('note') or {}
        s['imu_wait_drops'] = notes.get('Waiting for IMU data')
        if m:
            s['processed'] = int(m['matched'])
            s['missing'] = int(m.get('judged_missing', m['missing']))
            s['tail_missing'] = int(m.get('tail_missing', 0))
            s['match_missing'] = s['missing']
            s['missing_stamps'] = m.get('judged_missing_stamps', m.get('first_missing'))
    return s


def manifest_entry(out_dir: str, row: str, replay: int) -> Dict[str, object]:
    """대칭 비교표(amr_evaluation.slam3d.symmetric) manifest 의 실행 항목."""
    r = rows_mod.row(row)
    e: Dict[str, object] = {'row': row, 'replay': int(replay),
                            'tum': os.path.join(out_dir, 'traj_online.tum'),
                            'dim': '3d' if r.input_kind == 'cloud' else '2d', 'oracle': r.oracle}
    ply = os.path.join(out_dir, 'cloud_world.ply')
    if os.path.isfile(ply):
        e['ply'] = ply
    return e
