#!/usr/bin/env python3
"""
lidar3d_probe.py 결과(probe.json) 여러 개 → 부하 측정 Markdown 표.

    lidar3d_load_table.py logs/s1_load_L0_a logs/s1_load_L1_a ...

열: 조건, RTF(창 전체·10 s 창 최소/중앙/최대), Gazebo 서버 CPU(코어)와 sim 1 s 당 CPU 초, 브리지·전처리 CPU,
3D 점군·2D 스캔·IMU 주기(시뮬레이션 시각 기준 hz, 간격 최대), GPU(공유 장치 전체), loadavg(중앙/최소/최대),
시작·끝 시점에 떠 있던 다른 Gazebo 컨테이너. 공유 서버 측정이므로 표 머리에 "공유 부하 하 예비값" 을 적는다.
"""

import json
from pathlib import Path
import sys


def fmt(v, nd=3):
    return '—' if v is None else f'{v:.{nd}f}'


def row(run: Path) -> str:
    d = json.loads((run / 'probe.json').read_text(encoding='utf-8'))
    pc = d.get('proc_cores', {})
    gz = pc.get('ign gazebo', {}).get('cores')
    rtf = d.get('rtf')
    w = d.get('rtf_10s', {})
    topics = d.get('topics', {})
    l3 = [v for t, v in topics.items() if t.endswith('lidar3d/points')]
    l3_hz = (f"{min(v['hz_sim'] for v in l3):.2f}–{max(v['hz_sim'] for v in l3):.2f} "
             f"({len(l3)}대)") if l3 and all(v['hz_sim'] for v in l3) else '—'
    l3_max = fmt(max(v['dt_sim_max'] for v in l3), 3) if l3 and all(v['dt_sim_max'] for v in l3) \
        else '—'
    scan = next((v for t, v in topics.items() if t.endswith('/scan')), {})
    imu = next((v for t, v in topics.items() if t.endswith('imu/data_raw')), {})
    g, la = d.get('gpu', {}), d.get('loadavg', {})
    others = []
    for name in ('containers_start.txt', 'containers_end.txt'):     # 시작·끝 시점의 다른 Gazebo
        cfile = run / name
        if cfile.is_file():
            others += [c for c in cfile.read_text().split()
                       if '-gz' in c and 's1-sensor' not in c and c not in others]
    cells = [
        d.get('label', run.name), run.name, fmt(rtf),
        f"{fmt(w.get('min'))}/{fmt(w.get('median'))}/{fmt(w.get('max'))}",
        fmt(gz, 2), fmt(gz / rtf if gz and rtf else None, 2),
        fmt(pc.get('parameter_bridge', {}).get('cores'), 2),
        fmt(pc.get('cloud_preprocessor_node', {}).get('cores'), 3),
        l3_hz, l3_max, fmt(scan.get('hz_sim'), 2), fmt(imu.get('hz_sim'), 1),
        f"{g.get('util_median', '—')}/{g.get('util_max', '—')} %, "
        f"{g.get('mem_median_mib', '—')} MiB",
        f"{la.get('median', '—')} ({la.get('min', '—')}–{la.get('max', '—')})",
        ', '.join(others) or '없음',
    ]
    return '| ' + ' | '.join(str(c) for c in cells) + ' |'


def main(argv) -> int:
    print('공유 부하 하 예비값 (32 코어 공유 서버, 다른 에이전트 작업과 동시). RTF 는 /clock 시뮬레이션 시각 / 벽시계.')
    print()
    print('| 조건 | 태그 | RTF | RTF 10 s 창 최소/중앙/최대 | gz 서버 코어 | gz CPU s / sim s | '
          '브리지 코어 | 전처리 코어 | 3D 점군 hz(sim) | 3D 간격 최대 s | 2D scan hz | IMU hz(amr_01) | '
          'GPU 사용률 중앙/최대·메모리(장치 전체) | loadavg 중앙 (최소–최대) | 동시 Gazebo |')
    print('|' + '---|' * 15)
    for a in argv:
        print(row(Path(a)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
