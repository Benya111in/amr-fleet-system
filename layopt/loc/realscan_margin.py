"""
탐색 분석 (사전 등록 외): E2 실스캔으로 구한 여백 대 E0 가상 스캔 여백.

E2 러너는 시행마다 자연 단계 시작과 LOST 시각의 scan_filtered 를 남긴다(scans.json). 같은 운용 코드
(global_seed.search · alias_margin, 같은 지도)를 실스캔에 돌려 참 자세의 여백을 구하고, 시행 목록의 E0 값과 나란히 둔다.
E0 의 가상 스캔(지도 레이캐스트 + 상자 이웃)이 실제 LiDAR 를 얼마나 닮았는지 보는 진단이다.

    python3 -m layopt.loc.realscan_margin --run-dir <e2 run> --out <csv>
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys

from layopt.loc import mapio, raycast

sys.path.insert(0, str(mapio.REPO / 'src' / 'amr_localization'))
from amr_localization import global_seed  # noqa: E402
from amr_localization.scan_map_match import match_ratio  # noqa: E402


def scan_margin(field, scan, pose):
    """실스캔 하나 → (여백, 참 자세 순위, 인라이어)."""
    ranges = [math.inf if r is None else float(r) for r in scan['ranges']]
    amin, inc, rmax = scan['angle_min'], scan['angle_increment'], scan['range_max']
    hyps = global_seed.search(field, field.free, ranges, amin, inc, rmax, raycast.LIDAR)
    beams = global_seed.base_beams(ranges, amin, inc, rmax, 180, raycast.LIDAR)
    alts = global_seed.alternative_poses(hyps, pose, (0.0, 0.0, 0.0))
    margin = global_seed.alias_margin(field, beams, pose, alts, 0.2)
    rank = next((i + 1 for i, h in enumerate(hyps) if global_seed.pose_close(
        (h.x, h.y, h.yaw), pose, 1.0, math.radians(20.0))), None)
    inl, _ = match_ratio(field, pose, raycast.LIDAR, ranges, amin, inc, rmax, 0.2, 180)
    return margin, rank, inl


def main(argv=None) -> int:
    """명령줄."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    run = Path(a.run_dir)
    spec = json.loads((run / 'trials.json').read_text(encoding='utf-8'))
    m = mapio.load_map(spec['map'])
    field = mapio.distance_field(m)
    by_id = {t['trial_id']: t for t in spec['trials']}
    rows = []
    for line in (run / 'trials.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        t = by_id[r['trial_id']]
        d = run / 'trials' / f'{r["trial_id"]}_a{r["attempt"]}'
        scans = json.loads((d / 'scans.json').read_text(encoding='utf-8'))
        pose = (t['x'], t['y'], t['yaw'])
        row = {'trial_id': r['trial_id'], 'pose_id': t['pose_id'], 'kind': t['kind'],
               'k': t['k'], 'e0_margin': t.get('e0', {}).get('margin'),
               'outcome': r.get('outcome')}
        for key in ('natural_start', 'lost'):
            sc = scans.get(key)
            if sc:
                mg, rk, inl = scan_margin(field, sc, pose)
                row[f'real_margin_{key}'] = None if mg is None else round(mg, 4)
                row[f'real_rank_{key}'] = rk
                row[f'real_inlier_{key}'] = round(inl, 4)
        rows.append(row)
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in (
        'trial_id', 'pose_id', 'kind', 'k', 'e0_margin', 'outcome'), k))
    out = Path(a.out)
    if out.exists():
        print(f'이미 있다: {out}', file=sys.stderr)
        return 3
    with out.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f'{out}: {len(rows)} rows')
    return 0


if __name__ == '__main__':
    sys.exit(main())
